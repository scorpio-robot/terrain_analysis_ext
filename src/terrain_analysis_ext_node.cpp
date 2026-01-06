// Original code from jizhang-cmu/autonomy_stack_mecanum_wheel_platform.
// Modified by Lihan Chen on 2026/01/06
// Copyright 2025 Ji Zhang
//
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file or at
// https://developers.google.com/open-source/licenses/bsd

#include "terrain_analysis_ext/terrain_analysis_ext_node.hpp"

#include <algorithm>
#include <cmath>
#include <queue>

#include "pcl_conversions/pcl_conversions.h"
#include "tf2_eigen/tf2_eigen.hpp"

namespace terrain_analysis_ext
{

TerrainAnalysisExtNode::TerrainAnalysisExtNode(const rclcpp::NodeOptions & options)
: Node("terrain_analysis_ext", options),
  system_initialized_(false),
  system_init_count_(0),
  point_decay_count_(0)
{
  // Declare and get frame parameters
  this->declare_parameter("odom_frame", "odom");
  this->declare_parameter("lidar_frame", "mid360");

  this->get_parameter("odom_frame", odom_frame_);
  this->get_parameter("lidar_frame", lidar_frame_);

  // Declare and get voxel parameters
  this->declare_parameter("terrain_voxel_size", 2.0);
  this->declare_parameter("terrain_voxel_width", 41);
  this->declare_parameter("planar_voxel_size", 0.4);
  this->declare_parameter("planar_voxel_width", 101);

  this->get_parameter("terrain_voxel_size", terrain_voxel_size_);
  this->get_parameter("terrain_voxel_width", terrain_voxel_width_);
  this->get_parameter("planar_voxel_size", planar_voxel_size_);
  this->get_parameter("planar_voxel_width", planar_voxel_width_);

  // Initialize shifts
  terrain_voxel_shift_x_ = 0;
  terrain_voxel_shift_y_ = 0;
  planar_voxel_shift_x_ = 0;
  planar_voxel_shift_y_ = 0;

  // Declare and get processing parameters
  this->declare_parameter("voxel_point_count_thre", 100);
  this->declare_parameter("ground_elevation_thre", 0.1);
  this->declare_parameter("use_terrain_dyn_thre", 0.6);
  this->declare_parameter("check_terrain_conn", true);
  this->declare_parameter("terrain_under_vehicle", -0.75);
  this->declare_parameter("ceiling_filtering_thre", 2.0);
  this->declare_parameter("local_terrain_map_radius", 4.0);
  this->declare_parameter("downsize_filter_corner", 0.2);
  this->declare_parameter("downsize_filter_surface", 0.1);
  this->declare_parameter("point_decay_period", 10);
  this->declare_parameter("point_decay_ratio", 0.5);
  this->get_parameter("voxel_point_count_thre", voxel_point_count_thre_);
  this->get_parameter("ground_elevation_thre", ground_elevation_thre_);
  this->get_parameter("use_terrain_dyn_thre", use_terrain_dyn_thre_);
  this->get_parameter("check_terrain_conn", check_terrain_conn_);
  this->get_parameter("terrain_under_vehicle", terrain_under_vehicle_);
  this->get_parameter("ceiling_filtering_thre", ceiling_filtering_thre_);
  this->get_parameter("local_terrain_map_radius", local_terrain_map_radius_);
  this->get_parameter("downsize_filter_corner", downsize_filter_corner_);
  this->get_parameter("downsize_filter_surface", downsize_filter_surface_);
  this->get_parameter("point_decay_period", point_decay_period_);
  this->get_parameter("point_decay_ratio", point_decay_ratio_);

  // Declare and get geometric constraint parameters
  this->declare_parameter("vertical_angle_thre", 0.0);
  this->declare_parameter("max_terrain_angle", 30.0);
  this->declare_parameter("ground_z_thre", 0.1);
  this->get_parameter("vertical_angle_thre", vertical_angle_thre_);
  this->get_parameter("max_terrain_angle", max_terrain_angle_);
  this->get_parameter("ground_z_thre", ground_z_thre_);

  // Initialize TF2
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Initialize point clouds
  laser_cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  laser_cloud_stack_ = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  terrain_cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  terrain_cloud_elev_ = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  terrain_map_cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();

  kd_tree_ = std::make_shared<pcl::KdTreeFLANN<pcl::PointXYZI>>();
  downsize_filter_.setLeafSize(
    downsize_filter_surface_, downsize_filter_surface_, downsize_filter_surface_);

  // Initialize voxel arrays
  const int terrain_voxel_num = terrain_voxel_width_ * terrain_voxel_width_;
  const int planar_voxel_num = planar_voxel_width_ * planar_voxel_width_;

  point_count_array_.resize(terrain_voxel_num, 0);
  point_elevation_array_.resize(planar_voxel_num, 0.0f);
  point_planar_array_.resize(planar_voxel_num, 0);
  point_terrain_array_.resize(terrain_voxel_num, pcl::PointCloud<pcl::PointXYZI>());

  // Create subscribers and publisher
  laser_cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "registered_scan", 5,
    std::bind(&TerrainAnalysisExtNode::laserCloudHandler, this, std::placeholders::_1));

  terrain_map_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "terrain_map", 2,
    std::bind(&TerrainAnalysisExtNode::terrainMapHandler, this, std::placeholders::_1));

  terrain_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("terrain_map_ext", 2);

  RCLCPP_INFO(this->get_logger(), "Terrain Analysis Ext Node initialized");
}

std::optional<VehiclePose> TerrainAnalysisExtNode::getLidarPose(const rclcpp::Time & stamp)
{
  try {
    auto transform = tf_buffer_->lookupTransform(
      odom_frame_, lidar_frame_, stamp, rclcpp::Duration::from_seconds(0.05));
    Eigen::Affine3d eigen_transform = tf2::transformToEigen(transform.transform);
    return VehiclePose(eigen_transform);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(this->get_logger(), "TF lookup failed: %s", ex.what());
    return std::nullopt;
  }
}

void TerrainAnalysisExtNode::laserCloudHandler(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  auto pose_opt = getLidarPose(msg->header.stamp);
  if (!pose_opt) {
    return;
  }

  if (!system_initialized_) {
    system_initialized_ = true;
    RCLCPP_INFO(this->get_logger(), "System initialized");
  }

  laser_cloud_->clear();
  pcl::fromROSMsg(*msg, *laser_cloud_);

  processTerrainAnalysisExt(pose_opt.value());

  // Publish result
  sensor_msgs::msg::PointCloud2 output;
  pcl::toROSMsg(*terrain_cloud_elev_, output);
  output.header = msg->header;
  output.header.frame_id = odom_frame_;
  terrain_cloud_pub_->publish(output);
}

void TerrainAnalysisExtNode::terrainMapHandler(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  terrain_map_cloud_->clear();
  pcl::fromROSMsg(*msg, *terrain_map_cloud_);
}

void TerrainAnalysisExtNode::processTerrainAnalysisExt(const VehiclePose & pose)
{
  rollTerrainVoxels(pose);
  stackLaserScans(pose);
  downsampleAndDecayVoxels();
  estimateGroundElevation(pose);
  checkTerrainConnectivity(pose);
  buildExtendedTerrainMap(pose);
  mergeLocalTerrainMap(pose);
}

void TerrainAnalysisExtNode::rollTerrainVoxels(const VehiclePose & pose)
{
  // Critical: avoid const variables for center calculation (learned from
  // terrain_analysis bug)
  while (pose.x - terrain_voxel_size_ * terrain_voxel_shift_x_ < -terrain_voxel_size_) {
    for (int ind_y = 0; ind_y < terrain_voxel_width_; ++ind_y) {
      pcl::PointCloud<pcl::PointXYZI> temp_cloud =
        point_terrain_array_[terrain_voxel_width_ * (terrain_voxel_width_ - 1) + ind_y];

      for (int ind_x = terrain_voxel_width_ - 1; ind_x >= 1; --ind_x) {
        point_terrain_array_[terrain_voxel_width_ * ind_x + ind_y] =
          point_terrain_array_[terrain_voxel_width_ * (ind_x - 1) + ind_y];
      }

      point_terrain_array_[ind_y].clear();
    }
    terrain_voxel_shift_x_--;
  }

  while (pose.x - terrain_voxel_size_ * terrain_voxel_shift_x_ > terrain_voxel_size_) {
    for (int ind_y = 0; ind_y < terrain_voxel_width_; ++ind_y) {
      pcl::PointCloud<pcl::PointXYZI> temp_cloud = point_terrain_array_[ind_y];

      for (int ind_x = 0; ind_x < terrain_voxel_width_ - 1; ++ind_x) {
        point_terrain_array_[terrain_voxel_width_ * ind_x + ind_y] =
          point_terrain_array_[terrain_voxel_width_ * (ind_x + 1) + ind_y];
      }

      int dest_idx = terrain_voxel_width_ * (terrain_voxel_width_ - 1) + ind_y;
      point_terrain_array_[dest_idx].clear();
    }
    terrain_voxel_shift_x_++;
  }

  while (pose.y - terrain_voxel_size_ * terrain_voxel_shift_y_ < -terrain_voxel_size_) {
    for (int ind_x = 0; ind_x < terrain_voxel_width_; ++ind_x) {
      pcl::PointCloud<pcl::PointXYZI> temp_cloud =
        point_terrain_array_[terrain_voxel_width_ * ind_x + (terrain_voxel_width_ - 1)];

      for (int ind_y = terrain_voxel_width_ - 1; ind_y >= 1; --ind_y) {
        point_terrain_array_[terrain_voxel_width_ * ind_x + ind_y] =
          point_terrain_array_[terrain_voxel_width_ * ind_x + (ind_y - 1)];
      }

      point_terrain_array_[terrain_voxel_width_ * ind_x].clear();
    }
    terrain_voxel_shift_y_--;
  }

  while (pose.y - terrain_voxel_size_ * terrain_voxel_shift_y_ > terrain_voxel_size_) {
    for (int ind_x = 0; ind_x < terrain_voxel_width_; ++ind_x) {
      pcl::PointCloud<pcl::PointXYZI> temp_cloud =
        point_terrain_array_[terrain_voxel_width_ * ind_x];

      for (int ind_y = 0; ind_y < terrain_voxel_width_ - 1; ++ind_y) {
        point_terrain_array_[terrain_voxel_width_ * ind_x + ind_y] =
          point_terrain_array_[terrain_voxel_width_ * ind_x + (ind_y + 1)];
      }

      int dest_idx = terrain_voxel_width_ * ind_x + (terrain_voxel_width_ - 1);
      point_terrain_array_[dest_idx].clear();
    }
    terrain_voxel_shift_y_++;
  }
}

void TerrainAnalysisExtNode::stackLaserScans(const VehiclePose & pose)
{
  const int laser_cloud_size = laser_cloud_->points.size();
  const int terrain_voxel_half_width = (terrain_voxel_width_ - 1) / 2;

  for (int i = 0; i < laser_cloud_size; ++i) {
    const auto & point = laser_cloud_->points[i];

    int ind_x =
      static_cast<int>((point.x - pose.x + terrain_voxel_size_ / 2) / terrain_voxel_size_) +
      terrain_voxel_half_width;
    int ind_y =
      static_cast<int>((point.y - pose.y + terrain_voxel_size_ / 2) / terrain_voxel_size_) +
      terrain_voxel_half_width;

    if (point.x - pose.x + terrain_voxel_size_ / 2 < 0) ind_x--;
    if (point.y - pose.y + terrain_voxel_size_ / 2 < 0) ind_y--;

    if (ind_x >= 0 && ind_x < terrain_voxel_width_ && ind_y >= 0 && ind_y < terrain_voxel_width_) {
      int voxel_idx = terrain_voxel_width_ * ind_x + ind_y;
      point_terrain_array_[voxel_idx].push_back(point);
      point_count_array_[voxel_idx]++;
    }
  }
}

void TerrainAnalysisExtNode::downsampleAndDecayVoxels()
{
  const int terrain_voxel_num = terrain_voxel_width_ * terrain_voxel_width_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZI>());

  for (int ind = 0; ind < terrain_voxel_num; ++ind) {
    if (point_count_array_[ind] >= voxel_point_count_thre_) {
      temp_cloud->clear();
      downsize_filter_.setInputCloud(point_terrain_array_[ind].makeShared());
      downsize_filter_.filter(*temp_cloud);

      point_terrain_array_[ind].clear();
      point_terrain_array_[ind] = *temp_cloud;
      point_count_array_[ind] = 0;
    }
  }

  // Increment decay counter
  point_decay_count_++;
  if (point_decay_count_ >= point_decay_period_) {
    for (int ind = 0; ind < terrain_voxel_num; ++ind) {
      size_t new_size =
        static_cast<size_t>(point_terrain_array_[ind].points.size() * (1.0 - point_decay_ratio_));
      if (new_size < point_terrain_array_[ind].points.size()) {
        point_terrain_array_[ind].points.resize(new_size);
      }
    }
    point_decay_count_ = 0;
  }
}

void TerrainAnalysisExtNode::estimateGroundElevation(const VehiclePose & pose)
{
  const int planar_voxel_num = planar_voxel_width_ * planar_voxel_width_;
  const int terrain_voxel_half_width = (terrain_voxel_width_ - 1) / 2;
  const int planar_voxel_half_width = (planar_voxel_width_ - 1) / 2;

  // Clear planar elevation arrays
  std::fill(point_elevation_array_.begin(), point_elevation_array_.end(), 0.0f);
  std::fill(point_planar_array_.begin(), point_planar_array_.end(), 0);

  std::vector<std::vector<float>> planar_point_elev(planar_voxel_num);

  // Collect terrain cloud from nearby voxels
  terrain_cloud_->clear();
  for (int ind_x = terrain_voxel_half_width - 10; ind_x <= terrain_voxel_half_width + 10; ++ind_x) {
    for (int ind_y = terrain_voxel_half_width - 10; ind_y <= terrain_voxel_half_width + 10;
         ++ind_y) {
      if (
        ind_x >= 0 && ind_x < terrain_voxel_width_ && ind_y >= 0 && ind_y < terrain_voxel_width_) {
        *terrain_cloud_ += point_terrain_array_[terrain_voxel_width_ * ind_x + ind_y];
      }
    }
  }

  // Populate planar elevation arrays
  const int terrain_cloud_size = terrain_cloud_->points.size();
  for (int i = 0; i < terrain_cloud_size; ++i) {
    const auto & point = terrain_cloud_->points[i];

    int ind_x = static_cast<int>((point.x - pose.x + planar_voxel_size_ / 2) / planar_voxel_size_) +
                planar_voxel_half_width;
    int ind_y = static_cast<int>((point.y - pose.y + planar_voxel_size_ / 2) / planar_voxel_size_) +
                planar_voxel_half_width;

    if (point.x - pose.x + planar_voxel_size_ / 2 < 0) ind_x--;
    if (point.y - pose.y + planar_voxel_size_ / 2 < 0) ind_y--;

    // Add to neighboring planar voxels
    for (int d_x = -1; d_x <= 1; ++d_x) {
      for (int d_y = -1; d_y <= 1; ++d_y) {
        int idx = ind_x + d_x;
        int idy = ind_y + d_y;
        if (idx >= 0 && idx < planar_voxel_width_ && idy >= 0 && idy < planar_voxel_width_) {
          int planar_idx = planar_voxel_width_ * idx + idy;
          planar_point_elev[planar_idx].push_back(point.z);
        }
      }
    }
  }

  // Compute minimum elevation for each planar voxel
  for (int i = 0; i < planar_voxel_num; ++i) {
    if (!planar_point_elev[i].empty()) {
      point_elevation_array_[i] =
        *std::min_element(planar_point_elev[i].begin(), planar_point_elev[i].end());
    }
  }
}

void TerrainAnalysisExtNode::checkTerrainConnectivity(const VehiclePose & pose)
{
  if (!check_terrain_conn_) {
    return;
  }

  const int planar_voxel_half_width = (planar_voxel_width_ - 1) / 2;

  std::fill(point_planar_array_.begin(), point_planar_array_.end(), 0);

  // Start BFS from vehicle position
  int center_idx = planar_voxel_width_ * planar_voxel_half_width + planar_voxel_half_width;

  // If no elevation at center, use vehicle z + offset
  if (std::abs(point_elevation_array_[center_idx]) < 1e-6) {
    point_elevation_array_[center_idx] = pose.z + terrain_under_vehicle_;
  }

  std::queue<int> voxel_queue;
  voxel_queue.push(center_idx);
  point_planar_array_[center_idx] = 1;  // Mark as queued

  while (!voxel_queue.empty()) {
    int front = voxel_queue.front();
    voxel_queue.pop();
    point_planar_array_[front] = 2;  // Mark as connected

    int ind_x = front / planar_voxel_width_;
    int ind_y = front % planar_voxel_width_;

    // Check neighbors in a 21x21 window
    for (int d_x = -10; d_x <= 10; ++d_x) {
      for (int d_y = -10; d_y <= 10; ++d_y) {
        int idx = ind_x + d_x;
        int idy = ind_y + d_y;

        if (idx >= 0 && idx < planar_voxel_width_ && idy >= 0 && idy < planar_voxel_width_) {
          int neighbor_idx = planar_voxel_width_ * idx + idy;

          if (
            point_planar_array_[neighbor_idx] == 0 &&
            std::abs(point_elevation_array_[neighbor_idx]) > 1e-6) {
            float elev_diff =
              std::abs(point_elevation_array_[front] - point_elevation_array_[neighbor_idx]);

            if (elev_diff < ground_elevation_thre_) {
              voxel_queue.push(neighbor_idx);
              point_planar_array_[neighbor_idx] = 1;  // Mark as queued
            } else if (elev_diff > ceiling_filtering_thre_) {
              point_planar_array_[neighbor_idx] = -1;  // Mark as ceiling
            }
          }
        }
      }
    }
  }
}

void TerrainAnalysisExtNode::buildExtendedTerrainMap(const VehiclePose & pose)
{
  terrain_cloud_elev_->clear();

  const int planar_voxel_half_width = (planar_voxel_width_ - 1) / 2;
  const int terrain_cloud_size = terrain_cloud_->points.size();

  for (int i = 0; i < terrain_cloud_size; ++i) {
    const auto & point = terrain_cloud_->points[i];
    const float dis = std::hypot(point.x - pose.x, point.y - pose.y);

    // Only process points beyond local_terrain_map_radius_
    if (dis <= local_terrain_map_radius_) {
      continue;
    }

    int ind_x = static_cast<int>((point.x - pose.x + planar_voxel_size_ / 2) / planar_voxel_size_) +
                planar_voxel_half_width;
    int ind_y = static_cast<int>((point.y - pose.y + planar_voxel_size_ / 2) / planar_voxel_size_) +
                planar_voxel_half_width;

    if (point.x - pose.x + planar_voxel_size_ / 2 < 0) ind_x--;
    if (point.y - pose.y + planar_voxel_size_ / 2 < 0) ind_y--;

    if (ind_x >= 0 && ind_x < planar_voxel_width_ && ind_y >= 0 && ind_y < planar_voxel_width_) {
      int planar_idx = planar_voxel_width_ * ind_x + ind_y;
      float elev_diff = std::abs(point.z - point_elevation_array_[planar_idx]);

      // Check if point is near ground and connected (or connectivity check
      // disabled)
      if (
        elev_diff < use_terrain_dyn_thre_ &&
        (point_planar_array_[planar_idx] == 2 || !check_terrain_conn_)) {
        pcl::PointXYZI new_point;
        new_point.x = point.x;
        new_point.y = point.y;
        new_point.z = point.z;
        new_point.intensity = elev_diff;
        terrain_cloud_elev_->push_back(new_point);
      }
    }
  }
}

void TerrainAnalysisExtNode::mergeLocalTerrainMap(const VehiclePose & pose)
{
  // Merge terrain_map within local_terrain_map_radius_
  const int terrain_map_size = terrain_map_cloud_->points.size();
  for (int i = 0; i < terrain_map_size; ++i) {
    const auto & point = terrain_map_cloud_->points[i];
    const float dis = std::hypot(point.x - pose.x, point.y - pose.y);

    if (dis <= local_terrain_map_radius_) {
      terrain_cloud_elev_->push_back(point);
    }
  }
}

}  // namespace terrain_analysis_ext

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(terrain_analysis_ext::TerrainAnalysisExtNode)
