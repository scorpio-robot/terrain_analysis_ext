// Original code from jizhang-cmu/autonomy_stack_mecanum_wheel_platform.
// Modified by Lihan Chen on 2026/01/06
// Copyright 2025 Ji Zhang
//
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file or at
// https://developers.google.com/open-source/licenses/bsd

#ifndef TERRAIN_ANALYSIS_EXT__TERRAIN_ANALYSIS_EXT_NODE_HPP_
#define TERRAIN_ANALYSIS_EXT__TERRAIN_ANALYSIS_EXT_NODE_HPP_

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "pcl/filters/voxel_grid.h"
#include "pcl/kdtree/kdtree_flann.h"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"

namespace terrain_analysis_ext
{

struct VehiclePose
{
  Eigen::Affine3d transform;
  float x, y, z;
  float roll, pitch, yaw;
  float cos_roll, sin_roll;
  float cos_pitch, sin_pitch;

  explicit VehiclePose(const Eigen::Affine3d & t) : transform(t)
  {
    x = t.translation().x();
    y = t.translation().y();
    z = t.translation().z();

    auto rotation = t.rotation();
    roll = std::atan2(rotation(2, 1), rotation(2, 2));
    pitch = std::asin(-rotation(2, 0));
    yaw = std::atan2(rotation(1, 0), rotation(0, 0));

    cos_roll = std::cos(roll);
    sin_roll = std::sin(roll);
    cos_pitch = std::cos(pitch);
    sin_pitch = std::sin(pitch);
  }
};

class TerrainAnalysisExtNode : public rclcpp::Node
{
public:
  explicit TerrainAnalysisExtNode(const rclcpp::NodeOptions & options);

private:
  void laserCloudHandler(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void terrainMapHandler(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  std::optional<VehiclePose> getLidarPose(const rclcpp::Time & stamp);
  void processTerrainAnalysisExt(const VehiclePose & pose);

  void rollTerrainVoxels(const VehiclePose & pose);
  void stackLaserScans(const VehiclePose & pose);
  void downsampleAndDecayVoxels();
  void estimateGroundElevation(const VehiclePose & pose);
  void checkTerrainConnectivity(const VehiclePose & pose);
  void buildExtendedTerrainMap(const VehiclePose & pose);
  void mergeLocalTerrainMap(const VehiclePose & pose);

  // TF2
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Subscribers and Publisher
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr laser_cloud_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr terrain_map_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr terrain_cloud_pub_;

  // Point clouds
  pcl::PointCloud<pcl::PointXYZI>::Ptr laser_cloud_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr laser_cloud_stack_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr terrain_cloud_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr terrain_cloud_elev_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr terrain_map_cloud_;

  pcl::VoxelGrid<pcl::PointXYZI> downsize_filter_;
  pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kd_tree_;

  // Voxel arrays
  std::vector<int> point_count_array_;
  std::vector<float> point_elevation_array_;
  std::vector<int> point_planar_array_;
  std::vector<pcl::PointCloud<pcl::PointXYZI>> point_terrain_array_;

  // Parameters - Frame IDs
  std::string odom_frame_;
  std::string lidar_frame_;

  // Parameters - Terrain voxel
  float terrain_voxel_size_;
  int terrain_voxel_width_;
  int terrain_voxel_shift_x_;
  int terrain_voxel_shift_y_;

  // Parameters - Planar voxel
  float planar_voxel_size_;
  int planar_voxel_width_;
  int planar_voxel_shift_x_;
  int planar_voxel_shift_y_;

  // Parameters - Processing
  int voxel_point_count_thre_;
  float ground_elevation_thre_;
  float use_terrain_dyn_thre_;
  bool check_terrain_conn_;
  float terrain_under_vehicle_;
  float ceiling_filtering_thre_;
  float local_terrain_map_radius_;
  float downsize_filter_corner_;
  float downsize_filter_surface_;
  int point_decay_period_;
  float point_decay_ratio_;

  // Parameters - Geometric constraints
  float vertical_angle_thre_;
  float max_terrain_angle_;
  float ground_z_thre_;

  // State
  bool system_initialized_;
  int system_init_count_;
  int point_decay_count_;
};

}  // namespace terrain_analysis_ext

#endif  // TERRAIN_ANALYSIS_EXT__TERRAIN_ANALYSIS_EXT_NODE_HPP_
