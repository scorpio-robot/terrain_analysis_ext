# Terrain Analysis Ext

[![License](https://img.shields.io/badge/License-BSD--3--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)

A ROS2 package for extended-scale terrain analysis, providing enhanced terrain mapping capabilities with larger voxel sizes and advanced connectivity analysis.

## Overview

The `terrain_analysis_ext` package extends the base terrain analysis functionality by operating at a larger scale with enhanced features for terrain connectivity analysis and extended mapping capabilities. It processes registered LiDAR point clouds to generate detailed terrain elevation maps suitable for autonomous navigation in complex environments.

## Features

- **Extended Scale Mapping**: Uses larger voxel sizes (2.0m terrain, 0.4m planar) for broader terrain coverage
- **Terrain Connectivity Analysis**: Optional BFS-based connectivity checking to identify traversable terrain regions
- **Ceiling Filtering**: Removes overhead obstacles and structures from terrain analysis
- **Point Decay Mechanism**: Automatically removes outdated point measurements over time
- **Local Terrain Merging**: Integrates local terrain maps within a configurable radius
- **TF2 Integration**: Uses modern ROS2 transform system for accurate pose estimation

## Inputs

### Topics Subscribed

- **`registered_scan`** (`sensor_msgs/PointCloud2`)
  - Registered LiDAR point cloud data
  - Expected frame: LiDAR sensor frame (automatically detected from message header)
  - Used for real-time terrain analysis and mapping

- **`terrain_map`** (`sensor_msgs/PointCloud2`)
  - Base terrain map from other terrain analysis nodes
  - Frame: `odom`
  - Integrated into extended terrain map within local radius

### TF Requirements

- **`odom` → `lidar_frame`**: Transform from odometry frame to LiDAR sensor frame
  - Required for accurate pose estimation during terrain analysis
  - `lidar_frame` defaults to `mid360` but can be configured

## Outputs

### Topics Published

- **`terrain_map_ext`** (`sensor_msgs/PointCloud2`)
  - Extended terrain elevation map
  - Frame: `odom`
  - Point intensity represents elevation difference from estimated ground plane
  - Includes both analyzed terrain points and merged local terrain data

## Parameters

### Frame Configuration

- **`odom_frame`** (string, default: "odom")
  - Name of the odometry coordinate frame

- **`lidar_frame`** (string, default: "mid360")
  - Name of the LiDAR sensor frame

### Terrain Voxel Parameters

- **`terrain_voxel_size`** (double, default: 2.0)
  - Size of terrain voxels in meters

- **`terrain_voxel_width`** (int, default: 41)
  - Width of terrain voxel grid (must be odd)

### Planar Voxel Parameters

- **`planar_voxel_size`** (double, default: 0.4)
  - Size of planar analysis voxels in meters

- **`planar_voxel_width`** (int, default: 101)
  - Width of planar voxel grid (must be odd)

### Processing Parameters

- **`voxel_point_count_thre`** (int, default: 100)
  - Minimum point count threshold for voxel processing

- **`ground_elevation_thre`** (double, default: 0.5)
  - Ground elevation estimation threshold

- **`use_terrain_dyn_thre`** (double, default: 1.5)
  - Terrain dynamics threshold

- **`check_terrain_conn`** (bool, default: true)
  - Enable/disable terrain connectivity analysis

- **`terrain_under_vehicle`** (double, default: -0.75)
  - Expected terrain height under vehicle (for connectivity analysis)

- **`ceiling_filtering_thre`** (double, default: 2.0)
  - Height threshold for ceiling filtering

- **`local_terrain_map_radius`** (double, default: 4.0)
  - Radius for local terrain map merging

### Filtering Parameters

- **`downsize_filter_corner`** (double, default: 0.2)
  - Corner point downsampling leaf size

- **`downsize_filter_surface`** (double, default: 0.1)
  - Surface point downsampling leaf size

### Decay Parameters

- **`point_decay_period`** (int, default: 10)
  - Number of processing cycles before point decay

- **`point_decay_ratio`** (double, default: 0.5)
  - Ratio of points to retain during decay

### Geometric Constraints

- **`vertical_angle_thre`** (double, default: 0.0)
  - Vertical angle threshold for point filtering

- **`max_terrain_angle`** (double, default: 30.0)
  - Maximum allowable terrain slope angle (degrees)

- **`ground_z_thre`** (double, default: 0.1)
  - Ground height threshold

## Usage

### Launch with Default Parameters

```bash
ros2 launch terrain_analysis_ext terrain_analysis_ext.launch.py
```

### Launch with Custom Parameters

```bash
ros2 launch terrain_analysis_ext terrain_analysis_ext.launch.py params_file:=path/to/custom_params.yaml
```

## Contributing

Please follow the existing code style and submit pull requests for any improvements.

## License

This project is licensed under the BSD-3-Clause License - see the [LICENSE](LICENSE) file for details.

## Acknowledgments

Based on original work from [jizhang-cmu/autonomy_stack_mecanum_wheel_platform](https://github.com/jizhang-cmu/autonomy_stack_mecanum_wheel_platform)
