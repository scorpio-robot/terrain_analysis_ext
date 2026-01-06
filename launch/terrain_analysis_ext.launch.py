# Copyright 2026 Lihan Chen
#
# Use of this source code is governed by a BSD-style
# license that can be found in the LICENSE file or at
# https://developers.google.com/open-source/licenses/bsd

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory("terrain_analysis_ext")

    # Create launch configuration variables
    params_file = LaunchConfiguration("params_file")

    stdout_linebuf_envvar = SetEnvironmentVariable(
        "RCUTILS_LOGGING_BUFFERED_STREAM", "1"
    )

    colorized_output_envvar = SetEnvironmentVariable("RCUTILS_COLORIZED_OUTPUT", "1")

    declare_params_file_cmd = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(
            pkg_dir, "params", "terrain_analysis_ext_param.yaml"
        ),
        description="Path to terrain analysis parameter file",
    )

    start_terrain_analysis_ext_node = Node(
        package="terrain_analysis_ext",
        executable="terrain_analysis_ext_node",
        name="terrain_analysis_ext",
        output="screen",
        parameters=[params_file],
    )

    # Create launch description
    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(stdout_linebuf_envvar)
    ld.add_action(colorized_output_envvar)

    # Declare launch options
    ld.add_action(declare_params_file_cmd)

    # Add nodes
    ld.add_action(start_terrain_analysis_ext_node)

    return ld
