# Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause-Clear
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    """Launch file to start qti ARM node standalone."""
    config = os.path.join(
        get_package_share_directory('qrb_ros_manipulator'),
        'config',
        'manipulator_config.yaml'
    )
    manipulator_controller_node = Node(
        package='qrb_ros_manipulator',
        executable='qrb_ros_manipulator',
        output='screen',
        parameters=[config]
    )

    return LaunchDescription([manipulator_controller_node])
