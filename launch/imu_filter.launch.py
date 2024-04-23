#!/usr/bin/env python3

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from math import pi

def generate_launch_description():
    ld = LaunchDescription()

    imu_filter_config = os.path.join(get_package_share_directory('navrover_localization'), 'config/imu_filter_config.yaml')
    
    imu_filter_madgwick = Node(
    	package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        parameters=[imu_filter_config],
        remappings=[('/imu/og_data_raw', '/imu')],
        output='screen'
        )
    ld.add_action(imu_filter_madgwick)

    return ld
