"""Shared pieces for the SLAM launch files (not a launch file itself)."""

import os

from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PKG = 'csen934_project_pkg'


def share():
    return get_package_share_directory(PKG)


def common_args():
    """base_link -> lidar_link is published by Tesla_Node (measured in Webots)."""
    return [
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
    ]


def cloud_to_scan():
    return Node(
        package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan', output='screen',
        remappings=[('cloud_in', '/Car_Lidar_Points'), ('scan', '/scan')],
        parameters=[os.path.join(share(), 'config', 'pointcloud_to_laserscan.yaml'),
                    {'use_sim_time': LaunchConfiguration('use_sim_time')}])


def rviz(config_name):
    from launch.conditions import IfCondition
    return Node(
        package='rviz2', executable='rviz2', output='screen',
        arguments=['-d', os.path.join(share(), 'rviz', config_name)],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        condition=IfCondition(LaunchConfiguration('rviz')))
