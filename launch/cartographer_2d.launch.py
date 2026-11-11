"""2D SLAM trial B: /scan + EKF odometry (/Car_Odom_Filtered) -> Cartographer -> /map.

    ros2 launch csen934_project_pkg cartographer_2d.launch.py

If the occupancy-grid executable is not found, check its name with
`ros2 pkg executables cartographer_ros` and pass occupancy_exe:=<name>.
"""

import os
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import slam_common as sc  # noqa: E402


def generate_launch_description():
    sim = {'use_sim_time': LaunchConfiguration('use_sim_time')}
    return LaunchDescription(sc.common_args() + [
        DeclareLaunchArgument('occupancy_exe', default_value='cartographer_occupancy_grid_node'),
        sc.cloud_to_scan(),
        Node(package='cartographer_ros', executable='cartographer_node',
             name='cartographer_node', output='screen', parameters=[sim],
             arguments=['-configuration_directory', os.path.join(sc.share(), 'config'),
                        '-configuration_basename', 'cartographer_2d.lua'],
             remappings=[('scan', '/scan'), ('odom', '/Car_Odom_Filtered')]),
        Node(package='cartographer_ros', executable=LaunchConfiguration('occupancy_exe'),
             name='cartographer_occupancy_grid_node', output='screen', parameters=[sim],
             arguments=['-resolution', '0.1', '-publish_period_sec', '1.0']),
        sc.rviz('slam_2d.rviz'),
    ])
