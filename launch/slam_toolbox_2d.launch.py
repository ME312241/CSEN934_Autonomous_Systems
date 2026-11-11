"""2D SLAM trial A: lidar cloud -> /scan -> slam_toolbox -> /map.

    ros2 launch csen934_project_pkg slam_toolbox_2d.launch.py
"""

import os
import sys

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import slam_common as sc  # noqa: E402


def generate_launch_description():
    return LaunchDescription(sc.common_args() + [
        sc.cloud_to_scan(),
        Node(package='slam_toolbox', executable='async_slam_toolbox_node',
             name='slam_toolbox', output='screen',
             parameters=[os.path.join(sc.share(), 'config', 'slam_toolbox_2d.yaml'),
                         {'use_sim_time': LaunchConfiguration('use_sim_time')}]),
        sc.rviz('slam_2d.rviz'),
    ])
