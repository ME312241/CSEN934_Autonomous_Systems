"""Control + noise + EKF + visualization helper (Webots must already be running).

    ros2 launch csen934_project_pkg localization.launch.py [plot:=true] [rviz:=true]
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PKG = 'csen934_project_pkg'


def generate_launch_description():
    share = get_package_share_directory(PKG)
    params = os.path.join(share, 'config', 'localization.yaml')
    use_sim_time = LaunchConfiguration('use_sim_time')
    common = [params, {'use_sim_time': use_sim_time}]

    plot_topics = [
        '/Car_Pose2D_GT/x', '/Car_Pose2D_Noisy/x', '/Car_Pose2D_Filtered/x',
        '/Car_Pose2D_GT/y', '/Car_Pose2D_Noisy/y', '/Car_Pose2D_Filtered/y',
        '/Car_Pose2D_GT/theta', '/Car_Pose2D_Noisy/theta', '/Car_Pose2D_Filtered/theta',
    ]

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('plot', default_value='false',
                              description='open one rqt_plot window per quantity'),
        DeclareLaunchArgument('rviz', default_value='false'),

        Node(package=PKG, executable='control', output='screen', parameters=common),
        Node(package=PKG, executable='localization_add_noise', output='screen',
             parameters=common),
        Node(package=PKG, executable='localization_ekf_node', output='screen',
             parameters=common),
        Node(package=PKG, executable='visualization_helper', output='screen',
             parameters=common),

        # One window per quantity keeps the scales readable (x, y, yaw).
        ExecuteProcess(cmd=['ros2', 'run', 'rqt_plot', 'rqt_plot'] + plot_topics[0:3],
                       condition=IfCondition(LaunchConfiguration('plot'))),
        ExecuteProcess(cmd=['ros2', 'run', 'rqt_plot', 'rqt_plot'] + plot_topics[3:6],
                       condition=IfCondition(LaunchConfiguration('plot'))),
        ExecuteProcess(cmd=['ros2', 'run', 'rqt_plot', 'rqt_plot'] + plot_topics[6:9],
                       condition=IfCondition(LaunchConfiguration('plot'))),

        Node(package='rviz2', executable='rviz2', output='screen',
             arguments=['-d', os.path.join(share, 'rviz', 'localization.rviz')],
             parameters=[{'use_sim_time': use_sim_time}],
             condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
