"""3D SLAM trial: LIO-SAM (ros2 branch, built from source) on /Car_Lidar_Points + /Car_Imu.

    ros2 launch csen934_project_pkg lio_sam_3d.launch.py

TF layout: odom (GT world) -> liosam_map (static, = first GT position, published
by Visualization_Helper_Node) -> liosam_odom (static identity) -> liosam_base_link
(LIO-SAM). This keeps LIO-SAM's tree separate from the GT odom -> base_link
transform while still drawing both in the same RViz fixed frame.
Run localization.launch.py as well (it provides the GT path); this file adds a
second helper instance only for the alignment TF.
"""

import os
import sys

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import slam_common as sc  # noqa: E402


def generate_launch_description():
    sim = {'use_sim_time': LaunchConfiguration('use_sim_time')}
    params = os.path.join(sc.share(), 'config', 'lio_sam.yaml')
    lio = [('lio_sam_imuPreintegration', 'lio_sam_imuPreintegration'),
           ('lio_sam_imageProjection', 'lio_sam_imageProjection'),
           ('lio_sam_featureExtraction', 'lio_sam_featureExtraction'),
           ('lio_sam_mapOptimization', 'lio_sam_mapOptimization')]
    return LaunchDescription(sc.common_args() + [
        Node(package='tf2_ros', executable='static_transform_publisher',
             name='liosam_map_to_odom',
             arguments=['--frame-id', 'liosam_map', '--child-frame-id', 'liosam_odom'],
             parameters=[sim]),
        Node(package='csen934_project_pkg', executable='visualization_helper',
             name='LioSam_Alignment_Node', output='screen',
             parameters=[sim, {'align_frame': 'liosam_map', 'slam_frame': 'liosam_map',
                               'base_frame': 'liosam_base_link'}],
             remappings=[('/Slam_Path', '/LioSam_TF_Path'),
                         ('/Car_Pose2D_GT', '/_unused/p_gt'),
                         ('/Car_Pose2D_Noisy', '/_unused/p_noisy'),
                         ('/Car_Pose2D_Filtered', '/_unused/p_filt'),
                         ('/Car_Path_GT', '/_unused/path_gt'),
                         ('/Car_Path_Noisy', '/_unused/path_noisy'),
                         ('/Car_Path_Filtered', '/_unused/path_filt'),
                         ('/Car_Error_Noisy', '/_unused/e_noisy'),
                         ('/Car_Error_Filtered', '/_unused/e_filt')]),
    ] + [Node(package='lio_sam', executable=exe, name=name, output='screen',
              parameters=[params, sim]) for exe, name in lio] + [
        sc.rviz('slam_3d.rviz'),
    ])
