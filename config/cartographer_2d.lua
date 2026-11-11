-- 2D SLAM trial B: Google Cartographer, LaserScan + EKF odometry (/Car_Odom_Filtered).
-- Lidar-only matching slides along a long straight street (all scans look alike),
-- so the filtered odometry is used as the motion prior.
include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "base_link",
  published_frame = "odom",          -- cartographer publishes map -> odom
  odom_frame = "odom",
  provide_odom_frame = false,        -- Tesla_Node already provides odom -> base_link
  publish_frame_projected_to_2d = true,
  use_odometry = true,               -- topic "odom", remapped to /Car_Odom_Filtered
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 1,               -- subscribes to "scan" (remapped to /scan)
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_2d = true
MAP_BUILDER.num_background_threads = 4

TRAJECTORY_BUILDER_2D.use_imu_data = false
TRAJECTORY_BUILDER_2D.min_range = 1.0
TRAJECTORY_BUILDER_2D.max_range = 79.0
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 5.0
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = false  -- would jump along the corridor
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 200.   -- trust the odometry prior
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 400.
TRAJECTORY_BUILDER_2D.motion_filter.max_time_seconds = 0.5
TRAJECTORY_BUILDER_2D.motion_filter.max_distance_meters = 0.5
TRAJECTORY_BUILDER_2D.motion_filter.max_angle_radians = math.rad(1.0)
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 60
TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution = 0.1

POSE_GRAPH.optimize_every_n_nodes = 35
POSE_GRAPH.constraint_builder.min_score = 0.6
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.65

return options
