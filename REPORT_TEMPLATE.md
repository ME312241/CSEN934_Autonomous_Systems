# CSEN934 Milestone 01 – Short Report

## 1. System overview
- rqt_graph screenshot (all nodes/topics).
- One paragraph: Webots `Tesla_Node` → `/Car_Odom_GT` → noise → EKF; control on `/Car_Ackrm`.

## 2. EKF performance and noise suppression
- Model: state [x, y, ψ, v, ω], bicycle model driven by `/Car_Ackrm`, full-state measurement, Q and R values used.
- Table from `results/metrics.txt`:

| Quantity | Noisy RMSE | Filtered RMSE | Reduction |
|---|---|---|---|
| x [m] | | | |
| y [m] | | | |
| yaw [deg] | | | |
| v [m/s] | | | |

- Figures: `xy_yaw_vs_time.png`, `error_vs_time.png`, rqt_plot screenshots.
- Discussion points: lag during acceleration (τ mismatch), effect of Q/R tuning, why the commanded input helps (prediction carries the motion, measurements only correct it), noisy std vs spec (validates noise node).

## 3. SLAM experiments
| Trial | Method | Input | Notes |
|---|---|---|---|
| 1 | slam_toolbox (2D) | /scan from pointcloud_to_laserscan | |
| 2 | Cartographer (2D) | /scan | |
| 3 | LIO-SAM (3D) | /Car_Lidar_Points + /Car_Imu | |

- Map screenshots / saved maps, trajectory overlay vs GT.
- Comparison: map consistency, drift vs GT, loop closure, CPU load, sensitivity to straight-line 10 m/s motion (few constraints along a corridor), 2D slice vs full 3D geometry.

## 4. Conclusions and issues encountered
