# csen934_project_pkg — CSEN934 Milestone 01

Webots ground-truth odometry, Ackermann control, noisy odometry, EKF
localization and 2D/3D SLAM, all in one `ament_python` package.

## 1. What is where (spec → file)

| Milestone item | Implementation | Interface |
|---|---|---|
| Webots world | `Webots_Worlds_Controller/Worlds/city.wbt` | TeslaModel3 on a 1.4 km straight road lined with 87 buildings, poles, trees |
| Webots node `Tesla_Node` | `Webots_Worlds_Controller/Controllers/CSEN934_Test_OL_Sensors/CSEN934_Test_OL_Sensors.py` | pub `/Car_Odom_GT` (nav_msgs/Odometry), sub `/Car_Ackrm` (ackermann_msgs/AckermannDrive) |
| Sensor setup | `.../Define_Sensors.py` | finds + enables GPS, InertialUnit, Gyro, Accelerometer, Lidar; measures lidar mount |
| Helpers | `.../Helping_Functions.py` | Euler ↔ quaternion, rotations, sim-time stamps, lidar → PointCloud2 |
| Control node | `csen934_project_pkg/control.py` (`Control_Node`) | pub `/Car_Ackrm`, speed 10 m/s, steering 0 rad |
| Noise node | `csen934_project_pkg/localization_add_noise.py` (`Localization_Add_Noise_Node`) | sub `/Car_Odom_GT`, pub `/Car_Odom_Noisy`, σ = 0.3 m / 1° / 0.3 m/s / 1°/s, covariance filled |
| EKF node `Localization_EKF_Node` | `csen934_project_pkg/localization_ekf_node.py` (+ `ekf_core.py`) | sub `/Car_Odom_Noisy`, `/Car_Ackrm`, pub `/Car_Odom_Filtered` |
| rqt_plot of x, y, yaw | `visualization_helper.py` publishes `/Car_Pose2D_{GT,Noisy,Filtered}` (rqt_plot can't plot yaw from a quaternion) | `localization.launch.py plot:=true` |
| 2D SLAM ×2 | slam_toolbox, Cartographer (`launch/*_2d.launch.py`, `config/`) | `/Car_Lidar_Points` → pointcloud_to_laserscan → `/scan` → `/map` |
| 3D SLAM ×1 | LIO-SAM (`launch/lio_sam_3d.launch.py`, `config/lio_sam.yaml`) | `/Car_Lidar_Points` + `/Car_Imu` |
| RViz: map, trajectory, comparison | `rviz/slam_2d.rviz`, `rviz/slam_3d.rviz`, `rviz/localization.rviz` | `/Car_Path_*`, `/Slam_Path`, `/lio_sam/mapping/path` |
| Report numbers / figures | `analyze_bag.py` | RMSE + PNGs from a recorded bag |

Extra topics published by `Tesla_Node` (needed by EKF timing and SLAM, not
contradicting the spec): `/clock`, `/tf` (odom → base_link), `/tf_static`
(base_link → lidar_link, measured from the world through the supervisor API),
`/Car_Imu`, `/Car_Lidar_Points` (fields x y z intensity ring time, 10 Hz).

### Car and sensors in `city.wbt`
| Slot | Device (name) | Notes |
|---|---|---|
| sensorsSlotCenter (car origin = rear-axle centre) | GPS (`gps`) | defines `base_link` |
| sensorsSlotTop, +0.35 m | Lidar (`lidar`) | 16 layers × 1800, 360° × 30°, 2.5–100 m, 10 Hz, ≈1.9 m above ground |
| sensorsSlotTop, +0.35 m | InertialUnit, Gyro, Accelerometer | co-located with the lidar → LIO-SAM extrinsics = identity |

Car: Webots `TeslaModel3`, wheelbase 2.94 m, `supervisor TRUE`, starts at
(0, −1.75) in the right lane heading +x. `basicTimeStep` 10 ms → `/Car_Odom_GT` at 100 Hz.

### EKF model (for the report)
State `[x, y, ψ, v, ω]`, input `u = [v_cmd, δ]` from `/Car_Ackrm`.
Prediction: `x += v cosψ dt`, `y += v sinψ dt`, `ψ += ω dt`,
`v += (dt/τ)(v_cmd − v)`, `ω = v tanδ / L` (kinematic bicycle, L = 2.94 m).
Update: full-state measurement `[x, y, ψ, v, ω]` from `/Car_Odom_Noisy`,
`R` taken from the message covariance, Joseph-form covariance update,
yaw innovation wrapped to [−π, π). `dt` comes from header stamps (sim time).
Offline simulation (`test/test_ekf_core.py`): position RMSE 0.30 m → ~0.05 m,
yaw RMSE 0.99° → ~0.28°, speed RMSE 0.30 → ~0.16 m/s.

## 2. Dependencies (inside the ros2-humble container)

```bash
sudo apt update && sudo apt install -y \
  ros-humble-ackermann-msgs ros-humble-tf2-ros ros-humble-rviz2 \
  ros-humble-rqt ros-humble-rqt-plot ros-humble-rqt-graph \
  ros-humble-slam-toolbox ros-humble-cartographer-ros \
  ros-humble-pointcloud-to-laserscan \
  ros-humble-rosbag2 ros-humble-rosbag2-storage-default-plugins \
  python3-numpy python3-matplotlib python3-pytest python3-colcon-common-extensions
```

**Webots (R2023b or newer)** must run in the same container as ROS, because the
controller imports `rclpy`. Official apt repo:
```bash
sudo mkdir -p /etc/apt/keyrings && cd /etc/apt/keyrings
sudo wget -q https://cyberbotics.com/Cyberbotics.asc
echo "deb [arch=amd64 signed-by=/etc/apt/keyrings/Cyberbotics.asc] https://cyberbotics.com/debian binary-amd64/" \
  | sudo tee /etc/apt/sources.list.d/Cyberbotics.list
sudo apt update && sudo apt install -y webots
```
The container needs X11 (`-e DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix`, your
alias already runs `xhost +local:docker`) and ideally the GPU (`--device /dev/dri`).
If the existing container was created without these, they cannot be added
with `docker start`; recreate it. Software rendering fallback:
`LIBGL_ALWAYS_SOFTWARE=1 webots`. The first time `city.wbt` opens, Webots
downloads the PROTOs it references from GitHub, so the container needs internet
once (they are cached afterwards).

**LIO-SAM (3D SLAM, source build):**
```bash
sudo apt install -y software-properties-common
sudo add-apt-repository ppa:borglab/gtsam-release-4.1
sudo apt install -y libgtsam-dev libgtsam-unstable-dev \
  ros-humble-perception-pcl ros-humble-pcl-msgs ros-humble-vision-opencv ros-humble-xacro
cd ~/ros2_ws/src && git clone -b ros2 https://github.com/TixiaoShan/LIO-SAM.git
```

## 3. Setup

```bash
cd ~/ros2_ws/src            # your mounted workspace (e.g. ~/ros2_ws_humble/src)
unzip csen934_project_pkg.zip
cd ~/ros2_ws && colcon build --symlink-install && source install/setup.bash
```
`Webots_Worlds_Controller/` keeps the capitalised `Controllers/` and `Worlds/`
names from the milestone tree; the lower-case `controllers` and `worlds`
symlinks next to them are what Webots resolves on Linux. If your unzip tool
dropped the symlinks, recreate them:
`cd csen934_project_pkg/Webots_Worlds_Controller && ln -s Controllers controllers && ln -s Worlds worlds`.

## 4. Run

Every terminal: `source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash`.

```bash
# T1: Webots (must be started from a sourced shell so the controller finds rclpy)
webots ~/ros2_ws/src/csen934_project_pkg/Webots_Worlds_Controller/worlds/city.wbt
# T2: control + noise + EKF + plot helper, with rqt_plot windows for x, y, yaw
ros2 launch csen934_project_pkg localization.launch.py plot:=true
```
Spec-literal equivalent of T2 (the milestone text says `my_robot_pkg`; the
required package name is `csen934_project_pkg`):
```bash
ros2 run csen934_project_pkg control.py
ros2 run csen934_project_pkg localization_add_noise.py
ros2 run csen934_project_pkg localization_ekf_node.py
ros2 run csen934_project_pkg visualization_helper.py   # needed for yaw plots
```
SLAM, **one at a time**, with T1 and T2 running (restart the Webots
simulation, *Reload World*, between trials so each starts from the same pose):
```bash
ros2 launch csen934_project_pkg slam_toolbox_2d.launch.py
ros2 launch csen934_project_pkg cartographer_2d.launch.py
ros2 launch csen934_project_pkg lio_sam_3d.launch.py
```
The road is 1.4 km long: at 10 m/s the car reaches its end after ~2 min.

Alternative if you want controller logs in a terminal: set the car's
`controller` field to `"<extern>"` in `city.wbt`, start Webots, then run
`webots-controller --robot-name=Tesla Webots_Worlds_Controller/Controllers/CSEN934_Test_OL_Sensors/CSEN934_Test_OL_Sensors.py`
from a sourced shell.

## 5. Testing checklist

| # | Check | Command | Pass criterion |
|---|---|---|---|
| 1 | Node names | `ros2 node list` | `/Tesla_Node`, `/Control_Node`, `/Localization_Add_Noise_Node`, `/Localization_EKF_Node` |
| 0 | Controller start | Webots console | `Sensors: gps=gps, imu=inertial unit, ...`, `base_link -> lidar_link = (1.11, 0.00, 1.51) m, measured (supervisor)` |
| 2 | GT rate | `ros2 topic hz /Car_Odom_GT` | ≈ 100 Hz (basicTimeStep 10 ms) |
| 3 | GT content | `ros2 topic echo --once /Car_Odom_GT` | frame `odom`/`base_link`, covariances all 0, quaternion normalised |
| 4 | Control | `ros2 topic echo /Car_Odom_GT --field twist.twist.linear.x` | rises to ≈ 10.0 and holds; `linear.y` ≈ 0 |
| 5 | Steering sign | stop T2, then `ros2 topic pub -r 10 /Car_Ackrm ackermann_msgs/msg/AckermannDrive "{speed: 5.0, steering_angle: 0.2}"` | car turns **left**, `angular.z` > 0; else `ros2 param set /Tesla_Node steering_sign -1.0` |
| 6 | Noise | `ros2 topic echo --once /Car_Odom_Noisy --field pose.covariance` | `[0]=[7]=0.09`, `[35]=3.05e-4`; analyze_bag shows noisy std ≈ 0.3 m, 1°, 0.3 m/s, 1°/s |
| 7 | EKF | watch T2 log (`Visualization_Helper_Node`) | Filtered RMSE clearly below Noisy RMSE for x, y, yaw |
| 8 | Graph | `ros2 run rqt_graph rqt_graph` | screenshot for the deliverable |
| 8b | Lidar frame | RViz, fixed frame `base_link`, show `/Car_Lidar_Points` | road surface ≈ 1.9 m below the lidar; buildings on the car's right appear at y < 0 |
| 9 | 2D SLAM | RViz `slam_2d.rviz` | `/map` grows, green SLAM path overlays white GT path |
| 10 | 3D SLAM | RViz `slam_3d.rviz` | `/lio_sam/mapping/map_global` builds; LIO-SAM path follows GT |
| 11 | Unit tests | `colcon test --packages-select csen934_project_pkg && colcon test-result --verbose` | EKF tests + flake8 pass |

## 6. Recording deliverables

```bash
# Localization bag (small) -> report numbers
ros2 bag record -o m1_localization /clock /tf /tf_static /Car_Ackrm \
  /Car_Odom_GT /Car_Odom_Noisy /Car_Odom_Filtered /Car_Imu
ros2 run csen934_project_pkg analyze_bag m1_localization --out results
# SLAM bag per trial (point clouds are large; record ~1-2 min)
ros2 bag record -o m1_slam_toolbox /clock /tf /tf_static /scan /map /Slam_Path /Car_Path_GT /Car_Odom_GT
ros2 bag record -o m1_lio_sam /clock /tf /tf_static /Car_Lidar_Points /Car_Imu \
  /lio_sam/mapping/path /lio_sam/mapping/map_global /Car_Path_GT
# Save each 2D map
ros2 run nav2_map_server map_saver_cli -f map_slam_toolbox   # needs ros-humble-nav2-map-server
```
Videos: Webots has built-in *File → Make Movie*; RViz can be recorded with any
screen recorder on the host (e.g. OBS / SimpleScreenRecorder on Arch).

## 7. Notes and limits

* **Not run here.** The package was written and linted without a ROS or
  Webots install; the EKF and helper math were checked offline. Run the
  checklist above on first launch.
* **Lidar read path.** `Helping_Functions.lidar_to_points` reads Webots' own
  point cloud through the C binding (fast, exact axes) and falls back to
  rebuilding points from the range image. If check 8b shows a mirrored or
  upside-down cloud, the fallback is in use and its conventions need flipping.
* **Steering sign.** Steering is 0 for this milestone; check 5 confirms the
  sign before later milestones use it.
* **Straight-line SLAM.** With constant 10 m/s and 0 rad there is no loop, so
  no loop closure happens; the comparison is about drift along the road and
  map quality. Building gaps and poles give the scan matcher structure.
* The 2D trials use the GT `odom → base_link` TF. For a more realistic trial,
  set `Tesla_Node publish_tf:=false` and EKF `publish_tf:=true` so SLAM runs on
  the filtered odometry (`ros2 param set /Tesla_Node publish_tf false`).
