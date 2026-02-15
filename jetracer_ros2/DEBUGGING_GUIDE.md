# JetRacer ROS 2 — Debugging & Reference Guide

> **Package:** `jetracer` | **ROS:** Jazzy | **Robot:** Jetson (headless, SSH) | **Viz:** Laptop (rviz2 / browser)

---

## Table of Contents

1. [Build & Source](#1-build--source)
2. [Quick Start Scenarios](#2-quick-start-scenarios)
3. [Launch Files](#3-launch-files)
4. [Visualization](#4-visualization)
5. [Saving Maps](#5-saving-maps)
6. [Troubleshooting](#6-troubleshooting)
7. [Reference](#7-reference)
8. [Dependencies](#8-dependencies)
9. [File Map](#9-file-map)

---

## 1. Build & Source

```bash
cd ~/jetracerWork
colcon build --packages-select jetracer
source install/setup.bash
```

---

## 2. Quick Start Scenarios

### A) SLAM + Navigation + Multipoint Patrol

```bash
# Terminal 1: SLAM + Nav2 stack (driver + lidar + camera + SLAM + planner/controller)
ros2 launch jetracer slam_nav.launch.py

# Terminal 2: Camera web stream → http://<jetson-ip>:8080
ros2 run jetracer web_camera_stream.py

# Terminal 3: Multipoint patrol
ros2 run jetracer multipoint_nav.py

# Laptop/VM: rviz2 → use 'Publish Point' to add waypoints

# Save map when done:
cd ~/jetracerWork/src/jetracer_ros2/maps && ./savemap.sh
```

### B) Navigation on a Saved Map

```bash
# Terminal 1: Driver + LiDAR
ros2 launch jetracer jetracer.launch.py
ros2 launch jetracer lidar.launch.py

# Terminal 2: Nav2 with map (includes AMCL + map_server)
ros2 launch jetracer nav.launch.py map:=/path/to/maps/mymap.yaml

# Terminal 3: Multipoint patrol or teleop
ros2 run jetracer multipoint_nav.py

# Laptop: rviz2 → '2D Pose Estimate' first, then 'Nav2 Goal' or 'Publish Point'
```

### C) Quick Teleop Test

```bash
ros2 launch jetracer slam.launch.py
ros2 run teleop_twist_keyboard teleop_twist_keyboard
# Laptop browser: http://<jetson-ip>:8080
```

### Healthy Startup Log

```
[SERIAL] ✓ Port opened successfully!
[CONFIG] ✓ Initial configuration sent
[RX] ✓ Serial receive thread started
[STATUS] Serial:OK | Cmds:0 | Sent:50 | Recv:50 | ChkErr:0 | WrErr:0
```

---

## 3. Launch Files

| Launch File                  | What It Starts                                          | Notes                                             |
| ---------------------------- | ------------------------------------------------------- | ------------------------------------------------- |
| `jetracer.launch.py`         | Driver + odom_ekf + TF                                  | Base — motors, odom, IMU                          |
| `lidar.launch.py`            | rplidar_node + TF                                       | Port: `/dev/ttyACM1`                              |
| `csi_camera.launch.py`       | csi_camera_node                                         | OpenCV + GStreamer (`nvarguscamerasrc`)           |
| `slam.launch.py`             | jetracer + lidar + camera + SLAM                        | `slam_method:=slam_toolbox\|cartographer\|hector` |
| `nav.launch.py`              | Nav2 bringup (AMCL + map_server + planner + controller) | Requires `map:=` argument                         |
| `slam_nav.launch.py`         | SLAM + Nav2 (no AMCL — SLAM provides map→odom)          | Uses `navigation_launch.py`                       |
| `slam_toolbox.launch.py`     | SLAM Toolbox node                                       | Standalone SLAM                                   |
| `cartographer.launch.py`     | Cartographer node                                       | Standalone SLAM                                   |
| `hector.launch.py`           | Hector SLAM node                                        | May not be available for Jazzy                    |
| `amcl.launch.py`             | AMCL node                                               | Localization only (no map_server)                 |
| `laser_filter.launch.py`     | Laser scan filter                                       | Range filter 0.2–8.0m                             |
| `calibrate_linear.launch.py` | Odometry calibration tool                               | Drive straight test                               |

> **`nav.launch.py` vs `slam_nav.launch.py`:** `nav` starts AMCL + map_server (needs a saved map). `slam_nav` skips AMCL (SLAM handles `map→odom`). Do NOT run both — you'll get conflicting TF transforms.

---

## 4. Visualization

### rviz2 (on PC/VM with ROS 2)

```bash
export ROS_DOMAIN_ID=0   # must match Jetson
rviz2 -d $(ros2 pkg prefix jetracer)/share/jetracer/config/nav2_view.rviz
```

Pre-configured displays: Map, LaserScan, TF, Global/Local Plan, AMCL Particles, Camera, Waypoint Markers, Robot Footprint.  
Tools: SetInitialPose (`/initialpose`), SetGoal (`/goal_pose`), PublishPoint (`/clicked_point`).

### Web Camera Stream (no ROS 2 needed on PC)

```bash
# On Jetson:
ros2 launch jetracer csi_camera.launch.py &
ros2 run jetracer web_camera_stream.py
```

| URL                                | Purpose           |
| ---------------------------------- | ----------------- |
| `http://<jetson-ip>:8080`          | Live MJPEG page   |
| `http://<jetson-ip>:8080/stream`   | Raw MJPEG stream  |
| `http://<jetson-ip>:8080/snapshot` | Single JPEG frame |

### Foxglove Studio (no ROS 2 needed on PC)

```bash
# Jetson: install bridge
sudo apt install ros-${ROS_DISTRO}-foxglove-bridge
ros2 launch foxglove_bridge foxglove_bridge_launch.xml

# PC: download from https://foxglove.dev/, connect to ws://<jetson-ip>:8765
```

---

## 5. Saving Maps

```bash
# SLAM Toolbox / Hector:
cd ~/jetracerWork/src/jetracer_ros2/maps
./savemap.sh              # → mymap.yaml + mymap.pgm
./savemap.sh my_room      # → my_room.yaml + my_room.pgm

# Cartographer:
./carto_savemap.sh        # finishes trajectory, writes pbstream, converts to yaml+pgm

# Direct CLI (any SLAM that publishes /map):
ros2 run nav2_map_server map_saver_cli -f ~/jetracerWork/src/jetracer_ros2/maps/mymap
```

---

## 6. Troubleshooting

### Hardware & Serial

| Problem                   | Fix                                                                                                          |
| ------------------------- | ------------------------------------------------------------------------------------------------------------ |
| Serial port not found     | `ls /dev/ttyACM*` — expect ACM0 (MCU), ACM1 (LiDAR)                                                          |
| Permission denied         | `sudo usermod -aG dialout $USER` then re-login                                                               |
| Port busy                 | `sudo fuser -k /dev/ttyACM0`                                                                                 |
| No odom / IMU data        | MCU not responding — `ros2 run jetracer jetracer --ros-args --log-level debug`                               |
| Motors don't move         | Check battery; try `ros2 param set /jetracer coefficient_d 1480\|1500\|1520`; check `/cmd_vel` is publishing |
| Tegra lib errors (Docker) | `export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/lib/aarch64-linux-gnu/tegra`                                   |

### Camera

| Problem                 | Fix                                                                                                                          |
| ----------------------- | ---------------------------------------------------------------------------------------------------------------------------- |
| All green frames        | Must use `nvarguscamerasrc` pipeline (not v4l2)                                                                              |
| No image published      | Verify GStreamer: `python3 -c "import cv2; print('GStreamer' in cv2.getBuildInformation())"`                                 |
| "Failed to open camera" | Test: `gst-launch-1.0 nvarguscamerasrc sensor-id=0 ! 'video/x-raw(memory:NVMM),width=640,height=480' ! nvvidconv ! fakesink` |
| No camera_info          | Check `config/camera_calibration/cam_640x480.yaml` is installed                                                              |

### SLAM

| Problem                              | Fix                                                                                                      |
| ------------------------------------ | -------------------------------------------------------------------------------------------------------- |
| Map not building                     | Check LiDAR: `ros2 topic hz /scan`                                                                       |
| Map drifting / doubling              | Tune `slam_toolbox_params.yaml` — increase `minimum_travel_distance`, `loop_match_minimum_response_fine` |
| TF timeout errors                    | `ros2 run tf2_tools view_frames` — check for gaps in the chain                                           |
| "Lookup would require extrapolation" | Ensure `use_sim_time` is consistent across all nodes                                                     |

### Navigation

| Problem                           | Fix                                                                    |
| --------------------------------- | ---------------------------------------------------------------------- |
| "Waiting for navigate_to_pose..." | Nav2 not running — start `nav.launch.py` or `slam_nav.launch.py`       |
| Goal rejected                     | Lifecycle nodes not active yet — `ros2 lifecycle list /bt_navigator`   |
| Robot stuck / oscillating         | Check `robot_radius` (0.12), `inflation_radius`, controller tolerances |
| Multipoint waypoints not visible  | Add MarkerArray display for `/path_point` in rviz2                     |

---

## 7. Reference

### Topics (after `slam.launch.py`)

| Topic                        | Hz    | Source          |
| ---------------------------- | ----- | --------------- |
| `/cmd_vel`                   | —     | teleop / Nav2   |
| `/odom`                      | ~20   | odom_ekf        |
| `/odom_raw`                  | ~20   | jetracer driver |
| `/imu`                       | ~20   | jetracer driver |
| `/scan`                      | ~5-10 | rplidar_node    |
| `/map`                       | ~1    | SLAM node       |
| `/csi_cam_0/image_raw`       | ~20   | csi_camera_node |
| `/motor/lvel`, `/motor/rvel` | ~20   | jetracer driver |

```bash
ros2 topic hz /scan       # verify rates
ros2 topic echo /odom --once
```

### TF Tree

```
map → odom → base_footprint → base_imu_link
                             → laser_frame
                             → csi_cam_0_link
```

```bash
ros2 run tf2_tools view_frames        # generates frames.pdf
ros2 run tf2_ros tf2_echo map base_footprint
```

### Motor Commands

```bash
# Keyboard teleop:
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# Manual commands:
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}, angular: {z: 0.0}}" --once  # forward
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}" --once  # stop

# Nav2 goal from CLI:
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
    "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 1.0, y: 0.0}, orientation: {w: 1.0}}}}"
```

### Parameters (config/jetracer.yaml)

```bash
ros2 param list /jetracer
ros2 param set /jetracer servo_bias 10
ros2 param set /jetracer kp 400
```

| Parameter           | Default        | Description           |
| ------------------- | -------------- | --------------------- |
| `port_name`         | `/dev/ttyACM0` | Serial port           |
| `kp` / `ki` / `kd`  | 350 / 120 / 0  | PID gains             |
| `coefficient_d`     | 1500.0         | Steering center PWM   |
| `servo_bias`        | 0              | Steering trim         |
| `linear_correction` | 1.0            | Odometry scale factor |

### Serial Protocol

| Packet       | Header  | Size | Type | Data                              | Checksum |
| ------------ | ------- | ---- | ---- | --------------------------------- | -------- |
| Velocity     | `AA 55` | `0B` | `11` | x(2), y(2), yaw(2)                | sum      |
| Params       | `AA 55` | `0F` | `12` | kp(2), ki(2), kd(2), lc(2), sb(2) | sum      |
| Coefficients | `AA 55` | `15` | `13` | a(4), b(4), c(4), d(4)            | sum      |

Values: `(value * 1000)` → int16, big-endian.

### Steering Calibration

```bash
ros2 launch jetracer calibrate_linear.launch.py
```

Polynomial: `PWM = a*θ³ + b*θ² + c*θ + d`

---

## 8. Dependencies

```bash
# Jetson (robot / Docker):
sudo apt install -y \
    ros-${ROS_DISTRO}-slam-toolbox \
    ros-${ROS_DISTRO}-cartographer-ros \
    ros-${ROS_DISTRO}-nav2-bringup \
    ros-${ROS_DISTRO}-nav2-msgs \
    ros-${ROS_DISTRO}-robot-localization \
    ros-${ROS_DISTRO}-laser-filters \
    ros-${ROS_DISTRO}-rplidar-ros \
    ros-${ROS_DISTRO}-teleop-twist-keyboard

# PC/VM (for rviz2 visualization):
sudo apt install -y \
    ros-${ROS_DISTRO}-rviz2 \
    ros-${ROS_DISTRO}-nav2-rviz-plugins \
    ros-${ROS_DISTRO}-teleop-twist-keyboard
```

Camera uses OpenCV with GStreamer (pre-installed on Jetson) — no extra packages needed.

---

## 9. File Map

| File                                         | Purpose                                       |
| -------------------------------------------- | --------------------------------------------- |
| **Launch**                                   |                                               |
| `launch/jetracer.launch.py`                  | Driver + odom_ekf + TF                        |
| `launch/lidar.launch.py`                     | RPLiDAR + TF                                  |
| `launch/csi_camera.launch.py`                | CSI camera (OpenCV+GStreamer)                 |
| `launch/slam.launch.py`                      | Unified SLAM (driver + lidar + camera + SLAM) |
| `launch/slam_nav.launch.py`                  | SLAM + Nav2 (no AMCL)                         |
| `launch/nav.launch.py`                       | Nav2 with pre-built map                       |
| `launch/slam_toolbox.launch.py`              | SLAM Toolbox standalone                       |
| `launch/cartographer.launch.py`              | Cartographer standalone                       |
| `launch/hector.launch.py`                    | Hector SLAM standalone                        |
| `launch/amcl.launch.py`                      | AMCL localization                             |
| `launch/laser_filter.launch.py`              | Laser scan filter                             |
| `launch/calibrate_linear.launch.py`          | Odometry calibration                          |
| **Config**                                   |                                               |
| `config/jetracer.yaml`                       | Driver parameters                             |
| `config/nav2_params.yaml`                    | Nav2 stack parameters                         |
| `config/slam_toolbox_params.yaml`            | SLAM Toolbox parameters                       |
| `config/amcl_params.yaml`                    | AMCL parameters                               |
| `config/laser_filter_params.yaml`            | Laser filter parameters                       |
| `config/nav2_view.rviz`                      | rviz2 navigation layout                       |
| `config/cartographer/jetracer.lua`           | Cartographer config                           |
| `config/camera_calibration/cam_640x480.yaml` | Camera intrinsics                             |
| **Scripts**                                  |                                               |
| `scripts/csi_camera_node.py`                 | CSI camera node                               |
| `scripts/web_camera_stream.py`               | Browser MJPEG stream                          |
| `scripts/odom_ekf.py`                        | Odometry EKF filter                           |
| `scripts/calibrate_linear.py`                | Linear calibration tool                       |
| `scripts/multipoint_nav.py`                  | Multi-waypoint patrol                         |
| **Source**                                   |                                               |
| `src/jetracer.cpp`                           | Core driver (serial, motors, odom, IMU)       |
| **Maps**                                     |                                               |
| `maps/savemap.sh`                            | Save map (SLAM Toolbox/Hector)                |
| `maps/carto_savemap.sh`                      | Save map (Cartographer)                       |
