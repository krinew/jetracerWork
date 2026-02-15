# JetRacer ROS2 Debugging Guide

> **Package name:** `jetracer`
> **ROS distro:** Jazzy
> **Setup:** Jetson accessed via SSH (no GUI on the robot)
> **Laptop:** Used for visualization (browser, rviz2)

---

## 0. Build & Source (do this first)

```bash
cd ~/jetracerWork
colcon build --packages-select jetracer
source install/setup.bash
```

---

## 1. Hardware Checks

### Serial port (MCU)

```bash
ls -la /dev/ttyACM* /dev/ttyUSB* 2>/dev/null
# Expect /dev/ttyACM0 for MCU, /dev/ttyACM1 for LiDAR

# Permission fix (one time, then re-login):
sudo usermod -aG dialout $USER
```

### USB devices

```bash
lsusb
dmesg | tail -20
```

### Kill a stuck serial port

```bash
sudo fuser /dev/ttyACM0        # who's using it?
sudo fuser -k /dev/ttyACM0     # kill it
```

---

## 2. Launch Files — What to Run

### Base driver only

```bash
ros2 launch jetracer jetracer.launch.py
```

Starts: `jetracer` node (serial/motors/odom/IMU), `odom_ekf` node, static TF `base_footprint → base_imu_link`.

### LiDAR only

```bash
ros2 launch jetracer lidar.launch.py
# Override port: ros2 launch jetracer lidar.launch.py serial_port:=/dev/ttyUSB0
```

Starts: `rplidar_node`, static TF `base_footprint → laser_frame`.

### Camera only

```bash
ros2 launch jetracer csi_camera.launch.py
```

Starts: custom `csi_camera_node.py` using OpenCV + GStreamer (`nvarguscamerasrc → NV12 → nvvidconv → videoconvert → BGR`).
Publishes to `/csi_cam_0/image_raw` and `/csi_cam_0/camera_info`.
Loads calibration from `config/camera_calibration/cam_640x480.yaml`.
No extra apt packages needed — uses OpenCV (pre-installed on Jetson) with GStreamer backend.

### SLAM (includes driver + lidar + camera)

```bash
# Default: slam_toolbox
ros2 launch jetracer slam.launch.py

# Use cartographer instead:
ros2 launch jetracer slam.launch.py slam_method:=cartographer

# Use hector_slam:
ros2 launch jetracer slam.launch.py slam_method:=hector
```

### SLAM + Navigation (full autonomous stack)

```bash
ros2 launch jetracer slam_nav.launch.py
# or with cartographer:
ros2 launch jetracer slam_nav.launch.py slam_method:=cartographer
```

> **Note:** `slam_nav.launch.py` uses `navigation_launch.py` (NOT `bringup_launch.py`).
> This means it starts only the planner/controller/recovery stack — no AMCL or map_server.
> SLAM provides the `map → odom` transform, so AMCL would conflict.

### Navigation only (with pre-built map)

```bash
# Starts full Nav2 bringup: map_server + AMCL + planner + controller + recoveries
ros2 launch jetracer nav.launch.py map:=/path/to/mymap.yaml

# With base driver + lidar (run separately if not already running):
ros2 launch jetracer jetracer.launch.py
ros2 launch jetracer lidar.launch.py
```

### Multi-point patrol navigation

```bash
# Requires navigation stack to be running (nav.launch.py or slam_nav.launch.py)
ros2 run jetracer multipoint_nav.py
```

Usage in rviz2:

1. **"Publish Point"** tool — click on the map to add waypoints (numbered red markers)
2. Robot navigates to each waypoint in order, then loops
3. If a waypoint is unreachable, retries once then skips to next
4. **"2D Pose Estimate"** — resets all waypoints and cancels current goal

---

## 3. Camera — Viewing the Feed (SSH / Browser)

### Option A: Browser MJPEG stream (recommended for SSH)

```bash
# On the Jetson:
ros2 launch jetracer csi_camera.launch.py &
ros2 run jetracer web_camera_stream.py
```

Open **`http://<jetson-ip>:8080`** on your laptop browser.

| URL                                | Purpose                        |
| ---------------------------------- | ------------------------------ |
| `http://<jetson-ip>:8080`          | Live MJPEG stream page         |
| `http://<jetson-ip>:8080/stream`   | Raw MJPEG (embeddable in apps) |
| `http://<jetson-ip>:8080/snapshot` | Single JPEG frame              |

**Tune quality / resolution:**

```bash
ros2 run jetracer web_camera_stream.py --ros-args \
    -p image_topic:=/csi_cam_0/image_raw \
    -p port:=8080 \
    -p quality:=60 \
    -p width:=320 -p height:=240
```

### Option B: rviz2 on laptop (needs ROS2 on laptop)

Make sure both machines share the same `ROS_DOMAIN_ID` and network:

```bash
# On laptop:
export ROS_DOMAIN_ID=0   # must match Jetson
rviz2
# Add Image display → topic: /csi_cam_0/image_raw
```

### Camera troubleshooting

| Symptom                       | Cause                             | Fix                                                                                           |
| ----------------------------- | --------------------------------- | --------------------------------------------------------------------------------------------- |
| **All green frames**          | Wrong pixel format / wrong driver | Must use `nvarguscamerasrc` GStreamer pipeline (not v4l2_camera with UYVY)                    |
| **No image published**        | OpenCV not built with GStreamer   | Verify: `python3 -c "import cv2; print(cv2.getBuildInformation())"` — look for GStreamer: YES |
| **"cannot open /dev/video0"** | Not a V4L2 device / wrong driver  | CSI cameras need `nvarguscamerasrc` (GStreamer), not `/dev/video0`                            |
| **GStreamer error**           | Missing Jetson multimedia libs    | `export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/lib/aarch64-linux-gnu/tegra`                    |
| **No camera_info**            | Calibration file missing          | Check `config/camera_calibration/cam_640x480.yaml` is installed                               |
| **"Failed to open camera"**   | nvarguscamerasrc not available    | Camera not connected or argus daemon not running — test pipeline below                        |

```bash
# Test GStreamer pipeline directly:
gst-launch-1.0 nvarguscamerasrc sensor-id=0 ! \
    'video/x-raw(memory:NVMM), width=640, height=480, format=NV12, framerate=20/1' ! \
    nvvidconv ! videoconvert ! fakesink

# Verify OpenCV has GStreamer support:
python3 -c "import cv2; print('GStreamer' in cv2.getBuildInformation())"
```

---

## 4. Topic Checks

### Expected topics after `slam.launch.py`

```bash
ros2 topic list
```

| Topic                    | Source                      | Description                |
| ------------------------ | --------------------------- | -------------------------- |
| `/cmd_vel`               | teleop / Nav2               | Velocity commands to motor |
| `/odom_raw`              | jetracer node               | Raw wheel odometry         |
| `/odom`                  | odom_ekf node               | Filtered odometry          |
| `/imu`                   | jetracer node               | IMU data from MCU          |
| `/scan`                  | rplidar_node                | LiDAR scan                 |
| `/csi_cam_0/image_raw`   | csi_camera_node             | Camera image               |
| `/csi_cam_0/camera_info` | csi_camera_node             | Camera intrinsics          |
| `/map`                   | slam_toolbox / cartographer | Occupancy grid             |
| `/motor/lvel`            | jetracer node               | Left motor velocity        |
| `/motor/rvel`            | jetracer node               | Right motor velocity       |
| `/motor/lset`            | jetracer node               | Left motor setpoint        |
| `/motor/rset`            | jetracer node               | Right motor setpoint       |

### Monitor key topics

```bash
ros2 topic echo /odom --once
ros2 topic echo /scan --once
ros2 topic echo /csi_cam_0/camera_info --once
ros2 topic hz /scan          # should be ~5-10 Hz
ros2 topic hz /odom          # should be ~20 Hz
ros2 topic hz /csi_cam_0/image_raw   # should be ~20 fps
```

---

## 5. TF Tree

Expected transform chain:

```
map → odom → base_footprint → base_imu_link
                             → laser_frame
                             → csi_cam_0_link
```

```bash
# Check TF tree
ros2 run tf2_tools view_frames
# Generates frames.pdf

# Check a specific transform
ros2 run tf2_ros tf2_echo map base_footprint
```

---

## 6. Send Test Commands

```bash
# Stop
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}" --once

# Forward
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}, angular: {z: 0.0}}" --once

# Turn in place
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.5}}" --once

# Continuous at 10 Hz
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}, angular: {z: 0.0}}" -r 10

# Keyboard teleop (install: sudo apt install ros-${ROS_DISTRO}-teleop-twist-keyboard)
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

---

## 7. Parameters

### View / modify at runtime

```bash
ros2 param list /jetracer
ros2 param get /jetracer port_name
ros2 param get /jetracer coefficient_d

# Tune servo
ros2 param set /jetracer servo_bias 10
ros2 param set /jetracer coefficient_d 1500

# Tune PID
ros2 param set /jetracer kp 400
ros2 param set /jetracer ki 100
```

### Default parameters (config/jetracer.yaml)

```yaml
jetracer:
  ros__parameters:
    port_name: "/dev/ttyACM0"
    publish_odom_transform: true
    linear_correction: 1.0
    coefficient_a: -0.016073
    coefficient_b: 0.176183
    coefficient_c: -23.428084
    coefficient_d: 1500.0
    kp: 350
    ki: 120
    kd: 0
    servo_bias: 0
```

---

## 8. Serial Port Debugging

### Monitor raw serial

```bash
sudo apt install screen
sudo screen /dev/ttyACM0 115200
# Ctrl+A then K to exit
```

### Python serial test

```bash
python3 << 'EOF'
import serial, time
try:
    ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
    print("Port opened OK")
    cmd = bytes([0xAA, 0x55, 0x0B, 0x11, 0x00, 0x64, 0x00, 0x64, 0x00, 0x00])
    checksum = sum(cmd) & 0xFF
    ser.write(cmd + bytes([checksum]))
    time.sleep(0.1)
    if ser.in_waiting > 0:
        print(f"Response: {ser.read(ser.in_waiting).hex()}")
    else:
        print("No response!")
    ser.close()
except Exception as e:
    print(f"Error: {e}")
EOF
```

### Protocol format

| Packet       | Header  | Size | Type | Data                              | Checksum |
| ------------ | ------- | ---- | ---- | --------------------------------- | -------- |
| Velocity     | `AA 55` | `0B` | `11` | x(2), y(2), yaw(2)                | sum      |
| Params       | `AA 55` | `0F` | `12` | kp(2), ki(2), kd(2), lc(2), sb(2) | sum      |
| Coefficients | `AA 55` | `15` | `13` | a(4), b(4), c(4), d(4)            | sum      |

Values encoded as `(value * 1000)` → int16, big-endian.

---

## 9. SLAM Debugging

### Save a map

**Method 1: Use the provided scripts** (recommended)

```bash
# For slam_toolbox / hector / gmapping maps:
cd ~/jetracerWork/src/jetracer_ros2/maps
./savemap.sh              # saves as mymap.yaml + mymap.pgm
./savemap.sh my_room      # saves as my_room.yaml + my_room.pgm

# For cartographer maps (converts pbstream → yaml+pgm):
./carto_savemap.sh
./carto_savemap.sh my_room
```

**Method 2: Direct CLI commands**

```bash
# With slam_toolbox:
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: {data: 'mymap'}}"

# Or use nav2_map_server (works with any SLAM that publishes /map):
ros2 run nav2_map_server map_saver_cli -f ~/jetracerWork/src/jetracer_ros2/maps/mymap

# With cartographer (3-step process):
ros2 service call /finish_trajectory cartographer_ros_msgs/srv/FinishTrajectory "{trajectory_id: 0}"
ros2 service call /write_state cartographer_ros_msgs/srv/WriteState "{filename: '/path/to/mymap.pbstream'}"
ros2 run cartographer_ros cartographer_pbstream_to_ros_map \
    -pbstream_filename=/path/to/mymap.pbstream \
    -map_filestem=/path/to/mymap
```

**After saving**, use the map with navigation:

```bash
ros2 launch jetracer nav.launch.py map:=$(pwd)/mymap.yaml
```

### Common SLAM issues

| Problem                                  | Likely cause                 | Fix                                                                                                     |
| ---------------------------------------- | ---------------------------- | ------------------------------------------------------------------------------------------------------- |
| **Map not building**                     | No `/scan` data              | Check LiDAR: `ros2 topic hz /scan`                                                                      |
| **Map drifting / doubling**              | Bad loop closure or odometry | Tune `slam_toolbox_params.yaml`: increase `minimum_travel_distance`, `loop_match_minimum_response_fine` |
| **TF timeout errors**                    | Transform chain broken       | `ros2 run tf2_tools view_frames` — check for gaps                                                       |
| **Cartographer won't start**             | Missing config               | Ensure `config/cartographer/jetracer.lua` exists and is installed                                       |
| **"Lookup would require extrapolation"** | Clock skew or slow TF        | Check `transform_timeout` param, ensure `use_sim_time` is consistent                                    |

### SLAM Toolbox tuning (config/slam_toolbox_params.yaml)

Key parameters:

- `resolution: 0.05` — map grid resolution (meters per pixel)
- `max_laser_range: 8.0` — should match your LiDAR's range
- `minimum_travel_distance: 0.2` — distance before adding a new scan
- `do_loop_closing: true` — set false to disable loop closure
- `loop_match_minimum_response_fine: 0.45` — higher = stricter loop closure

---

## 10. Navigation Debugging

### Check Nav2 is running

```bash
ros2 lifecycle list /controller_server
ros2 lifecycle list /planner_server
ros2 lifecycle list /bt_navigator
```

All should be in `active` state.

### Send a nav goal from CLI

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
    "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 1.0, y: 0.0}, orientation: {w: 1.0}}}}"
```

### Multi-point navigation debugging

```bash
# Check the node is running:
ros2 node list | grep multipoint

# Check waypoint markers are being published:
ros2 topic echo /path_point --once

# Check Nav2 action server is available:
ros2 action list | grep navigate_to_pose

# Monitor navigation result:
ros2 topic echo /navigate_to_pose/_action/status
```

| Problem                               | Cause                             | Fix                                                                  |
| ------------------------------------- | --------------------------------- | -------------------------------------------------------------------- |
| **"Waiting for navigate_to_pose..."** | Nav2 not running                  | Start nav.launch.py or slam_nav.launch.py first                      |
| **Goal rejected**                     | Nav2 not fully initialized        | Wait for all lifecycle nodes to reach `active` state                 |
| **Robot doesn't move to waypoints**   | No waypoints added                | Use "Publish Point" in rviz2 to click waypoints on the map           |
| **Robot stuck / oscillating**         | Costmap or planner issue          | Check `robot_radius` (0.12), inflation_radius, controller tolerances |
| **Waypoints not visible**             | rviz2 missing MarkerArray display | Add MarkerArray display for `/path_point` topic                      |

### Navigation params (config/nav2_params.yaml)

Key sections:

- `controller_server` — RegulatedPurePursuitController (desired_vel=0.5, lookahead=0.6)
- `planner_server` — NavfnPlanner (tolerance=0.5)
- `bt_navigator` — Behavior tree navigator with recovery plugins
- `amcl` — Only used by `nav.launch.py` (NOT by `slam_nav.launch.py`)

> **Important:** `nav.launch.py` uses `bringup_launch.py` (includes AMCL + map_server).
> `slam_nav.launch.py` uses `navigation_launch.py` (planning/control only — no AMCL).
> Do NOT mix them — you'll get conflicting `map → odom` transforms.

---

## 10a. Visualization with rviz2

rviz2 is the primary visualization tool. Since it's a GUI app, run it on a **PC or VM with a display** — not on the headless Jetson Docker container.

### Setup: PC/VM with ROS2

```bash
# Make sure both machines are on the same network
export ROS_DOMAIN_ID=0   # must match the Jetson

# Launch rviz2 with the pre-configured navigation layout:
rviz2 -d $(ros2 pkg prefix jetracer)/share/jetracer/config/nav2_view.rviz

# Or just launch rviz2 and add displays manually:
rviz2
```

### Pre-configured rviz2 layout (config/nav2_view.rviz)

Displays included:

| Display         | Topic                                | Description                         |
| --------------- | ------------------------------------ | ----------------------------------- |
| TF              | —                                    | Transform tree                      |
| Map             | `/map`                               | Occupancy grid from SLAM/map_server |
| LaserScan       | `/scan`                              | LiDAR points (red)                  |
| Global Plan     | `/plan`                              | Green path line                     |
| Local Plan      | `/local_plan`                        | Yellow path line                    |
| Global Costmap  | `/global_costmap/costmap`            | (off by default)                    |
| Local Costmap   | `/local_costmap/costmap`             | (off by default)                    |
| AMCL Particles  | `/particle_cloud`                    | Blue particle cloud                 |
| Camera          | `/csi_cam_0/image_raw`               | Live camera feed                    |
| Waypoints       | `/path_point`                        | Red numbered markers                |
| Robot Footprint | `/local_costmap/published_footprint` | Green robot outline                 |

Tools:

- **SetInitialPose** → publishes to `/initialpose` (set robot position on map)
- **SetGoal** → publishes to `/goal_pose` (Nav2 single goal)
- **PublishPoint** → publishes to `/clicked_point` (adds multipoint waypoints)

### Alternative: Foxglove Studio (no ROS2 install needed on PC)

```bash
# On the Jetson Docker container, install foxglove_bridge:
sudo apt install ros-${ROS_DISTRO}-foxglove-bridge
ros2 launch foxglove_bridge foxglove_bridge_launch.xml

# On your Windows PC, download Foxglove Studio from https://foxglove.dev/
# Connect to: ws://<jetson-ip>:8765
```

---

## 11. Common Issues & Solutions

### Issue: "Failed to open serial port"

```bash
sudo fuser /dev/ttyACM0        # check what's using it
sudo fuser -k /dev/ttyACM0     # kill it
sudo chmod 666 /dev/ttyACM0    # quick permission fix
```

### Issue: Motors don't move

- Check battery voltage
- Try sweeping `coefficient_d`: `ros2 param set /jetracer coefficient_d 1480` / `1500` / `1520`
- Listen for ESC arming beep
- Check `/cmd_vel` is publishing: `ros2 topic echo /cmd_vel`

### Issue: No odom / IMU data

- MCU not powered or not responding
- Check debug logs: `ros2 run jetracer jetracer --ros-args --log-level debug`
- Look for `[RX] Checksum ERROR` or `Recv:0` in status lines

### Issue: Tegra library errors in Docker

```bash
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/lib/aarch64-linux-gnu/tegra
```

---

## 12. Steering Calibration

The steering uses a polynomial: `PWM = a*θ³ + b*θ² + c*θ + d`

```bash
# Run the calibration tool:
ros2 launch jetracer calibrate_linear.launch.py

# Or manually sweep steering:
python3 << 'EOF'
import serial, time

def send_velocity(ser, x, yaw):
    x_enc = int(x * 1000)
    yaw_enc = int(yaw * 1000)
    cmd = bytes([
        0xAA, 0x55, 0x0B, 0x11,
        (x_enc >> 8) & 0xFF, x_enc & 0xFF,
        (x_enc >> 8) & 0xFF, x_enc & 0xFF,
        (yaw_enc >> 8) & 0xFF, yaw_enc & 0xFF
    ])
    checksum = sum(cmd) & 0xFF
    ser.write(cmd + bytes([checksum]))

ser = serial.Serial('/dev/ttyACM0', 115200)
for yaw in [-1.0, -0.5, 0.0, 0.5, 1.0]:
    print(f"Setting yaw={yaw}")
    for _ in range(10):
        send_velocity(ser, 0.0, yaw)
        time.sleep(0.05)
    input("Press Enter for next value...")
ser.close()
EOF
```

---

## 13. Full System Test (step by step)

### Scenario A: SLAM + Navigation + Multipoint patrol

```bash
# Terminal 1: Full SLAM + Nav stack
ros2 launch jetracer slam_nav.launch.py

# Terminal 2: Camera web stream
ros2 run jetracer web_camera_stream.py

# Terminal 3: Multi-point patrol (or teleop to drive manually first)
ros2 run jetracer multipoint_nav.py

# On laptop/VM: rviz2 -d nav2_view.rviz
# Use 'Publish Point' to add waypoints, robot will patrol them

# When done mapping, save the map:
cd ~/jetracerWork/src/jetracer_ros2/maps && ./savemap.sh
```

### Scenario B: Navigation on a saved map

```bash
# Terminal 1: Base driver + lidar
ros2 launch jetracer jetracer.launch.py
ros2 launch jetracer lidar.launch.py

# Terminal 2: Navigation with saved map (includes AMCL + map_server)
ros2 launch jetracer nav.launch.py map:=/path/to/maps/mymap.yaml

# Terminal 3: Multi-point patrol
ros2 run jetracer multipoint_nav.py

# Terminal 4: Camera web stream
ros2 run jetracer web_camera_stream.py

# On laptop/VM:
# rviz2 -d nav2_view.rviz
# 1. Set '2D Pose Estimate' to tell AMCL where the robot is
# 2. Use 'Nav2 Goal' for single-point or 'Publish Point' for multi-point
```

### Scenario C: Quick teleop test (no navigation)

```bash
# Terminal 1: Full SLAM + Nav stack
ros2 launch jetracer slam.launch.py

# Terminal 2: Camera web stream
ros2 run jetracer web_camera_stream.py

# Terminal 3: Teleop
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# On laptop browser: http://<jetson-ip>:8080
# On laptop rviz2 (optional): add /map, /scan, /odom topics
```

### Healthy startup log

```
[INIT] JetRacer ROS2 Driver Starting...
[SERIAL] ✓ Port opened successfully!
[CONFIG] ✓ Initial configuration sent
[RX] ✓ Serial receive thread started
[RX #1] ✓ Valid frame received
[STATUS] Serial:OK | Cmds:0 | Sent:50 | Recv:50 | ChkErr:0 | WrErr:0
```

### Bad signs

```
[SERIAL] ✗ FAILED to open serial port!     → Check hardware / permissions
[RX] Checksum ERROR                         → Comm issue / baud mismatch
[WATCHDOG] No cmd_vel for 1.0s             → Nothing publishing cmd_vel
[STATUS] Serial:OK | Recv:0                 → MCU not responding
```

---

## 14. Installed Dependencies

```bash
# On the Jetson (robot / Docker container):
sudo apt install -y \
    ros-${ROS_DISTRO}-slam-toolbox \
    ros-${ROS_DISTRO}-cartographer-ros \
    ros-${ROS_DISTRO}-nav2-bringup \
    ros-${ROS_DISTRO}-nav2-msgs \
    ros-${ROS_DISTRO}-robot-localization \
    ros-${ROS_DISTRO}-laser-filters \
    ros-${ROS_DISTRO}-rplidar-ros \
    ros-${ROS_DISTRO}-teleop-twist-keyboard

# On a separate PC/VM for visualization (optional):
sudo apt install -y \
    ros-${ROS_DISTRO}-rviz2 \
    ros-${ROS_DISTRO}-nav2-rviz-plugins \
    ros-${ROS_DISTRO}-teleop-twist-keyboard

# Or use Foxglove Studio on Windows (no ROS2 needed on PC):
# Install foxglove-bridge on the Jetson:
sudo apt install -y ros-${ROS_DISTRO}-foxglove-bridge
```

Camera uses a custom Python node — no extra apt packages. Just needs OpenCV with GStreamer (pre-installed on Jetson):

```bash
# Verify OpenCV GStreamer support:
python3 -c "import cv2; bi = cv2.getBuildInformation(); print('GStreamer:', 'YES' in bi[bi.find('GStreamer'):])"

# If OpenCV missing (unlikely on Jetson):
pip3 install opencv-python
```

---

## 15. File Reference

| File                                         | Purpose                                   |
| -------------------------------------------- | ----------------------------------------- |
| `launch/jetracer.launch.py`                  | Driver + odom_ekf + TF                    |
| `launch/lidar.launch.py`                     | RPLiDAR + TF                              |
| `launch/csi_camera.launch.py`                | CSI camera via OpenCV+GStreamer           |
| `launch/slam.launch.py`                      | Driver + LiDAR + Camera + SLAM (unified)  |
| `launch/slam_nav.launch.py`                  | SLAM + Nav2 combined                      |
| `launch/nav.launch.py`                       | Nav2 with pre-built map                   |
| `launch/slam_toolbox.launch.py`              | SLAM Toolbox node                         |
| `launch/cartographer.launch.py`              | Cartographer SLAM node                    |
| `launch/hector.launch.py`                    | Hector SLAM node                          |
| `launch/amcl.launch.py`                      | AMCL localization                         |
| `launch/laser_filter.launch.py`              | Laser scan filtering                      |
| `launch/calibrate_linear.launch.py`          | Odometry calibration                      |
| `config/jetracer.yaml`                       | Driver parameters                         |
| `config/nav2_params.yaml`                    | Nav2 stack parameters                     |
| `config/slam_toolbox_params.yaml`            | SLAM Toolbox parameters                   |
| `config/amcl_params.yaml`                    | AMCL parameters                           |
| `config/laser_filter_params.yaml`            | Laser filter parameters                   |
| `config/cartographer/jetracer.lua`           | Cartographer configuration                |
| `config/camera_calibration/cam_640x480.yaml` | Camera intrinsics                         |
| `scripts/web_camera_stream.py`               | Browser MJPEG camera stream               |
| `scripts/csi_camera_node.py`                 | CSI camera node (OpenCV+GStreamer)        |
| `scripts/odom_ekf.py`                        | Odometry EKF filter                       |
| `scripts/calibrate_linear.py`                | Linear calibration tool                   |
| `scripts/multipoint_nav.py`                  | Multi-waypoint patrol navigation          |
| `src/jetracer.cpp`                           | Core driver (serial, motors, odom, IMU)   |
| `config/nav2_view.rviz`                      | rviz2 config for navigation visualization |
| `maps/savemap.sh`                            | Save map (slam_toolbox/hector/gmapping)   |
| `maps/carto_savemap.sh`                      | Save map (cartographer → pbstream → pgm)  |
