# JetRacer ROS 2 Package

ROS 2 (Humble) package for NVIDIA JetRacer autonomous robot. Supports SLAM, navigation, odometry fusion, and multi-sensor integration.

## Setup

### 1. Install ROS 2 Humble

```bash
curl -sSL https://raw.githubusercontent.com/ros/ros-setup-scripts/master/ros-humble-setup.sh | bash
source /opt/ros/humble/setup.bash
```

### 2. Install Dependencies

```bash
sudo apt update
sudo apt install -y \
  python3-colcon-common-extensions \
  ros-humble-slam-toolbox \
  ros-humble-cartographer-ros \
  ros-humble-nav2-bringup \
  ros-humble-nav2-amcl \
  ros-humble-robot-localization \
  ros-humble-laser-filters \
  ros-humble-rplidar-ros
```

### 3. Build Package

```bash
cd ~/jetracerWork
colcon build
source install/local_setup.bash
```

### 4. Verify Installation

```bash
ros2 pkg list | grep jetracer
ros2 launch jetracer jetracer.launch.py --show-args
```

## Hardware Setup

### Serial Connection

The package expects the microcontroller at `/dev/ttyACM0`. Edit [config/jetracer.yaml](config/jetracer.yaml) if using a different port:

```yaml
jetracer:
  ros__parameters:
    port_name: "/dev/ttyACM0"  # Change this if needed
```

### LiDAR

- RPLiDAR A1/A2 connected via USB
- Auto-detected on startup

## Basic Operations

### Start Robot

```bash
ros2 launch jetracer jetracer.launch.py
```

Runs: driver node, odometry EKF, transform broadcaster

**Topics:**
- `/cmd_vel` - Motor commands (Twist)
- `/odom` - Odometry
- `/odom_raw` - Raw motor odometry
- `/imu/data` - IMU data
- `tf` - Transforms

### SLAM Mapping

```bash
# SLAM Toolbox (recommended)
ros2 launch jetracer slam.launch.py slam_method:=slam_toolbox

# Cartographer
ros2 launch jetracer slam.launch.py slam_method:=cartographer

# Hector SLAM
ros2 launch jetracer slam.launch.py slam_method:=hector
```

Save map:
```bash
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: {data: my_map}}"
```

Maps saved to `~/.ros/`

### Navigation with Saved Map

```bash
# Localization + Navigation
ros2 launch jetracer nav.launch.py map:=~/.ros/my_map.yaml

# Send navigation goal via RViz or command:
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {pose: {position: {x: 1.0, y: 1.0}}}}"
```

### Combined SLAM + Navigation

```bash
ros2 launch jetracer slam_nav.launch.py
```

Simultaneously map and navigate in real-time.

### Localization Only (AMCL)

```bash
ros2 launch jetracer amcl.launch.py map:=~/.ros/my_map.yaml
```

### Linear Calibration

Calibrate wheel diameter and baseline:

```bash
ros2 launch jetracer calibrate_linear.launch.py

# Follow on-screen prompts to drive straight line
```

Updates calibration coefficients in [config/jetracer.yaml](config/jetracer.yaml)

### Laser Filtering

```bash
ros2 launch jetracer laser_filter.launch.py
```

Applies outlier removal and statistical filtering to lidar scans.

## Troubleshooting

### Serial Port Not Found

```bash
# Check connected devices
ls /dev/tty*

# Add user permissions
sudo usermod -a -G dialout $USER
# Log out and back in
```

### Package Not Found

```bash
# Ensure workspace is sourced
source install/local_setup.bash

# Verify build
ros2 pkg list | grep jetracer
```

### Node Crashes on Startup

Check `/home/pi0/.ros/log/` for error messages:
```bash
ls ~/.ros/log/*/
```

### LiDAR Not Detected

```bash
# Check USB devices
lsusb | grep -i "prolific\|silicon\|ch"

# Set correct port in [config/jetracer.yaml](config/jetracer.yaml)
```

## Configuration Files

- [config/jetracer.yaml](config/jetracer.yaml) - Hardware parameters
- [config/nav2_params.yaml](config/nav2_params.yaml) - Navigation tuning
- [config/slam_toolbox_params.yaml](config/slam_toolbox_params.yaml) - SLAM parameters
- [config/laser_filter_params.yaml](config/laser_filter_params.yaml) - Lidar filtering

## Launch Files

| File | Purpose |
|------|---------|
| `jetracer.launch.py` | Base driver + odometry |
| `lidar.launch.py` | LiDAR driver only |
| `slam.launch.py` | SLAM mapping (selectable method) |
| `slam_toolbox.launch.py` | SLAM Toolbox SLAM |
| `cartographer.launch.py` | Cartographer SLAM |
| `hector.launch.py` | Hector SLAM |
| `nav.launch.py` | Navigation with localization |
| `amcl.launch.py` | AMCL localization only |
| `slam_nav.launch.py` | Simultaneous SLAM + navigation |
| `laser_filter.launch.py` | Laser scan filtering |
| `calibrate_linear.launch.py` | Motion calibration |
| `csi_camera.launch.py` | CSI camera (optional) |

## ROS 2 Quick Commands

```bash
# List topics
ros2 topic list

# Echo topic
ros2 topic echo /odom

# View transforms
ros2 run tf2_tools view_frames
firefox frames.pdf

# Monitor nodes
ros2 node list
ros2 node info /jetracer

# Check parameters
ros2 param list
ros2 param get /jetracer port_name
ros2 launch jetracer slam.launch.py slam_method:=cartographer

# Save map (Cartographer)
ros2 run nav2_map_server map_saver_cli -f my_map
```

#### Using Hector SLAM

```bash
ros2 launch jetracer hector.launch.py

# Save map
ros2 run nav2_map_server map_saver_cli -f my_map
```

### 3. Localization (Using Pre-built Map)

```bash
# Launch AMCL localization with existing map
ros2 launch jetracer amcl.launch.py map:=/path/to/my_map.yaml

# Set initial pose in RViz2:
# - Click "2D Pose Estimate"
# - Click and drag on map to set robot position and orientation
```

### 4. Autonomous Navigation

```bash
# Launch full navigation stack with localization
ros2 launch jetracer nav.launch.py map:=/path/to/my_map.yaml

# In RViz2:
# 1. Set initial pose (2D Pose Estimate)
# 2. Set goal (2D Goal Pose)
# Robot will plan and execute path autonomously
```

### 5. Simultaneous SLAM and Navigation

```bash
# Map and navigate at the same time
ros2 launch jetracer slam_nav.launch.py slam_method:=slam_toolbox
```

### 6. Calibration

```bash
# Launch calibration tool
ros2 launch jetracer calibrate_linear.launch.py

# Start calibration test
ros2 param set /calibrate_linear start_test true

# Check results and adjust linear_correction parameter
ros2 param set /jetracer linear_correction 1.05  # Example adjustment
```

### 7. Laser Filtering

```bash
# Launch with laser filtering
ros2 launch jetracer lidar.launch.py
ros2 launch jetracer laser_filter.launch.py

# Filtered scan published on /scan
# Raw scan on /scan_raw
```

### 8. Camera

```bash
# Launch CSI camera (Jetson native)
ros2 launch jetracer csi_camera.launch.py

# View camera feed
ros2 run rqt_image_view rqt_image_view
```

## Launch Files Reference

| Launch File                  | Description                           |
| ---------------------------- | ------------------------------------- |
| `jetracer.launch.py`         | Base robot driver + odometry + TF     |
| `lidar.launch.py`            | RPLidar driver                        |
| `slam_toolbox.launch.py`     | SLAM Toolbox mapping                  |
| `cartographer.launch.py`     | Cartographer SLAM                     |
| `hector.launch.py`           | Hector SLAM                           |
| `slam.launch.py`             | Unified SLAM launcher (choose method) |
| `amcl.launch.py`             | AMCL localization                     |
| `nav.launch.py`              | Nav2 navigation stack                 |
| `slam_nav.launch.py`         | Combined SLAM + Navigation            |
| `laser_filter.launch.py`     | Laser scan filtering                  |
| `calibrate_linear.launch.py` | Linear calibration tool               |
| `csi_camera.launch.py`       | CSI camera driver                     |

## Configuration Files

### Core Config

- `config/jetracer.yaml` - Robot driver parameters (PID, ports, etc.)
- `config/nav2_params.yaml` - Nav2 stack configuration

### SLAM Config

- `config/slam_toolbox_params.yaml` - SLAM Toolbox parameters
- `config/cartographer/jetracer.lua` - Cartographer configuration

### Navigation Config

- `config/amcl_params.yaml` - AMCL localization parameters

### Sensor Config

- `config/laser_filter_params.yaml` - Laser filter chain configuration

## Parameters

### Jetracer Driver Node

```bash
# View all parameters
ros2 param list /jetracer

# Common parameters
ros2 param set /jetracer kp 350          # PID proportional gain
ros2 param set /jetracer ki 120          # PID integral gain
ros2 param set /jetracer kd 0            # PID derivative gain
ros2 param set /jetracer servo_bias 0    # Steering servo bias
ros2 param set /jetracer linear_correction 1.0  # Odometry scale factor
```

### Calibration Node

```bash
# Calibration parameters
ros2 param set /calibrate_linear test_distance 1.0  # meters
ros2 param set /calibrate_linear speed 0.3          # m/s
ros2 param set /calibrate_linear start_test true    # Start test
```

## Visualization

### RViz2

```bash
# Launch RViz2 with appropriate config
rviz2

# Add displays:
# - RobotModel
# - TF
# - LaserScan (topic: /scan)
# - Map (topic: /map)
# - Path (topic: /plan)
# - Odometry (topic: /odom)
```

## Troubleshooting

### Serial Port Permission

```bash
# Add user to dialout group for serial access
sudo usermod -a -G dialout $USER
# Log out and log back in for changes to take effect
```

### TF Issues

```bash
# Check TF tree
ros2 run tf2_tools view_frames
evince frames.pdf

# Monitor TF transforms
ros2 run tf2_ros tf2_echo map base_footprint
```

### SLAM Not Working

```bash
# Check laser scan
ros2 topic echo /scan --once

# Verify odometry
ros2 topic echo /odom --once

# Check TF chain
ros2 run tf2_ros tf2_echo odom base_footprint
```

### Navigation Issues

```bash
# Check costmaps
ros2 topic echo /local_costmap/costmap --once
ros2 topic echo /global_costmap/costmap --once

# Check if map is loaded
ros2 topic echo /map --once

# Verify AMCL is localizing
ros2 topic echo /amcl_pose
```

## Development

### Building from Source

```bash
cd ~/ros2_ws
colcon build --packages-select jetracer --symlink-install
source install/setup.bash
```

### Running Tests

```bash
colcon test --packages-select jetracer
colcon test-result --verbose
```

## Migration from ROS 1

This package has been fully migrated from ROS 1 to ROS 2 following official migration guidelines:

- Python scripts: `rospy` → `rclpy`
- C++ code: `roscpp` → `rclcpp`, `tf` → `tf2`
- Launch files: XML → Python
- Parameters: `dynamic_reconfigure` → native ROS 2 parameters
- SLAM: `gmapping`/`karto` → `slam_toolbox`
- Navigation: `move_base` → `nav2`

## Dependencies

### Core ROS 2 Packages

- `rclcpp`, `rclpy`
- `geometry_msgs`, `nav_msgs`, `sensor_msgs`, `std_msgs`
- `tf2`, `tf2_ros`, `tf2_geometry_msgs`

### SLAM & Navigation

- `slam_toolbox`
- `cartographer_ros`
- `nav2_bringup`
- `nav2_amcl`
- `robot_localization`

### Sensors

- `rplidar_ros`
- `laser_filters`
- `v4l2_camera` (optional)

## License

[Specify your license here]

## Authors & Contributors

[Original ROS 1 package credits]
ROS 2 Migration: 2026-01-23

## Support

For issues, questions, or contributions, please [specify contact method or repository].
