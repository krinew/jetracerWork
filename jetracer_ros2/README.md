# JetRacer ROS 2 Package

Complete ROS 2 (Jazzy) package for the NVIDIA JetRacer autonomous robot platform. Migrated from ROS 1 with full functionality parity.

## Features

### Core Functionality

- **Driver Node**: C++ node for hardware interface and motor control
- **Odometry Fusion**: EKF-based sensor fusion for accurate odometry
- **TF Broadcasting**: Complete robot state transforms

### SLAM (Simultaneous Localization and Mapping)

- **SLAM Toolbox** (recommended): Modern ROS 2 SLAM solution, replaces gmapping/karto
- **Cartographer**: Google's Cartographer SLAM
- **Hector SLAM**: Scan-matching based SLAM (no odometry required)

### Navigation

- **Nav2 Stack**: Full autonomous navigation with DWB controller
- **AMCL Localization**: Adaptive Monte Carlo Localization
- **Combined SLAM+Nav**: Simultaneous mapping and navigation

### Sensor Processing

- **Laser Filtering**: Multi-stage laser scan filtering
- **Calibration Tools**: Linear motion calibration

### Vision (Optional)

- **CSI Camera Support**: Native Jetson camera interface
- **V4L2 Camera**: USB camera support

## Installation

### Prerequisites

```bash
# Install ROS 2 Jazzy (if not already installed)
# Follow: https://docs.ros.org/en/jazzy/Installation.html

# Install dependencies
sudo apt update
sudo apt install -y \
  ros-jazzy-slam-toolbox \
  ros-jazzy-cartographer-ros \
  ros-jazzy-nav2-bringup \
  ros-jazzy-nav2-amcl \
  ros-jazzy-robot-localization \
  ros-jazzy-laser-filters \
  ros-jazzy-rplidar-ros \
  ros-jazzy-v4l2-camera
```

### Build

```bash
# Create workspace (if needed)
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Clone or copy this package
# cp -r /path/to/jetracer_ros2 .

# Build
cd ~/ros2_ws
colcon build --packages-select jetracer
source install/setup.bash
```

## Usage

### 1. Basic Robot Operation

```bash
# Launch the base driver (jetracer hardware + EKF + TF)
ros2 launch jetracer jetracer.launch.py

# In another terminal, check topics
ros2 topic list
ros2 topic echo /odom
```

### 2. SLAM Mapping

#### Using SLAM Toolbox (Recommended)

```bash
# Launch robot + lidar + SLAM
ros2 launch jetracer slam.launch.py slam_method:=slam_toolbox

# Save map when done
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: {data: my_map}}"
# Map saved to ~/.ros/my_map.yaml and my_map.pgm
```

#### Using Cartographer

```bash
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
