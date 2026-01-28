# JetRacer ROS2 Debugging Guide

## Quick Diagnostic Commands

Run these commands **on the JetRacer** to diagnose issues.

---

## 1. Hardware Checks

### Check Serial Port Exists
```bash
# List all serial ports
ls -la /dev/ttyACM* /dev/ttyUSB* 2>/dev/null

# Expected output: /dev/ttyACM0 should exist
# If not found, the microcontroller is not connected or not powered
```

### Check Serial Port Permissions
```bash
# Check if you have permission to access the port
ls -la /dev/ttyACM0

# If permission denied, add user to dialout group:
sudo usermod -aG dialout $USER
# Then LOGOUT and LOGIN again!
```

### Check USB Connection
```bash
# See all USB devices
lsusb

# Check kernel messages for USB events
dmesg | tail -20

# Watch for USB connect/disconnect
dmesg -w
```

### Check I2C Devices (if using PCA9685 directly)
```bash
# Install i2c-tools if needed
sudo apt install i2c-tools

# Scan I2C bus 1 (common on Jetson)
sudo i2cdetect -y 1

# Expected addresses for Waveshare JetRacer:
# 0x40 - PCA9685 (steering servo)
# 0x60 - PCA9685 (motor driver)
```

---

## 2. ROS2 Node Checks

### Source ROS2 Environment
```bash
# Source ROS2 (adjust path if needed)
source /opt/ros/humble/setup.bash

# Source your workspace
cd ~/jetracerWork
source install/setup.bash
```

### Build the Package
```bash
cd ~/jetracerWork
colcon build --packages-select jetracer_ros2
source install/setup.bash
```

### Run JetRacer Node
```bash
# Run with default parameters
ros2 run jetracer_ros2 jetracer

# Run with debug logging enabled
ros2 run jetracer_ros2 jetracer --ros-args --log-level debug
```

### Run with Launch File
```bash
ros2 launch jetracer_ros2 jetracer.launch.py
```

---

## 3. Topic Checks

### List All Topics
```bash
ros2 topic list
```

### Expected Topics:
- `/cmd_vel` - Input velocity commands
- `/odom` - Odometry output
- `/imu` - IMU data
- `/motor/lvel` - Left motor actual velocity
- `/motor/rvel` - Right motor actual velocity
- `/motor/lset` - Left motor setpoint
- `/motor/rset` - Right motor setpoint

### Monitor cmd_vel
```bash
# See if cmd_vel is being published
ros2 topic echo /cmd_vel
```

### Monitor Odometry
```bash
# Check if odom is being published (indicates MCU is responding)
ros2 topic echo /odom
```

### Monitor Motor Values
```bash
# Check motor feedback
ros2 topic echo /motor/lvel
ros2 topic echo /motor/rvel
```

---

## 4. Send Test Commands

### Send Zero Velocity (Stop)
```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}" --once
```

### Send Forward Command
```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2}, angular: {z: 0.0}}" --once
```

### Send Turn Command
```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.5}}" --once
```

### Send Forward + Turn
```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.15}, angular: {z: 0.3}}" --once
```

### Continuous Command (hold down)
```bash
# Publish at 10Hz continuously
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}, angular: {z: 0.0}}" -r 10
```

---

## 5. Serial Port Debugging

### Monitor Raw Serial Data
```bash
# Install screen if needed
sudo apt install screen

# Monitor serial port (Ctrl+A, K to exit)
sudo screen /dev/ttyACM0 115200
```

### Use minicom
```bash
sudo apt install minicom
sudo minicom -D /dev/ttyACM0 -b 115200
```

### Python Serial Test Script
```bash
# Create and run a test script
python3 << 'EOF'
import serial
import time

try:
    ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
    print("Serial port opened successfully!")
    
    # Send a test velocity command
    # Format: [0xAA, 0x55, 0x0B, 0x11, x_hi, x_lo, y_hi, y_lo, yaw_hi, yaw_lo, checksum]
    cmd = bytes([0xAA, 0x55, 0x0B, 0x11, 0x00, 0x64, 0x00, 0x64, 0x00, 0x00])
    checksum = sum(cmd) & 0xFF
    cmd = cmd + bytes([checksum])
    
    print(f"Sending: {cmd.hex()}")
    ser.write(cmd)
    
    # Read response
    time.sleep(0.1)
    if ser.in_waiting > 0:
        response = ser.read(ser.in_waiting)
        print(f"Received: {response.hex()}")
    else:
        print("No response received!")
    
    ser.close()
except Exception as e:
    print(f"Error: {e}")
EOF
```

---

## 6. Parameter Adjustment

### Check Current Parameters
```bash
ros2 param list /jetracer
ros2 param get /jetracer port_name
ros2 param get /jetracer coefficient_d
```

### Set Parameters at Runtime
```bash
# Change servo bias
ros2 param set /jetracer servo_bias 10

# Change PID values
ros2 param set /jetracer kp 400
ros2 param set /jetracer ki 100
```

### Run with Custom Parameters
```bash
ros2 run jetracer_ros2 jetracer --ros-args \
  -p port_name:=/dev/ttyACM0 \
  -p coefficient_d:=1520.0 \
  -p servo_bias:=5
```

---

## 7. Common Issues & Solutions

### Issue: "Failed to open serial port"
**Causes:**
- Port doesn't exist → Check USB connection
- Permission denied → Add user to dialout group
- Port in use → Kill other processes using it

**Solution:**
```bash
# Find what's using the port
sudo fuser /dev/ttyACM0

# Kill the process
sudo fuser -k /dev/ttyACM0
```

### Issue: Serial opens but no response from MCU
**Causes:**
- MCU not powered
- Wrong baud rate
- MCU firmware issue
- Wrong protocol/encoding

**Check:**
- Is the MCU LED blinking?
- Try 9600, 57600, 115200 baud rates
- Check if MCU responds to any input

### Issue: Commands sent but motors don't move
**Causes:**
- Steering coefficients wrong
- Motor driver not connected
- ESC not armed
- Battery too low

**Solutions:**
```bash
# Try different coefficient_d values (servo neutral)
ros2 param set /jetracer coefficient_d 1500
ros2 param set /jetracer coefficient_d 1520
ros2 param set /jetracer coefficient_d 1480

# Check battery voltage
ros2 topic echo /battery_voltage  # if available
```

### Issue: Odom/IMU data not received
**Causes:**
- MCU not sending data
- Checksum errors
- Frame sync lost

**Check logs for:**
```
[RX] Checksum ERROR
[RX] Invalid frame size
```

---

## 8. Steering Calibration

The steering uses a polynomial: `PWM = a*θ³ + b*θ² + c*θ + d`

### Sweep Test to Find Correct PWM Range
```bash
# Run this Python script to test different PWM values
python3 << 'EOF'
import serial
import time

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
print("Testing steering range...")

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

## 9. Full System Test

```bash
# Terminal 1: Run jetracer node with debug
ros2 run jetracer_ros2 jetracer --ros-args --log-level debug

# Terminal 2: Monitor status
watch -n 0.5 'ros2 topic echo /odom --once 2>/dev/null | head -5'

# Terminal 3: Send commands
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}, angular: {z: 0.0}}" -r 10
```

---

## 10. Log Output Reference

### Successful Startup:
```
[INIT] JetRacer ROS2 Driver Starting...
[SERIAL] ✓ Port opened successfully!
[CONFIG] ✓ Initial configuration sent
[RX] ✓ Serial receive thread started
[RX #1] ✓ Valid frame received
[STATUS] Serial:OK | Cmds:0 | Sent:50 | Recv:50 | ChkErr:0 | WrErr:0
```

### Problem Indicators:
```
[SERIAL] ✗ FAILED to open serial port!     → Check hardware connection
[RX] Checksum ERROR                         → Communication issue
[WATCHDOG] No cmd_vel for 1.0s             → No commands being sent
[STATUS] Serial:OK | Recv:0                 → MCU not responding
```

---

## Quick Reference: Protocol Format

| Packet | Header | Size | Type | Data | Checksum |
|--------|--------|------|------|------|----------|
| Velocity | `AA 55` | `0B` | `11` | x(2), y(2), yaw(2) | sum |
| Params | `AA 55` | `0F` | `12` | kp(2), ki(2), kd(2), lc(2), sb(2) | sum |
| Coefficients | `AA 55` | `15` | `13` | a(4), b(4), c(4), d(4) | sum |

Values are encoded as: `(value * 1000)` as int16, big-endian.
