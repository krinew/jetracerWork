#!/usr/bin/env python3
"""
Direct Motor Test for JetRacer ROS AI Kit
==========================================
This script bypasses ROS entirely and sends velocity commands
directly to the RP2040 microcontroller via serial.

Run this BEFORE running any ROS nodes to verify hardware works.
Usage: python3 motor_test.py
"""

import serial
import time
import struct
import sys

# Serial settings
PORT = "/dev/ttyACM0"
BAUD = 115200

# Protocol constants
HEAD1 = 0xAA
HEAD2 = 0x55
CMD_VELOCITY = 0x11
CMD_PARAMS = 0x12
CMD_COEFFICIENT = 0x13

def checksum(data):
    """Calculate checksum (sum of all bytes, keeping lower 8 bits)"""
    return sum(data) & 0xFF

def make_velocity_cmd(x_mms, y_mms, yaw_mms):
    """
    Create velocity command packet
    x_mms, y_mms, yaw_mms: velocity in mm/s (or mrad/s for yaw)
    """
    # Convert to int16 and pack as big-endian
    x = int(x_mms) & 0xFFFF
    y = int(y_mms) & 0xFFFF  
    yaw = int(yaw_mms) & 0xFFFF
    
    packet = bytearray([
        HEAD1, HEAD2,
        0x0B,  # Size = 11 bytes
        CMD_VELOCITY,
        (x >> 8) & 0xFF, x & 0xFF,      # X velocity (big endian)
        (y >> 8) & 0xFF, y & 0xFF,      # Y velocity (big endian)  
        (yaw >> 8) & 0xFF, yaw & 0xFF,  # Yaw velocity (big endian)
    ])
    packet.append(checksum(packet))
    return bytes(packet)

def make_params_cmd(kp=350, ki=120, kd=0, linear_corr=1000, servo_bias=0):
    """
    Create PID parameters command packet
    linear_corr: linear_correction * 1000 (default 1.0 -> 1000)
    """
    packet = bytearray([
        HEAD1, HEAD2,
        0x0F,  # Size = 15 bytes
        CMD_PARAMS,
        (kp >> 8) & 0xFF, kp & 0xFF,
        (ki >> 8) & 0xFF, ki & 0xFF,
        (kd >> 8) & 0xFF, kd & 0xFF,
        (linear_corr >> 8) & 0xFF, linear_corr & 0xFF,
        (servo_bias >> 8) & 0xFF, servo_bias & 0xFF,
    ])
    packet.append(checksum(packet))
    return bytes(packet)

def parse_frame(data):
    """Parse a received frame and extract motor values"""
    if len(data) < 45 or data[0] != HEAD1 or data[1] != HEAD2:
        return None
    
    frame_size = data[2]
    if frame_size != 0x2D:  # Expected size = 45
        return None
    
    # Extract motor data (positions 34-41 in 0-indexed)
    lvel = struct.unpack('>h', bytes(data[34:36]))[0]
    rvel = struct.unpack('>h', bytes(data[36:38]))[0]
    lset = struct.unpack('>h', bytes(data[38:40]))[0]
    rset = struct.unpack('>h', bytes(data[40:42]))[0]
    
    # Extract IMU accel Z (to verify data is valid)
    acc_z = struct.unpack('>h', bytes(data[14:16]))[0] / 32768.0 * 2 * 9.8
    
    return {
        'lvel': lvel, 'rvel': rvel,
        'lset': lset, 'rset': rset,
        'acc_z': acc_z
    }

def read_one_frame(ser, timeout=1.0):
    """Read one complete frame from serial"""
    start = time.time()
    data = bytearray()
    
    while time.time() - start < timeout:
        if ser.in_waiting:
            b = ser.read(1)
            if len(data) == 0:
                if b[0] == HEAD1:
                    data.append(b[0])
            elif len(data) == 1:
                if b[0] == HEAD2:
                    data.append(b[0])
                else:
                    data = bytearray()
            elif len(data) == 2:
                data.append(b[0])  # Size byte
                frame_size = b[0]
                if frame_size > 50 or frame_size < 5:
                    data = bytearray()
            elif len(data) < data[2]:
                data.append(b[0])
                if len(data) >= data[2]:
                    return bytes(data)
        else:
            time.sleep(0.001)
    return None

def main():
    print("=" * 60)
    print("JetRacer Direct Motor Test")
    print("=" * 60)
    print()
    
    # Open serial port
    try:
        ser = serial.Serial(PORT, BAUD, timeout=1)
        print(f"✓ Opened {PORT} at {BAUD} baud")
    except Exception as e:
        print(f"✗ Failed to open {PORT}: {e}")
        print("  Make sure no other program is using the port!")
        print("  Stop any ROS nodes first: pkill -f jetracer")
        sys.exit(1)
    
    time.sleep(0.5)  # Wait for connection
    ser.reset_input_buffer()
    
    # First, verify we can receive data from PCB
    print("\n--- Step 1: Verify PCB Communication ---")
    frame = read_one_frame(ser)
    if frame:
        parsed = parse_frame(frame)
        if parsed:
            print(f"✓ Receiving data from PCB!")
            print(f"  IMU Accel Z: {parsed['acc_z']:.2f} m/s² (should be ~10 for gravity)")
            print(f"  Motor velocities: L={parsed['lvel']} R={parsed['rvel']}")
            print(f"  Motor setpoints:  L={parsed['lset']} R={parsed['rset']}")
        else:
            print("✗ Received data but failed to parse frame")
    else:
        print("✗ No data received from PCB!")
        print("  Check if LED on expansion board is blinking")
        print("  Try pressing RESET button on the board")
        ser.close()
        sys.exit(1)
    
    # Send PID parameters (optional but good practice)
    print("\n--- Step 2: Send PID Parameters ---")
    params_cmd = make_params_cmd(kp=350, ki=120, kd=0)
    ser.write(params_cmd)
    print(f"  Sent: {params_cmd.hex(' ').upper()}")
    time.sleep(0.1)
    
    # Now test motor control
    print("\n--- Step 3: Motor Control Test ---")
    print("  Sending forward velocity command for 2 seconds...")
    print("  >>> THE MOTORS SHOULD SPIN NOW! <<<")
    print()
    
    # Send velocity commands continuously for 2 seconds
    velocity = 200  # 0.2 m/s = 200 mm/s
    cmd = make_velocity_cmd(velocity, velocity, 0)
    print(f"  Command: {cmd.hex(' ').upper()}")
    print(f"  (x={velocity} mm/s, y={velocity} mm/s, yaw=0)")
    print()
    
    start_time = time.time()
    last_log = 0
    motor_responded = False
    
    while time.time() - start_time < 2.0:
        # Send velocity command at ~50Hz
        ser.write(cmd)
        
        # Read response
        frame = read_one_frame(ser, timeout=0.05)
        if frame:
            parsed = parse_frame(frame)
            if parsed:
                # Log every 0.5 seconds
                if time.time() - last_log > 0.5:
                    print(f"  t={time.time()-start_time:.1f}s: L_vel={parsed['lvel']:4d} R_vel={parsed['rvel']:4d} | " +
                          f"L_set={parsed['lset']:4d} R_set={parsed['rset']:4d}")
                    last_log = time.time()
                
                if parsed['lset'] != 0 or parsed['rset'] != 0:
                    motor_responded = True
                    if parsed['lvel'] != 0 or parsed['rvel'] != 0:
                        print(f"\n  ✓ Motors are responding! Velocity detected!")
        
        time.sleep(0.02)  # 50Hz
    
    # Stop motors
    print("\n  Sending stop command...")
    stop_cmd = make_velocity_cmd(0, 0, 0)
    for _ in range(10):
        ser.write(stop_cmd)
        time.sleep(0.02)
    
    # Final analysis
    print("\n" + "=" * 60)
    print("RESULTS")
    print("=" * 60)
    
    if motor_responded:
        print("✓ PCB is receiving and processing velocity commands!")
        print()
        print("If the wheels did NOT physically spin, check:")
        print("  1. Battery voltage - is it charged and connected?")
        print("  2. Battery protection - did you plug in charger briefly?")
        print("  3. Motor wires - are they securely connected?")
        print("  4. Motor driver - may be damaged")
    else:
        print("✗ Motor setpoints stayed at zero!")
        print()
        print("This suggests the PCB is NOT processing velocity commands.")
        print("Possible causes:")
        print("  1. Firmware needs reflashing (see Waveshare wiki)")
        print("  2. Wrong serial protocol")
        print("  3. RP2040 crashed - try pressing RESET button")
    
    ser.close()
    print()
    print("Test complete.")

if __name__ == "__main__":
    main()
