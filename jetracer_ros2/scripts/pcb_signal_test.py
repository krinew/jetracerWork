#!/usr/bin/env python3
"""
PCB Signal Diagnostic Tool for JetRacer ROS AI Kit
This script tests different serial protocols to find what the PCB responds to.
"""

import serial
import time
import struct
import sys

# Configuration
PORT = "/dev/ttyACM0"  # Change if needed
BAUD_RATES = [115200, 9600, 57600, 38400, 19200]

def checksum(data):
    """Calculate checksum (sum of all bytes)"""
    return sum(data) & 0xFF

def send_and_receive(ser, data, description):
    """Send data and print response"""
    print(f"\n{'='*60}")
    print(f"TEST: {description}")
    print(f"TX ({len(data)} bytes): {' '.join(f'{b:02X}' for b in data)}")
    
    # Clear input buffer
    ser.reset_input_buffer()
    
    # Send data
    ser.write(data)
    ser.flush()
    
    # Wait for response
    time.sleep(0.1)
    
    # Read response
    response = ser.read(ser.in_waiting or 100)
    if response:
        print(f"RX ({len(response)} bytes): {' '.join(f'{b:02X}' for b in response)}")
        return True
    else:
        print("RX: No response")
        return False

def test_velocity_commands(ser):
    """Test different velocity command formats"""
    print("\n" + "="*60)
    print("TESTING VELOCITY COMMANDS")
    print("="*60)
    
    # Standard format: AA 55 0B 11 [x_hi x_lo y_hi y_lo yaw_hi yaw_lo] checksum
    # x = 200 (0.2 m/s * 1000)
    x_val = 200
    y_val = 200
    yaw_val = 0
    
    # Test 1: Standard protocol (Big Endian)
    data = bytearray([
        0xAA, 0x55,  # Header
        0x0B,        # Size (11 bytes)
        0x11,        # Type (velocity)
        (x_val >> 8) & 0xFF, x_val & 0xFF,      # X velocity (big endian)
        (y_val >> 8) & 0xFF, y_val & 0xFF,      # Y velocity (big endian)
        (yaw_val >> 8) & 0xFF, yaw_val & 0xFF,  # Yaw velocity (big endian)
    ])
    data.append(checksum(data))
    send_and_receive(ser, data, "Standard Protocol (Big Endian) - x=0.2 m/s")
    
    # Test 2: Little Endian
    data = bytearray([
        0xAA, 0x55,
        0x0B,
        0x11,
        x_val & 0xFF, (x_val >> 8) & 0xFF,      # X velocity (little endian)
        y_val & 0xFF, (y_val >> 8) & 0xFF,      # Y velocity (little endian)
        yaw_val & 0xFF, (yaw_val >> 8) & 0xFF,  # Yaw velocity (little endian)
    ])
    data.append(checksum(data))
    send_and_receive(ser, data, "Little Endian - x=0.2 m/s")
    
    # Test 3: Different header (55 AA)
    data = bytearray([
        0x55, 0xAA,  # Reversed header
        0x0B,
        0x11,
        (x_val >> 8) & 0xFF, x_val & 0xFF,
        (y_val >> 8) & 0xFF, y_val & 0xFF,
        (yaw_val >> 8) & 0xFF, yaw_val & 0xFF,
    ])
    data.append(checksum(data))
    send_and_receive(ser, data, "Reversed Header (55 AA)")
    
    # Test 4: Higher velocity value
    x_val = 1000  # 1.0 m/s
    data = bytearray([
        0xAA, 0x55,
        0x0B,
        0x11,
        (x_val >> 8) & 0xFF, x_val & 0xFF,
        (x_val >> 8) & 0xFF, x_val & 0xFF,
        0x00, 0x00,
    ])
    data.append(checksum(data))
    send_and_receive(ser, data, "Higher velocity x=1.0 m/s")
    
    # Test 5: Raw PWM style (0-255 range)
    pwm_val = 100
    data = bytearray([
        0xAA, 0x55,
        0x0B,
        0x11,
        0x00, pwm_val,  # Maybe it expects raw PWM?
        0x00, pwm_val,
        0x00, 0x00,
    ])
    data.append(checksum(data))
    send_and_receive(ser, data, "Raw PWM style (100)")

def test_different_command_types(ser):
    """Test different command type bytes"""
    print("\n" + "="*60)
    print("TESTING DIFFERENT COMMAND TYPES")
    print("="*60)
    
    x_val = 500
    
    for cmd_type in [0x01, 0x10, 0x11, 0x12, 0x13, 0x20, 0x21, 0x30, 0x31, 0xFF]:
        data = bytearray([
            0xAA, 0x55,
            0x0B,
            cmd_type,
            (x_val >> 8) & 0xFF, x_val & 0xFF,
            (x_val >> 8) & 0xFF, x_val & 0xFF,
            0x00, 0x00,
        ])
        data.append(checksum(data))
        send_and_receive(ser, data, f"Command Type 0x{cmd_type:02X}")
        time.sleep(0.05)

def test_params_command(ser):
    """Test parameter setting commands"""
    print("\n" + "="*60)
    print("TESTING PARAMETER COMMANDS")
    print("="*60)
    
    # SetParams format from original code
    kp, ki, kd = 350, 120, 0
    linear_correction = 1000  # 1.0 * 1000
    servo_bias = 0
    
    data = bytearray([
        0xAA, 0x55,
        0x0F,  # Size (15 bytes)
        0x12,  # Type (params)
        (kp >> 8) & 0xFF, kp & 0xFF,
        (ki >> 8) & 0xFF, ki & 0xFF,
        (kd >> 8) & 0xFF, kd & 0xFF,
        (linear_correction >> 8) & 0xFF, linear_correction & 0xFF,
        (servo_bias >> 8) & 0xFF, servo_bias & 0xFF,
    ])
    data.append(checksum(data))
    send_and_receive(ser, data, "SetParams (Kp=350, Ki=120, Kd=0)")

def test_coefficient_command(ser):
    """Test coefficient setting commands"""
    print("\n" + "="*60)
    print("TESTING COEFFICIENT COMMANDS")
    print("="*60)
    
    # SetCoefficient format from original code
    a = -0.016073
    b = 0.176183
    c = -23.428084
    d = 1500.0
    
    data = bytearray([0xAA, 0x55, 0x15, 0x13])  # Header, size, type
    data.extend(struct.pack('<f', a))  # Little endian float
    data.extend(struct.pack('<f', b))
    data.extend(struct.pack('<f', c))
    data.extend(struct.pack('<f', d))
    data.append(checksum(data))
    send_and_receive(ser, data, "SetCoefficient (a,b,c,d)")

def test_simple_patterns(ser):
    """Test simple byte patterns to see what triggers a response"""
    print("\n" + "="*60)
    print("TESTING SIMPLE PATTERNS")
    print("="*60)
    
    patterns = [
        ([0xAA], "Single 0xAA"),
        ([0xAA, 0x55], "Header only"),
        ([0xAA, 0x55, 0x01], "Header + 1 byte"),
        ([0x00] * 10, "10 zeros"),
        ([0xFF] * 10, "10 FFs"),
        ([0x01, 0x02, 0x03, 0x04, 0x05], "Sequential 1-5"),
    ]
    
    for pattern, desc in patterns:
        send_and_receive(ser, bytearray(pattern), desc)
        time.sleep(0.05)

def continuous_read(ser, duration=5):
    """Continuously read from serial for a duration"""
    print("\n" + "="*60)
    print(f"CONTINUOUS READ FOR {duration} SECONDS")
    print("(This shows what the PCB sends without any commands)")
    print("="*60)
    
    start = time.time()
    all_data = bytearray()
    
    while time.time() - start < duration:
        if ser.in_waiting:
            data = ser.read(ser.in_waiting)
            all_data.extend(data)
            print(f"RX: {' '.join(f'{b:02X}' for b in data)}")
        time.sleep(0.01)
    
    if all_data:
        print(f"\nTotal received: {len(all_data)} bytes")
        # Try to find frame headers
        for i in range(len(all_data) - 1):
            if all_data[i] == 0xAA and all_data[i+1] == 0x55:
                print(f"  Found header AA 55 at position {i}")
    else:
        print("No data received from PCB")

def main():
    print("="*60)
    print("JetRacer ROS AI Kit - PCB Signal Diagnostic Tool")
    print("="*60)
    
    # Try to find the right baud rate first
    print(f"\nTrying to connect to {PORT}...")
    
    for baud in BAUD_RATES:
        print(f"\n>>> Testing baud rate: {baud}")
        try:
            ser = serial.Serial(
                port=PORT,
                baudrate=baud,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.5
            )
            print(f"    Connected at {baud} baud")
            
            # First, see if PCB is sending anything
            time.sleep(0.5)
            if ser.in_waiting:
                data = ser.read(ser.in_waiting)
                print(f"    PCB is sending data at this baud rate!")
                print(f"    Sample: {' '.join(f'{b:02X}' for b in data[:50])}")
                
                # Check if it looks valid (has AA 55 headers)
                if 0xAA in data and 0x55 in data:
                    print(f"    ✓ Found valid frame headers at {baud} baud!")
            
            ser.close()
            
        except serial.SerialException as e:
            print(f"    Failed: {e}")
    
    # Now do full tests at default baud rate
    print("\n" + "="*60)
    print("RUNNING FULL DIAGNOSTICS AT 115200 BAUD")
    print("="*60)
    
    try:
        ser = serial.Serial(
            port=PORT,
            baudrate=115200,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.5
        )
    except serial.SerialException as e:
        print(f"Failed to open {PORT}: {e}")
        print("\nTry: ls -la /dev/ttyACM* /dev/ttyUSB*")
        sys.exit(1)
    
    try:
        # Run all tests
        continuous_read(ser, duration=3)
        test_simple_patterns(ser)
        test_coefficient_command(ser)
        test_params_command(ser)
        test_different_command_types(ser)
        test_velocity_commands(ser)
        
        # Send stop command at end
        print("\n" + "="*60)
        print("SENDING STOP COMMAND")
        print("="*60)
        data = bytearray([0xAA, 0x55, 0x0B, 0x11, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])
        data.append(checksum(data))
        send_and_receive(ser, data, "Stop (velocity = 0)")
        
    finally:
        ser.close()
    
    print("\n" + "="*60)
    print("DIAGNOSTICS COMPLETE")
    print("="*60)
    print("""
Next steps:
1. If PCB sends data but motors don't move:
   - Check motor wiring
   - Check battery voltage
   - Check if motor driver IC (TB6612FNG) is getting power

2. If no data from PCB at any baud rate:
   - Check USB connection
   - Check if STM32 is powered (LED?)
   - Try different USB port/cable

3. If data looks garbled:
   - Wrong baud rate (try the one that shows AA 55 patterns)
   - Electrical noise on serial line
""")

if __name__ == "__main__":
    main()
