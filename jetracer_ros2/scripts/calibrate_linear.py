#!/usr/bin/env python3

""" calibrate_linear.py - ROS 2 Version
    Move the robot 1.0 meter to check on the PID parameters of the base controller.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
import tf2_ros
from tf2_ros import Buffer, TransformListener
import math
import time

class CalibrateLinear(Node):
    def __init__(self):
        super().__init__('calibrate_linear')
        
        # Declare parameters
        self.declare_parameter('test_distance', 1.0)
        self.declare_parameter('speed', 0.3)
        self.declare_parameter('tolerance', 0.03)
        self.declare_parameter('odom_linear_scale_correction', 1.0)
        
        # Get parameters
        self.test_distance = self.get_parameter('test_distance').value
        self.speed = self.get_parameter('speed').value
        self.tolerance = self.get_parameter('tolerance').value
        self.odom_linear_scale_correction = self.get_parameter('odom_linear_scale_correction').value
        
        # Publisher
        self.cmd_vel = self.create_publisher(Twist, '/cmd_vel', 5)
        
        # TF Buffer
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.get_logger().info("Calibrate Linear Node Started. Moving robot in 1 second...")
        
        # Ensure connection/setup time
        time.sleep(1) 
        
        # Get initial transform
        self.position = Point()
        
        try:
            # Get the starting position from the tf transform between the odom and base_footprint frames
            # In ROS 2, getting transform is usually async, but for a script we can try waiting or look up most recent.
            # We will try to lookup transform.
            pass
        except Exception as e:
            self.get_logger().error(f"Failed to get initial transform: {e}")

        # Start the calibration loop
        self.run_calibration()
        
    def get_odom_position(self):
        try:
            # Look up transform from odom to base_footprint
            trans = self.tf_buffer.lookup_transform('odom', 'base_footprint', rclpy.time.Time())
            return trans.transform.translation
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            return None

    def run_calibration(self):
        rate = self.create_rate(20)
        twist = Twist()
        
        # Wait for tf to become available
        while rclpy.ok():
            start_pos = self.get_odom_position()
            if start_pos:
                break
            self.get_logger().info("Waiting for tf...")
            rclpy.spin_once(self)
        
        self.get_logger().info(f"Start Position: x={start_pos.x}, y={start_pos.y}")
        
        twist.linear.x = self.speed
        
        distance = 0.0
        
        while rclpy.ok() and distance < self.test_distance:
            self.cmd_vel.publish(twist)
            rclpy.spin_once(self)
            
            current_pos = self.get_odom_position()
            if current_pos:
                # Euclidean distance
                distance = math.sqrt(
                    math.pow(current_pos.x - start_pos.x, 2) + 
                    math.pow(current_pos.y - start_pos.y, 2)
                )
                distance *= self.odom_linear_scale_correction # Apply correction
                
                # self.get_logger().info(f"Dist: {distance:.3f} / {self.test_distance}")
            else:
                 self.get_logger().warn("TF failed")

            # rudimentary sleep since 'rate.sleep()' in ROS 2 works differently inside a single threaded spin
            time.sleep(0.05) 
            
        # Stop
        twist.linear.x = 0.0
        self.cmd_vel.publish(twist)
        self.get_logger().info(f"Finished. Traveled: {distance:.4f}m")


def main(args=None):
    rclpy.init(args=args)
    node = CalibrateLinear()
    # Logic is inside init/methods for this simple script
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
