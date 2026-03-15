#!/usr/bin/env python3
"""
Remote Data Collector Node (Runs on Laptop/Client Machine)

This script passively listens to the Jetson's camera stream over the ROS 2 network
and saves the images to the local hard drive for training ML models.

Requirements:
- Must be run on a machine in the same local network as the JetRacer.
- The Jetson MUST be running its csi_camera_node for this to work.
  This script will NOT try to open hardware cameras itself.

Usage:
    python client_data_collector.py
"""

import os
import time
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class PassiveDataCollector(Node):
    def __init__(self):
        super().__init__('remote_data_collector_node')
        
        self.bridge = CvBridge()
        
        # Folder setup for dataset
        self.dataset_dir = os.path.join(os.getcwd(), 'collected_dataset')
        os.makedirs(self.dataset_dir, exist_ok=True)
   
        self.subscription = self.create_subscription(  #In this we wait until this node is pinged as soon as it is pinged we receive the image and then we save it to the local directory of the laptop which is running this script
            Image, 
            '/csi_cam_0/image_raw', 
            self.image_callback, 
            10
        )
        
        self.get_logger().info("==========================================")
        self.get_logger().info(f" Passive Data Collector Started")
        self.get_logger().info(f" Saving images to: {self.dataset_dir}")
        self.get_logger().info(" Waiting for Jetson camera stream...")
        self.get_logger().info("==========================================")
        
        self.frames_saved = 0
        self.frames_received = 0
        
        # Configurable: Save 1 out of every X frames received
        # If the Jetson camera publishes at 30 FPS, saving every 6th frame gives ~5 FPS locally.
        # This completely ignores Wi-Fi delay, focusing purely on frame count.
        self.save_every_n_frames = 6

    def image_callback(self, msg):
        self.frames_received += 1
        
        # Filter by strict frame count rather than local time
        if self.frames_received % self.save_every_n_frames != 0:
            return
            
        try:
            # Convert the ROS byte array straight into an OpenCV Image
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)  # Convert BGR to RGB
            
            # Generate a unique timestamped filename (using the actual hardware timestamp 
            # from the jetson camera if available, falling back to local time)
            msg_time = msg.header.stamp.sec + (msg.header.stamp.nanosec / 1e9)
            if msg_time == 0:
                msg_time = time.time()
                
            filename = f"frame_{int(msg_time * 1000)}.jpg"
            filepath = os.path.join(self.dataset_dir, filename)
            
            # Save it!
            cv2.imwrite(filepath, cv_image)
            
            self.frames_saved += 1
            
            # Only print every 10th frame so we don't spam the console too badly
            if self.frames_saved % 10 == 0:
                self.get_logger().info(f"Successfully saved {self.frames_saved} total frames.")
                
        except Exception as e:
            self.get_logger().error(f"Failed to process and save image: {e}")

def main(args=None):
    rclpy.init(args=args)
    
    node = PassiveDataCollector()
    
    try:
        # spin() blocks and keeps the node alive, waiting for incoming Wi-Fi messages
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("\nStopping Data Collection. Shutting down node.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
