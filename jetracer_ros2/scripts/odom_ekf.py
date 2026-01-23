#!/usr/bin/env python3

""" odom_ekf.py - ROS 2 Version
    Republish the /odom_combined (PoseWithCovarianceStamped) 
    as nav_msgs/Odometry.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry

class OdomEKF(Node):
    def __init__(self):
        super().__init__('odom_ekf')
        
        # Declare parameters
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value

        # Publisher of type nav_msgs/Odometry
        self.ekf_pub = self.create_publisher(Odometry, 'odom', 10)
        
        # Subscribe to the /odom_combined topic
        self.subscription = self.create_subscription(
            PoseWithCovarianceStamped,
            'odom_combined',
            self.pub_ekf_odom,
            10)
        
        self.get_logger().info("Publishing combined odometry on /odom")
        
    def pub_ekf_odom(self, msg):
        odom = Odometry()
        odom.header = msg.header
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose = msg.pose
        
        self.ekf_pub.publish(odom)

def main(args=None):
    rclpy.init(args=args)
    node = OdomEKF()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
