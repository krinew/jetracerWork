#!/usr/bin/env python3
"""
CSI Camera node for JetRacer — publishes camera images using GStreamer via OpenCV.

Replaces gscam/gscam2 with zero extra dependencies (OpenCV is pre-installed on
Jetson with GStreamer support). Uses the same nvarguscamerasrc pipeline as the
original ROS1 package.

Published topics:
    /csi_cam_0/image_raw     (sensor_msgs/Image)
    /csi_cam_0/camera_info   (sensor_msgs/CameraInfo)

Parameters:
    ~/sensor_id      (int,    default: 0)
    ~/width          (int,    default: 640)
    ~/height         (int,    default: 480)
    ~/fps            (int,    default: 20)
    ~/flip_method    (int,    default: 0)
    ~/frame_id       (string, default: csi_cam_0_link)
    ~/camera_info_url (string, default: '')  path to camera calibration YAML
"""

import os
import yaml

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Image, CameraInfo
from builtin_interfaces.msg import Time


class CSICameraNode(Node):
    def __init__(self):
        super().__init__('csi_cam_0')

        # Declare parameters
        self.declare_parameter('sensor_id', 0)
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 20)
        self.declare_parameter('flip_method', 0)
        self.declare_parameter('frame_id', 'csi_cam_0_link')
        self.declare_parameter('camera_info_url', '')

        self.sensor_id = self.get_parameter('sensor_id').value
        self.width = self.get_parameter('width').value
        self.height = self.get_parameter('height').value
        self.fps = self.get_parameter('fps').value
        self.flip_method = self.get_parameter('flip_method').value
        self.frame_id = self.get_parameter('frame_id').value
        camera_info_url = self.get_parameter('camera_info_url').value

        # Build GStreamer pipeline — identical to ROS1 gscam config
        self.pipeline = (
            f'nvarguscamerasrc sensor-id={self.sensor_id} ! '
            f'video/x-raw(memory:NVMM), '
            f'width=(int){self.width}, height=(int){self.height}, '
            f'format=(string)NV12, framerate=(fraction){self.fps}/1 ! '
            f'nvvidconv flip-method={self.flip_method} ! '
            f'video/x-raw, format=(string)BGRx ! '
            f'videoconvert ! '
            f'video/x-raw, format=(string)BGR ! '
            f'appsink drop=1'
        )

        self.get_logger().info(f'GStreamer pipeline: {self.pipeline}')

        # Open capture
        self.cap = cv2.VideoCapture(self.pipeline, cv2.CAP_GSTREAMER)
        if not self.cap.isOpened():
            self.get_logger().error(
                'Failed to open camera! Check that nvarguscamerasrc is available '
                'and the CSI camera is connected.')
            self.get_logger().error(
                'Test with: gst-launch-1.0 nvarguscamerasrc ! fakesink')
            raise RuntimeError('Cannot open CSI camera')

        self.get_logger().info(
            f'Camera opened: sensor_id={self.sensor_id} '
            f'{self.width}x{self.height}@{self.fps}fps '
            f'flip={self.flip_method}')

        # QoS for image (best effort, keep last 1 — standard for camera topics)
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Publishers
        self.image_pub = self.create_publisher(Image, '/csi_cam_0/image_raw', qos)
        self.info_pub = self.create_publisher(CameraInfo, '/csi_cam_0/camera_info', qos)

        # Load camera info if available
        self.camera_info_msg = self._load_camera_info(camera_info_url)

        # Timer at target FPS
        period = 1.0 / self.fps
        self.timer = self.create_timer(period, self._timer_callback)
        self.frame_count = 0

    def _timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn('Failed to read frame', throttle_duration_sec=5.0)
            return

        now = self.get_clock().now().to_msg()

        # Publish Image
        img_msg = Image()
        img_msg.header.stamp = now
        img_msg.header.frame_id = self.frame_id
        img_msg.height = frame.shape[0]
        img_msg.width = frame.shape[1]
        img_msg.encoding = 'bgr8'
        img_msg.is_bigendian = False
        img_msg.step = frame.shape[1] * 3
        img_msg.data = frame.tobytes()
        self.image_pub.publish(img_msg)

        # Publish CameraInfo
        if self.camera_info_msg is not None:
            self.camera_info_msg.header.stamp = now
            self.camera_info_msg.header.frame_id = self.frame_id
            self.info_pub.publish(self.camera_info_msg)

        self.frame_count += 1
        if self.frame_count == 1:
            self.get_logger().info('First frame published')

    def _load_camera_info(self, url: str):
        """Load camera calibration from a YAML file (same format as ROS1 camera_calibration)."""
        if not url:
            self.get_logger().info('No camera_info_url set — CameraInfo will not be published')
            return None

        # Strip file:// prefix if present
        path = url
        if path.startswith('file://'):
            path = path[7:]

        if not os.path.isfile(path):
            self.get_logger().warn(f'Camera calibration file not found: {path}')
            return None

        try:
            with open(path, 'r') as f:
                cal = yaml.safe_load(f)

            msg = CameraInfo()
            msg.width = cal.get('image_width', self.width)
            msg.height = cal.get('image_height', self.height)
            msg.distortion_model = cal.get('distortion_model', 'plumb_bob')

            d = cal.get('distortion_coefficients', {})
            msg.d = [float(x) for x in d.get('data', [])]

            k = cal.get('camera_matrix', {})
            msg.k = [float(x) for x in k.get('data', [0]*9)]

            r = cal.get('rectification_matrix', {})
            msg.r = [float(x) for x in r.get('data', [1,0,0, 0,1,0, 0,0,1])]

            p = cal.get('projection_matrix', {})
            msg.p = [float(x) for x in p.get('data', [0]*12)]

            self.get_logger().info(f'Loaded camera calibration from {path}')
            return msg
        except Exception as e:
            self.get_logger().warn(f'Failed to load calibration: {e}')
            return None

    def destroy_node(self):
        if self.cap is not None:
            self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CSICameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
