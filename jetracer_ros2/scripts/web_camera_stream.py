#!/usr/bin/env python3
"""
Web camera streamer for JetRacer — view the camera feed in any browser.

Usage:
    ros2 run jetracer web_camera_stream.py
    Then open http://<jetson-ip>:8080 in your browser.

Parameters:
    ~/image_topic  (string, default: /csi_cam_0/image_raw)
    ~/port         (int,    default: 8080)
    ~/quality      (int,    default: 50)  JPEG quality 1-100
    ~/width        (int,    default: 0)   Resize width (0 = no resize)
    ~/height       (int,    default: 0)   Resize height (0 = no resize)
"""

import threading
import time
from http.server import HTTPServer, BaseHTTPRequestHandler

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


# Global frame buffer shared between ROS subscriber and HTTP server
_frame_jpeg = None
_frame_lock = threading.Lock()


class CameraSubscriber(Node):
    def __init__(self):
        super().__init__('web_camera_stream')

        # Declare parameters
        self.declare_parameter('image_topic', '/csi_cam_0/image_raw')
        self.declare_parameter('port', 8080)
        self.declare_parameter('quality', 50)
        self.declare_parameter('width', 0)
        self.declare_parameter('height', 0)

        self.topic = self.get_parameter('image_topic').value
        self.quality = self.get_parameter('quality').value
        self.resize_w = self.get_parameter('width').value
        self.resize_h = self.get_parameter('height').value

        self.subscription = self.create_subscription(
            Image, self.topic, self._image_cb, 1)

        self.get_logger().info(
            f'Subscribed to {self.topic}  |  '
            f'JPEG quality={self.quality}  |  '
            f'resize={"off" if not self.resize_w else f"{self.resize_w}x{self.resize_h}"}')

    # ------------------------------------------------------------------
    def _image_cb(self, msg: Image):
        """Convert ROS Image to JPEG and store in global buffer."""
        global _frame_jpeg

        try:
            # Decode ROS Image into numpy array
            frame = self._ros_image_to_cv2(msg)
            if frame is None:
                return

            # Optional resize
            if self.resize_w > 0 and self.resize_h > 0:
                frame = cv2.resize(frame, (self.resize_w, self.resize_h))

            # Encode as JPEG
            ret, jpeg = cv2.imencode(
                '.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, self.quality])
            if ret:
                with _frame_lock:
                    _frame_jpeg = jpeg.tobytes()
        except Exception as e:
            self.get_logger().error(f'Frame conversion error: {e}')

    # ------------------------------------------------------------------
    @staticmethod
    def _ros_image_to_cv2(msg: Image) -> np.ndarray:
        """
        Minimal ROS Image -> OpenCV conversion (avoids cv_bridge dependency).
        Supports the encodings gscam2 / v4l2_camera typically produce.
        """
        h, w = msg.height, msg.width
        encoding = msg.encoding.lower()
        data = np.frombuffer(msg.data, dtype=np.uint8)

        if encoding in ('rgb8',):
            return cv2.cvtColor(data.reshape((h, w, 3)), cv2.COLOR_RGB2BGR)
        elif encoding in ('bgr8',):
            return data.reshape((h, w, 3))
        elif encoding in ('mono8',):
            return data.reshape((h, w))
        elif encoding in ('rgba8',):
            return cv2.cvtColor(data.reshape((h, w, 4)), cv2.COLOR_RGBA2BGR)
        elif encoding in ('bgra8',):
            return cv2.cvtColor(data.reshape((h, w, 4)), cv2.COLOR_BGRA2BGR)
        elif encoding in ('yuv422', 'uyvy'):
            yuv = data.reshape((h, w, 2))
            return cv2.cvtColor(yuv, cv2.COLOR_YUV2BGR_UYVY)
        elif encoding in ('yuyv',):
            yuv = data.reshape((h, w, 2))
            return cv2.cvtColor(yuv, cv2.COLOR_YUV2BGR_YUYV)
        elif encoding in ('16uc1', 'mono16'):
            data16 = np.frombuffer(msg.data, dtype=np.uint16).reshape((h, w))
            return (data16 / 256).astype(np.uint8)
        else:
            # Best-effort: assume 3-channel BGR
            try:
                return data.reshape((h, w, 3))
            except ValueError:
                return None


# ======================================================================
# HTTP server — serves MJPEG stream and a simple HTML page
# ======================================================================

HTML_PAGE = b"""\
<!DOCTYPE html>
<html>
<head>
  <title>JetRacer Camera</title>
  <style>
    body { background: #1e1e1e; margin: 0; display: flex;
           justify-content: center; align-items: center; height: 100vh;
           flex-direction: column; font-family: sans-serif; color: #ccc; }
    img  { max-width: 95vw; max-height: 85vh; border: 2px solid #444; }
    h2   { margin-bottom: 10px; }
  </style>
</head>
<body>
  <h2>JetRacer Camera Stream</h2>
  <img src="/stream" />
</body>
</html>
"""


class StreamHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == '/':
            self.send_response(200)
            self.send_header('Content-Type', 'text/html')
            self.end_headers()
            self.wfile.write(HTML_PAGE)

        elif self.path == '/stream':
            self.send_response(200)
            self.send_header('Content-Type',
                             'multipart/x-mixed-replace; boundary=frame')
            self.end_headers()
            try:
                while True:
                    with _frame_lock:
                        jpeg = _frame_jpeg
                    if jpeg is None:
                        time.sleep(0.05)
                        continue
                    self.wfile.write(b'--frame\r\n')
                    self.wfile.write(b'Content-Type: image/jpeg\r\n')
                    self.wfile.write(
                        f'Content-Length: {len(jpeg)}\r\n\r\n'.encode())
                    self.wfile.write(jpeg)
                    self.wfile.write(b'\r\n')
                    time.sleep(0.033)  # ~30 fps cap
            except (BrokenPipeError, ConnectionResetError):
                pass

        elif self.path == '/snapshot':
            # Single JPEG snapshot
            with _frame_lock:
                jpeg = _frame_jpeg
            if jpeg is None:
                self.send_response(503)
                self.end_headers()
                self.wfile.write(b'No frame available yet')
                return
            self.send_response(200)
            self.send_header('Content-Type', 'image/jpeg')
            self.send_header('Content-Length', str(len(jpeg)))
            self.end_headers()
            self.wfile.write(jpeg)

        else:
            self.send_response(404)
            self.end_headers()

    def log_message(self, format, *args):
        pass  # Suppress per-request log spam


# ======================================================================
# Main
# ======================================================================

def main(args=None):
    rclpy.init(args=args)
    node = CameraSubscriber()

    port = node.get_parameter('port').value

    # Start HTTP server in a background thread
    server = HTTPServer(('0.0.0.0', port), StreamHandler)
    server_thread = threading.Thread(target=server.serve_forever, daemon=True)
    server_thread.start()
    node.get_logger().info(
        f'Web stream ready  ->  http://0.0.0.0:{port}')
    node.get_logger().info(
        f'  Open http://<jetson-ip>:{port} in your laptop browser')
    node.get_logger().info(
        f'  Snapshot: http://<jetson-ip>:{port}/snapshot')

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
