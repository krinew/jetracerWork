#!/usr/bin/env python3
"""
Web camera streamer for JetRacer — view the camera feed in any browser.

Usage:
    ros2 run jetracer_ros2 web_camera_stream
    Then open http://<jetson-ip>:5000 in your browser.
"""

import threading
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from flask import Flask, Response, render_template_string

# Global variables to share between ROS and Flask
app = Flask(__name__)
latest_frame = None
lock = threading.Lock()


class CameraSubscriber(Node):
    def __init__(self):
        super().__init__('web_camera_stream')
        self.bridge = CvBridge()
        
        self.declare_parameter('image_topic', '/csi_cam_0/image_raw')
        self.topic = self.get_parameter('image_topic').value
        
        self.subscription = self.create_subscription(
            Image, self.topic, self.image_callback, 1)
            
        self.get_logger().info(f'Subscribed to {self.topic} for web streaming')

    def image_callback(self, msg):
        global latest_frame
        try:
            # Convert ROS message straight to OpenCV array
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Compress to JPEG
            ret, jpeg = cv2.imencode('.jpg', cv_image, [cv2.IMWRITE_JPEG_QUALITY, 60])
            if ret:
                with lock:
                    latest_frame = jpeg.tobytes()
        except Exception as e:
            self.get_logger().error(f"Frame conversion error: {e}")


# ======================================================================
# Flask Web Server Logic 
# ======================================================================

def generate_stream():
    """Yields frames continuously to the browser"""
    import time
    while True:
        with lock:
            frame = latest_frame
        if frame is not None:
            # MJPEG stream formatting
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
            time.sleep(0.033) # roughly 30 FPS cap
        else:
            time.sleep(0.1)

@app.route('/stream')
def video_feed():
    # Return the stream with the special MJPEG content type
    return Response(generate_stream(), mimetype='multipart/x-mixed-replace; boundary=frame')

@app.route('/')
def index():
    # Dead simple HTML page
    html = """
    <html>
      <head><title>JetRacer Camera Feed</title></head>
      <body style="background: #1e1e1e; color: white; display: flex; flex-direction: column; align-items: center; justify-content: center; height: 100vh; margin: 0;">
        <h2>JetRacer Camera Feed</h2>
        <img src="/stream" style="max-width: 95vw; max-height: 85vh; border: 2px solid #444;" />
      </body>
    </html>
    """
    return render_template_string(html)


# ======================================================================
# Main
# ======================================================================

def ros_spin_thread():
    rclpy.init()
    node = CameraSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

def main():
    # 1. Start ROS2 Node in a background thread
    ros_thread = threading.Thread(target=ros_spin_thread, daemon=True)
    ros_thread.start()

    # 2. Run Flask Web Server on the main thread
    print("Starting Flask server on http://0.0.0.0:5000")
    app.run(host='0.0.0.0', port=5000, threaded=True)

if __name__ == '__main__':
    main()
