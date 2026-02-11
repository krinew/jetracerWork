import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """
    Launch file for CSI camera on Jetson.
    Uses a custom Python node (csi_camera_node.py) that captures from the
    nvarguscamerasrc GStreamer pipeline via OpenCV — same pipeline as the
    original ROS1 gscam launch, zero extra apt packages required.

    Published topics:
        /csi_cam_0/image_raw     (sensor_msgs/Image, BGR8)
        /csi_cam_0/camera_info   (sensor_msgs/CameraInfo)

    View over SSH:
        ros2 run jetracer web_camera_stream.py
        → open http://<jetson-ip>:8080 in laptop browser
    """

    pkg_share = get_package_share_directory('jetracer')

    # Camera calibration file
    camera_info_url = 'file://' + os.path.join(
        pkg_share, 'config', 'camera_calibration', 'cam_640x480.yaml')

    # Declare arguments — matching ROS1 csi_camera.launch
    declare_sensor_id = DeclareLaunchArgument(
        'sensor_id', default_value='0',
        description='CSI camera sensor ID')
    declare_width = DeclareLaunchArgument(
        'width', default_value='640',
        description='Image width')
    declare_height = DeclareLaunchArgument(
        'height', default_value='480',
        description='Image height')
    declare_fps = DeclareLaunchArgument(
        'fps', default_value='20',
        description='Target framerate')
    declare_flip_method = DeclareLaunchArgument(
        'flip_method', default_value='0',
        description='nvvidconv flip method (0=none, 2=180deg)')
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation clock')

    # Custom camera node — uses OpenCV + GStreamer (nvarguscamerasrc)
    camera_node = Node(
        package='jetracer',
        executable='csi_camera_node.py',
        name='csi_cam_0',
        output='screen',
        parameters=[{
            'sensor_id': LaunchConfiguration('sensor_id'),
            'width': LaunchConfiguration('width'),
            'height': LaunchConfiguration('height'),
            'fps': LaunchConfiguration('fps'),
            'flip_method': LaunchConfiguration('flip_method'),
            'frame_id': 'csi_cam_0_link',
            'camera_info_url': camera_info_url,
        }]
    )

    ld = LaunchDescription()
    ld.add_action(declare_sensor_id)
    ld.add_action(declare_width)
    ld.add_action(declare_height)
    ld.add_action(declare_fps)
    ld.add_action(declare_flip_method)
    ld.add_action(declare_use_sim_time)
    ld.add_action(camera_node)

    return ld
