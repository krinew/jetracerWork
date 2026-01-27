import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """
    Launch file for CSI camera
    Note: You'll need to install appropriate ROS 2 camera drivers
    Example uses image_publisher or v4l2_camera package
    """
    
    # Declare arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    camera_device = LaunchConfiguration('camera_device', default='/dev/video0')
    image_width = LaunchConfiguration('image_width', default='640')
    image_height = LaunchConfiguration('image_height', default='480')
    framerate = LaunchConfiguration('framerate', default='30')
    
    declare_use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation/Gazebo clock'
    )
    
    declare_camera_device = DeclareLaunchArgument(
        'camera_device',
        default_value='/dev/video0',
        description='Camera device path'
    )
    
    declare_image_width = DeclareLaunchArgument(
        'image_width',
        default_value='640',
        description='Image width'
    )
    
    declare_image_height = DeclareLaunchArgument(
        'image_height',
        default_value='480',
        description='Image height'
    )
    
    declare_framerate = DeclareLaunchArgument(
        'framerate',
        default_value='30',
        description='Camera framerate'
    )
    
    # V4L2 Camera node (install ros-jazzy-v4l2-camera)
    # Alternative: use gscam or other camera drivers compatible with Jetson
    camera_node = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='v4l2_camera',
        output='screen',
        parameters=[
            {
                'use_sim_time': use_sim_time,
                'video_device': camera_device,
                'image_size': [640, 480],  # Must be passed as a verified integer array, inputs often parsed as str
                'camera_frame_id': 'camera_link',
                'io_method': 'mmap',
                'pixel_format': 'YUYV'
            }
        ],
        remappings=[
            ('/image_raw', '/camera/image_raw'),
            ('/camera_info', '/camera/camera_info')
        ]
    )
    
    ld = LaunchDescription()
    
    # Add arguments
    ld.add_action(declare_use_sim_time_argument)
    ld.add_action(declare_camera_device)
    ld.add_action(declare_image_width)
    ld.add_action(declare_image_height)
    ld.add_action(declare_framerate)
    
    # Add nodes
    ld.add_action(camera_node)
    
    return ld
