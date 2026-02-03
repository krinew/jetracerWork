import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    serial_port = LaunchConfiguration('serial_port', default='/dev/ttyACM1')
    serial_baudrate = LaunchConfiguration('serial_baudrate', default='115200')
    frame_id = LaunchConfiguration('frame_id', default='laser_frame')

    return LaunchDescription([
        DeclareLaunchArgument(
            'serial_port',
            default_value='/dev/ttyACM1',
            description='Specifying usb port to connected lidar'),

        DeclareLaunchArgument(
            'serial_baudrate',
            default_value='115200',
            description='Specifying usb port baudrate to connected lidar'),
        
        DeclareLaunchArgument(
            'frame_id',
            default_value='laser_frame',
            description='Specifying frame_id of lidar'),

        # RPLidar Node
        # Note: Ensure 'rplidar_ros' package is installed in ROS 2
        # Use 'rplidar_composition' if 'rplidar_node' is missing (ROS 2 Jazzy update)
        Node(
            package='rplidar_ros',
            executable='rplidar_composition',
            name='rplidar_node',
            parameters=[{
                'serial_port': serial_port,
                'serial_baudrate': serial_baudrate,
                'frame_id': frame_id,
                'inverted': False,
                'angle_compensate': True
            }],
            output='screen'
        ),

        # Static Transform: base_footprint -> laser_frame
        # ROS 1: args="0 0.0 0.1 3.14 0.0 0.0 /base_footprint /laser_frame 20"
        # ROS 2: x y z yaw pitch roll
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_footprint_to_laser',
            arguments=['0', '0', '0.1', '3.14', '0', '0', 'base_footprint', 'laser_frame']
        )
    ])
