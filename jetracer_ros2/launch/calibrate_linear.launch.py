import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """
    Launch file for linear calibration
    Moves the robot to test and calibrate odometry
    """
    
    # Get the package share directory
    pkg_share = get_package_share_directory('jetracer')
    
    # Declare arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    test_distance = LaunchConfiguration('test_distance', default='1.0')
    speed = LaunchConfiguration('speed', default='0.3')
    
    declare_use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation/Gazebo clock'
    )
    
    declare_test_distance = DeclareLaunchArgument(
        'test_distance',
        default_value='1.0',
        description='Distance to travel for calibration (meters)'
    )
    
    declare_speed = DeclareLaunchArgument(
        'speed',
        default_value='0.3',
        description='Speed for calibration test (m/s)'
    )
    
    # Calibrate linear node
    calibrate_node = Node(
        package='jetracer',
        executable='calibrate_linear.py',
        name='calibrate_linear',
        output='screen',
        parameters=[
            {
                'use_sim_time': use_sim_time,
                'test_distance': test_distance,
                'speed': speed,
                'tolerance': 0.03,
                'odom_linear_scale_correction': 1.0,
                'start_test': False,  # Set to true via ros2 param set to start
                'base_frame': 'base_footprint',
                'odom_frame': 'odom'
            }
        ]
    )
    
    ld = LaunchDescription()
    
    # Add arguments
    ld.add_action(declare_use_sim_time_argument)
    ld.add_action(declare_test_distance)
    ld.add_action(declare_speed)
    
    # Add nodes
    ld.add_action(calibrate_node)
    
    return ld
