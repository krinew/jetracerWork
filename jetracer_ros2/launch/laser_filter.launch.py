import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """
    Launch file for laser scan filter
    Filters laser scan data to remove invalid readings
    """
    
    # Get the package share directory
    pkg_share = get_package_share_directory('jetracer')
    
    # Path to filter configuration
    filter_params_file = os.path.join(pkg_share, 'config', 'laser_filter_params.yaml')
    
    # Declare arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    declare_use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation/Gazebo clock'
    )
    
    # Laser filter node
    laser_filter_node = Node(
        package='laser_filters',
        executable='scan_to_scan_filter_chain',
        name='laser_filter',
        output='screen',
        parameters=[
            filter_params_file,
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            ('/scan', '/scan_raw'),
            ('/scan_filtered', '/scan')
        ]
    )
    
    ld = LaunchDescription()
    
    # Add arguments
    ld.add_action(declare_use_sim_time_argument)
    
    # Add nodes
    ld.add_action(laser_filter_node)
    
    return ld
