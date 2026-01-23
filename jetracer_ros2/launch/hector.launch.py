import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """
    Launch file for Hector SLAM in ROS 2
    Note: Hector SLAM is available for ROS 2 but less commonly used than SLAM Toolbox
    """
    
    # Declare arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    declare_use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation/Gazebo clock'
    )
    
    # Hector mapping node
    hector_mapping_node = Node(
        package='hector_mapping',
        executable='hector_mapping',
        name='hector_mapping',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'map_frame': 'map'},
            {'base_frame': 'base_footprint'},
            {'odom_frame': 'base_footprint'},  # Hector doesn't use odometry
            {'scan_topic': 'scan'},
            {'map_resolution': 0.05},
            {'map_size': 2048},
            {'map_start_x': 0.5},
            {'map_start_y': 0.5},
            {'map_update_distance_thresh': 0.2},
            {'map_update_angle_thresh': 0.2},
            {'laser_min_dist': 0.2},
            {'laser_max_dist': 8.0},
            {'pub_map_odom_transform': True},
            {'tf_map_scanmatch_transform_frame_name': 'scanmatcher_frame'}
        ]
    )
    
    ld = LaunchDescription()
    
    # Add arguments
    ld.add_action(declare_use_sim_time_argument)
    
    # Add nodes
    ld.add_action(hector_mapping_node)
    
    return ld
