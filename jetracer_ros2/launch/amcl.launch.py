import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """
    Launch file for AMCL (Adaptive Monte Carlo Localization) in ROS 2
    """
    
    # Get the package share directory
    pkg_share = get_package_share_directory('jetracer')
    
    # Declare arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    amcl_params_file = LaunchConfiguration(
        'params_file',
        default=os.path.join(pkg_share, 'config', 'amcl_params.yaml')
    )
    
    declare_use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation/Gazebo clock'
    )
    
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(pkg_share, 'config', 'amcl_params.yaml'),
        description='Full path to the AMCL parameters file'
    )
    
    # AMCL node
    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[amcl_params_file, {'use_sim_time': use_sim_time}],
        remappings=[
            ('/scan', '/scan')
        ]
    )
    
    # Lifecycle manager for AMCL
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'autostart': True},
            {'node_names': ['amcl']}
        ]
    )
    
    ld = LaunchDescription()
    
    # Add arguments
    ld.add_action(declare_use_sim_time_argument)
    ld.add_action(declare_params_file_cmd)
    
    # Add nodes
    ld.add_action(amcl_node)
    ld.add_action(lifecycle_manager_node)
    
    return ld
