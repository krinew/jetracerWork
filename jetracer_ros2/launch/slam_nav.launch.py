import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """
    Combined SLAM and Navigation launch file
    Launches SLAM for mapping and Nav2 for autonomous navigation simultaneously
    """
    
    # Get the package share directory
    pkg_share = get_package_share_directory('jetracer')
    
    # Declare arguments
    slam_method = LaunchConfiguration('slam_method', default='slam_toolbox')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    
    declare_slam_method_cmd = DeclareLaunchArgument(
        'slam_method',
        default_value='slam_toolbox',
        description='SLAM method to use: slam_toolbox, cartographer, or hector'
    )
    
    declare_use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation/Gazebo clock'
    )
    
    # Include SLAM launch (which includes jetracer and lidar)
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'slam.launch.py')
        ),
        launch_arguments={
            'slam_method': slam_method,
            'use_sim_time': use_sim_time
        }.items()
    )
    
    # Include Nav2 launch
    nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'nav.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'autostart': 'true'
        }.items()
    )
    
    ld = LaunchDescription()
    
    # Add arguments
    ld.add_action(declare_slam_method_cmd)
    ld.add_action(declare_use_sim_time_argument)
    
    # Add launch files
    ld.add_action(slam_launch)
    ld.add_action(nav_launch)
    
    return ld
