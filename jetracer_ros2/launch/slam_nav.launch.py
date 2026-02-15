import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """
    Combined SLAM and Navigation launch file.
    Launches SLAM for mapping and Nav2 for autonomous navigation simultaneously.
    
    Key difference from nav.launch.py: here SLAM provides the map->odom transform,
    so we must NOT run AMCL or map_server (they would conflict with SLAM).
    Nav2 bringup supports this via the 'slam' parameter — when slam:=True it
    skips amcl and map_server and expects an external SLAM node to provide the map.
    """
    
    # Get the package share directory
    pkg_share = get_package_share_directory('jetracer')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    # Declare arguments
    slam_method = LaunchConfiguration('slam_method', default='slam_toolbox')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    params_file = LaunchConfiguration('params_file',
        default=os.path.join(pkg_share, 'config', 'nav2_params.yaml'))
    
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

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(pkg_share, 'config', 'nav2_params.yaml'),
        description='Full path to the Nav2 parameters file'
    )
    
    # Include SLAM launch (which includes jetracer, lidar, and camera)
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'slam.launch.py')
        ),
        launch_arguments={
            'slam_method': slam_method,
            'use_sim_time': use_sim_time
        }.items()
    )
    
    # Include Nav2 bringup with slam:=True
    # This tells Nav2 to NOT start amcl and map_server (SLAM provides the map)
    # Only starts: controller_server, planner_server, recoveries_server,
    #              bt_navigator, waypoint_follower, and lifecycle_manager
    nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'autostart': 'true'
        }.items()
    )
    
    ld = LaunchDescription()
    
    # Add arguments
    ld.add_action(declare_slam_method_cmd)
    ld.add_action(declare_use_sim_time_argument)
    ld.add_action(declare_params_file_cmd)
    
    # Add launch files
    ld.add_action(slam_launch)
    ld.add_action(nav_launch)
    
    return ld
