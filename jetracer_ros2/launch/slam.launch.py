import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition, UnlessCondition


def generate_launch_description():
    """
    Unified SLAM launch file - allows choosing between different SLAM methods
    Similar to ROS1's slam.launch with map_type argument
    """
    
    # Get the package share directory
    pkg_share = get_package_share_directory('jetracer')
    
    # Declare arguments
    slam_method = LaunchConfiguration('slam_method')
    use_sim_time = LaunchConfiguration('use_sim_time')
    enable_camera = LaunchConfiguration('enable_camera')
    
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
    
    declare_enable_camera_cmd = DeclareLaunchArgument(
        'enable_camera',
        default_value='false',
        description='Enable CSI camera node (requires Tegra libs in Docker)'
    )
    
    # Include jetracer base launch
    jetracer_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'jetracer.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )
    
    # Include lidar launch
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'lidar.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )
    
    # Include CSI camera launch (optional — requires Tegra libs mounted in Docker)
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'csi_camera.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
        condition=IfCondition(enable_camera)
    )
    
    # Conditionally include SLAM Toolbox
    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'slam_toolbox.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
        condition=IfCondition(PythonExpression(["'", slam_method, "' == 'slam_toolbox'"]))
    )

    # Conditionally include Cartographer
    cartographer_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'cartographer.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
        condition=IfCondition(PythonExpression(["'", slam_method, "' == 'cartographer'"]))
    )

    # Conditionally include Hector SLAM
    hector_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'hector.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
        condition=IfCondition(PythonExpression(["'", slam_method, "' == 'hector'"]))
    )
    
    ld = LaunchDescription()
    
    # Add arguments
    ld.add_action(declare_slam_method_cmd)
    ld.add_action(declare_use_sim_time_argument)
    ld.add_action(declare_enable_camera_cmd)
    
    # Add launch files
    ld.add_action(jetracer_launch)
    ld.add_action(lidar_launch)
    ld.add_action(camera_launch)
    ld.add_action(slam_toolbox_launch)
    ld.add_action(cartographer_launch)
    ld.add_action(hector_launch)
    
    return ld
