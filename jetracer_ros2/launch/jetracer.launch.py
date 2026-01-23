import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Declare Arguments
    # Note: 'use_sim_time' is standard in ROS 2.
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # 2. Define Nodes
    
    # Jetracer Driver Node (The C++ Driver)
    config = os.path.join(
        get_package_share_directory('jetracer'),
        'config',
        'jetracer.yaml'
        )

    jetracer_node = Node(
        package='jetracer',
        executable='jetracer',
        name='jetracer',
        output='screen',
        emulate_tty=True, # Improves log color on terminals
        parameters=[config, {'use_sim_time': use_sim_time}],
        remappings=[
            ('/odom', '/odom_raw') # Remap output to allow EKF to take over /odom
        ]
    )



    # Robot Pose EKF equivalent
    # In ROS 2, robot_pose_ekf is largely replaced by robot_localization (ekf_node).
    # Since robot_pose_ekf is quite old, the standard recommendation is robot_localization.
    # However, if you specifically need robot_pose_ekf, it might not be ported to ROS 2 fully.
    # We will assume you want to use the modern ROS 2 standard: robot_localization.
    
    # NOTE: You will need to install 'ros-jazzy-robot-localization' for this to work.
    # If you don't have it, this node will fail to start.
    # For now, I will comment it out and provide the 'static_transform' which is safe.
    
    # robot_localization_node = Node(
    #     package='robot_localization',
    #     executable='ekf_node',
    #     name='ekf_filter_node',
    #     output='screen',
    #     parameters=[os.path.join(get_package_share_directory('jetracer'), 'config', 'ekf.yaml')],
    # )

    
    # Static Transform: base_footprint -> base_imu_link
    # ROS 1: args="0 0 0.02 0 0 0 base_footprint base_imu_link 20"
    # ROS 2: arguments are x y z yaw pitch roll frame_id child_frame_id
    tf_base_to_imu = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_footprint_to_imu',
        arguments=['0', '0', '0.02', '0', '0', '0', 'base_footprint', 'base_imu_link']
    )

    # Odom EKF Python Node (Ported)
    odom_ekf_node = Node(
        package='jetracer',
        executable='odom_ekf.py', 
        name='odom_ekf_node',
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),
        
        jetracer_node,
        tf_base_to_imu,
        odom_ekf_node,
        # robot_localization_node # TODO: Enable if EKF is needed
    ])

