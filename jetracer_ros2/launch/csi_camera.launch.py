import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """
    Launch file for CSI camera using gscam2 (ROS 2 port of gscam).
    Mirrors the original ROS1 csi_camera.launch using nvarguscamerasrc pipeline.

    Install gscam2:
        sudo apt install ros-${ROS_DISTRO}-gscam2
    or build from source: https://github.com/clydemcqueen/gscam2

    For headless (SSH) usage, the camera publishes image_raw which can be
    viewed on the laptop side via:
        ros2 run rqt_image_view rqt_image_view
    or with compressed transport:
        ros2 run image_transport republish raw compressed --ros-args \\
            -r in:=/csi_cam_0/image_raw -r out/compressed:=/csi_cam_0/image_compressed
    """

    pkg_share = get_package_share_directory('jetracer')

    # Declare arguments — matching ROS1 csi_camera.launch
    sensor_id = LaunchConfiguration('sensor_id', default='0')
    width = LaunchConfiguration('width', default='640')
    height = LaunchConfiguration('height', default='480')
    fps = LaunchConfiguration('fps', default='20')
    flip_method = LaunchConfiguration('flip_method', default='0')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    declare_sensor_id = DeclareLaunchArgument(
        'sensor_id', default_value='0',
        description='CSI camera sensor ID')

    declare_width = DeclareLaunchArgument(
        'width', default_value='640',
        description='Image width')

    declare_height = DeclareLaunchArgument(
        'height', default_value='480',
        description='Image height')

    declare_fps = DeclareLaunchArgument(
        'fps', default_value='20',
        description='Target framerate')

    declare_flip_method = DeclareLaunchArgument(
        'flip_method', default_value='0',
        description='nvvidconv flip method (0=none, 2=180deg)')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation clock')

    # GStreamer pipeline — identical to the original ROS1 gscam config.
    # nvarguscamerasrc -> NV12 in NVMM memory -> nvvidconv (flip) -> videoconvert -> appsink
    # The 'videoconvert' at the end lets gscam negotiate a format it can publish
    # (typically BGR or RGB). This is the pipeline that produces correct colors.
    gscam_config = (
        'nvarguscamerasrc sensor-id=0 ! '
        'video/x-raw(memory:NVMM), '
        'width=(int)640, height=(int)480, '
        'format=(string)NV12, framerate=(fraction)20/1 ! '
        'nvvidconv flip-method=0 ! '
        'videoconvert'
    )

    # Camera calibration file path
    camera_info_url = 'file://' + os.path.join(
        pkg_share, 'config', 'camera_calibration', 'cam_640x480.yaml')

    # gscam2 node — the ROS2 equivalent of the ROS1 gscam node
    gscam_node = Node(
        package='gscam2',
        executable='gscam_main',
        name='csi_cam_0',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'gscam_config': gscam_config,
            'camera_name': 'csi_cam_0',
            'camera_info_url': camera_info_url,
            'frame_id': 'csi_cam_0_link',
            'sync_sink': False,
        }],
        remappings=[
            ('camera/image_raw', '/csi_cam_0/image_raw'),
            ('camera/camera_info', '/csi_cam_0/camera_info'),
        ]
    )

    ld = LaunchDescription()
    ld.add_action(declare_sensor_id)
    ld.add_action(declare_width)
    ld.add_action(declare_height)
    ld.add_action(declare_fps)
    ld.add_action(declare_flip_method)
    ld.add_action(declare_use_sim_time)
    ld.add_action(gscam_node)

    return ld
