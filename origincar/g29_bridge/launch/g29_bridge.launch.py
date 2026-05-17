from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_path = PathJoinSubstitution([
        FindPackageShare('g29_bridge'),
        'config',
        'g29_bridge_params.yaml',
    ])

    return LaunchDescription([
        DeclareLaunchArgument('camera_device', default_value='/dev/video0'),
        DeclareLaunchArgument('image_width', default_value='640'),
        DeclareLaunchArgument('image_height', default_value='480'),
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyACM0'),

        # Horizon USB camera driver (installed in /opt/tros)
        Node(
            package='hobot_usb_cam',
            executable='hobot_usb_cam',
            name='hobot_usb_cam',
            parameters=[{
                'video_device': LaunchConfiguration('camera_device'),
                'image_width': LaunchConfiguration('image_width'),
                'image_height': LaunchConfiguration('image_height'),
                'pixel_format': 'mjpeg',
                'io_method': 'mmap',
                'framerate': 30,
                'frame_id': 'camera_link',
            }],
            remappings=[('image_raw', '/image_raw')],
            arguments=['--ros-args', '--log-level', 'warn'],
        ),

        # Camera HTTP MJPEG streamer
        Node(
            package='g29_bridge',
            executable='camera_streamer.py',
            name='camera_streamer',
            parameters=[config_path],
        ),

        # G29 UDP bridge node
        Node(
            package='g29_bridge',
            executable='g29_bridge_node.py',
            name='g29_bridge_node',
            parameters=[config_path],
        ),

        # Origincar base driver (direct angle mode)
        Node(
            package='origincar_base',
            executable='origincar_base_node',
            name='origincar_base',
            parameters=[config_path],
        ),
    ])
