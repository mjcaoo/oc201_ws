from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('udp_port', default_value='9999'),
        DeclareLaunchArgument('max_speed', default_value='1.0'),
        DeclareLaunchArgument('max_steering_angle', default_value='45.0'),
        DeclareLaunchArgument('camera_device', default_value='/dev/video0'),
        DeclareLaunchArgument('image_width', default_value='640'),
        DeclareLaunchArgument('image_height', default_value='480'),
        DeclareLaunchArgument('http_port', default_value='8080'),
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyACM0'),

        # USB camera driver
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usb_cam',
            parameters=[{
                'video_device': LaunchConfiguration('camera_device'),
                'image_width': LaunchConfiguration('image_width'),
                'image_height': LaunchConfiguration('image_height'),
                'pixel_format': 'yuyv',
                'io_method': 'mmap',
                'framerate': 30.0,
                'camera_frame_id': 'camera_link',
            }],
            remappings=[('image_raw', '/image_raw')],
        ),

        # Camera HTTP MJPEG streamer
        Node(
            package='g29_bridge',
            executable='camera_streamer.py',
            name='camera_streamer',
            parameters=[{
                'image_topic': '/image_raw',
                'http_port': LaunchConfiguration('http_port'),
            }],
        ),

        # G29 UDP bridge node
        Node(
            package='g29_bridge',
            executable='g29_bridge_node.py',
            name='g29_bridge_node',
            parameters=[{
                'udp_port': LaunchConfiguration('udp_port'),
                'max_speed': LaunchConfiguration('max_speed'),
                'max_steering_angle': LaunchConfiguration('max_steering_angle'),
                'ackermann_topic': 'ackermann_cmd',
                'timeout_sec': 0.5,
            }],
        ),

        # Origincar base driver (direct angle mode)
        Node(
            package='origincar_base',
            executable='origincar_base_node',
            name='origincar_base',
            parameters=[{
                'usart_port_name': LaunchConfiguration('serial_port'),
                'serial_baud_rate': 115200,
                'robot_frame_id': 'base_footprint',
                'odom_frame_id': 'odom_combined',
                'cmd_vel': 'cmd_vel',
                'akm_cmd_vel': 'ackermann_cmd',
                'use_direct_angle': True,
            }],
        ),
    ])
