from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
import launch_ros.actions

def generate_launch_description():
    akmcar = LaunchConfiguration('akmcar', default='false')
    use_direct_angle = LaunchConfiguration('use_direct_angle', default='false')

    robot_parameters = [
        {'usart_port_name': '/dev/ttyACM0',
         'serial_baud_rate': 115200,
         'robot_frame_id': 'base_footprint',
         'odom_frame_id': 'odom_combined',
         'cmd_vel': 'cmd_vel',
         'product_number': 0}
    ]

    return LaunchDescription([
        DeclareLaunchArgument(
            'akmcar',
            default_value='false',
            description='Use Ackermann car mode if true'
        ),
        DeclareLaunchArgument(
            'use_direct_angle',
            default_value='false',
            description='Use direct steering angle mode for Akm car (Vy_raw=1)'
        ),

        launch_ros.actions.Node(
            condition=IfCondition(akmcar),
            package='origincar_base',
            executable='origincar_base_node',
            parameters=robot_parameters + [
                {'akm_cmd_vel': 'ackermann_cmd'},
                {'use_direct_angle': use_direct_angle}
            ],
            remappings=[('/cmd_vel', 'cmd_vel')],
        ),

        launch_ros.actions.Node(
            condition=IfCondition(akmcar),
            package='origincar_base',
            executable='cmd_vel_to_ackermann_drive.py',
            name='cmd_vel_to_ackermann_drive',
        ),

        launch_ros.actions.Node(
            condition=UnlessCondition(akmcar),
            package='origincar_base',
            executable='origincar_base_node',
            parameters=robot_parameters + [{'akm_cmd_vel': 'none'}],
        )
    ])
