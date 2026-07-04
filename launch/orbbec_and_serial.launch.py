from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('serial_port', default_value='/dev/orbbec'),
        DeclareLaunchArgument('serial_baud', default_value='115200'),

        Node(
            package='camera_bridge',
            executable='orbbec_detect',
            name='orbbec_detect',
            output='screen'
        ),
        Node(
            package='camera_bridge',
            executable='serial_node',
            name='serial_node',
            output='screen',
            parameters=[{
                'port': LaunchConfiguration('serial_port'),
                'baudrate': LaunchConfiguration('serial_baud')
            }]
        ),
    ])
