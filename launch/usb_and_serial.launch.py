from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='camera_bridge',
            executable='usb_detect',
            name='usb_detect',
            output='screen'
        ),
        Node(
            package='camera_bridge',
            executable='k4a_serial_node',
            name='k4a_serial_node',
            output='screen',
            parameters=[{
                'port': '/dev/azurekinect',
                'baudrate': 115200
            }]
        ),
    ])
