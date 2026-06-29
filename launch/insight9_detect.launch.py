from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='camera_bridge',
            executable='insight9_detect_ros',
            name='insight9_detect_ros',
            output='screen'
        ),
    ])
