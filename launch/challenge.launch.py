from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='snc_challenge',
            executable='navigation_node_executable',
            output='screen'
        ),
        Node(
            package='snc_challenge',
            executable='detection_node_executable',
            output='screen'
        ),
        Node(
            package='snc_challenge',
            executable='tracking_node_executable',
            output='screen'
        )
    ])