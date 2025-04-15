from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Path to slam.yaml
    slam_config_path = os.path.join(
        get_package_share_directory('snc_challenge'),
        'config',
        'slam.yaml'
    )

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
        ),
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[slam_config_path]
        )
    ])
