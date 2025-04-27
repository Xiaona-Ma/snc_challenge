#!/usr/bin/env python3
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import RegisterEventHandler, ExecuteProcess
from launch.event_handlers import OnShutdown
import os

def generate_launch_description():
    config_dir = get_package_share_directory('snc_challenge')
    slam_config_path = os.path.join(config_dir, 'config', 'slam.yaml')

    
    save_map_on_shutdown = RegisterEventHandler(
        OnShutdown(
            on_shutdown=[
                ExecuteProcess(
                    cmd=['ros2', 'run', 'nav2_map_server', 'map_saver_cli', '-f', 'auto_saved_map'],
                    output='screen'
                )
            ]
        )
    )
    
    return LaunchDescription([    

        # Nodes
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

        # SLAM Toolbox
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[slam_config_path, {'use_sim_time': False}],
            arguments=['--ros-args', '--log-level', 'slam_toolbox:=warn']
        ),

        # TF Publisher
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='laser_to_body_link_tf',
            arguments=['0', '0', '0.2', '0', '0', '0', 'body_link', 'laser'],
            parameters=[{'use_sim_time': False}],
            output='screen'
        ),
    ])