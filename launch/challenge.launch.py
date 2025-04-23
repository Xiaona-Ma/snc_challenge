#!/usr/bin/env python3
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import os

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
            parameters=[
                {'use_sim_time': False},    # important if using simulation
                slam_config_path           # custom config file
            ]
        ), 
        
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='laser_to_body_link_tf',
            arguments=[
            '0', '0', '0.2',   # x y z of your lidar relative to body_link
            '0', '0', '0',     # roll pitch yaw
            'body_link',       # parent frame
            'laser'            # child frame
            ],
            parameters=[{'use_sim_time': False}],
            output='screen'
        )
    ])