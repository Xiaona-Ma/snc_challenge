#!/usr/bin/env python3
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import RegisterEventHandler, ExecuteProcess, DeclareLaunchArgument, SetEnvironmentVariable, LogInfo
from launch.event_handlers import OnShutdown
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
import os

def generate_launch_description():
    config_dir = get_package_share_directory('snc_challenge')
    slam_config_path = os.path.join(config_dir, 'config', 'slam.yaml')

    # Path to the map file
    image_topic = '/camera/color/image_raw/compressed'  # ROS2
    # image_topic = '/oak/rgb/image_raw/compressed' # ROS3
    depth_topic = '/camera/depth/image_raw'
    camera_info_topic = '/camera/depth/camera_info'

    # Repeat topic
    image_topic_repeat = image_topic + '/repeat'
    depth_topic_repeat = depth_topic + '/repeat'
    use_compressed = 'true'


    # Save map using map_saver_cli after shut down (ctrl C)
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

        # --- Log Output Settings --- #
        SetEnvironmentVariable('RCUTILS_LOGGING_USE_STDOUT', '1'),
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '0'),


        # --- Launch Arguments --- #
        DeclareLaunchArgument('gui', default_value='true', description='Launch GUI.'),
        DeclareLaunchArgument('use_compressed', default_value=use_compressed, description='Determine if compressed image is to be used.'),

        DeclareLaunchArgument('image_topic', default_value=image_topic, description='Image topic from the camera (best_effort).'),
        DeclareLaunchArgument('depth_topic', default_value=depth_topic, description='Original depth map topic。'),
        DeclareLaunchArgument('camera_info_topic', default_value=camera_info_topic, description='Original camera internal parameters topic.'),
        DeclareLaunchArgument('image_topic_repeat', default_value=image_topic_repeat, description='Image to repeat to for find object (reliable).'),
        DeclareLaunchArgument('depth_topic_repeat', default_value=depth_topic_repeat, description='depth map topic (reliable).'), 

        DeclareLaunchArgument('objects_path', default_value = [EnvironmentVariable(name='AIIL_CHECKOUT_DIR'),'/humble_workspace/src/snc_challenge/objects'], description = 'Path to the training images'),
        DeclareLaunchArgument('settings_path', default_value = '~/.ros/find_object_2d.ini', description = 'Config file.'),    

        # Nodes
        # --- Target Detection Node: find_object_2d --- #
        Node(
            package='find_object_2d',
            executable='find_object_2d',
            output='screen',
            parameters=[{
                'subscribe_depth': False, # True,
                'gui': LaunchConfiguration('gui'),
                'objects_path': LaunchConfiguration('objects_path'),          # Path to the training images
                'settings_path': LaunchConfiguration('settings_path')         # Path to the config file
            }],
            remappings=[
                ('image', LaunchConfiguration('image_topic_repeat')),
        ]),

        # Best Effort repeater since find_object ONLY uses reliable QoS
        Node(
            package='aiil_rosbot_demo',
            executable='best_effort_repeater',
            name='best_effort_repeater',
            output='screen',
            parameters=[
                {'sub_topic_name': LaunchConfiguration('image_topic')},
                {'repeat_topic_name': LaunchConfiguration('image_topic_repeat')},
                {'use_compressed': LaunchConfiguration('use_compressed')},
            ]
        ),

        Node(
            package='snc_challenge',
            executable='navigation_node_executable',
            output='screen'
        ),
        Node(
           package='snc_challenge',
           executable='detection_node_executable',
           name='detection_node',
           output='screen',
           remappings=[
               ('image',        LaunchConfiguration('image_topic_repeat')),
               ('depth',        LaunchConfiguration('depth_topic')),
               ('camera_info',  LaunchConfiguration('camera_info_topic')),
               ('hazards',      'hazards'),
               ('start_marker', 'start_marker'),
           ],
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

        LogInfo(msg='========== hazard.launch.py Startup Complete. =========='),
    ])
