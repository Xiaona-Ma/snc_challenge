from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, LogInfo
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from launch_ros.actions import Node

def generate_launch_description() : 
     
    image_topic = '/camera/color/image_raw/compressed'
    image_topic_repeat = image_topic + '/repeat'
    use_compressed = 'true'

    depth_topic = '/camera/depth/image_raw'
    depth_topic_repeat = depth_topic + '/repeat'

    camera_info_topic = '/camera/depth/camera_info'
    camera_info_topic_repeat = '/camera/depth/camera_info/repeat'

    return LaunchDescription([
        # Setting environment variables (to ensure that logs are output to the terminal)
        SetEnvironmentVariable('RCUTILS_LOGGING_USE_STDOUT', '1'),
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '0'),

        # Set to false when running after training is finished
        DeclareLaunchArgument('gui', default_value = 'false', description = 'Launch GUI.'),
    
        # Our default camera topic. If streaming images, consider using the compressed image instead
        DeclareLaunchArgument('image_topic', default_value = image_topic, description = 'Image topic from the camera (best_effort).'),
        DeclareLaunchArgument('image_topic_repeat', default_value = image_topic_repeat, description = 'Image to repeat to for find object (reliable).'),
        DeclareLaunchArgument('use_compressed', default_value = use_compressed, description = 'Determine if compressed image is to be used'),

        # Path where you have saved the existing trained images
        LogInfo(msg=('AIIL_CHECKOUT_DIR, ', EnvironmentVariable(name='AIIL_CHECKOUT_DIR'))),
        DeclareLaunchArgument('objects_path', default_value = [EnvironmentVariable(name='AIIL_CHECKOUT_DIR'),'/humble_workspace/src/snc_challenge/trained_objects'], description = 'Path to the training images'),

        # Find Object 2D Setting. By default just use the standard settingsf
        DeclareLaunchArgument('settings_path', default_value = '~/.ros/find_object_2d.ini', description = 'Config file.'),     

        DeclareLaunchArgument('depth_topic', default_value = depth_topic, description = 'Depth of the object'),
        DeclareLaunchArgument('depth_topic_repeat', default_value = depth_topic_repeat, description = 'Depth of the object'),

        DeclareLaunchArgument('camera_info_topic', default_value = camera_info_topic, description = 'Depth of the object'),
        DeclareLaunchArgument('camera_info_topic_repeat', default_value = camera_info_topic_repeat, description = 'Depth of the object'),

        # Nodes to launch
        # Find Object 2D node
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
                # ('depth_registered/image_raw', LaunchConfiguration('depth_topic_repeat')),
                # ('depth_registered/camera_info', LaunchConfiguration('camera_info_topic_repeat')),
        ]),
        
        # Best Effort repeater since find_object ONLY uses reliable QoS
        Node(
            package='snc_challenge',
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
            executable='detection_node_executable',
            name='detection_node',
            output='screen',
            parameters=[
                {'sub_topic_name': LaunchConfiguration('image_topic')},
                {'repeat_topic_name': LaunchConfiguration('image_topic_repeat')},
                {'use_compressed': LaunchConfiguration('use_compressed')},
            ]
        ),
    ])