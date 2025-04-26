from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, LogInfo
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from launch_ros.actions import Node

def generate_launch_description() :
  
   ### --- Topic and parameter definitions --- ###
   # Original topic
   image_topic = '/oak/rgb/image_raw/compressed'
   # image_topic = '/camera/color/image_raw'
   depth_topic = '/camera/depth/image_raw'
   camera_info_topic = '/camera/depth/camera_info'

   # Repeat topic
   image_topic_repeat = image_topic + '/repeat'
   depth_topic_repeat = depth_topic + '/repeat'
   # camera_info_topic_repeat = camera_info_topic + '/repeat'

   # if the image is compressed or not
   use_compressed = 'true'

   return LaunchDescription([
       # --- Log Output Settings --- #
       SetEnvironmentVariable('RCUTILS_LOGGING_USE_STDOUT', '1'),
       SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '0'),


       # --- Launch Arguments --- #
       DeclareLaunchArgument('image_topic', default_value=image_topic, description='Image topic from the camera (best_effort).'),
       DeclareLaunchArgument('image_topic_repeat', default_value=image_topic_repeat,description='Image to repeat to for find object (reliable).'),

       DeclareLaunchArgument('depth_topic', default_value=depth_topic, description='Original depth map topic。'),
       DeclareLaunchArgument('depth_topic_repeat', default_value=depth_topic_repeat,description='depth map topic (reliable).'),

       DeclareLaunchArgument('camera_info_topic', default_value=camera_info_topic, description='Original camera internal parameters topic.'),
       
       DeclareLaunchArgument('gui', default_value='true', description='Launch GUI.'),
       DeclareLaunchArgument('use_compressed', default_value=use_compressed, description='Determine if compressed image is to be used.'),


       DeclareLaunchArgument('objects_path', default_value = [EnvironmentVariable(name='AIIL_CHECKOUT_DIR'),'/humble_workspace/src/snc_challenge/objects'], description = 'Path to the training images'),
       # Find Object 2D Setting. By default just use the standard settings
       DeclareLaunchArgument('settings_path', default_value = '~/.ros/find_object_2d.ini', description = 'Config file.'),    

       # --- Target Detection Node: find_object_2d --- #
       Node(
            package='find_object_2d',
            executable='find_object_2d',
            output='screen',
            parameters=[{
              'subscribe_depth': False, # True,
              'gui': LaunchConfiguration('gui'),
              'objects_path': LaunchConfiguration('objects_path'),
              'settings_path': LaunchConfiguration('settings_path')
            }],
            remappings=[
                ('image', LaunchConfiguration('image_topic_repeat')),
                # ('depth_registered/image_raw', LaunchConfiguration('depth_topic_repeat')),
                # ('depth_registered/camera_info', LaunchConfiguration('camera_info_topic_repeat')),
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
            package='aiil_rosbot_demo',
            executable='best_effort_repeater',
            name='depth_best_effort_repeater',
            output='screen',
            parameters=[
                {'sub_topic_name': LaunchConfiguration('depth_topic')},
                {'repeat_topic_name': LaunchConfiguration('depth_topic_repeat')},
                {'use_compressed': 'false'}
            ]
        ),

       # --- Hazard Sign Detection Node: detection_node --- #
       Node(
           package='snc_challenge',
           executable='detection_node',
           name='detection_node',
           output='screen',
           remappings=[
            #    ('objects',      LaunchConfiguration('objects_path')),
               ('image',        LaunchConfiguration('image_topic_repeat')),
               ('depth',        LaunchConfiguration('depth_topic_repeat')),
               ('camera_info',  LaunchConfiguration('camera_info_topic')),
               ('hazards',      'hazards'),
               ('start_marker', 'start_marker'),
           ],
       ),

       LogInfo(msg='========== hazard.launch.py Startup Complete. =========='),
   ])