from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 设置环境变量（日志格式优化）
        SetEnvironmentVariable('RCUTILS_LOGGING_USE_STDOUT', '1'),
        SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '0'),

        # 声明启动参数
        DeclareLaunchArgument(
            'objects_path',  # 预训练好的危险标志模型路径
            default_value=[EnvironmentVariable('AIIL_CHECKOUT_DIR'), '/humble_workspace/src/snc_challenge/trained_objects'],
            description='Path to pre-trained hazard marker models'
        ),
        DeclareLaunchArgument(
            'image_topic',  # 摄像头原始图像话题
            default_value='/camera/color/image_raw/compressed',
            description='Input image topic (compressed)'
        ),
        DeclareLaunchArgument(
            'use_compressed',  # 是否使用压缩图像
            default_value='true',
            description='Use compressed image input'
        ),

        # 启动 find_object_2d 节点（核心检测逻辑）
        Node(
            package='find_object_2d',
            executable='find_object_2d',
            output='screen',
            parameters=[{
                'subscribe_depth': False,  # 是否订阅深度信息（此处用激光雷达替代）
                'gui': 'false',           # 关闭GUI以节省资源
                'objects_path': LaunchConfiguration('objects_path'),
            }],
            remappings=[
                ('image', '/image_repeat'),  # 重映射输入图像话题
            ]
        ),

        # 启动图像中继节点（解决 QoS 不匹配问题）
        Node(
            package='aiil_rosbot_demo',
            executable='best_effort_repeater',
            name='image_repeater',
            parameters=[
                {'sub_topic_name': LaunchConfiguration('image_topic')},
                {'repeat_topic_name': '/image_repeat'},
                {'use_compressed': LaunchConfiguration('use_compressed')},
            ]
        ),

        # 启动自定义检测节点
        Node(
            package='snc_challenge',
            executable='detection_node',
            output='screen',
            parameters=[{
                'marker_frame': 'camera_link',  # 摄像头坐标系
                'map_frame': 'map',             # 目标地图坐标系
            }]
        )
    ])