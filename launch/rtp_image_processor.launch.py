from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'udp_port',
            default_value='5008',
            description='UDP port for RTP stream'
        ),
        DeclareLaunchArgument(
            'jpeg_quality',
            default_value='90',
            description='JPEG compression quality (0-100)'
        ),
        DeclareLaunchArgument(
            'buffer_size',
            default_value='8388608',
            description='Buffer size in bytes'
        ),
        DeclareLaunchArgument(
            'max_buffers',
            default_value='3',
            description='Maximum number of buffers'
        ),
        DeclareLaunchArgument(
            'publish_raw',
            default_value='false',
            description='Publish raw images'
        ),
        DeclareLaunchArgument(
            'publish_compressed',
            default_value='true',
            description='Publish compressed images'
        ),
        DeclareLaunchArgument(
            'frame_id',
            default_value='camera',
            description='Frame ID for published images'
        ),
        DeclareLaunchArgument(
            'namespace',
            default_value='rtp_processor',
            description='Namespace for the node'
        ),
        
        Node(
            package='rtp_image_processor',
            executable='rtp_image_processor_node',
            name='rtp_image_processor',
            namespace=LaunchConfiguration('namespace'),
            parameters=[{
                'udp_port': LaunchConfiguration('udp_port'),
                'jpeg_quality': LaunchConfiguration('jpeg_quality'),
                'buffer_size': LaunchConfiguration('buffer_size'),
                'max_buffers': LaunchConfiguration('max_buffers'),
                'publish_raw': LaunchConfiguration('publish_raw'),
                'publish_compressed': LaunchConfiguration('publish_compressed'),
                'frame_id': LaunchConfiguration('frame_id'),
            }],
            output='screen',
            emulate_tty=True,
        )
    ])