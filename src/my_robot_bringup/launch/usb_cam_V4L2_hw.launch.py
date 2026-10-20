from typing import List

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    image_width = LaunchConfiguration('image_width')
    image_height = LaunchConfiguration('image_height')
    video_device = LaunchConfiguration('video_device')
    pixel_format = LaunchConfiguration('pixel_format')
    output_encoding = LaunchConfiguration('output_encoding')

    declare_width = DeclareLaunchArgument(
        'image_width',
        default_value='640',
        description='Width of the camera image'
    )
    declare_height = DeclareLaunchArgument(
        'image_height',
        default_value='480',
        description='Height of the camera image'
    )
    declare_device = DeclareLaunchArgument(
        'video_device',
        default_value='/dev/video0',
        description='Video device for USB camera'
    )
    declare_pixel_format = DeclareLaunchArgument(
        'pixel_format',
        default_value='YUYV',
        description='Pixel format requested from the camera (e.g. YUYV, UYVY, GREY)'
    )
    declare_output_encoding = DeclareLaunchArgument(
        'output_encoding',
        default_value='rgb8',
        description='Output encoding published in ROS'
    )

    camera_node = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='camera',
        output='screen',
        respawn=True,
        respawn_delay=2.0,
        parameters=[{
            'video_device': video_device,
            'image_size': ParameterValue(
                ['[', image_width, ', ', image_height, ']'],
                value_type=List[int],
            ),
            'pixel_format': pixel_format,
            'output_encoding': output_encoding,
        }]
    )

    return LaunchDescription([
        declare_width,
        declare_height,
        declare_device,
        declare_pixel_format,
        declare_output_encoding,
        camera_node
    ])
