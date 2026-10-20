from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    parameters = {
        'follow_distance': LaunchConfiguration('follow_distance'),
        'tolerance': LaunchConfiguration('tolerance'),
        'right_detection_limit': LaunchConfiguration('right_detection_limit'),
        'forward_speed': LaunchConfiguration('forward_speed'),
        'turn_speed': LaunchConfiguration('turn_speed'),
        'lateral_gain': LaunchConfiguration('lateral_gain'),
        'alignment_gain': LaunchConfiguration('alignment_gain'),
        'max_lateral_speed': LaunchConfiguration('max_lateral_speed'),
        'max_angular_speed': LaunchConfiguration('max_angular_speed'),
        'back_right_limit_factor': LaunchConfiguration(
            'back_right_limit_factor'
        ),
    }

    return LaunchDescription([
        DeclareLaunchArgument('follow_distance', default_value='0.35'),
        DeclareLaunchArgument('tolerance', default_value='0.05'),
        DeclareLaunchArgument('right_detection_limit', default_value='1.0'),
        DeclareLaunchArgument('forward_speed', default_value='0.30'),
        DeclareLaunchArgument('turn_speed', default_value='0.50'),
        DeclareLaunchArgument('lateral_gain', default_value='0.5'),
        DeclareLaunchArgument('alignment_gain', default_value='0.5'),
        DeclareLaunchArgument('max_lateral_speed', default_value='0.40'),
        DeclareLaunchArgument('max_angular_speed', default_value='1.00'),
        DeclareLaunchArgument(
            'back_right_limit_factor', default_value='1.50'
        ),
        Node(
            package='my_robot_control',
            executable='my_robot_wallfollower_holonomic_exec',
            name='robot_wall_follower_holonomic_node',
            output='screen',
            parameters=[parameters],
        ),
    ])
