from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Defaults match the declare_parameter() defaults in the script.
    parameters = {
        'follow_distance': '0.35',
        'max_front_length': '0.50',
        'forward_speed': '0.30',
        'lateral_speed': '0.40',
        'distance_gain': '0.80',
        'angle_gain': '1.20',
        'max_angular_speed': '1.00',
        'detection_limit': '1.20',
        'back_right_limit_factor': '1.50',
    }

    return LaunchDescription([
        *[
            DeclareLaunchArgument(name, default_value=default)
            for name, default in parameters.items()
        ],
        Node(
            package='my_robot_control',
            executable='my_robot_wallfollower_holonomic_improved_exec',
            name='holonomic_wall_follower_improved',
            output='screen',
            parameters=[{
                name: LaunchConfiguration(name)
                for name in parameters
            }],
        ),
    ])