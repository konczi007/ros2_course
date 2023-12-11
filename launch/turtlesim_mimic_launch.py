from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='turtlesim_node',
            output='screen',
        ),
        Node(
            package='ros2_course',
            executable='turtlesim_koch',
            name='turtlesim_koch',
            output='screen',
        ),
        Node(
            package='ros2_course',
            executable='turtlesim_koch.py',
            name='run_turtlesim_koch',
            output='screen',
        ),
    ])
