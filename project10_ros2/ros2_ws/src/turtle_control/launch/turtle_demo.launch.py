from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="turtlesim",
            executable="turtlesim_node",
            name="turtlesim",
            output="screen",
        ),
        Node(
            package="turtle_control",
            executable="turtle_circle",
            name="turtle_circle",
            output="screen",
            parameters=[{
                "linear_speed": 2.0,
                "angular_speed": 1.0,
                "publish_rate": 10.0,
            }],
        ),
        Node(
            package="turtle_control",
            executable="pose_listener",
            name="pose_listener",
            output="screen",
        ),
    ])
