from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="camera_mount_tf",
            arguments=[
                "--x", "0.3", "--y", "0.0", "--z", "0.5",
                "--roll", "-1.5708", "--pitch", "0.0", "--yaw", "-1.5708",
                "--frame-id", "base_link",
                "--child-frame-id", "camera_optical_frame",
            ],
            output="screen",
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="panda_base_tf",
            arguments=[
                "--x", "0.0", "--y", "0.0", "--z", "0.0",
                "--frame-id", "panda_link0",
                "--child-frame-id", "base_link",
            ],
            output="screen",
        ),
        Node(
            package="vision_demo",
            executable="synthetic_camera",
            name="synthetic_camera",
            output="screen",
        ),
        Node(
            package="vision_demo",
            executable="detector_node",
            name="color_detector",
            output="screen",
        ),
        Node(
            package="vision_demo",
            executable="task_node",
            name="target_task",
            output="screen",
        ),
    ])
