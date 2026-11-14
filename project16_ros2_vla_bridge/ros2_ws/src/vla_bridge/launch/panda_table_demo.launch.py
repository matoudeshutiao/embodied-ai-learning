#!/usr/bin/env python3
"""Panda + 桌面 + RGB-D 相机 + HSV 检测 + RViz。"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    panda_share = get_package_share_directory("panda_gazebo_description")
    vla_share = get_package_share_directory("vla_bridge")
    world = os.path.join(
        panda_share, "worlds", "panda_table.world"
    )
    rviz_file = os.path.join(
        vla_share, "rviz", "panda_vision_view.rviz"
    )
    start_rviz = LaunchConfiguration("start_rviz")

    panda_demo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                vla_share,
                "launch",
                "panda_gazebo_demo.launch.py",
            )
        ),
        launch_arguments={
            "world": world,
            "start_rviz": "false",
        }.items(),
    )

    camera_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "--x", "0.55", "--y", "0.0", "--z", "1.15",
            "--roll", "0.0", "--pitch", "1.5708", "--yaw", "0.0",
            "--frame-id", "panda_link0",
            "--child-frame-id", "camera_link",
        ],
        output="screen",
    )
    optical_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "--x", "0", "--y", "0", "--z", "0",
            "--roll", "-1.5708", "--pitch", "0", "--yaw", "-1.5708",
            "--frame-id", "camera_link",
            "--child-frame-id", "camera_optical_frame",
        ],
        output="screen",
    )

    detector = Node(
        package="vla_bridge",
        executable="panda_color_detector",
        output="screen",
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_file],
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(start_rviz),
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "start_rviz",
                default_value="true",
                description="Start RViz with camera and debug image.",
            ),
            panda_demo,
            camera_tf,
            optical_tf,
            TimerAction(period=10.0, actions=[detector, rviz]),
        ]
    )
