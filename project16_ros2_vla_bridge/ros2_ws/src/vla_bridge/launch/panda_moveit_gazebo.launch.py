#!/usr/bin/env python3
"""Panda + Gazebo + MoveIt2 move_group + MoveIt RViz。"""

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
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    vla_share = get_package_share_directory("vla_bridge")
    panda_share = get_package_share_directory("panda_gazebo_description")
    moveit_share = get_package_share_directory(
        "moveit_resources_panda_moveit_config"
    )

    start_rviz = LaunchConfiguration("start_moveit_rviz")
    start_camera_view = LaunchConfiguration("start_camera_view")
    rviz_config = LaunchConfiguration("rviz_config")
    camera_rviz_config = os.path.join(
        vla_share, "rviz", "panda_vision_view.rviz"
    )
    xacro_file = os.path.join(
        panda_share, "urdf", "panda_gazebo.urdf.xacro"
    )

    # 复用官方 Panda 的 SRDF/运动学/规划配置，
    # robot_description 换成带 Gazebo/ros2_control 的版本。
    moveit_config = (
        MoveItConfigsBuilder(
            "moveit_resources_panda",
            package_name="moveit_resources_panda_moveit_config",
        )
        .robot_description(file_path=xacro_file)
        .robot_description_semantic(file_path="config/panda.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    gazebo_demo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                vla_share,
                "launch",
                "panda_table_demo.launch.py",
            )
        ),
        launch_arguments={"start_rviz": "false"}.items(),
    )

    panda_scene = Node(
        package="vla_bridge",
        executable="panda_scene",
        output="screen",
    )

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": True},
        ],
        arguments=["--ros-args", "--log-level", "info"],
    )

    default_rviz_config = os.path.join(
        moveit_share, "launch", "moveit.rviz"
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            {"use_sim_time": True},
        ],
        condition=IfCondition(start_rviz),
    )

    # 第二个 RViz：只显示相机原图、检测 debug 图、目标 marker 和 TF。
    # 与 MoveIt RViz 分开，避免改乱 MoveIt 的默认配置。
    camera_rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="camera_rviz",
        output="screen",
        arguments=["-d", camera_rviz_config],
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(start_camera_view),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "start_moveit_rviz",
                default_value="true",
                description="Start MoveIt RViz MotionPlanning.",
            ),
            DeclareLaunchArgument(
                "start_camera_view",
                default_value="true",
                description="Start a second RViz with camera and debug image.",
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=default_rviz_config,
                description="RViz config file for MoveIt.",
            ),
            gazebo_demo,
            TimerAction(period=16.0, actions=[move_group]),
            TimerAction(period=12.0, actions=[camera_rviz]),
            TimerAction(period=18.0, actions=[panda_scene]),
            rviz,
        ]
    )
