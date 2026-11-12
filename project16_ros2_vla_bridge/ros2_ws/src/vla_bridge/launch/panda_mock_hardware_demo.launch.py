#!/usr/bin/env python3
"""Panda + mock hardware + RViz，不启动 Gazebo。"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    panda_config = get_package_share_directory(
        "moveit_resources_panda_moveit_config"
    )
    vla_share = get_package_share_directory("vla_bridge")

    xacro_file = os.path.join(panda_config, "config", "panda.urdf.xacro")
    initial_positions = os.path.join(
        panda_config, "config", "initial_positions.yaml"
    )
    controllers_file = os.path.join(
        panda_config, "config", "ros2_controllers.yaml"
    )
    rviz_file = os.path.join(vla_share, "rviz", "panda_mock_view.rviz")

    start_rviz = LaunchConfiguration("start_rviz")

    # panda.urdf.xacro 用 initial_positions_file 初始化 mock hardware 的关节角。
    robot_description = ParameterValue(
        Command(
            [
                FindExecutable(name="xacro"),
                " ",
                xacro_file,
                " initial_positions_file:=",
                initial_positions,
            ]
        ),
        value_type=str,
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"robot_description": robot_description},
            controllers_file,
        ],
        output="screen",
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
        output="screen",
    )

    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

    # MoveIt 的 Panda 配置里 JTC 叫 panda_arm_controller。
    panda_arm_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["panda_arm_controller"],
        output="screen",
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_file],
        condition=IfCondition(start_rviz),
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "start_rviz",
                default_value="true",
                description="Start RViz with the Panda model.",
            ),
            control_node,
            robot_state_publisher,
            TimerAction(
                period=3.0,
                actions=[joint_state_broadcaster],
            ),
            TimerAction(
                period=5.0,
                actions=[panda_arm_controller],
            ),
            TimerAction(
                period=7.0,
                actions=[rviz],
            ),
        ]
    )
