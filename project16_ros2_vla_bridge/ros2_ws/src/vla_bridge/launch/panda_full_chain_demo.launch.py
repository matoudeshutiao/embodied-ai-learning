#!/usr/bin/env python3
"""Panda + mock hardware + safety_filter + safe_executor + mock_policy。"""

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


PANDA_JOINTS = [f"panda_joint{i}" for i in range(1, 8)]
PANDA_LOWER = [
    -2.8973,
    -1.7628,
    -2.8973,
    -3.0718,
    -2.8973,
    -0.0175,
    -2.8973,
]
PANDA_UPPER = [
    2.8973,
    1.7628,
    2.8973,
    -0.0698,
    2.8973,
    3.7525,
    2.8973,
]
PANDA_VELOCITY = [2.175, 2.175, 2.175, 2.175, 2.610, 2.610, 2.610]
PANDA_HOME = [
    0.0,
    -0.785398,
    0.0,
    -2.356194,
    0.0,
    1.570796,
    0.785398,
]
PANDA_AMPLITUDES = [0.15] * 7
PANDA_PHASES = [0.0, 0.4, 0.8, 1.2, 1.6, 2.0, 2.4]


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

    panda_arm_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["panda_arm_controller"],
        output="screen",
    )

    safety_filter = Node(
        package="vla_bridge",
        executable="safety_filter",
        parameters=[
            {
                "state_timeout": 0.5,
                "watchdog_timeout": 2.0,
                "joint_names": PANDA_JOINTS,
                "lower_limits": PANDA_LOWER,
                "upper_limits": PANDA_UPPER,
                "velocity_limits": PANDA_VELOCITY,
            }
        ],
        output="screen",
    )

    safe_executor = Node(
        package="vla_bridge",
        executable="safe_executor",
        parameters=[
            {
                "action_name": (
                    "/panda_arm_controller/follow_joint_trajectory"
                ),
                "max_in_flight": 2,
                "max_action_age": 0.5,
            }
        ],
        output="screen",
    )

    mock_policy = Node(
        package="vla_bridge",
        executable="mock_policy",
        parameters=[
            {
                "joint_names": PANDA_JOINTS,
                "home_positions": PANDA_HOME,
                "amplitudes": PANDA_AMPLITUDES,
                "phase_offsets": PANDA_PHASES,
            }
        ],
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
                actions=[
                    safety_filter,
                    safe_executor,
                    mock_policy,
                    rviz,
                ],
            ),
        ]
    )
