#!/usr/bin/env python3
"""不启动 Gazebo，用 ros2_control mock hardware 跑同一套控制器。"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    mock_share = get_package_share_directory("arm_mock_description")
    vla_share = get_package_share_directory("vla_bridge")

    xacro_file = os.path.join(
        mock_share, "urdf", "simple_arm_mock.urdf.xacro"
    )
    controllers_file = os.path.join(
        mock_share, "config", "controllers.yaml"
    )
    rviz_file = os.path.join(
        vla_share, "rviz", "real_controller_view.rviz"
    )

    start_mock_policy = LaunchConfiguration("start_mock_policy")
    start_rviz = LaunchConfiguration("start_rviz")

    robot_description = ParameterValue(
        Command([FindExecutable(name="xacro"), " ", xacro_file]),
        value_type=str,
    )

    # ros2_control_node 直接加载 mock hardware，不经过 Gazebo。
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

    joint_trajectory_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller"],
        output="screen",
    )

    # 保留和第 16 天相同的执行链，只是把 Gazebo 换成了 mock hardware。
    safety_filter = Node(
        package="vla_bridge",
        executable="safety_filter",
        parameters=[
            {
                "state_timeout": 0.5,
                "watchdog_timeout": 2.0,
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
                    "/joint_trajectory_controller/"
                    "follow_joint_trajectory"
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
        condition=IfCondition(start_mock_policy),
        output="screen",
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_file],
        condition=IfCondition(start_rviz),
        output="screen",
    )

    # 控制器必须等 ros2_control_node 和 URDF 就绪后再 spawn。
    start_jsb = TimerAction(
        period=4.0,
        actions=[joint_state_broadcaster],
    )
    start_jtc = TimerAction(
        period=6.0,
        actions=[joint_trajectory_controller],
    )
    start_bridge = TimerAction(
        period=8.0,
        actions=[
            safety_filter,
            safe_executor,
            mock_policy,
            rviz,
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "start_mock_policy",
                default_value="true",
                description="Start mock_policy after controllers are ready.",
            ),
            DeclareLaunchArgument(
                "start_rviz",
                default_value="true",
                description="Start RViz.",
            ),
            control_node,
            robot_state_publisher,
            start_jsb,
            start_jtc,
            start_bridge,
        ]
    )
