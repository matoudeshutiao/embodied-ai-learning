#!/usr/bin/env python3
"""Panda + Gazebo Classic + 7 关节 + 夹爪。"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    panda_share = get_package_share_directory("panda_gazebo_description")
    vla_share = get_package_share_directory("vla_bridge")
    gazebo_share = get_package_share_directory("gazebo_ros")

    xacro_file = os.path.join(
        panda_share, "urdf", "panda_gazebo.urdf.xacro"
    )
    rviz_file = os.path.join(vla_share, "rviz", "panda_mock_view.rviz")
    start_rviz = LaunchConfiguration("start_rviz")

    robot_description = ParameterValue(
        Command([FindExecutable(name="xacro"), " ", xacro_file]),
        value_type=str,
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_share, "launch", "gazebo.launch.py")
        )
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {
                "robot_description": robot_description,
                "use_sim_time": True,
            }
        ],
        output="screen",
    )

    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-topic", "robot_description",
            "-entity", "panda",
            "-timeout", "60.0",
        ],
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
    panda_gripper_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["panda_gripper_controller"],
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

    start_jsb = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_entity,
            on_exit=[
                TimerAction(
                    period=1.0,
                    actions=[joint_state_broadcaster],
                )
            ],
        )
    )
    start_controllers = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster,
            on_exit=[
                TimerAction(
                    period=1.0,
                    actions=[
                        panda_arm_controller,
                        panda_gripper_controller,
                    ],
                )
            ],
        )
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "start_rviz",
                default_value="true",
                description="Start RViz with the Panda model.",
            ),
            gazebo,
            robot_state_publisher,
            spawn_entity,
            start_jsb,
            start_controllers,
            TimerAction(period=4.0, actions=[rviz]),
        ]
    )
