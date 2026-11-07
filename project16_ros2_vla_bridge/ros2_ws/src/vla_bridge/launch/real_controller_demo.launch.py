#!/usr/bin/env python3

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
    vla_share = get_package_share_directory("vla_bridge")
    arm_share = get_package_share_directory("arm_description")
    gazebo_share = get_package_share_directory("gazebo_ros")

    # 做隔离队列测试时设为 false，避免 mock_policy 和测试节点同时发动作。
    start_mock_policy = LaunchConfiguration("start_mock_policy")

    xacro_file = os.path.join(
        arm_share, "urdf", "simple_arm.urdf.xacro"
    )
    rviz_file = os.path.join(
        vla_share, "rviz", "real_controller_view.rviz"
    )

    # 把 xacro 展开成 URDF 字符串，交给 robot_state_publisher。
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

    # 等 Gazebo 和 robot_description 准备好，再把模型生成到世界里。
    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-topic", "robot_description",
            "-entity", "simple_arm",
            "-timeout", "60.0",
        ],
        output="screen",
    )

    # 这两个控制器由 Gazebo 里的 controller_manager 管理。
    # 先启动状态广播，再启动轨迹控制器。
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

    # 策略仍然使用 mock，但执行端已经换成真实控制器。
    safety_filter = Node(
        package="vla_bridge",
        executable="safety_filter",
        parameters=[{"use_sim_time": True}],
        output="screen",
    )
    mock_policy = Node(
        package="vla_bridge",
        executable="mock_policy",
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(start_mock_policy),
        output="screen",
    )
    safe_executor = Node(
        package="vla_bridge",
        executable="safe_executor",
        parameters=[
            {
                "use_sim_time": True,
                "action_name": (
                    "/joint_trajectory_controller/"
                    "follow_joint_trajectory"
                ),
            }
        ],
        output="screen",
    )

    # 默认打开可视化，直接看到机械臂和 TF。
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_file],
        parameters=[{"use_sim_time": True}],
        output="screen",
    )

    # 用进程退出事件串联启动顺序，避免控制器在 Gazebo 还没准备好时启动。
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
    start_jtc = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster,
            on_exit=[
                TimerAction(
                    period=1.0,
                    actions=[joint_trajectory_controller],
                )
            ],
        )
    )
    start_bridge = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_trajectory_controller,
            on_exit=[
                TimerAction(
                    period=1.0,
                    actions=[
                        safety_filter,
                        mock_policy,
                        safe_executor,
                        rviz,
                    ],
                )
            ],
        )
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "start_mock_policy",
                default_value="true",
                description=(
                    "Start mock_policy. Set false for isolated queue tests."
                ),
            ),
            gazebo,
            robot_state_publisher,
            TimerAction(period=3.0, actions=[spawn_entity]),
            start_jsb,
            start_jtc,
            start_bridge,
        ]
    )
