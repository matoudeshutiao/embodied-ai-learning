#!/usr/bin/env python3
"""给 Panda mock hardware 发一条 7 关节轨迹，让模型在 RViz 里动起来。

不经过 safety_filter / safe_executor，只验证：
  panda_arm_controller 是否接受 FollowJointTrajectory
  /joint_states 是否反馈 7 个关节
"""

import time

import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


JOINTS = [f"panda_joint{i}" for i in range(1, 8)]
LIMITS = [
    (-2.8973, 2.8973),
    (-1.7628, 1.7628),
    (-2.8973, 2.8973),
    (-3.0718, -0.0698),
    (-2.8973, 2.8973),
    (-0.0175, 3.7525),
    (-2.8973, 2.8973),
]
DELTA = [0.15, 0.15, -0.15, 0.15, 0.15, 0.15, 0.15]


def clamp(value, lower, upper):
    return min(max(value, lower), upper)


def duration(seconds):
    sec = int(seconds)
    nanosec = int(round((seconds - sec) * 1e9))
    if nanosec >= 1_000_000_000:
        sec += 1
        nanosec = 0
    return Duration(sec=sec, nanosec=nanosec)


class PandaWave(Node):
    def __init__(self):
        super().__init__("panda_wave")
        self.positions = {}
        self.client = ActionClient(
            self,
            FollowJointTrajectory,
            "/panda_arm_controller/follow_joint_trajectory",
        )
        self.create_subscription(
            JointState, "/joint_states", self.on_state, 50
        )
        self.get_logger().info("waiting for 7 Panda joints on /joint_states")

    def on_state(self, msg):
        values = dict(zip(msg.name, msg.position))
        if all(name in values for name in JOINTS):
            self.positions = {name: values[name] for name in JOINTS}

    def wait_for_state(self, timeout=10.0):
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if len(self.positions) == len(JOINTS):
                return True
        return False

    def send_wave(self):
        if not self.client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("panda_arm_controller action not ready")
            return False

        q0 = [self.positions[name] for name in JOINTS]
        target = [
            clamp(q0[i] + DELTA[i], LIMITS[i][0], LIMITS[i][1])
            for i in range(len(JOINTS))
        ]

        traj = JointTrajectory()
        traj.header.stamp = self.get_clock().now().to_msg()
        traj.joint_names = JOINTS

        point = JointTrajectoryPoint()
        point.positions = target
        point.time_from_start = duration(2.0)
        traj.points.append(point)

        back = JointTrajectoryPoint()
        back.positions = q0
        back.time_from_start = duration(4.0)
        traj.points.append(back)

        self.get_logger().info(
            "sending 7-joint trajectory: "
            f"{len(traj.points)} points over 4.0s"
        )
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj

        send_future = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error("goal rejected")
            return False

        self.get_logger().info("goal accepted, waiting for result")
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result
        self.get_logger().info(
            f"result: code={result.error_code}, "
            f"message={result.error_string}"
        )
        return result.error_code == 0


def main(args=None):
    rclpy.init(args=args)
    node = PandaWave()
    try:
        if not node.wait_for_state():
            node.get_logger().error("no complete /joint_states")
            return
        node.send_wave()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
