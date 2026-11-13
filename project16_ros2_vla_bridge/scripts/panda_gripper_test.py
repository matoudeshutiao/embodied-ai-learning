#!/usr/bin/env python3
"""测试 Gazebo 里的 Panda 夹爪：打开 → 闭合 → 打开。"""

import time

import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


JOINTS = ["panda_finger_joint1", "panda_finger_joint2"]


def duration(seconds):
    sec = int(seconds)
    nanosec = int(round((seconds - sec) * 1e9))
    if nanosec >= 1_000_000_000:
        sec += 1
        nanosec = 0
    return Duration(sec=sec, nanosec=nanosec)


class GripperTest(Node):
    def __init__(self):
        super().__init__("panda_gripper_test")
        self.client = ActionClient(
            self,
            FollowJointTrajectory,
            "/panda_gripper_controller/follow_joint_trajectory",
        )

    def send_position(self, position, seconds, label):
        trajectory = JointTrajectory()
        trajectory.header.stamp = self.get_clock().now().to_msg()
        trajectory.joint_names = JOINTS

        point = JointTrajectoryPoint()
        point.positions = [position, position]
        point.time_from_start = duration(seconds)
        trajectory.points.append(point)

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory

        self.get_logger().info(
            f"{label}: position={position:.3f}, duration={seconds:.1f}s"
        )
        send_future = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error(f"{label}: goal rejected")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result
        self.get_logger().info(
            f"{label}: code={result.error_code}, "
            f"message={result.error_string}"
        )
        return result.error_code == 0


def main(args=None):
    rclpy.init(args=args)
    node = GripperTest()
    try:
        if not node.client.wait_for_server(timeout_sec=10.0):
            node.get_logger().error(
                "panda_gripper_controller action not ready"
            )
            return

        node.send_position(0.04, 1.5, "open")
        time.sleep(0.5)
        node.send_position(0.00, 1.5, "close")
        time.sleep(0.5)
        node.send_position(0.04, 1.5, "open again")
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
