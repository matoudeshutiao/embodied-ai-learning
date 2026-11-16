#!/usr/bin/env python3
"""把 Gazebo 里的 Panda 从默认零位移动到合法初始位姿。"""

import rclpy
from builtin_interfaces.msg import Duration, Time
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


JOINTS = [f"panda_joint{i}" for i in range(1, 8)]
HOME = [
    0.0,
    -0.785398,
    0.0,
    -2.356194,
    0.0,
    1.570796,
    0.785398,
]


class PandaHome(Node):
    def __init__(self):
        super().__init__("panda_home")
        self.client = ActionClient(
            self,
            FollowJointTrajectory,
            "/panda_arm_controller/follow_joint_trajectory",
        )

    def run(self):
        if not self.client.wait_for_server(timeout_sec=30.0):
            self.get_logger().error(
                "panda_arm_controller action not ready"
            )
            return False

        trajectory = JointTrajectory()
        # 零时间戳 = 立即开始。
        trajectory.header.stamp = Time()
        trajectory.joint_names = JOINTS

        point = JointTrajectoryPoint()
        point.positions = HOME
        point.velocities = [0.0] * len(JOINTS)
        point.time_from_start = Duration(sec=4, nanosec=0)
        trajectory.points.append(point)

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory
        self.get_logger().info(
            "moving Panda to initial joint positions"
        )

        send_future = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().error("home goal rejected")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result
        self.get_logger().info(
            f"home result: code={result.error_code}, "
            f"message={result.error_string}"
        )
        return result.error_code == 0


def main(args=None):
    rclpy.init(args=args)
    node = PandaHome()
    try:
        node.run()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
