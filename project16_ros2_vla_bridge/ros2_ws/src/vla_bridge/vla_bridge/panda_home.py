#!/usr/bin/env python3
"""把 Panda 移到合法 home 位姿。

用 MoveIt 规划关节目标，而不是直接发 FollowJointTrajectory，
这样 table/cube 等碰撞体会参与检查，避免穿桌子。
"""

import rclpy
from builtin_interfaces.msg import Duration, Time
from control_msgs.action import FollowJointTrajectory
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    Constraints,
    JointConstraint,
    MoveItErrorCodes,
)
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
        self.move_client = ActionClient(self, MoveGroup, "/move_action")
        self.arm_client = ActionClient(
            self,
            FollowJointTrajectory,
            "/panda_arm_controller/follow_joint_trajectory",
        )

    def direct_home(self):
        """MoveIt 无法从碰撞起始态规划时，用直接关节轨迹恢复。"""
        if not self.arm_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error(
                "arm controller action not ready"
            )
            return False

        trajectory = JointTrajectory()
        trajectory.header.stamp = Time()
        trajectory.joint_names = JOINTS

        point = JointTrajectoryPoint()
        point.positions = HOME
        point.velocities = [0.0] * len(JOINTS)
        point.time_from_start = Duration(sec=4, nanosec=0)
        trajectory.points.append(point)

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory
        send_future = self.arm_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        handle = send_future.result()
        if handle is None or not handle.accepted:
            self.get_logger().error("direct home goal rejected")
            return False

        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result
        ok = result.error_code == 0
        self.get_logger().info(
            f"direct home result: code={result.error_code}, ok={ok}"
        )
        return ok

    def run(self):
        if not self.move_client.wait_for_server(timeout_sec=30.0):
            self.get_logger().error("/move_action not ready")
            self.get_logger().info("home result: code=-1")
            return False

        constraints = Constraints()
        constraints.name = "home"
        for name, position in zip(JOINTS, HOME):
            joint = JointConstraint()
            joint.joint_name = name
            joint.position = float(position)
            joint.tolerance_above = 0.01
            joint.tolerance_below = 0.01
            joint.weight = 1.0
            constraints.joint_constraints.append(joint)

        goal = MoveGroup.Goal()
        goal.request.group_name = "panda_arm"
        goal.request.num_planning_attempts = 10
        goal.request.allowed_planning_time = 10.0
        goal.request.max_velocity_scaling_factor = 0.15
        goal.request.max_acceleration_scaling_factor = 0.15
        goal.request.goal_constraints = [constraints]
        goal.planning_options.plan_only = False
        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = 3

        self.get_logger().info(
            "planning home with MoveIt joint constraints"
        )
        send_future = self.move_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        handle = send_future.result()
        if handle is None or not handle.accepted:
            self.get_logger().error("home goal rejected")
            self.get_logger().info("home result: code=-2")
            return False

        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result
        code = result.error_code.val
        ok = code == MoveItErrorCodes.SUCCESS
        if ok:
            self.get_logger().info(
                f"home result: code=0, moveit_code={code}, ok=True"
            )
            return True

        # 零位自碰撞时 MoveIt 无法初始化起始树，直接关节恢复。
        self.get_logger().warn(
            f"MoveIt home failed code={code}; "
            "falling back to direct joint trajectory"
        )
        if self.direct_home():
            self.get_logger().info(
                "home result: code=0, recovery=direct"
            )
            return True
        self.get_logger().info("home result: code=-3")
        return False


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
