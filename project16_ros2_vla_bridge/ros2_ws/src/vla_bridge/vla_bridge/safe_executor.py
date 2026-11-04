#!/usr/bin/env python3

import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory


class SafeExecutor(Node):
    def __init__(self):
        super().__init__("safe_executor")

        self.client = ActionClient(
            self,
            FollowJointTrajectory,
            "/task/execute_trajectory",
        )
        self.sub = self.create_subscription(
            JointTrajectory,
            "/policy/action_safe",
            self.on_safe_action,
            10,
        )

        self.busy = False
        self.get_logger().info("Waiting for action server...")
        self.client.wait_for_server()
        self.get_logger().info("Trajectory action server ready")

    def on_safe_action(self, msg):
        if self.busy:
            return
        if not self.client.server_is_ready():
            return

        self.busy = True
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = msg

        self.get_logger().info(
            f"sending action: {len(msg.points)} points, "
            f"joints={list(msg.joint_names)}"
        )

        future = self.client.send_goal_async(
            goal,
            feedback_callback=self.on_feedback,
        )
        future.add_done_callback(self.on_goal_response)

    def on_feedback(self, feedback_msg):
        pass

    def on_goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("action goal rejected")
            self.busy = False
            return

        self.get_logger().info("action goal accepted")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.on_result)

    def on_result(self, future):
        result = future.result().result
        self.get_logger().info(
            f"action finished: code={result.error_code}, "
            f"message={result.error_string}"
        )
        self.busy = False


def main(args=None):
    rclpy.init(args=args)
    node = SafeExecutor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
