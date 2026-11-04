#!/usr/bin/env python3

import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionServer
from rclpy.node import Node


class MockTrajectoryServer(Node):
    def __init__(self):
        super().__init__("mock_trajectory_server")
        self.server = ActionServer(
            self,
            FollowJointTrajectory,
            "/task/execute_trajectory",
            self.execute_callback,
        )
        self.get_logger().info("Mock trajectory action server ready")

    def execute_callback(self, goal_handle):
        trajectory = goal_handle.request.trajectory
        self.get_logger().info(
            f"accepted trajectory: {len(trajectory.points)} points, "
            f"joints={list(trajectory.joint_names)}"
        )

        feedback = FollowJointTrajectory.Feedback()
        feedback.joint_names = list(trajectory.joint_names)
        if trajectory.points:
            feedback.desired = trajectory.points[0]
        goal_handle.publish_feedback(feedback)

        goal_handle.succeed()

        result = FollowJointTrajectory.Result()
        result.error_code = 0
        result.error_string = "mock execution ok"
        return result


def main(args=None):
    rclpy.init(args=args)
    node = MockTrajectoryServer()
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
