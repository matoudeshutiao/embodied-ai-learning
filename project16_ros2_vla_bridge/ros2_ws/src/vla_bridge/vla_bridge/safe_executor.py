#!/usr/bin/env python3

from collections import deque

import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory


class SafeExecutor(Node):
    def __init__(self):
        super().__init__("safe_executor")

        # 默认接 Gazebo 里的 ros2_control 控制器。
        # 想回到昨天的 mock 执行器时，用参数覆盖为
        # /task/execute_trajectory 即可。
        self.declare_parameter(
            "action_name",
            "/joint_trajectory_controller/follow_joint_trajectory",
        )
        self.action_name = self.get_parameter("action_name").value

        # 队列只保存“还没开始执行”的动作块。
        # 默认只留 2 块，避免真机上积压太多过期动作。
        self.declare_parameter("max_queue_size", 2)
        self.max_queue_size = max(
            1,
            int(self.get_parameter("max_queue_size").value),
        )

        self.client = ActionClient(
            self,
            FollowJointTrajectory,
            self.action_name,
        )
        self.sub = self.create_subscription(
            JointTrajectory,
            "/policy/action_safe",
            self.on_safe_action,
            10,
        )

        self.pending = deque()
        self.busy = False
        self.queued_count = 0
        self.dropped_count = 0
        self.executed_count = 0

        self.get_logger().info(
            f"Waiting for action server: {self.action_name}"
        )
        self.client.wait_for_server()
        self.get_logger().info(
            f"Trajectory action server ready, "
            f"queue size={self.max_queue_size}"
        )

    def on_safe_action(self, msg):
        if not self.client.server_is_ready():
            self.get_logger().warn("action server not ready, drop action")
            return

        # 队列满时丢掉最旧的未执行块，保留较新的动作。
        # 当前块正在执行时不受影响，避免中途硬切轨迹。
        if len(self.pending) >= self.max_queue_size:
            dropped = self.pending.popleft()
            self.dropped_count += 1
            self.get_logger().warn(
                f"queue full, dropped oldest pending action "
                f"({len(dropped.points)} points)"
            )

        self.pending.append(msg)
        self.queued_count += 1
        self.get_logger().info(
            f"queued action: {len(msg.points)} points, "
            f"pending={len(self.pending)}, busy={self.busy}"
        )
        self.start_next()

    def start_next(self):
        if self.busy or not self.pending:
            return

        msg = self.pending.popleft()
        self.busy = True
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = msg

        self.get_logger().info(
            f"sending action: {len(msg.points)} points, "
            f"joints={list(msg.joint_names)}, "
            f"pending={len(self.pending)}"
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
            self.start_next()
            return

        self.get_logger().info("action goal accepted")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.on_result)

    def on_result(self, future):
        result = future.result().result
        self.executed_count += 1
        self.get_logger().info(
            f"action finished: code={result.error_code}, "
            f"message={result.error_string}"
        )
        self.get_logger().info(
            f"stats: queued={self.queued_count}, "
            f"executed={self.executed_count}, "
            f"dropped={self.dropped_count}"
        )
        self.busy = False
        self.start_next()


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
