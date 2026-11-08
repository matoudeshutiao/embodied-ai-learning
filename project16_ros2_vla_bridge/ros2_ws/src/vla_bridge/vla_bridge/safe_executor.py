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

        # 限制“总在途”数量：正在执行 + 等待执行。
        # 默认 2 = 1 条 active + 1 条 pending，优先保证低延迟。
        # 想保留更多计划可设为 3，但真机会增加旧动作风险。
        self.declare_parameter("max_in_flight", 2)
        self.max_in_flight = max(
            1,
            int(self.get_parameter("max_in_flight").value),
        )

        # 动作在队列里最多允许等多久。
        # 超过这个年龄就丢弃，避免执行过期计划。
        self.declare_parameter("max_action_age", 0.5)
        self.max_action_age = max(
            0.0,
            float(self.get_parameter("max_action_age").value),
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
            f"max_in_flight={self.max_in_flight}, "
            f"max_action_age={self.max_action_age:.3f}s"
        )

    def in_flight(self):
        """总在途 = 正在执行 + 等待执行。"""
        return len(self.pending) + (1 if self.busy else 0)

    @staticmethod
    def duration_to_sec(duration):
        return float(duration.sec) + float(duration.nanosec) * 1e-9

    def prepare_for_execution(self, msg):
        """检查动作年龄；安全时重新打时间戳，避免轨迹被压缩。"""
        if not msg.points:
            self.get_logger().warn("empty trajectory, drop action")
            return False

        now = self.get_clock().now()
        stamp = msg.header.stamp
        stamp_sec = float(stamp.sec) + float(stamp.nanosec) * 1e-9
        now_sec = float(now.nanoseconds) * 1e-9
        age = max(0.0, now_sec - stamp_sec)
        total_duration = self.duration_to_sec(
            msg.points[-1].time_from_start
        )

        # 已经超过轨迹总时长，剩余时间 <= 0，不能执行。
        if total_duration <= 0.0 or age >= total_duration:
            self.get_logger().warn(
                f"expired action: age={age:.3f}s, "
                f"duration={total_duration:.3f}s, drop"
            )
            return False

        # 还没完全过期，但已经太旧，也按过期处理。
        if age > self.max_action_age:
            self.get_logger().warn(
                f"stale action: age={age:.3f}s > "
                f"max_action_age={self.max_action_age:.3f}s, drop"
            )
            return False

        # 关键：把轨迹起点改成“现在”，让控制器执行完整时长。
        # 不重新打时间戳，控制器会以为队列等待已经消耗了轨迹时间。
        msg.header.stamp = now.to_msg()
        self.get_logger().info(
            f"time aligned: age={age:.3f}s, "
            f"duration={total_duration:.3f}s"
        )
        return True

    def on_safe_action(self, msg):
        if not self.client.server_is_ready():
            self.get_logger().warn("action server not ready, drop action")
            return

        # 手动测试消息可能没有时间戳；用接收时间作为起点。
        if msg.header.stamp.sec == 0 and msg.header.stamp.nanosec == 0:
            msg.header.stamp = self.get_clock().now().to_msg()
            self.get_logger().warn(
                "action has zero timestamp, using arrival time"
            )

        # 到达总在途上限时，优先丢掉最旧的“未执行”块。
        # 当前正在执行的 active 永远不在这里被取消。
        if self.in_flight() >= self.max_in_flight:
            if self.pending:
                dropped = self.pending.popleft()
                self.dropped_count += 1
                self.get_logger().warn(
                    f"max_in_flight reached, drop oldest pending action "
                    f"({len(dropped.points)} points)"
                )
            else:
                # 只有 active、没有 pending 可丢，只能丢弃新动作。
                # max_in_flight=1 时会走到这里。
                self.dropped_count += 1
                self.get_logger().warn(
                    "max_in_flight reached with no pending action, "
                    "drop newest action"
                )
                return

        self.pending.append(msg)
        self.queued_count += 1
        self.get_logger().info(
            f"queued action: {len(msg.points)} points, "
            f"in_flight={self.in_flight()}, "
            f"pending={len(self.pending)}, busy={self.busy}"
        )
        self.start_next()

    def start_next(self):
        if self.busy:
            return

        # 跳过已经过期/太旧的动作，只发送仍可安全执行的动作。
        while self.pending:
            msg = self.pending.popleft()
            if not self.prepare_for_execution(msg):
                self.dropped_count += 1
                continue

            self.busy = True
            goal = FollowJointTrajectory.Goal()
            goal.trajectory = msg

            self.get_logger().info(
                f"sending action: {len(msg.points)} points, "
                f"joints={list(msg.joint_names)}, "
                f"in_flight={self.in_flight()}, "
                f"pending={len(self.pending)}"
            )

            future = self.client.send_goal_async(
                goal,
                feedback_callback=self.on_feedback,
            )
            future.add_done_callback(self.on_goal_response)
            return

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
