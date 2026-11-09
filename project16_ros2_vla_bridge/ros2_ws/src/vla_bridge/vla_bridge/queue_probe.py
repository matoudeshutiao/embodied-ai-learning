#!/usr/bin/env python3

import rclpy
from builtin_interfaces.msg import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class QueueProbe(Node):
    """只用一个发布者，向安全层连续发送不同目标的短轨迹。"""

    def __init__(self):
        super().__init__("queue_probe")

        self.declare_parameter("count", 8)
        self.declare_parameter("interval", 0.25)
        self.declare_parameter("duration", 0.6)

        self.count = int(self.get_parameter("count").value)
        self.interval = float(self.get_parameter("interval").value)
        self.duration = float(self.get_parameter("duration").value)

        self.pub = self.create_publisher(
            JointTrajectory,
            "/policy/action_raw",
            10,
        )

        # 每个目标都不同，避免“到达后原地保持”被误认为卡住。
        self.targets = [
            (0.4, -0.3),
            (0.0, 0.2),
            (-0.4, 0.3),
            (0.2, -0.2),
            (-0.2, 0.1),
            (0.3, 0.0),
            (-0.3, -0.1),
            (0.1, 0.15),
        ]

        self.index = 0
        self.finished = False
        self.timer = self.create_timer(self.interval, self.tick)
        self.get_logger().info(
            f"queue probe started: count={self.count}, "
            f"interval={self.interval}s, duration={self.duration}s"
        )

    def tick(self):
        if self.index >= self.count:
            self.timer.cancel()
            self.finished = True
            self.get_logger().info("queue probe finished")
            return

        joint1, joint2 = self.targets[self.index % len(self.targets)]

        traj = JointTrajectory()
        traj.header.stamp = self.get_clock().now().to_msg()
        traj.joint_names = ["joint1", "joint2"]

        point = JointTrajectoryPoint()
        point.positions = [joint1, joint2]
        point.velocities = [0.0, 0.0]
        point.time_from_start = Duration(
            sec=int(self.duration),
            nanosec=int((self.duration % 1.0) * 1e9),
        )
        traj.points.append(point)

        self.pub.publish(traj)
        self.get_logger().info(
            f"probe #{self.index + 1}: "
            f"target=({joint1:.2f}, {joint2:.2f})"
        )
        self.index += 1


def main(args=None):
    rclpy.init(args=args)
    node = QueueProbe()
    try:
        # 发完 count 条后主动退出，避免依赖外部 timeout 杀进程。
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
