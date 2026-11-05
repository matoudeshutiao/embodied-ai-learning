#!/usr/bin/env python3

import math

import rclpy
from builtin_interfaces.msg import Duration
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class MockPolicy(Node):
    def __init__(self):
        super().__init__("mock_policy")

        self.pub = self.create_publisher(
            JointTrajectory,
            "/policy/action_raw",
            10,
        )
        # 每 1.2 s 发一个 1.0 s 的动作块，留 0.2 s 给控制器收尾。
        # 这样轨迹块之间不会严重重叠，Gazebo 里运动更连续。
        self.timer = self.create_timer(1.2, self.tick)
        self.t = 0.0

        self.joint_names = ["joint1", "joint2"]
        self.get_logger().info("Mock policy started")

    def tick(self):
        traj = JointTrajectory()
        traj.header.stamp = self.get_clock().now().to_msg()
        traj.joint_names = self.joint_names

        # 一次策略推理输出未来 1 秒内的 5 个动作点。
        for i in range(5):
            point = JointTrajectoryPoint()
            t = (i + 1) * 0.2

            point.positions = [
                0.3 * math.sin(self.t + t),
                -0.2 * math.cos(self.t + t),
            ]
            point.velocities = [0.0, 0.0]
            point.time_from_start = Duration(
                sec=int(t),
                nanosec=int((t % 1.0) * 1e9),
            )
            traj.points.append(point)

        self.pub.publish(traj)
        self.get_logger().info(
            f"published action chunk: {len(traj.points)} points"
        )
        self.t += 1.0


def main(args=None):
    rclpy.init(args=args)
    node = MockPolicy()
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
