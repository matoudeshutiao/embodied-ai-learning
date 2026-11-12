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

        # 默认保留 2 关节；Panda launch 用参数覆盖成 7 关节。
        self.declare_parameter("joint_names", ["joint1", "joint2"])
        self.declare_parameter("home_positions", [0.0, 0.0])
        self.declare_parameter("amplitudes", [0.3, 0.2])
        self.declare_parameter(
            "phase_offsets",
            [0.0, -math.pi / 2.0],
        )

        self.joint_names = list(
            self.get_parameter("joint_names").value
        )
        self.home_positions = list(
            self.get_parameter("home_positions").value
        )
        self.amplitudes = list(
            self.get_parameter("amplitudes").value
        )
        self.phase_offsets = list(
            self.get_parameter("phase_offsets").value
        )

        count = len(self.joint_names)
        if not (
            len(self.home_positions) == count
            and len(self.amplitudes) == count
            and len(self.phase_offsets) == count
        ):
            raise ValueError(
                "joint_names/home_positions/amplitudes/phase_offsets "
                "must have the same length"
            )

        self.get_logger().info(
            f"Mock policy started, joints={self.joint_names}"
        )

    def tick(self):
        traj = JointTrajectory()
        traj.header.stamp = self.get_clock().now().to_msg()
        traj.joint_names = self.joint_names

        # 一次策略推理输出未来 1 秒内的 5 个动作点。
        for i in range(5):
            point = JointTrajectoryPoint()
            t = (i + 1) * 0.2

            point.positions = [
                self.home_positions[index]
                + self.amplitudes[index]
                * math.sin(
                    self.t + t + self.phase_offsets[index]
                )
                for index in range(len(self.joint_names))
            ]
            point.velocities = [0.0] * len(self.joint_names)
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
