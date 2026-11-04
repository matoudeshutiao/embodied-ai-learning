#!/usr/bin/env python3

import rclpy
from builtin_interfaces.msg import Duration
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class SafetyFilter(Node):
    def __init__(self):
        super().__init__("safety_filter")

        self.sub = self.create_subscription(
            JointTrajectory,
            "/policy/action_raw",
            self.callback,
            10,
        )
        self.safe_pub = self.create_publisher(
            JointTrajectory,
            "/policy/action_safe",
            10,
        )
        self.status_pub = self.create_publisher(
            String,
            "/policy/safety_status",
            10,
        )
        self.state_sub = self.create_subscription(
            JointState,
            "/joint_states",
            self.state_callback,
            10,
        )

        self.position_limits = {
            "joint1": (-2.967, 2.967),
            "joint2": (-2.0, 2.0),
        }
        self.velocity_limits = {
            "joint1": 1.5,
            "joint2": 1.5,
        }

        self.current_positions = {}
        self.last_time = self.get_clock().now()
        self.received = False
        self.watchdog_active = False
        self.watchdog = self.create_timer(0.5, self.check_watchdog)

        self.get_logger().info("Safety filter started")

    def publish_status(self, text):
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)

    def state_callback(self, msg):
        self.current_positions = {
            name: position
            for name, position in zip(msg.name, msg.position)
        }

    def duration_to_sec(self, duration):
        return float(duration.sec) + float(duration.nanosec) * 1e-9

    def sec_to_duration(self, seconds):
        sec = int(seconds)
        nanosec = int((seconds - sec) * 1e9)
        return Duration(sec=sec, nanosec=nanosec)

    def callback(self, msg):
        self.last_time = self.get_clock().now()
        self.received = True

        if self.watchdog_active:
            self.watchdog_active = False
            self.get_logger().info("watchdog reset")

        unknown = [
            name for name in msg.joint_names
            if name not in self.position_limits
        ]
        if unknown:
            self.get_logger().warn(f"unknown joints: {unknown}")
            self.publish_status("drop: unknown joint")
            return

        if not msg.points:
            self.publish_status("drop: empty trajectory")
            return

        safe = JointTrajectory()
        safe.header = msg.header
        safe.joint_names = list(msg.joint_names)

        clamped_count = 0
        points = []

        for point in msg.points:
            if len(point.positions) != len(msg.joint_names):
                self.get_logger().warn("position count mismatch")
                self.publish_status("drop: size mismatch")
                return

            safe_point = JointTrajectoryPoint()
            safe_point.time_from_start = point.time_from_start

            positions = []
            for name, position in zip(msg.joint_names, point.positions):
                lower, upper = self.position_limits[name]
                clamped = min(max(position, lower), upper)
                if abs(clamped - position) > 1e-9:
                    clamped_count += 1
                positions.append(clamped)

            safe_point.positions = positions
            points.append(safe_point)

        # Use latest joint state as the trajectory start for velocity checks.
        previous_positions = [
            self.current_positions.get(name, 0.0)
            for name in safe.joint_names
        ]
        previous_time = 0.0
        max_ratio = 1.0

        for point in points:
            current_time = self.duration_to_sec(point.time_from_start)
            dt = current_time - previous_time
            if dt <= 0.0:
                dt = 1e-3

            for index, name in enumerate(safe.joint_names):
                velocity = abs(
                    point.positions[index] - previous_positions[index]
                ) / dt
                ratio = velocity / self.velocity_limits[name]
                if ratio > max_ratio:
                    max_ratio = ratio

            previous_positions = list(point.positions)
            previous_time = current_time

        if max_ratio > 1.0:
            for point in points:
                scaled = (
                    self.duration_to_sec(point.time_from_start)
                    * max_ratio
                )
                point.time_from_start = self.sec_to_duration(scaled)

        safe.points = points
        self.safe_pub.publish(safe)

        status = f"ok: {len(safe.points)} points"
        if clamped_count:
            status += f", clamped {clamped_count}"
        if max_ratio > 1.0:
            status += f", time scaled x{max_ratio:.2f}"

        self.publish_status(status)

    def check_watchdog(self):
        if not self.received:
            return

        elapsed = (
            self.get_clock().now() - self.last_time
        ).nanoseconds / 1e9

        if elapsed > 2.0 and not self.watchdog_active:
            self.watchdog_active = True
            self.get_logger().warn("watchdog: no new action")
            self.publish_status("watchdog: no new action")


def main(args=None):
    rclpy.init(args=args)
    node = SafetyFilter()
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
