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

        # 软限位必须和 URDF 的 joint limit 保持一致。
        # 这里只是软件保护，真机还必须有硬件急停和机械限位。
        # 默认保留 2 关节，Panda launch 会用参数覆盖成 7 关节。
        self.declare_parameter("joint_names", ["joint1", "joint2"])
        self.declare_parameter("lower_limits", [-2.967, -2.0])
        self.declare_parameter("upper_limits", [2.967, 2.0])
        self.declare_parameter("velocity_limits", [1.5, 1.5])

        names = list(self.get_parameter("joint_names").value)
        lower = list(self.get_parameter("lower_limits").value)
        upper = list(self.get_parameter("upper_limits").value)
        velocity = list(self.get_parameter("velocity_limits").value)

        if not (len(names) == len(lower) == len(upper) == len(velocity)):
            raise ValueError(
                "joint_names/lower_limits/upper_limits/velocity_limits "
                "must have the same length"
            )

        self.position_limits = {
            name: (float(lower[i]), float(upper[i]))
            for i, name in enumerate(names)
        }
        self.velocity_limits = {
            name: float(velocity[i])
            for i, name in enumerate(names)
        }
        self.get_logger().info(
            f"Safety joints: {names}"
        )

        # /joint_states 超过这个时间没更新，就认为状态过期。
        # 真机上绝不能拿旧关节位置继续做安全判断。
        self.declare_parameter("state_timeout", 0.5)
        self.state_timeout = max(
            0.05,
            float(self.get_parameter("state_timeout").value),
        )

        # 启动后给 /joint_states 一点时间上线，避免刚启动就误报过期。
        self.declare_parameter("startup_grace", 2.0)
        self.startup_grace = max(
            0.0,
            float(self.get_parameter("startup_grace").value),
        )
        self.node_start_time = self.get_clock().now()

        # 策略多久没有新动作就认为断流。
        # 做状态超时测试时可临时调大，避免 watchdog 先触发。
        self.declare_parameter("watchdog_timeout", 2.0)
        self.watchdog_timeout = max(
            0.5,
            float(self.get_parameter("watchdog_timeout").value),
        )

        self.current_positions = {}
        self.last_state_time = None
        self.state_warned = False
        self.last_time = self.get_clock().now()
        self.received = False
        self.watchdog_active = False
        self.watchdog = self.create_timer(0.5, self.check_watchdog)
        self.state_timer = self.create_timer(0.2, self.check_state)

        self.get_logger().info(
            f"Safety filter started, "
            f"state_timeout={self.state_timeout:.3f}s, "
            f"watchdog_timeout={self.watchdog_timeout:.3f}s, "
            f"startup_grace={self.startup_grace:.3f}s"
        )

    def publish_status(self, text):
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)

    def state_callback(self, msg):
        self.current_positions = {
            name: position
            for name, position in zip(msg.name, msg.position)
        }
        self.last_state_time = self.get_clock().now()

        if self.state_warned:
            self.state_warned = False
            self.get_logger().info("joint state restored")
            self.publish_status("state: ok")

    def state_is_fresh(self):
        if self.last_state_time is None:
            return False

        age = (
            self.get_clock().now() - self.last_state_time
        ).nanoseconds / 1e9
        return age <= self.state_timeout

    def check_state(self):
        if self.state_is_fresh():
            return

        # 启动阶段还没收到第一帧 /joint_states 时，先等待 grace。
        if self.last_state_time is None:
            elapsed = (
                self.get_clock().now() - self.node_start_time
            ).nanoseconds / 1e9
            if elapsed < self.startup_grace:
                return

        if not self.state_warned:
            self.state_warned = True
            self.get_logger().warn(
                f"joint state stale or missing "
                f"(timeout={self.state_timeout:.3f}s)"
            )
            self.publish_status("drop: stale state")

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
            self.publish_status("watchdog reset")

        # 状态过期时拒绝动作，避免拿旧位置做速度校验。
        if not self.state_is_fresh():
            self.publish_status("drop: stale state")
            if not self.state_warned:
                self.state_warned = True
                self.get_logger().warn(
                    "joint state stale, dropping action"
                )
            return

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

        # 超速时不改路径，只把整条轨迹的时间轴拉长。
        # 例如 x3.33 表示同样的路径用 3.33 倍时间走完。
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

        # 策略断流超过 2 秒就报警。
        # 注意：今天只报警，还没有自动发送“保持当前位置”的轨迹。
        if elapsed > self.watchdog_timeout and not self.watchdog_active:
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
