#!/usr/bin/env python3
"""安全停止体检。

流程：
1. 等 /joint_states 上线；
2. 发一条 5 秒慢轨迹；
3. 不再发新动作，让 watchdog 触发取消；
4. 记录停止前后的位置/速度，输出 CSV 和 SVG。

这个脚本只观察，不改安全层，也不使用 queue_probe。
"""

import csv
import time
from datetime import datetime
from html import escape
from pathlib import Path

import rclpy
from builtin_interfaces.msg import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class StopProbe(Node):
    def __init__(self):
        super().__init__("stop_probe")

        self.declare_parameter("duration", 5.0)
        self.declare_parameter("record_after", 3.0)
        self.declare_parameter("target1", 0.35)
        self.declare_parameter("target2", -0.25)
        default_output = Path(__file__).resolve().parents[1] / "output"
        self.declare_parameter("output_dir", str(default_output))

        self.duration = float(self.get_parameter("duration").value)
        self.record_after = float(self.get_parameter("record_after").value)
        self.target1 = float(self.get_parameter("target1").value)
        self.target2 = float(self.get_parameter("target2").value)
        self.output_dir = Path(self.get_parameter("output_dir").value)

        self.pub = self.create_publisher(
            JointTrajectory, "/policy/action_raw", 10
        )
        self.create_subscription(
            JointState, "/joint_states", self.on_state, 50
        )
        self.create_subscription(
            String, "/policy/safety_status", self.on_status, 10
        )

        self.samples = []
        self.statuses = []
        self.record_start = None
        self.first_state_time = None
        self.send_time = None
        self.stop_time = None
        self.stop_reason = ""
        self.finished = False
        self.written = False

        self.wall_start = time.monotonic()
        self.timer = self.create_timer(0.1, self.tick)
        self.get_logger().info(
            "stop_probe ready: waiting for /joint_states, "
            f"duration={self.duration:.1f}s, "
            f"record_after={self.record_after:.1f}s"
        )

    def now(self):
        return time.monotonic()

    def on_state(self, msg):
        if self.record_start is None:
            self.record_start = self.now()
            self.get_logger().info("recording /joint_states")

        if self.first_state_time is None:
            self.first_state_time = self.now()

        positions = dict(zip(msg.name, msg.position))
        velocities = dict(zip(msg.name, msg.velocity))
        if "joint1" not in positions or "joint2" not in positions:
            return

        t = self.now() - self.record_start
        self.samples.append(
            {
                "t": t,
                "j1_pos": float(positions["joint1"]),
                "j1_vel": float(velocities.get("joint1", 0.0)),
                "j2_pos": float(positions["joint2"]),
                "j2_vel": float(velocities.get("joint2", 0.0)),
            }
        )

    def on_status(self, msg):
        if self.record_start is None:
            return
        t = self.now() - self.record_start
        self.statuses.append((t, msg.data))

        # 第一次出现 watchdog / stale 就是安全层开始停止的时刻。
        if self.stop_time is None and (
            "watchdog" in msg.data or "stale" in msg.data
        ):
            self.stop_time = t
            self.stop_reason = msg.data
            self.get_logger().warn(f"stop event: {msg.data}")

    def send_action(self):
        traj = JointTrajectory()
        traj.header.stamp = self.get_clock().now().to_msg()
        traj.joint_names = ["joint1", "joint2"]

        point = JointTrajectoryPoint()
        point.positions = [self.target1, self.target2]
        point.velocities = [0.0, 0.0]
        sec = int(self.duration)
        nanosec = int(round((self.duration - sec) * 1e9))
        if nanosec >= 1_000_000_000:
            sec += 1
            nanosec = 0
        point.time_from_start = Duration(sec=sec, nanosec=nanosec)
        traj.points = [point]

        self.pub.publish(traj)
        self.send_time = self.now()
        self.get_logger().info(
            f"sent one trajectory: duration={self.duration:.1f}s, "
            f"target=[{self.target1:.2f}, {self.target2:.2f}]"
        )

    def tick(self):
        if self.finished:
            return

        elapsed = self.now() - self.wall_start
        if elapsed > 20.0 and self.first_state_time is None:
            self.get_logger().error("no /joint_states; is the launch running?")
            self.finish()
            return

        if self.first_state_time is None:
            return

        if self.send_time is None:
            # 给 safe_executor / 安全层一点时间完成订阅。
            if self.pub.get_subscription_count() == 0:
                if self.now() - self.first_state_time > 10.0:
                    self.get_logger().error(
                        "no subscriber on /policy/action_raw; "
                        "is safety_filter running?"
                    )
                    self.finish()
                return
            if self.now() - self.first_state_time >= 1.5:
                self.send_action()
            return

        # 发完动作后继续录一段，确保覆盖 watchdog 取消和停止后的保持。
        if self.now() - self.send_time >= self.duration + self.record_after:
            self.finish()

    def finish(self):
        if self.finished:
            return
        self.finished = True
        self.write_outputs()

    def mean_velocity(self, key, start, end):
        values = [
            s[key]
            for s in self.samples
            if start <= s["t"] <= end
        ]
        if not values:
            return None
        return sum(values) / len(values)

    def write_outputs(self):
        if self.written:
            return
        self.written = True

        if not self.samples:
            self.get_logger().error("no samples recorded")
            return

        self.output_dir.mkdir(parents=True, exist_ok=True)
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        csv_path = self.output_dir / f"stop_probe_{stamp}.csv"
        svg_path = self.output_dir / f"stop_probe_{stamp}.svg"

        with csv_path.open("w", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(
                f,
                fieldnames=["t", "j1_pos", "j1_vel", "j2_pos", "j2_vel"],
            )
            writer.writeheader()
            writer.writerows(self.samples)

        if self.stop_time is None:
            stop_t = self.samples[-1]["t"]
            reason = "no stop event"
        else:
            stop_t = self.stop_time
            reason = self.stop_reason

        self.write_svg(svg_path, stop_t, reason)
        self.print_summary(stop_t, reason)

        self.get_logger().info(f"CSV: {csv_path}")
        self.get_logger().info(f"SVG: {svg_path}")
        self.get_logger().info(f"safety statuses: {self.statuses[-5:]}")

    def print_summary(self, stop_t, reason):
        before_start = max(0.0, stop_t - 0.4)
        after_start = stop_t + 0.2
        after_end = stop_t + 0.8

        print("")
        print("=== stop probe ===")
        print(f"samples      : {len(self.samples)}")
        print(f"stop time    : {stop_t:.3f} s")
        print(f"stop reason  : {reason}")
        for joint, key in (("joint1", "j1_vel"), ("joint2", "j2_vel")):
            before = self.mean_velocity(key, before_start, stop_t)
            after = self.mean_velocity(key, after_start, after_end)
            if before is None or after is None:
                print(f"{joint}      : not enough samples")
                continue
            print(
                f"{joint}      : v_before={before:+.4f} rad/s, "
                f"v_after={after:+.4f} rad/s, "
                f"drop={abs(before - after):.4f} rad/s"
            )
        print("==================")
        print("")

    def write_svg(self, path, stop_t, reason):
        width = 960
        height = 680
        left = 85
        right = 35
        top = 55
        bottom = 45
        panel_h = 230
        gap = 80

        t_max = max(s["t"] for s in self.samples)
        if t_max <= 0.0:
            t_max = 1.0

        pos_values = [
            s["j1_pos"] for s in self.samples
        ] + [
            s["j2_pos"] for s in self.samples
        ]
        vel_values = [
            s["j1_vel"] for s in self.samples
        ] + [
            s["j2_vel"] for s in self.samples
        ]

        def y_range(values):
            lo = min(values)
            hi = max(values)
            if hi - lo < 1e-9:
                hi = lo + 1.0
            pad = (hi - lo) * 0.12
            return lo - pad, hi + pad

        pos_lo, pos_hi = y_range(pos_values)
        vel_lo, vel_hi = y_range(vel_values)

        def sx(t):
            return left + (t / t_max) * (width - left - right)

        def sy(value, lo, hi, panel_top):
            return panel_top + (hi - value) / (hi - lo) * panel_h

        def polyline(key, lo, hi, panel_top):
            points = " ".join(
                f"{sx(s['t']):.2f},{sy(s[key], lo, hi, panel_top):.2f}"
                for s in self.samples
            )
            return points

        def grid(panel_top, lo, hi, title):
            parts = [
                f'<text x="{left}" y="{panel_top - 12}" '
                f'font-size="16" fill="#111827">{escape(title)}</text>'
            ]
            for i in range(5):
                value = lo + (hi - lo) * i / 4
                y = sy(value, lo, hi, panel_top)
                parts.append(
                    f'<line x1="{left}" y1="{y:.2f}" '
                    f'x2="{width - right}" y2="{y:.2f}" '
                    f'stroke="#e5e7eb" stroke-width="1"/>'
                )
                parts.append(
                    f'<text x="{left - 10}" y="{y + 4:.2f}" '
                    f'text-anchor="end" font-size="11" fill="#6b7280">'
                    f"{value:.3f}</text>"
                )
            for i in range(6):
                t = t_max * i / 5
                x = sx(t)
                parts.append(
                    f'<line x1="{x:.2f}" y1="{panel_top}" '
                    f'x2="{x:.2f}" y2="{panel_top + panel_h}" '
                    f'stroke="#f3f4f6" stroke-width="1"/>'
                )
                parts.append(
                    f'<text x="{x:.2f}" y="{panel_top + panel_h + 20}" '
                    f'text-anchor="middle" font-size="11" fill="#6b7280">'
                    f"{t:.2f}s</text>"
                )
            parts.append(
                f'<rect x="{left}" y="{panel_top}" width="{width - left - right}" '
                f'height="{panel_h}" fill="none" stroke="#9ca3af"/>'
            )
            return parts

        svg = [
            f'<svg xmlns="http://www.w3.org/2000/svg" '
            f'width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
            '<rect width="100%" height="100%" fill="#ffffff"/>',
            f'<text x="{left}" y="28" font-size="20" fill="#111827" '
            f'font-family="sans-serif">Safety stop profile</text>',
            f'<text x="{left}" y="46" font-size="12" fill="#6b7280">'
            f'{escape(reason)}</text>',
        ]

        pos_top = top + 30
        vel_top = pos_top + panel_h + gap
        svg.extend(grid(pos_top, pos_lo, pos_hi, "joint position (rad)"))
        svg.extend(grid(vel_top, vel_lo, vel_hi, "joint velocity (rad/s)"))

        svg.append(
            f'<polyline points="{polyline("j1_pos", pos_lo, pos_hi, pos_top)}" '
            f'fill="none" stroke="#2563eb" stroke-width="2"/>'
        )
        svg.append(
            f'<polyline points="{polyline("j2_pos", pos_lo, pos_hi, pos_top)}" '
            f'fill="none" stroke="#dc2626" stroke-width="2"/>'
        )
        svg.append(
            f'<polyline points="{polyline("j1_vel", vel_lo, vel_hi, vel_top)}" '
            f'fill="none" stroke="#2563eb" stroke-width="2"/>'
        )
        svg.append(
            f'<polyline points="{polyline("j2_vel", vel_lo, vel_hi, vel_top)}" '
            f'fill="none" stroke="#dc2626" stroke-width="2"/>'
        )

        stop_x = sx(min(max(stop_t, 0.0), t_max))
        svg.append(
            f'<line x1="{stop_x:.2f}" y1="{pos_top}" '
            f'x2="{stop_x:.2f}" y2="{vel_top + panel_h}" '
            f'stroke="#111827" stroke-width="2" stroke-dasharray="6 5"/>'
        )
        svg.append(
            f'<text x="{stop_x + 6:.2f}" y="{pos_top + 16}" '
            f'font-size="12" fill="#111827">stop</text>'
        )

        legend_y = pos_top - 28
        svg.append(
            f'<line x1="{width - 220}" y1="{legend_y}" '
            f'x2="{width - 190}" y2="{legend_y}" '
            f'stroke="#2563eb" stroke-width="3"/>'
        )
        svg.append(
            f'<text x="{width - 184}" y="{legend_y + 4}" '
            f'font-size="12" fill="#374151">joint1</text>'
        )
        svg.append(
            f'<line x1="{width - 120}" y1="{legend_y}" '
            f'x2="{width - 90}" y2="{legend_y}" '
            f'stroke="#dc2626" stroke-width="3"/>'
        )
        svg.append(
            f'<text x="{width - 84}" y="{legend_y + 4}" '
            f'font-size="12" fill="#374151">joint2</text>'
        )

        svg.append("</svg>")
        path.write_text("\n".join(svg), encoding="utf-8")


def main(args=None):
    rclpy.init(args=args)
    node = StopProbe()
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if not node.finished:
            node.finish()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
