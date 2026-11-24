#!/usr/bin/env python3
"""腕部相机 PBVS + 抓放。

0.30 m 闭环修正 x/y；
下降到 0.20 m 只做一致性检查，不用 0.20 m 的偏差去二次修正；
最后复用 PandaPick.grasp_from_pose 完成抓取、抬升和放置。

用法：
  python3 scripts/wrist_pick.py
  python3 scripts/wrist_pick.py --approach-offset 0.30 --verify-offset 0.20
"""

import argparse
import math
import os
import time
from datetime import datetime

import cv2
import message_filters
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.parameter import Parameter
from rclpy.time import Time as RosTime
from sensor_msgs.msg import CameraInfo, Image

from vla_bridge.panda_pick import PandaPick
from wrist_vision import (
    CUBE_HALF,
    depth_to_meters,
    draw_debug,
    estimate_cube,
    load_calibration,
)


class WristPick(PandaPick):
    def __init__(self, args):
        super().__init__()
        self.args = args
        self.bridge = CvBridge()
        self.K = None
        self.latest = None
        self.frame_count = 0
        self.T_link8_cam, self.calib = load_calibration()

        self.create_subscription(
            CameraInfo,
            "/wrist_camera/camera_info",
            self.on_camera_info,
            10,
        )
        image_sub = message_filters.Subscriber(
            self, Image, "/wrist_camera/image_raw"
        )
        depth_sub = message_filters.Subscriber(
            self, Image, "/wrist_camera/depth/image_raw"
        )
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [image_sub, depth_sub], queue_size=10, slop=0.1
        )
        self.sync.registerCallback(self.on_images)

        self.outdir = os.path.abspath(
            os.path.join(
                os.path.dirname(__file__), "..", "output"
            )
        )
        os.makedirs(self.outdir, exist_ok=True)
        self.stamp = datetime.now().strftime("%Y%m%d_%H%M%S")

    def on_camera_info(self, msg):
        if len(msg.k) >= 9 and msg.k[0] > 0.0:
            self.K = (
                float(msg.k[0]),
                float(msg.k[4]),
                float(msg.k[2]),
                float(msg.k[5]),
            )

    def on_images(self, image_msg, depth_msg):
        try:
            image = self.bridge.imgmsg_to_cv2(
                image_msg, desired_encoding="bgr8"
            )
            depth = self.bridge.imgmsg_to_cv2(
                depth_msg, desired_encoding="passthrough"
            )
        except Exception as exc:
            self.get_logger().warn(f"cv_bridge failed: {exc}")
            return
        self.latest = (image.copy(), depth_to_meters(depth))
        self.frame_count += 1

    def wait_ready(self, timeout=15.0):
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if (
                self.K is not None
                and self.latest is not None
                and self.cube_pose is not None
            ):
                return True
        return False

    def capture(self, timeout=3.0):
        start = self.frame_count
        deadline = time.monotonic() + timeout
        while (
            rclpy.ok()
            and self.frame_count == start
            and time.monotonic() < deadline
        ):
            rclpy.spin_once(self, timeout_sec=0.05)
        if self.latest is None or self.frame_count == start:
            return None
        return self.latest

    def lookup_link8(self):
        transform = self.tf_buffer.lookup_transform(
            "panda_link0", "panda_link8", RosTime()
        )
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        x, y, z, w = (
            rotation.x, rotation.y, rotation.z, rotation.w
        )
        norm = x * x + y * y + z * z + w * w
        s = 2.0 / norm
        R = np.array(
            [
                [1 - s * (y * y + z * z), s * (x * y - z * w),
                 s * (x * z + y * w)],
                [s * (x * y + z * w), 1 - s * (x * x + z * z),
                 s * (y * z - x * w)],
                [s * (x * z - y * w), s * (y * z + x * w),
                 1 - s * (x * x + y * y)],
            ]
        )
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = [translation.x, translation.y, translation.z]
        return T

    def wait_tf_ready(self, timeout=10.0):
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            try:
                self.lookup_link8()
                return True
            except Exception:
                self.settle(0.2)
        return False

    def quality(self, result):
        return (
            result["plane_inliers"]
            >= 0.85 * result["red_points"]
            and result["red_points"] >= 3000
            and result["normal"][2] >= 0.98
            and 0.36 <= result["cube_center"][2] <= 0.39
        )

    def capture_estimate(self):
        frame = self.capture()
        if frame is None:
            return None, None, "no fresh frame", None
        image, depth = frame
        try:
            T_base_link8 = self.lookup_link8()
        except Exception as exc:
            return image, None, f"TF failed: {exc}", None
        T_base_cam = T_base_link8 @ self.T_link8_cam
        result, message = estimate_cube(
            image, depth, self.K, T_base_cam
        )
        return image, result, message, T_base_link8

    def save_debug(self, image, result, lines, suffix):
        if image is None or result is None:
            return
        debug = draw_debug(image, result, lines)
        path = os.path.join(
            self.outdir, f"wrist_pick_{self.stamp}_{suffix}.png"
        )
        cv2.imwrite(path, debug)
        self.get_logger().info(f"debug: {path}")

    def servo_to_cube(self):
        if not self.wait_ready():
            self.get_logger().error(
                "没有收到 wrist_camera / model_states"
            )
            return None
        if not self.wait_tf_ready():
            self.get_logger().error("TF 未就绪")
            return None

        truth = self.get_cube_pose()
        if truth is None:
            return None
        start_z = (
            truth[2] + CUBE_HALF + self.args.approach_offset
        )
        if not self.move_to(
            truth[0], truth[1], start_z, "wrist start"
        ):
            return None
        self.settle(1.0)

        for iteration in range(self.args.max_iter):
            image, result, message, T_base_link8 = (
                self.capture_estimate()
            )
            if result is None:
                self.get_logger().error(
                    f"iter {iteration}: detect failed: {message}"
                )
                return None

            red = result["red_points"]
            inliers = result["plane_inliers"]
            ratio = inliers / max(1, red)
            if not self.quality(result):
                self.get_logger().error(
                    f"iter {iteration}: low-quality detection "
                    f"(red={red}, inliers={inliers}, "
                    f"ratio={ratio:.3f}, "
                    f"normal_z={result['normal'][2]:.3f}), "
                    "stop"
                )
                return None

            top = result["top_center"]
            current = T_base_link8[:3, 3]
            desired = np.array(
                [top[0], top[1], top[2] + self.args.approach_offset]
            )
            error = desired - current
            err_norm = float(np.linalg.norm(error))
            self.get_logger().info(
                f"iter {iteration}: "
                f"cube=({result['cube_center'][0]:.4f},"
                f"{result['cube_center'][1]:.4f},"
                f"{result['cube_center'][2]:.4f}) "
                f"err={err_norm * 1000:.2f} mm "
                f"red={red} inliers={inliers} "
                f"ratio={ratio:.3f}"
            )

            if err_norm < self.args.tol_mm / 1000.0:
                target_top = result["top_center"]
                lines = [
                    "wrist servo converged",
                    f"err={err_norm * 1000:.2f} mm",
                    f"inliers={inliers}",
                ]
                self.save_debug(image, result, lines, "030")
                return result

            target = current + self.args.alpha * error
            if not self.move_to(
                float(target[0]),
                float(target[1]),
                float(target[2]),
                f"wrist servo {iteration}",
            ):
                self.get_logger().warn(
                    "MoveIt failed, retry +5 mm z"
                )
                if not self.move_to(
                    float(target[0]),
                    float(target[1]),
                    float(target[2]) + 0.005,
                    f"wrist servo retry {iteration}",
                ):
                    self.get_logger().error(
                        "MoveIt failed again"
                    )
                    return None
            self.settle(1.0)

        self.get_logger().error(
            f"没有在 {self.args.max_iter} 次内收敛"
        )
        return None

    def verify_at_020(self, top30):
        z20 = top30[2] + self.args.verify_offset
        if not self.move_cartesian_linear(
            top30[0], top30[1], z20, "wrist descend verify"
        ):
            return False
        self.settle(0.3)

        image, result, message, _ = self.capture_estimate()
        if result is None:
            self.get_logger().warn(
                f"0.20 m detect failed: {message}; "
                "blind continue with 0.30 m target"
            )
            return True
        if not self.quality(result):
            red = result["red_points"]
            inliers = result["plane_inliers"]
            ratio = inliers / max(1, red)
            normal_z = result["normal"][2]
            cube_z = result["cube_center"][2]
            self.get_logger().warn(
                "0.20 m low-quality detection: "
                f"red={red}, inliers={inliers}, "
                f"ratio={ratio:.3f}, normal_z={normal_z:.3f}, "
                f"cube_z={cube_z:.4f}; "
                "blind continue with 0.30 m target"
            )
            lines = [
                "0.20 m low quality",
                f"red={red} inliers={inliers}",
                f"ratio={ratio:.3f}",
                f"normal_z={normal_z:.3f}",
                f"cube_z={cube_z:.4f}",
            ]
            self.save_debug(image, result, lines, "020_low")
            return True

        top20 = result["top_center"]
        diff = math.hypot(
            top20[0] - top30[0], top20[1] - top30[1]
        )
        self.get_logger().info(
            f"0.20 m verify: top20=({top20[0]:.4f},"
            f"{top20[1]:.4f},{top20[2]:.4f}) "
            f"diff_xy={diff * 1000:.2f} mm "
            f"inliers={result['plane_inliers']}"
        )
        lines = [
            "wrist verify 0.20 m",
            f"diff_xy={diff * 1000:.2f} mm",
            f"inliers={result['plane_inliers']}",
        ]
        self.save_debug(image, result, lines, "020")

        if diff > self.args.max_xy_diff_mm / 1000.0:
            self.get_logger().error(
                f"0.20 m 和 0.30 m 的 x/y 差 {diff * 1000:.1f} mm "
                f"超过 {self.args.max_xy_diff_mm:.1f} mm，中止"
            )
            return False
        return True

    def run_wrist(self):
        if not self.move_client.wait_for_server(timeout_sec=30.0):
            self.get_logger().error("/move_action not ready")
            return 1
        if not self.arm_client.wait_for_server(timeout_sec=30.0):
            self.get_logger().error("arm controller action not ready")
            return 1

        # 夹爪先张开，避免 0.20 m 视角里手指挡住方块。
        self.effort(
            self.gripper_effort, 0.6, "open before wrist approach"
        )
        self.settle(0.3)
        self.get_logger().info(
            f"fingers before approach: {self.finger_positions}"
        )

        result30 = self.servo_to_cube()
        if result30 is None:
            return 1
        top30 = result30["top_center"]
        cube30 = result30["cube_center"]

        if not self.verify_at_020(top30):
            return 1
        if self.args.approach_only:
            self.get_logger().info(
                "approach-only: stopped at 0.20 m"
            )
            return 0

        ok = self.grasp_from_pose(
            float(cube30[0]),
            float(cube30[1]),
            float(cube30[2]),
            label="wrist",
            skip_pre_grasp=True,
            scene_settle=0.3,
        )
        self.get_logger().info(f"wrist pick success={ok}")
        return 0 if ok else 1


def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--approach-offset", type=float, default=0.30)
    parser.add_argument("--verify-offset", type=float, default=0.20)
    parser.add_argument("--alpha", type=float, default=0.7)
    parser.add_argument("--tol-mm", type=float, default=2.5)
    parser.add_argument("--max-iter", type=int, default=5)
    parser.add_argument("--max-xy-diff-mm", type=float, default=8.0)
    parser.add_argument("--approach-only", action="store_true")
    parser.add_argument("--place-x", type=float, default=0.40)
    parser.add_argument("--place-y", type=float, default=0.08)
    parser.add_argument("--place-z", type=float, default=0.375)
    parsed = parser.parse_args(args)

    rclpy.init(args=[])
    node = WristPick(parsed)
    node.set_parameters(
        [
            Parameter("place_x", value=parsed.place_x),
            Parameter("place_y", value=parsed.place_y),
            Parameter("place_z", value=parsed.place_z),
        ]
    )
    try:
        return node.run_wrist()
    except KeyboardInterrupt:
        return 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
