#!/usr/bin/env python3
"""腕部相机拍照位姿四判据检查。

判据：
  1. 目标在画面内，四周留 >= 20% 图像宽/高
  2. 无自遮挡（脚本给提示，最终看标注图）
  3. 相机到目标深度在 0.3-1.0 m（按 RealSense 常用范围）
  4. MoveIt 能规划到拍照位姿

用法：
  python3 project16_ros2_vla_bridge/scripts/wrist_view_check.py

前置：
  panda_moveit_gazebo.launch.py 已启动，红方块在桌面上。
"""

import argparse
import json
import os
import time
from datetime import datetime

import cv2
import message_filters
import numpy as np
import rclpy
from cv_bridge import CvBridge
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Pose
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    BoundingVolume,
    Constraints,
    MoveItErrorCodes,
    OrientationConstraint,
    PositionConstraint,
)
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from shape_msgs.msg import SolidPrimitive

CUBE_NAME = "red_cube"
CUBE_HALF = 0.025
GRASP_QX = 0.9238795325
GRASP_QY = 0.3826834324
COLOR_RANGES = [
    ((0, 100, 60), (10, 255, 255)),
    ((170, 100, 60), (180, 255, 255)),
]
MARGIN_RATIO = 0.20
DEPTH_MIN = 0.30
DEPTH_MAX = 1.00
MIN_CONTOUR_AREA = 50.0


class WristViewCheck(Node):
    def __init__(self, offsets, span):
        super().__init__("wrist_view_check")
        self.offsets = offsets
        self.span = span
        self.bridge = CvBridge()
        self.K = None
        self.image_size = None
        self.latest = None
        self.frame_count = 0
        self.cube_truth = None

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
        self.create_subscription(
            ModelStates, "/model_states", self.on_model_states, 10
        )

        self.move_client = ActionClient(self, MoveGroup, "/move_action")

    def on_camera_info(self, msg):
        if len(msg.k) >= 9 and msg.k[0] > 0.0:
            self.K = (
                float(msg.k[0]),
                float(msg.k[4]),
                float(msg.k[2]),
                float(msg.k[5]),
            )
            self.image_size = (int(msg.width), int(msg.height))

    def on_model_states(self, msg):
        if CUBE_NAME not in msg.name:
            return
        pose = msg.pose[msg.name.index(CUBE_NAME)]
        self.cube_truth = np.array(
            [pose.position.x, pose.position.y, pose.position.z],
            dtype=np.float64,
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
        # Gazebo 深度图可能是 uint16 毫米，也可能是 float32 米
        if depth.dtype == np.uint16:
            depth = depth.astype(np.float32) * 0.001
        else:
            depth = depth.astype(np.float32)
        self.latest = (image.copy(), depth.copy())
        self.frame_count += 1

    def spin_for(self, seconds):
        end = time.monotonic() + seconds
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def wait_ready(self, timeout=15.0):
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if (
                self.K is not None
                and self.image_size is not None
                and self.latest is not None
                and self.cube_truth is not None
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

    def move_to(self, x, y, z):
        # 主线程还没 spin 过，需要一边 spin 一边等 action server
        deadline = time.monotonic() + 10.0
        while (
            rclpy.ok()
            and not self.move_client.server_is_ready()
            and time.monotonic() < deadline
        ):
            rclpy.spin_once(self, timeout_sec=0.1)
        if not self.move_client.server_is_ready():
            self.get_logger().error("/move_action action not ready")
            return False, -1

        constraint = PositionConstraint()
        constraint.header.frame_id = "panda_link0"
        constraint.link_name = "panda_link8"
        constraint.weight = 1.0
        region = BoundingVolume()
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [0.003, 0.003, 0.003]
        region.primitives = [box]
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        pose.orientation.w = 1.0
        region.primitive_poses = [pose]
        constraint.constraint_region = region

        orientation = OrientationConstraint()
        orientation.header.frame_id = "panda_link0"
        orientation.link_name = "panda_link8"
        orientation.orientation.x = GRASP_QX
        orientation.orientation.y = GRASP_QY
        orientation.orientation.w = 0.0
        orientation.absolute_x_axis_tolerance = 0.1
        orientation.absolute_y_axis_tolerance = 0.1
        orientation.absolute_z_axis_tolerance = 0.1
        orientation.weight = 1.0

        constraints = Constraints()
        constraints.position_constraints = [constraint]
        constraints.orientation_constraints = [orientation]
        constraints.name = "wrist_view_check"

        goal = MoveGroup.Goal()
        goal.request.group_name = "panda_arm"
        goal.request.num_planning_attempts = 10
        goal.request.allowed_planning_time = 5.0
        goal.request.max_velocity_scaling_factor = 0.15
        goal.request.max_acceleration_scaling_factor = 0.15
        goal.request.goal_constraints = [constraints]
        goal.planning_options.plan_only = False
        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = 3

        send_future = self.move_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=10.0)
        if not send_future.done():
            self.get_logger().error("MoveGroup send timeout")
            return False, -2
        handle = send_future.result()
        if handle is None or not handle.accepted:
            return False, -3

        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(
            self, result_future, timeout_sec=20.0
        )
        if not result_future.done():
            self.get_logger().error("MoveGroup result timeout, canceling")
            handle.cancel_goal_async()
            return False, -4
        code = result_future.result().result.error_code.val
        return code == MoveItErrorCodes.SUCCESS, code

    def detect(self, image, depth):
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        red = None
        for lower, upper in COLOR_RANGES:
            part = cv2.inRange(
                hsv,
                np.array(lower, dtype=np.uint8),
                np.array(upper, dtype=np.uint8),
            )
            red = part if red is None else cv2.bitwise_or(red, part)
        red = cv2.morphologyEx(
            red, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8)
        )
        contours, _ = cv2.findContours(
            red, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        contours = [
            c for c in contours if cv2.contourArea(c) >= 20.0
        ]
        if not contours:
            return None, "no red contour"
        contours.sort(key=cv2.contourArea, reverse=True)
        contour = contours[0]
        area = float(cv2.contourArea(contour))
        if area < MIN_CONTOUR_AREA:
            return None, "red contour too small"

        x, y, w, h = cv2.boundingRect(contour)
        height, width = image.shape[:2]
        margins = {
            "left": float(x),
            "right": float(width - 1 - (x + w - 1)),
            "top": float(y),
            "bottom": float(height - 1 - (y + h - 1)),
        }
        margin_ratios = {
            "left": margins["left"] / width,
            "right": margins["right"] / width,
            "top": margins["top"] / height,
            "bottom": margins["bottom"] / height,
        }

        hull = cv2.convexHull(contour)
        hull_area = float(cv2.contourArea(hull))
        solidity = area / hull_area if hull_area > 0.0 else 0.0

        mask = np.zeros(depth.shape, dtype=np.uint8)
        cv2.drawContours(mask, [contour], -1, 255, -1)
        values = depth[mask > 0]
        total = int(values.size)
        valid = values[np.isfinite(values) & (values > 0.0)]
        depth_coverage = (
            float(valid.size) / float(total) if total > 0 else 0.0
        )
        if valid.size == 0:
            return None, "no valid depth on target"
        depth_med = float(np.median(valid))
        depth_p5 = float(np.percentile(valid, 5.0))

        second_ratio = 0.0
        if len(contours) > 1:
            second_ratio = float(
                cv2.contourArea(contours[1]) / area
            )

        return {
            "contour": contour,
            "bbox": (int(x), int(y), int(w), int(h)),
            "area": area,
            "margins": margins,
            "margin_ratios": margin_ratios,
            "min_margin_ratio": min(margin_ratios.values()),
            "solidity": solidity,
            "depth_med": depth_med,
            "depth_p5": depth_p5,
            "depth_coverage": depth_coverage,
            "n_contours": len(contours),
            "second_ratio": second_ratio,
        }, "ok"

    def annotate(self, image, metrics, label):
        debug = image.copy()
        height, width = debug.shape[:2]
        mx = int(round(MARGIN_RATIO * width))
        my = int(round(MARGIN_RATIO * height))
        cv2.rectangle(
            debug, (mx, my), (width - 1 - mx, height - 1 - my),
            (255, 160, 0), 1,
        )
        cv2.drawContours(
            debug, [metrics["contour"]], -1, (0, 255, 0), 2
        )
        x, y, w, h = metrics["bbox"]
        cv2.rectangle(debug, (x, y), (x + w, y + h), (0, 255, 255), 1)
        cv2.drawMarker(
            debug, (width // 2, height // 2), (255, 0, 255),
            cv2.MARKER_CROSS, 12, 1,
        )
        lines = [
            f"{label}",
            f"area={metrics['area']:.0f}px2  "
            f"min_margin={100.0 * metrics['min_margin_ratio']:.1f}%",
            f"depth_med={metrics['depth_med']:.3f}m  "
            f"p5={metrics['depth_p5']:.3f}m",
            f"solidity={metrics['solidity']:.2f}  "
            f"contours={metrics['n_contours']}",
        ]
        for index, text in enumerate(lines):
            cv2.putText(
                debug, text, (6, 14 + 14 * index),
                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1,
            )
        return debug

    def evaluate(self, label, dx, dy, offset, outdir, save=True):
        target_x = float(self.cube_truth[0] + dx)
        target_y = float(self.cube_truth[1] + dy)
        # offset 是 hand 到方块顶面的高度
        target_z = float(
            self.cube_truth[2] + CUBE_HALF + offset
        )
        print(
            f"\n[{label}] move to x={target_x:.3f}, "
            f"y={target_y:.3f}, z={target_z:.3f}"
        )
        ok, code = self.move_to(target_x, target_y, target_z)
        record = {
            "label": label,
            "dx": dx,
            "dy": dy,
            "offset": offset,
            "target": [target_x, target_y, target_z],
            "moveit_ok": bool(ok),
            "moveit_code": int(code),
            "detected": False,
        }
        if not ok:
            print(f"  MoveIt failed: code={code}")
            return record, None

        self.spin_for(1.0)
        frame = self.capture()
        if frame is None:
            print("  no fresh wrist frame")
            return record, None
        image, depth = frame
        metrics, reason = self.detect(image, depth)
        if metrics is None:
            print(f"  detect failed: {reason}")
            if save:
                path = os.path.join(outdir, f"{label}_nodetect.png")
                cv2.imwrite(path, image)
                print(f"  image: {path}")
            return record, None

        margin_px_x = MARGIN_RATIO * image.shape[1]
        margin_px_y = MARGIN_RATIO * image.shape[0]
        pass1 = (
            metrics["margins"]["left"] >= margin_px_x
            and metrics["margins"]["right"] >= margin_px_x
            and metrics["margins"]["top"] >= margin_px_y
            and metrics["margins"]["bottom"] >= margin_px_y
        )
        pass3 = (
            DEPTH_MIN <= metrics["depth_med"] <= DEPTH_MAX
        )
        occlusion_hint = (
            metrics["solidity"] < 0.90
            or metrics["depth_coverage"] < 0.90
            or metrics["second_ratio"] > 0.15
        )
        # contour 是 numpy 数组，不能写进 JSON
        record.update(metrics)
        record.pop("contour", None)
        record.update(
            {
                "detected": True,
                "pass1_margin": bool(pass1),
                "pass3_depth": bool(pass3),
                "pass4_moveit": bool(ok),
                "occlusion_hint": bool(occlusion_hint),
            }
        )

        print(
            f"  detect: area={metrics['area']:.0f}px2, "
            f"margins L/R/T/B="
            f"{metrics['margins']['left']:.0f}/"
            f"{metrics['margins']['right']:.0f}/"
            f"{metrics['margins']['top']:.0f}/"
            f"{metrics['margins']['bottom']:.0f} px"
        )
        print(
            f"  depth: med={metrics['depth_med']:.3f} m, "
            f"p5={metrics['depth_p5']:.3f} m, "
            f"coverage={metrics['depth_coverage']:.2f}"
        )
        print(
            f"  solidity={metrics['solidity']:.2f}, "
            f"contours={metrics['n_contours']}, "
            f"second_ratio={metrics['second_ratio']:.2f}"
        )
        print(
            f"  P1(margin)={'PASS' if pass1 else 'FAIL'}  "
            f"P3(depth)={'PASS' if pass3 else 'FAIL'}  "
            f"P4(moveit)=PASS  "
            f"occlusion_hint={'YES' if occlusion_hint else 'no'}"
        )

        if save:
            debug = self.annotate(image, metrics, label)
            path = os.path.join(outdir, f"{label}.png")
            cv2.imwrite(path, debug)
            print(f"  image: {path}")
        return record, metrics

    def run(self, outdir):
        if not self.wait_ready():
            print("ERROR: 没有收到 wrist_camera / model_states")
            return None

        print("=== wrist_view_check ===")
        print(f"cube truth: {self.cube_truth}")
        print(f"K: {self.K}, image: {self.image_size}")
        if abs(float(self.cube_truth[2]) - CUBE_HALF) > 0.02:
            print(
                "WARNING: 方块 z 不像在桌面上，请先复位场景"
            )

        records = []
        chosen_offset = None
        chosen_center = None

        # 第一轮：在中心位姿找能同时满足边距和深度的最低高度
        print("\n--- stage 1: 中心位姿选高度 ---")
        for offset in self.offsets:
            record, _ = self.evaluate(
                f"probe_h{int(round(offset * 100)):02d}",
                0.0, 0.0, offset, outdir,
            )
            records.append(record)
            if (
                record.get("detected")
                and record.get("pass1_margin")
                and record.get("pass3_depth")
                and record.get("pass4_moveit")
                and not record.get("occlusion_hint")
            ):
                chosen_offset = offset
                chosen_center = record
                print(f"\nchosen offset: {offset:.2f} m")
                break

        if chosen_offset is None:
            candidates = [
                r for r in records
                if r.get("detected") and r.get("moveit_ok")
            ]
            if candidates:
                best = max(
                    candidates, key=lambda r: r["min_margin_ratio"]
                )
                chosen_offset = best["offset"]
                chosen_center = best
                print(
                    f"\nWARNING: 没有完全通过的高度，"
                    f"用边距最大的 {chosen_offset:.2f} m 继续"
                )
            else:
                print("\nERROR: 所有高度都检测失败")
                return records

        # 第二轮：在选定高度检查 5 个相对位置
        print(
            f"\n--- stage 2: {chosen_offset:.2f} m, 5 positions ---"
        )
        span = self.span
        positions = [
            ("center", 0.0, 0.0),
            ("corner_xm_ym", -span, -span),
            ("corner_xm_yp", -span, span),
            ("corner_xp_ym", span, -span),
            ("corner_xp_yp", span, span),
        ]
        for label, dx, dy in positions:
            if (
                label == "center"
                and chosen_center is not None
                and chosen_center.get("offset") == chosen_offset
            ):
                # 中心位姿第一轮已经采过，直接复用
                continue
            record, _ = self.evaluate(
                f"pos_{label}", dx, dy, chosen_offset, outdir
            )
            records.append(record)

        # 把第一轮选中的中心记录也放进最终表
        final = [
            r for r in records
            if r.get("offset") == chosen_offset
        ]
        return final


def print_table(records):
    header = (
        "pose            moveit  area   min_m%  depth_m  sol   P1  P3  hint"
    )
    print("\n" + header)
    print("-" * len(header))
    for r in records:
        if not r.get("moveit_ok"):
            print(
                f"{r['label']:<15} FAIL    -      -       -        "
                f"-     -   -   -"
            )
            continue
        if not r.get("detected"):
            print(
                f"{r['label']:<15} ok      none   -       -        "
                f"-     -   -   -"
            )
            continue
        print(
            f"{r['label']:<15} ok      "
            f"{r['area']:>5.0f}  "
            f"{100.0 * r['min_margin_ratio']:>5.1f}   "
            f"{r['depth_med']:>5.3f}   "
            f"{r['solidity']:>4.2f}  "
            f"{'ok' if r['pass1_margin'] else 'NO':<4}"
            f"{'ok' if r['pass3_depth'] else 'NO':<4}"
            f"{'YES' if r['occlusion_hint'] else 'no'}"
        )


def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--offsets", nargs="+", type=float,
        default=[0.30, 0.35, 0.40],
        help="候选拍照高度（hand 到方块顶面，米）",
    )
    parser.add_argument(
        "--span", type=float, default=0.03,
        help="四个角相对中心的偏移量（米）",
    )
    parsed = parser.parse_args(args)

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    outdir = os.path.abspath(
        os.path.join(
            os.path.dirname(__file__),
            "..",
            "output",
            f"wrist_view_check_{stamp}",
        )
    )
    os.makedirs(outdir, exist_ok=True)

    rclpy.init()
    node = WristViewCheck(parsed.offsets, parsed.span)
    try:
        records = node.run(outdir)
        if records is None:
            return 2
        print_table(records)
        summary = {
            "offsets": parsed.offsets,
            "span": parsed.span,
            "outdir": outdir,
            "records": records,
        }
        with open(
            os.path.join(outdir, "summary.json"), "w", encoding="utf-8"
        ) as handle:
            json.dump(summary, handle, indent=2, ensure_ascii=False)

        all_ok = all(
            r.get("moveit_ok")
            and r.get("detected")
            and r.get("pass1_margin")
            and r.get("pass3_depth")
            and not r.get("occlusion_hint")
            for r in records
        )
        print("\nP1 margin >=20% | P2 visual (look at images) | "
              "P3 depth 0.3-1.0m | P4 MoveIt")
        print(f"P1/P3/P4 + hint: {'ALL PASS' if all_ok else 'NOT YET'}")
        print(f"output: {outdir}")
        return 0
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
