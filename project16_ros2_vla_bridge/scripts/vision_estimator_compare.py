#!/usr/bin/env python3
"""静态对比三种方块中心估计方法（不动机器人）。

A：整个红色轮廓的 bounding box 中心 + 深度中值（当前方法）
B：顶面深度过滤后的 3D 质心
C：整个轮廓的矩质心 + 深度中值

前置：
  1. 已启动 vla_bridge/panda_moveit_gazebo.launch.py
  2. 机械臂停在 home，不要同时运行 panda_pick
  3. 终端已经 source ROS 和工作空间

用法：
  python3 project16_ros2_vla_bridge/scripts/vision_estimator_compare.py --trials 10
"""

import argparse
import csv
import math
import os
import random
import time
from datetime import datetime

import cv2
import message_filters
import numpy as np
import rclpy
from cv_bridge import CvBridge
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import SetEntityState
from geometry_msgs.msg import Point, PointStamped
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_geometry_msgs import do_transform_point
from tf2_ros import Buffer, TransformListener

CUBE_NAME = "red_cube"
CUBE_HALF = 0.025
CUBE_Z = 0.375
CAMERA_XY = (0.55, 0.0)
TOP_DEPTH_MARGIN = 0.004

COLOR_RANGES = [
    ((0, 100, 60), (10, 255, 255)),
    ((170, 100, 60), (180, 255, 255)),
]


class EstimatorCompare(Node):
    def __init__(self, args):
        super().__init__("vision_estimator_compare")
        self.args = args
        self.bridge = CvBridge()
        self.K = None
        self.latest = None
        self.frame_count = 0
        self.truth = None

        self.create_subscription(
            CameraInfo, "/camera/camera_info", self.on_camera_info, 10
        )
        image_sub = message_filters.Subscriber(
            self, Image, "/camera/image_raw"
        )
        depth_sub = message_filters.Subscriber(
            self, Image, "/camera/depth/image_raw"
        )
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [image_sub, depth_sub], queue_size=10, slop=0.1
        )
        self.sync.registerCallback(self.on_images)
        self.create_subscription(
            ModelStates, "/model_states", self.on_model_states, 10
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.set_state_client = None
        for name in ("/set_entity_state", "/gazebo/set_entity_state"):
            client = self.create_client(SetEntityState, name)
            if client.wait_for_service(timeout_sec=5.0):
                self.set_state_client = client
                self.get_logger().info(f"using service {name}")
                break
        if self.set_state_client is None:
            raise RuntimeError(
                "找不到 /set_entity_state 或 /gazebo/set_entity_state"
            )

    def on_camera_info(self, msg):
        if len(msg.k) >= 9 and msg.k[0] > 0.0:
            self.K = (
                float(msg.k[0]),
                float(msg.k[4]),
                float(msg.k[2]),
                float(msg.k[5]),
            )

    def on_model_states(self, msg):
        if CUBE_NAME not in msg.name:
            return
        index = msg.name.index(CUBE_NAME)
        position = msg.pose[index].position
        self.truth = (
            float(position.x),
            float(position.y),
            float(position.z),
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
        if depth.dtype == np.uint16:
            depth = depth.astype(np.float32) * 0.001
        else:
            depth = depth.astype(np.float32)
        self.latest = (
            image.copy(),
            depth.copy(),
            image_msg.header,
            self.K,
        )
        self.frame_count += 1

    def spin_for(self, seconds):
        end = time.monotonic() + seconds
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def wait_ready(self):
        deadline = time.monotonic() + 8.0
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if (
                self.K is not None
                and self.latest is not None
                and self.truth is not None
            ):
                return True
        return False

    def read_truth(self, timeout=2.0):
        self.truth = None
        deadline = time.monotonic() + timeout
        while (
            rclpy.ok()
            and self.truth is None
            and time.monotonic() < deadline
        ):
            rclpy.spin_once(self, timeout_sec=0.05)
        return self.truth

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

    def teleport(self, x, y):
        request = SetEntityState.Request()
        request.state.name = CUBE_NAME
        request.state.pose.position.x = x
        request.state.pose.position.y = y
        request.state.pose.position.z = CUBE_Z
        request.state.pose.orientation.w = 1.0
        request.state.twist.linear.x = 0.0
        request.state.twist.linear.y = 0.0
        request.state.twist.linear.z = 0.0
        request.state.twist.angular.x = 0.0
        request.state.twist.angular.y = 0.0
        request.state.twist.angular.z = 0.0
        request.state.reference_frame = "world"
        future = self.set_state_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if not future.done():
            return False, "set_entity_state timeout"
        response = future.result()
        if response is None:
            return False, "no response"
        return bool(response.success), ""

    def to_base(self, camera_point, header):
        point = PointStamped()
        point.header = header
        point.point.x = float(camera_point[0])
        point.point.y = float(camera_point[1])
        point.point.z = float(camera_point[2])
        try:
            transform = self.tf_buffer.lookup_transform(
                "panda_link0", header.frame_id, Time()
            )
            result = do_transform_point(point, transform)
        except Exception as exc:
            self.get_logger().warn(f"TF failed: {exc}")
            return None
        return np.array(
            [result.point.x, result.point.y, result.point.z]
        )

    def estimate(self, image, depth, header):
        fx, fy, cx, cy = self.K
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        mask = None
        for lower, upper in COLOR_RANGES:
            part = cv2.inRange(
                hsv,
                np.array(lower, dtype=np.uint8),
                np.array(upper, dtype=np.uint8),
            )
            mask = part if mask is None else cv2.bitwise_or(mask, part)
        mask = cv2.morphologyEx(
            mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8)
        )
        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        if not contours:
            return None
        contour = max(contours, key=cv2.contourArea)
        if cv2.contourArea(contour) < 50.0:
            return None

        x, y, w, h = cv2.boundingRect(contour)
        roi = np.zeros(depth.shape, dtype=np.uint8)
        cv2.drawContours(roi, [contour], -1, 255, -1)
        values = depth[roi > 0]
        values = values[np.isfinite(values) & (values > 0.0)]
        if values.size == 0:
            return None
        z_median = float(np.median(values))

        # A：bbox 中心
        u_a = x + w / 2.0
        v_a = y + h / 2.0
        camera_a = (
            (u_a - cx) * z_median / fx,
            (v_a - cy) * z_median / fy,
            z_median,
        )
        base_a = self.to_base(camera_a, header)

        # C：轮廓矩质心
        moments = cv2.moments(contour)
        if moments["m00"] == 0.0:
            return None
        u_c = moments["m10"] / moments["m00"]
        v_c = moments["m01"] / moments["m00"]
        camera_c = (
            (u_c - cx) * z_median / fx,
            (v_c - cy) * z_median / fy,
            z_median,
        )
        base_c = self.to_base(camera_c, header)

        # B：顶面深度过滤 + 3D 质心
        rows, cols = np.nonzero(roi)
        zs = depth[rows, cols].astype(np.float64)
        valid = np.isfinite(zs) & (zs > 0.0)
        rows, cols, zs = rows[valid], cols[valid], zs[valid]
        if zs.size == 0:
            return None
        z_top = float(np.percentile(zs, 5))
        keep = zs <= z_top + TOP_DEPTH_MARGIN
        rows, cols, zs = rows[keep], cols[keep], zs[keep]
        if zs.size == 0:
            return None
        X = (cols - cx) * zs / fx
        Y = (rows - cy) * zs / fy
        camera_b = (float(X.mean()), float(Y.mean()), float(zs.mean()))
        base_b = self.to_base(camera_b, header)
        u_b = float(cols.mean())
        v_b = float(rows.mean())

        if base_a is None or base_b is None or base_c is None:
            return None

        return {
            "contour": contour,
            "bbox": (x, y, w, h),
            "A": {"uv": (u_a, v_a), "xyz": base_a},
            "B": {"uv": (u_b, v_b), "xyz": base_b},
            "C": {"uv": (u_c, v_c), "xyz": base_c},
            "top_points": int(zs.size),
            "z_top": z_top,
        }

    def project_truth(self, header, truth):
        point = PointStamped()
        point.header.frame_id = "panda_link0"
        point.point.x = truth[0]
        point.point.y = truth[1]
        point.point.z = truth[2] + CUBE_HALF
        try:
            transform = self.tf_buffer.lookup_transform(
                header.frame_id, "panda_link0", Time()
            )
            camera = do_transform_point(point, transform)
        except Exception:
            return None
        fx, fy, cx, cy = self.K
        if camera.point.z <= 0.0:
            return None
        u = fx * camera.point.x / camera.point.z + cx
        v = fy * camera.point.y / camera.point.z + cy
        return float(u), float(v)

    @staticmethod
    def radial_error(estimate, truth):
        ex = estimate[0] - CAMERA_XY[0]
        ey = estimate[1] - CAMERA_XY[1]
        tx = truth[0] - CAMERA_XY[0]
        ty = truth[1] - CAMERA_XY[1]
        return (
            1000.0 * (math.hypot(tx, ty) - math.hypot(ex, ey)),
            math.degrees(math.atan2(tx * ey - ty * ex, tx * ex + ty * ey)),
        )


def parse_args(argv):
    parser = argparse.ArgumentParser(
        description="Compare three red-cube center estimators"
    )
    parser.add_argument("--trials", type=int, default=10)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--x-min", type=float, default=0.42)
    parser.add_argument("--x-max", type=float, default=0.48)
    parser.add_argument("--y-min", type=float, default=-0.18)
    parser.add_argument("--y-max", type=float, default=-0.12)
    parser.add_argument(
        "--out-dir",
        default=os.path.join(
            os.path.dirname(__file__), "..", "output"
        ),
    )
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)
    rclpy.init()
    node = EstimatorCompare(args)
    try:
        if not node.wait_ready():
            print("ERROR: 没有收到 camera_info / 图像 / model_states")
            return 1

        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        out_dir = os.path.abspath(
            os.path.join(args.out_dir, f"vision_compare_{stamp}")
        )
        os.makedirs(out_dir, exist_ok=True)
        csv_path = os.path.join(out_dir, "summary.csv")

        fieldnames = [
            "trial",
            "cmd_x",
            "cmd_y",
            "truth_x",
            "truth_y",
            "truth_z",
            "a_x",
            "a_y",
            "a_z",
            "a_err_mm",
            "a_radial_mm",
            "b_x",
            "b_y",
            "b_z",
            "b_err_mm",
            "b_radial_mm",
            "c_x",
            "c_y",
            "c_z",
            "c_err_mm",
            "c_radial_mm",
            "b_points",
            "image_file",
        ]

        rng = random.Random(args.seed)
        rows = []
        print(f"results: {out_dir}")

        with open(csv_path, "w", newline="", encoding="utf-8") as handle:
            writer = csv.DictWriter(handle, fieldnames=fieldnames)
            writer.writeheader()

            for trial in range(1, args.trials + 1):
                x = round(rng.uniform(args.x_min, args.x_max), 4)
                y = round(rng.uniform(args.y_min, args.y_max), 4)
                ok, message = node.teleport(x, y)
                if not ok:
                    print(f"ABORT: teleport failed: {message}")
                    break
                node.spin_for(1.0)
                truth = node.read_truth(timeout=2.0)
                frame = node.capture(timeout=3.0)
                if truth is None or frame is None:
                    print(f"trial {trial}: missing truth/frame")
                    continue

                image, depth, header, _ = frame
                estimate = node.estimate(image, depth, header)
                if estimate is None:
                    print(f"trial {trial}: estimator failed")
                    continue

                truth_center = np.array(truth)
                debug = image.copy()
                cv2.drawContours(
                    debug, [estimate["contour"]], -1, (0, 255, 0), 1
                )
                colors = {
                    "A": ((0, 0, 255), "A bbox"),
                    "B": ((0, 255, 0), "B top"),
                    "C": ((255, 0, 0), "C moments"),
                }
                for key, (color, label) in colors.items():
                    u, v = estimate[key]["uv"]
                    cv2.circle(debug, (int(u), int(v)), 4, color, -1)
                    cv2.putText(
                        debug,
                        label,
                        (int(u) + 5, int(v) - 5),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.4,
                        color,
                        1,
                    )
                truth_px = node.project_truth(header, truth_center)
                if truth_px is not None:
                    cv2.drawMarker(
                        debug,
                        (int(truth_px[0]), int(truth_px[1])),
                        (255, 255, 255),
                        cv2.MARKER_CROSS,
                        12,
                        1,
                    )
                image_name = f"trial_{trial:02d}.png"
                cv2.imwrite(os.path.join(out_dir, image_name), debug)

                row = {
                    "trial": trial,
                    "cmd_x": x,
                    "cmd_y": y,
                    "truth_x": truth[0],
                    "truth_y": truth[1],
                    "truth_z": truth[2],
                    "b_points": estimate["top_points"],
                    "image_file": image_name,
                }
                for key in ("A", "B", "C"):
                    xyz = estimate[key]["xyz"]
                    error = 1000.0 * math.hypot(
                        xyz[0] - truth[0], xyz[1] - truth[1]
                    )
                    radial, _ = node.radial_error(xyz, truth_center)
                    row[f"{key.lower()}_x"] = round(xyz[0], 4)
                    row[f"{key.lower()}_y"] = round(xyz[1], 4)
                    row[f"{key.lower()}_z"] = round(xyz[2], 4)
                    row[f"{key.lower()}_err_mm"] = round(error, 2)
                    row[f"{key.lower()}_radial_mm"] = round(
                        radial, 2
                    )
                rows.append(row)
                writer.writerow(row)
                handle.flush()

                print(
                    f"trial {trial}: "
                    f"A={row['a_err_mm']:.2f} mm, "
                    f"B={row['b_err_mm']:.2f} mm, "
                    f"C={row['c_err_mm']:.2f} mm, "
                    f"top_points={estimate['top_points']}"
                )

        node.teleport(0.45, -0.15)

        if not rows:
            print("没有完成任何 trial")
            return 0

        print("\n===== ESTIMATOR SUMMARY =====")
        for key in ("a", "b", "c"):
            errors = [row[f"{key}_err_mm"] for row in rows]
            radial = [row[f"{key}_radial_mm"] for row in rows]
            print(
                f"{key.upper()}: mean={sum(errors) / len(errors):.2f} mm, "
                f"max={max(errors):.2f} mm, "
                f"mean_radial={sum(radial) / len(radial):.2f} mm"
            )
        print(f"csv: {csv_path}")
        print(f"images: {out_dir}")
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
