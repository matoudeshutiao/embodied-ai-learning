#!/usr/bin/env python3
"""腕部相机单帧探测：红色方块顶面四边形 + PnP。

前置：
  1. 已启动 panda_moveit_gazebo.launch.py
  2. 机械臂在方块上方（可先运行 panda_move_to_target）
  3. 终端已 source ROS 和工作空间

用法：
  python3 project16_ros2_vla_bridge/scripts/wrist_target_probe.py
"""

import math
import os
import time
from datetime import datetime

import cv2
import message_filters
import numpy as np
import rclpy
from cv_bridge import CvBridge
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_geometry_msgs import do_transform_point
from tf2_ros import Buffer, TransformListener

CUBE_NAME = "red_cube"
CUBE_HALF = 0.025
TOP_DEPTH_MARGIN = 0.002
COLOR_RANGES = [
    ((0, 100, 60), (10, 255, 255)),
    ((170, 100, 60), (180, 255, 255)),
]


def rotate_by_quaternion(vector, q):
    """把向量按四元数 (x,y,z,w) 旋转。"""
    x, y, z, w = q
    norm = x * x + y * y + z * z + w * w
    if norm == 0.0:
        return vector
    s = 2.0 / norm
    vx, vy, vz = vector
    return (
        (1.0 - s * (y * y + z * z)) * vx
        + s * (x * y - z * w) * vy
        + s * (x * z + y * w) * vz,
        s * (x * y + z * w) * vx
        + (1.0 - s * (x * x + z * z)) * vy
        + s * (y * z - x * w) * vz,
        s * (x * z - y * w) * vx
        + s * (y * z + x * w) * vy
        + (1.0 - s * (x * x + y * y)) * vz,
    )


class WristProbe(Node):
    def __init__(self):
        super().__init__("wrist_target_probe")
        self.bridge = CvBridge()
        self.K = None
        self.latest = None
        self.frame_count = 0
        self.truth = None

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

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

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
        pose = msg.pose[index]
        self.truth = (
            (
                float(pose.position.x),
                float(pose.position.y),
                float(pose.position.z),
            ),
            (
                float(pose.orientation.x),
                float(pose.orientation.y),
                float(pose.orientation.z),
                float(pose.orientation.w),
            ),
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
        deadline = time.monotonic() + 10.0
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if (
                self.K is not None
                and self.latest is not None
                and self.truth is not None
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

    @staticmethod
    def order_quad(points):
        pts = np.array(points, dtype=np.float32).reshape(-1, 2)
        center = pts.mean(axis=0)
        angles = np.arctan2(
            pts[:, 1] - center[1], pts[:, 0] - center[0]
        )
        return pts[np.argsort(angles)]

    def top_face_quad(self, image, depth):
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
        if not contours:
            return None, "no red contour"
        contour = max(contours, key=cv2.contourArea)
        if cv2.contourArea(contour) < 50.0:
            return None, "red contour too small"

        roi = np.zeros(depth.shape, dtype=np.uint8)
        cv2.drawContours(roi, [contour], -1, 255, -1)
        values = depth[roi > 0]
        values = values[np.isfinite(values) & (values > 0.0)]
        if values.size == 0:
            return None, "no valid depth"

        z_top = float(np.percentile(values, 5))
        top_mask = np.zeros(depth.shape, dtype=np.uint8)
        valid = (
            (roi > 0)
            & np.isfinite(depth)
            & (depth > 0.0)
            & (depth <= z_top + TOP_DEPTH_MARGIN)
        )
        top_mask[valid] = 255
        top_mask = cv2.morphologyEx(
            top_mask, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8)
        )
        top_contours, _ = cv2.findContours(
            top_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        if not top_contours:
            return None, "no top-face contour"
        top = max(top_contours, key=cv2.contourArea)
        area = cv2.contourArea(top)
        if area < 80.0:
            return None, "top-face contour too small"

        perimeter = cv2.arcLength(top, True)
        approx = None
        for epsilon in (0.02, 0.03, 0.05, 0.08):
            candidate = cv2.approxPolyDP(
                top, epsilon * perimeter, True
            )
            if len(candidate) == 4:
                approx = candidate
                break
        if approx is None:
            rect = cv2.minAreaRect(top)
            approx = cv2.boxPoints(rect).reshape(-1, 1, 2)
        if len(approx) != 4:
            return None, f"quad has {len(approx)} points"
        return self.order_quad(approx), f"top area={area:.0f}px2"

    def expected_target_in_camera(self, target_base, header):
        point = PointStamped()
        point.header.frame_id = "panda_link0"
        point.point.x = float(target_base[0])
        point.point.y = float(target_base[1])
        point.point.z = float(target_base[2])
        try:
            transform = self.tf_buffer.lookup_transform(
                header.frame_id, "panda_link0", Time()
            )
            result = do_transform_point(point, transform)
        except Exception as exc:
            self.get_logger().warn(f"TF failed: {exc}")
            return None
        return np.array(
            [result.point.x, result.point.y, result.point.z]
        )

    def run(self):
        if not self.wait_ready():
            print("ERROR: 没有收到 wrist_camera / model_states")
            return 1
        frame = self.capture()
        if frame is None:
            print("ERROR: 没有拿到腕部图像")
            return 1

        image, depth, header, _ = frame
        corners, message = self.top_face_quad(image, depth)
        print(f"detect: {message}")
        if corners is None:
            stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            out = os.path.abspath(
                os.path.join(
                    os.path.dirname(__file__),
                    "..",
                    "output",
                    f"wrist_probe_{stamp}.png",
                )
            )
            cv2.imwrite(out, image)
            print(f"debug image: {out}")
            return 1

        print("corners (u,v):")
        for index, (u, v) in enumerate(corners):
            print(f"  {index}: ({u:.1f}, {v:.1f})")

        cube_center, cube_quaternion = self.truth
        top_offset = rotate_by_quaternion(
            (0.0, 0.0, CUBE_HALF), cube_quaternion
        )
        target_base = (
            cube_center[0] + top_offset[0],
            cube_center[1] + top_offset[1],
            cube_center[2] + top_offset[2],
        )
        expected = self.expected_target_in_camera(
            target_base, header
        )
        if expected is None:
            return 1

        half = CUBE_HALF
        object_points = np.array(
            [
                [-half, -half, 0.0],
                [half, -half, 0.0],
                [half, half, 0.0],
                [-half, half, 0.0],
            ],
            dtype=np.float32,
        )
        fx, fy, cx, cy = self.K
        camera_matrix = np.array(
            [[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]],
            dtype=np.float64,
        )
        count, rvecs, tvecs, _ = cv2.solvePnPGeneric(
            object_points,
            corners.reshape(-1, 1, 2),
            camera_matrix,
            None,
            flags=cv2.SOLVEPNP_IPPE,
        )
        if count == 0:
            print("ERROR: solvePnP failed")
            return 1

        best = None
        for candidate_rvec, candidate_tvec in zip(rvecs, tvecs):
            candidate_projected, _ = cv2.projectPoints(
                object_points,
                candidate_rvec,
                candidate_tvec,
                camera_matrix,
                None,
            )
            candidate_error = float(
                np.mean(
                    np.linalg.norm(
                        candidate_projected.reshape(-1, 2) - corners,
                        axis=1,
                    )
                )
            )
            rotation_matrix, _ = cv2.Rodrigues(candidate_rvec)
            candidate_normal = rotation_matrix @ np.array(
                [0.0, 0.0, 1.0]
            )
            # IPPE 的两组平面解都可能 z<0；选法向最接近相机 -z 的一组
            facing = math.acos(
                float(np.clip(-candidate_normal[2], -1.0, 1.0))
            )
            score = candidate_error + 100.0 * facing
            if best is None or score < best[0]:
                best = (
                    score,
                    candidate_rvec,
                    candidate_tvec,
                    candidate_normal,
                    candidate_error,
                )
        _, rvec, tvec, normal, reprojection = best
        translation_error = 1000.0 * float(
            np.linalg.norm(tvec.reshape(3) - expected)
        )
        rotation_matrix, _ = cv2.Rodrigues(rvec)
        normal = rotation_matrix @ np.array([0.0, 0.0, 1.0])
        try:
            transform = self.tf_buffer.lookup_transform(
                header.frame_id, "panda_link0", Time()
            )
            rotation = transform.transform.rotation
            tf_quaternion = (
                rotation.x,
                rotation.y,
                rotation.z,
                rotation.w,
            )
            world_normal = rotate_by_quaternion(
                (0.0, 0.0, 1.0), cube_quaternion
            )
            expected_normal = np.array(
                rotate_by_quaternion(world_normal, tf_quaternion)
            )
            expected_normal /= np.linalg.norm(expected_normal)
            cos_angle = float(
                np.clip(
                    np.dot(normal, expected_normal),
                    -1.0,
                    1.0,
                )
            )
            normal_error = math.degrees(math.acos(cos_angle))
        except Exception as exc:
            self.get_logger().warn(f"TF normal failed: {exc}")
            normal_error = None

        print(f"K: fx={fx:.3f}, fy={fy:.3f}, cx={cx:.3f}, cy={cy:.3f}")
        print(f"cube truth: {cube_center}")
        print(f"PnP tvec (camera): {tvec.reshape(3)}")
        print(f"expected target (camera): {expected}")
        print(f"translation error: {translation_error:.2f} mm")
        if normal_error is None:
            print("normal angle error: —")
        else:
            print(f"normal angle error: {normal_error:.2f} deg")
        print(f"reprojection mean: {reprojection:.2f} px")

        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        out = os.path.abspath(
            os.path.join(
                os.path.dirname(__file__),
                "..",
                "output",
                f"wrist_probe_{stamp}.png",
            )
        )
        debug = image.copy()
        cv2.drawContours(
            debug,
            [corners.reshape(-1, 1, 2).astype(np.int32)],
            -1,
            (0, 255, 0),
            2,
        )
        for index, (u, v) in enumerate(corners):
            cv2.circle(
                debug, (int(u), int(v)), 3, (0, 255, 255), -1
            )
            cv2.putText(
                debug,
                str(index),
                (int(u) + 5, int(v) - 5),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.4,
                (0, 255, 255),
                1,
            )
        cv2.imwrite(out, debug)
        print(f"debug image: {out}")
        return 0


def main(args=None):
    rclpy.init(args=args)
    node = WristProbe()
    try:
        return node.run()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
