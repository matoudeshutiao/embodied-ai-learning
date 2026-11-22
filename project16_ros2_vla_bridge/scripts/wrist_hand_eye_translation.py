#!/usr/bin/env python3
"""腕部相机手眼标定初次实验：先标定 hand→camera 平移。

- 手臂姿态由 /model_states 的红方块真值生成，不依赖俯视视觉
- 每个姿态用红色点云 RANSAC 平面拟合取顶面质心
- 相机旋转用 URDF 名义值；只估计平移
- 与 TF 里的真实安装平移对比

用法：
  python3 project16_ros2_vla_bridge/scripts/wrist_hand_eye_translation.py
"""

import math
import os
import time
from datetime import datetime

import cv2
import message_filters
import numpy as np
import rclpy
from control_msgs.action import FollowJointTrajectory  # noqa: F401
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
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from shape_msgs.msg import SolidPrimitive
from tf2_ros import Buffer, TransformListener

CUBE_NAME = "red_cube"
CUBE_HALF = 0.025
GRASP_QX = 0.9238795325
GRASP_QY = 0.3826834324
COLOR_RANGES = [
    ((0, 100, 60), (10, 255, 255)),
    ((170, 100, 60), (180, 255, 255)),
]


def quaternion_to_matrix(q):
    x, y, z, w = q
    norm = x * x + y * y + z * z + w * w
    if norm == 0.0:
        return np.eye(3)
    s = 2.0 / norm
    return np.array(
        [
            [
                1 - s * (y * y + z * z),
                s * (x * y - z * w),
                s * (x * z + y * w),
            ],
            [
                s * (x * y + z * w),
                1 - s * (x * x + z * z),
                s * (y * z - x * w),
            ],
            [
                s * (x * z - y * w),
                s * (y * z + x * w),
                1 - s * (x * x + y * y),
            ],
        ]
    )


def transform_parts(transform):
    t = transform.transform.translation
    q = transform.transform.rotation
    return (
        np.array([t.x, t.y, t.z]),
        quaternion_to_matrix((q.x, q.y, q.z, q.w)),
    )


class WristHandEye(Node):
    def __init__(self):
        super().__init__("wrist_hand_eye_translation")
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
        self.move_client = ActionClient(self, MoveGroup, "/move_action")

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
        self.truth = np.array(
            [position.x, position.y, position.z], dtype=np.float64
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
        self.latest = (image.copy(), depth.copy())
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

    def move_to(self, x, y, z):
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
        constraints.name = "wrist_hand_eye"

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
        rclpy.spin_until_future_complete(self, send_future)
        handle = send_future.result()
        if handle is None or not handle.accepted:
            return False
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result
        return result.error_code.val == MoveItErrorCodes.SUCCESS

    def top_plane_center(self, image, depth):
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
            return None, "no red contour", 0
        contour = max(contours, key=cv2.contourArea)
        if cv2.contourArea(contour) < 50.0:
            return None, "red contour too small", 0
        roi = np.zeros(depth.shape, dtype=np.uint8)
        cv2.drawContours(roi, [contour], -1, 255, -1)
        rows, cols = np.nonzero(roi)
        zs = depth[rows, cols].astype(np.float64)
        valid = np.isfinite(zs) & (zs > 0.0)
        rows, cols, zs = rows[valid], cols[valid], zs[valid]
        if zs.size < 50:
            return None, "too few depth points", 0

        z_min = float(np.percentile(zs, 5))
        keep = zs <= z_min + 0.03
        rows, cols, zs = rows[keep], cols[keep], zs[keep]
        if rows.size < 50:
            return None, "too few near-top points", 0

        fx, fy, cx, cy = self.K
        points = np.column_stack(
            [
                (cols - cx) * zs / fx,
                (rows - cy) * zs / fy,
                zs,
            ]
        )
        rng = np.random.default_rng(0)
        best = None
        for _ in range(300):
            sample = rng.choice(points.shape[0], 3, replace=False)
            p0, p1, p2 = points[sample]
            normal = np.cross(p1 - p0, p2 - p0)
            norm = float(np.linalg.norm(normal))
            if norm < 1e-9:
                continue
            normal /= norm
            distance = -float(np.dot(normal, p0))
            inliers = np.abs(points @ normal + distance) < 0.003
            count = int(inliers.sum())
            if count < 50:
                continue
            mean_z = float(points[inliers, 2].mean())
            score = (count, -mean_z)
            if best is None or score > best[0]:
                best = (score, inliers.copy())
        if best is None:
            return None, "plane RANSAC failed", 0
        inliers = best[1]
        center = points[inliers].mean(axis=0)
        return center, f"plane inliers={int(inliers.sum())}", int(
            inliers.sum()
        )

    def lookup(self, target, source):
        try:
            transform = self.tf_buffer.lookup_transform(
                target, source, Time()
            )
        except Exception as exc:
            self.get_logger().warn(f"TF {target}<-{source}: {exc}")
            return None
        return transform

    def run(self):
        if not self.wait_ready():
            print("ERROR: wrist camera / model_states not ready")
            return 1
        if not self.move_client.wait_for_server(timeout_sec=30.0):
            print("ERROR: /move_action not ready")
            return 1

        cube = self.truth.copy()
        target_base = cube + np.array([0.0, 0.0, CUBE_HALF])
        print(f"cube truth: {cube}")

        offsets = []
        for dz in (0.20, 0.24, 0.28):
            for dx, dy in (
                (0.0, 0.0),
                (0.02, 0.0),
                (-0.02, 0.0),
                (0.0, 0.02),
                (0.0, -0.02),
            ):
                offsets.append((dx, dy, dz))

        rows = []
        for index, (dx, dy, dz) in enumerate(offsets, start=1):
            x = float(cube[0] + dx)
            y = float(cube[1] + dy)
            z = float(cube[2] + dz)
            if not self.move_to(x, y, z):
                print(f"pose {index}: planning failed, skip")
                continue
            self.spin_for(0.8)
            frame = self.capture()
            if frame is None:
                continue

            image, depth = frame
            center_cam, message, count = self.top_plane_center(
                image, depth
            )
            if center_cam is None:
                print(f"pose {index}: {message}, skip")
                continue

            base_hand = self.lookup("panda_link0", "panda_hand")
            hand_cam = self.lookup(
                "panda_hand", "wrist_camera_optical_frame"
            )
            if base_hand is None or hand_cam is None:
                continue
            t_base_hand, R_base_hand = transform_parts(base_hand)
            t_hand_cam_nominal, R_hand_cam_nominal = transform_parts(
                hand_cam
            )
            R_base_cam = R_base_hand @ R_hand_cam_nominal
            t_base_cam = target_base - R_base_cam @ center_cam
            t_hand_cam = R_base_hand.T @ (t_base_cam - t_base_hand)
            rows.append(
                {
                    "index": index,
                    "center_cam": center_cam,
                    "t_hand_cam": t_hand_cam,
                    "nominal": t_hand_cam_nominal,
                    "count": count,
                }
            )
            print(
                f"pose {index}: inliers={count}, "
                f"t_hand_cam={np.round(t_hand_cam, 5)}"
            )

        if not rows:
            print("没有有效的标定姿态")
            return 1

        estimates = np.array([row["t_hand_cam"] for row in rows])
        average = estimates.mean(axis=0)
        nominal = rows[0]["nominal"]
        error = 1000.0 * np.linalg.norm(average - nominal)
        print("\n===== WRIST HAND-EYE TRANSLATION =====")
        print(f"poses used: {len(rows)}")
        print(f"average t_hand_cam: {np.round(average, 5)}")
        print(f"true nominal:       {np.round(nominal, 5)}")
        print(f"translation error:  {error:.2f} mm")
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        out = os.path.abspath(
            os.path.join(
                os.path.dirname(__file__),
                "..",
                "output",
                f"wrist_hand_eye_{stamp}.txt",
            )
        )
        with open(out, "w", encoding="utf-8") as handle:
            handle.write(f"poses={len(rows)}\n")
            handle.write(f"average={average.tolist()}\n")
            handle.write(f"nominal={nominal.tolist()}\n")
            handle.write(f"error_mm={error}\n")
        print(f"saved: {out}")
        return 0


def main(args=None):
    rclpy.init(args=args)
    node = WristHandEye()
    try:
        return node.run()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
