#!/usr/bin/env python3
"""腕部相机单帧红方块检测 + 标定后 base 坐标验证。

测试位姿用 /model_states 真值选姿态（只选姿态）；
方块坐标由腕部相机 + 手眼标定计算，不用真值。

用法：
  python3 scripts/wrist_cube_detect.py --offset 0.20
  python3 scripts/wrist_cube_detect.py --no-move
"""

import argparse
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
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from shape_msgs.msg import SolidPrimitive
from tf2_ros import Buffer, TransformListener

from wrist_vision import (
    CUBE_HALF,
    depth_to_meters,
    draw_debug,
    estimate_cube,
    load_calibration,
)

CUBE_NAME = "red_cube"
GRASP_QX = 0.9238795325
GRASP_QY = 0.3826834324


class WristCubeDetect(Node):
    def __init__(self, args):
        super().__init__("wrist_cube_detect")
        self.args = args
        self.bridge = CvBridge()
        self.K = None
        self.latest = None
        self.frame_count = 0
        self.truth = None
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
        pose = msg.pose[msg.name.index(CUBE_NAME)]
        self.truth = np.array(
            [pose.position.x, pose.position.y, pose.position.z],
            dtype=float,
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
        deadline = time.monotonic() + 10.0
        while (
            rclpy.ok()
            and not self.move_client.server_is_ready()
            and time.monotonic() < deadline
        ):
            rclpy.spin_once(self, timeout_sec=0.1)
        if not self.move_client.server_is_ready():
            return False

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
        constraints.name = "wrist_cube_test"

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
            return False
        handle = send_future.result()
        if handle is None or not handle.accepted:
            return False
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(
            self, result_future, timeout_sec=25.0
        )
        if not result_future.done():
            handle.cancel_goal_async()
            return False
        return (
            result_future.result().result.error_code.val
            == MoveItErrorCodes.SUCCESS
        )

    def lookup_link8(self):
        transform = self.tf_buffer.lookup_transform(
            "panda_link0", "panda_link8", Time()
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

    def run(self):
        if not self.wait_ready():
            print("ERROR: 没有收到 wrist_camera / model_states")
            return 1

        if not self.args.no_move:
            x, y, z_center = self.truth
            target_z = z_center + CUBE_HALF + self.args.offset
            print(
                f"test pose from truth: x={x:.3f} y={y:.3f} "
                f"z={target_z:.3f} (selection only)"
            )
            if not self.move_to(x, y, target_z):
                print("ERROR: MoveIt 没到达测试位姿")
                return 1
            self.spin_for(1.0)

        print(
            f"hand-eye: method={self.calib.get('method')} "
            f"validation={self.calib.get('validation')}"
        )

        errors = []
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        outdir = os.path.abspath(
            os.path.join(
                os.path.dirname(__file__), "..", "output"
            )
        )
        os.makedirs(outdir, exist_ok=True)

        for index in range(self.args.frames):
            frame = self.capture()
            if frame is None:
                print(f"frame {index}: no fresh image")
                continue
            image, depth = frame
            T_base_link8 = self.lookup_link8()
            T_base_cam = T_base_link8 @ self.T_link8_cam
            result, message = estimate_cube(
                image, depth, self.K, T_base_cam
            )
            if result is None:
                print(f"frame {index}: detect failed: {message}")
                cv2.imwrite(
                    os.path.join(
                        outdir, f"wrist_cube_fail_{stamp}_{index}.png"
                    ),
                    image,
                )
                continue

            cube_center = result["cube_center"]
            top_center = result["top_center"]
            error = 1000.0 * float(
                np.linalg.norm(cube_center - self.truth)
            )
            errors.append(error)
            print(
                f"frame {index}: red_points={result['red_points']} "
                f"plane_inliers={result['plane_inliers']} "
                f"top={np.round(top_center, 4)} "
                f"cube={np.round(cube_center, 4)} "
                f"truth={np.round(self.truth, 4)} "
                f"error={error:.2f} mm"
            )

            lines = [
                f"cube=({cube_center[0]:.3f},{cube_center[1]:.3f},"
                f"{cube_center[2]:.3f})",
                f"truth=({self.truth[0]:.3f},{self.truth[1]:.3f},"
                f"{self.truth[2]:.3f})",
                f"error={error:.2f} mm "
                f"inliers={result['plane_inliers']}",
            ]
            debug = draw_debug(image, result, lines)
            path = os.path.join(
                outdir, f"wrist_cube_{stamp}_{index}.png"
            )
            cv2.imwrite(path, debug)
            print(f"  debug: {path}")

        if errors:
            print(
                f"\nsummary: n={len(errors)} "
                f"mean={np.mean(errors):.2f} mm "
                f"max={np.max(errors):.2f} mm"
            )
        return 0 if errors else 1


def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--offset", type=float, default=0.20)
    parser.add_argument("--frames", type=int, default=3)
    parser.add_argument("--no-move", action="store_true")
    parsed = parser.parse_args(args)

    rclpy.init()
    node = WristCubeDetect(parsed)
    try:
        return node.run()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
