#!/usr/bin/env python3
"""单帧 ChArUco 检测：验证腕部相机能否完整看到标定板。

用法：
  python3 project16_ros2_vla_bridge/scripts/charuco_detect.py

前置：
  Gazebo 已带 GAZEBO_MODEL_PATH / GAZEBO_RESOURCE_PATH 启动，
  并且 charuco_board 已经 spawn 到桌面。
"""

import argparse
import os
import time
from datetime import datetime

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
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

SQUARES_X = 5
SQUARES_Y = 7
SQUARE_LENGTH = 0.03
MARKER_LENGTH = 0.0225
GRASP_QX = 0.9238795325
GRASP_QY = 0.3826834324


def get_dictionary():
    # 兼容 OpenCV 4.5 和 4.7+
    if hasattr(cv2.aruco, "getPredefinedDictionary"):
        return cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    return cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)


def get_board(dictionary):
    if hasattr(cv2.aruco, "CharucoBoard_create"):
        return cv2.aruco.CharucoBoard_create(
            SQUARES_X, SQUARES_Y, SQUARE_LENGTH, MARKER_LENGTH, dictionary
        )
    return cv2.aruco.CharucoBoard(
        (SQUARES_X, SQUARES_Y),
        SQUARE_LENGTH,
        MARKER_LENGTH,
        dictionary,
    )


def board_object_points(board):
    if hasattr(board, "getChessboardCorners"):
        points = board.getChessboardCorners()
    else:
        points = board.chessboardCorners
    return np.asarray(points, dtype=np.float64).reshape(-1, 3)


class CharucoDetect(Node):
    def __init__(self, args):
        super().__init__("charuco_detect")
        self.args = args
        self.bridge = CvBridge()
        self.K = None
        self.latest = None
        self.frame_count = 0

        self.create_subscription(
            CameraInfo,
            "/wrist_camera/camera_info",
            self.on_camera_info,
            10,
        )
        self.create_subscription(
            Image,
            "/wrist_camera/image_raw",
            self.on_image,
            10,
        )
        self.move_client = ActionClient(self, MoveGroup, "/move_action")

        self.dictionary = get_dictionary()
        self.board = get_board(self.dictionary)
        # OpenCV 4.7+ 用 CharucoDetector；4.5 用旧函数
        self.detector = (
            cv2.aruco.CharucoDetector(self.board)
            if hasattr(cv2.aruco, "CharucoDetector")
            else None
        )

    def on_camera_info(self, msg):
        if len(msg.k) >= 9 and msg.k[0] > 0.0:
            self.K = np.array(
                [
                    [msg.k[0], 0.0, msg.k[2]],
                    [0.0, msg.k[4], msg.k[5]],
                    [0.0, 0.0, 1.0],
                ],
                dtype=np.float64,
            )

    def on_image(self, msg):
        try:
            image = self.bridge.imgmsg_to_cv2(
                msg, desired_encoding="bgr8"
            )
        except Exception as exc:
            self.get_logger().warn(f"cv_bridge failed: {exc}")
            return
        self.latest = image.copy()
        self.frame_count += 1

    def spin_for(self, seconds):
        end = time.monotonic() + seconds
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def wait_ready(self, timeout=15.0):
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.K is not None and self.latest is not None:
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
            self.get_logger().error("/move_action action not ready")
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
        constraints.name = "charuco_photo"

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
            self, result_future, timeout_sec=20.0
        )
        if not result_future.done():
            handle.cancel_goal_async()
            return False
        return (
            result_future.result().result.error_code.val
            == MoveItErrorCodes.SUCCESS
        )

    def detect(self, image):
        if hasattr(cv2.aruco, "detectMarkers"):
            marker_corners, marker_ids, _ = cv2.aruco.detectMarkers(
                image, self.dictionary
            )
            if marker_ids is None:
                return None, "no ArUco markers"
            count, charuco_corners, charuco_ids = (
                cv2.aruco.interpolateCornersCharuco(
                    marker_corners, marker_ids, image, self.board
                )
            )
            if charuco_ids is None or int(count) < 6:
                return None, f"too few ChArUco corners: {count}"
            charuco_count = int(count)
        else:
            (charuco_corners, charuco_ids, marker_corners,
             marker_ids) = self.detector.detectBoard(image)
            if charuco_ids is None:
                return None, "no ChArUco corners"
            charuco_count = int(len(charuco_ids))
            if charuco_count < 6:
                return None, (
                    f"too few ChArUco corners: {charuco_count}"
                )
            if marker_ids is None:
                marker_ids = np.empty((0, 1), dtype=np.int32)

        object_points = board_object_points(self.board)
        object_points = object_points[charuco_ids.flatten()]
        image_points = charuco_corners.reshape(-1, 2)

        # 平面板有两组 PnP 解，选板面法向朝向相机的那个
        solution_count, rvecs, tvecs, _ = cv2.solvePnPGeneric(
            object_points,
            image_points,
            self.K,
            None,
            flags=cv2.SOLVEPNP_IPPE,
        )
        if solution_count == 0:
            return None, "solvePnP failed"
        # IPPE 对平面板返回两组解。只按重投影误差选，
        # 不强制板子 +z 的符号，否则可能选中误差更大的镜像解。
        print(f"IPPE solutions: {solution_count}")
        solutions = []
        best = None
        for index, (rvec, tvec) in enumerate(zip(rvecs, tvecs)):
            projected, _ = cv2.projectPoints(
                object_points, rvec, tvec, self.K, None
            )
            error = float(
                np.mean(
                    np.linalg.norm(
                        projected.reshape(-1, 2) - image_points,
                        axis=1,
                    )
                )
            )
            rotation, _ = cv2.Rodrigues(rvec)
            normal = rotation[:, 2]
            facing_camera = normal[2] < 0.0
            solutions.append(
                {
                    "index": index,
                    "reproj": error,
                    "rvec": rvec.reshape(3).tolist(),
                    "tvec": tvec.reshape(3).tolist(),
                    "normal": normal.tolist(),
                    "facing_camera": bool(facing_camera),
                }
            )
            print(
                f"  solution {index}: reproj={error:.3f} px, "
                f"normal={np.round(normal, 3)}, "
                f"tvec={np.round(tvec.reshape(3), 3)}, "
                f"facing_camera={facing_camera}"
            )
            score = error
            if best is None or score < best[0]:
                best = (
                    score,
                    rvec,
                    tvec,
                    normal,
                    error,
                    facing_camera,
                    index,
                )

        (
            _,
            rvec,
            tvec,
            normal,
            reprojection,
            facing,
            chosen_index,
        ) = best
        print(f"chosen solution: {chosen_index}")
        reproj_gap = 0.0
        if len(solutions) >= 2:
            reproj_gap = abs(
                solutions[0]["reproj"] - solutions[1]["reproj"]
            )
        return {
            "solutions": solutions,
            "chosen_solution": chosen_index,
            "reproj_gap": reproj_gap,
            "marker_count": int(len(marker_ids)),
            "marker_ids": [int(v) for v in marker_ids.flatten()],
            "charuco_count": charuco_count,
            "charuco_ids": [int(v) for v in charuco_ids.flatten()],
            "rvec": rvec.reshape(3),
            "tvec": tvec.reshape(3),
            "normal": normal,
            "reprojection": reprojection,
            "facing_camera": bool(facing),
            "charuco_corners": charuco_corners,
            "charuco_ids_raw": charuco_ids,
            "marker_corners": marker_corners,
        }, "ok"


def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--board-x", type=float, default=0.40)
    parser.add_argument("--board-y", type=float, default=0.02)
    parser.add_argument(
        "--board-top", type=float, default=0.353,
        help="标定板顶面高度（米）",
    )
    parser.add_argument(
        "--offset", type=float, default=0.40,
        help="hand 到板面的高度（米）",
    )
    parser.add_argument(
        "--no-move", action="store_true",
        help="不移动机械臂，直接用当前画面",
    )
    parsed = parser.parse_args(args)

    rclpy.init()
    node = CharucoDetect(parsed)
    try:
        if not node.wait_ready():
            print("ERROR: 没有收到 wrist_camera 图像或内参")
            return 2

        if not parsed.no_move:
            target_z = parsed.board_top + parsed.offset
            print(
                f"move to x={parsed.board_x:.3f}, "
                f"y={parsed.board_y:.3f}, z={target_z:.3f}"
            )
            if not node.move_to(
                parsed.board_x, parsed.board_y, target_z
            ):
                print("ERROR: MoveIt 没到达拍照位姿")
                return 2
            node.spin_for(1.0)

        image = node.capture()
        if image is None:
            print("ERROR: 没有拿到新的腕部图像")
            return 2

        # 原图单独保存，方便离线复核 marker / charuco 的 PnP
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        outdir = os.path.abspath(
            os.path.join(
                os.path.dirname(__file__),
                "..",
                "output",
            )
        )
        os.makedirs(outdir, exist_ok=True)
        raw_out = os.path.join(outdir, f"charuco_raw_{stamp}.png")
        cv2.imwrite(raw_out, image)
        print(f"raw image: {raw_out}")

        result, message = node.detect(image)
        if result is None:
            print(f"detect failed: {message}")
            return 1

        print(f"markers: {result['marker_count']}")
        print(f"marker ids: {result['marker_ids']}")
        print(f"charuco corners: {result['charuco_count']}")
        print(f"charuco ids: {result['charuco_ids']}")
        print(f"K: {node.K.tolist()}")
        print(f"tvec (board in camera): {result['tvec']}")
        print(f"normal: {result['normal']}")
        print(
            f"reprojection mean: "
            f"{result['reprojection']:.3f} px"
        )
        print(
            f"facing camera: {result['facing_camera']}"
        )

        debug = image.copy()
        if result["marker_count"] > 0:
            cv2.aruco.drawDetectedMarkers(
                debug,
                result["marker_corners"],
                np.array(
                    result["marker_ids"], dtype=np.int32
                ).reshape(-1, 1),
            )
        cv2.aruco.drawDetectedCornersCharuco(
            debug,
            result["charuco_corners"],
            result["charuco_ids_raw"],
            (0, 255, 0),
        )
        cv2.drawFrameAxes(
            debug,
            node.K,
            None,
            result["rvec"],
            result["tvec"],
            0.05,
        )
        out = os.path.join(outdir, f"charuco_detect_{stamp}.png")
        cv2.imwrite(out, debug)
        print(f"debug image: {out}")
        return 0
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
