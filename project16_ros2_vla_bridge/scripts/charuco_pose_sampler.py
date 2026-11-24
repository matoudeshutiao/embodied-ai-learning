#!/usr/bin/env python3
"""采 ChArUco 手眼标定姿态，记录 T_target2cam 与 T_base_link8。

用法：
  python3 scripts/charuco_pose_sampler.py --set pilot
  python3 scripts/charuco_pose_sampler.py --set full

前置：
  Gazebo + MoveIt 已启动，charuco_board 已 spawn。
"""

import argparse
import csv
import json
import math
import os
import time
from datetime import datetime

import numpy as np
import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    BoundingVolume,
    Constraints,
    MoveItErrorCodes,
    OrientationConstraint,
    PositionConstraint,
)
from rclpy.time import Time
from shape_msgs.msg import SolidPrimitive
from tf2_ros import Buffer, TransformListener

from charuco_detect import CharucoDetect

# 成功单帧检测时使用的拍照朝向
BASE_Q = np.array([0.9238795325, 0.3826834324, 0.0, 0.0])
BOARD_X = 0.40
BOARD_Y = 0.02
BOARD_TOP = 0.353
BASE_HAND_Y = BOARD_Y - 0.04


def quat_mul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return np.array(
        [
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz,
        ]
    )


def quat_axis_angle(axis, angle_rad):
    axis = np.asarray(axis, dtype=float)
    axis /= np.linalg.norm(axis)
    half = 0.5 * angle_rad
    return np.array(
        [
            axis[0] * math.sin(half),
            axis[1] * math.sin(half),
            axis[2] * math.sin(half),
            math.cos(half),
        ]
    )


def quat_to_matrix(q):
    x, y, z, w = q
    norm = x * x + y * y + z * z + w * w
    if norm == 0.0:
        return np.eye(3)
    s = 2.0 / norm
    return np.array(
        [
            [1 - s * (y * y + z * z), s * (x * y - z * w),
             s * (x * z + y * w)],
            [s * (x * y + z * w), 1 - s * (x * x + z * z),
             s * (y * z - x * w)],
            [s * (x * z - y * w), s * (y * z + x * w),
             1 - s * (x * x + y * y)],
        ]
    )


def transform_to_matrix(transform):
    t = transform.transform.translation
    q = transform.transform.rotation
    mat = np.eye(4)
    mat[:3, :3] = quat_to_matrix((q.x, q.y, q.z, q.w))
    mat[:3, 3] = [t.x, t.y, t.z]
    return mat


def build_poses(set_name):
    """返回 (label, distance, dx, dy, tilt_x_deg, tilt_y_deg, roll_deg)。

    distance 是相机光心到板中心的距离；手位由瞄准公式反算。
    """
    if set_name == "pilot":
        return [
            ("aim_d46", 0.46, 0.0, 0.0, 0.0, 0.0, 0.0),
            ("aim_d50", 0.50, 0.0, 0.0, 0.0, 0.0, 0.0),
            ("tilt_xp10", 0.52, 0.0, 0.0, 10.0, 0.0, 0.0),
            ("tilt_xm10", 0.52, 0.0, 0.0, -10.0, 0.0, 0.0),
            ("tilt_yp10", 0.52, 0.0, 0.0, 0.0, 10.0, 0.0),
            ("roll_zp30", 0.50, 0.0, 0.0, 0.0, 0.0, 30.0),
        ]
    return [
        ("d46", 0.46, 0.0, 0.0, 0.0, 0.0, 0.0),
        ("d50", 0.50, 0.0, 0.0, 0.0, 0.0, 0.0),
        ("d54", 0.54, 0.0, 0.0, 0.0, 0.0, 0.0),
        ("x_plus", 0.50, 0.03, 0.0, 0.0, 0.0, 0.0),
        ("x_minus", 0.50, -0.03, 0.0, 0.0, 0.0, 0.0),
        ("y_plus", 0.50, 0.0, 0.03, 0.0, 0.0, 0.0),
        ("y_minus", 0.50, 0.0, -0.03, 0.0, 0.0, 0.0),
        ("xy_plus", 0.50, 0.03, 0.03, 0.0, 0.0, 0.0),
        ("xy_minus", 0.50, -0.03, -0.03, 0.0, 0.0, 0.0),
        ("tilt_xp10", 0.52, 0.0, 0.0, 10.0, 0.0, 0.0),
        ("tilt_xm10", 0.52, 0.0, 0.0, -10.0, 0.0, 0.0),
        ("tilt_yp10", 0.52, 0.0, 0.0, 0.0, 10.0, 0.0),
        ("tilt_ym10", 0.52, 0.0, 0.0, 0.0, -10.0, 0.0),
        ("roll_zp30", 0.50, 0.0, 0.0, 0.0, 0.0, 30.0),
        ("roll_zm30", 0.50, 0.0, 0.0, 0.0, 0.0, -30.0),
    ]


class PoseSampler(CharucoDetect):
    def __init__(self, args):
        super().__init__(args)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.board_x = float(args.board_x)
        self.board_y = float(args.board_y)
        self.board_top = float(args.board_top)

    def wait_tf(self, timeout=10.0):
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            try:
                self.lookup_gripper()
                self.lookup_truth()
                return True
            except Exception:
                self.spin_for(0.2)
        return False

    def lookup_gripper(self):
        transform = self.tf_buffer.lookup_transform(
            "panda_link0", "panda_link8", Time()
        )
        return transform_to_matrix(transform)

    def lookup_truth(self):
        transform = self.tf_buffer.lookup_transform(
            "panda_link8", "wrist_camera_optical_frame", Time()
        )
        return transform_to_matrix(transform)

    def move_to_pose(self, x, y, z, quat):
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
        orientation.orientation.x = float(quat[0])
        orientation.orientation.y = float(quat[1])
        orientation.orientation.z = float(quat[2])
        orientation.orientation.w = float(quat[3])
        # 收紧到约 1.7°，避免冗余 IK 选到让板子偏出画面的姿态
        orientation.absolute_x_axis_tolerance = 0.03
        orientation.absolute_y_axis_tolerance = 0.03
        orientation.absolute_z_axis_tolerance = 0.03
        orientation.weight = 1.0

        constraints = Constraints()
        constraints.position_constraints = [constraint]
        constraints.orientation_constraints = [orientation]
        constraints.name = "charuco_pose"

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

    def sample(self, outdir, frames, set_name):
        if not self.wait_ready():
            print("ERROR: 没有收到 wrist_camera 图像或内参")
            return 1
        if not self.wait_tf():
            print("ERROR: 等不到 panda_link8 TF")
            return 1

        truth_link8_cam = self.lookup_truth()
        records = []
        accepted = 0
        poses = build_poses(set_name)
        p_board = np.array(
            [self.board_x, self.board_y, self.board_top],
            dtype=float,
        )

        for index, (
            label, distance, dx, dy, tilt_x, tilt_y, roll
        ) in enumerate(poses):
            q = BASE_Q.copy()
            for axis, angle_deg in (
                ((1.0, 0.0, 0.0), tilt_x),
                ((0.0, 1.0, 0.0), tilt_y),
                ((0.0, 0.0, 1.0), roll),
            ):
                if abs(angle_deg) > 1e-9:
                    q = quat_mul(
                        q,
                        quat_axis_angle(
                            axis, math.radians(angle_deg)
                        ),
                    )
            q /= np.linalg.norm(q)

            # 让相机光轴穿过板中心，再叠加一个小平移
            r_base_link8 = quat_to_matrix(q)
            r_link8_cam = truth_link8_cam[:3, :3]
            t_link8_cam = truth_link8_cam[:3, 3]
            axis_base = (
                r_base_link8 @ r_link8_cam @ np.array([0.0, 0.0, 1.0])
            )
            axis_base /= np.linalg.norm(axis_base)
            p_cam = p_board - distance * axis_base
            p_link8 = p_cam - r_base_link8 @ t_link8_cam
            target_x = float(p_link8[0] + dx)
            target_y = float(p_link8[1] + dy)
            target_z = float(p_link8[2])

            print(
                f"\n[{index:02d} {label}] "
                f"d={distance:.2f} "
                f"x={target_x:.3f} y={target_y:.3f} "
                f"z={target_z:.3f} tilt_x={tilt_x} "
                f"tilt_y={tilt_y} roll={roll}"
            )
            move_ok = self.move_to_pose(
                target_x, target_y, target_z, q
            )
            if not move_ok:
                print("  MoveIt failed")
                records.append(
                    {
                        "id": index,
                        "label": label,
                        "dist": distance,
                        "dx": dx,
                        "dy": dy,
                        "tilt_x": tilt_x,
                        "tilt_y": tilt_y,
                        "roll": roll,
                        "moveit_ok": False,
                        "accepted": False,
                    }
                )
                continue

            self.spin_for(1.0)
            best = None
            for _ in range(frames):
                image = self.capture(timeout=3.0)
                if image is None:
                    continue
                result, message = self.detect(image)
                if result is None:
                    continue
                if best is None or (
                    result["reprojection"]
                    < best[0]["reprojection"]
                ):
                    best = (result, image)

            if best is None:
                print("  detect failed on all frames")
                records.append(
                    {
                        "id": index,
                        "label": label,
                        "dist": distance,
                        "dx": dx,
                        "dy": dy,
                        "tilt_x": tilt_x,
                        "tilt_y": tilt_y,
                        "roll": roll,
                        "moveit_ok": True,
                        "accepted": False,
                    }
                )
                continue

            result, image = best
            image_path = os.path.join(
                outdir, f"pose_{index:02d}_{label}.png"
            )
            import cv2

            cv2.imwrite(image_path, image)
            t_base_link8 = self.lookup_gripper()
            target_r, _ = cv2.Rodrigues(result["rvec"])
            target_t = result["tvec"].reshape(3)
            target_mat = np.eye(4)
            target_mat[:3, :3] = target_r
            target_mat[:3, 3] = target_t

            ok = (
                result["charuco_count"] >= 24
                and result["reprojection"] < 1.0
            )
            if ok:
                accepted += 1

            gap = float(result.get("reproj_gap", 0.0))
            # 近正视姿态 gap 很小，分支不可靠；倾斜姿态 gap 通常 >1 px
            ambiguous = gap < 0.1
            print(
                f"  markers={result['marker_count']} "
                f"corners={result['charuco_count']} "
                f"reproj={result['reprojection']:.3f}px "
                f"gap={gap:.3f}px ambiguous={ambiguous} "
                f"accepted={ok}"
            )
            records.append(
                {
                    "id": index,
                    "label": label,
                    "dist": distance,
                    "dx": dx,
                    "dy": dy,
                    "tilt_x": tilt_x,
                    "tilt_y": tilt_y,
                    "roll": roll,
                    "moveit_ok": True,
                    "accepted": bool(ok),
                    "ambiguous": bool(ambiguous),
                    "marker_count": result["marker_count"],
                    "charuco_count": result["charuco_count"],
                    "reprojection": result["reprojection"],
                    "reproj_gap": gap,
                    "chosen_solution": result.get(
                        "chosen_solution", 0
                    ),
                    "solutions": result.get("solutions", []),
                    "image": image_path,
                    "target_rvec": result["rvec"].tolist(),
                    "target_tvec": result["tvec"].tolist(),
                    "T_target2cam": target_mat.tolist(),
                    "T_base_link8": t_base_link8.tolist(),
                }
            )

        with open(
            os.path.join(outdir, "poses.json"), "w", encoding="utf-8"
        ) as handle:
            json.dump(
                {
                    "set": set_name,
                    "frames": frames,
                    "truth_T_link8_cam": truth_link8_cam.tolist(),
                    "records": records,
                },
                handle,
                indent=2,
                ensure_ascii=False,
            )

        fieldnames = [
            "id", "label", "dist", "dx", "dy",
            "tilt_x", "tilt_y", "roll", "moveit_ok",
            "accepted", "ambiguous", "marker_count",
            "charuco_count", "reprojection", "reproj_gap",
            "chosen_solution", "image",
        ]
        with open(
            os.path.join(outdir, "poses.csv"), "w",
            newline="", encoding="utf-8",
        ) as handle:
            writer = csv.DictWriter(handle, fieldnames=fieldnames)
            writer.writeheader()
            for record in records:
                writer.writerow(
                    {key: record.get(key, "") for key in fieldnames}
                )

        print(
            f"\naccepted {accepted}/{len(poses)} poses"
        )
        print(f"output: {outdir}")
        return 0 if accepted >= 6 else 1


def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--set", dest="set_name", choices=["pilot", "full"],
        default="pilot",
    )
    parser.add_argument("--frames", type=int, default=3)
    parser.add_argument("--board-x", type=float, default=BOARD_X)
    parser.add_argument("--board-y", type=float, default=BOARD_Y)
    parser.add_argument("--board-top", type=float, default=BOARD_TOP)
    parsed = parser.parse_args(args)

    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    outdir = os.path.abspath(
        os.path.join(
            os.path.dirname(__file__),
            "..",
            "output",
            f"charuco_poses_{parsed.set_name}_{stamp}",
        )
    )
    os.makedirs(outdir, exist_ok=True)

    rclpy.init()
    node = PoseSampler(parsed)
    try:
        return node.sample(
            outdir, parsed.frames, parsed.set_name
        )
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
