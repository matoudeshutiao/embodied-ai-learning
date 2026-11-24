#!/usr/bin/env python3
"""腕部相机红方块视觉：标定加载、深度反投影、顶面平面拟合。

不依赖 ROS，供 detector 和 servo 共用。
"""

import json
import os

import cv2
import numpy as np

CUBE_HALF = 0.025
COLOR_RANGES = [
    ((0, 100, 60), (10, 255, 255)),
    ((170, 100, 60), (180, 255, 255)),
]


def load_calibration(path=None):
    if path is None:
        path = os.path.abspath(
            os.path.join(
                os.path.dirname(__file__),
                "..",
                "config",
                "wrist_hand_eye.json",
            )
        )
    with open(path, encoding="utf-8") as handle:
        data = json.load(handle)
    return np.asarray(data["T_link8_cam"], dtype=float), data


def depth_to_meters(depth):
    if depth.dtype == np.uint16:
        return depth.astype(np.float32) * 0.001
    return depth.astype(np.float32)


def red_mask(image):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    mask = None
    for lower, upper in COLOR_RANGES:
        part = cv2.inRange(
            hsv,
            np.array(lower, dtype=np.uint8),
            np.array(upper, dtype=np.uint8),
        )
        mask = part if mask is None else cv2.bitwise_or(mask, part)
    return cv2.morphologyEx(
        mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8)
    )


def largest_contour(mask, min_area=50.0):
    contours, _ = cv2.findContours(
        mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )
    if not contours:
        return None
    contour = max(contours, key=cv2.contourArea)
    if cv2.contourArea(contour) < min_area:
        return None
    return contour


def backproject(depth, contour, K, T_base_cam):
    """把轮廓内有效深度点反投影到 base。"""
    roi = np.zeros(depth.shape, dtype=np.uint8)
    cv2.drawContours(roi, [contour], -1, 255, -1)
    rows, cols = np.nonzero(roi > 0)
    z = depth[rows, cols]
    valid = np.isfinite(z) & (z > 0.0)
    rows, cols, z = rows[valid], cols[valid], z[valid]
    if rows.size == 0:
        return None, None
    fx, fy, cx, cy = K[0], K[1], K[2], K[3]
    points_cam = np.column_stack(
        [
            (cols - cx) * z / fx,
            (rows - cy) * z / fy,
            z,
        ]
    )
    points_base = (
        T_base_cam[:3, :3] @ points_cam.T
    ).T + T_base_cam[:3, 3]
    uv = np.column_stack([cols, rows])
    return points_base, uv


def fit_plane_ransac(points, threshold=0.003, iterations=300, seed=0):
    """在 base 坐标里找点数最多的平面，返回法向、质心和 inlier。"""
    if points.shape[0] < 50:
        return None
    rng = np.random.default_rng(seed)
    best = None
    for _ in range(iterations):
        sample = rng.choice(points.shape[0], 3, replace=False)
        p0, p1, p2 = points[sample]
        normal = np.cross(p1 - p0, p2 - p0)
        norm = float(np.linalg.norm(normal))
        if norm < 1e-9:
            continue
        normal /= norm
        distance = -float(np.dot(normal, p0))
        inliers = np.abs(points @ normal + distance) < threshold
        count = int(inliers.sum())
        if best is None or count > best[0]:
            best = (count, normal, distance, inliers)
    if best is None or best[0] < 40:
        return None

    # 用 inlier 做一次最小二乘平面精修
    inlier_points = points[best[3]]
    center = inlier_points.mean(axis=0)
    centered = inlier_points - center
    _, _, vh = np.linalg.svd(centered, full_matrices=False)
    normal = vh[-1]
    normal /= np.linalg.norm(normal)
    if normal[2] < 0.0:
        normal = -normal
    return {
        "normal": normal,
        "center": center,
        "inliers": inlier_points,
        "count": int(inlier_points.shape[0]),
    }


def estimate_cube(image, depth, K, T_base_cam):
    mask = red_mask(image)
    contour = largest_contour(mask)
    if contour is None:
        return None, "no red contour"

    points, uv = backproject(depth, contour, K, T_base_cam)
    if points is None or points.shape[0] < 50:
        return None, "too few red depth points"

    plane = fit_plane_ransac(points)
    if plane is None:
        return None, "no dominant top plane"

    top_center = plane["inliers"].mean(axis=0)
    normal = plane["normal"]
    cube_center = top_center - CUBE_HALF * normal
    return {
        "contour": contour,
        "mask": mask,
        "top_center": top_center,
        "cube_center": cube_center,
        "normal": normal,
        "plane_inliers": plane["count"],
        "red_points": int(points.shape[0]),
        "uv": uv,
    }, "ok"


def draw_debug(image, result, text_lines):
    debug = image.copy()
    if result is not None:
        cv2.drawContours(
            debug, [result["contour"]], -1, (0, 255, 0), 2
        )
        uv = result["uv"]
        center_uv = uv.mean(axis=0)
        cv2.circle(
            debug,
            (int(center_uv[0]), int(center_uv[1])),
            4,
            (0, 255, 255),
            -1,
        )
    for index, line in enumerate(text_lines):
        cv2.putText(
            debug,
            line,
            (6, 18 + 18 * index),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (255, 255, 255),
            1,
        )
    return debug
