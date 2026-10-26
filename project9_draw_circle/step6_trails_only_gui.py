"""
第 6 天 · 简化版：只看三条动态轨迹
=====================================
不放机械臂，只保留：

  - 三个圆（constant / min-jerk / adaptive）
  - 三条动态生长的轨迹
  - 目标点、实际点
  - 误差箭头（放大 20 倍）
  - 实时误差 / 关节速度读数

俯视相机，三条轨迹并排，不会再重叠。

数据仍然来自真实的 PyBullet 动力学仿真，只是渲染时隐藏了机器人。
"""

import csv
import math
import os
import time

import numpy as np
import pybullet as p
import pybullet_data

import step2_circle_timing as base
import step3_adaptive_circle as adapt
from screenshot_util import capture_window


BASE_DIR = os.path.dirname(os.path.abspath(__file__))
OUT_DIR = os.path.join(BASE_DIR, "output")

STEPS = base.STEPS
CENTER = base.CENTER
RADIUS = base.RADIUS
LANES = [-0.22, 0.0, 0.22]
NAMES = ["constant", "minjerk", "adaptive"]
COLORS = {
    "constant": [0.10, 0.40, 1.00],
    "minjerk": [1.00, 0.50, 0.00],
    "adaptive": [0.00, 0.70, 0.20],
}
ERROR_GAIN = 20.0
FRAMES = 240
FPS = 30


# ============================================================
# 1. 先用 headless 仿真收集三条轨迹
# ============================================================
def collect_data():
    base.connect(False)
    panda = base.build_scene()
    movable = [j for j in range(p.getNumJoints(panda))
               if p.getJointInfo(panda, j)[2] != p.JOINT_FIXED]

    print("warmup ...")
    q_start = base.warmup_to_start(panda, False)

    print("constant ...")
    theta_const = np.linspace(0, 2 * math.pi, STEPS)
    const_rows, _ = adapt.run_schedule(panda, movable, q_start, theta_const)
    sig = np.array([r["sigma"] for r in const_rows])

    print("min-jerk ...")
    u = np.linspace(0, 1, STEPS)
    theta_min = 2 * math.pi * np.array([base.minjerk_s(x) for x in u])
    min_rows, _ = adapt.run_schedule(panda, movable, q_start, theta_min)

    print("adaptive ...")
    theta_adapt, _, _ = adapt.build_adaptive_schedule(sig)
    adapt_rows, _ = adapt.run_schedule(panda, movable, q_start, theta_adapt)
    p.disconnect()

    def extract(rows):
        target = np.array([r["target"] for r in rows])
        actual = np.array([r["actual"] for r in rows])
        qd = np.array([r["qd"] for r in rows])
        err = np.linalg.norm(actual - target, axis=1)
        return {
            "target_local": target[:, :2] - CENTER[:2],
            "actual_local": actual[:, :2] - CENTER[:2],
            "err": err,
            "qdmax": np.max(np.abs(qd), axis=1),
        }

    return {
        "constant": extract(const_rows),
        "minjerk": extract(min_rows),
        "adaptive": extract(adapt_rows),
    }


# ============================================================
# 2. 渲染
# ============================================================
def map_to_lane(local_xy, lane_y):
    return np.array([0.50 + local_xy[0],
                     lane_y + local_xy[1],
                     0.50])


def draw_ideal_circles():
    for name, y in zip(NAMES, LANES):
        prev = None
        for k in range(121):
            a = 2 * math.pi * k / 120.0
            pt = map_to_lane(
                np.array([RADIUS * math.cos(a),
                          RADIUS * math.sin(a)]), y)
            if prev is not None:
                p.addUserDebugLine(prev.tolist(), pt.tolist(),
                                   [0.55, 0.55, 0.55], lineWidth=1.0,
                                   lifeTime=0)
            prev = pt
        p.addUserDebugText(name, [0.36, y, 0.56],
                           textColorRGB=COLORS[name], textSize=1.0,
                           lifeTime=0)
    p.addUserDebugText("error arrows x%d" % int(ERROR_GAIN),
                       [0.36, -0.30, 0.60], textColorRGB=[0, 0, 0],
                       textSize=1.0, lifeTime=0)
    p.addUserDebugText("all three finish at t = 8.0 s",
                       [0.36, -0.34, 0.56], textColorRGB=[0, 0, 0],
                       textSize=0.9, lifeTime=0)


def make_sphere(pos, color, radius):
    vis = p.createVisualShape(p.GEOM_SPHERE, radius=radius,
                              rgbaColor=[color[0], color[1], color[2], 1.0])
    return p.createMultiBody(baseMass=0, baseVisualShapeIndex=vis,
                             basePosition=pos.tolist())


def update_sphere(obj, pos):
    p.resetBasePositionAndOrientation(obj, pos.tolist(), [0, 0, 0, 1])


def make_gauge(name, lane_y):
    return p.addUserDebugText(name, [0.36, lane_y, 0.52],
                              textColorRGB=COLORS[name], textSize=0.85,
                              lifeTime=0)


def update_gauge(item_id, name, lane_y, err, qd):
    if item_id is not None:
        p.removeUserDebugItem(item_id)
    return p.addUserDebugText(
        "%s err %.1fmm qdot %.2f" % (name, err * 1000, qd),
        [0.36, lane_y, 0.52], textColorRGB=COLORS[name],
        textSize=0.85, lifeTime=0)


def main():
    data = collect_data()
    os.makedirs(OUT_DIR, exist_ok=True)

    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.loadURDF("plane.urdf")
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
    p.resetDebugVisualizerCamera(
        cameraDistance=1.15, cameraYaw=0, cameraPitch=-89.0,
        cameraTargetPosition=[0.50, 0.0, 0.50],
    )
    draw_ideal_circles()

    targets = []
    actuals = []
    gauges = []
    for name, y in zip(NAMES, LANES):
        targets.append(make_sphere(
            map_to_lane([0.0, 0.0], y), [0.85, 0.85, 0.85], 0.006))
        actuals.append(make_sphere(
            map_to_lane([0.0, 0.0], y), COLORS[name], 0.010))
        gauges.append(make_gauge(name, y))

    trail_prev = [None, None, None]
    max_err = [0.0, 0.0, 0.0]
    peak_qd = [0.0, 0.0, 0.0]
    clock = p.addUserDebugText(
        "t = 0.0 s / 8.0 s", [0.36, -0.38, 0.52],
        textColorRGB=[0, 0, 0], textSize=1.0, lifeTime=0)

    for frame in range(FRAMES):
        idx = int(round(frame / (FRAMES - 1) * (STEPS - 1)))
        for i, (name, y) in enumerate(zip(NAMES, LANES)):
            d = data[name]
            target = map_to_lane(d["target_local"][idx], y)
            actual = map_to_lane(d["actual_local"][idx], y)
            err = float(d["err"][idx])
            qd = float(d["qdmax"][idx])
            max_err[i] = max(max_err[i], err)
            peak_qd[i] = max(peak_qd[i], qd)

            if trail_prev[i] is None:
                trail_prev[i] = actual
            if np.linalg.norm(actual - trail_prev[i]) > 1e-6:
                p.addUserDebugLine(trail_prev[i].tolist(), actual.tolist(),
                                   COLORS[name], lineWidth=3.0,
                                   lifeTime=0)
                trail_prev[i] = actual

            update_sphere(targets[i], target)
            update_sphere(actuals[i], actual)

            # 误差放大箭头
            local_err = target + ERROR_GAIN * (actual - target)
            p.addUserDebugLine(target.tolist(), local_err.tolist(),
                               COLORS[name], lineWidth=5.0,
                               lifeTime=0.12)

            if frame % 10 == 0:
                gauges[i] = update_gauge(
                    gauges[i], name, y, err, peak_qd[i])

        if frame % 10 == 0:
            p.removeUserDebugItem(clock)
            clock = p.addUserDebugText(
                "t = %.1f s / 8.0 s" % (frame / (FRAMES - 1) * 8.0),
                [0.36, -0.38, 0.52], textColorRGB=[0, 0, 0],
                textSize=1.0, lifeTime=0)

        p.stepSimulation()
        time.sleep(1.0 / FPS)

    p.removeUserDebugItem(clock)
    p.addUserDebugText("t = 8.0 s  —  ALL FINISH",
                       [0.36, -0.38, 0.52], textColorRGB=[0, 0, 0],
                       textSize=1.0, lifeTime=0)

    # 末状态也保留误差箭头，截图时能看到最终误差方向和大小
    for i, (name, y) in enumerate(zip(NAMES, LANES)):
        d = data[name]
        target = map_to_lane(d["target_local"][-1], y)
        actual = map_to_lane(d["actual_local"][-1], y)
        tip = target + ERROR_GAIN * (actual - target)
        p.addUserDebugLine(target.tolist(), tip.tolist(),
                           COLORS[name], lineWidth=5.0, lifeTime=0)

    for i, (name, y) in enumerate(zip(NAMES, LANES)):
        p.removeUserDebugItem(gauges[i])
        p.addUserDebugText(
            "%s: max err %.1fmm, peak qdot %.2f"
            % (name, max_err[i] * 1000, peak_qd[i]),
            [0.36, y, 0.46], textColorRGB=COLORS[name],
            textSize=0.9, lifeTime=0)

    csv_path = os.path.join(OUT_DIR, "three_trails_gui.csv")
    with open(csv_path, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["timing", "max_err_mm", "peak_qdot"])
        for i, name in enumerate(NAMES):
            w.writerow([name, f"{max_err[i] * 1000:.3f}",
                        f"{peak_qd[i]:.3f}"])
    print("CSV:", csv_path)
    time.sleep(1.0)
    capture_window(os.path.join(OUT_DIR, "trails_only_final.png"))
    print("window stays open for 8 seconds.")
    time.sleep(8.0)
    p.disconnect()


if __name__ == "__main__":
    try:
        main()
    except p.error as exc:
        print("仿真正在退出：", exc)
        try:
            p.disconnect()
        except p.error:
            pass
