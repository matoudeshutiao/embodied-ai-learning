"""
第 6 天 · 三机械臂并排动画对比
====================================
三个 Panda 同时画同一个圆，但速度分配不同：

  左边（蓝）：constant
  中间（橙）：min-jerk
  右边（绿）：adaptive（按 sigma_min 调速）

俯视相机 + 分开放置，三条轨迹不再重叠。
误差用放大 20 倍的箭头画出来，并配实时数值，否则 1~5 mm
在整幅场景里肉眼根本看不见。

运行：
  python project9_draw_circle/step5_three_arms_gui.py
"""

import csv
import math
import os
import time

import numpy as np
import pybullet as p
import pybullet_data
from PIL import Image, ImageDraw, ImageFont

import step2_circle_timing as base
import step3_adaptive_circle as adapt
from screenshot_util import capture_window


BASE_DIR = os.path.dirname(os.path.abspath(__file__))
OUT_DIR = os.path.join(BASE_DIR, "output")

DT = base.DT
STEPS = base.STEPS
T_TOTAL = base.T_TOTAL
RADIUS = base.RADIUS
LOCAL_CENTER = np.array([0.50, 0.0, 0.35])
LOCAL_START = LOCAL_CENTER + np.array([RADIUS, 0.0, 0.0])
TARGET_ORN = base.TARGET_ORN
EE_LINK = 11

# 三个底座沿 y 方向分开，避免重叠
BASE_Y = [-0.80, 0.0, 0.80]
NAMES = ["constant", "minjerk", "adaptive"]
RGB = {
    "constant": [0.10, 0.40, 1.00],
    "minjerk": [1.00, 0.50, 0.00],
    "adaptive": [0.00, 0.70, 0.20],
}
ERROR_GAIN = 20.0


def world_center(base):
    return np.array(base, dtype=float) + LOCAL_CENTER


def world_target(base, theta):
    c = world_center(base)
    return c + np.array([RADIUS * math.cos(theta),
                         RADIUS * math.sin(theta), 0.0])


def connect():
    base.connect(True)
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
    # 俯视，三个机械臂沿 y 方向并排
    p.resetDebugVisualizerCamera(
        cameraDistance=2.4, cameraYaw=0, cameraPitch=-89.0,
        cameraTargetPosition=[0.55, 0.0, 0.30],
    )


def build_arms():
    p.loadURDF("plane.urdf")
    arms = []
    for y in BASE_Y:
        arm = p.loadURDF("franka_panda/panda.urdf",
                         basePosition=[0.0, y, 0.0], useFixedBase=True)
        default_pose = [0, -0.785, 0, -2.356, 0, 1.571, 0.785]
        for i, q in enumerate(default_pose):
            p.resetJointState(arm, i, q)
        for j in range(p.getNumJoints(arm)):
            if b"finger" in p.getJointInfo(arm, j)[1]:
                p.setJointMotorControl2(arm, j, p.POSITION_CONTROL,
                                        targetPosition=0.04, force=200)
        arms.append(arm)
    return arms


def ee_pos(arm):
    return np.array(p.getLinkState(arm, EE_LINK,
                                   computeForwardKinematics=True)[4],
                    dtype=float)


def ik(arm, target):
    q = p.calculateInverseKinematics(
        arm, EE_LINK, target.tolist(), TARGET_ORN,
        maxNumIterations=200, residualThreshold=1e-8,
    )
    return np.array(q[:7], dtype=float)


def command(arm, q_cmd):
    for j, q in enumerate(q_cmd):
        p.setJointMotorControl2(arm, j, p.POSITION_CONTROL,
                                targetPosition=float(q), force=200)


def joint_qd(arm):
    qd = []
    for j in range(7):
        qd.append(p.getJointState(arm, j)[1])
    return np.array(qd)


def jacobian_sigma(arm):
    movable = [j for j in range(p.getNumJoints(arm))
               if p.getJointInfo(arm, j)[2] != p.JOINT_FIXED]
    pos = [p.getJointState(arm, j)[0] for j in movable]
    z = [0.0] * len(pos)
    Jl, Ja = p.calculateJacobian(arm, EE_LINK, [0.0, 0.0, 0.0],
                                 pos, z, z)
    Jv = np.array(Jl, dtype=float)[:, :7]
    Jw = np.array(Ja, dtype=float)[:, :7]
    J = np.vstack([Jv, Jw])
    return float(np.linalg.svd(J, compute_uv=False).min())


def warmup_all(arms):
    """三个手臂一起走到各自圆的起点，不录入动画。"""
    initial = [ee_pos(a) for a in arms]
    starts = [np.array([0.0, y, 0.0]) + LOCAL_START
              for y in BASE_Y]
    for k in range(base.WARMUP_STEPS):
        s = k / (base.WARMUP_STEPS - 1)
        for arm, p0, p1 in zip(arms, initial, starts):
            command(arm, ik(arm, p0 + s * (p1 - p0)))
        p.stepSimulation()
    for _ in range(180):
        for arm, p1 in zip(arms, starts):
            command(arm, ik(arm, p1))
        p.stepSimulation()
    q_starts = []
    for arm in arms:
        q_starts.append(np.array([p.getJointState(arm, j)[0]
                                  for j in range(7)]))
    return q_starts


def build_sigma_map(arm, q_start, theta_grid):
    """用中间的手臂做一个快速运动学扫描，得到 sigma_min(theta)。"""
    for j, q in enumerate(q_start):
        p.resetJointState(arm, j, float(q), 0.0)
    sig = []
    for theta in theta_grid:
        target = world_target([0.0, BASE_Y[1], 0.0], theta)
        q = ik(arm, target)
        for j, qq in enumerate(q):
            p.resetJointState(arm, j, float(qq), 0.0)
        sig.append(jacobian_sigma(arm))
    for j, q in enumerate(q_start):
        p.resetJointState(arm, j, float(q), 0.0)
    return np.array(sig)


def draw_ideal_circles():
    for y in BASE_Y:
        prev = None
        for k in range(121):
            a = 2 * math.pi * k / 120.0
            pt = world_target([0.0, y, 0.0], a)
            if prev is not None:
                p.addUserDebugLine(prev.tolist(), pt.tolist(),
                                   [0.55, 0.55, 0.55], lineWidth=1.0,
                                   lifeTime=0)
            prev = pt
        p.addUserDebugText(
            NAMES[BASE_Y.index(y)],
            [0.30, y, 0.55], textColorRGB=RGB[NAMES[BASE_Y.index(y)]],
            textSize=1.0, lifeTime=0)
    p.addUserDebugText("error arrows x%d" % int(ERROR_GAIN),
                       [0.30, -1.35, 0.62], textColorRGB=[0, 0, 0],
                       textSize=1.0, lifeTime=0)


def make_marker(pos, color):
    vis = p.createVisualShape(p.GEOM_SPHERE, radius=0.010,
                              rgbaColor=[color[0], color[1], color[2], 1.0])
    return p.createMultiBody(baseMass=0, baseVisualShapeIndex=vis,
                             basePosition=pos.tolist())


def draw_trail(prev, actual, color):
    if np.linalg.norm(actual - prev) > 1e-6:
        p.addUserDebugLine(prev.tolist(), actual.tolist(), color,
                           lineWidth=3.0, lifeTime=0)


def draw_error_arrow(target, actual, color, life_time=0.15):
    """把真实误差放大 ERROR_GAIN 倍，从目标点画出去。"""
    tip = target + ERROR_GAIN * (actual - target)
    p.addUserDebugLine(target.tolist(), tip.tolist(), color,
                       lineWidth=5.0, lifeTime=life_time)


def make_gauge(name, y):
    return p.addUserDebugText(
        name, [0.28, y, 0.50], textColorRGB=RGB[name],
        textSize=0.9, lifeTime=0)


def update_gauge(item_id, name, y, err, qd, peak_qd):
    if item_id is not None:
        p.removeUserDebugItem(item_id)
    return p.addUserDebugText(
        "%s err %.1fmm qdot %.2f/%.2f" % (name, err * 1000, qd, peak_qd),
        [0.28, y, 0.50], textColorRGB=RGB[name],
        textSize=0.85, lifeTime=0)


def run_animation(arms, q_starts, theta_schedules):
    colors = [RGB[n] for n in NAMES]
    starts = [np.array([0.0, y, 0.0]) + LOCAL_START for y in BASE_Y]
    markers = [make_marker(starts[i], colors[i]) for i in range(3)]
    gauges = [make_gauge(NAMES[i], BASE_Y[i]) for i in range(3)]
    trail_prev = [ee_pos(a) for a in arms]
    peak_qd = [0.0, 0.0, 0.0]
    max_err = [0.0, 0.0, 0.0]
    mean_err = [0.0, 0.0, 0.0]
    count = 0

    for k in range(STEPS):
        for i, arm in enumerate(arms):
            target = world_target([0.0, BASE_Y[i], 0.0],
                                  theta_schedules[i][k])
            command(arm, ik(arm, target))
        p.stepSimulation()
        time.sleep(DT)

        for i, arm in enumerate(arms):
            target = world_target([0.0, BASE_Y[i], 0.0],
                                  theta_schedules[i][k])
            actual = ee_pos(arm)
            qd = joint_qd(arm)
            err = float(np.linalg.norm(actual - target))
            cur_qd = float(np.max(np.abs(qd)))
            peak_qd[i] = max(peak_qd[i], cur_qd)
            max_err[i] = max(max_err[i], err)
            mean_err[i] += err
            p.resetBasePositionAndOrientation(
                markers[i], target.tolist(), [0, 0, 0, 1])
            if k % 4 == 0:
                draw_trail(trail_prev[i], actual, colors[i])
                draw_error_arrow(target, actual, colors[i])
                trail_prev[i] = actual
            if k % 24 == 0:
                gauges[i] = update_gauge(
                    gauges[i], NAMES[i], BASE_Y[i], err, cur_qd,
                    peak_qd[i])
        count += 1

    # 最后留一支“放大误差”箭头，截图时也能看到误差方向
    for i, arm in enumerate(arms):
        target = world_target([0.0, BASE_Y[i], 0.0],
                              theta_schedules[i][-1])
        actual = ee_pos(arm)
        draw_error_arrow(target, actual, colors[i], life_time=0)

    for i in range(3):
        p.removeBody(markers[i])
        p.removeUserDebugItem(gauges[i])
        p.addUserDebugText(
            "%s: max err %.1fmm, peak qdot %.2f"
            % (NAMES[i], max_err[i] * 1000, peak_qd[i]),
            [0.28, BASE_Y[i], 0.44], textColorRGB=RGB[NAMES[i]],
            textSize=0.9, lifeTime=0)

    return {
        "names": NAMES,
        "max_err": max_err,
        "mean_err": [x / max(count, 1) for x in mean_err],
        "peak_qd": peak_qd,
    }


def capture_triptych(result):
    """每个机械臂单独拍一张近景，再拼成三联图。"""
    paths = []
    for i in range(3):
        center = np.array([0.0, BASE_Y[i], 0.0]) + LOCAL_CENTER
        p.resetDebugVisualizerCamera(
            cameraDistance=0.95, cameraYaw=35, cameraPitch=-30,
            cameraTargetPosition=center.tolist(),
        )
        p.stepSimulation()
        time.sleep(0.5)
        path = os.path.join(OUT_DIR, "_arm_%s.png" % NAMES[i])
        if capture_window(path):
            paths.append(path)

    if not paths:
        return
    imgs = [Image.open(p).convert("RGB") for p in paths]
    target_h = 520
    imgs = [im.resize((int(im.width * target_h / im.height), target_h))
            for im in imgs]
    header = 105
    total_w = sum(im.width for im in imgs)
    canvas = Image.new("RGB", (total_w, target_h + header), "white")
    draw = ImageDraw.Draw(canvas)
    try:
        font = ImageFont.truetype("C:/Windows/Fonts/arial.ttf", 30)
        small = ImageFont.truetype("C:/Windows/Fonts/arial.ttf", 22)
    except Exception:
        font = ImageFont.load_default()
        small = ImageFont.load_default()

    x = 0
    for i, im in enumerate(imgs):
        canvas.paste(im, (x, header))
        draw.text((x + 18, 18), NAMES[i], fill=tuple(
            int(c * 255) for c in RGB[NAMES[i]]), font=font)
        draw.text(
            (x + 18, 58),
            "max err %.1f mm  |  peak qdot %.2f rad/s"
            % (result["max_err"][i] * 1000, result["peak_qd"][i]),
            fill=(0, 0, 0), font=small)
        x += im.width

    out = os.path.join(OUT_DIR, "three_arms_final_triptych.png")
    canvas.save(out)
    print("triptych:", out)


def main():
    os.makedirs(OUT_DIR, exist_ok=True)
    connect()
    arms = build_arms()
    draw_ideal_circles()
    print("warmup ...")
    q_starts = warmup_all(arms)
    print("building sigma map ...")
    theta_grid = np.linspace(0, 2 * math.pi, STEPS)
    sig = build_sigma_map(arms[1], q_starts[1], theta_grid)

    u = np.linspace(0, 1, STEPS)
    theta_const = 2 * math.pi * u
    theta_min = 2 * math.pi * np.array([base.minjerk_s(x) for x in u])
    theta_adapt, _, _ = adapt.build_adaptive_schedule(sig)

    print("three arms running ...")
    result = run_animation(
        arms, q_starts, [theta_const, theta_min, theta_adapt])

    csv_path = os.path.join(OUT_DIR, "three_arms_gui.csv")
    with open(csv_path, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["timing", "max_err_mm", "mean_err_mm", "peak_qdot"])
        for i, name in enumerate(NAMES):
            w.writerow([name,
                        f"{result['max_err'][i] * 1000:.3f}",
                        f"{result['mean_err'][i] * 1000:.3f}",
                        f"{result['peak_qd'][i]:.3f}"])
    print("CSV:", csv_path)
    print("error arrows are magnified x%d" % int(ERROR_GAIN))
    time.sleep(1.0)
    capture_triptych(result)
    print("window will stay open for 10 seconds.")
    time.sleep(10.0)
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
