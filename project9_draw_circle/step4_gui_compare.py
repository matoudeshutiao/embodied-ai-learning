"""
第 6 天 · 一小时趣味实验 · GUI 动画对比
==========================================
依次演示同一个圆的三种速度分配：

  constant  匀速
  min-jerk  起停平滑，但中段峰值速度更高
  adaptive  弱扇区慢、强区快（总时间仍 8 s）

运行：
  python project9_draw_circle/step4_gui_compare.py
"""

import csv
import math
import os
import time

import numpy as np
import pybullet as p

import step2_circle_timing as base
import step3_adaptive_circle as adapt


BASE_DIR = os.path.dirname(os.path.abspath(__file__))
OUT_DIR = os.path.join(BASE_DIR, "output")

STEPS = base.STEPS
DT = base.DT
T_TOTAL = base.T_TOTAL
CENTER = base.CENTER
RADIUS = base.RADIUS
START = base.START

RGB = {
    "constant": [0.10, 0.40, 1.00],
    "minjerk": [1.00, 0.50, 0.00],
    "adaptive": [0.00, 0.70, 0.20],
}


def draw_ideal_circle():
    prev = None
    for k in range(121):
        a = 2 * math.pi * k / 120.0
        pt = CENTER + np.array([RADIUS * math.cos(a),
                                RADIUS * math.sin(a), 0.0])
        if prev is not None:
            p.addUserDebugLine(prev.tolist(), pt.tolist(),
                               [0.55, 0.55, 0.55], lineWidth=1.2,
                               lifeTime=0)
        prev = pt
    p.addUserDebugText(
        "blue=constant   orange=min-jerk   green=adaptive",
        [0.35, 0.28, 0.58], textColorRGB=[0, 0, 0],
        textSize=1.0, lifeTime=0)


def make_marker(color):
    vis = p.createVisualShape(p.GEOM_SPHERE, radius=0.010,
                              rgbaColor=[color[0], color[1], color[2], 1.0])
    return p.createMultiBody(baseMass=0, baseVisualShapeIndex=vis,
                             basePosition=START.tolist())


def draw_trail(prev, actual, color):
    if np.linalg.norm(actual - prev) > 1e-6:
        p.addUserDebugLine(prev.tolist(), actual.tolist(), color,
                           lineWidth=3.2, lifeTime=0)


def make_gauge(text, color):
    return p.addUserDebugText(text, [0.35, 0.32, 0.54],
                              textColorRGB=color, textSize=0.9,
                              lifeTime=0)


def replace_gauge(item_id, text, color):
    if item_id is not None:
        p.removeUserDebugItem(item_id)
    return make_gauge(text, color)


def run_gui(panda, movable, q_start, theta_schedule, name):
    base.reset_to_start(panda, q_start, True)
    color = RGB[name]
    marker = make_marker(color)
    gauge = make_gauge(name, color)
    trail_prev = base.ee_pos(panda)
    rows = []
    peak_qd = 0.0

    for k, theta in enumerate(theta_schedule):
        target = CENTER + np.array([RADIUS * math.cos(theta),
                                    RADIUS * math.sin(theta), 0.0])
        q_cmd = base.solve_ik(panda, target)
        base.command_arm(panda, q_cmd)
        p.stepSimulation()
        time.sleep(DT)
        p.resetBasePositionAndOrientation(
            marker, target.tolist(), [0, 0, 0, 1])

        actual = base.ee_pos(panda)
        q, qd = base.joint_state(panda)
        sigma = adapt.jacobian_sigma(panda, movable)
        peak_qd = max(peak_qd, float(np.max(np.abs(qd))))
        rows.append({
            "theta": float(theta),
            "target": target.copy(),
            "actual": actual.copy(),
            "err": actual - target,
            "q": q.copy(),
            "qd": qd.copy(),
            "sigma": sigma,
        })

        if k % 4 == 0:
            draw_trail(trail_prev, actual, color)
            trail_prev = actual
        if k % 24 == 0:
            gauge = replace_gauge(
                gauge,
                f"{name}  sigma={sigma:.3f}  qdot={peak_qd:.2f}  "
                f"err={np.linalg.norm(actual - target) * 1000:.1f}mm",
                color)

    base.hold_ik(panda, START, 240, True)
    final = base.ee_pos(panda)
    p.removeBody(marker)
    if gauge is not None:
        p.removeUserDebugItem(gauge)
    return rows, final


def print_table(results):
    print("\n===== GUI comparison =====")
    for r in results:
        print("%-9s max err %5.1f mm | mean %4.1f mm | max qdot %.2f | "
              "weak err %5.1f mm | weak qdot %.2f"
              % (r["name"], r["max_err"] * 1000, r["mean_err"] * 1000,
                 r["max_qd"], r["max_sigma_err"] * 1000,
                 r["max_sigma_qd"]))


def save_csv(results, weak_lo, weak_hi, path):
    fields = ["timing", "weak_lo_deg", "weak_hi_deg", "max_err_mm",
              "mean_err_mm", "max_qdot", "weak_max_err_mm",
              "weak_max_qdot", "closure_mm", "path_len_cm"]
    with open(path, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(fields)
        for r in results:
            w.writerow([
                r["name"], f"{math.degrees(weak_lo):.1f}",
                f"{math.degrees(weak_hi):.1f}",
                f"{r['max_err'] * 1000:.3f}",
                f"{r['mean_err'] * 1000:.3f}",
                f"{r['max_qd']:.3f}",
                f"{r['max_sigma_err'] * 1000:.3f}",
                f"{r['max_sigma_qd']:.3f}",
                f"{r['close_err'] * 1000:.3f}",
                f"{r['path_len'] * 100:.3f}",
            ])


def main():
    os.makedirs(OUT_DIR, exist_ok=True)
    base.connect(True)
    panda = base.build_scene()
    movable = [j for j in range(p.getNumJoints(panda))
               if p.getJointInfo(panda, j)[2] != p.JOINT_FIXED]
    draw_ideal_circle()

    print("warmup ...")
    q_start = base.warmup_to_start(panda, True)

    print("[1/3] constant ...")
    theta_const = np.linspace(0, 2 * math.pi, STEPS)
    const_rows, const_final = run_gui(
        panda, movable, q_start, theta_const, "constant")
    sig = np.array([r["sigma"] for r in const_rows])
    i_min = int(np.argmin(sig))
    weak_center = theta_const[i_min]
    weak_lo = max(0.0, weak_center - math.radians(20))
    weak_hi = min(2 * math.pi, weak_center + math.radians(20))

    print("[2/3] min-jerk ...")
    u = np.linspace(0, 1, STEPS)
    theta_min = 2 * math.pi * np.array([base.minjerk_s(x) for x in u])
    min_rows, min_final = run_gui(
        panda, movable, q_start, theta_min, "minjerk")

    print("[3/3] adaptive ...")
    theta_adapt, _, _ = adapt.build_adaptive_schedule(sig)
    adapt_rows, adapt_final = run_gui(
        panda, movable, q_start, theta_adapt, "adaptive")

    results = [
        adapt.analyse(const_rows, const_final, "constant",
                      weak_lo, weak_hi),
        adapt.analyse(min_rows, min_final, "minjerk",
                      weak_lo, weak_hi),
        adapt.analyse(adapt_rows, adapt_final, "adaptive",
                      weak_lo, weak_hi),
    ]
    print_table(results)
    csv_path = os.path.join(OUT_DIR, "circle_gui_compare.csv")
    save_csv(results, weak_lo, weak_hi, csv_path)
    print("CSV :", csv_path)

    p.addUserDebugText(
        f"weak sector {math.degrees(weak_lo):.0f}°~{math.degrees(weak_hi):.0f}°",
        [0.35, 0.36, 0.50], textColorRGB=[0, 0, 0],
        textSize=0.9, lifeTime=0)
    for idx, r in enumerate(results):
        p.addUserDebugText(
            f"{r['name']}: err {r['max_err'] * 1000:.1f}mm  "
            f"qdot {r['max_qd']:.2f}",
            [0.35, 0.40 + idx * 0.04, 0.46 - idx * 0.04],
            textColorRGB=RGB[r["name"]], textSize=0.9, lifeTime=0)
    print("\n窗口保留 10 秒。")
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
