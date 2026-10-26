"""
第 6 天 · 一小时趣味实验 · 第 2 步
=====================================
同一个圆，换两种“什么时候走到哪”：

  constant：角度匀速变化，转一圈 8 s
  min-jerk：角度按五次多项式变化，起停速度平滑

记录实际末端轨迹、跟踪误差、径向误差和关节速度。

运行：
  python project9_draw_circle/step2_circle_timing.py          # GUI
  python project9_draw_circle/step2_circle_timing.py --headless
"""

import argparse
import csv
import math
import os
import time

import numpy as np
import pybullet as p
import pybullet_data
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt


# ============================================================
# 1. 参数
# ============================================================
DT = 1.0 / 240.0
T_TOTAL = 8.0
STEPS = int(round(T_TOTAL / DT)) + 1
HOLD_STEPS = 240
WARMUP_STEPS = 360
WARMUP_HOLD = 240
RESET_HOLD = 240

CENTER = np.array([0.50, 0.0, 0.35])
RADIUS = 0.08
START = CENTER + np.array([RADIUS, 0.0, 0.0])
TARGET_ORN = p.getQuaternionFromEuler([0, math.pi, 0])

EE_LINK = 11
ARM_JOINTS = list(range(7))

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
OUT_DIR = os.path.join(BASE_DIR, "output")

COLORS = {"constant": "tab:blue", "minjerk": "tab:green"}
RGB = {"constant": [0.10, 0.40, 1.00],
       "minjerk": [0.00, 0.70, 0.20]}


# ============================================================
# 2. 仿真基础
# ============================================================
def connect(gui):
    if gui:
        p.connect(p.GUI)
        p.resetDebugVisualizerCamera(
            cameraDistance=1.25, cameraYaw=45, cameraPitch=-35,
            cameraTargetPosition=[0.50, 0.0, 0.32],
        )
    else:
        p.connect(p.DIRECT)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.8)
    p.setRealTimeSimulation(0)
    p.setTimeStep(DT)


def build_scene():
    p.loadURDF("plane.urdf")
    panda = p.loadURDF("franka_panda/panda.urdf", useFixedBase=True)
    default_pose = [0, -0.785, 0, -2.356, 0, 1.571, 0.785]
    for i, q in enumerate(default_pose):
        p.resetJointState(panda, i, q)
    for j in range(p.getNumJoints(panda)):
        if b"finger" in p.getJointInfo(panda, j)[1]:
            p.setJointMotorControl2(panda, j, p.POSITION_CONTROL,
                                    targetPosition=0.04, force=200)
    return panda


def ee_pos(panda):
    state = p.getLinkState(panda, EE_LINK, computeForwardKinematics=True)
    return np.array(state[4], dtype=float)


def solve_ik(panda, target):
    q = p.calculateInverseKinematics(
        panda, EE_LINK, target.tolist(), TARGET_ORN,
        maxNumIterations=200, residualThreshold=1e-8,
    )
    return np.array(q[:7], dtype=float)


def command_arm(panda, q_cmd):
    for j, q in zip(ARM_JOINTS, q_cmd):
        p.setJointMotorControl2(panda, j, p.POSITION_CONTROL,
                                targetPosition=float(q), force=200)


def joint_state(panda):
    q, qd = [], []
    for j in ARM_JOINTS:
        state = p.getJointState(panda, j)
        q.append(state[0])
        qd.append(state[1])
    return np.array(q), np.array(qd)


def hold_ik(panda, target, steps, gui):
    q_cmd = solve_ik(panda, target)
    for _ in range(steps):
        command_arm(panda, q_cmd)
        p.stepSimulation()
        if gui:
            time.sleep(DT)


def warmup_to_start(panda, gui):
    initial = ee_pos(panda)
    for k in range(WARMUP_STEPS):
        s = k / (WARMUP_STEPS - 1)
        command_arm(panda, solve_ik(
            panda, initial + s * (START - initial)))
        p.stepSimulation()
        if gui:
            time.sleep(DT)
    hold_ik(panda, START, WARMUP_HOLD, gui)
    q, _ = joint_state(panda)
    return q


def reset_to_start(panda, q_start, gui):
    for j, q in enumerate(q_start):
        p.resetJointState(panda, j, float(q), 0.0)
    hold_ik(panda, START, RESET_HOLD, gui)


# ============================================================
# 3. 时间曲线
# ============================================================
def minjerk_s(u):
    return 10 * u ** 3 - 15 * u ** 4 + 6 * u ** 5


def theta_of(u, timing):
    if timing == "constant":
        s = u
    else:
        s = minjerk_s(u)
    return 2.0 * math.pi * s


# ============================================================
# 4. GUI
# ============================================================
def draw_ideal_circle(gui):
    if not gui:
        return
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
    p.addUserDebugText("start", (START + [0, 0, 0.05]).tolist(),
                       textColorRGB=[0.2, 0.2, 0.2], textSize=1.0,
                       lifeTime=0)
    p.addUserDebugText("blue = constant   green = min-jerk",
                       [0.35, 0.28, 0.56], textColorRGB=[0, 0, 0],
                       textSize=1.0, lifeTime=0)


def make_marker(color):
    vis = p.createVisualShape(p.GEOM_SPHERE, radius=0.010,
                              rgbaColor=[color[0], color[1], color[2], 1.0])
    return p.createMultiBody(baseMass=0, baseVisualShapeIndex=vis,
                             basePosition=START.tolist())


def draw_trail(prev, actual, color):
    if np.linalg.norm(actual - prev) > 1e-6:
        p.addUserDebugLine(prev.tolist(), actual.tolist(), color,
                           lineWidth=3.0, lifeTime=0)


# ============================================================
# 5. 跑圆
# ============================================================
def run_circle(panda, timing, q_start, gui):
    reset_to_start(panda, q_start, gui)
    color = RGB[timing]
    marker = make_marker(color) if gui else None
    trail_prev = ee_pos(panda)
    rows = []

    for k in range(STEPS):
        u = k / (STEPS - 1)
        theta = theta_of(u, timing)
        target = CENTER + np.array([RADIUS * math.cos(theta),
                                    RADIUS * math.sin(theta), 0.0])
        q_cmd = solve_ik(panda, target)
        command_arm(panda, q_cmd)
        p.stepSimulation()
        if gui:
            time.sleep(DT)
            p.resetBasePositionAndOrientation(
                marker, target.tolist(), [0, 0, 0, 1])

        actual = ee_pos(panda)
        q, qd = joint_state(panda)
        rows.append({
            "phase": "motion",
            "theta": theta,
            "target": target.copy(),
            "actual": actual.copy(),
            "err": actual - target,
            "q": q.copy(),
            "qd": qd.copy(),
        })
        if gui and k % 4 == 0:
            draw_trail(trail_prev, actual, color)
            trail_prev = actual

    hold_ik(panda, START, HOLD_STEPS, gui)
    actual = ee_pos(panda)
    q, qd = joint_state(panda)
    rows.append({
        "phase": "hold",
        "theta": 2 * math.pi,
        "target": START.copy(),
        "actual": actual.copy(),
        "err": actual - START,
        "q": q.copy(),
        "qd": qd.copy(),
    })
    if gui:
        p.removeBody(marker)
    return analyse(rows, timing)


# ============================================================
# 6. 分析
# ============================================================
def analyse(rows, timing):
    motion = np.array([r["phase"] == "motion" for r in rows])
    theta = np.array([r["theta"] for r in rows])[motion]
    target = np.array([r["target"] for r in rows])
    actual = np.array([r["actual"] for r in rows])
    err = np.array([r["err"] for r in rows])
    q = np.array([r["q"] for r in rows])[motion]
    qd = np.array([r["qd"] for r in rows])[motion]

    t = np.arange(motion.sum()) * DT
    err_m = np.linalg.norm(err[motion], axis=1)
    act_m = actual[motion]
    radial = (np.sqrt((act_m[:, 0] - CENTER[0]) ** 2 +
                      (act_m[:, 1] - CENTER[1]) ** 2) - RADIUS)
    target_m = target[motion]
    step_speed = (np.linalg.norm(np.diff(target_m, axis=0), axis=1) / DT
                  if len(target_m) > 1 else np.array([0.0]))
    path_len = float(np.linalg.norm(np.diff(act_m, axis=0), axis=1).sum())

    return {
        "name": timing,
        "t": t,
        "target": target_m,
        "actual": act_m,
        "err": err_m,
        "radial": radial,
        "q": q,
        "qd": qd,
        "target_speed": step_speed,
        "max_err": float(err_m.max()),
        "mean_err": float(err_m.mean()),
        "max_radial": float(np.abs(radial).max()),
        "mean_radial": float(np.abs(radial).mean()),
        "close_err": float(np.linalg.norm(actual[-1] - START)),
        "max_qd": float(np.abs(qd).max()),
        "q1_max": float(np.abs(qd[:, 0]).max()),
        "peak_target_speed": float(step_speed.max()),
        "path_len": path_len,
        "min_z": float(act_m[:, 2].min()),
        "max_z": float(act_m[:, 2].max()),
    }


# ============================================================
# 7. 图与表
# ============================================================
def make_plot(const, minj, path):
    fig = plt.figure(figsize=(18, 10))
    fig.suptitle("Circle timing  |  constant vs min-jerk (T=8 s)", fontsize=14)

    ax = fig.add_subplot(231)
    ax.plot(const["target"][:, 0], const["target"][:, 1], "--",
            color="0.5", lw=1.2, label="desired")
    ax.plot(const["actual"][:, 0], const["actual"][:, 1],
            color=COLORS["constant"], lw=1.8, label="constant")
    ax.plot(minj["actual"][:, 0], minj["actual"][:, 1],
            color=COLORS["minjerk"], lw=1.8, label="min-jerk")
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]")
    ax.set_title("Top view")
    ax.legend(fontsize=8); ax.grid(alpha=0.3)

    ax = fig.add_subplot(232)
    ax.plot(const["t"][1:], const["target_speed"],
            color=COLORS["constant"], lw=2, label="constant")
    ax.plot(minj["t"][1:], minj["target_speed"],
            color=COLORS["minjerk"], lw=2, label="min-jerk")
    ax.set_xlabel("time [s]"); ax.set_ylabel("target speed [m/s]")
    ax.set_title("Cartesian target speed")
    ax.legend(fontsize=8); ax.grid(alpha=0.3)

    ax = fig.add_subplot(233)
    ax.plot(const["t"], const["err"] * 1000,
            color=COLORS["constant"], lw=1.5, label="constant")
    ax.plot(minj["t"], minj["err"] * 1000,
            color=COLORS["minjerk"], lw=1.5, label="min-jerk")
    ax.set_xlabel("time [s]"); ax.set_ylabel("tracking error [mm]")
    ax.set_title("Tracking error")
    ax.legend(fontsize=8); ax.grid(alpha=0.3)

    ax = fig.add_subplot(234)
    ax.plot(const["t"], const["radial"] * 1000,
            color=COLORS["constant"], lw=1.5, label="constant")
    ax.plot(minj["t"], minj["radial"] * 1000,
            color=COLORS["minjerk"], lw=1.5, label="min-jerk")
    ax.set_xlabel("time [s]"); ax.set_ylabel("radial error [mm]")
    ax.set_title("Distance from ideal circle")
    ax.legend(fontsize=8); ax.grid(alpha=0.3)

    ax = fig.add_subplot(235)
    ax.plot(const["t"], np.max(np.abs(const["qd"]), axis=1),
            color=COLORS["constant"], lw=1.5, label="constant")
    ax.plot(minj["t"], np.max(np.abs(minj["qd"]), axis=1),
            color=COLORS["minjerk"], lw=1.5, label="min-jerk")
    ax.set_xlabel("time [s]"); ax.set_ylabel("max |qdot| [rad/s]")
    ax.set_title("Actual joint speed")
    ax.legend(fontsize=8); ax.grid(alpha=0.3)

    ax = fig.add_subplot(236)
    ax.axis("off")
    lines = ["Metrics", "=" * 68,
             f"{'':<14}{'constant':>12}{'min-jerk':>12}",
             f"{'max err':<14}{const['max_err'] * 1000:10.1f}mm"
             f"{minj['max_err'] * 1000:10.1f}mm",
             f"{'mean err':<14}{const['mean_err'] * 1000:10.1f}mm"
             f"{minj['mean_err'] * 1000:10.1f}mm",
             f"{'max radial':<14}{const['max_radial'] * 1000:10.1f}mm"
             f"{minj['max_radial'] * 1000:10.1f}mm",
             f"{'closure':<14}{const['close_err'] * 1000:10.2f}mm"
             f"{minj['close_err'] * 1000:10.2f}mm",
             f"{'max qdot':<14}{const['max_qd']:10.2f}"
             f"{minj['max_qd']:10.2f}",
             f"{'peak target v':<14}{const['peak_target_speed']:10.3f}"
             f"{minj['peak_target_speed']:10.3f}",
             f"{'path length':<14}{const['path_len'] * 100:10.1f}cm"
             f"{minj['path_len'] * 100:10.1f}cm",
             "=" * 68,
             "constant: instant start/stop in angle",
             "min-jerk: smooth angular start/stop",
             "same circle, same total time 8 s"]
    ax.text(0, 1, "\n".join(lines), va="top", family="monospace",
            fontsize=9)

    fig.tight_layout(rect=[0, 0, 1, 0.95])
    fig.savefig(path, dpi=150)
    plt.close(fig)


def print_summary(r):
    print("%-9s max err %5.1fmm | mean %4.1fmm | radial %5.1fmm | "
          "closure %.2fmm | max qdot %.2f | peak v %.3f m/s"
          % (r["name"], r["max_err"] * 1000, r["mean_err"] * 1000,
             r["max_radial"] * 1000, r["close_err"] * 1000,
             r["max_qd"], r["peak_target_speed"]))


def save_csv(const, minj, path):
    fields = ["timing", "max_err_mm", "mean_err_mm", "max_radial_mm",
              "mean_radial_mm", "closure_mm", "max_qdot", "q1_max",
              "peak_target_speed", "path_len_cm", "min_z", "max_z"]
    with open(path, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(fields)
        for r in (const, minj):
            w.writerow([
                r["name"], f"{r['max_err'] * 1000:.3f}",
                f"{r['mean_err'] * 1000:.3f}",
                f"{r['max_radial'] * 1000:.3f}",
                f"{r['mean_radial'] * 1000:.3f}",
                f"{r['close_err'] * 1000:.3f}",
                f"{r['max_qd']:.3f}", f"{r['q1_max']:.3f}",
                f"{r['peak_target_speed']:.4f}",
                f"{r['path_len'] * 100:.3f}",
                f"{r['min_z']:.4f}", f"{r['max_z']:.4f}",
            ])


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--gui", action="store_true",
                        help="打开 PyBullet 窗口（默认）")
    parser.add_argument("--headless", action="store_true",
                        help="不打开窗口，只算指标和出图")
    args = parser.parse_args()
    if args.gui and args.headless:
        parser.error("--gui 和 --headless 只能选一个")
    gui = not args.headless

    os.makedirs(OUT_DIR, exist_ok=True)
    connect(gui)
    panda = build_scene()
    draw_ideal_circle(gui)

    print("warmup ...")
    q_start = warmup_to_start(panda, gui)

    print("[1/2] constant angular speed ...")
    const = run_circle(panda, "constant", q_start, gui)
    print("[2/2] min-jerk angular speed ...")
    minj = run_circle(panda, "minjerk", q_start, gui)
    print_summary(const)
    print_summary(minj)

    png_path = os.path.join(OUT_DIR, "circle_timing.png")
    csv_path = os.path.join(OUT_DIR, "circle_timing.csv")
    make_plot(const, minj, png_path)
    save_csv(const, minj, csv_path)
    print("\nCSV :", csv_path)
    print("图片:", png_path)

    if gui:
        p.addUserDebugText(
            "constant: err %.1f mm, qdot %.2f" %
            (const["max_err"] * 1000, const["max_qd"]),
            [0.35, 0.28, 0.54], textColorRGB=RGB["constant"],
            textSize=0.9, lifeTime=0)
        p.addUserDebugText(
            "min-jerk: err %.1f mm, qdot %.2f" %
            (minj["max_err"] * 1000, minj["max_qd"]),
            [0.35, 0.32, 0.50], textColorRGB=RGB["minjerk"],
            textSize=0.9, lifeTime=0)
        print("\n窗口保留 8 秒。")
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
