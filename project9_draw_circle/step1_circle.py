"""
第 6 天 · 一小时趣味实验 · 第 1 步
=====================================
让 Panda 在空中画一个圆。

第一版先不追求花哨：
  - 圆心固定在 (0.50, 0, 0.35)
  - 半径 8 cm，圆在水平面内
  - 夹爪始终朝下
  - 参数从 0 到 2*pi 匀速转一圈，总时间 8 s

运行时记录：
  - 理想圆上的目标点
  - 实际末端轨迹
  - 径向误差、跟踪误差、关节速度

默认打开 GUI：
  python project9_draw_circle/step1_circle.py
只看图和指标：
  python project9_draw_circle/step1_circle.py --headless
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

CENTER = np.array([0.50, 0.0, 0.35])
RADIUS = 0.08
START = CENTER + np.array([RADIUS, 0.0, 0.0])
TARGET_ORN = p.getQuaternionFromEuler([0, math.pi, 0])

EE_LINK = 11
ARM_JOINTS = list(range(7))

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
OUT_DIR = os.path.join(BASE_DIR, "output")

COLOR = [0.10, 0.40, 1.00]


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


# ============================================================
# 3. GUI
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


def make_marker():
    vis = p.createVisualShape(p.GEOM_SPHERE, radius=0.010,
                              rgbaColor=[COLOR[0], COLOR[1], COLOR[2], 1.0])
    return p.createMultiBody(baseMass=0, baseVisualShapeIndex=vis,
                             basePosition=START.tolist())


def draw_trail(prev, actual):
    if np.linalg.norm(actual - prev) > 1e-6:
        p.addUserDebugLine(prev.tolist(), actual.tolist(), COLOR,
                           lineWidth=3.0, lifeTime=0)


# ============================================================
# 4. 画圆
# ============================================================
def run_circle(panda, gui):
    warmup_to_start(panda, gui)
    marker = make_marker() if gui else None
    trail_prev = ee_pos(panda)
    rows = []

    for k in range(STEPS):
        u = k / (STEPS - 1)
        theta = 2.0 * math.pi * u
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
            draw_trail(trail_prev, actual)
            trail_prev = actual

    # 回到 start 附近再保持 1 s，观察闭合
    hold_ik(panda, START, HOLD_STEPS, gui)
    actual = ee_pos(panda)
    q, qd = joint_state(panda)
    rows.append({
        "phase": "hold",
        "theta": 2.0 * math.pi,
        "target": START.copy(),
        "actual": actual.copy(),
        "err": actual - START,
        "q": q.copy(),
        "qd": qd.copy(),
    })

    if gui:
        p.removeBody(marker)

    return analyse(rows)


# ============================================================
# 5. 指标与图
# ============================================================
def analyse(rows):
    motion = np.array([r["phase"] == "motion" for r in rows])
    theta = np.array([r["theta"] for r in rows])
    target = np.array([r["target"] for r in rows])
    actual = np.array([r["actual"] for r in rows])
    err = np.array([r["err"] for r in rows])
    q = np.array([r["q"] for r in rows])
    qd = np.array([r["qd"] for r in rows])

    err_n = np.linalg.norm(err, axis=1)
    act_m = actual[motion]
    radial = (np.sqrt((act_m[:, 0] - CENTER[0]) ** 2 +
                      (act_m[:, 1] - CENTER[1]) ** 2) - RADIUS)
    qd_m = qd[motion]
    path_len = float(np.linalg.norm(np.diff(act_m, axis=0), axis=1).sum())

    return {
        "theta": theta,
        "target": target,
        "actual": actual,
        "err": err_n,
        "q": q[motion],
        "radial": radial,
        "motion": motion,
        "max_err": float(err_n[motion].max()),
        "mean_err": float(err_n[motion].mean()),
        "max_radial": float(np.abs(radial).max()),
        "mean_radial": float(np.abs(radial).mean()),
        "close_err": float(np.linalg.norm(actual[-1] - START)),
        "max_qd": float(np.abs(qd_m).max()),
        "q1_max": float(np.abs(qd_m[:, 0]).max()),
        "path_len": path_len,
        "ideal_len": 2 * math.pi * RADIUS,
        "min_z": float(act_m[:, 2].min()),
        "max_z": float(act_m[:, 2].max()),
    }


def make_plot(r, path):
    fig = plt.figure(figsize=(18, 10))
    fig.suptitle("Draw a circle  |  Cartesian path + IK, T=8 s", fontsize=14)
    t = np.arange(len(r["theta"])) * DT

    ax = fig.add_subplot(231)
    ax.plot(r["target"][:, 0], r["target"][:, 1], "--",
            color="0.5", lw=1.5, label="desired")
    ax.plot(r["actual"][:, 0], r["actual"][:, 1], color="tab:blue",
            lw=2, label="actual")
    ax.scatter(*CENTER[:2], color="green", s=30, marker="+")
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]")
    ax.set_title("Top view")
    ax.legend(fontsize=8); ax.grid(alpha=0.3)

    ax = fig.add_subplot(232, projection="3d")
    ax.plot(r["target"][:, 0], r["target"][:, 1], r["target"][:, 2],
            "--", color="0.5", lw=1.2)
    ax.plot(r["actual"][:, 0], r["actual"][:, 1], r["actual"][:, 2],
            color="tab:blue", lw=2)
    ax.set_xlabel("x"); ax.set_ylabel("y"); ax.set_zlabel("z")
    ax.set_title("3D path")

    ax = fig.add_subplot(233)
    ax.plot(t[r["motion"]], r["radial"] * 1000, color="tab:orange", lw=1.5)
    ax.set_xlabel("time [s]"); ax.set_ylabel("radial error [mm]")
    ax.set_title("Distance from ideal circle")
    ax.grid(alpha=0.3)

    ax = fig.add_subplot(234)
    ax.plot(t[r["motion"]], r["err"][r["motion"]] * 1000,
            color="tab:red", lw=1.5)
    ax.set_xlabel("time [s]"); ax.set_ylabel("tracking error [mm]")
    ax.set_title("End-effector tracking error")
    ax.grid(alpha=0.3)

    ax = fig.add_subplot(235)
    for j in range(7):
        ax.plot(t[r["motion"]], r["q"][:, j], label=f"q{j+1}")
    ax.set_xlabel("time [s]"); ax.set_ylabel("joint angle [rad]")
    ax.set_title("Joint angles")
    ax.legend(fontsize=6, ncol=4); ax.grid(alpha=0.3)

    ax = fig.add_subplot(236)
    ax.axis("off")
    lines = [
        "Metrics", "=" * 60,
        f"max tracking error : {r['max_err'] * 1000:6.1f} mm",
        f"mean tracking error: {r['mean_err'] * 1000:6.1f} mm",
        f"max radial error   : {r['max_radial'] * 1000:6.1f} mm",
        f"mean radial error  : {r['mean_radial'] * 1000:6.1f} mm",
        f"closure error      : {r['close_err'] * 1000:6.1f} mm",
        "=" * 60,
        f"actual path length : {r['path_len'] * 100:6.2f} cm",
        f"ideal circumference: {r['ideal_len'] * 100:6.2f} cm",
        f"max |qdot|         : {r['max_qd']:6.2f} rad/s",
        f"q1 max             : {r['q1_max']:6.2f} rad/s",
        f"z range            : {r['min_z']:.3f} ~ {r['max_z']:.3f} m",
    ]
    ax.text(0, 1, "\n".join(lines), va="top", family="monospace",
            fontsize=9)

    fig.tight_layout(rect=[0, 0, 1, 0.95])
    fig.savefig(path, dpi=150)
    plt.close(fig)


def print_summary(r):
    print("\n===== circle v1 =====")
    print("max tracking error : %.1f mm" % (r["max_err"] * 1000))
    print("mean tracking error: %.1f mm" % (r["mean_err"] * 1000))
    print("max radial error   : %.1f mm" % (r["max_radial"] * 1000))
    print("closure error      : %.2f mm" % (r["close_err"] * 1000))
    print("actual path length : %.2f cm (ideal %.2f cm)"
          % (r["path_len"] * 100, r["ideal_len"] * 100))
    print("max |qdot|         : %.2f rad/s" % r["max_qd"])
    print("z range            : %.3f ~ %.3f m"
          % (r["min_z"], r["max_z"]))


def save_summary(r, path):
    fields = ["max_tracking_err_mm", "mean_tracking_err_mm",
              "max_radial_err_mm", "mean_radial_err_mm",
              "closure_err_mm", "actual_path_cm", "ideal_path_cm",
              "max_qdot_rad_s", "q1_max_rad_s", "min_z_m", "max_z_m"]
    with open(path, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(fields)
        w.writerow([
            f"{r['max_err'] * 1000:.3f}",
            f"{r['mean_err'] * 1000:.3f}",
            f"{r['max_radial'] * 1000:.3f}",
            f"{r['mean_radial'] * 1000:.3f}",
            f"{r['close_err'] * 1000:.3f}",
            f"{r['path_len'] * 100:.3f}",
            f"{r['ideal_len'] * 100:.3f}",
            f"{r['max_qd']:.3f}",
            f"{r['q1_max']:.3f}",
            f"{r['min_z']:.4f}",
            f"{r['max_z']:.4f}",
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

    print("开始画圆：center=%s radius=%.3f m T=%.1fs"
          % (np.round(CENTER, 3), RADIUS, T_TOTAL))
    r = run_circle(panda, gui)
    print_summary(r)

    png_path = os.path.join(OUT_DIR, "circle_v1.png")
    csv_path = os.path.join(OUT_DIR, "circle_v1.csv")
    make_plot(r, png_path)
    save_summary(r, csv_path)
    print("\nCSV :", csv_path)
    print("图片:", png_path)

    if gui:
        p.addUserDebugText(
            "max err %.1f mm  |  closure %.1f mm  |  max qdot %.2f rad/s"
            % (r["max_err"] * 1000, r["close_err"] * 1000, r["max_qd"]),
            [0.35, 0.28, 0.58], textColorRGB=COLOR,
            textSize=1.0, lifeTime=0)
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
