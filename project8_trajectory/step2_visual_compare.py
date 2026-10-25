"""
第 5 天 · 轨迹可视化实验 · 第 2 步的 GUI 演示
================================================
在 PyBullet 窗口里连续演示三种走法：

    constant  →  trapezoid  →  min-jerk

三种方式都走同一条 39 cm 直线，总时间都是 2.5 s。
屏幕上：
  - 灰色细线：规划路径
  - 蓝色/橙色/绿色粗线：三种方式的实际末端轨迹
  - 同色小球：当前“指令目标点”（它领先实际轨迹，差距就是跟踪滞后）

运行结束后：
  - 终端打印对比表
  - 生成 output/step2_visual_summary.csv
  - 生成 output/step2_visual_table.png

用法：
  python project8_trajectory/step2_visual_compare.py          # 打开 GUI 演示
  python project8_trajectory/step2_visual_compare.py --headless
  python project8_trajectory/step2_visual_compare.py --a-max 0.8 --j-max 5.0
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
T_TOTAL = 2.5
HOLD_STEPS = 240
WARMUP_STEPS = 360
WARMUP_HOLD = 240
RESET_HOLD = 240

START = np.array([0.40, -0.18, 0.35])
END = np.array([0.55, 0.18, 0.35])
DIST = float(np.linalg.norm(END - START))
TARGET_ORN = p.getQuaternionFromEuler([0, math.pi, 0])

EE_LINK = 11
ARM_JOINTS = list(range(7))

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
OUT_DIR = os.path.join(BASE_DIR, "output")

COLORS = {
    "constant": [0.10, 0.40, 1.00],
    "trapezoid": [1.00, 0.48, 0.00],
    "minjerk": [0.00, 0.70, 0.20],
}
LABELS = {
    "constant": "constant (instant start/stop)",
    "trapezoid": "trapezoid",
    "minjerk": "min-jerk",
}


# ============================================================
# 2. 三种速度曲线
# ============================================================
def profile_constant(u):
    return u, 1.0, 0.0, 0.0


def profile_trapezoid(u, accel_frac=0.2):
    a = accel_frac
    vmax = 1.0 / (1.0 - a)
    amax = vmax / a
    if u < a:
        return 0.5 * amax * u * u, amax * u, amax, 0.0
    if u < 1.0 - a:
        return (0.5 * amax * a * a + vmax * (u - a),
                vmax, 0.0, 0.0)
    r = 1.0 - u
    return 1.0 - 0.5 * amax * r * r, amax * r, -amax, 0.0


def profile_minjerk(u):
    s = 10.0 * u ** 3 - 15.0 * u ** 4 + 6.0 * u ** 5
    ds = 30.0 * u ** 2 - 60.0 * u ** 3 + 30.0 * u ** 4
    dds = 60.0 * u - 180.0 * u ** 2 + 120.0 * u ** 3
    ddds = 60.0 - 360.0 * u + 360.0 * u ** 2
    return s, ds, dds, ddds


PROFILES = [
    ("constant", profile_constant, COLORS["constant"], LABELS["constant"]),
    ("trapezoid", lambda u: profile_trapezoid(u, 0.2),
     COLORS["trapezoid"], LABELS["trapezoid"]),
    ("minjerk", profile_minjerk, COLORS["minjerk"], LABELS["minjerk"]),
]


# ============================================================
# 3. 仿真基础
# ============================================================
def connect(gui):
    if gui:
        p.connect(p.GUI)
        p.resetDebugVisualizerCamera(
            cameraDistance=1.35, cameraYaw=45, cameraPitch=-35,
            cameraTargetPosition=[0.45, 0.0, 0.30],
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
    vel_limits = np.array(
        [p.getJointInfo(panda, j)[11] for j in ARM_JOINTS], dtype=float)
    return panda, vel_limits


def ee_pos(panda):
    return np.array(p.getLinkState(panda, EE_LINK,
                                   computeForwardKinematics=True)[4],
                    dtype=float)


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


def hold(panda, target, steps, gui):
    for _ in range(steps):
        command_arm(panda, solve_ik(panda, target))
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
    hold(panda, START, WARMUP_HOLD, gui)
    q, _ = joint_state(panda)
    return q


def reset_to_start(panda, q_start, gui):
    for j, q in enumerate(q_start):
        p.resetJointState(panda, j, float(q), 0.0)
    hold(panda, START, RESET_HOLD, gui)


# ============================================================
# 4. GUI 标记与轨迹
# ============================================================
def draw_reference(gui):
    if not gui:
        return
    for k in range(60):
        a = START + (END - START) * (k / 60.0)
        b = START + (END - START) * ((k + 1) / 60.0)
        p.addUserDebugLine(a.tolist(), b.tolist(),
                           [0.6, 0.6, 0.6], lineWidth=1.2, lifeTime=0)
    p.addUserDebugText("START", (START + [0, 0, 0.05]).tolist(),
                       textColorRGB=[0.3, 0.3, 0.3], textSize=1.0,
                       lifeTime=0)
    p.addUserDebugText("END", (END + [0, 0, 0.05]).tolist(),
                       textColorRGB=[0.3, 0.3, 0.3], textSize=1.0,
                       lifeTime=0)
    p.addUserDebugText(
        "blue=constant  orange=trapezoid  green=min-jerk",
        [0.22, 0.30, 0.58],
        textColorRGB=[0.0, 0.0, 0.0], textSize=1.0, lifeTime=0)


def make_target_marker(color):
    vis = p.createVisualShape(p.GEOM_SPHERE, radius=0.010,
                              rgbaColor=[color[0], color[1], color[2], 1.0])
    return p.createMultiBody(baseMass=0, baseVisualShapeIndex=vis,
                             basePosition=START.tolist())


def update_marker(marker, target):
    p.resetBasePositionAndOrientation(marker, target.tolist(),
                                      [0, 0, 0, 1])


# ============================================================
# 5. 跑一条轨迹
# ============================================================
def run_trial(panda, name, profile_fn, color, label, q_start, gui,
              vel_limits):
    if q_start is None:
        q_start = warmup_to_start(panda, gui)
    else:
        reset_to_start(panda, q_start, gui)

    marker = make_target_marker(color) if gui else None
    steps = int(round(T_TOTAL / DT)) + 1
    t_actual = (steps - 1) * DT
    d = END - START
    rows = []
    trail_prev = ee_pos(panda)
    draw_counter = 0

    for k in range(steps):
        u = k / (steps - 1)
        s, _, _, _ = profile_fn(u)
        target = START + s * d
        q_cmd = solve_ik(panda, target)
        command_arm(panda, q_cmd)
        p.stepSimulation()
        if gui:
            time.sleep(DT)
            update_marker(marker, target)

        actual = ee_pos(panda)
        q, qd = joint_state(panda)
        rows.append({
            "phase": "motion",
            "target": target.copy(),
            "actual": actual.copy(),
            "err": actual - target,
            "q_cmd": q_cmd.copy(),
            "q": q.copy(),
            "qd": qd.copy(),
        })
        if gui:
            draw_counter += 1
            if (draw_counter % 4 == 0 and
                    np.linalg.norm(actual - trail_prev) > 1e-6):
                p.addUserDebugLine(trail_prev.tolist(), actual.tolist(),
                                   color, lineWidth=3.2, lifeTime=0)
                trail_prev = actual

    # 到终点后保持 1 s，观察停下来后的收敛
    for _ in range(HOLD_STEPS):
        target = END.copy()
        q_cmd = solve_ik(panda, target)
        command_arm(panda, q_cmd)
        p.stepSimulation()
        if gui:
            time.sleep(DT)
            update_marker(marker, target)
        actual = ee_pos(panda)
        q, qd = joint_state(panda)
        rows.append({
            "phase": "hold",
            "target": target.copy(),
            "actual": actual.copy(),
            "err": actual - target,
            "q_cmd": q_cmd.copy(),
            "q": q.copy(),
            "qd": qd.copy(),
        })
        if gui:
            draw_counter += 1
            if (draw_counter % 4 == 0 and
                    np.linalg.norm(actual - trail_prev) > 1e-6):
                p.addUserDebugLine(trail_prev.tolist(), actual.tolist(),
                                   color, lineWidth=3.2, lifeTime=0)
                trail_prev = actual

    if gui:
        p.removeBody(marker)

    return compute_metrics(rows, name, label, t_actual, vel_limits), q_start


# ============================================================
# 6. 指标
# ============================================================
def compute_metrics(rows, name, label, t_actual, vel_limits):
    target = np.array([r["target"] for r in rows])
    actual = np.array([r["actual"] for r in rows])
    err = np.array([r["err"] for r in rows])
    qd = np.array([r["qd"] for r in rows])
    q_cmd = np.array([r["q_cmd"] for r in rows])
    phase = np.array([r["phase"] for r in rows])
    motion = phase == "motion"
    n_m = int(motion.sum())

    tgt = target[motion]
    seq = np.vstack([START, tgt, END])
    seg = np.diff(seq, axis=0)
    speed = np.linalg.norm(seg, axis=1) / DT
    accn = np.abs(np.diff(speed)) / DT
    jerkn = np.abs(np.diff(accn)) / DT

    err_m = err[motion]
    err_n = np.linalg.norm(err_m, axis=1)
    qd_m = qd[motion]
    qcmd_dot = np.gradient(q_cmd[motion], DT, axis=0)
    speed_ratio = max(
        float(np.max(np.abs(qd_m) / vel_limits[None, :])),
        float(np.max(np.abs(qcmd_dot) / vel_limits[None, :])),
    )
    a_ratio = float(accn.max()) / A_MAX
    j_ratio = float(jerkn.max()) / J_MAX
    fails = []
    if speed_ratio > 1.0:
        fails.append("joint speed")
    if a_ratio > 1.0:
        fails.append("acceleration")
    if j_ratio > 1.0:
        fails.append("jerk")

    return {
        "name": name,
        "label": label,
        "duration": t_actual,
        "peak_speed": float(speed.max()),
        "max_acc": float(accn.max()),
        "max_jerk": float(jerkn.max()),
        "max_err": float(err_n.max()),
        "rms_err": float(np.sqrt(np.mean(err_n ** 2))),
        "q1_max": float(np.abs(qd_m[:, 0]).max()),
        "speed_ratio": speed_ratio,
        "a_ratio": a_ratio,
        "j_ratio": j_ratio,
        "feasible": len(fails) == 0,
        "fails": fails,
    }


# ============================================================
# 7. 输出表格
# ============================================================
def print_table(results):
    print("\n" + "=" * 112)
    print("三种时间参数化对比（同一路径、总时间 %.2f s）" % T_TOTAL)
    print("示例限制: a_max=%.2f m/s², jerk_max=%.2f m/s³" %
          (A_MAX, J_MAX))
    print("=" * 112)
    print("%-12s %5s %9s %10s %10s %9s %8s %8s  %s"
          % ("方式", "T[s]", "峰值v", "最大a", "最大jerk",
             "最大误差", "q1最大", "速度比", "可执行性"))
    print("-" * 112)
    for r in results:
        gate = "PASS" if r["feasible"] else "REJECT: " + ", ".join(r["fails"])
        print("%-12s %5.2f %9.3f %10.2f %10.2f %8.1fmm %7.2f %8.2f  %s"
              % (r["name"], r["duration"], r["peak_speed"],
                 r["max_acc"], r["max_jerk"], r["max_err"] * 1000,
                 r["q1_max"], r["speed_ratio"], gate))
    print("=" * 112)
    print("注：PASS/REJECT 只针对示例的速度、加速度、jerk 三项限制。\n")


def save_csv(results, path):
    fields = ["name", "duration", "peak_speed", "max_acc", "max_jerk",
              "max_err", "rms_err", "q1_max", "speed_ratio",
              "a_ratio", "j_ratio", "feasible", "fails"]
    with open(path, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(fields)
        for r in results:
            w.writerow([
                r["name"], f"{r['duration']:.4f}",
                f"{r['peak_speed']:.5f}", f"{r['max_acc']:.3f}",
                f"{r['max_jerk']:.2f}", f"{r['max_err']:.5f}",
                f"{r['rms_err']:.5f}", f"{r['q1_max']:.3f}",
                f"{r['speed_ratio']:.3f}", f"{r['a_ratio']:.3f}",
                f"{r['j_ratio']:.3f}", r["feasible"],
                ";".join(r["fails"]),
            ])


def save_table_png(results, path):
    headers = ["Profile", "T (s)", "peak v", "max a", "max jerk",
               "max error", "q1 max", "speed", "gate"]
    cell_text = []
    for r in results:
        cell_text.append([
            r["name"], f"{r['duration']:.2f}", f"{r['peak_speed']:.3f}",
            f"{r['max_acc']:.2f}", f"{r['max_jerk']:.1f}",
            f"{r['max_err'] * 1000:.1f} mm", f"{r['q1_max']:.2f}",
            f"{r['speed_ratio']:.2f}",
            "PASS" if r["feasible"] else "REJECT",
        ])

    fig, ax = plt.subplots(figsize=(14, 3.2))
    ax.axis("off")
    tab = ax.table(cellText=cell_text, colLabels=headers,
                   loc="center", cellLoc="center")
    tab.auto_set_font_size(False)
    tab.set_fontsize(11)
    tab.scale(1.0, 1.6)
    for j in range(len(headers)):
        tab[(0, j)].set_facecolor("#dfe8f3")
        tab[(0, j)].set_text_props(weight="bold")
    for i in range(1, len(results) + 1):
        if not results[i - 1]["feasible"]:
            tab[(i, len(headers) - 1)].set_facecolor("#ffd6d6")
        else:
            tab[(i, len(headers) - 1)].set_facecolor("#d8f0dc")
    ax.set_title(
        "Time parameterization comparison  |  same path, T=2.5 s  |  "
        f"limits: a_max={A_MAX:g} m/s², jerk_max={J_MAX:g} m/s³",
        fontsize=12, pad=18)
    fig.tight_layout()
    fig.savefig(path, dpi=150, bbox_inches="tight")
    plt.close(fig)


# ============================================================
# 8. 主流程
# ============================================================
A_MAX = 0.5
J_MAX = 2.0


def main():
    global A_MAX, J_MAX
    parser = argparse.ArgumentParser()
    parser.add_argument("--gui", action="store_true",
                        help="打开 PyBullet 窗口演示")
    parser.add_argument("--headless", action="store_true",
                        help="不打开窗口，只算指标")
    parser.add_argument("--a-max", type=float, default=0.5,
                        help="示例末端加速度上限 [m/s^2]")
    parser.add_argument("--j-max", type=float, default=2.0,
                        help="示例末端 jerk 上限 [m/s^3]")
    args = parser.parse_args()

    if args.gui and args.headless:
        parser.error("--gui 和 --headless 只能选一个")
    gui = not args.headless
    A_MAX = args.a_max
    J_MAX = args.j_max

    os.makedirs(OUT_DIR, exist_ok=True)
    connect(gui)
    panda, vel_limits = build_scene()
    draw_reference(gui)

    print("预热到起点 ...")
    q_start = warmup_to_start(panda, gui)
    print("起点误差: %.3f mm" %
          (np.linalg.norm(ee_pos(panda) - START) * 1000))

    results = []
    for idx, (name, profile_fn, color, label) in enumerate(PROFILES):
        print("\n[%d/%d] %s" % (idx + 1, len(PROFILES), label))
        r, q_start = run_trial(
            panda, name, profile_fn, color, label, q_start, gui,
            vel_limits)
        results.append(r)
        print("    峰值速度 %.3f m/s | 最大误差 %.1f mm | "
              "q1 %.2f rad/s" %
              (r["peak_speed"], r["max_err"] * 1000, r["q1_max"]))
        if gui:
            time.sleep(0.8)

    print_table(results)
    csv_path = os.path.join(OUT_DIR, "step2_visual_summary.csv")
    png_path = os.path.join(OUT_DIR, "step2_visual_table.png")
    save_csv(results, csv_path)
    save_table_png(results, png_path)
    print("CSV  :", csv_path)
    print("表格 :", png_path)

    if gui:
        print("\n演示结束，窗口保留 8 秒。")
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
