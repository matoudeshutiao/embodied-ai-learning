"""
第 5 天 · 轨迹可视化实验 · 第 3 步
=====================================
同样的起点、终点和姿态，比较两种插值空间：

  1. Cartesian interpolation
     末端位置沿直线插值，姿态固定；每一帧重新解 IK。

  2. Joint-space interpolation
     直接用 IK 求出起点/终点的 7 个关节角，
     再让关节角在起点和终点之间线性插值。

两者都用同一个 min-jerk 时间曲线，总时间 2.5 s。
这样比较的变量只有“在哪里插值”，而不是速度曲线。

重点观察：
  - 关节空间插值的末端轨迹会不会弯？
  - 姿态会不会在中间偏离“夹爪朝下”？
  - 两种方法的关节跟踪误差、关节速度差多少？

运行：
  python project8_trajectory/step3_interpolation.py          # 默认打开 GUI
  python project8_trajectory/step3_interpolation.py --headless
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
MOVE_STEPS = int(round(T_TOTAL / DT)) + 1
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

COLORS = {"cartesian": "tab:blue", "joint": "tab:orange"}


# ============================================================
# 2. 归一化 min-jerk 时间曲线
# ============================================================
def minjerk_s(u):
    return 10.0 * u ** 3 - 15.0 * u ** 4 + 6.0 * u ** 5


# ============================================================
# 3. 仿真
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
    return panda


def ee_state(panda):
    state = p.getLinkState(panda, EE_LINK, computeForwardKinematics=True)
    return np.array(state[4], dtype=float), np.array(state[5], dtype=float)


def ee_pos(panda):
    return ee_state(panda)[0]


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


def hold(panda, q_cmd_or_target, steps, mode, gui):
    for _ in range(steps):
        if mode == "joint":
            q_cmd = q_cmd_or_target
        else:
            q_cmd = solve_ik(panda, q_cmd_or_target)
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
    hold(panda, START, WARMUP_HOLD, "cartesian", gui)
    q, _ = joint_state(panda)
    return q


def reset_to_start(panda, q_start, gui):
    for j, q in enumerate(q_start):
        p.resetJointState(panda, j, float(q), 0.0)
    hold(panda, START, RESET_HOLD, "cartesian", gui)


def quat_angle_deg(qa, qb):
    dot = abs(float(np.dot(qa, qb)))
    dot = min(1.0, max(-1.0, dot))
    return math.degrees(2.0 * math.acos(dot))


# ============================================================
# 3.5 GUI 辅助
# ============================================================
RGB = {"cartesian": [0.10, 0.40, 1.00],
       "joint": [1.00, 0.48, 0.00]}


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
    p.addUserDebugText("blue = Cartesian   orange = joint-space",
                       [0.22, 0.30, 0.60],
                       textColorRGB=[0.0, 0.0, 0.0], textSize=1.0,
                       lifeTime=0)


def make_marker(color):
    vis = p.createVisualShape(p.GEOM_SPHERE, radius=0.010,
                              rgbaColor=[color[0], color[1], color[2], 1.0])
    return p.createMultiBody(baseMass=0, baseVisualShapeIndex=vis,
                             basePosition=START.tolist())


def update_marker(marker, target):
    p.resetBasePositionAndOrientation(marker, target.tolist(),
                                      [0, 0, 0, 1])


def draw_trail(prev, actual, color):
    if np.linalg.norm(actual - prev) > 1e-6:
        p.addUserDebugLine(prev.tolist(), actual.tolist(),
                           color, lineWidth=3.2, lifeTime=0)


# ============================================================
# 4. 两种运行
# ============================================================
def run_cartesian(panda, gui):
    rows = []
    marker = make_marker(RGB["cartesian"]) if gui else None
    trail_prev = ee_pos(panda)
    draw_counter = 0

    for k in range(MOVE_STEPS):
        u = k / (MOVE_STEPS - 1)
        s = minjerk_s(u)
        target = START + s * (END - START)
        q_cmd = solve_ik(panda, target)
        command_arm(panda, q_cmd)
        p.stepSimulation()
        if gui:
            time.sleep(DT)
            update_marker(marker, target)

        actual, quat = ee_state(panda)
        q, qd = joint_state(panda)
        rows.append({
            "phase": "motion",
            "target": target.copy(),
            "actual": actual.copy(),
            "quat": quat.copy(),
            "q_cmd": q_cmd.copy(),
            "q": q.copy(),
            "qd": qd.copy(),
        })
        if gui:
            draw_counter += 1
            if draw_counter % 4 == 0:
                draw_trail(trail_prev, actual, RGB["cartesian"])
                trail_prev = actual

    for _ in range(HOLD_STEPS):
        target = END.copy()
        q_cmd = solve_ik(panda, target)
        command_arm(panda, q_cmd)
        p.stepSimulation()
        if gui:
            time.sleep(DT)
            update_marker(marker, target)
        actual, quat = ee_state(panda)
        q, qd = joint_state(panda)
        rows.append({
            "phase": "hold",
            "target": target.copy(),
            "actual": actual.copy(),
            "quat": quat.copy(),
            "q_cmd": q_cmd.copy(),
            "q": q.copy(),
            "qd": qd.copy(),
        })
        if gui:
            draw_counter += 1
            if draw_counter % 4 == 0:
                draw_trail(trail_prev, actual, RGB["cartesian"])
                trail_prev = actual

    if gui:
        p.removeBody(marker)
    return rows


def run_joint(panda, q_start, q_end, gui):
    rows = []
    trail_prev = ee_pos(panda)
    draw_counter = 0

    for k in range(MOVE_STEPS):
        u = k / (MOVE_STEPS - 1)
        s = minjerk_s(u)
        q_cmd = (1.0 - s) * q_start + s * q_end
        command_arm(panda, q_cmd)
        p.stepSimulation()
        if gui:
            time.sleep(DT)

        actual, quat = ee_state(panda)
        q, qd = joint_state(panda)
        rows.append({
            "phase": "motion",
            "target": START + s * (END - START),  # 只作为直线参考
            "actual": actual.copy(),
            "quat": quat.copy(),
            "q_cmd": q_cmd.copy(),
            "q": q.copy(),
            "qd": qd.copy(),
        })
        if gui:
            draw_counter += 1
            if draw_counter % 4 == 0:
                draw_trail(trail_prev, actual, RGB["joint"])
                trail_prev = actual

    for _ in range(HOLD_STEPS):
        command_arm(panda, q_end)
        p.stepSimulation()
        if gui:
            time.sleep(DT)
        actual, quat = ee_state(panda)
        q, qd = joint_state(panda)
        rows.append({
            "phase": "hold",
            "target": END.copy(),
            "actual": actual.copy(),
            "quat": quat.copy(),
            "q_cmd": q_end.copy(),
            "q": q.copy(),
            "qd": qd.copy(),
        })
        if gui:
            draw_counter += 1
            if draw_counter % 4 == 0:
                draw_trail(trail_prev, actual, RGB["joint"])
                trail_prev = actual
    return rows


# ============================================================
# 5. 分析
# ============================================================
def analyse(rows, name, q_start, q_end):
    target = np.array([r["target"] for r in rows])
    actual = np.array([r["actual"] for r in rows])
    quat = np.array([r["quat"] for r in rows])
    q = np.array([r["q"] for r in rows])
    qd = np.array([r["qd"] for r in rows])
    q_cmd = np.array([r["q_cmd"] for r in rows])
    phase = np.array([r["phase"] for r in rows])
    motion = phase == "motion"
    n = int(motion.sum())

    act = actual[motion]
    # 末端路径偏离理想线段（垂直距离）
    d = END - START
    denom = float(d @ d)
    rel = act - START
    s = np.clip((rel @ d) / denom, 0.0, 1.0)
    closest = START + s[:, None] * d[None, :]
    perp = np.linalg.norm(act - closest, axis=1)

    orient = np.array([quat_angle_deg(qq, TARGET_ORN)
                       for qq in quat[motion]])
    q_err = q[motion] - q_cmd[motion]
    q_err_norm = np.linalg.norm(q_err, axis=1)
    q_err_max = float(np.abs(q_err).max())
    qd_m = qd[motion]

    path_len = float(np.linalg.norm(np.diff(act, axis=0), axis=1).sum())
    final_err = float(np.linalg.norm(actual[-1] - END))

    return {
        "name": name,
        "t": np.arange(n) * DT,
        "actual": act,
        "perp": perp,
        "orient": orient,
        "q_err": q_err_norm,
        "q_err_max": q_err_max,
        "max_dev": float(perp.max()),
        "mean_dev": float(perp.mean()),
        "max_orient": float(orient.max()),
        "mean_orient": float(orient.mean()),
        "max_q1": float(np.abs(qd_m[:, 0]).max()),
        "max_qd": float(np.abs(qd_m).max()),
        "max_joint_err": float(np.abs(q_err).max()),
        "path_len": path_len,
        "final_err": final_err,
        "min_z": float(act[:, 2].min()),
        "q_start": q_start.copy(),
        "q_end": q_end.copy(),
    }


# ============================================================
# 6. 画图
# ============================================================
def set_equal_3d(ax, pts):
    lo = pts.min(axis=0)
    hi = pts.max(axis=0)
    c = (lo + hi) / 2.0
    r = max(float((hi - lo).max()) / 2.0, 0.05)
    ax.set_xlim(c[0] - r, c[0] + r)
    ax.set_ylim(c[1] - r, c[1] + r)
    ax.set_zlim(c[2] - r, c[2] + r)
    ax.set_box_aspect([1, 1, 1])


def make_plot(cart, joint, path):
    fig = plt.figure(figsize=(18, 10))
    fig.suptitle("Step 3  |  Cartesian-space vs joint-space interpolation "
                 "(same min-jerk timing)", fontsize=14)

    # (1) 3D 路径
    ax = fig.add_subplot(231, projection="3d")
    ax.plot([START[0], END[0]], [START[1], END[1]], [START[2], END[2]],
            "--", color="0.5", lw=1.5, label="desired straight line")
    ax.plot(cart["actual"][:, 0], cart["actual"][:, 1],
            cart["actual"][:, 2], color=COLORS["cartesian"],
            lw=2, label="Cartesian interp")
    ax.plot(joint["actual"][:, 0], joint["actual"][:, 1],
            joint["actual"][:, 2], color=COLORS["joint"],
            lw=2, label="joint-space interp")
    ax.scatter(*START, color="green", s=35)
    ax.scatter(*END, color="green", s=35)
    ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]"); ax.set_zlabel("z [m]")
    ax.set_title("3D end-effector path")
    ax.legend(fontsize=7)
    set_equal_3d(ax, np.vstack([cart["actual"], joint["actual"],
                               START[None, :], END[None, :]]))

    # (2) XY 俯视
    ax = fig.add_subplot(232)
    ax.plot([START[0], END[0]], [START[1], END[1]],
            "--", color="0.5", lw=1.5, label="desired")
    ax.plot(cart["actual"][:, 0], cart["actual"][:, 1],
            color=COLORS["cartesian"], lw=2, label="Cartesian")
    ax.plot(joint["actual"][:, 0], joint["actual"][:, 1],
            color=COLORS["joint"], lw=2, label="joint-space")
    ax.scatter([START[0], END[0]], [START[1], END[1]],
               color="green", s=35)
    ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]")
    ax.set_title("Top view (XY)")
    ax.set_aspect("equal", adjustable="box")
    ax.legend(fontsize=8)
    ax.grid(alpha=0.3)

    # (3) 姿态误差
    ax = fig.add_subplot(233)
    ax.plot(cart["t"], cart["orient"], color=COLORS["cartesian"],
            lw=2, label="Cartesian")
    ax.plot(joint["t"], joint["orient"], color=COLORS["joint"],
            lw=2, label="joint-space")
    ax.set_xlabel("time [s]"); ax.set_ylabel("orientation error [deg]")
    ax.set_title("Gripper orientation vs downward target")
    ax.legend(fontsize=8); ax.grid(alpha=0.3)

    # (4) 偏离直线的距离
    ax = fig.add_subplot(234)
    ax.plot(cart["t"], cart["perp"] * 1000, color=COLORS["cartesian"],
            lw=2, label="Cartesian")
    ax.plot(joint["t"], joint["perp"] * 1000, color=COLORS["joint"],
            lw=2, label="joint-space")
    ax.set_xlabel("time [s]"); ax.set_ylabel("distance from line [mm]")
    ax.set_title("Path deviation from straight line")
    ax.legend(fontsize=8); ax.grid(alpha=0.3)

    # (5) 关节跟踪误差
    ax = fig.add_subplot(235)
    ax.plot(cart["t"], cart["q_err"], color=COLORS["cartesian"],
            lw=2, label="Cartesian")
    ax.plot(joint["t"], joint["q_err"], color=COLORS["joint"],
            lw=2, label="joint-space")
    ax.set_xlabel("time [s]"); ax.set_ylabel("||q - q_cmd|| [rad]")
    ax.set_title("Joint tracking error norm")
    ax.legend(fontsize=8); ax.grid(alpha=0.3)

    # (6) 表格
    ax = fig.add_subplot(236)
    ax.axis("off")
    lines = [
        "Metrics",
        "=" * 78,
        f"{'method':<18}{'path dev':>12}{'orient':>10}{'joint err':>12}"
        f"{'q1 max':>10}{'end err':>10}",
        f"{'':<18}{'mm':>12}{'deg':>10}{'rad':>12}"
        f"{'rad/s':>10}{'mm':>10}",
        "-" * 78,
    ]
    for r, nm in ((cart, "Cartesian"), (joint, "joint-space")):
        lines.append(
            f"{nm:<18}{r['max_dev'] * 1000:12.1f}"
            f"{r['max_orient']:10.1f}{r['q_err_max']:12.3f}"
            f"{r['max_q1']:10.2f}{r['final_err'] * 1000:10.2f}"
        )
    lines += [
        "=" * 78,
        f"Cartesian path length : {cart['path_len'] * 100:.1f} cm",
        f"joint path length     : {joint['path_len'] * 100:.1f} cm",
        f"desired length        : {DIST * 100:.1f} cm",
        f"Cartesian min z       : {cart['min_z']:.3f} m",
        f"joint min z           : {joint['min_z']:.3f} m",
        "",
        "orientation: angle between actual gripper quaternion",
        "             and the downward target quaternion",
        "joint err : max |q_actual - q_cmd| over 7 joints",
    ]
    ax.text(0.0, 1.0, "\n".join(lines), va="top", family="monospace",
            fontsize=9)

    fig.tight_layout(rect=[0, 0, 1, 0.95])
    fig.savefig(path, dpi=150)
    plt.close(fig)


# ============================================================
# 7. 主流程
# ============================================================
def print_summary(cart, joint):
    print("\n===== Cartesian vs joint-space interpolation =====")
    print("Cartesian: path dev max %.1f mm, orient max %.1f deg, "
          "q_err max %.3f rad, q1 %.2f rad/s, end %.2f mm"
          % (cart["max_dev"] * 1000, cart["max_orient"],
             cart["q_err_max"], cart["max_q1"],
             cart["final_err"] * 1000))
    print("Joint    : path dev max %.1f mm, orient max %.1f deg, "
          "q_err max %.3f rad, q1 %.2f rad/s, end %.2f mm"
          % (joint["max_dev"] * 1000, joint["max_orient"],
             joint["q_err_max"], joint["max_q1"],
             joint["final_err"] * 1000))
    print("Cartesian path length %.2f cm | joint %.2f cm | desired %.2f cm"
          % (cart["path_len"] * 100, joint["path_len"] * 100, DIST * 100))
    print("min z: Cartesian %.3f m | joint %.3f m"
          % (cart["min_z"], joint["min_z"]))


def save_csv(cart, joint, path):
    fields = ["method", "max_dev_mm", "mean_dev_mm", "max_orient_deg",
              "mean_orient_deg", "max_joint_err_rad", "max_q1_rad_s",
              "max_qd_rad_s", "path_len_cm", "final_err_mm", "min_z_m"]
    with open(path, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(fields)
        for name, r in (("cartesian", cart), ("joint", joint)):
            w.writerow([
                name, f"{r['max_dev'] * 1000:.3f}",
                f"{r['mean_dev'] * 1000:.3f}", f"{r['max_orient']:.3f}",
                f"{r['mean_orient']:.3f}", f"{r['q_err_max']:.4f}",
                f"{r['max_q1']:.4f}", f"{r['max_qd']:.4f}",
                f"{r['path_len'] * 100:.3f}",
                f"{r['final_err'] * 1000:.3f}", f"{r['min_z']:.4f}",
            ])


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--gui", action="store_true",
                        help="打开 PyBullet 窗口演示（默认）")
    parser.add_argument("--headless", action="store_true",
                        help="不打开窗口，只计算并出图")
    args = parser.parse_args()
    if args.gui and args.headless:
        parser.error("--gui 和 --headless 只能选一个")
    gui = not args.headless

    os.makedirs(OUT_DIR, exist_ok=True)
    connect(gui)
    panda = build_scene()
    draw_reference(gui)

    print("预热到起点 ...")
    q_start = warmup_to_start(panda, gui)
    print("start pos:", np.round(ee_pos(panda), 4),
          " error %.3f mm" % (np.linalg.norm(ee_pos(panda) - START) * 1000))

    print("q_start:", np.round(q_start, 3))

    print("\n[1/2] Cartesian-space interpolation ...")
    cart_rows = run_cartesian(panda, gui)
    # 关键：不要“从起点一次解 IK 得到终点”当作 q_end。
    # 单次 IK 可能收敛到差的局部解（实测会差 24 mm）。
    # 先让 Cartesian 路径真正走到 END，再读取那里的实际关节角。
    q_end = joint_state(panda)[0].copy()
    cart = analyse(cart_rows, "cartesian", q_start, q_end)
    print("q_end from reached Cartesian endpoint:",
          np.round(q_end, 3))
    print("Cartesian endpoint error: %.2f mm" %
          (np.linalg.norm(ee_pos(panda) - END) * 1000))

    print("[2/2] joint-space interpolation ...")
    reset_to_start(panda, q_start, gui)
    joint_rows = run_joint(panda, q_start, q_end, gui)
    joint = analyse(joint_rows, "joint", q_start, q_end)

    print_summary(cart, joint)
    png_path = os.path.join(OUT_DIR, "step3_interpolation.png")
    csv_path = os.path.join(OUT_DIR, "step3_interpolation.csv")
    make_plot(cart, joint, png_path)
    save_csv(cart, joint, csv_path)
    print("\nCSV :", csv_path)
    print("图片:", png_path)

    if gui:
        p.addUserDebugText(
            "Cartesian: dev %.1f mm, orient %.1f deg"
            % (cart["max_dev"] * 1000, cart["max_orient"]),
            [0.22, 0.34, 0.55],
            textColorRGB=RGB["cartesian"], textSize=1.0, lifeTime=0)
        p.addUserDebugText(
            "joint-space: dev %.1f mm, orient %.1f deg"
            % (joint["max_dev"] * 1000, joint["max_orient"]),
            [0.22, 0.38, 0.51],
            textColorRGB=RGB["joint"], textSize=1.0, lifeTime=0)
        print("\n动画结束，窗口保留 10 秒。绿色点=起点/终点，"
              "蓝/橙粗线=两种方法的实际轨迹。")
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
