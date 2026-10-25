"""
第 5 天 · 轨迹可视化实验 · 第 1 步
=====================================
让末端沿一条空间直线运动，同时记录三样东西：

  1. 指令位置（IK 每一帧想让它去哪）
  2. 实际位置（仿真里它真正到了哪）
  3. 7 个关节的角度和角速度

运行结束后生成一张六联图，并保存 CSV，方便看清
“规划的理想轨迹”和“控制器真正走出来的轨迹”差在哪里。

用法：
  python project8_trajectory/step1_trace.py          # 无窗口，直接出图
  python project8_trajectory/step1_trace.py --gui    # 打开 PyBullet 看红色实时轨迹
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
DT = 1.0 / 240.0                 # 仿真步长：每秒 240 步
MOVE_STEPS = 240                 # 走完直线用 600 步 ≈ 2.5 s
HOLD_STEPS = 240                 # 到终点后再保持 1 s，观察收敛
WARMUP_STEPS = 360               # 从初始姿态走到起点的过渡
WARMUP_HOLD = 240

START = np.array([0.40, -0.18, 0.35])   # 直线起点（世界坐标，米）
END = np.array([0.55, 0.18, 0.35])      # 直线终点
TARGET_ORN = p.getQuaternionFromEuler([0, math.pi, 0])  # 夹爪朝下

EE_LINK = 11                     # panda_grasptarget
ARM_JOINTS = list(range(7))      # 参与运动的 7 个关节

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
OUT_DIR = os.path.join(BASE_DIR, "output")


# ============================================================
# 2. 仿真环境
# ============================================================
def connect(gui):
    if gui:
        p.connect(p.GUI)
        p.resetDebugVisualizerCamera(
            cameraDistance=1.25, cameraYaw=45, cameraPitch=-35,
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

    # 夹爪保持张开，避免手指挡住视线
    for j in range(p.getNumJoints(panda)):
        info = p.getJointInfo(panda, j)
        if b"finger" in info[1]:
            p.setJointMotorControl2(panda, j, p.POSITION_CONTROL,
                                    targetPosition=0.04, force=200)
    return panda


# ============================================================
# 3. 底层动作：IK → 位置控制 → 仿真步进
# ============================================================
def ee_pos(panda):
    """末端连杆坐标系原点，和 IK 的目标参考点一致（不是质心）。"""
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


def read_joint_state(panda):
    q, qd = [], []
    for j in ARM_JOINTS:
        state = p.getJointState(panda, j)
        q.append(state[0])
        qd.append(state[1])
    return np.array(q), np.array(qd)


def record_row(panda, phase, target, actual, q_cmd):
    q, qd = read_joint_state(panda)
    err = actual - target
    return {
        "phase": phase,
        "target": target.copy(),
        "actual": actual.copy(),
        "err": err,
        "q_cmd": q_cmd.copy(),
        "q": q,
        "qd": qd,
    }


def linear_move(panda, p0, p1, steps, phase, record, gui,
                trail_color=None, trail_width=3.0):
    """
    每一帧把指令位置沿 p0→p1 线性推进一点，再解一次 IK。

    注意：这里“直线”是指令空间里的直线；真正走出来的路线由
    电机跟踪能力决定，未必严格是直线。
    """
    rows = []
    prev_actual = ee_pos(panda)
    for k in range(steps):
        s = 1.0 if steps == 1 else k / (steps - 1)
        target = p0 + s * (p1 - p0)
        q_cmd = solve_ik(panda, target)
        command_arm(panda, q_cmd)
        p.stepSimulation()
        if gui:
            time.sleep(DT)

        actual = ee_pos(panda)
        if record:
            rows.append(record_row(panda, phase, target, actual, q_cmd))

        if gui and trail_color is not None:
            if np.linalg.norm(actual - prev_actual) > 1e-6:
                p.addUserDebugLine(prev_actual.tolist(), actual.tolist(),
                                   trail_color, lineWidth=trail_width,
                                   lifeTime=0)
        prev_actual = actual
    return rows


def hold_position(panda, target, steps, phase, record, gui):
    """目标不动，继续保持，观察误差是否收敛到 0。"""
    rows = []
    prev_actual = ee_pos(panda)
    for _ in range(steps):
        q_cmd = solve_ik(panda, target)
        command_arm(panda, q_cmd)
        p.stepSimulation()
        if gui:
            time.sleep(DT)

        actual = ee_pos(panda)
        if record:
            rows.append(record_row(panda, phase, target, actual, q_cmd))

        if gui and np.linalg.norm(actual - prev_actual) > 1e-6:
            p.addUserDebugLine(prev_actual.tolist(), actual.tolist(),
                               [0.9, 0.1, 0.1], lineWidth=3.0, lifeTime=0)
        prev_actual = actual
    return rows


def draw_desired_and_markers():
    """在 GUI 里把指令直线画成绿色，并标出起点/终点。"""
    for k in range(80):
        a = START + (END - START) * (k / 80.0)
        b = START + (END - START) * ((k + 1) / 80.0)
        p.addUserDebugLine(a.tolist(), b.tolist(),
                           [0.1, 0.8, 0.1], lineWidth=2.0, lifeTime=0)
    p.addUserDebugText("START", (START + [0, 0, 0.05]).tolist(),
                       textColorRGB=[0.0, 0.7, 0.0], textSize=1.0, lifeTime=0)
    p.addUserDebugText("END", (END + [0, 0, 0.05]).tolist(),
                       textColorRGB=[0.0, 0.7, 0.0], textSize=1.0, lifeTime=0)


# ============================================================
# 4. 数据分析
# ============================================================
def analyse(rows):
    target = np.array([r["target"] for r in rows])
    actual = np.array([r["actual"] for r in rows])
    err = actual - target
    err_norm = np.linalg.norm(err, axis=1)
    q = np.array([r["q"] for r in rows])
    qd = np.array([r["qd"] for r in rows])
    q_cmd = np.array([r["q_cmd"] for r in rows])
    phase = np.array([r["phase"] for r in rows])
    t = np.arange(len(rows)) * DT

    moving = phase == "motion"
    hold = ~moving

    # 路径长度：相邻实际点之间的距离和
    seg = np.linalg.norm(np.diff(actual[moving], axis=0), axis=1)
    actual_len = float(seg.sum())
    desired_len = float(np.linalg.norm(END - START))

    # 实际路线偏离“理想线段”的最大距离
    d = END - START
    denom = float(d @ d)
    rel = actual[moving] - START
    s = np.clip((rel @ d) / denom, 0.0, 1.0)
    closest = START + s[:, None] * d[None, :]
    line_dev = np.linalg.norm(actual[moving] - closest, axis=1)

    # 把误差拆成“沿路径方向”和“垂直路径方向”：
    #   沿路径的误差 ≈ 控制器跟不上目标造成的滞后；
    #   垂直路径的误差才是“路线走弯了”。
    unit = d / np.linalg.norm(d)
    along = err[moving] @ unit
    perp = np.linalg.norm(err[moving] - along[:, None] * unit[None, :],
                          axis=1)
    target_speed = (np.linalg.norm(np.diff(target[moving], axis=0), axis=1)
                    .sum() / ((moving.sum() - 1) * DT))
    lag_ms = float(-along.mean() / target_speed * 1000.0)

    # 到终点后，误差第一次小于 2 mm 所需的时间
    hold_idx = np.where(hold)[0]
    settle_idx = hold_idx[err_norm[hold_idx] < 0.002]
    settle_time = (float((settle_idx[0] - hold_idx[0] + 1) * DT)
                   if len(settle_idx) else float("nan"))

    # 指令关节速度和实际关节速度的最大值
    cmd_qd = np.abs(np.diff(q_cmd, axis=0)) / DT
    max_cmd_qd = float(cmd_qd.max()) if len(cmd_qd) else 0.0
    max_actual_qd = float(np.abs(qd).max())
    max_joint = int(np.abs(qd).argmax() % 7) + 1

    metrics = {
        "max_err": float(err_norm[moving].max()),
        "rms_err": float(np.sqrt(np.mean(err_norm[moving] ** 2))),
        "mean_err": float(err_norm[moving].mean()),
        "line_dev": float(line_dev.max()),
        "mean_along": float(along.mean()),
        "mean_perp": float(perp.mean()),
        "target_speed": float(target_speed),
        "lag_ms": lag_ms,
        "final_err": float(err_norm[-1]),
        "actual_len": actual_len,
        "desired_len": desired_len,
        "len_ratio": actual_len / desired_len,
        "max_cmd_qd": max_cmd_qd,
        "max_actual_qd": max_actual_qd,
        "max_joint": max_joint,
        "settle_time": settle_time,
    }
    return t, target, actual, err, err_norm, q, qd, q_cmd, phase, metrics


# ============================================================
# 5. 出图
# ============================================================
def set_3d_equal(ax, points):
    lo = points.min(axis=0)
    hi = points.max(axis=0)
    center = (lo + hi) / 2.0
    radius = max(float((hi - lo).max()) / 2.0, 0.05)
    ax.set_xlim(center[0] - radius, center[0] + radius)
    ax.set_ylim(center[1] - radius, center[1] + radius)
    ax.set_zlim(center[2] - radius, center[2] + radius)
    ax.set_box_aspect([1, 1, 1])


def make_plot(t, target, actual, err, err_norm, q, qd, phase, metrics, path):
    moving = phase == "motion"
    t0, t1 = t[0], t[-1]
    t_hold = t[moving][-1] if moving.any() else t1

    fig = plt.figure(figsize=(18, 10))
    fig.suptitle("Step 1  |  Cartesian straight line: commanded vs actual  "
                 "(IK + position control)", fontsize=14)

    # (1) 3D 轨迹
    ax = fig.add_subplot(231, projection="3d")
    ax.plot(target[:, 0], target[:, 1], target[:, 2],
            "--", color="tab:green", lw=2, label="commanded")
    ax.plot(actual[:, 0], actual[:, 1], actual[:, 2],
            "-", color="tab:red", lw=1.5, label="actual")
    ax.scatter(*START, color="green", s=35)
    ax.scatter(*END, color="green", s=35)
    ax.scatter(*actual[-1], color="red", s=35)
    ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]"); ax.set_zlabel("z [m]")
    ax.set_title("3D path")
    ax.legend(loc="upper left", fontsize=8)
    set_3d_equal(ax, np.vstack([target, actual]))

    # (2) 俯视图：直线走得直不直，一眼就能看出来
    ax = fig.add_subplot(232)
    ax.plot(target[:, 0], target[:, 1], "--", color="tab:green",
            lw=2, label="commanded")
    ax.plot(actual[:, 0], actual[:, 1], "-", color="tab:red",
            lw=1.5, label="actual")
    ax.scatter([START[0], END[0]], [START[1], END[1]],
               color="green", s=35)
    ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]")
    ax.set_title("Top view (XY)")
    ax.set_aspect("equal", adjustable="box")
    ax.legend(fontsize=8)

    # (3) 跟踪误差随时间
    ax = fig.add_subplot(233)
    ax.axvspan(t_hold, t1, color="gray", alpha=0.15, label="hold")
    for i, name in enumerate("xyz"):
        ax.plot(t, err[:, i] * 1000, lw=1.0, label=f"e_{name}")
    ax.plot(t, err_norm * 1000, color="black", lw=1.8, label="|e|")
    ax.set_xlabel("time [s]"); ax.set_ylabel("error [mm]")
    ax.set_title("Tracking error")
    ax.legend(fontsize=7, ncol=2)
    ax.set_xlim(t0, t1)

    # (4) 7 个关节角
    ax = fig.add_subplot(234)
    for j in range(7):
        ax.plot(t, q[:, j], lw=1.2, label=f"q{j + 1}")
    ax.axvspan(t_hold, t1, color="gray", alpha=0.15)
    ax.set_xlabel("time [s]"); ax.set_ylabel("joint angle [rad]")
    ax.set_title("Joint angles (actual)")
    ax.legend(fontsize=6, ncol=4)
    ax.set_xlim(t0, t1)

    # (5) 关节角速度
    ax = fig.add_subplot(235)
    for j in range(7):
        ax.plot(t, qd[:, j], lw=1.0, label=f"qd{j + 1}")
    ax.axvspan(t_hold, t1, color="gray", alpha=0.15)
    ax.set_xlabel("time [s]"); ax.set_ylabel("joint speed [rad/s]")
    ax.set_title("Joint velocities (actual)")
    ax.legend(fontsize=6, ncol=4)
    ax.set_xlim(t0, t1)

    # (6) 数值指标
    ax = fig.add_subplot(236)
    ax.axis("off")
    txt = (
        "Metrics\n"
        "----------------------------------------\n"
        f"max |error| (motion) : {metrics['max_err'] * 1000:6.1f} mm\n"
        f"RMS |error|          : {metrics['rms_err'] * 1000:6.1f} mm\n"
        f"mean |error|         : {metrics['mean_err'] * 1000:6.1f} mm\n"
        f"max deviation / line : {metrics['line_dev'] * 1000:6.1f} mm\n"
        f"mean along-path error: {metrics['mean_along'] * 1000:6.1f} mm\n"
        f"mean cross-path error: {metrics['mean_perp'] * 1000:6.1f} mm\n"
        f"command speed        : {metrics['target_speed']:6.3f} m/s\n"
        f"equivalent lag       : {metrics['lag_ms']:6.1f} ms\n"
        f"final |error|        : {metrics['final_err'] * 1000:6.1f} mm\n"
        f"settle to 2 mm       : {metrics['settle_time']:6.3f} s\n"
        "----------------------------------------\n"
        f"commanded length     : {metrics['desired_len'] * 100:6.2f} cm\n"
        f"actual path length   : {metrics['actual_len'] * 100:6.2f} cm\n"
        f"length ratio         : {metrics['len_ratio']:6.4f}\n"
        "----------------------------------------\n"
        f"max commanded qdot   : {metrics['max_cmd_qd']:6.2f} rad/s\n"
        f"max actual qdot      : {metrics['max_actual_qd']:6.2f} rad/s\n"
        f"  (joint q{metrics['max_joint']})\n"
    )
    ax.text(0.0, 1.0, txt, va="top", ha="left",
            family="monospace", fontsize=10)

    fig.tight_layout(rect=[0, 0, 1, 0.95])
    fig.savefig(path, dpi=150)
    plt.close(fig)


def save_csv(rows, path):
    header = (["phase"]
              + ["des_x", "des_y", "des_z"]
              + ["act_x", "act_y", "act_z"]
              + ["err_x", "err_y", "err_z", "err_norm"]
              + [f"cmd_q{i + 1}" for i in range(7)]
              + [f"q{i + 1}" for i in range(7)]
              + [f"qd{i + 1}" for i in range(7)])
    with open(path, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(header)
        for r in rows:
            w.writerow(
                [r["phase"]]
                + [f"{v:.6f}" for v in r["target"]]
                + [f"{v:.6f}" for v in r["actual"]]
                + [f"{v:.6f}" for v in r["err"]]
                + [f"{np.linalg.norm(r['err']):.6f}"]
                + [f"{v:.6f}" for v in r["q_cmd"]]
                + [f"{v:.6f}" for v in r["q"]]
                + [f"{v:.6f}" for v in r["qd"]]
            )


# ============================================================
# 6. 主流程
# ============================================================
def run(gui):
    connect(gui)
    panda = build_scene()
    os.makedirs(OUT_DIR, exist_ok=True)

    initial = ee_pos(panda)
    print("末端初始位置       :", np.round(initial, 3))
    print("规划直线           :", np.round(START, 3), "->", np.round(END, 3),
          "  长度 %.3f m" % np.linalg.norm(END - START))

    if gui:
        draw_desired_and_markers()

    # 先移到起点，并停稳；这段不写入实验数据
    print("移到起点并稳定 ...")
    linear_move(panda, initial, START, WARMUP_STEPS, "warmup", False, gui,
                trail_color=[0.5, 0.5, 0.5], trail_width=1.5)
    hold_position(panda, START, WARMUP_HOLD, "warmup_hold", False, gui)
    e0 = np.linalg.norm(ee_pos(panda) - START)
    print("起点稳定后误差     : %.1f mm" % (e0 * 1000))

    # 正式实验：直线 + 终点保持
    print("沿直线运动 %d 步（%.2f s）..." % (MOVE_STEPS, MOVE_STEPS * DT))
    rows = linear_move(panda, START, END, MOVE_STEPS, "motion", True, gui,
                       trail_color=[0.9, 0.1, 0.1], trail_width=3.0)
    print("终点保持 %d 步（%.2f s）..." % (HOLD_STEPS, HOLD_STEPS * DT))
    rows += hold_position(panda, END, HOLD_STEPS, "hold", True, gui)

    t, target, actual, err, err_norm, q, qd, q_cmd, phase, metrics = \
        analyse(rows)

    print("\n===== 结果 =====")
    print("运动段最大误差     : %.1f mm" % (metrics["max_err"] * 1000))
    print("运动段 RMS 误差    : %.1f mm" % (metrics["rms_err"] * 1000))
    print("偏离理想直线最大   : %.1f mm" % (metrics["line_dev"] * 1000))
    print("沿路径平均滞后     : %.1f mm" % (metrics["mean_along"] * 1000))
    print("垂直路径平均偏差   : %.1f mm" % (metrics["mean_perp"] * 1000))
    print("指令速度           : %.3f m/s" % metrics["target_speed"])
    print("等效时间滞后       : %.1f ms" % metrics["lag_ms"])
    print("终点最终误差       : %.1f mm" % (metrics["final_err"] * 1000))
    print("指令路径长度       : %.2f cm" % (metrics["desired_len"] * 100))
    print("实际路径长度       : %.2f cm" % (metrics["actual_len"] * 100))
    print("实际/指令长度比    : %.4f" % metrics["len_ratio"])
    print("最大指令关节速度   : %.2f rad/s" % metrics["max_cmd_qd"])
    print("最大实际关节速度   : %.2f rad/s（q%d）"
          % (metrics["max_actual_qd"], metrics["max_joint"]))
    print("终点收敛到 2mm 用时: %.3f s" % metrics["settle_time"])

    csv_path = os.path.join(OUT_DIR, "step1_trace.csv")
    png_path = os.path.join(OUT_DIR, "step1_trace.png")
    save_csv(rows, csv_path)
    make_plot(t, target, actual, err, err_norm, q, qd, phase, metrics,
              png_path)
    print("\nCSV :", csv_path)
    print("图片:", png_path)

    if gui:
        print("\nGUI 中绿色是规划直线，红色是真实轨迹。3 秒后关闭窗口...")
        time.sleep(3.0)

    p.disconnect()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--gui", action="store_true",
                        help="打开 PyBullet 窗口，显示规划和实际轨迹")
    args = parser.parse_args()
    try:
        run(args.gui)
    except p.error as exc:
        print("仿真正在退出：", exc)
        try:
            p.disconnect()
        except p.error:
            pass


if __name__ == "__main__":
    main()
