"""
第 5 天 · 轨迹可视化实验 · 第 4 步
=====================================
奇异位形、雅可比与可操作度。

实验内容
--------
取两条长度、时间、速度曲线都相同的直线路径：

  safe  : x 0.36 → 0.50, y=0, z=0.35
  near  : x 0.60 → 0.74, y=0, z=0.35

near 路径更靠近工作空间边缘，末端姿态也更容易接近奇异。
沿两条路径计算：

  - 6×7 雅可比 J = [Jv; Jw]
  - 奇异值 sigma
  - 可操作度 manipulability = prod(sigma)
  - sigma_min（越小越接近奇异）
  - 实际关节速度、跟踪误差

重点：
  - 同等末端速度下，near 路径需要更大的关节速度；
  - sigma_min 很小的时候，某些方向的运动需要 1/sigma_min 的放大；
  - 跟踪误差差不多，不代表运动能力一样。

运行：
  python project8_trajectory/step4_singularity.py
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
T_TOTAL = 1.5
STEPS = int(round(T_TOTAL / DT)) + 1
WARMUP_STEPS = 360
WARMUP_HOLD = 240
HOLD_STEPS = 240

SAFE_PATH = (np.array([0.36, 0.0, 0.35]),
             np.array([0.50, 0.0, 0.35]))
NEAR_PATH = (np.array([0.60, 0.0, 0.35]),
             np.array([0.74, 0.0, 0.35]))
TARGET_ORN = p.getQuaternionFromEuler([0, math.pi, 0])

EE_LINK = 11
ARM_JOINTS = list(range(7))

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
OUT_DIR = os.path.join(BASE_DIR, "output")

COLORS = {"safe": "tab:blue", "near": "tab:red"}


# ============================================================
# 2. 仿真基础
# ============================================================
def connect(gui):
    if gui:
        p.connect(p.GUI)
        p.resetDebugVisualizerCamera(
            cameraDistance=1.4, cameraYaw=45, cameraPitch=-32,
            cameraTargetPosition=[0.55, 0.0, 0.30],
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
    movable = [j for j in range(p.getNumJoints(panda))
               if p.getJointInfo(panda, j)[2] != p.JOINT_FIXED]
    return panda, movable


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


def hold(panda, target, steps, gui):
    for _ in range(steps):
        command_arm(panda, solve_ik(panda, target))
        p.stepSimulation()
        if gui:
            time.sleep(DT)


def warmup_to(panda, target, gui):
    initial = ee_pos(panda)
    for k in range(WARMUP_STEPS):
        s = k / (WARMUP_STEPS - 1)
        command_arm(panda, solve_ik(
            panda, initial + s * (target - initial)))
        p.stepSimulation()
        if gui:
            time.sleep(DT)
    hold(panda, target, WARMUP_HOLD, gui)


# ============================================================
# 3. 雅可比
# ============================================================
def jacobian_metrics(panda, movable):
    """
    PyBullet 的 calculateJacobian 需要传入所有可动关节（Panda 是 9 个：
    7 个手臂 + 2 个手指），否则会报 numDof 错误。
    返回的矩阵是 3×9，前 7 列才对应手臂关节。
    """
    all_pos = [p.getJointState(panda, j)[0] for j in movable]
    zeros = [0.0] * len(all_pos)
    Jl, Ja = p.calculateJacobian(
        panda, EE_LINK, [0.0, 0.0, 0.0],
        all_pos, zeros, zeros,
    )
    Jv = np.array(Jl, dtype=float)[:, :7]
    Jw = np.array(Ja, dtype=float)[:, :7]
    J = np.vstack([Jv, Jw])             # 6×7
    sv_full = np.linalg.svd(J, compute_uv=False)
    sv_pos = np.linalg.svd(Jv, compute_uv=False)
    return {
        "Jv": Jv,
        "Jw": Jw,
        "J": J,
        "sv_full": sv_full,
        "sv_pos": sv_pos,
        "sigma_min": float(sv_full.min()),
        "manip": float(np.prod(sv_full)),
        "sigma_min_pos": float(sv_pos.min()),
        "manip_pos": float(np.prod(sv_pos)),
    }


# ============================================================
# 3.5 GUI 辅助
# ============================================================
RGB = {"safe": [0.10, 0.40, 1.00],
       "near": [0.90, 0.10, 0.10]}


def draw_static_paths(gui):
    if not gui:
        return
    for name, (a, b), color in (
            ("safe", SAFE_PATH, RGB["safe"]),
            ("near", NEAR_PATH, RGB["near"])):
        for k in range(30):
            p0 = a + (b - a) * (k / 30.0)
            p1 = a + (b - a) * ((k + 1) / 30.0)
            p.addUserDebugLine(p0.tolist(), p1.tolist(), color,
                               lineWidth=1.0, lifeTime=0)
        p.addUserDebugText(name, (a + [0, 0, 0.07]).tolist(),
                           textColorRGB=color, textSize=1.1, lifeTime=0)
    p.addUserDebugText("blue = safe   red = near-singular",
                       [0.28, 0.30, 0.58],
                       textColorRGB=[0.0, 0.0, 0.0], textSize=1.0,
                       lifeTime=0)


def make_marker(color):
    vis = p.createVisualShape(p.GEOM_SPHERE, radius=0.010,
                              rgbaColor=[color[0], color[1], color[2], 1.0])
    return p.createMultiBody(baseMass=0, baseVisualShapeIndex=vis,
                             basePosition=SAFE_PATH[0].tolist())


def update_marker(marker, pos):
    p.resetBasePositionAndOrientation(marker, pos.tolist(),
                                      [0, 0, 0, 1])


def draw_trail(prev, actual, color):
    if np.linalg.norm(actual - prev) > 1e-6:
        p.addUserDebugLine(prev.tolist(), actual.tolist(), color,
                           lineWidth=3.2, lifeTime=0)


def make_gauge(text, color):
    return p.addUserDebugText(text, [0.18, 0.30, 0.62],
                              textColorRGB=color, textSize=1.0,
                              lifeTime=0)


def replace_gauge(item_id, text, color):
    if item_id is not None:
        p.removeUserDebugItem(item_id)
    return make_gauge(text, color)


# ============================================================
# 4. 跑一条路径
# ============================================================
def minjerk_s(u):
    return 10.0 * u ** 3 - 15.0 * u ** 4 + 6.0 * u ** 5


def run_path(panda, movable, name, start, end, gui):
    print("  warmup to", np.round(start, 3))
    warmup_to(panda, start, gui)

    color = RGB[name]
    marker = make_marker(color) if gui else None
    gauge = make_gauge(
        f"{name}: booting...", color) if gui else None
    trail_prev = ee_pos(panda)
    draw_counter = 0
    peak_qd = 0.0

    rows = []
    for k in range(STEPS):
        u = k / (STEPS - 1)
        s = minjerk_s(u)
        target = start + s * (end - start)
        q_cmd = solve_ik(panda, target)
        command_arm(panda, q_cmd)
        p.stepSimulation()
        if gui:
            time.sleep(DT)
            update_marker(marker, target)

        actual, quat = ee_state(panda)
        q, qd = joint_state(panda)
        peak_qd = max(peak_qd, float(np.max(np.abs(qd))))
        jm = jacobian_metrics(panda, movable)
        rows.append({
            "phase": "motion",
            "target": target.copy(),
            "actual": actual.copy(),
            "quat": quat.copy(),
            "q_cmd": q_cmd.copy(),
            "q": q.copy(),
            "qd": qd.copy(),
            "err": actual - target,
            "sigma_min": jm["sigma_min"],
            "manip": jm["manip"],
            "sigma_min_pos": jm["sigma_min_pos"],
            "manip_pos": jm["manip_pos"],
            "sv_full": jm["sv_full"],
        })
        if gui:
            draw_counter += 1
            if draw_counter % 4 == 0:
                draw_trail(trail_prev, actual, color)
                trail_prev = actual
            if draw_counter % 24 == 0:
                gauge = replace_gauge(
                    gauge,
                    f"{name}: sigma_min={jm['sigma_min']:.3f}  "
                    f"peak|qdot|={peak_qd:.2f} rad/s",
                    color)

    hold(panda, end, HOLD_STEPS, gui)
    final, final_quat = ee_state(panda)
    q_end, qd_end = joint_state(panda)
    jm_end = jacobian_metrics(panda, movable)
    if gui:
        update_marker(marker, end)
        if gauge is not None:
            p.removeUserDebugItem(gauge)
        p.removeBody(marker)

    metrics = analyse(rows, name, start, end, final, jm_end)
    return metrics


# ============================================================
# 5. 指标
# ============================================================
def analyse(rows, name, start, end, final, jm_end):
    target = np.array([r["target"] for r in rows])
    actual = np.array([r["actual"] for r in rows])
    err = np.array([r["err"] for r in rows])
    qd = np.array([r["qd"] for r in rows])
    sigma = np.array([r["sigma_min"] for r in rows])
    manip = np.array([r["manip"] for r in rows])
    t = np.arange(len(rows)) * DT

    err_n = np.linalg.norm(err, axis=1)
    qd_abs = np.abs(qd)
    max_qd = float(qd_abs.max())
    worst_joint = int(qd_abs.max(axis=0).argmax()) + 1

    path_len = float(np.linalg.norm(np.diff(actual, axis=0), axis=1).sum())
    return {
        "name": name,
        "start": start,
        "end": end,
        "t": t,
        "actual": actual,
        "err": err_n,
        "qd": qd,
        "sigma": sigma,
        "manip": manip,
        "max_qd": max_qd,
        "worst_joint": worst_joint,
        "q1_max": float(qd_abs[:, 0].max()),
        "max_err": float(err_n.max()),
        "mean_sigma": float(sigma.mean()),
        "min_sigma": float(sigma.min()),
        "mean_manip": float(manip.mean()),
        "min_manip": float(manip.min()),
        "final_err": float(np.linalg.norm(final - end)),
        "path_len": path_len,
        "min_z": float(actual[:, 2].min()),
        "sv_end": jm_end["sv_full"],
        "J_end": jm_end["J"],
    }


# ============================================================
# 6. 画图
# ============================================================
def make_plot(safe, near, path):
    fig = plt.figure(figsize=(18, 10))
    fig.suptitle("Step 4  |  Singularity, Jacobian and manipulability "
                 "(same 14 cm path, T=1.5 s)", fontsize=14)

    # (1) sigma_min
    ax = fig.add_subplot(231)
    ax.plot(safe["t"], safe["sigma"], color=COLORS["safe"], lw=2,
            label="safe path")
    ax.plot(near["t"], near["sigma"], color=COLORS["near"], lw=2,
            label="near-singular path")
    ax.set_xlabel("time [s]"); ax.set_ylabel(r"$\sigma_{min}$")
    ax.set_title("Smallest singular value of 6x7 Jacobian")
    ax.legend(fontsize=8); ax.grid(alpha=0.3)

    # (2) manipulability
    ax = fig.add_subplot(232)
    ax.semilogy(safe["t"], safe["manip"], color=COLORS["safe"], lw=2,
                label="safe path")
    ax.semilogy(near["t"], near["manip"], color=COLORS["near"], lw=2,
                label="near-singular path")
    ax.set_xlabel("time [s]"); ax.set_ylabel("manipulability")
    ax.set_title("Manipulability = product of singular values")
    ax.legend(fontsize=8); ax.grid(alpha=0.3, which="both")

    # (3) 关节速度
    ax = fig.add_subplot(233)
    ax.plot(safe["t"], np.max(np.abs(safe["qd"]), axis=1),
            color=COLORS["safe"], lw=2, label="safe path")
    ax.plot(near["t"], np.max(np.abs(near["qd"]), axis=1),
            color=COLORS["near"], lw=2, label="near-singular path")
    ax.set_xlabel("time [s]"); ax.set_ylabel("max |qdot| [rad/s]")
    ax.set_title("Actual joint speed (same Cartesian path length/time)")
    ax.legend(fontsize=8); ax.grid(alpha=0.3)
    ax.text(0.03, 0.95,
            f"near/safe peak = {near['max_qd'] / safe['max_qd']:.2f}x",
            transform=ax.transAxes, va="top", fontsize=9)

    # (4) 跟踪误差
    ax = fig.add_subplot(234)
    ax.plot(safe["t"], safe["err"] * 1000, color=COLORS["safe"], lw=2,
            label="safe path")
    ax.plot(near["t"], near["err"] * 1000, color=COLORS["near"], lw=2,
            label="near-singular path")
    ax.set_xlabel("time [s]"); ax.set_ylabel("|error| [mm]")
    ax.set_title("Tracking error (both track well at this speed)")
    ax.legend(fontsize=8); ax.grid(alpha=0.3)

    # (5) 终点奇异值柱状图
    ax = fig.add_subplot(235)
    idx = np.arange(6)
    width = 0.38
    ax.bar(idx - width / 2, safe["sv_end"], width,
           color=COLORS["safe"], label="safe end")
    ax.bar(idx + width / 2, near["sv_end"], width,
           color=COLORS["near"], label="near end")
    ax.set_yscale("log")
    ax.set_xticks(idx); ax.set_xticklabels([f"s{i+1}" for i in idx])
    ax.set_ylabel("singular value")
    ax.set_title("Six singular values at endpoint")
    ax.legend(fontsize=8); ax.grid(alpha=0.3, axis="y", which="both")

    # (6) 表格
    ax = fig.add_subplot(236)
    ax.axis("off")
    lines = [
        "Metrics",
        "=" * 78,
        f"{'path':<8}{'sigma_min':>11}{'mean':>9}{'max qdot':>10}"
        f"{'max err':>9}{'end err':>9}",
        f"{'':<8}{'min':>11}{'sigma':>9}{'rad/s':>10}"
        f"{'mm':>9}{'mm':>9}",
        "-" * 78,
        f"{'safe':<8}{safe['min_sigma']:11.3f}{safe['mean_sigma']:9.3f}"
        f"{safe['max_qd']:10.2f}{safe['max_err'] * 1000:9.1f}"
        f"{safe['final_err'] * 1000:9.2f}",
        f"{'near':<8}{near['min_sigma']:11.3f}{near['mean_sigma']:9.3f}"
        f"{near['max_qd']:10.2f}{near['max_err'] * 1000:9.1f}"
        f"{near['final_err'] * 1000:9.2f}",
        "=" * 78,
        f"safe path : x {safe['start'][0]:.2f} -> {safe['end'][0]:.2f} m",
        f"near path : x {near['start'][0]:.2f} -> {near['end'][0]:.2f} m",
        f"path length: {safe['path_len'] * 100:.1f} cm each",
        f"near/safe max qdot: {near['max_qd'] / safe['max_qd']:.2f}x",
        "",
        "J is 6x7 (3 linear + 3 angular).",
        "sigma_min small => some directions need",
        "large joint velocity (1/sigma_min).",
    ]
    ax.text(0.0, 1.0, "\n".join(lines), va="top", family="monospace",
            fontsize=9)

    fig.tight_layout(rect=[0, 0, 1, 0.95])
    fig.savefig(path, dpi=150)
    plt.close(fig)


# ============================================================
# 7. 输出
# ============================================================
def print_summary(safe, near):
    print("\n===== Singularity experiment =====")
    print("safe: sigma_min %.3f (mean %.3f), manip mean %.4f, "
          "max qdot %.2f rad/s (q%d), max err %.1f mm, end %.2f mm"
          % (safe["min_sigma"], safe["mean_sigma"],
             safe["mean_manip"], safe["max_qd"], safe["worst_joint"],
             safe["max_err"] * 1000, safe["final_err"] * 1000))
    print("near: sigma_min %.3f (mean %.3f), manip mean %.4f, "
          "max qdot %.2f rad/s (q%d), max err %.1f mm, end %.2f mm"
          % (near["min_sigma"], near["mean_sigma"],
             near["mean_manip"], near["max_qd"], near["worst_joint"],
             near["max_err"] * 1000, near["final_err"] * 1000))
    print("near/safe max qdot ratio: %.2fx"
          % (near["max_qd"] / safe["max_qd"]))


def save_csv(safe, near, path):
    fields = ["path", "x_start", "x_end", "min_sigma", "mean_sigma",
              "mean_manip", "max_qd", "worst_joint", "q1_max",
              "max_err_mm", "final_err_mm", "path_len_cm", "min_z_m"]
    with open(path, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(fields)
        for r in (safe, near):
            w.writerow([
                r["name"], f"{r['start'][0]:.3f}", f"{r['end'][0]:.3f}",
                f"{r['min_sigma']:.4f}", f"{r['mean_sigma']:.4f}",
                f"{r['mean_manip']:.5f}", f"{r['max_qd']:.3f}",
                r["worst_joint"], f"{r['q1_max']:.3f}",
                f"{r['max_err'] * 1000:.3f}",
                f"{r['final_err'] * 1000:.3f}",
                f"{r['path_len'] * 100:.3f}", f"{r['min_z']:.4f}",
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
    panda, movable = build_scene()
    draw_static_paths(gui)

    print("[1/2] safe path ...")
    safe = run_path(panda, movable, "safe", SAFE_PATH[0], SAFE_PATH[1],
                    gui)
    print("[2/2] near-singular path ...")
    near = run_path(panda, movable, "near", NEAR_PATH[0], NEAR_PATH[1],
                    gui)

    print_summary(safe, near)
    png_path = os.path.join(OUT_DIR, "step4_singularity.png")
    csv_path = os.path.join(OUT_DIR, "step4_singularity.csv")
    make_plot(safe, near, png_path)
    save_csv(safe, near, csv_path)
    print("\nCSV :", csv_path)
    print("图片:", png_path)

    if gui:
        p.addUserDebugText(
            "safe: sigma_min %.3f, max qdot %.2f rad/s"
            % (safe["min_sigma"], safe["max_qd"]),
            [0.28, 0.34, 0.55], textColorRGB=RGB["safe"],
            textSize=1.0, lifeTime=0)
        p.addUserDebugText(
            "near: sigma_min %.3f, max qdot %.2f rad/s (%.2fx)"
            % (near["min_sigma"], near["max_qd"],
               near["max_qd"] / safe["max_qd"]),
            [0.28, 0.38, 0.51], textColorRGB=RGB["near"],
            textSize=1.0, lifeTime=0)
        print("\n动画结束，窗口保留 10 秒。"
              "蓝/红粗线是两条路径的实际末端轨迹；"
              "屏幕上实时显示 sigma_min 和 peak|qdot|。")
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
