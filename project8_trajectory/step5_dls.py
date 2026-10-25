"""
第 5 天 · 轨迹可视化实验 · 第 5 步
=====================================
奇异位形处理：普通伪逆 vs 阻尼最小二乘（DLS）。

在 near-singular 路径上比较两种速度级控制器：

  standard: qdot = pinv(J) * twist
  DLS:      qdot = J^T * (J J^T + lambda^2 I)^(-1) * twist

两者使用：
  - 同一个起点/终点（near 路径）
  - 同一个 min-jerk 时间曲线
  - 同一个位置/姿态反馈增益
  - 同一组关节速度上限（来自 Panda URDF）

关注：
  - 普通伪逆在奇异附近会不会要求很大的关节速度？
  - DLS 如何压低关节速度？
  - 压低速度后，末端跟踪误差会增大多少？

运行：
  python project8_trajectory/step5_dls.py
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
HOLD_STEPS = 240
WARMUP_STEPS = 360
WARMUP_HOLD = 240
RESET_HOLD = 240

START = np.array([0.66, 0.0, 0.35])
END = np.array([0.80, 0.0, 0.35])
DIST = float(np.linalg.norm(END - START))
TARGET_ORN = p.getQuaternionFromEuler([0, math.pi, 0])

KP = 15.0       # 位置误差反馈增益
KO = 8.0        # 姿态误差反馈增益
LAMBDAS = [0.0, 0.02, 0.05, 0.10, 0.20, 0.30]

EE_LINK = 11
ARM_JOINTS = list(range(7))

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
OUT_DIR = os.path.join(BASE_DIR, "output")

COLORS = {"standard": "tab:blue", "dls": "tab:orange"}


# ============================================================
# 2. 仿真基础
# ============================================================
def connect(gui):
    if gui:
        p.connect(p.GUI)
        p.resetDebugVisualizerCamera(
            cameraDistance=1.4, cameraYaw=45, cameraPitch=-32,
            cameraTargetPosition=[0.68, 0.0, 0.30],
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
    vmax = np.array([p.getJointInfo(panda, j)[11] for j in ARM_JOINTS],
                    dtype=float)
    return panda, movable, vmax


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


def command_arm_velocity(panda, qdot):
    for j, v in zip(ARM_JOINTS, qdot):
        p.setJointMotorControl2(panda, j, p.VELOCITY_CONTROL,
                                targetVelocity=float(v), force=200)


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
# 3. 四元数姿态误差
# ============================================================
def quat_mul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return np.array([
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    ])


def quat_conj(q):
    return np.array([-q[0], -q[1], -q[2], q[3]])


def quat_error_vec(q_cur, q_des):
    """从当前姿态到目标姿态的旋转向量（axis * angle）。"""
    qe = quat_mul(q_des, quat_conj(q_cur))
    if qe[3] < 0:
        qe = -qe
    w = float(np.clip(qe[3], -1.0, 1.0))
    angle = 2.0 * math.acos(w)
    s = math.sqrt(max(1.0 - w * w, 0.0))
    if s < 1e-8:
        return np.zeros(3)
    return qe[:3] / s * angle


# ============================================================
# 4. 雅可比
# ============================================================
def full_jacobian(panda, movable):
    all_pos = [p.getJointState(panda, j)[0] for j in movable]
    zeros = [0.0] * len(all_pos)
    Jl, Ja = p.calculateJacobian(
        panda, EE_LINK, [0.0, 0.0, 0.0],
        all_pos, zeros, zeros,
    )
    Jv = np.array(Jl, dtype=float)[:, :7]
    Jw = np.array(Ja, dtype=float)[:, :7]
    J = np.vstack([Jv, Jw])
    sv = np.linalg.svd(J, compute_uv=False)
    return J, float(sv.min())


# ============================================================
# 4.5 GUI 辅助
# ============================================================
GUI_COLORS = {"standard": [0.10, 0.40, 1.00],
              "dls": [0.00, 0.70, 0.20]}


def draw_static_path(gui):
    if not gui:
        return
    for k in range(40):
        a = START + (END - START) * (k / 40.0)
        b = START + (END - START) * ((k + 1) / 40.0)
        p.addUserDebugLine(a.tolist(), b.tolist(), [0.55, 0.55, 0.55],
                           lineWidth=1.2, lifeTime=0)
    p.addUserDebugText("x=0.66", (START + [0, 0, 0.06]).tolist(),
                       textColorRGB=[0.3, 0.3, 0.3], textSize=1.0,
                       lifeTime=0)
    p.addUserDebugText("x=0.80", (END + [0, 0, 0.06]).tolist(),
                       textColorRGB=[0.3, 0.3, 0.3], textSize=1.0,
                       lifeTime=0)
    p.addUserDebugText("blue = standard pinv   green = DLS",
                       [0.40, 0.28, 0.62], textColorRGB=[0, 0, 0],
                       textSize=1.0, lifeTime=0)


def make_marker(color):
    vis = p.createVisualShape(p.GEOM_SPHERE, radius=0.010,
                              rgbaColor=[color[0], color[1], color[2], 1.0])
    return p.createMultiBody(baseMass=0, baseVisualShapeIndex=vis,
                             basePosition=START.tolist())


def update_marker(marker, pos):
    p.resetBasePositionAndOrientation(marker, pos.tolist(),
                                      [0, 0, 0, 1])


def draw_trail(prev, actual, color):
    if np.linalg.norm(actual - prev) > 1e-6:
        p.addUserDebugLine(prev.tolist(), actual.tolist(), color,
                           lineWidth=3.2, lifeTime=0)


def make_gauge(text, color):
    return p.addUserDebugText(text, [0.40, 0.32, 0.58],
                              textColorRGB=color, textSize=0.9,
                              lifeTime=0)


def replace_gauge(item_id, text, color):
    if item_id is not None:
        p.removeUserDebugItem(item_id)
    return make_gauge(text, color)


# ============================================================
# 5. 速度级控制器
# ============================================================
def minjerk(u):
    s = 10 * u ** 3 - 15 * u ** 4 + 6 * u ** 5
    sdot = 30 * u ** 2 - 60 * u ** 3 + 30 * u ** 4
    return s, sdot


def run_controller(panda, movable, vmax, mode, lam, q_start, gui=False):
    reset_to_start(panda, q_start, gui)
    rows = []
    d = END - START
    color = GUI_COLORS["standard" if mode == "standard" else "dls"]
    marker = make_marker(color) if gui else None
    gauge = make_gauge(
        f"{mode} lambda={lam:.2f}", color) if gui else None
    trail_prev = ee_pos(panda)
    draw_counter = 0
    peak_cmd = 0.0

    for phase, steps in (("motion", STEPS), ("hold", HOLD_STEPS)):
        for k in range(steps):
            if phase == "motion":
                u = k / (steps - 1)
                s, sdot = minjerk(u)
                p_des = START + s * d
                v_ff = d * (sdot / T_TOTAL)
            else:
                p_des = END.copy()
                v_ff = np.zeros(3)

            p_cur, quat_cur = ee_state(panda)
            q_cur, qd_cur = joint_state(panda)
            e_p = p_des - p_cur
            e_o = quat_error_vec(quat_cur, TARGET_ORN)
            twist = np.concatenate([
                v_ff + KP * e_p,
                KO * e_o,
            ])

            J, sigma_min = full_jacobian(panda, movable)
            if mode == "standard":
                qdot_raw = np.linalg.pinv(J) @ twist
            else:
                A = J @ J.T + (lam ** 2) * np.eye(6)
                qdot_raw = J.T @ np.linalg.solve(A, twist)

            qdot = np.clip(qdot_raw, -vmax, vmax)
            saturated = bool(np.any(np.abs(qdot_raw) > vmax))
            command_arm_velocity(panda, qdot)
            p.stepSimulation()
            if gui:
                time.sleep(DT)
                update_marker(marker, p_des)

            actual, quat = ee_state(panda)
            q_act, qd_act = joint_state(panda)
            peak_cmd = max(peak_cmd, float(np.linalg.norm(qdot)))
            rows.append({
                "phase": phase,
                "target": p_des.copy(),
                "actual": actual.copy(),
                "quat": quat.copy(),
                "q": q_act.copy(),
                "qd": qd_act.copy(),
                "qdot_raw": qdot_raw.copy(),
                "qdot_cmd": qdot.copy(),
                "sigma_min": sigma_min,
                "err_p": float(np.linalg.norm(e_p)),
                "err_o": float(np.linalg.norm(e_o)),
                "saturated": saturated,
            })

            if gui:
                draw_counter += 1
                if draw_counter % 4 == 0:
                    draw_trail(trail_prev, actual, color)
                    trail_prev = actual
                if draw_counter % 24 == 0:
                    gauge = replace_gauge(
                        gauge,
                        f"{mode} lam={lam:.2f}  sigma={sigma_min:.3f}  "
                        f"raw={np.linalg.norm(qdot_raw):.1f}  "
                        f"cmd={peak_cmd:.2f}  err={np.linalg.norm(e_p) * 1000:.0f}mm",
                        color)

    if gui:
        update_marker(marker, END)
        if gauge is not None:
            p.removeUserDebugItem(gauge)
        p.removeBody(marker)

    return analyse(rows, mode, lam)


# ============================================================
# 6. 指标
# ============================================================
def analyse(rows, mode, lam):
    motion = np.array([r["phase"] == "motion" for r in rows])
    t = np.arange(len(rows)) * DT
    target = np.array([r["target"] for r in rows])
    actual = np.array([r["actual"] for r in rows])
    qd = np.array([r["qd"] for r in rows])
    qdot_raw = np.array([r["qdot_raw"] for r in rows])
    qdot_cmd = np.array([r["qdot_cmd"] for r in rows])
    sigma = np.array([r["sigma_min"] for r in rows])
    err_p = np.array([r["err_p"] for r in rows])
    err_o = np.array([r["err_o"] for r in rows])
    sat = np.array([r["saturated"] for r in rows])

    # 只统计运动段
    idx = np.where(motion)[0]
    m = motion
    norm_raw = np.linalg.norm(qdot_raw, axis=1)
    norm_cmd = np.linalg.norm(qdot_cmd, axis=1)
    norm_qd = np.linalg.norm(qd, axis=1)

    return {
        "name": "standard" if mode == "standard" else f"DLS λ={lam:.2f}",
        "mode": mode,
        "lambda": lam,
        "t": t,
        "target": target,
        "actual": actual,
        "sigma": sigma,
        "err_p": err_p,
        "err_o": err_o,
        "norm_raw": norm_raw,
        "norm_cmd": norm_cmd,
        "norm_qd": norm_qd,
        "saturated": sat,
        "max_raw": float(norm_raw[m].max()),
        "max_cmd": float(norm_cmd[m].max()),
        "max_qd": float(norm_qd[m].max()),
        "sat_ratio": float(sat[m].mean()),
        "max_err_p": float(err_p[m].max()),
        "max_err_o": float(err_o[m].max()),
        "final_err_p": float(err_p[-1]),
        "final_err_o": float(err_o[-1]),
        "min_sigma": float(sigma[m].min()),
        "mean_sigma": float(sigma[m].mean()),
        "path_len": float(np.linalg.norm(
            np.diff(actual[m], axis=0), axis=1).sum()),
    }


# ============================================================
# 7. 画图 / 输出
# ============================================================
def make_plot(results, path):
    lams = [r["lambda"] for r in results]
    fig = plt.figure(figsize=(18, 10))
    fig.suptitle("Step 5  |  Standard pseudoinverse vs DLS damping sweep "
                 "(near-singular path, T=1.5 s)", fontsize=14)

    # (1) qdot trade-off
    ax = fig.add_subplot(231)
    ax.plot(lams, [r["max_raw"] for r in results], "o-",
            color="tab:blue", lw=1.8)
    ax.set_yscale("log")
    ax.set_xlabel("DLS lambda  (0 = standard pinv)")
    ax.set_ylabel("max ||qdot|| raw [rad/s]")
    ax.set_title("Joint-speed demand vs damping")
    ax.grid(alpha=0.3, which="both")

    # (2) position error trade-off
    ax = fig.add_subplot(232)
    ax.plot(lams, [r["max_err_p"] * 1000 for r in results], "o-",
            color="tab:orange", lw=1.8)
    ax.set_xlabel("DLS lambda")
    ax.set_ylabel("max position error [mm]")
    ax.set_title("Tracking error vs damping")
    ax.grid(alpha=0.3)

    # (3) orientation error trade-off
    ax = fig.add_subplot(233)
    ax.plot(lams, [r["max_err_o"] * 180 / math.pi for r in results],
            "o-", color="tab:green", lw=1.8)
    ax.set_xlabel("DLS lambda")
    ax.set_ylabel("max orientation error [deg]")
    ax.set_title("Orientation error vs damping")
    ax.grid(alpha=0.3)

    # 选中三条曲线画时间序列
    selected = [results[0]]
    for r in results[1:]:
        if any(abs(r["lambda"] - x) < 1e-9 for x in (0.05, 0.20)):
            selected.append(r)
    colors = ["tab:blue", "tab:orange", "tab:green"]

    ax = fig.add_subplot(234)
    for r, c in zip(selected, colors):
        ax.plot(r["t"], np.maximum(r["norm_raw"], 1e-3),
                color=c, lw=1.8, label=r["name"])
    ax.set_yscale("log")
    ax.set_xlabel("time [s]"); ax.set_ylabel("||qdot_raw|| [rad/s]")
    ax.set_title("Raw joint-speed demand (log scale)")
    ax.legend(fontsize=7); ax.grid(alpha=0.3, which="both")

    ax = fig.add_subplot(235)
    for r, c in zip(selected, colors):
        ax.plot(r["t"], np.maximum(r["err_p"] * 1000, 1e-3),
                color=c, lw=1.8, label=r["name"])
    ax.set_yscale("log")
    ax.set_xlabel("time [s]"); ax.set_ylabel("position error [mm]")
    ax.set_title("Position error (log scale)")
    ax.legend(fontsize=7); ax.grid(alpha=0.3, which="both")

    # (6) table
    ax = fig.add_subplot(236)
    ax.axis("off")
    lines = ["Metrics", "=" * 92,
             f"{'lambda':>7}{'raw':>9}{'cmd':>9}{'sat%':>7}"
             f"{'pos err':>10}{'ori err':>10}{'final':>9}",
             f"{'':>7}{'rad/s':>9}{'rad/s':>9}{'':>7}"
             f"{'mm':>10}{'deg':>10}{'mm':>9}",
             "-" * 92]
    for r in results:
        lines.append(
            f"{r['lambda']:7.2f}{r['max_raw']:9.2f}{r['max_cmd']:9.2f}"
            f"{r['sat_ratio'] * 100:6.1f}%{r['max_err_p'] * 1000:10.1f}"
            f"{r['max_err_o'] * 180 / math.pi:10.2f}"
            f"{r['final_err_p'] * 1000:9.2f}"
        )
    lines += ["=" * 92,
              "lambda=0 is the standard pseudoinverse.",
              "DLS trades tracking accuracy for lower joint speed.",
              "near path: x .66 -> .80, 14 cm, T=1.5 s",
              "clipping uses Panda URDF joint speed limits."]
    ax.text(0, 1, "\n".join(lines), va="top", family="monospace",
            fontsize=9)

    fig.tight_layout(rect=[0, 0, 1, 0.95])
    fig.savefig(path, dpi=150)
    plt.close(fig)


def print_summary(results):
    print("\n===== standard vs DLS sweep =====")
    for r in results:
        print("%-14s max raw %8.2f rad/s | cmd %6.2f | sat %5.1f%% | "
              "pos err %7.1f mm | ori err %5.2f deg | final %6.2f mm"
              % (r["name"], r["max_raw"], r["max_cmd"],
                 r["sat_ratio"] * 100, r["max_err_p"] * 1000,
                 r["max_err_o"] * 180 / math.pi,
                 r["final_err_p"] * 1000))


def save_csv(results, path):
    fields = ["controller", "lambda", "max_raw", "max_cmd", "max_actual",
              "sat_ratio", "max_pos_err_mm", "max_ori_err_deg",
              "final_pos_err_mm", "final_ori_err_deg", "min_sigma",
              "mean_sigma", "path_len_cm"]
    with open(path, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(fields)
        for r in results:
            w.writerow([
                r["name"], f"{r['lambda']:.3f}", f"{r['max_raw']:.3f}",
                f"{r['max_cmd']:.3f}", f"{r['max_qd']:.3f}",
                f"{r['sat_ratio']:.4f}", f"{r['max_err_p'] * 1000:.3f}",
                f"{r['max_err_o'] * 180 / math.pi:.3f}",
                f"{r['final_err_p'] * 1000:.3f}",
                f"{r['final_err_o'] * 180 / math.pi:.3f}",
                f"{r['min_sigma']:.4f}", f"{r['mean_sigma']:.4f}",
                f"{r['path_len'] * 100:.3f}",
            ])


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--gui", action="store_true",
                        help="打开 PyBullet 窗口演示（默认）")
    parser.add_argument("--headless", action="store_true",
                        help="不打开窗口，跑完整 lambda 扫描并出图")
    args = parser.parse_args()
    if args.gui and args.headless:
        parser.error("--gui 和 --headless 只能选一个")
    gui = not args.headless

    os.makedirs(OUT_DIR, exist_ok=True)
    connect(gui)
    panda, movable, vmax = build_scene()
    draw_static_path(gui)
    print("joint speed limits:", np.round(vmax, 3))

    print("warmup ...")
    q_start = warmup_to_start(panda, gui)
    print("q_start:", np.round(q_start, 3))

    if gui:
        print("\n[1/2] standard pseudoinverse ...")
        std = run_controller(panda, movable, vmax, "standard", 0.0,
                             q_start, gui=True)
        print("[2/2] DLS lambda=0.05 ...")
        dls = run_controller(panda, movable, vmax, "dls", 0.05,
                             q_start, gui=True)
        print_summary([std, dls])

        p.addUserDebugText(
            "standard: raw %.0f rad/s, err %.0f mm"
            % (std["max_raw"], std["max_err_p"] * 1000),
            [0.40, 0.36, 0.54], textColorRGB=GUI_COLORS["standard"],
            textSize=0.9, lifeTime=0)
        p.addUserDebugText(
            "DLS 0.05: raw %.2f rad/s, err %.0f mm"
            % (dls["max_raw"], dls["max_err_p"] * 1000),
            [0.40, 0.40, 0.50], textColorRGB=GUI_COLORS["dls"],
            textSize=0.9, lifeTime=0)
        print("\n动画结束，窗口保留 10 秒。"
              "蓝色=standard，绿色=DLS；屏幕上会显示 raw qdot 和误差。")
        time.sleep(10.0)
        p.disconnect()
        return

    results = []
    for lam in LAMBDAS:
        mode = "standard" if lam == 0.0 else "dls"
        print("\n[lambda %.2f] %s ..." % (lam, mode))
        results.append(run_controller(
            panda, movable, vmax, mode, lam, q_start, gui=False))

    print_summary(results)
    png_path = os.path.join(OUT_DIR, "step5_dls.png")
    csv_path = os.path.join(OUT_DIR, "step5_dls.csv")
    make_plot(results, png_path)
    save_csv(results, csv_path)
    print("\nCSV :", csv_path)
    print("图片:", png_path)
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
