"""
第 5 天 · 轨迹可视化实验 · 第 2 步
=====================================
时间参数化：同一条直线，不同的“什么时候走到哪”。

比较三种速度曲线：
  1. constant  匀速，起步/停止是瞬间完成（当前 step1 的做法）
  2. trapezoid 梯形速度：加速 → 匀速 → 减速，加速度有限
  3. minjerk   五次多项式最小 jerk：位置、速度、加速度都平滑

做两组实验：
  A. 同样的总时间 T=2.5 s
     → 看“平滑”是不是免费的：为了同样时间走完，平滑曲线要更高的峰值速度
  B. 同样的峰值速度 v=0.156 m/s
     → 看“平滑”要花多少时间：峰值速度相同，平滑曲线需要更长时间

所有轨迹都会检查关节速度是否超过 Panda URDF 的标称上限，
这是从“仿真能跑”走向“真机安全”必须过的一关。

用法：
  python project8_trajectory/step2_profiles.py
"""

import csv
import math
import os

import numpy as np
import pybullet as p
import pybullet_data
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt


# ============================================================
# 1. 参数（和 step1 保持一致，方便对比）
# ============================================================
DT = 1.0 / 240.0
START = np.array([0.40, -0.18, 0.35])
END = np.array([0.55, 0.18, 0.35])
DIST = float(np.linalg.norm(END - START))
TARGET_ORN = p.getQuaternionFromEuler([0, math.pi, 0])

EE_LINK = 11
ARM_JOINTS = list(range(7))
HOLD_STEPS = 240
WARMUP_STEPS = 360
WARMUP_HOLD = 240
RESET_HOLD = 240

SAME_TIME = 2.5          # 实验 A：同样的总时间
SAME_PEAK = 0.156        # 实验 B：同样的峰值速度（step1 的 600 步速度）
TRAP_ACCEL_TIME = 0.5    # 实验 B 中梯形的加速时长

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
OUT_DIR = os.path.join(BASE_DIR, "output")


# ============================================================
# 2. 三种归一化速度曲线
#    u ∈ [0, 1] 是归一化时间，s(u) 是归一化路程
#    ds/du、dds/du²、ddds/du³ 分别是归一化速度、加速度、jerk
# ============================================================
def profile_constant(u):
    """匀速：s=u。优点是最快；缺点是起停时速度阶跃。"""
    return u, 1.0, 0.0, 0.0


def profile_trapezoid(u, accel_frac=0.2):
    """梯形：前 accel_frac 加速，中间匀速，最后 accel_frac 减速。"""
    a = accel_frac
    vmax = 1.0 / (1.0 - a)       # 归一化峰值速度
    amax = vmax / a              # 归一化加速度大小

    if u < a:
        s = 0.5 * amax * u * u
        ds = amax * u
        dds = amax
        ddds = 0.0
    elif u < 1.0 - a:
        s = 0.5 * amax * a * a + vmax * (u - a)
        ds = vmax
        dds = 0.0
        ddds = 0.0
    else:
        r = 1.0 - u
        s = 1.0 - 0.5 * amax * r * r
        ds = amax * r
        dds = -amax
        ddds = 0.0
    return s, ds, dds, ddds


def profile_minjerk(u):
    """五次多项式最小 jerk：s=10u³-15u⁴+6u⁵。"""
    s = 10.0 * u ** 3 - 15.0 * u ** 4 + 6.0 * u ** 5
    ds = 30.0 * u ** 2 - 60.0 * u ** 3 + 30.0 * u ** 4
    dds = 60.0 * u - 180.0 * u ** 2 + 120.0 * u ** 3
    ddds = 60.0 - 360.0 * u + 360.0 * u ** 2
    return s, ds, dds, ddds


PROFILES = {
    "constant": (profile_constant, "constant"),
    "trapezoid": (lambda u: profile_trapezoid(u, 0.2), "trapezoid (20% accel)"),
    "minjerk": (profile_minjerk, "min-jerk"),
}
COLORS = {"constant": "tab:blue", "trapezoid": "tab:orange",
          "minjerk": "tab:green"}


# ============================================================
# 3. 仿真与动作
# ============================================================
def connect():
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


def hold(panda, target, steps):
    for _ in range(steps):
        command_arm(panda, solve_ik(panda, target))
        p.stepSimulation()


def warmup_to_start(panda):
    """第一次从默认姿态走到起点，并停稳；返回起点的关节角。"""
    initial = ee_pos(panda)
    for k in range(WARMUP_STEPS):
        s = k / (WARMUP_STEPS - 1)
        target = initial + s * (START - initial)
        command_arm(panda, solve_ik(panda, target))
        p.stepSimulation()
    hold(panda, START, WARMUP_HOLD)
    q, _ = joint_state(panda)
    return q


def reset_to_start(panda, q_start):
    """把机械臂放回起点关节角，再保持一会儿让物理状态稳定。"""
    for j, q in enumerate(q_start):
        p.resetJointState(panda, j, float(q), 0.0)
    hold(panda, START, RESET_HOLD)


def run_profile(panda, name, duration, q_start=None):
    """
    跑一条按指定时间参数化的直线。
    duration 是总运动时间；q_start=None 表示第一次运行，需要预热。
    """
    if q_start is None:
        q_start = warmup_to_start(panda)
    else:
        reset_to_start(panda, q_start)

    profile_fn, label = PROFILES[name]
    steps = max(2, int(round(duration / DT)) + 1)
    t_actual = (steps - 1) * DT
    d = END - START

    rows = []
    for k in range(steps):
        u = k / (steps - 1)
        s, _, _, _ = profile_fn(u)
        target = START + s * d
        q_cmd = solve_ik(panda, target)
        command_arm(panda, q_cmd)
        p.stepSimulation()

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

    # 到终点后继续保持，观察收敛
    for _ in range(HOLD_STEPS):
        target = END
        q_cmd = solve_ik(panda, target)
        command_arm(panda, q_cmd)
        p.stepSimulation()
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

    return rows, t_actual, label, q_start


# ============================================================
# 4. 指标计算
# ============================================================
MAX_VEL = np.array([2.175, 2.175, 2.175, 2.175, 2.610, 2.610, 2.610])


def analyse(rows, name, t_actual, label):
    target = np.array([r["target"] for r in rows])
    actual = np.array([r["actual"] for r in rows])
    err = np.array([r["err"] for r in rows])
    qd = np.array([r["qd"] for r in rows])
    q_cmd = np.array([r["q_cmd"] for r in rows])
    phase = np.array([r["phase"] for r in rows])
    motion = phase == "motion"
    n_m = int(motion.sum())
    t_m = np.arange(n_m) * DT

    # 计算“指令”的速度/加速度/jerk。
    # 前面补一个运动前的 START，后面补一个运动后的 END，
    # 这样 constant 在起停处的速度阶跃才会被算出来，
    # 不会因为 np.gradient 的边界处理而把它误当成 0。
    tgt = target[motion]
    seq = np.vstack([START, tgt, END])
    seg = np.diff(seq, axis=0)
    speed = np.linalg.norm(seg, axis=1) / DT          # n+1 段
    t_speed = (np.arange(len(speed)) - 0.5) * DT
    accn = np.abs(np.diff(speed)) / DT                # n 个边界
    t_acc = np.arange(len(accn)) * DT
    jerkn = np.abs(np.diff(accn)) / DT                # n-1 个边界
    t_jerk = (np.arange(len(jerkn)) + 0.5) * DT

    err_m = err[motion]
    err_n = np.linalg.norm(err_m, axis=1)
    d = END - START
    unit = d / np.linalg.norm(d)
    along = err_m @ unit
    perp = np.linalg.norm(err_m - along[:, None] * unit[None, :], axis=1)

    hold = ~motion
    hold_idx = np.where(hold)[0]
    settle_idx = hold_idx[np.linalg.norm(err[hold], axis=1) < 0.002]
    settle_time = (float((settle_idx[0] - hold_idx[0] + 1) * DT)
                   if len(settle_idx) else float("nan"))

    qd_m = qd[motion]
    ratio = np.abs(qd_m) / MAX_VEL[None, :]
    j_ratio_flat = int(ratio.argmax())
    _, worst_joint_idx = np.unravel_index(j_ratio_flat, ratio.shape)
    worst_joint = int(worst_joint_idx)
    cmd_qd = np.gradient(q_cmd[motion], DT, axis=0)
    cmd_ratio = np.abs(cmd_qd) / MAX_VEL[None, :]
    cmd_ratio_max = float(cmd_ratio.max())
    actual_ratio_max = float(ratio.max())

    return {
        "name": name,
        "label": label,
        "duration": t_actual,
        "t": t_m,
        "t_speed": t_speed,
        "t_acc": t_acc,
        "t_jerk": t_jerk,
        "speed": speed,
        "acc": accn,
        "jerk": jerkn,
        "err": err_n,
        "max_err": float(err_n.max()),
        "rms_err": float(np.sqrt(np.mean(err_n ** 2))),
        "mean_along": float(along.mean()),
        "mean_perp": float(perp.mean()),
        "final_err": float(np.linalg.norm(err[-1])),
        "settle_time": settle_time,
        "peak_speed": float(speed.max()),
        "max_acc": float(accn.max()),
        "max_jerk": float(jerkn.max()),
        "max_qd": float(np.abs(qd_m).max()),
        "worst_qd_joint": int(worst_joint) + 1,
        "limit_ratio": actual_ratio_max,
        "cmd_limit_ratio": cmd_ratio_max,
        "speed_ok": bool(actual_ratio_max <= 1.0 and cmd_ratio_max <= 1.0),
        "q1_speed": qd_m[:, 0],
    }


# ============================================================
# 5. 画图
# ============================================================
def plot_same_duration(results, path):
    fig = plt.figure(figsize=(18, 10))
    fig.suptitle("Step 2A  |  Same task time T = 2.5 s: "
                 "smoother profiles need higher peak speed", fontsize=14)

    # (1) 指令速度
    ax = fig.add_subplot(231)
    for r in results:
        ax.plot(r["t_speed"], r["speed"], color=COLORS[r["name"]],
                lw=2, label=r["label"])
    ax.set_xlabel("time [s]"); ax.set_ylabel("speed [m/s]")
    ax.set_title("Commanded speed")
    ax.legend(fontsize=8)
    ax.grid(alpha=0.3)

    # (2) 指令加速度绝对值（对数坐标，才能同时看见常数曲线的尖峰）
    ax = fig.add_subplot(232)
    for r in results:
        ax.plot(r["t"], np.maximum(r["acc"], 1e-3),
                color=COLORS[r["name"]], lw=1.5, label=r["label"])
    ax.set_yscale("log")
    ax.set_xlabel("time [s]"); ax.set_ylabel("|acceleration| [m/s²]")
    ax.set_title("Commanded acceleration (log scale)")
    ax.legend(fontsize=8)
    ax.grid(alpha=0.3, which="both")

    # (3) 指令 jerk 绝对值
    ax = fig.add_subplot(233)
    for r in results:
        ax.plot(r["t_jerk"], np.maximum(r["jerk"], 1e-2),
                color=COLORS[r["name"]], lw=1.5, label=r["label"])
    ax.set_yscale("log")
    ax.set_xlabel("time [s]"); ax.set_ylabel("|jerk| [m/s³]")
    ax.set_title("Commanded jerk (log scale)")
    ax.legend(fontsize=8)
    ax.grid(alpha=0.3, which="both")

    # (4) 跟踪误差
    ax = fig.add_subplot(234)
    for r in results:
        ax.plot(r["t"], r["err"] * 1000, color=COLORS[r["name"]],
                lw=1.8, label=r["label"])
    ax.set_xlabel("time [s]"); ax.set_ylabel("|error| [mm]")
    ax.set_title("Tracking error")
    ax.legend(fontsize=8)
    ax.grid(alpha=0.3)

    # (5) q1 关节速度与标称上限
    ax = fig.add_subplot(235)
    for r in results:
        ax.plot(r["t"], np.abs(r["q1_speed"]), color=COLORS[r["name"]],
                lw=1.5, label=r["label"])
    ax.axhline(MAX_VEL[0], color="red", ls="--", lw=1.5,
               label="q1 URDF limit")
    ax.set_xlabel("time [s]"); ax.set_ylabel("|q1 speed| [rad/s]")
    ax.set_title("Joint 1 speed vs hardware limit")
    ax.legend(fontsize=7)
    ax.grid(alpha=0.3)

    # (6) 指标表
    ax = fig.add_subplot(236)
    ax.axis("off")
    lines = ["Metrics", "=" * 78]
    lines.append(f"{'profile':<22}{'T[s]':>6}{'vmax':>7}"
                 f"{'amax':>9}{'jmax':>10}{'err':>8}{'q1':>7}{'spd':>7}")
    for r in results:
        lines.append(
            f"{r['label']:<22}{r['duration']:6.2f}"
            f"{r['peak_speed']:7.3f}{r['max_acc']:9.1f}"
            f"{r['max_jerk']:10.0f}{r['max_err'] * 1000:7.1f}"
            f"{abs(r['q1_speed']).max():7.2f}"
            f"{'ok' if r['speed_ok'] else 'OVER':>7}"
        )
    lines.append("=" * 78)
    lines.append("vmax: peak commanded speed [m/s]")
    lines.append("amax: peak |commanded acceleration| [m/s^2]")
    lines.append("jmax: peak |commanded jerk| [m/s^3]")
    lines.append("err : max tracking error [mm]")
    lines.append("q1  : max |q1 speed| [rad/s], limit 2.175")
    lines.append("")
    lines.append("constant: instant start/stop")
    lines.append("trapezoid: finite accel, accel jumps at corners")
    lines.append("min-jerk: smooth accel, finite jerk")
    ax.text(0.0, 1.0, "\n".join(lines), va="top", family="monospace",
            fontsize=8.5)

    fig.tight_layout(rect=[0, 0, 1, 0.95])
    fig.savefig(path, dpi=150)
    plt.close(fig)


def plot_same_peak(results, path):
    fig, axs = plt.subplots(1, 2, figsize=(14, 5))
    fig.suptitle("Step 2B  |  Same peak speed 0.156 m/s: "
                 "smoothing costs time", fontsize=14)

    for r in results:
        axs[0].plot(r["t_speed"], r["speed"], color=COLORS[r["name"]],
                    lw=2, label=f"{r['label']}  T={r['duration']:.2f}s")
        axs[1].plot(r["t"], r["err"] * 1000, color=COLORS[r["name"]],
                    lw=2, label=r["label"])
    axs[0].set_xlabel("time [s]"); axs[0].set_ylabel("speed [m/s]")
    axs[0].set_title("Commanded speed")
    axs[0].legend(fontsize=8); axs[0].grid(alpha=0.3)

    axs[1].set_xlabel("time [s]"); axs[1].set_ylabel("|error| [mm]")
    axs[1].set_title("Tracking error")
    axs[1].legend(fontsize=8); axs[1].grid(alpha=0.3)

    fig.tight_layout(rect=[0, 0, 1, 0.93])
    fig.savefig(path, dpi=150)
    plt.close(fig)


def save_summary(all_results, path):
    fields = ["experiment", "profile", "duration", "peak_speed", "max_acc",
              "max_jerk", "max_err", "rms_err", "mean_along", "max_qd",
              "worst_qd_joint", "limit_ratio", "cmd_limit_ratio",
              "speed_ok"]
    with open(path, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(fields)
        for exp, r in all_results:
            w.writerow([exp, r["label"], f"{r['duration']:.4f}",
                        f"{r['peak_speed']:.5f}", f"{r['max_acc']:.3f}",
                        f"{r['max_jerk']:.2f}", f"{r['max_err']:.5f}",
                        f"{r['rms_err']:.5f}", f"{r['mean_along']:.5f}",
                        f"{r['max_qd']:.3f}", r["worst_qd_joint"],
                        f"{r['limit_ratio']:.3f}",
                        f"{r['cmd_limit_ratio']:.3f}", r["speed_ok"]])


# ============================================================
# 6. 主流程
# ============================================================
def print_result(tag, r):
    print("%-30s T=%.2fs  vmax=%.3f m/s  amax=%7.1f  jmax=%8.0f  "
          "err=%5.1fmm  q1=%4.2f  ratio=%4.2f  %s"
          % (tag, r["duration"], r["peak_speed"], r["max_acc"],
             r["max_jerk"], r["max_err"] * 1000,
             abs(r["q1_speed"]).max(), r["limit_ratio"],
             "SPEED OK" if r["speed_ok"] else "OVER SPEED LIMIT"))


def main():
    os.makedirs(OUT_DIR, exist_ok=True)
    connect()
    panda = build_scene()
    print("Panda loaded. q limits (maxVel):",
          [round(float(v), 3) for v in MAX_VEL])

    # 第一次：预热到 START，并保存起点关节角
    print("\n预热到起点 ...")
    q_start = warmup_to_start(panda)
    print("起点误差: %.3f mm" % (np.linalg.norm(ee_pos(panda) - START) * 1000))

    results_a, results_b = [], []
    for name in ("constant", "trapezoid", "minjerk"):
        print("\n[A] same time  T=%.2fs  profile=%s" % (SAME_TIME, name))
        rows, T, label, q_start = run_profile(
            panda, name, SAME_TIME, q_start=q_start)
        r = analyse(rows, name, T, label)
        results_a.append(r)
        print_result("[A] " + label, r)

    for name in ("constant", "trapezoid", "minjerk"):
        if name == "constant":
            T = DIST / SAME_PEAK
        elif name == "trapezoid":
            T = DIST / SAME_PEAK + TRAP_ACCEL_TIME
        else:
            T = 1.875 * DIST / SAME_PEAK  # 五次多项式峰值速度 = 1.875/T
        print("\n[B] same peak v=%.3f m/s  profile=%s  T=%.2fs"
              % (SAME_PEAK, name, T))
        # B 组梯形要保证加速时长 0.5s，所以重新按实际 T 计算 accel_frac
        if name == "trapezoid":
            PROFILES["trapezoid"] = (
                lambda u, a=TRAP_ACCEL_TIME / T: profile_trapezoid(u, a),
                "trapezoid (0.5s accel)")
        rows, T_actual, label, q_start = run_profile(
            panda, name, T, q_start=q_start)
        r = analyse(rows, name, T_actual, label)
        results_b.append(r)
        print_result("[B] " + label, r)

    plot_same_duration(results_a,
                       os.path.join(OUT_DIR, "step2_same_duration.png"))
    plot_same_peak(results_b,
                   os.path.join(OUT_DIR, "step2_same_peak.png"))
    save_summary([(f"A_same_T", r) for r in results_a]
                 + [(f"B_same_vmax", r) for r in results_b],
                 os.path.join(OUT_DIR, "step2_summary.csv"))

    print("\n输出目录:", OUT_DIR)
    print("  step2_same_duration.png")
    print("  step2_same_peak.png")
    print("  step2_summary.csv")

    p.disconnect()


if __name__ == "__main__":
    main()
