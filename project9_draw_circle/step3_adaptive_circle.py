"""
第 6 天 · 一小时趣味实验 · 第 3 步
=====================================
让圆“自己看情况调速”。

思路：
  1. 先匀速画一圈，沿途记录 sigma_min；
  2. 在 sigma_min 低的位置把时间权重调高（走慢一点）；
  3. 在 sigma_min 高的位置把时间权重调低（走快一点）；
  4. 总时间仍然保持 8 s，比较：

       constant 匀速圆
       adaptive 按可操作度分配速度的圆

关注：
  - 弱扇区里关节速度是否下降；
  - 跟踪误差是否下降；
  - 总时间不变时，强扇区会不会变得更吃力。

运行：
  python project9_draw_circle/step3_adaptive_circle.py
"""

import csv
import math
import os

import numpy as np
import pybullet as p
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt

import step2_circle_timing as base


BASE_DIR = os.path.dirname(os.path.abspath(__file__))
OUT_DIR = os.path.join(BASE_DIR, "output")
DT = base.DT
T_TOTAL = base.T_TOTAL
STEPS = base.STEPS
CENTER = base.CENTER
RADIUS = base.RADIUS
START = base.START
EE_LINK = 11

COLORS = {"constant": "tab:blue", "adaptive": "tab:green"}


def jacobian_sigma(panda, movable):
    all_pos = [p.getJointState(panda, j)[0] for j in movable]
    zeros = [0.0] * len(all_pos)
    Jl, Ja = p.calculateJacobian(
        panda, EE_LINK, [0.0, 0.0, 0.0],
        all_pos, zeros, zeros,
    )
    Jv = np.array(Jl, dtype=float)[:, :7]
    Jw = np.array(Ja, dtype=float)[:, :7]
    J = np.vstack([Jv, Jw])
    return float(np.linalg.svd(J, compute_uv=False).min())


def run_schedule(panda, movable, q_start, theta_schedule):
    base.reset_to_start(panda, q_start, False)
    rows = []
    for theta in theta_schedule:
        target = CENTER + np.array([RADIUS * math.cos(theta),
                                    RADIUS * math.sin(theta), 0.0])
        q_cmd = base.solve_ik(panda, target)
        base.command_arm(panda, q_cmd)
        p.stepSimulation()
        actual = base.ee_pos(panda)
        q, qd = base.joint_state(panda)
        rows.append({
            "theta": float(theta),
            "target": target.copy(),
            "actual": actual.copy(),
            "err": actual - target,
            "q": q.copy(),
            "qd": qd.copy(),
            "sigma": jacobian_sigma(panda, movable),
        })
    base.hold_ik(panda, START, 240, False)
    final = base.ee_pos(panda)
    return rows, final


def analyse(rows, final, name, weak_lo, weak_hi):
    theta = np.array([r["theta"] for r in rows])
    target = np.array([r["target"] for r in rows])
    actual = np.array([r["actual"] for r in rows])
    err = np.array([r["err"] for r in rows])
    qd = np.array([r["qd"] for r in rows])
    sigma = np.array([r["sigma"] for r in rows])

    err_n = np.linalg.norm(err, axis=1)
    radial = (np.sqrt((actual[:, 0] - CENTER[0]) ** 2 +
                      (actual[:, 1] - CENTER[1]) ** 2) - RADIUS)
    qmax = np.max(np.abs(qd), axis=1)
    weak = (theta >= weak_lo) & (theta <= weak_hi)
    path_len = float(np.linalg.norm(np.diff(actual, axis=0), axis=1).sum())
    t = np.arange(len(rows)) * DT

    return {
        "name": name,
        "t": t,
        "theta": theta,
        "target": target,
        "actual": actual,
        "err": err_n,
        "radial": radial,
        "q": np.array([r["q"] for r in rows]),
        "qd": qd,
        "qmax": qmax,
        "sigma": sigma,
        "weak": weak,
        "max_err": float(err_n.max()),
        "mean_err": float(err_n.mean()),
        "max_radial": float(np.abs(radial).max()),
        "close_err": float(np.linalg.norm(final - START)),
        "max_qd": float(qmax.max()),
        "max_sigma_err": float(err_n[weak].max()) if weak.any() else float("nan"),
        "max_sigma_qd": float(qmax[weak].max()) if weak.any() else float("nan"),
        "mean_sigma": float(sigma[weak].mean()) if weak.any() else float("nan"),
        "path_len": path_len,
    }


def build_adaptive_schedule(sigma):
    """根据 sigma 给每个角度分配时间：sigma 小 -> 权重大 -> 走得慢。"""
    # 平滑一下，避免单个采样点的噪声把速度曲线弄得很尖
    kernel = np.ones(31) / 31.0
    smooth = np.convolve(sigma, kernel, mode="same")
    smooth = np.maximum(smooth, 0.03)
    weight = 1.0 / smooth
    cum = np.concatenate([[0.0], np.cumsum((weight[1:] + weight[:-1]) / 2)])
    cum = cum / cum[-1] * T_TOTAL
    theta_grid = np.linspace(0, 2 * math.pi, STEPS)
    t_grid = np.linspace(0, T_TOTAL, STEPS)
    theta_adapt = np.interp(t_grid, cum, theta_grid)
    theta_adapt[0] = 0.0
    theta_adapt[-1] = 2 * math.pi
    return theta_adapt, weight, theta_grid


def make_plot(const, adapt, theta_adapt, weak_lo, weak_hi, path):
    fig = plt.figure(figsize=(18, 10))
    fig.suptitle("Circle with manipulability-aware speed  |  "
                 "same 8 s, redistributed speed", fontsize=14)

    # (1) 用 sigma_min 给轨迹着色
    ax = fig.add_subplot(231)
    sc = ax.scatter(const["actual"][:, 0], const["actual"][:, 1],
                    c=const["sigma"], cmap="viridis", s=6)
    ax.plot(const["target"][:, 0], const["target"][:, 1], "--",
            color="0.5", lw=1.0)
    ax.set_aspect("equal", adjustable="box")
    ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]")
    ax.set_title("Constant-speed circle colored by sigma_min")
    plt.colorbar(sc, ax=ax, label="sigma_min")

    # (2) sigma_min 随角度
    ax = fig.add_subplot(232)
    ax.plot(np.degrees(const["theta"]), const["sigma"],
            color="tab:purple", lw=1.8)
    ax.axvspan(math.degrees(weak_lo), math.degrees(weak_hi),
               color="tab:red", alpha=0.12, label="weak sector")
    ax.set_xlabel("theta [deg]"); ax.set_ylabel("sigma_min")
    ax.set_title("Manipulability along the circle")
    ax.legend(fontsize=8); ax.grid(alpha=0.3)

    # (3) 角速度
    ax = fig.add_subplot(233)
    speed_const = np.gradient(const["theta"], DT)
    speed_adapt = np.gradient(theta_adapt) / DT
    ax.plot(const["t"], speed_const, color=COLORS["constant"],
            lw=1.8, label="constant")
    ax.plot(const["t"], speed_adapt, color=COLORS["adaptive"],
            lw=1.8, label="adaptive")
    ax.set_xlabel("time [s]"); ax.set_ylabel("angular speed [rad/s]")
    ax.set_title("Target angular speed")
    ax.legend(fontsize=8); ax.grid(alpha=0.3)

    # (4) 跟踪误差
    ax = fig.add_subplot(234)
    ax.plot(const["t"], const["err"] * 1000,
            color=COLORS["constant"], lw=1.5, label="constant")
    ax.plot(adapt["t"], adapt["err"] * 1000,
            color=COLORS["adaptive"], lw=1.5, label="adaptive")
    ax.axvspan(const["t"][const["weak"]][0], const["t"][const["weak"]][-1],
               color="tab:red", alpha=0.08)
    ax.set_xlabel("time [s]"); ax.set_ylabel("tracking error [mm]")
    ax.set_title("Tracking error")
    ax.legend(fontsize=8); ax.grid(alpha=0.3)

    # (5) 关节速度
    ax = fig.add_subplot(235)
    ax.plot(const["t"], const["qmax"], color=COLORS["constant"],
            lw=1.5, label="constant")
    ax.plot(adapt["t"], adapt["qmax"], color=COLORS["adaptive"],
            lw=1.5, label="adaptive")
    ax.set_xlabel("time [s]"); ax.set_ylabel("max |qdot| [rad/s]")
    ax.set_title("Actual joint speed")
    ax.legend(fontsize=8); ax.grid(alpha=0.3)

    # (6) 表格
    ax = fig.add_subplot(236)
    ax.axis("off")
    lines = [
        "Metrics", "=" * 70,
        f"{'':<16}{'constant':>12}{'adaptive':>12}",
        f"{'max err':<16}{const['max_err'] * 1000:10.1f}mm"
        f"{adapt['max_err'] * 1000:10.1f}mm",
        f"{'mean err':<16}{const['mean_err'] * 1000:10.1f}mm"
        f"{adapt['mean_err'] * 1000:10.1f}mm",
        f"{'max qdot':<16}{const['max_qd']:10.2f}"
        f"{adapt['max_qd']:10.2f}",
        f"{'weak max err':<16}{const['max_sigma_err'] * 1000:10.1f}mm"
        f"{adapt['max_sigma_err'] * 1000:10.1f}mm",
        f"{'weak max qdot':<16}{const['max_sigma_qd']:10.2f}"
        f"{adapt['max_sigma_qd']:10.2f}",
        f"{'closure':<16}{const['close_err'] * 1000:10.2f}mm"
        f"{adapt['close_err'] * 1000:10.2f}mm",
        f"{'path length':<16}{const['path_len'] * 100:10.1f}cm"
        f"{adapt['path_len'] * 100:10.1f}cm",
        "=" * 70,
        f"weak sector: {math.degrees(weak_lo):.0f}° ~ "
        f"{math.degrees(weak_hi):.0f}°",
        "adaptive: more time where sigma_min is low,",
        "less time where sigma_min is high.",
        "total time fixed at 8 s.",
    ]
    ax.text(0, 1, "\n".join(lines), va="top", family="monospace",
            fontsize=9)

    fig.tight_layout(rect=[0, 0, 1, 0.95])
    fig.savefig(path, dpi=150)
    plt.close(fig)


def print_summary(const, adapt):
    print("\n===== circle speed allocation =====")
    for r in (const, adapt):
        print("%-9s max err %5.1f mm | mean %4.1f mm | max qdot %.2f | "
              "weak err %5.1f mm | weak qdot %.2f"
              % (r["name"], r["max_err"] * 1000, r["mean_err"] * 1000,
                 r["max_qd"], r["max_sigma_err"] * 1000,
                 r["max_sigma_qd"]))


def save_csv(const, adapt, weak_lo, weak_hi, path):
    fields = ["timing", "weak_lo_deg", "weak_hi_deg", "max_err_mm",
              "mean_err_mm", "max_qdot", "weak_max_err_mm",
              "weak_max_qdot", "closure_mm", "path_len_cm"]
    with open(path, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(fields)
        for r in (const, adapt):
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
    base.connect(False)
    panda = base.build_scene()
    movable = [j for j in range(p.getNumJoints(panda))
               if p.getJointInfo(panda, j)[2] != p.JOINT_FIXED]

    print("warmup ...")
    q_start = base.warmup_to_start(panda, False)

    print("[1/2] constant-speed probe ...")
    theta_const = np.linspace(0, 2 * math.pi, STEPS)
    const_rows, const_final = run_schedule(
        panda, movable, q_start, theta_const)
    const_sigma = np.array([r["sigma"] for r in const_rows])

    # 以 sigma 最小的位置为中心，取 ±20° 作为弱扇区
    i_min = int(np.argmin(const_sigma))
    weak_center = theta_const[i_min]
    weak_lo = max(0.0, weak_center - math.radians(20))
    weak_hi = min(2 * math.pi, weak_center + math.radians(20))
    print("weak center theta = %.1f deg" % math.degrees(weak_center))

    theta_adapt, weight, theta_grid = build_adaptive_schedule(const_sigma)
    print("[2/2] adaptive-speed circle ...")
    adapt_rows, adapt_final = run_schedule(
        panda, movable, q_start, theta_adapt)

    const = analyse(const_rows, const_final, "constant",
                    weak_lo, weak_hi)
    adapt = analyse(adapt_rows, adapt_final, "adaptive",
                    weak_lo, weak_hi)

    print_summary(const, adapt)
    png_path = os.path.join(OUT_DIR, "circle_adaptive.png")
    csv_path = os.path.join(OUT_DIR, "circle_adaptive.csv")
    make_plot(const, adapt, theta_adapt, weak_lo, weak_hi, png_path)
    save_csv(const, adapt, weak_lo, weak_hi, csv_path)
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
