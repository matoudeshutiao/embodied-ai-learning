"""
第 5 天 · 轨迹可视化实验 · 第 2.5 步
=======================================
给轨迹加“可执行性门槛”。

前面我们只检查了关节速度，发现 constant 虽然速度没超限，
但加速度/jerk 是巨大尖峰。这一步同时检查：

    关节速度  |qdot| / v_limit
    末端加速度 |a|   / a_limit
    末端 jerk  |j|   / j_limit

任何一项超过 1，就判定这条轨迹不可执行。

重要：下面的 A_MAX / J_MAX 是教学用的示例阈值，不是 Franka 官方参数。
真机要根据厂家数据、负载、任务精度和安全要求来设定。
可以用命令行参数改，例如：
    python project8_trajectory/step2_5_limits.py --a-max 0.8 --j-max 5.0

输入是 step2_profiles.py 已经算好的 step2_summary.csv，
不重新跑仿真。
"""

import argparse
import csv
import os

import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from matplotlib.patches import Rectangle


BASE_DIR = os.path.dirname(os.path.abspath(__file__))
OUT_DIR = os.path.join(BASE_DIR, "output")
SUMMARY = os.path.join(OUT_DIR, "step2_summary.csv")


def profile_color(label):
    if label.startswith("constant"):
        return "tab:blue"
    if label.startswith("trapezoid"):
        return "tab:orange"
    return "tab:green"


def load_rows():
    rows = []
    with open(SUMMARY, "r", encoding="utf-8") as f:
        for r in csv.DictReader(f):
            r["duration"] = float(r["duration"])
            r["peak_speed"] = float(r["peak_speed"])
            r["max_acc"] = float(r["max_acc"])
            r["max_jerk"] = float(r["max_jerk"])
            r["max_err"] = float(r["max_err"])
            r["limit_ratio"] = float(r["limit_ratio"])
            r["cmd_limit_ratio"] = float(r["cmd_limit_ratio"])
            r["speed_ratio"] = max(r["limit_ratio"], r["cmd_limit_ratio"])
            rows.append(r)
    return rows


def classify(rows, a_max, j_max):
    for r in rows:
        r["acc_ratio"] = r["max_acc"] / a_max
        r["jerk_ratio"] = r["max_jerk"] / j_max
        r["utilization"] = max(r["speed_ratio"], r["acc_ratio"],
                               r["jerk_ratio"])
        fails = []
        if r["speed_ratio"] > 1.0:
            fails.append("joint speed")
        if r["acc_ratio"] > 1.0:
            fails.append("acceleration")
        if r["jerk_ratio"] > 1.0:
            fails.append("jerk")
        r["fails"] = fails
        r["feasible"] = len(fails) == 0
    return rows


def print_table(rows, a_max, j_max):
    print("\nLimits (example): a_max=%.2f m/s^2, jerk_max=%.2f m/s^3"
          % (a_max, j_max))
    print("-" * 122)
    print("%-10s %-25s %6s %8s %9s %9s %8s %8s  %s"
          % ("exp", "profile", "T[s]", "vmax", "a_ratio", "j_ratio",
             "qd_ratio", "util", "result"))
    print("-" * 122)
    for r in rows:
        result = "PASS" if r["feasible"] else "REJECT: " + ", ".join(r["fails"])
        print("%-10s %-25s %6.2f %8.3f %9.2f %9.2f %8.2f %8.2f  %s"
              % (r["experiment"], r["profile"], r["duration"],
                 r["peak_speed"], r["acc_ratio"], r["jerk_ratio"],
                 r["speed_ratio"], r["utilization"], result))
    print("-" * 122)


def make_plot(rows, a_max, j_max, path):
    fig = plt.figure(figsize=(16, 7))
    fig.suptitle("Step 2.5  |  Trajectory feasibility gate "
                 "(speed + acceleration + jerk)", fontsize=14)

    # (1) 加速度-jerk 平面：左下角是可行域
    ax = fig.add_subplot(121)
    ax.set_xscale("log")
    ax.set_yscale("log")
    ax.add_patch(Rectangle((1e-2, 1e-1), a_max - 1e-2, j_max - 1e-1,
                           color="tab:green", alpha=0.08))
    ax.axvline(a_max, color="tab:green", ls="--", lw=1.5)
    ax.axhline(j_max, color="tab:green", ls="--", lw=1.5)
    marker = {"A_same_T": "o", "B_same_vmax": "s"}
    for r in rows:
        ax.scatter(r["max_acc"], r["max_jerk"],
                   color=profile_color(r["profile"]),
                   marker=marker.get(r["experiment"], "x"),
                   s=90, edgecolors="black", linewidths=0.7)
    ax.set_xlim(1e-2, 1e2)
    ax.set_ylim(1e-1, 1e5)
    ax.set_xlabel("peak commanded acceleration [m/s²]")
    ax.set_ylabel("peak commanded jerk [m/s³]")
    ax.set_title("Feasible region: below/left of green lines")
    ax.grid(alpha=0.3, which="both")
    handles = [
        Line2D([0], [0], color="tab:blue", marker="o", ls="", label="constant"),
        Line2D([0], [0], color="tab:orange", marker="o", ls="",
               label="trapezoid"),
        Line2D([0], [0], color="tab:green", marker="o", ls="",
               label="min-jerk"),
        Line2D([0], [0], color="black", marker="o", ls="",
               label="A: same T"),
        Line2D([0], [0], color="black", marker="s", ls="",
               label="B: same peak v"),
        Line2D([0], [0], color="tab:green", ls="--", lw=1.5,
               label=f"limits: a={a_max:g}, j={j_max:g}"),
    ]
    ax.legend(handles=handles, fontsize=7, loc="lower right",
              framealpha=0.9)

    # (2) 各项限制利用率；超过 1 就是不可执行
    ax = fig.add_subplot(122)
    x = np.arange(len(rows))
    width = 0.26
    ax.bar(x - width, [r["speed_ratio"] for r in rows], width,
           label="joint speed", color="tab:blue")
    ax.bar(x, [r["acc_ratio"] for r in rows], width,
           label="acceleration", color="tab:orange")
    ax.bar(x + width, [r["jerk_ratio"] for r in rows], width,
           label="jerk", color="tab:green")
    ax.axhline(1.0, color="red", ls="--", lw=1.5, label="limit")
    ax.set_yscale("log")
    ax.set_xticks(x)
    ax.set_xticklabels([f"{r['experiment']}\n{r['profile']}" for r in rows],
                       rotation=35, ha="right", fontsize=7)
    ax.set_ylabel("measured / limit  (>1 = rejected)")
    ax.set_title("Which limit is violated?")
    ax.legend(fontsize=8)
    ax.grid(alpha=0.3, axis="y", which="both")

    fig.tight_layout(rect=[0, 0, 1, 0.94])
    fig.savefig(path, dpi=150)
    plt.close(fig)


def save_csv(rows, path):
    fields = ["experiment", "profile", "duration", "peak_speed",
              "max_acc", "max_jerk", "max_err", "speed_ratio",
              "acc_ratio", "jerk_ratio", "utilization", "feasible",
              "fails"]
    with open(path, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(fields)
        for r in rows:
            w.writerow([r["experiment"], r["profile"],
                        f"{r['duration']:.4f}", f"{r['peak_speed']:.5f}",
                        f"{r['max_acc']:.3f}", f"{r['max_jerk']:.2f}",
                        f"{r['max_err']:.5f}", f"{r['speed_ratio']:.3f}",
                        f"{r['acc_ratio']:.3f}", f"{r['jerk_ratio']:.3f}",
                        f"{r['utilization']:.3f}", r["feasible"],
                        ";".join(r["fails"])])


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--a-max", type=float, default=0.5,
                        help="示例末端加速度上限 [m/s^2]")
    parser.add_argument("--j-max", type=float, default=2.0,
                        help="示例末端 jerk 上限 [m/s^3]")
    args = parser.parse_args()

    os.makedirs(OUT_DIR, exist_ok=True)
    rows = classify(load_rows(), args.a_max, args.j_max)
    print_table(rows, args.a_max, args.j_max)

    csv_path = os.path.join(OUT_DIR, "step2_5_feasibility.csv")
    png_path = os.path.join(OUT_DIR, "step2_5_feasibility.png")
    save_csv(rows, csv_path)
    make_plot(rows, args.a_max, args.j_max, png_path)

    passed = [r for r in rows if r["feasible"]]
    print("\n通过: %d / %d" % (len(passed), len(rows)))
    for r in passed:
        print("  PASS  %s / %s  (T=%.2fs)" %
              (r["experiment"], r["profile"], r["duration"]))
    print("\nCSV :", csv_path)
    print("图片:", png_path)
    print("\n提醒：PASS 只代表通过了当前设定的三项限制；"
          "真机还要检查力矩、关节限位和碰撞。")


if __name__ == "__main__":
    main()
