"""
第 5 天 · 奇异位形/雅可比/可操作度的概念可视化
=================================================
用一个最简单的平面二连杆机械臂，把三个词画出来：

  - 雅可比 J：把“关节速度”翻译成“末端速度”的矩阵
  - 奇异位形：J 的某个奇异值接近 0，某个方向几乎动不了
  - 可操作度：给关节单位速度时，末端能到达的速度椭球

左图：弯曲构型，速度椭球比较大、比较圆
右图：接近伸直的构型，速度椭球被压扁，弱方向几乎为零

这不是 Panda 的精确模型，只是帮助理解概念的示意图。
真实的 Panda 六维雅可比结果见 step4_singularity.png。

运行：
  python project8_trajectory/step4_ellipsoid_concept.py
"""

import os
import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Circle

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
OUT_DIR = os.path.join(BASE_DIR, "output")
L1, L2 = 1.0, 1.0


def fk(th1, th2):
    x = L1 * np.cos(th1) + L2 * np.cos(th1 + th2)
    y = L1 * np.sin(th1) + L2 * np.sin(th1 + th2)
    elbow = np.array([L1 * np.cos(th1), L1 * np.sin(th1)])
    end = np.array([x, y])
    return elbow, end


def jacobian(th1, th2):
    s1, c1 = np.sin(th1), np.cos(th1)
    s12, c12 = np.sin(th1 + th2), np.cos(th1 + th2)
    return np.array([
        [-L1 * s1 - L2 * s12, -L2 * s12],
        [L1 * c1 + L2 * c12, L2 * c12],
    ])


def draw_arm(ax, th1, th2, color):
    elbow, end = fk(th1, th2)
    ax.plot([0, elbow[0]], [0, elbow[1]], '-o', color=color, lw=7,
            ms=9, solid_capstyle="round")
    ax.plot([elbow[0], end[0]], [elbow[1], end[1]], '-o',
            color=color, lw=7, ms=9, solid_capstyle="round")
    ax.scatter([0], [0], s=90, color="black", zorder=5)
    return end


def draw_velocity_ellipse(ax, J, center, color, scale=0.35):
    """给关节单位速度 (cos t, sin t)，画出末端速度 J·qdot 的轨迹。"""
    t = np.linspace(0, 2 * np.pi, 300)
    qdot = np.vstack([np.cos(t), np.sin(t)])
    v = J @ qdot
    pts = center[:, None] + scale * v
    ax.fill(pts[0], pts[1], color=color, alpha=0.18, zorder=2)
    ax.plot(pts[0], pts[1], color=color, lw=2.2, zorder=3)

    U, S, _ = np.linalg.svd(J)
    for i in range(2):
        axis = scale * S[i] * U[:, i]
        ax.arrow(center[0], center[1], axis[0], axis[1],
                 color=color, width=0.012,
                 head_width=0.06, length_includes_head=True, zorder=4)
        label = r"$\sigma_%d$=%.2f" % (i + 1, S[i])
        ax.text(center[0] + axis[0] * 1.12,
                center[1] + axis[1] * 1.12,
                label, color=color, fontsize=9)
    return S


def panel(ax, th1_deg, th2_deg, title, color):
    th1 = np.deg2rad(th1_deg)
    th2 = np.deg2rad(th2_deg)
    end = draw_arm(ax, th1, th2, color)
    J = jacobian(th1, th2)
    S = draw_velocity_ellipse(ax, J, end, color)
    ax.set_title(title, fontsize=12)
    ax.set_xlim(-1.2, 2.9)
    ax.set_ylim(-1.2, 2.4)
    ax.set_aspect("equal", adjustable="box")
    ax.grid(alpha=0.25)
    ax.set_xlabel("x")
    ax.set_ylabel("y")
    ax.text(-1.05, 2.05,
            "σ = [%.2f, %.2f]\narea = σ1·σ2 = %.3f"
            % (S[0], S[1], S[0] * S[1]),
            fontsize=9, va="top",
            bbox=dict(boxstyle="round", fc="white", ec="0.7", alpha=0.9))
    ax.text(end[0] + 0.04, end[1] + 0.04, "end-effector", fontsize=8)


def main():
    os.makedirs(OUT_DIR, exist_ok=True)
    fig, axs = plt.subplots(1, 2, figsize=(14, 6.5))
    fig.suptitle("Jacobian and velocity ellipsoid: why a near-straight arm is weak",
                 fontsize=14)

    # 弯曲：两个奇异值都不小，椭球比较圆
    panel(axs[0], 45, 60, "bent configuration", "tab:blue")
    # 接近伸直：一个奇异值接近 0，椭球被压扁
    panel(axs[1], 45, 4, "near-straight configuration (near singular)",
          "tab:red")

    fig.text(0.5, 0.015,
             "For a unit joint-speed circle (||qdot|| <= 1), the reachable "
             "end-effector velocities form an ellipse.  "
             "Near a singularity one axis shrinks toward zero.",
             ha="center", fontsize=10)

    fig.tight_layout(rect=[0, 0.05, 1, 0.93])
    path = os.path.join(OUT_DIR, "step4_ellipsoid_concept.png")
    fig.savefig(path, dpi=150)
    plt.close(fig)
    print("图片:", path)


if __name__ == "__main__":
    main()
