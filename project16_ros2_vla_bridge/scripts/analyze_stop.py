#!/usr/bin/env python3
"""分析 stop_probe 的 CSV。

回答三个问题：
1. 从开始掉速到完全停住，用了多久？
2. 对应的减速度大约是多少？
3. 换成 0.25 / 0.5 / 1.0 / 2.0 rad/s² 的受控减速，分别需要多久、多远？
"""

import csv
import sys
from datetime import datetime
from pathlib import Path
from statistics import mean, median


PROJECT_DIR = Path(__file__).resolve().parents[1]
DEFAULT_OUTPUT = PROJECT_DIR / "output"


def latest_csv(output_dir):
    files = sorted(
        output_dir.glob("stop_probe_*.csv"),
        key=lambda p: p.stat().st_mtime,
    )
    return files[-1] if files else None


def load_rows(path):
    rows = []
    with path.open(newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for raw in reader:
            try:
                row = {key: float(value) for key, value in raw.items()}
            except (TypeError, ValueError):
                continue
            rows.append(row)

    if not rows:
        return rows

    # 兼容旧 CSV：没有 calc 字段时用位置差分补上。
    if "j1_vel_calc" not in rows[0]:
        add_calc_velocity(rows, "j1_pos", "j1_vel_calc")
    if "j2_vel_calc" not in rows[0]:
        add_calc_velocity(rows, "j2_pos", "j2_vel_calc")
    return rows


def add_calc_velocity(rows, pos_key, vel_key):
    n = len(rows)
    for i, row in enumerate(rows):
        lo = max(0, i - 3)
        hi = min(n - 1, i + 3)
        dt = rows[hi]["t"] - rows[lo]["t"]
        if dt <= 1e-9:
            row[vel_key] = 0.0
        else:
            row[vel_key] = (
                rows[hi][pos_key] - rows[lo][pos_key]
            ) / dt


def detect_stop(rows):
    times = [row["t"] for row in rows]
    speeds = [
        max(abs(row["j1_vel_calc"]), abs(row["j2_vel_calc"]))
        for row in rows
    ]

    # 用原始速度找最大落差，避免平滑把过渡时间拉长。
    index = max(
        range(1, len(speeds)),
        key=lambda i: speeds[i - 1] - speeds[i],
    )
    stop_t = times[index]

    pre_values = [
        value
        for t, value in zip(times, speeds)
        if stop_t - 0.25 <= t < stop_t - 0.02
    ]
    post_values = [
        value
        for t, value in zip(times, speeds)
        if stop_t + 0.02 < t <= stop_t + 0.25
    ]
    pre_speed = mean(pre_values) if pre_values else speeds[index - 1]
    post_speed = mean(post_values) if post_values else speeds[index]

    threshold = max(pre_speed * 0.1, 1e-6)
    high_times = [
        t
        for t, value in zip(times, speeds)
        if t < stop_t and value > threshold
    ]
    low_times = [
        t
        for t, value in zip(times, speeds)
        if t >= stop_t and value <= threshold
    ]

    if high_times and low_times:
        transition = max(low_times[0] - high_times[-1], sample_dt(times))
    else:
        transition = sample_dt(times)

    deceleration = pre_speed / transition if transition > 0 else 0.0
    return {
        "stop_t": stop_t,
        "pre_speed": pre_speed,
        "post_speed": post_speed,
        "transition": transition,
        "deceleration": deceleration,
        "index": index,
    }


def sample_dt(times):
    diffs = [
        times[i + 1] - times[i]
        for i in range(len(times) - 1)
        if times[i + 1] > times[i]
    ]
    return median(diffs) if diffs else 0.01


def value_at(rows, key, target_t):
    closest = min(rows, key=lambda row: abs(row["t"] - target_t))
    return closest[key]


def build_ideal_table(v0):
    result = []
    for accel in (0.25, 0.5, 1.0, 2.0):
        duration = v0 / accel
        distance = v0 * duration / 2.0
        result.append((accel, duration, distance))
    return result


def svg_grid(left, top, width, height, x_max, y_lo, y_hi):
    parts = []
    for i in range(5):
        value = y_lo + (y_hi - y_lo) * i / 4
        y = top + (y_hi - value) / (y_hi - y_lo) * height
        parts.append(
            f'<line x1="{left}" y1="{y:.2f}" '
            f'x2="{left + width}" y2="{y:.2f}" '
            f'stroke="#e5e7eb"/>'
        )
        parts.append(
            f'<text x="{left - 10}" y="{y + 4:.2f}" '
            f'text-anchor="end" font-size="11" fill="#6b7280">'
            f"{value:.4f}</text>"
        )
    for i in range(6):
        value = x_max * i / 5
        x = left + value / x_max * width
        parts.append(
            f'<line x1="{x:.2f}" y1="{top}" '
            f'x2="{x:.2f}" y2="{top + height}" '
            f'stroke="#f3f4f6"/>'
        )
        parts.append(
            f'<text x="{x:.2f}" y="{top + height + 20}" '
            f'text-anchor="middle" font-size="11" fill="#6b7280">'
            f"{value:.3f}</text>"
        )
    parts.append(
        f'<rect x="{left}" y="{top}" width="{width}" height="{height}" '
        f'fill="none" stroke="#9ca3af"/>'
    )
    return parts


def svg_polyline(rows, key, t0, t1, left, top, width, height, x_max, y_lo, y_hi):
    points = []
    for row in rows:
        t = row["t"]
        if t < t0 or t > t1:
            continue
        x = left + (t - t0) / (t1 - t0) * width
        y = top + (y_hi - row[key]) / (y_hi - y_lo) * height
        points.append(f"{x:.2f},{y:.2f}")
    return " ".join(points)


def write_svg(path, rows, analysis, ideal_table):
    width = 960
    height = 700
    left = 90
    right = 35
    top = 80
    panel_h = 230
    gap = 85

    stop_t = analysis["stop_t"]
    pre_speed = analysis["pre_speed"]
    transition = analysis["transition"]

    t0 = max(0.0, stop_t - 0.35)
    t1 = stop_t + 0.35
    window = [
        row for row in rows if t0 <= row["t"] <= t1
    ]
    if len(window) < 3:
        window = rows

    values = []
    for row in window:
        values.extend([row["j1_vel_calc"], row["j2_vel_calc"]])
    lo = min(values + [0.0])
    hi = max(values + [0.0])
    pad = max((hi - lo) * 0.12, 0.005)
    lo -= pad
    hi += pad

    svg = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" '
        f'viewBox="0 0 {width} {height}">',
        '<rect width="100%" height="100%" fill="#ffffff"/>',
        f'<text x="{left}" y="30" font-size="20" fill="#111827" '
        f'font-family="sans-serif">Measured stop vs ideal deceleration</text>',
        f'<text x="{left}" y="50" font-size="12" fill="#6b7280">'
        f'measured transition={transition * 1000:.1f} ms, '
        f'deceleration={analysis["deceleration"]:.2f} rad/s^2</text>',
    ]

    # 上图：停止点附近的实测速度。
    svg.append(
        f'<text x="{left}" y="{top - 12}" font-size="15" fill="#111827">'
        f'measured joint velocity (rad/s), time relative to window</text>'
    )
    svg.extend(svg_grid(left, top, width - left - right, panel_h, t1 - t0, lo, hi))
    svg.append(
        f'<polyline points="{svg_polyline(rows, "j1_vel_calc", t0, t1, left, top, width - left - right, panel_h, t1 - t0, lo, hi)}" '
        f'fill="none" stroke="#2563eb" stroke-width="2"/>'
    )
    svg.append(
        f'<polyline points="{svg_polyline(rows, "j2_vel_calc", t0, t1, left, top, width - left - right, panel_h, t1 - t0, lo, hi)}" '
        f'fill="none" stroke="#dc2626" stroke-width="2"/>'
    )

    stop_x = left + (stop_t - t0) / (t1 - t0) * (width - left - right)
    svg.append(
        f'<line x1="{stop_x:.2f}" y1="{top}" x2="{stop_x:.2f}" '
        f'y2="{top + panel_h}" stroke="#111827" '
        f'stroke-width="2" stroke-dasharray="6 5"/>'
    )
    svg.append(
        f'<text x="{stop_x + 5:.2f}" y="{top + 15}" font-size="12" '
        f'fill="#111827">stop</text>'
    )

    # 下图：同样初速度下，不同减速度的停止曲线。
    ideal_top = top + panel_h + gap
    max_duration = max(item[1] for item in ideal_table) * 1.1
    svg.append(
        f'<text x="{left}" y="{ideal_top - 12}" font-size="15" fill="#111827">'
        f'ideal velocity profiles for different decelerations (rad/s)</text>'
    )
    svg.extend(
        svg_grid(
            left,
            ideal_top,
            width - left - right,
            panel_h,
            max_duration,
            0.0,
            pre_speed * 1.12,
        )
    )

    colors = ["#059669", "#0891b2", "#7c3aed", "#ea580c"]
    for (accel, duration, _), color in zip(ideal_table, colors):
        x_end = left + duration / max_duration * (width - left - right)
        # 速度从左上角的 v0 开始，向右下角降到 0。
        y_start = ideal_top
        y_end = ideal_top + panel_h
        svg.append(
            f'<line x1="{left}" y1="{y_start:.2f}" x2="{x_end:.2f}" '
            f'y2="{y_end:.2f}" stroke="{color}" stroke-width="2"/>'
        )
        svg.append(
            f'<text x="{x_end - 4:.2f}" y="{y_end - 6:.2f}" '
            f'text-anchor="end" font-size="11" fill="{color}">'
            f"a={accel:g}</text>"
        )

    # 实测急停：在 transition 时间内从 pre_speed 掉到 0。
    x_stop = left + transition / max_duration * (width - left - right)
    y_top = ideal_top
    y_bottom = ideal_top + panel_h
    svg.append(
        f'<polyline points="{left},{y_top:.2f} {x_stop:.2f},{y_top:.2f} '
        f'{x_stop:.2f},{y_bottom:.2f}" fill="none" '
        f'stroke="#111827" stroke-width="3" stroke-dasharray="5 4"/>'
    )
    svg.append(
        f'<text x="{left + 5}" y="{y_top + 18}" font-size="12" '
        f'fill="#111827">measured e-stop</text>'
    )

    svg.append("</svg>")
    path.write_text("\n".join(svg), encoding="utf-8")


def main():
    if len(sys.argv) > 1:
        csv_path = Path(sys.argv[1])
    else:
        csv_path = latest_csv(DEFAULT_OUTPUT)

    if csv_path is None or not csv_path.exists():
        print("ERROR: no stop_probe_*.csv found")
        print(f"expected under: {DEFAULT_OUTPUT}")
        return 1

    rows = load_rows(csv_path)
    if len(rows) < 5:
        print(f"ERROR: not enough rows in {csv_path}")
        return 1

    analysis = detect_stop(rows)
    ideal = build_ideal_table(analysis["pre_speed"])

    stop_t = analysis["stop_t"]
    stop_distance_1 = abs(
        value_at(rows, "j1_pos", stop_t + 0.2)
        - value_at(rows, "j1_pos", stop_t)
    )
    stop_distance_2 = abs(
        value_at(rows, "j2_pos", stop_t + 0.2)
        - value_at(rows, "j2_pos", stop_t)
    )

    print("")
    print("=== stop analysis ===")
    print(f"csv            : {csv_path}")
    print(f"samples        : {len(rows)}")
    print(f"stop time      : {stop_t:.3f} s")
    print(f"v before       : {analysis['pre_speed']:.4f} rad/s")
    print(f"transition     : {analysis['transition'] * 1000:.1f} ms")
    print(f"deceleration   : {analysis['deceleration']:.2f} rad/s^2")
    print(f"stop distance  : joint1={stop_distance_1:.5f} rad, "
          f"joint2={stop_distance_2:.5f} rad")
    print("")
    print("ideal controlled stop:")
    for accel, duration, distance in ideal:
        print(
            f"  a={accel:4.2f} rad/s^2 -> "
            f"T={duration * 1000:6.1f} ms, distance={distance:.5f} rad"
        )
    print("=====================")
    print("")

    output_dir = csv_path.parent
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    svg_path = output_dir / f"stop_analysis_{stamp}.svg"
    write_svg(svg_path, rows, analysis, ideal)
    print(f"SVG: {svg_path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
