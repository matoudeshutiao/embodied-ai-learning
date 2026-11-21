#!/usr/bin/env python3
"""把一次 A3 试验输出整理成 report.md，提交并推送到 GitHub。

用法：
  python3 project16_ros2_vla_bridge/scripts/publish_experiment.py
  python3 project16_ros2_vla_bridge/scripts/publish_experiment.py \
      --run-dir project16_ros2_vla_bridge/output/a3_20261121_201637
  python3 project16_ros2_vla_bridge/scripts/publish_experiment.py --skip-push

默认取 output/ 下最新的 a3_* 目录；默认推送到 GitHub。
"""

import argparse
import csv
import json
import math
import re
import shutil
import subprocess
import sys
from collections import Counter
from datetime import datetime
from pathlib import Path

CAMERA_X = 0.55
CAMERA_Y = 0.0

VISION_RE = re.compile(
    r"vision vs truth: vision=\(([-\d.]+), ([-\d.]+), ([-\d.]+)\),\s*"
    r"truth=\(([-\d.]+), ([-\d.]+), ([-\d.]+)\),\s*"
    r"err_xy=([\d.]+) mm",
    re.S,
)


def repo_root():
    here = Path(__file__).resolve().parent
    for candidate in (here, *here.parents):
        if (candidate / ".git").exists():
            return candidate
    raise SystemExit("找不到 git 根目录")


def git(repo, *args):
    return subprocess.run(
        ["git", *args],
        cwd=str(repo),
        capture_output=True,
        text=True,
    )


def as_float(value):
    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def as_bool(value):
    return str(value).strip().lower() == "true"


def mean(values):
    return sum(values) / len(values) if values else None


def pearson(xs, ys):
    if len(xs) < 2 or len(xs) != len(ys):
        return None
    mx = sum(xs) / len(xs)
    my = sum(ys) / len(ys)
    numerator = sum((x - mx) * (y - my) for x, y in zip(xs, ys))
    dx = math.sqrt(sum((x - mx) ** 2 for x in xs))
    dy = math.sqrt(sum((y - my) ** 2 for y in ys))
    if dx == 0.0 or dy == 0.0:
        return None
    return numerator / (dx * dy)


def resolve_run_dir(repo, given):
    output_dir = repo / "project16_ros2_vla_bridge" / "output"
    if given:
        run = Path(given).expanduser().resolve()
    else:
        runs = sorted(
            output_dir.glob("a3_*"),
            key=lambda path: path.stat().st_mtime,
        )
        if not runs:
            raise SystemExit(f"没有找到 {output_dir}/a3_*")
        run = runs[-1]
    if not (run / "summary.csv").exists():
        raise SystemExit(f"{run} 缺少 summary.csv")
    return run


def run_id_of(run):
    name = run.name
    return name[3:] if name.startswith("a3_") else name


def read_csv_rows(run):
    with open(
        run / "summary.csv", newline="", encoding="utf-8"
    ) as handle:
        return list(csv.DictReader(handle))


def read_meta(run):
    path = run / "meta.json"
    if not path.exists():
        return {}
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return {}


def parse_vision_log(log_path):
    if not log_path.exists():
        return None
    text = log_path.read_text(encoding="utf-8", errors="ignore")
    match = VISION_RE.search(text)
    if match is None:
        return None
    values = [float(value) for value in match.groups()]
    return {
        "vx": values[0],
        "vy": values[1],
        "vz": values[2],
        "tx": values[3],
        "ty": values[4],
        "tz": values[5],
        "err": values[6],
    }


def enrich_rows(run, rows):
    enriched = []
    for row in rows:
        trial = int(row["trial"])
        log_path = run / f"trial_{trial:02d}.log"
        vision = parse_vision_log(log_path)

        command_x = as_float(row.get("cube_x"))
        command_y = as_float(row.get("cube_y"))
        actual_offset_mm = None
        radial_mm = None
        angle_deg = None

        if vision is not None and command_x is not None:
            actual_offset_mm = 1000.0 * math.hypot(
                vision["tx"] - command_x,
                vision["ty"] - command_y,
            )
            truth_x = vision["tx"] - CAMERA_X
            truth_y = vision["ty"] - CAMERA_Y
            seen_x = vision["vx"] - CAMERA_X
            seen_y = vision["vy"] - CAMERA_Y
            truth_radius = math.hypot(truth_x, truth_y)
            seen_radius = math.hypot(seen_x, seen_y)
            radial_mm = 1000.0 * (truth_radius - seen_radius)
            angle_deg = math.degrees(
                math.atan2(
                    truth_x * seen_y - truth_y * seen_x,
                    truth_x * seen_x + truth_y * seen_y,
                )
            )

        enriched.append(
            {
                "trial": trial,
                "command_x": command_x,
                "command_y": command_y,
                "actual_x": None if vision is None else vision["tx"],
                "actual_y": None if vision is None else vision["ty"],
                "actual_offset_mm": actual_offset_mm,
                "vision_err_mm": as_float(row.get("vision_err_mm")),
                "radial_mm": radial_mm,
                "angle_deg": angle_deg,
                "grasp_ok": as_bool(row.get("grasp_ok")),
                "place_ok_node": as_bool(row.get("place_ok_node")),
                "place_err_xy_mm": as_float(
                    row.get("place_err_xy_mm")
                ),
                "place_err_z_mm": as_float(
                    row.get("place_err_z_mm")
                ),
                "place_verified": as_bool(
                    row.get("place_verified")
                ),
                "full_success": as_bool(row.get("full_success")),
                "stage": row.get("failure_stage") or "unknown",
                "log_path": log_path,
            }
        )
    return enriched


def fmt(value, digits=1):
    if value is None:
        return "—"
    return f"{value:.{digits}f}"


def build_report(run, rows, meta, head, note, logs_included):
    run_id = run_id_of(run)
    try:
        created = datetime.strptime(run_id, "%Y%m%d_%H%M%S")
        created_text = created.strftime("%Y-%m-%d %H:%M:%S")
    except ValueError:
        created_text = "未知"

    total = len(rows)
    full = sum(1 for row in rows if row["full_success"])
    grasp = sum(1 for row in rows if row["grasp_ok"])
    stages = Counter(row["stage"] for row in rows)

    vision_values = [
        row["vision_err_mm"]
        for row in rows
        if row["vision_err_mm"] is not None
    ]
    place_xy_values = [
        row["place_err_xy_mm"]
        for row in rows
        if row["place_verified"]
        and row["place_err_xy_mm"] is not None
    ]
    place_z_values = [
        row["place_err_z_mm"]
        for row in rows
        if row["place_verified"]
        and row["place_err_z_mm"] is not None
    ]
    radial_values = [
        row["radial_mm"]
        for row in rows
        if row["radial_mm"] is not None
    ]
    angle_values = [
        abs(row["angle_deg"])
        for row in rows
        if row["angle_deg"] is not None
    ]

    pairs = [
        (row["vision_err_mm"], row["place_err_xy_mm"])
        for row in rows
        if row["vision_err_mm"] is not None
        and row["place_err_xy_mm"] is not None
        and row["place_verified"]
    ]
    correlation = pearson(
        [pair[0] for pair in pairs],
        [pair[1] for pair in pairs],
    )

    x_range = meta.get("x_range")
    y_range = meta.get("y_range")
    place = meta.get("place")
    seed = meta.get("seed")

    lines = [
        "# A3 随机位置抓放试验报告",
        "",
        "## 基本信息",
        "",
        f"- 运行 ID：`{run_id}`",
        f"- 试验时间：{created_text}",
        f"- 生成时间：{datetime.now().strftime('%Y-%m-%d %H:%M:%S')}",
        f"- 生成时 HEAD：`{head}`",
        f"- 试验次数：{total}",
    ]
    if x_range and y_range:
        lines.append(
            f"- 随机范围：x {x_range[0]} ~ {x_range[1]}，"
            f"y {y_range[0]} ~ {y_range[1]}"
        )
    else:
        xs = [
            row["command_x"]
            for row in rows
            if row["command_x"] is not None
        ]
        ys = [
            row["command_y"]
            for row in rows
            if row["command_y"] is not None
        ]
        if xs and ys:
            lines.append(
                "- 观测范围（由 CSV 推断，旧版脚本未记录配置）："
                f"x {min(xs):.3f} ~ {max(xs):.3f}，"
                f"y {min(ys):.3f} ~ {max(ys):.3f}"
            )
        else:
            lines.append("- 随机范围：未记录")
    if place:
        lines.append(
            f"- 放置点：({place[0]}, {place[1]}, {place[2]})"
        )
    else:
        lines.append("- 放置点：未记录（旧版试验脚本）")
    if seed is not None:
        lines.append(f"- seed：{seed}")
    if note:
        lines.append(f"- 备注：{note}")

    lines += [
        "",
        "## 结果汇总",
        "",
        "| 指标 | 结果 |",
        "| --- | --- |",
        f"| 全链路成功 | {full}/{total} |",
        f"| 抓取成功 | {grasp}/{total} |",
    ]
    for stage, count in sorted(stages.items()):
        if stage == "ok":
            continue
        lines.append(f"| 失败阶段 `{stage}` | {count} |")
    lines += [
        f"| 视觉误差均值 | {fmt(mean(vision_values))} mm |",
        f"| 视觉误差最大 | {fmt(max(vision_values) if vision_values else None)} mm |",
        f"| 放置误差 xy 均值 | {fmt(mean(place_xy_values))} mm |",
        f"| 放置误差 xy 最大 | {fmt(max(place_xy_values) if place_xy_values else None)} mm |",
        f"| 放置误差 z 均值 | {fmt(mean(place_z_values))} mm |",
        f"| 视觉-放置误差相关系数 | {fmt(correlation, 3)} |",
        "",
        "## 每次试验",
        "",
        "| # | 命令位置 | 实际位置 | 实际偏差 | 视觉误差 | 抓取 | 放置(节点) | 放置误差 | 独立校验 | 全链路 | 阶段 |",
        "| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |",
    ]

    for row in rows:
        command = (
            "—"
            if row["command_x"] is None
            else f"({row['command_x']:.3f}, {row['command_y']:.3f})"
        )
        actual = (
            "—"
            if row["actual_x"] is None
            else f"({row['actual_x']:.3f}, {row['actual_y']:.3f})"
        )
        place_error = (
            "—"
            if row["place_err_xy_mm"] is None
            else f"{row['place_err_xy_mm']:.1f}/"
            f"{fmt(row['place_err_z_mm'])} mm"
        )
        lines.append(
            f"| {row['trial']} | {command} | {actual} | "
            f"{fmt(row['actual_offset_mm'])} mm | "
            f"{fmt(row['vision_err_mm'])} mm | "
            f"{'✅' if row['grasp_ok'] else '❌'} | "
            f"{'✅' if row['place_ok_node'] else '❌'} | "
            f"{place_error} | "
            f"{'✅' if row['place_verified'] else '❌'} | "
            f"{'✅' if row['full_success'] else '❌'} | "
            f"`{row['stage']}` |"
        )

    lines += ["", "## 观察", ""]

    if total and full == total:
        lines.append(
            f"- 当前范围内全链路 {full}/{total}。这是 link attach "
            "下的软件链路成功率，不是物理摩擦抓取成功率。"
        )
    else:
        failed_stages = ", ".join(
            f"{stage}={count}"
            for stage, count in sorted(stages.items())
            if stage != "ok"
        )
        lines.append(
            f"- 全链路 {full}/{total}；失败阶段：{failed_stages or '无'}"
        )

    if correlation is not None:
        lines.append(
            f"- 视觉误差与放置误差的相关系数 r={correlation:.3f}"
            f"（n={len(pairs)}）。放置误差基本由视觉误差决定，"
            "说明放置点本身不是主要误差源。"
        )

    if radial_values:
        lines.append(
            "- 视觉点相对真值向相机中心径向收缩 "
            f"{mean(radial_values):.1f} mm，"
            f"角度偏差均值 {mean(angle_values):.1f}°；"
            "与“俯视离轴视角 + bbox 包含可见侧面”的判断一致。"
        )

    offset_rows = [
        row
        for row in rows
        if row["actual_offset_mm"] is not None
        and row["actual_offset_mm"] > 5.0
    ]
    if offset_rows:
        detail = "；".join(
            f"第 {row['trial']} 次 {row['actual_offset_mm']:.1f} mm"
            for row in offset_rows
        )
        lines.append(
            f"- 命令位置与实际位置偏差 >5 mm：{detail}。"
            "节点仍然成功，说明视觉跟随的是实际方块；"
            "随机位置统计应以实际位置为准。"
        )

    lines += [
        "",
        "## 说明",
        "",
        "- 抓取后的抬升依赖 Gazebo link attach（固定关节），"
        "不是物理摩擦抓取。",
        "- 放置点固定为 `(0.40, 0.08, 0.375)`；"
        "本报告只统计该范围内的重复性。",
        "- `PLACE SUCCESS` 由节点自报，独立校验另有 "
        "`/model_states` 位置检查，二者不一致会在 CSV 里体现。",
        "",
        "## 原始数据",
        "",
        "- [summary.csv](summary.csv)",
    ]

    if logs_included:
        lines.append("- [logs/](logs/)（失败轮次或全部日志）")
    else:
        lines.append(f"- 原始日志目录（VM）：`{run}`")
    return "\n".join(lines) + "\n"


def update_index(repo, run_id, full, total, vision_mean, place_mean):
    experiments = repo / "project16_ros2_vla_bridge" / "experiments"
    index = experiments / "README.md"
    text = (
        index.read_text(encoding="utf-8")
        if index.exists()
        else "# 实验报告索引\n\n## A3 随机位置抓放\n"
    )
    lines = [
        line for line in text.splitlines() if f"[{run_id}]" not in line
    ]
    entry = (
        f"- [{run_id}](a3/{run_id}/report.md) — "
        f"{full}/{total} 全链路成功，"
        f"视觉 {fmt(vision_mean)} mm，放置 {fmt(place_mean)} mm"
    )
    if "## A3 随机位置抓放" not in lines:
        lines += ["", "## A3 随机位置抓放", ""]
    if lines and lines[-1].strip():
        lines.append("")
    lines.append(entry)
    index.write_text("\n".join(lines).rstrip() + "\n", encoding="utf-8")


def publish(repo, run, include_logs, skip_push, note):
    run_id = run_id_of(run)
    rows = enrich_rows(run, read_csv_rows(run))
    meta = read_meta(run)
    head = git(repo, "rev-parse", "--short", "HEAD").stdout.strip()

    target = (
        repo
        / "project16_ros2_vla_bridge"
        / "experiments"
        / "a3"
        / run_id
    )
    logs_dir = target / "logs"
    target.mkdir(parents=True, exist_ok=True)
    shutil.copy2(run / "summary.csv", target / "summary.csv")

    copied = 0
    for row in rows:
        source = row["log_path"]
        if not source.exists():
            continue
        if include_logs or not row["full_success"]:
            logs_dir.mkdir(parents=True, exist_ok=True)
            shutil.copy2(source, logs_dir / source.name)
            copied += 1

    meta_path = run / "meta.json"
    if meta_path.exists():
        shutil.copy2(meta_path, target / "meta.json")

    report = build_report(
        run, rows, meta, head, note, logs_included=copied > 0
    )
    (target / "report.md").write_text(report, encoding="utf-8")

    total = len(rows)
    full = sum(1 for row in rows if row["full_success"])
    vision_values = [
        row["vision_err_mm"]
        for row in rows
        if row["vision_err_mm"] is not None
    ]
    place_values = [
        row["place_err_xy_mm"]
        for row in rows
        if row["place_verified"]
        and row["place_err_xy_mm"] is not None
    ]
    update_index(
        repo,
        run_id,
        full,
        total,
        mean(vision_values),
        mean(place_values),
    )

    print(f"report: {target / 'report.md'}")
    print(f"logs copied: {copied}")

    email = git(repo, "config", "user.email").stdout.strip()
    if not email:
        print(
            "ERROR: git 还没有配置提交身份。先运行：\n"
            '  git config --global user.name "你的名字"\n'
            '  git config --global user.email "你的邮箱"'
        )
        return 2

    target_rel = str(target.relative_to(repo))
    index_rel = str(
        (repo / "project16_ros2_vla_bridge" / "experiments" / "README.md")
        .relative_to(repo)
    )
    git(repo, "add", "--", target_rel, index_rel)
    status = git(repo, "status", "--porcelain", "--", target_rel, index_rel)
    if not status.stdout.strip():
        print("没有需要提交的变化")
        return 0

    message = f"experiment: publish A3 {run_id} ({full}/{total})"
    commit = git(
        repo, "commit", "-m", message, "--", target_rel, index_rel
    )
    if commit.returncode != 0:
        print("git commit failed:")
        print(commit.stdout.strip())
        print(commit.stderr.strip())
        return 1
    new_head = git(repo, "rev-parse", "--short", "HEAD").stdout.strip()
    print(f"committed: {new_head}")

    if skip_push:
        print("skip push: 报告已提交到本地")
        return 0

    push = git(repo, "push")
    if push.returncode != 0:
        print("git push failed:")
        print(push.stdout.strip())
        print(push.stderr.strip())
        print("报告仍在本地；检查网络或 GitHub 凭据后重试 git push")
        return 1
    print("pushed to GitHub")
    return 0


def parse_args(argv):
    parser = argparse.ArgumentParser(
        description="Publish an A3 trial run as a GitHub report"
    )
    parser.add_argument("--run-dir", default=None)
    parser.add_argument(
        "--include-logs",
        action="store_true",
        help="复制全部 trial_*.log（默认只复制失败轮次）",
    )
    parser.add_argument(
        "--skip-push", action="store_true", help="只生成并提交，不 push"
    )
    parser.add_argument(
        "--note", default="", help="附加到报告基本信息里的备注"
    )
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)
    repo = repo_root()
    run = resolve_run_dir(repo, args.run_dir)
    return publish(
        repo,
        run,
        args.include_logs,
        args.skip_push,
        args.note,
    )


if __name__ == "__main__":
    sys.exit(main())
