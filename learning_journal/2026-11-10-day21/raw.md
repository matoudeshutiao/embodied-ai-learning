# 2026-11-10 Day 21 原始记录

## 今天的目标

不修改安全层，只测：

```text
watchdog 取消动作时，速度是慢慢降到 0，还是直接掉到 0
```

## 用户原始观察

1. 第一版图：

```text
什么狗屁图片，这速度全程没变
```

2. 第一次运行脚本时，20 秒没有收到 `/joint_states`：

```text
[ERROR] [1794310221.442463758] [stop_probe]: no /joint_states; is the launch running?
[ERROR] [1794310221.442676823] [stop_probe]: no samples recorded
```

3. 第二次运行成功，终端输出：

```text
=== stop probe ===
samples      : 952
stop time    : 3.759 s
stop reason  : watchdog: no new action
joint1 calc : v_before=+0.0693 rad/s, v_after=-0.0000 rad/s, drop=0.0693 rad/s
joint1 raw  : flat at +0.0000 rad/s; position difference is used instead
joint2 calc : v_before=-0.0772 rad/s, v_after=+0.0000 rad/s, drop=0.0772 rad/s
==================

[INFO] [1794310732.190746897] [stop_probe]: CSV: /home/wsq/embodied-ai-learning/project16_ros2_vla_bridge/output/stop_probe_20261110_193852.csv
[INFO] [1794310732.191058942] [stop_probe]: SVG: /home/wsq/embodied-ai-learning/project16_ros2_vla_bridge/output/stop_probe_20261110_193852.svg
[INFO] [1794310732.191250251] [stop_probe]: safety statuses: [(1.5629468499998893, 'ok: 1 points'), (3.75885510299986, 'watchdog: no new action')]
```

4. 打开 SVG 的命令报错：

```text
xdg-open: unexpected argument 'project16_ros2_vla_bridge/output/stop_probe_20261110_193852.svg'
Try 'xdg-open --help' for more information.
```

## 截图

- 原始文件：`personal journal/STOP.png`
- 项目副本：`project16_ros2_vla_bridge/assets/stop_profile.png`

## 我的困惑

- 为什么 `joint1` 的 `/joint_states.velocity` 全程是 `0.0000 rad/s`，但位置明明在变化？
- 第一次图的速度线为什么全程不变？
- 如果位置差分能看出急停，真机上应该信哪个速度来源？

## 失败尝试

- 第一版直接画 `/joint_states.velocity`，得到一条几乎不变的速度线。
- 第一次运行脚本时 launch 没启动或不在同一个 ROS 图，20 秒没有 `/joint_states`。
