# 2026-11-12 Day 23 原始记录

## 今天的目标

比较低速和高速下，watchdog 取消动作的停止时间是否变化。

## 预测

用户预测：

```text
B. transition 变成约 100–130 ms
```

## 低速基线（Day 22）

```text
v before       : 0.0794 rad/s
transition     : 52.5 ms
deceleration   : 1.51 rad/s^2
stop distance  : joint1=0.00070 rad, joint2=0.00075 rad
```

## 高速实验

命令：

```bash
python3 project16_ros2_vla_bridge/scripts/stop_probe.py --ros-args -p target1:=1.2 -p target2:=-0.8
```

`stop_probe` 输出：

```text
stop time    : 4.014 s
stop reason  : watchdog: no new action
joint1 calc : v_before=+0.2374 rad/s, v_after=+0.0000 rad/s, drop=0.2374 rad/s
joint2 calc : v_before=-0.1783 rad/s, v_after=+0.0000 rad/s, drop=0.1783 rad/s
```

`analyze_stop.py` 输出：

```text
v before       : 0.2448 rad/s
transition     : 51.7 ms
deceleration   : 4.74 rad/s^2
stop distance  : joint1=0.00245 rad, joint2=0.00181 rad
```

## 用户观察

```text
图看起来和昨天的没区别啊
```

```text
所以之前是你的脚本写的我们固定停止时间？还是机器人默认的？
如果是你写的，你凭什么问我，抱着已经知道的答案编出一个问题
```

## 我的困惑

- 为什么速度变成 3 倍，图看起来还是同一条垂直断崖？
- transition 几乎不变，说明 cancel 是固定时间还是固定减速度？
- 如果速度继续增大，减速度会一直线性增大吗？
- 真机上应该限制减速度，还是限制停止时间？

## 截图/输出

- CSV: `output/stop_probe_20261112_101729.csv`
- SVG: `output/stop_analysis_20261112_101734.svg`
