# 2026-11-11 Day 22 原始记录

## 今天的目标

把昨天的急停曲线变成数字：

```text
停止过渡时间
减速度
停止距离
```

并和不同减速度下的理想停止曲线对比。

## 预测

用户预测：

```text
A. 10 ms 左右
```

## 实际分析输出

```text
=== stop analysis ===
csv            : /home/wsq/embodied-ai-learning/project16_ros2_vla_bridge/output/stop_probe_20261110_193852.csv
samples        : 952
stop time      : 3.742 s
v before       : 0.0794 rad/s
transition     : 52.5 ms
deceleration   : 1.51 rad/s^2
stop distance  : joint1=0.00070 rad, joint2=0.00075 rad

ideal controlled stop:
  a=0.25 rad/s^2 -> T= 317.5 ms, distance=0.01260 rad
  a=0.50 rad/s^2 -> T= 158.8 ms, distance=0.00630 rad
  a=1.00 rad/s^2 -> T=  79.4 ms, distance=0.00315 rad
  a=2.00 rad/s^2 -> T=  39.7 ms, distance=0.00158 rad
=====================
```

## 第一次分析图的问题

第一次生成的 `stop_analysis_20261111_201215.svg` 中：

- 上半部分实测速度正确；
- 下半部分理想减速曲线画反了：从 `0` 往上升，而不是从 `v0` 往下降。

用户要求先检查图是否正确，确认后才发现这个绘图方向错误。

修复后重新生成：

```text
stop_analysis_20261111_202148.svg
```

## 截图

- 原始文件：`personal journal/stop2.png`
- 项目副本：`project16_ros2_vla_bridge/assets/stop_analysis.png`

## 我的困惑

- 预测的 10 ms 和实测的 52.5 ms 差了 5 倍，为什么整张图看起来还是像垂直的？
- joint2 停止后速度穿过 0 到正值，再回到 0，这是回弹还是计算噪声？
- 52.5 ms、1.51 rad/s² 放到真机上能不能接受？
- 速度更高时，停止时间会按比例变长，还是仍由控制器固定行为决定？
