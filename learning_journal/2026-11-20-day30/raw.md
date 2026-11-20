# 2026-11-20 Day 30 原始记录

## 今天的目标

- 把 Day 29 的“抓起来”补成完整的“抓 → 搬 → 放”。
- 抓取位置从 Gazebo 真值切换到视觉估计，真值只用于误差评估和最后校验。
- 记录过程中遇到的阻塞，区分“抓取问题”和“系统问题”。

## 完成状态

```text
A1 视觉去真值      ✅ 多帧中值，误差 6.3 mm
A2 place 完整回路   ✅ 搬移 → 下放 → detach → 张爪 → 抬离 → 校验
A3 20 次成功率统计   ❌ 未做
A4 空抓检测/重试    ✅ 已实现，重试逻辑未实测
```

最终成功日志（节选）：

```text
vision median over 5 samples: x=0.4538, y=-0.1450, z_top=0.3990
vision vs truth: vision=(0.4538, -0.1450, 0.3740),
  truth=(0.4500, -0.1500, 0.3740),
  err_xy=6.3 mm, err_z=0.0 mm
vertical grasp cube: x=0.454, y=-0.145, z=0.374 (source=vision)

cartesian descent: distance=0.105 m, chunks=6
cartesian descent [1/6]: waypoints=4, start_jump=0.0000 rad,
  max_step=0.0303 rad
...
cartesian descent [6/6]: code=0, ok=True

finger positions after close: [0.0205, 0.0277], average=0.0241
attach link panda_link7: success=True

cartesian lift: distance=0.205 m, chunks=11
cube after lift pose: (0.4502, -0.1498, 0.5790)
GRASP SUCCESS

place target: x=0.400, y=0.080, z=0.375
place traverse: distance=0.231 m, chunks=12
place descent: distance=0.205 m, chunks=11
detach link panda_link7: success=True
gripper open at place: effort=10.00 N
place retreat: distance=0.205 m, chunks=11

cube after place: (0.3964, 0.0752, 0.3741)
place error: xy=6.0 mm, z=0.9 mm
PLACE SUCCESS
```

## 今天遇到的三个阻塞

### 1. 抓起后一直没放下

现象：

```text
place descent: cartesian failed, code=1, fraction=0.364
```

原因：原来的放置点 `(0.45, 0.15)` 贴着 `blue_box` 的角。
`blue_box` 占 `x ∈ [0.47, 0.63]`、`y ∈ [0.16, 0.32]`、`z ∈ [0.33, 0.43]`，
左手指下探时会擦到它，笛卡尔路径到 36% 就被碰撞检查截断。

代码按设计直接返回，所以机械臂一直抓着方块停在放置点上方。

修复：

```text
放置点改为 (0.40, 0.08)
先保持抬升高度水平搬到放置点上方（place traverse）
再垂直下放（place descent）
```

### 2. 下爪不是垂直的，画了一个大弧线

第一轮日志：

```text
cartesian descent: waypoints=7
max_step=0.0699 rad
```

MoveIt 的笛卡尔路径本身是直的，但实际执行的轨迹只有 7 个点，
相邻点约 1.5 cm。控制器在点与点之间按关节空间插值，
末端就会鼓出弧线。

修复：把一条直线拆成每 2 cm 一段，逐段重新计算并执行：

```text
cartesian descent: distance=0.105 m, chunks=6
每段 waypoints=3~4，max_step≈0.019~0.030 rad
```

### 3. Gazebo 崩溃 + 节点永久卡住

现象：

```text
[ERROR] [gzserver-1]: process has died [pid 4173, exit code -11, ...]
```

`exit code -11` 是 SIGSEGV。`gzserver` 一死，Gazebo 窗口里机械臂就没了；
同时 `panda_pick` 卡在 attach 服务调用上，因为原来的
`rclpy.spin_until_future_complete` 没有超时。

修复：

```text
attach/detach 服务加 3 s 超时
attach 顺序改为 panda_link7 → panda_link8 → panda_hand
失败时打印 “service timeout (gzserver may have crashed)”
```

另外确认：崩溃后 `gzclient` 只是空壳，需要 `pkill -f gzserver` /
`pkill -f gzclient` 后整场重启，不能只重跑 `panda_pick`。

## 视觉在流程里的位置

相机是固定俯视 eye-to-hand：

```text
世界 / panda_link0 → camera_link
  x=0.55, y=0, z=1.15, pitch=+90°

camera_link → camera_optical_frame
  roll=-90°, yaw=-90°
```

检测器把像素 + 深度反投影到相机坐标，再用 TF 转到 `panda_link0`。
`panda_pick` 收 5 帧取中值，抓取 x/y/z 全部来自 `/vision/target_pose`：

```python
vx, vy, vz_top = vision
vz = vz_top - half      # 视觉看到顶面，减 2.5 cm 得到方块中心
x, y, z = vx, vy, vz
```

真值 `/model_states` 只用于：

```text
打印视觉误差
抬升前方块 z 基线
放置后位置校验
重试前判断方块是否被碰偏
```

没有参与抓取坐标。

用户没有“感觉到视觉参与”的原因：

1. 方块从未移动，视觉 `(0.4538, -0.1450)` 与真值 `(0.4500, -0.1500)`
   只差 6.3 mm，机械臂运动看起来完全一样；
2. 放置点是参数，不是视觉；
3. 只在开头看一次，运动过程中没有视觉反馈。

## 用户原话

> 终于实现了抓起放下，你用通俗易懂语言给我讲一下为什么我们这么简单一个动作，
> 抓取，放下，会这么麻烦，今天遇到了什么问题

> 不对，gazebo里面看不到机械臂了，怎么回事

> 我们现在的相机坐标在哪？我比较偏向于eye-in-hand 然后我们后面接手眼标定。
> 不过这个留给明天吧。然后我们今天的是视觉实现的吗，今天阶段A完成了吗，
> 为什么我没有感觉到视觉的参与，如果有的话，把主要代码给我看看

## 代码提交

```text
a5b456a feat(vla_bridge): close pick-and-place loop with vision pose
daf71b0 fix(vla_bridge): avoid blue box at place point and log cartesian jumps
a72d7ab fix(vla_bridge): time out attach services and chunk cartesian moves
```

## 还没做

```text
A3：20 次随机位置成功率统计
物理摩擦抓取：仍未解决，当前抬升依赖 link attach
成功率统计前需要让 panda_scene 的方块位置跟随真实位姿
eye-in-hand + 手眼标定：下一步
真机：未开始
```

## 说明

今天的成功是“视觉定位 + 规划 + link attach 固定关节 + 放置校验”的完整软件闭环，
不是真实夹爪在 10 N 下夹住 5 cm 方块。`PLACE SUCCESS` 证明的是流程和验证逻辑完整，
不能直接迁移成真机抓取结论。
