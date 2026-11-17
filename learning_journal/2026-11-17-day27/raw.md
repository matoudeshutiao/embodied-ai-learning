# 2026-11-17 Day 27 原始记录（调试延续到 11-18 凌晨）

## 今天的目标

- 放弃竖直下爪，改成侧向抓取：夹爪保持水平，从方块两侧夹住。
- 先跑通 `pre-grasp → grasp → close → lift`，暂不做 place。
- 保留桌子碰撞对象，不再用删除桌子的方式绕过碰撞。
- 代码、launch、world、URDF 和碰撞对象修改前必须先申请。

## 背景：Day 26 竖直抓取失败

Day 26 的 `panda_pick` 日志里所有 MoveIt action 都返回成功：

```text
gripper open:  code=0
pre-grasp:     code=1
grasp:         code=1
gripper close: code=0
lift:          code=1
pick sequence finished
```

但红方块仍在桌上。用户观察：

```text
确实是抓了但是没有抓取成功，抓歪了然后还告诉我执行完毕
```

后续尝试中，用户观察到：

```text
你的解决方案应该是没问题的，应该就是抓夹间距缩到0导致被挤走
```

以及明确约束：

```text
现在开始不允许你随便改代码，必须先请求我
```

## 今天新增的代码

新增：

```text
project16_ros2_vla_bridge/ros2_ws/src/vla_bridge/vla_bridge/panda_side_pick.py
```

并加入 `setup.py`：

```text
panda_side_pick = vla_bridge.panda_side_pick:main
```

侧抓姿态：

```text
panda_hand orientation q = (0, 0.7071068, 0, 0.7071068)
手指沿 world y 闭合
手部 z 轴朝 world +x
```

初始参数：

```text
pre_x_offset   = 0.20
grasp_x_offset = 0.12
side_z_offset  = 0.02
lift_height    = 0.20
close_position = 0.02
```

对应位姿：

```text
pre-grasp: x = 0.45 - 0.20 = 0.25
grasp:     x = 0.45 - 0.12 = 0.33
z = 0.375 + 0.02 = 0.395
```

第一次提交：

```text
cbbdbcb feat: add side grasp node
```

## 命令

```bash
source ~/embodied-ai-learning/project16_ros2_vla_bridge/scripts/dev_sync_vla.sh
```

```bash
ros2 run vla_bridge panda_side_pick
```

## 失败 1：pre-grasp 规划超时

节点日志：

```text
waiting for /vision/target_pose
target received: x=0.453, y=-0.145, z=0.400
side grasp cube: x=0.450, y=-0.150, z=0.375
removed red_cube from planning scene
gripper open: position=0.040
gripper open: code=0, ok=True
side pre-grasp: x=0.250, y=-0.150, z=0.395
side pre-grasp: code=99999, ok=False
```

从发出到失败约 5.13 秒，接近 `allowed_planning_time=5.0`。

move_group 日志：

```text
More than 80% of the sampled goal states fail to satisfy the constraints
Unable to sample any valid states for goal tree
ParallelPlan::solve(): Unable to find solution by any of the threads in 5.009499 seconds
Unable to solve the planning problem
Catastrophic failure
```

`99999` 是 MoveIt 的通用 FAILURE，不是具体原因。

## IK 诊断

### x = 0.25, z = 0.395

```text
error_code.val = 1
panda_joint4 = -3.0717998598841323
```

`panda_joint4` 的软下限正好是：

```text
-3.0718
```

也就是说，这个 pre-grasp 把肘关节压到了极限位置。

### x = 0.37, z = 0.395

```text
error_code.val = 1
panda_joint4 = -2.69902739960677
```

离开极限，说明手腕往远离底座的方向移动是可行的。

## 第二次修改：上方下探侧抓

修改内容：

```text
grasp_x_offset: 0.12 → 0.10
新增 pre_z_offset = 0.12
pre-grasp 和 grasp 使用同一个 x
```

新位姿：

```text
pre-grasp: x=0.350, y=-0.150, z=0.515
grasp:     x=0.350, y=-0.150, z=0.395
```

另外加了 3 秒等待，避免 `panda_scene` 在删除红方块之后又发布场景。

提交：

```text
2c8e89c fix: approach side grasp from above
```

## 失败 2：位置满足，姿态约束失败

节点日志：

```text
side pre-grasp: x=0.350, y=-0.150, z=0.515
side pre-grasp: code=99999, ok=False
```

用户观察：

```text
机械臂不动
```

move_group 日志：

```text
Position constraint satisfied on link 'panda_link8'.
Desired: 0.350000, -0.150000, 0.514801
Current: 0.349167, -0.146951, 0.510227

Orientation constraint violated for link 'panda_hand'.
Quaternion desired: 0.000000 0.707107 0.000000 0.707107
Quaternion actual:  0.879186 -0.466865 -0.056169 -0.076905
error: x=1.713462, y=1.380042, z=0.451822
tolerance: x=0.100000, y=0.100000, z=0.100000
```

结论：

```text
位置能到
采样阶段没有找到满足 panda_hand 姿态约束的状态
这次日志指向姿态约束，不是位置/桌高
是否完全无 IK，还没有单独用 /compute_ik 确认
```

## 失败尝试汇总

1. 竖直下爪：夹爪在方块顶部闭合，位置/间隙不对，方块被推走。
2. 侧抓 pre x=0.25：手腕太靠近底座，joint4 到达 -3.0718 极限，goal sampler 失败。
3. 侧抓上方下探 pre z=0.515：位置约束满足，`panda_hand` 姿态约束失败。
4. 规划场景时序：`panda_scene` 在侧抓节点删除红方块后重新发布桌面场景，可能把红方块加回来；后来加入等待。

## 当前状态

```text
侧抓节点已写
两个 pre 位姿都未规划成功
还没有成功抓起方块
没有新的成功截图
```

当前准备测试：

```bash
ros2 run vla_bridge panda_side_pick --ros-args -p pre_z_offset:=0.05
```

如果仍失败，再测：

```bash
ros2 run vla_bridge panda_side_pick --ros-args -p pre_z_offset:=0.0
```

## 截图

今天没有新的成功截图。

Day 26 的抓取失败截图仍保存在：

```text
project16_ros2_vla_bridge/assets/panda_pick_failed.png
```

## 用户明确约束

- 不允许未经确认删除桌子或修改碰撞对象。
- 代码、launch、world、URDF 和规划场景修改前必须先申请。

## 需要解释的问题（整理时补充，待用户回答）

以下不是用户原话，是整理日志时需要继续解释的问题：

- 为什么位置约束满足，姿态约束却失败？
- 为什么 `panda_joint4=-3.0718` 是危险信号？
- 为什么把 pre 抬高后反而采样不到？
- 夹爪 TCP 到 `panda_link8` 的实际距离是多少？
