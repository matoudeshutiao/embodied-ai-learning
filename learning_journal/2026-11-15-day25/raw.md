# 2026-11-15 Day 25 原始记录

## 今天的目标

把 MoveIt2 从“能启动”推进到“能执行 pre-grasp”。

## MoveIt Plan 失败

日志：

```text
Starting state is just outside bounds (joint 'panda_joint4')
Start state appears to be in collision
panda_link5 和 panda_link7 接触
panda_joint4 is constrained to be above the maximum bounds
OMPL: Skipping invalid start state
Motion planning start tree could not be initialized
```

`/joint_states`：

```text
panda_joint4 = 0.000055
```

真实合法范围：

```text
-3.0718 ~ -0.0698
```

## spawn_entity 没有 -J

`ros2 run gazebo_ros spawn_entity.py --help` 输出中只有：

```text
-file
-topic
-database
-entity
-timeout
-x / -y / -z
-R / -P / -Y
```

没有 `-J`。所以之前加的 `-J panda_joint4 -2.356194` 无效，spawn 失败。

## 修复

新增 `panda_home`：

```text
启动后通过 panda_arm_controller 把 Panda 移到合法初始位姿
```

新增 `panda_scene`：

```text
把桌子、三个方块、蓝盒发布到 /planning_scene
```

MoveGroup action 实际名字：

```text
/move_action
```

不是 `/move_group`。

## pre-grasp 实测

```text
[panda_move_to_target]: pre-grasp target: x=0.453, y=-0.145, z=0.600
[panda_move_to_target]: sending MoveGroup pre-grasp request
[panda_move_to_target]: pre-grasp execution succeeded
```

用户观察：

```text
现在确实是移动过去了，但是姿势很诡异啊，抓夹是歪的，整个机械臂躺在红色方块上方
```

## 为什么姿势奇怪

```text
Panda 7 自由度冗余
多组 IK 解
只约束位置和朝下，没有肘部/腕部自然姿态约束
OMPL RRTConnect 随机采样，只保证可行、无碰撞
```

## 一键同步

新增：

```text
scripts/dev_sync_vla.sh
```

用法：

```bash
source ~/embodied-ai-learning/project16_ros2_vla_bridge/scripts/dev_sync_vla.sh
```
