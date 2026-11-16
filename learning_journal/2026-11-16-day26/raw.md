# 2026-11-16 Day 26 原始记录

## 今天的目标

先不做放置，只做“抓起来”。

## 实现

新增 `panda_pick.py`：

```text
等 /vision/target_pose
张开夹爪 0.04
MoveIt 到 pre-grasp（z + 0.20）
下探到 grasp（z + 0.10）
闭合夹爪 0.00
抬起到 lift（z + 0.30）
```

MoveIt action：

```text
/move_action
```

夹爪 action：

```text
/panda_gripper_controller/follow_joint_trajectory
```

下探前删除 planning scene 里的红方块：

```text
red_cube → CollisionObject.REMOVE
```

`panda_scene` 改成只发布 3 次，避免重新添加红方块。

## 运行命令

```bash
source ~/embodied-ai-learning/project16_ros2_vla_bridge/scripts/dev_sync_vla.sh
```

```bash
ros2 launch vla_bridge panda_moveit_gazebo.launch.py rviz_config:=/home/wsq/panda_moveit_view.rviz
```

```bash
ros2 run vla_bridge panda_pick
```

## 第一次实跑：抓空

日志：

```text
[panda_pick]: gripper open: code=0, ok=True
[panda_pick]: pre-grasp: x=0.453, y=-0.145, z=0.600
[panda_pick]: pre-grasp: code=1, ok=True
[panda_pick]: grasp: x=0.453, y=-0.145, z=0.500
[panda_pick]: grasp: code=1, ok=True
[panda_pick]: gripper close: code=0, ok=True
[panda_pick]: lift: x=0.453, y=-0.145, z=0.700
[panda_pick]: lift: code=1, ok=True
[panda_pick]: pick sequence finished
```

用户观察：

```text
确实是抓了但是没有抓取成功，抓歪了然后还告诉我执行完毕
```

截图：

- 原始文件：`personal journal/failed_fetch.png`
- 项目副本：`project16_ros2_vla_bridge/assets/panda_pick_failed.png`

## 加验证

```text
1. /joint_states 的 finger 平均位置 > 0.005
2. /gazebo/get_model_state 的 red_cube z 增加 > 0.05
```

失败时输出 `GRASP FAILED`，并张开夹爪。

## 当前状态

```text
验证代码已加
VM 重测待补
```
