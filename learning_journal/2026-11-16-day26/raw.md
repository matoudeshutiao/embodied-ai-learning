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

## 当前状态

```text
代码完成
语法检查通过
VM 实跑待补
```

## 还没验证

- 夹爪下探高度是否合适
- 夹爪闭合后是否真的夹住
- 方块是否跟着抬起
- 是否会夹空或碰倒
