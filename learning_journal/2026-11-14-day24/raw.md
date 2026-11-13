# 2026-11-14 Day 24 原始记录

## 今天的目标

Panda 进 Gazebo：

```text
7 个关节能动
夹爪能开合
```

不接相机、不接 MoveIt2、不做抓取。

## 踩坑过程

### 1. MoveIt 的 Panda URDF 没有 inertial

Gazebo 日志：

```text
Skipping joint in the URDF named 'panda_joint1' which is not in the gazebo model.
...
Skipping joint in the URDF named 'panda_finger_joint2' which is not in the gazebo model.
```

检查：

```text
grep -c "<inertial" /tmp/panda.urdf → 0
```

MoveIt 的 Panda 模型是给规划用的，不含质量/惯性；Gazebo 需要 inertial。

### 2. panda_body.urdf 是 fragment

xacro 报错：

```text
XML parsing error: junk after document element
```

原因是多个顶层元素，没有单个 `<robot>` 根节点。

### 3. 旧 Gazebo 进程残留

```text
[gzserver-1]: process has died exit code 255
[spawn_entity]: Spawn status: Entity [panda] already exists.
Controller already loaded
Failed to activate controller
```

清掉旧 `gzserver`/`gzclient`/spawner 后恢复。

### 4. 时间源不一致

`panda_wave.py` 打印：

```text
goal accepted, waiting for result
```

然后一直不结束。

原因：

```text
Gazebo：use_sim_time=True，使用 /clock
脚本：没设置 use_sim_time，使用系统时间
```

JTC 在等一个“未来”的轨迹开始时间。

## 最终成功日志

机械臂：

```text
[panda_wave] sending 7-joint trajectory: 2 points over 4.0s
[panda_wave] goal accepted, waiting for result
[panda_wave] result: code=0, message=Goal successfully reached!
```

夹爪：

```text
open: position=0.040, duration=1.5s
open: code=0, message=Goal successfully reached!

close: position=0.000, duration=1.5s
close: code=0, message=Goal successfully reached!

open again: position=0.040, duration=1.5s
open again: code=0, message=Goal successfully reached!
```

用户观察：

```text
两个都动了
```

## 零时间戳验证

清掉旧 Gazebo/controller_manager 后，不带 `use_sim_time` 直接运行：

```bash
python3 ~/embodied-ai-learning/project16_ros2_vla_bridge/scripts/panda_wave.py
```

结果：

```text
sending 7-joint trajectory: 2 points over 4.0s
goal accepted, waiting for result
result: code=0, message=Goal successfully reached!
```

用户观察：

```text
ok了
```
