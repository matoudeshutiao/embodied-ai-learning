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

## 真机接口层第一步：mock hardware 实测

启动：

```bash
ros2 launch vla_bridge mock_hardware_demo.launch.py
```

完整一条动作块：

```text
[mock_policy-7] [INFO] [1794452315.489062468] [mock_policy]: published action chunk: 5 points
[safe_executor-6] [INFO] [1794452315.489805817] [safe_executor]: queued action: 5 points, in_flight=1, pending=1, busy=False
[safe_executor-6] [INFO] [1794452315.490021265] [safe_executor]: time aligned: age=0.001s, duration=1.000s
[safe_executor-6] [INFO] [1794452315.490197135] [safe_executor]: sending action: 5 points, joints=['joint1', 'joint2'], in_flight=1, pending=0
[ros2_control_node-1] [INFO] [1794452315.490712539] [joint_trajectory_controller]: Received new action goal
[ros2_control_node-1] [INFO] [1794452315.490746879] [joint_trajectory_controller]: Accepted new action goal
[safe_executor-6] [INFO] [1794452315.491252389] [safe_executor]: action goal accepted
[ros2_control_node-1] [INFO] [1794452316.491672793] [joint_trajectory_controller]: Goal reached, success!
[safe_executor-6] [INFO] [1794452316.544377956] [safe_executor]: action finished: code=0, message=Goal successfully reached!
[safe_executor-6] [INFO] [1794452316.544833521] [safe_executor]: stats: queued=159, executed=159, canceled=0, failed=0, dropped=0, capacity_dropped=0, age_dropped=0
```

关节状态频率：

```text
average rate: 100.048
	min: 0.009s max: 0.011s std dev: 0.00032s window: 101
```

安全状态：

```text
data: 'ok: 5 points'
```

用户观察：

```text
一直在动
```

## Panda mock hardware 实测

启动：

```bash
ros2 launch vla_bridge panda_mock_hardware_demo.launch.py
```

发送 7 关节轨迹：

```bash
python3 ~/embodied-ai-learning/project16_ros2_vla_bridge/scripts/panda_wave.py
```

日志：

```text
[panda_wave] sending 7-joint trajectory: 2 points over 4.0s
[panda_wave] goal accepted, waiting for result
[panda_wave] result: code=0, message=Goal successfully reached!
```

截图：

- 原始文件：`personal journal/no_gazebo(humble+panda).png`
- 项目副本：`project16_ros2_vla_bridge/assets/panda_mock_hardware.png`

用户观察：

```text
ok了
```

注意：

```text
panda_wave.py 直接发给 panda_arm_controller
没有经过 safety_filter / safe_executor
safety_filter 目前还是 joint1/joint2
```

## Panda 7 关节完整链路

启动：

```bash
ros2 launch vla_bridge panda_full_chain_demo.launch.py
```

实测日志：

```text
[ros2_control_node-1] [panda_arm_controller]: Accepted new action goal
[safe_executor-6]: action goal accepted
[ros2_control_node-1] [panda_arm_controller]: Goal reached, success!
[safe_executor-6]: action finished: code=0, message=Goal successfully reached!
[safe_executor-6]: stats: queued=69, executed=69, canceled=0,
                   failed=0, dropped=0, capacity_dropped=0, age_dropped=0
[mock_policy-7]: published action chunk: 5 points
[safe_executor-6]: queued action: 5 points, in_flight=1, pending=1, busy=False
[safe_executor-6]: time aligned: age=0.002s, duration=1.000s
[safe_executor-6]: sending action: 5 points,
                   joints=['panda_joint1', ..., 'panda_joint7']
```

安全状态：

```text
/policy/safety_status: ok: 5 points
```

`/joint_states`：

```text
name:
- panda_finger_joint1
- panda_joint1
- panda_joint2
- panda_finger_joint2
- panda_joint3
- panda_joint4
- panda_joint5
- panda_joint6
- panda_joint7

position: 正常变化
velocity: 全部 0.0
effort: 全部 .nan
```

用户观察：

```text
有在动
```

限制：

```text
MoveIt 的 mock hardware 没有打开 calculate_dynamics
velocity/effort 不可信
只能验证位置控制
```

