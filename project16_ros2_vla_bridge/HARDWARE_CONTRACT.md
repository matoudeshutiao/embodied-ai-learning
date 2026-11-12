# 真机接口合同 v0：Franka Emika Panda

> 状态：设计合同，未在真机验证。
>
> 依据：前几天的 PyBullet `franka_panda` 模型、MoveIt2 的 Panda 配置，
> 以及 Franka Emika Panda 公开规格。
>
> 用途：给 fake hardware plugin、真机驱动适配和安全层提供统一约定。
> 如果实验室实际拿到的是 FR3 或其他型号，必须逐项复核，不能直接套用。

---

## 1. 机器人规格

| 项目 | 数值 |
| --- | --- |
| 型号 | Franka Emika Panda（7 自由度协作臂） |
| 自由度 | 7 个旋转关节 |
| 负载 | 3 kg（包含夹爪/工具） |
| 最大臂展 | 约 855 mm |
| 重复定位精度 | ±0.1 mm（ISO 9283） |
| 本体重量 | 约 17.8 kg |
| 最大末端速度 | 约 2 m/s（厂家规格） |
| 控制接口 | Franka Control Interface（FCI），1 kHz |
| 夹爪 | Franka Hand，两个 prismatic 手指 |

注意：

```text
3 kg 是总负载，不是“额外负载”。
夹爪、相机、工具、线缆都算在里面。
```

---

## 2. 关节合同

关节名必须使用：

```text
panda_joint1 ... panda_joint7
```

### 2.1 位置、速度、力矩限制

| 关节 | 下限 rad | 上限 rad | 速度上限 rad/s | 力矩上限 N·m |
| --- | ---: | ---: | ---: | ---: |
| panda_joint1 | -2.8973 | 2.8973 | 2.175 | 87 |
| panda_joint2 | -1.7628 | 1.7628 | 2.175 | 87 |
| panda_joint3 | -2.8973 | 2.8973 | 2.175 | 87 |
| panda_joint4 | -3.0718 | -0.0698 | 2.175 | 87 |
| panda_joint5 | -2.8973 | 2.8973 | 2.610 | 12 |
| panda_joint6 | -0.0175 | 3.7525 | 2.610 | 12 |
| panda_joint7 | -2.8973 | 2.8973 | 2.610 | 12 |

重要：

```text
joint4 的合法范围是负区间，不是对称的 ±3.07
joint6 只有正区间
不能把 7 个关节都当成对称限位
```

### 2.2 零位与方向

```text
URDF 零位：所有关节角为 0 的姿态
真实机器人零位：必须现场标定/核对
正方向：必须逐关节低速验证
```

合同要求：

```text
1. 不允许假设真实机器人方向和 URDF 完全一致
2. 第一次上电必须单关节、低速、小角度验证方向
3. 记录每个关节的符号、零位偏差和减速比
```

---

## 3. 夹爪合同

| 项目 | 数值 |
| --- | --- |
| 关节名 | `panda_finger_joint1`, `panda_finger_joint2` |
| 类型 | prismatic |
| 每指行程 | 0.0 – 0.04 m |
| 总开度 | 约 0.08 m |
| 速度上限 | 0.2 m/s |
| 力上限 | 20 N（URDF 值，实际以硬件为准） |

v0 只要求机械臂 7 关节，不要求夹爪闭环。

夹爪后续单独用：

```text
GripperActionController
或 panda_hand_controller
```

---

## 4. ros2_control 接口合同

### 4.1 command interface

v0 选择：

```text
position
```

原因：

```text
当前 JTC 和 safety_filter 都是 position 语义
先保持执行栈不变
真机适配层负责 position → 真实控制量
```

但必须确认实验室的 Franka ROS2 驱动：

```text
官方 franka_ros2 的硬件接口可能是 effort
如果是 effort，需要适配层或改 JTC command interface
不能假设 position 一定可直接写进 Franka
```

### 4.2 state interface

至少提供：

```text
position
velocity
```

建议同时提供：

```text
effort
```

用途：

```text
position：JTC 反馈、限位判断
velocity：安全层、停止策略
effort：负载/碰撞/异常检测
```

当前 Panda mock hardware 实测：

```text
velocity：全部 0.0
effort：全部 NaN
```

原因是 MoveIt 的 `panda.ros2_control.xacro` 没有打开 `calculate_dynamics`。
因此 mock 只能验证位置控制，不能拿它的 velocity/effort 做安全判断。
真机合同要求真实 state interface，或者给 mock/fake hardware 补上动力学计算。

### 4.3 controller 与 action

目标 action：

```text
FollowJointTrajectory
```

控制器名可以是：

```text
/joint_trajectory_controller/follow_joint_trajectory
```

如果实验室使用：

```text
/panda_arm_controller/follow_joint_trajectory
```

只改 `safe_executor` 的 `action_name`，不改协议。

JTC 负责：

```text
按 time_from_start 插值
按更新率写 command interface
读 state interface
返回 goal 成功/取消/失败
```

---

## 5. 频率与时间合同

| 项目 | 当前仿真 | 真机目标 |
| --- | ---: | ---: |
| 控制器更新率 | 100 Hz | 1000 Hz（FCI） |
| `/joint_states` | 100 Hz | 500–1000 Hz |
| 状态超时 | 0.5 s | ≤ 20 ms |
| 动作最大年龄 | 0.5 s | 50–100 ms |
| 策略 watchdog | 2 s | 300–500 ms（按策略频率定） |
| 队列在途 | 2 | 1–2 |

说明：

```text
0.5 s 的 state_timeout 只适合仿真
真机 1 kHz 控制下，20 ms 已经很多个周期
真机必须使用系统时间，use_sim_time=false
```

时间戳要求：

```text
每条 /joint_states 带硬件或驱动时间戳
动作块带生成时间
安全层计算 age = now - stamp
不允许用“收到消息的时间”替代硬件时间
```

---

## 6. 安全合同

### 6.1 软件限位

建议在 URDF 硬限位内再加安全余量：

```text
位置软限位：硬限位 ± 0.05 rad
速度软限位：硬限位 × 0.8
力矩软限位：硬限位 × 0.8
```

具体余量必须根据实际负载和任务调整。

### 6.2 停止策略

| 状态 | 策略 |
| --- | --- |
| 状态正常、策略断流 | 受控减速到 0 |
| 状态正常、用户停止 | 受控减速到 0 |
| 状态过期/不可信 | 立即取消/刹车，不盲目规划新位置 |
| 硬件急停 | 断使能/抱闸，软件不参与 |
| 跟随误差过大 | 立即停止并报错 |

目标减速度 v0：

```text
初始上限：2.0 rad/s²
必须用真机低速验证后再调整
```

注意：

```text
当前仿真 cancel 在 0.245 rad/s 时约 4.74 rad/s²
已经高于这个初始上限
```

### 6.3 速度与负载

```text
初始末端速度上限：0.25 m/s
负载：不超过 3 kg，包含夹爪/工具
超过额定负载时，降速并重新做风险评估
```

### 6.4 硬件安全

软件安全层不能替代：

```text
硬件急停
STO / 断使能
抱闸
安全围栏/安全扫描仪
厂家的安全手册和风险评估
```

---

## 7. 故障语义

| 故障 | 检测 | 动作 |
| --- | --- | --- |
| `/joint_states` 超时 | age > state_timeout | 拒绝新动作，取消 active |
| 策略断流 | watchdog 超时 | 受控停止（状态可信时） |
| 动作过期 | age > max_action_age | 丢弃，不压缩执行 |
| 位置越界 | 软限位 | clamp 或拒绝，记录日志 |
| 速度超限 | 相邻点差分 | 时间缩放或拒绝 |
| 跟随误差过大 | JTC/驱动反馈 | 停止并报错 |
| 通信丢失 | 驱动心跳 | 硬件层停止 |
| 急停按下 | 硬件信号 | 断使能/抱闸 |

---

## 8. 启动与关闭顺序

启动：

```text
1. 确认急停可用、工作区无人
2. 上电，释放抱闸
3. 启动 Franka 驱动/FCI
4. 启动 robot_state_publisher
5. 启动 joint_state_broadcaster
6. 启动 joint_trajectory_controller
7. 确认 /joint_states 频率和数值正常
8. 启动 safety_filter、safe_executor
9. 单关节低速测试
10. 再允许 policy 发动作
```

关闭：

```text
1. 停止 policy
2. 停止执行器
3. 控制器保持/下使能
4. 抱闸
5. 关闭驱动
6. 断电
```

---

## 9. 与现有执行栈的映射

```text
policy / VLA
    ↓ JointTrajectory
safety_filter
    ↓
safe_executor
    ↓ FollowJointTrajectory
joint_trajectory_controller
    ↓ command interface
Franka hardware adapter / FCI
    ↑ state interface
/joint_states
```

现有代码需要改的地方：

```text
safety_filter：2 关节限位 → 7 关节 Panda 限位
mock_policy：2 维目标 → 7 维目标
queue_probe / stop_probe：2 关节 → 7 关节
JTC 配置：joints 列表改为 panda_joint1..7
safe_executor：action_name 按实验室控制器名配置
```

---

## 10. 测试阶梯

```text
1. mock hardware：7 关节、接口、RViz
2. fake hardware plugin：延迟、噪声、限位、故障注入
3. Gazebo：物理、碰撞、重力
4. 真机空载：低速、单关节、小角度
5. 真机带载：额定负载内
6. 任务测试：抓取/放置
```

每一级通过条件：

```text
动作能执行
状态频率稳定
超时能停止
取消能停止
没有未命令运动
停止距离/时间在合同范围内
```

---

## 11. 待确认

```text
1. 实验室实际是 Panda 4.2 还是 FR3？
2. 使用官方 franka_ros2 还是实验室自有驱动？
3. 驱动的 command interface 是 position 还是 effort？
4. FCI 版本和实时内核要求？
5. 夹爪是 Franka Hand 还是其他工具？
6. 急停/STO/抱闸的硬件接线和责任人？
7. 实际负载、工作空间和速度限制？
```

---

## 12. 参考

- Franka Research 3 数据手册（Panda 平台规格）：
  https://download.franka.de/documents/120020_Datasheet%20Franka%20Research%203_1.2_EN.pdf
- Franka Panda URDF 关节限制：
  https://raw.githubusercontent.com/Daniella1/urdf_files_dataset/main/urdf_files/matlab/franka_description/robots/frankaEmikaPanda.urdf
- RoboDK Franka Emika Panda 规格：
  https://robodk.com/robot/Franka/Emika-Panda
