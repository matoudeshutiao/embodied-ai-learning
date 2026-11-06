# 真机准备周：ROS2 执行栈盘点与路线图

> 时间：2026.11.06 ｜ 第 17 天 ｜ ROS2 周 Day 9
>
> 目标：从“仿真能跑”走向“真机可替换”，先把接口边界看清楚。

---

## 1. 这一周要解决什么

上一阶段完成的是：

```text
感知 → 策略 → 安全 → 执行器 → Gazebo 控制器
```

下一阶段不是继续堆 demo，而是把这条链变成：

```text
感知 → 策略 → 安全 → 执行器
→ joint_trajectory_controller
→ 真机硬件接口
→ 真实关节
```

核心原则：

```text
上层接口尽量不变
只替换底层硬件
```

真正需要替换的通常只有：

```text
GazeboSystem → 厂商 ros2_control 硬件插件
```

---

## 2. 一周学习路径

每天约 1 小时：

```text
10 分钟：回顾昨天
15 分钟：理解概念/画图
25 分钟：动手验证或改代码
10 分钟：写结论、更新 GitHub
```

| 天数 | 主题 | 产出 | 和真机的关系 |
| --- | --- | --- | --- |
| Day&nbsp;17 | 执行栈盘点与 action 语义 | 仿真/真机接口替换表 | 知道换哪里 |
| Day&nbsp;18 | 动作队列 | `safe_executor` 连续执行动作块 | 真机动作不丢块 |
| Day&nbsp;19 | watchdog 与取消 | 状态超时、保持位置、取消轨迹 | 真机失效保护 |
| Day&nbsp;20 | ros2_control 硬件接口 | 读懂 command/state interface | 换硬件不换上层 |
| Day&nbsp;21 | 反馈与监控 | 跟踪误差、控制器状态、异常检测 | 真机状态监控 |
| Day&nbsp;22 | 相机、TF、标定与数据 | 相机 TF 图、数据记录格式 | 真机感知与采数 |
| Day&nbsp;23 | 真机 dry run + 复盘 | 仿真中模拟真机限制、老师问题清单 | 为真机做准备 |

---

## 3. Day 17 实测：当前 ROS2 接口

### 3.1 节点

```text
/controller_manager
/gazebo
/gazebo_ros2_control
/joint_state_broadcaster
/joint_trajectory_controller
/mock_policy
/robot_state_publisher
/rviz
/safe_executor
/safety_filter
/transform_listener_impl_...
```

`transform_listener_impl_...` 是 TF 监听器的内部节点，不是需要手动管理的业务节点。

### 3.2 Action

```text
/joint_trajectory_controller/follow_joint_trajectory
[control_msgs/action/FollowJointTrajectory]
```

这是当前唯一的核心执行 action。

### 3.3 关键 topic

```text
/clock [rosgraph_msgs/msg/Clock]
/dynamic_joint_states [control_msgs/msg/DynamicJointState]
/joint_states [sensor_msgs/msg/JointState]
/joint_trajectory_controller/controller_state [control_msgs/msg/JointTrajectoryControllerState]
/joint_trajectory_controller/joint_trajectory [trajectory_msgs/msg/JointTrajectory]
/joint_trajectory_controller/state [control_msgs/msg/JointTrajectoryControllerState]
/policy/action_raw [trajectory_msgs/msg/JointTrajectory]
/policy/action_safe [trajectory_msgs/msg/JointTrajectory]
/policy/safety_status [std_msgs/msg/String]
/robot_description [std_msgs/msg/String]
/tf [tf2_msgs/msg/TFMessage]
/tf_static [tf2_msgs/msg/TFMessage]
```

### 3.4 硬件接口

```text
command interfaces
    joint1/position [available] [claimed]
    joint2/position [available] [claimed]

state interfaces
    joint1/position
    joint1/velocity
    joint2/position
    joint2/velocity
```

含义：

```text
claimed：已经被 joint_trajectory_controller 使用
command：控制器可以向关节写位置
state：广播器/控制器可以读位置和速度
```

### 3.5 控制器

```text
joint_state_broadcaster      active
joint_trajectory_controller  active
```

---

## 4. 仿真专用 vs 真机可复用

| 接口/节点 | 作用 | 仿真专用？ | 真机替换/处理 |
| --- | --- | --- | --- |
| `/gazebo` | Gazebo ROS 集成 | ✅ | 移除，换成真实机器人环境 |
| `/gazebo_ros2_control` | Gazebo 硬件接口插件 | ✅ | 换成厂商 ros2_control 插件 |
| `GazeboSystem` | 仿真关节硬件抽象 | ✅ | 换成真机硬件接口 |
| `/clock` | Gazebo 仿真时间 | ✅ | 真机关闭，使用系统时间 |
| `spawn_entity.py` | 把模型生成到 Gazebo | ✅ | 真机不需要 |
| `/controller_manager` | 加载/激活控制器 | ❌ | 保留 |
| `joint_state_broadcaster` | 发布 `/joint_states` | ❌ | 保留 |
| `joint_trajectory_controller` | 执行关节轨迹 | ❌ | 保留 |
| `FollowJointTrajectory` action | 轨迹执行接口 | ❌ | 保留 |
| `/joint_states` | 关节位置/速度 | ❌ | 保留，由真机编码器提供 |
| `/robot_description` | URDF 模型 | ❌ | 保留，换成真机 URDF |
| `/tf`、`/tf_static` | 坐标变换 | ❌ | 保留 |
| `robot_state_publisher` | URDF + 关节状态 → TF | ❌ | 保留 |
| `/policy/*` | 策略与安全层接口 | ❌ | 保留 |
| `safety_filter` | 位置/速度/watchdog | ❌ | 保留，但参数要按真机重调 |
| `safe_executor` | 发 FollowJointTrajectory | ❌ | 保留，增加队列和反馈 |
| RViz | 可视化调试 | ❌ | 可保留，作为调试工具 |

一句话：

```text
Gazebo 是“硬件替身”
ros2_control 控制器是“上层不变的部分”
```

---

## 5. 三个真机风险

### 5.1 状态会过期

如果 `joint_state_broadcaster` 停止：

```text
controller 可能还在执行
/joint_states 停止更新
RViz 不再更新
safety_filter 还保留最后一次关节位置
```

危险点不是“完全没数据”，而是：

```text
拿旧数据继续做安全判断
```

真机必须增加：

```text
状态超时检测
超时后拒绝新动作或进入安全状态
```

### 5.2 时间会失真

真机没有 Gazebo 的 `/clock`。

如果仍然：

```text
use_sim_time = true
```

可能出现：

```text
节点时钟不走
watchdog 不触发
TF 时间戳异常
RViz extrapolation
```

真机应设置：

```text
use_sim_time = false
```

### 5.3 动作会抢占

当前 `safe_executor` 在忙时会丢弃新动作。

如果改成“取消当前轨迹，立刻执行新轨迹”，真机风险包括：

```text
速度突变
跟踪误差
保护性停机
未到达安全中间点就转向
碰撞风险
```

更合理的策略：

```text
小容量队列
顺序执行
队列满时丢弃未执行的旧动作
紧急停止使用独立安全链
```

这里是 Day 18 要动手改的地方。

---

## 6. Day 17 结论

今天确认了：

```text
控制器的执行链是通的
硬件接口是 position command + position/velocity state
大部分上层接口可以原样迁移到真机
真正需要替换的是 Gazebo 硬件层和时间源
```

明天开始改执行层：

```text
1. safe_executor 增加动作队列
2. 不再因为 busy 直接丢动作
3. 记录 queued / dropped / executed
4. 用日志验证动作块连续执行
```

---

## 7. 找老师时可以问的问题

```text
1. 机械臂型号是什么？
2. 有没有 ROS2 驱动或 ros2_control 硬件插件？
3. 控制接口是位置、速度还是力矩？
4. 控制器更新频率是多少？
5. 编码器状态如何发布？
6. 有没有急停、限位和工作空间限制？
7. 相机型号、安装方式和标定文件？
8. 是否允许采集遥操作数据？
9. 实验室是否有 GPU 做推理？
10. 真机首次上电的流程和安全规范是什么？
```
