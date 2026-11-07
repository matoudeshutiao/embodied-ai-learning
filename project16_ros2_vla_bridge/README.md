# 项目 16：VLA / 真机桥接与真控制器

> 状态：✅ 已在 VM 中跑通。Gazebo 关节由真控制器实际驱动。
>
> 时间：2026.11.05 ｜ 第 16 天 ｜ ROS2 周 Day 8
>
> 环境：Ubuntu 22.04.5 + ROS2 Humble + Gazebo Classic + ros2_control
>
> 目标：把第 15 天的 mock action server 替换成 Gazebo 里真正的
> `joint_trajectory_controller`，让策略动作真正驱动关节。

---

## 1. 今天换的是什么

第 15 天的执行链最后一段是模拟的：

```text
Mock Policy
    ↓ /policy/action_raw
Safety Filter
    ↓ /policy/action_safe
Safe Executor
    ↓ /task/execute_trajectory
Mock Trajectory Server
```

`mock_trajectory_server` 收到轨迹后只打印日志并返回成功，关节不会动，
`/joint_states` 也不会变化。

今天把最后一段换成：

```text
Safe Executor
    ↓ /joint_trajectory_controller/follow_joint_trajectory
joint_trajectory_controller
    ↓ ros2_control
Gazebo 关节
    ↓
/joint_states → TF → RViz
```

### mock 和真控制器的区别

| 对比项 | mock action server | joint_trajectory_controller |
| --- | --- | --- |
| 接收轨迹 | ✅ | ✅ |
| 驱动关节 | ❌ | ✅ |
| 返回成功 | 立刻返回假成功 | 执行完成后返回 |
| 真实关节状态 | ❌ | ✅ `/joint_states` 更新 |
| 上真机时 | 不能直接复用 | 换硬件接口即可复用 |

注意：

```text
今天换的是“Gazebo 里的真控制器”
不是“物理机械臂”
```

真机链路里，控制器仍然是 `joint_trajectory_controller`，只需要把
`gazebo_ros2_control/GazeboSystem` 换成厂商的 ros2_control 硬件插件。

---

## 2. 今天的数据流

```text
mock_policy
    ↓ /policy/action_raw（1 秒 5 点动作块）
safety_filter
    ↓ 位置限位 + 速度时间缩放
    ↓ /policy/action_safe
safe_executor
    ↓ FollowJointTrajectory action
joint_trajectory_controller
    ↓ 插值 + 位置命令
ros2_control / GazeboSystem
    ↓
Gazebo 关节真实运动
    ↓
joint_state_broadcaster
    ↓ /joint_states
robot_state_publisher
    ↓ /tf
RViz
```

## 2.1 实测结果

![Gazebo 与 RViz 中的真控制器执行](assets/real_controller.png)

截图同时包含：

```text
Gazebo 中的 simple_arm
RViz 中的同姿态模型
joint_state_broadcaster active
joint_trajectory_controller active
action client /safe_executor
action server /joint_trajectory_controller
```

控制器日志确认轨迹真的执行完成：

```text
[joint_trajectory_controller]: Received new action goal
[joint_trajectory_controller]: Accepted new action goal
[safe_executor]: action goal accepted
[joint_trajectory_controller]: Goal reached, success!
[safe_executor]: action finished: code=0, message=Goal successfully reached!
```

安全层持续输出：

```text
data: 'ok: 5 points'
```

观测到 Gazebo 和 RViz 中的手臂都在持续摆动，`/joint_states` 同步变化。

---

## 3. 代码结构

```text
project16_ros2_vla_bridge/
├── README.md
├── assets/
│   └── action_chain_logs.png       # 第 15 天 mock 链路日志
└── ros2_ws/
    └── src/
        └── vla_bridge/
            ├── package.xml
            ├── setup.py
            ├── setup.cfg
            ├── resource/vla_bridge
            ├── launch/
            │   └── real_controller_demo.launch.py
            ├── rviz/
            │   └── real_controller_view.rviz
            └── vla_bridge/
                ├── __init__.py
                ├── mock_policy.py
                ├── safety_filter.py
                ├── mock_trajectory_server.py
                └── safe_executor.py
```

| 文件 | 作用 |
| --- | --- |
| `mock_policy.py` | 模拟 VLA 输出动作块 |
| `safety_filter.py` | 关节名校验、位置 clamp、速度时间缩放、watchdog |
| `safe_executor.py` | 把安全动作发给可配置的 action server |
| `mock_trajectory_server.py` | 第 15 天的假执行器，保留作对照 |
| `real_controller_demo.launch.py` | 启动 Gazebo、控制器、桥接节点和 RViz |
| `real_controller_view.rviz` | 默认显示 RobotModel、TF 和 Grid |

---

## 4. 今天的关键改动

### 4.1 `safe_executor` 可切换执行器

`safe_executor` 新增参数：

```text
action_name
```

默认值：

```text
/joint_trajectory_controller/follow_joint_trajectory
```

所以默认就连真控制器。

想回到第 15 天的 mock 版本时，不需要改代码：

```bash
ros2 run vla_bridge safe_executor --ros-args -p action_name:=/task/execute_trajectory
```

### 4.2 mock policy 的动作块变短

原来的动作块是 5 点、跨 2 秒，而执行器每 1 秒发一次，容易重叠。

现在改成：

```text
5 个点，点间隔 0.2 秒，总长 1 秒
每 1.2 秒发一次
```

给控制器留 0.2 秒收尾时间，Gazebo 里运动更连续。

### 4.3 launch 自动串联启动顺序

`real_controller_demo.launch.py` 按顺序启动：

```text
Gazebo
→ robot_state_publisher
→ spawn_entity
→ joint_state_broadcaster
→ joint_trajectory_controller
→ safety_filter
→ mock_policy
→ safe_executor
→ RViz
```

控制器没有 active 之前，执行器不会发动作。

### 4.4 默认可视化

RViz 配置文件已经包含：

```text
RobotModel
TF
Grid
Fixed Frame: base_link
```

不需要手动 Add display。

---

## 5. 在虚拟机中同步代码

推荐用 Git，不要直接在 hgfs 共享目录里编译。

如果虚拟机还没有仓库：

```bash
git clone https://github.com/matoudeshutiao/embodied-ai-learning.git ~/embodied-ai-learning
```

如果已经有仓库：

```bash
cd ~/embodied-ai-learning
```

```bash
git pull
```

然后把包复制到 ROS2 工作空间：

```bash
mkdir -p ~/ros2_ws/src/vla_bridge
```

```bash
cp -r ~/embodied-ai-learning/project16_ros2_vla_bridge/ros2_ws/src/vla_bridge/. ~/ros2_ws/src/vla_bridge/
```

如果 `~/ros2_ws/src` 里还没有 `arm_description`：

```bash
mkdir -p ~/ros2_ws/src/arm_description
```

```bash
cp -r ~/embodied-ai-learning/project12_ros2_gazebo/ros2_ws/src/arm_description/. ~/ros2_ws/src/arm_description/
```

---

## 6. 编译

```bash
cd ~/ros2_ws
```

```bash
source /opt/ros/humble/setup.bash
```

```bash
colcon build --symlink-install --packages-select arm_description vla_bridge
```

```bash
source install/setup.bash
```

---

## 7. 启动

```bash
ros2 launch vla_bridge real_controller_demo.launch.py
```

这个终端保持运行。

预期看到：

```text
Gazebo 打开并生成 simple_arm
RViz 自动打开并显示模型
joint_trajectory_controller active
机械臂缓慢摆动
```

---

## 8. 验证

另开终端：

```bash
source /opt/ros/humble/setup.bash
```

```bash
source ~/ros2_ws/install/setup.bash
```

查看控制器：

```bash
ros2 control list_controllers
```

期望：

```text
joint_state_broadcaster      active
joint_trajectory_controller  active
```

查看 action：

```bash
ros2 action info /joint_trajectory_controller/follow_joint_trajectory
```

期望 client：

```text
/safe_executor
```

查看关节状态：

```bash
ros2 topic echo /joint_states --once
```

多次执行，`joint1`、`joint2` 应该变化。

`safe_executor` 终端应出现：

```text
action goal accepted
action finished: code=0
```

---

## 9. 常见问题

### 9.1 `Package 'vla_bridge' not found`

新终端没有 source：

```bash
source /opt/ros/humble/setup.bash
```

```bash
source ~/ros2_ws/install/setup.bash
```

### 9.2 控制器不是 active

先确认 Gazebo 已经生成模型，再检查：

```bash
ros2 control list_controllers
```

必要时重启 launch。

### 9.3 机械臂不动

检查：

```text
/joint_trajectory_controller/follow_joint_trajectory 是否存在
safe_executor 是否连的是这个 action
安全层是否在发 /policy/action_safe
```

### 9.4 RViz 里没有模型

检查 RobotModel 的 Topic 是否为：

```text
/robot_description
```

Durability Policy 是否为：

```text
Transient Local
```

---

## 10. 今天的局限

```text
驱动的是 Gazebo 仿真关节，不是物理机械臂
policy 仍然是 mock，不读取图像
没有执行反馈回到 policy
safe_executor 执行期间会丢弃新动作，还没有动作队列
watchdog 目前只报警，不会自动保持位置
```

所以今天完成的是：

```text
从 mock 执行器 → 真 ros2_control 控制器
```

还不是完整 VLA 闭环。

---

## 11. 下一步

```text
1. 给 safe_executor 加动作队列/最新动作策略
2. 让 watchdog 在策略断流时发送保持位置轨迹
3. 把 mock_policy 换成读取图像的策略服务
4. 用真实相机和真机硬件接口替换 Gazebo
5. 采集 image + state + action，形成训练数据
```

---

## 12. 代码逐段解析

今天的关键代码、每段的作用、以及和第 15 天的逐项对比，见：

**[CODE_WALKTHROUGH.md](CODE_WALKTHROUGH.md)**

包含：

```text
safe_executor.py 的 action 参数与回调
mock_policy.py 的动作块
safety_filter.py 的限位/速度缩放/watchdog
real_controller_demo.launch.py 的启动顺序
第 15 天 vs 第 16 天对比
```

---

## 13. 真机准备周路线

第 17 天的 ROS2 接口盘点、仿真/真机替换表、一周学习路径见：

**[REAL_ROBOT_ROADMAP.md](REAL_ROBOT_ROADMAP.md)**

---

## 14. 第 18 天：动作队列

> 状态：✅ 已在 VM 中实测。队列串行执行、满时丢最旧未执行块。

### 14.1 为什么要队列

原来的 `safe_executor`：

```text
正在执行一条轨迹
    ↓
新动作到来
    ↓
busy=True → 直接 return
```

结果是：

```text
动作被丢弃
策略发得越快，丢得越多
真机上可能出现动作不连续
```

### 14.2 队列语义

```text
当前正在执行的块：不打断
还没执行的块：放进 pending 队列
队列满：丢掉最旧的 pending 块
当前块执行完：从队列取下一块
```

默认队列长度：

```text
max_queue_size = 2
```

### 14.3 关键代码

```python
from collections import deque

self.declare_parameter("max_queue_size", 2)
self.max_queue_size = max(
    1,
    int(self.get_parameter("max_queue_size").value),
)

self.pending = deque()
self.busy = False
```

收到新动作：

```python
if len(self.pending) >= self.max_queue_size:
    dropped = self.pending.popleft()
    self.dropped_count += 1
    self.get_logger().warn(
        f"queue full, dropped oldest pending action "
        f"({len(dropped.points)} points)"
    )

self.pending.append(msg)
self.queued_count += 1
self.get_logger().info(
    f"queued action: {len(msg.points)} points, "
    f"pending={len(self.pending)}, busy={self.busy}"
)
self.start_next()
```

取下一个执行：

```python
def start_next(self):
    if self.busy or not self.pending:
        return

    msg = self.pending.popleft()
    self.busy = True
    goal = FollowJointTrajectory.Goal()
    goal.trajectory = msg
```

执行结束后：

```python
self.busy = False
self.start_next()
```

### 14.4 关键设计

```text
只丢“未执行”的旧动作
不取消正在执行的轨迹
```

术语要分清：

```text
pending     = 还没开始执行的动作块
active      = 当前正在执行的那一条
max_queue_size 只限制 pending，不包含 active
```

所以“完整重规划时清空 pending”指的是：

```text
清空还没开始执行的旧计划
```

它不包含 active。  
如果连 active 也要放弃，必须另做“取消当前 goal / 受控停止”，不能只靠 `pending.clear()`。

因为当前动作块是顺序拼接的：

```text
chunk 1 → chunk 2 → chunk 3
```

如果直接取消正在执行的轨迹并切换，会造成：

```text
速度突变
跟踪误差
保护性停机
碰撞风险
```

以后接真正 VLA 的“完整重规划”时，再增加：

```text
清空 pending 队列，执行最新计划
```

### 14.5 实测日志（✅）

压力测试时实际看到：

```text
[mock_policy]: published action chunk: 5 points
[safe_executor]: queued action: 5 points, pending=2, busy=True

[mock_policy]: published action chunk: 5 points
[safe_executor]: queue full, dropped oldest pending action (5 points)
[safe_executor]: queued action: 5 points, pending=2, busy=True

[joint_trajectory_controller]: Goal reached, success!
[safe_executor]: action finished: code=0, message=Goal successfully reached!

[safe_executor]: stats: queued=741, executed=724, dropped=14
```

这组日志同时证明了四件事：

```text
1. 当前有一条轨迹正在执行：busy=True
2. 队列已经存了 2 条：pending=2
3. 队列满后丢弃的是最旧的未执行块
4. 控制器真实执行完成：Goal reached + code=0
```

### 14.6 统计数字怎么读

```text
queued   = 741
executed = 724
dropped  = 14
```

看起来少了一部分：

```text
741 - 724 - 14 = 3
```

这 3 条就是日志打印时仍然在途的动作：

```text
1 条正在执行
2 条还在 pending 队列
```

所以：

```text
queued = executed + dropped + in_flight
```

数字是自节点启动以来的累计值，不是这一次压力测试的 5 条。

### 14.7 真机启示：队列会带来延迟

默认 `max_queue_size = 2`，意味着最坏情况下：

```text
1 条正在执行
+ 2 条等待执行
```

如果每条动作块约 1 秒，机器人可能在新观测之后最多执行约 2～3 秒前的旧计划。

仿真里只是“卡一下”，真机上可能是：

```text
对移动目标反应变慢
避障不及时
动作和当前画面不一致
```

所以真机不能只看“队列不丢动作”，还要看：

```text
队列深度
动作年龄
观测到执行的总延迟
```

下一步应增加：

```text
动作时间戳/年龄检查
队列深度监控
必要时把 max_queue_size 降到 1
```

### 14.8 当前局限

```text
没有动作时间戳/年龄检查
没有状态反馈监控
watchdog 仍然只报警
没有“完整重规划清空队列”模式
```

### 14.9 隔离队列测试

之前的压力测试同时有 `mock_policy` 和测试 publisher 发动作，结果不干净。

现在 launch 支持关闭 `mock_policy`：

```bash
ros2 launch vla_bridge real_controller_demo.launch.py start_mock_policy:=false
```

另开终端，只运行一个测试发布者：

```bash
ros2 run vla_bridge queue_probe
```

`queue_probe` 会：

```text
每 0.25 秒发一条短轨迹
每条轨迹目标不同
每条轨迹时长 0.6 秒
共发 8 条
```

这样观察：

```text
robot 是否连续向不同目标运动
safe_executor 是否出现 queued/dropped
队列满时是否丢弃未执行的旧块
```

预期日志：

```text
queued action: 1 points, pending=...
queue full, dropped oldest pending action (1 points)
stats: queued=..., executed=..., dropped=...
```

隔离测试的意义：

```text
只有一个动作源
目标会变化
能区分“队列卡顿”和“目标原地保持”
```

---

## 附录：第 15 天的 mock 链路

第 15 天的完整日志仍保留在：

```text
assets/action_chain_logs.png
```

当时的链路：

```text
mock_policy → safety_filter → safe_executor
→ mock_trajectory_server
```

今天保留 `mock_trajectory_server.py`，用于对照和回归测试。
