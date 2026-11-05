# 第 16 天关键代码解析：mock 执行器 → Gazebo 真控制器

> 对应提交：`b3997b6`、`1016fa5`
>
> 目标：看懂今天每一段代码在链路里的位置，以及和第 15 天到底差在哪。

---

## 1. 今天代码的整体位置

```text
mock_policy.py
    │ 发布 /policy/action_raw
    ▼
safety_filter.py
    │ 位置限位、速度时间缩放、watchdog
    │ 发布 /policy/action_safe
    ▼
safe_executor.py
    │ 发送 FollowJointTrajectory action
    ▼
joint_trajectory_controller
    │ ros2_control
    ▼
Gazebo 关节
    │
    ▼
/joint_states → robot_state_publisher → /tf → RViz
```

今天没有改感知和 MoveIt2，重点是：

```text
让动作从“发给假执行器”
变成“发给 ros2_control 真控制器”
```

---

## 2. `safe_executor.py`：把动作交给谁

### 2.1 关键代码

```python
class SafeExecutor(Node):
    def __init__(self):
        super().__init__("safe_executor")

        # 默认接 Gazebo 里的 ros2_control 控制器。
        # 想回到昨天的 mock 执行器时，用参数覆盖即可。
        self.declare_parameter(
            "action_name",
            "/joint_trajectory_controller/follow_joint_trajectory",
        )
        self.action_name = self.get_parameter("action_name").value

        self.client = ActionClient(
            self,
            FollowJointTrajectory,
            self.action_name,
        )

        self.sub = self.create_subscription(
            JointTrajectory,
            "/policy/action_safe",
            self.on_safe_action,
            10,
        )

        # 一次只执行一条轨迹；执行期间的新动作先丢弃。
        self.busy = False
        self.client.wait_for_server()
```

### 2.2 各部分的含义

| 代码 | 作用 |
| --- | --- |
| `declare_parameter("action_name", ...)` | 把“执行器地址”变成可配置参数 |
| `ActionClient(...)` | 创建一个 action 客户端，不是 topic 发布者 |
| `FollowJointTrajectory` | ROS2 标准的关节轨迹 action 类型 |
| `create_subscription(...)` | 接收安全层处理后的轨迹 |
| `wait_for_server()` | 等待控制器 action server 上线 |
| `self.busy` | 防止同时发送多条轨迹 |

### 2.3 发送轨迹

```python
def on_safe_action(self, msg):
    if self.busy:
        return
    if not self.client.server_is_ready():
        return

    self.busy = True
    goal = FollowJointTrajectory.Goal()
    goal.trajectory = msg

    future = self.client.send_goal_async(
        goal,
        feedback_callback=self.on_feedback,
    )
    future.add_done_callback(self.on_goal_response)
```

这里的 `goal` 不是普通消息，而是一次“执行请求”：

```text
goal.trajectory = 安全层输出的 JointTrajectory
```

控制器收到后，会按 `time_from_start` 对轨迹插值并驱动关节。

### 2.4 接收执行结果

```python
def on_goal_response(self, future):
    goal_handle = future.result()
    if not goal_handle.accepted:
        self.get_logger().warn("action goal rejected")
        self.busy = False
        return

    result_future = goal_handle.get_result_async()
    result_future.add_done_callback(self.on_result)

def on_result(self, future):
    result = future.result().result
    self.get_logger().info(
        f"action finished: code={result.error_code}, "
        f"message={result.error_string}"
    )
    self.busy = False
```

`error_code=0` 的含义是：

```text
控制器认为轨迹已经成功执行完成
```

今天日志中的：

```text
action finished: code=0, message=Goal successfully reached!
```

就是真控制器返回的，不是 mock 假成功。

---

## 3. `mock_policy.py`：策略输出什么

### 3.1 关键代码

```python
self.timer = self.create_timer(1.2, self.tick)

def tick(self):
    traj = JointTrajectory()
    traj.header.stamp = self.get_clock().now().to_msg()
    traj.joint_names = ["joint1", "joint2"]

    for i in range(5):
        point = JointTrajectoryPoint()
        t = (i + 1) * 0.2

        point.positions = [
            0.3 * math.sin(self.t + t),
            -0.2 * math.cos(self.t + t),
        ]
        point.velocities = [0.0, 0.0]
        point.time_from_start = Duration(
            sec=int(t),
            nanosec=int((t % 1.0) * 1e9),
        )
        traj.points.append(point)

    self.pub.publish(traj)
    self.t += 1.0
```

### 3.2 这是什么

一次 `tick()` 发布一个 **action chunk**：

```text
5 个动作点
每个点间隔 0.2 秒
总时长 1 秒
每 1.2 秒发布一次
```

可以理解成：

```text
策略一次推理，输出未来 1 秒的关节位置
```

### 3.3 为什么改成 1.2 秒

昨天的节奏是：

```text
5 个点 × 0.4 秒 = 2 秒轨迹
每 1 秒发送一次
```

这会导致：

```text
第 1 个动作块还没执行完
第 2 个动作块已经进来
```

mock server 不在乎，因为它根本不执行；但真控制器会真的按时间走，
重叠的动作块会造成丢弃或轨迹不连续。

今天改成：

```text
1 秒动作块 + 1.2 秒发送周期
```

给控制器留 0.2 秒收尾。

---

## 4. `safety_filter.py`：在进入控制器前拦住危险动作

### 4.1 位置限位

```python
lower, upper = self.position_limits[name]
clamped = min(max(position, lower), upper)
```

含义：

```text
position < lower → lower
position > upper → upper
否则保持不变
```

例如：

```text
[5.0, -8.0] → [2.967, -2.0]
```

注意：

```text
软件 clamp ≠ 硬件急停
```

真机仍然需要机械限位、急停和安全工作空间。

### 4.2 速度限制：不改路径，只改时间

```python
velocity = abs(
    point.positions[index] - previous_positions[index]
) / dt

ratio = velocity / self.velocity_limits[name]
```

如果计算出的速度超过限制，安全层会缩放整条轨迹的时间轴：

```python
scaled = self.duration_to_sec(point.time_from_start) * max_ratio
point.time_from_start = self.sec_to_duration(scaled)
```

例如：

```text
原速度 5 rad/s，限制 1.5 rad/s
ratio = 5 / 1.5 = 3.33
```

结果：

```text
路径不变
整条轨迹慢 3.33 倍
```

这比直接截断速度安全，因为轨迹形状和终点都保留。

### 4.3 watchdog

```python
if elapsed > 2.0 and not self.watchdog_active:
    self.watchdog_active = True
    self.get_logger().warn("watchdog: no new action")
    self.publish_status("watchdog: no new action")
```

含义：

```text
策略超过 2 秒没有发新动作 → 报警
```

当前局限：

```text
只报警
不会自动发送“保持当前位置”的轨迹
```

这是下一步要补的安全功能。

---

## 5. `real_controller_demo.launch.py`：自动按顺序启动

### 5.1 展开 URDF

```python
robot_description = ParameterValue(
    Command([FindExecutable(name="xacro"), " ", xacro_file]),
    value_type=str,
)
```

作用：

```text
xacro 文件 → URDF 字符串 → robot_state_publisher
```

### 5.2 启动控制器和桥接节点

```python
joint_state_broadcaster = Node(
    package="controller_manager",
    executable="spawner",
    arguments=["joint_state_broadcaster"],
)

joint_trajectory_controller = Node(
    package="controller_manager",
    executable="spawner",
    arguments=["joint_trajectory_controller"],
)

safe_executor = Node(
    package="vla_bridge",
    executable="safe_executor",
    parameters=[{
        "action_name": (
            "/joint_trajectory_controller/"
            "follow_joint_trajectory"
        ),
    }],
)
```

### 5.3 用进程退出事件控制顺序

```python
start_jsb = RegisterEventHandler(
    OnProcessExit(
        target_action=spawn_entity,
        on_exit=[
            TimerAction(
                period=1.0,
                actions=[joint_state_broadcaster],
            )
        ],
    )
)
```

启动顺序：

```text
Gazebo
→ robot_state_publisher
→ spawn_entity 生成模型
→ joint_state_broadcaster
→ joint_trajectory_controller
→ safety_filter / mock_policy / safe_executor / RViz
```

这样做是为了避免：

```text
Gazebo 还没准备好
控制器就先启动
action server 不存在
safe_executor 一直等待
```

---

## 6. 今天和昨天到底差在哪

| 项目 | 第 15 天 | 第 16 天 |
| --- | --- | --- |
| 执行器 | `mock_trajectory_server` | `joint_trajectory_controller` |
| action 地址 | 固定 `/task/execute_trajectory` | 参数化，默认真控制器 |
| 关节是否运动 | ❌ 不动 | ✅ Gazebo 真实运动 |
| 成功含义 | 假成功 | 控制器 `Goal reached` |
| 关节状态 | 不更新 | `/joint_states` 持续变化 |
| 动作块 | 5 点 / 2 秒，1 秒发一次 | 5 点 / 1 秒，1.2 秒发一次 |
| 启动方式 | 4 个终端手动启动 | 一条 launch 自动启动 |
| 可视化 | 手动配置 | 默认 RViz 配置 |
| 验证 | action 返回 code=0 | Gazebo + RViz + joint_states 同步 |

### 最重要的一句话

```text
昨天证明了“命令能走到执行接口”
今天证明了“命令能让关节真的动”
```

---

## 7. 今天还没解决什么

```text
policy 仍然是 mock，不读图像
没有执行反馈回到 policy
safe_executor 执行期间会丢新动作
watchdog 只报警，不会主动保持位置
驱动的是 Gazebo，不是物理机械臂
```

所以今天完成的是：

```text
真控制器执行链
```

不是：

```text
完整 VLA 闭环
```

---

## 8. 下一步代码方向

```text
1. safe_executor 加动作队列
2. watchdog 断流时保持当前位置
3. policy 读取相机，形成仿真闭环
4. 真机硬件接口替换 GazeboSystem
5. 采集 image + state + action 数据
```
