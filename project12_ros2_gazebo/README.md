# 项目 12：Gazebo + ros2_control

> 状态：✅ 已跑通。
>
> 时间：2026.10.31 ｜ 第 11 天 ｜ ROS2 周 Day 3
>
> 环境：Ubuntu 22.04.5 + ROS2 Humble + Gazebo Classic 11（VMware 虚拟机）
>
> 目标：让 URDF 里的关节从“滑条假数据”变成“物理仿真 + 控制器真正驱动”。

第 10 天解决的是“机器人在哪”：

```text
URDF + /joint_states  →  TF 树  →  RViz
```

今天解决的是“机器人怎么被控制”：

```text
轨迹控制器 → ros2_control → Gazebo 物理引擎 → 关节状态 → TF/RViz
```

---

## 1. 今天的数据流

```text
joint_trajectory_controller
        │  position command
        ▼
controller_manager
        │
        ▼
ros2_control hardware interface
        │
        ▼
Gazebo joints（重力、碰撞、惯性）
        │  actual state
        ▼
joint_state_broadcaster
        │  /joint_states
        ▼
robot_state_publisher
        │  /tf
        ▼
RViz
```

这条链里任何一段断了，都会出现不同症状：

| 断掉的位置 | 现象 |
| --- | --- |
| Gazebo 没加载 ROS 插件 | 没有 `/gazebo`、`/spawn_entity` |
| 没有 broadcaster | Gazebo 可能动，但 `/joint_states` 和 TF 不更新 |
| 没有 trajectory controller | 关节没有位置命令，可能被重力拉下去 |
| RSP 没启动 | 有 `/joint_states`，但没有 `/tf` |
| RViz 配置错了 | Gazebo 动，RViz 不动或固定帧报错 |

---

## 2. 关键概念

### Gazebo 和 RViz 的分工

```text
Gazebo：物理仿真，负责重力、碰撞、惯性、关节动力学
RViz：可视化，负责显示模型、TF、传感器数据
```

Gazebo 里能动，不代表 RViz 里的 TF 一定对；RViz 只看 `/joint_states` 和 `/tf`。

### `spawn_entity.py` 是什么

```text
spawn_entity.py
= 一次性工具，把 URDF 模型生成到正在运行的 Gazebo 世界里
```

它本身不做仿真，只负责：

```text
读取 /robot_description
        ↓
调用 Gazebo 的 /spawn_entity 服务
        ↓
创建 simple_arm 模型
```

所以必须同时满足：

```text
1. Gazebo 里有 /gazebo 节点
2. 有 /spawn_entity 服务
3. robot_state_publisher 发布了 /robot_description
```

三者缺一个，`spawn_entity.py` 就会一直等。

### `ros2_control` 是什么

`ros2_control` 不是某一个控制器，而是一套控制框架：

```text
controller_manager：加载、配置、激活控制器
hardware interface：读写真实/仿真关节
controller：具体控制算法
```

今天使用的两个控制器：

| 控制器 | 作用 |
| --- | --- |
| `joint_state_broadcaster` | 读取关节状态，发布 `/joint_states` |
| `joint_trajectory_controller` | 接收轨迹，向关节写位置命令 |

它们必须分别启动。`controllers.yaml` 只是声明，不会自动把控制器全部拉起来。

### `use_sim_time`

Gazebo 发布 `/clock`。所有依赖时间的节点应该使用：

```text
use_sim_time = true
```

否则 TF 时间戳可能和 Gazebo 对不上，RViz 会报 extrapolation 错误。

### 惯性、碰撞和世界固定

Gazebo 需要：

- `collision`：碰撞几何；
- `inertial`：质量、质心、转动惯量。

只有 `visual` 不够。底座还需要通过：

```text
world → base_link（fixed）
```

固定到世界，否则整条手臂会被重力推倒。

---

## 3. 代码结构

```text
project12_ros2_gazebo/
├── README.md
├── assets/
│   ├── gazebo.png
│   ├── control.png
│   └── gazebo_rviz.png
└── ros2_ws/
    └── src/
        └── arm_description/
            ├── package.xml
            ├── CMakeLists.txt
            ├── config/
            │   └── controllers.yaml
            └── urdf/
                └── simple_arm.urdf.xacro
```

| 文件 | 作用 |
| --- | --- |
| `simple_arm.urdf.xacro` | 机器人结构、惯性、碰撞、`ros2_control` 接口、Gazebo 插件 |
| `controllers.yaml` | controller_manager 更新率、两个控制器的类型和关节配置 |
| `package.xml` | Gazebo、ros2_control、控制器和 RSP 等依赖 |
| `CMakeLists.txt` | 安装 `urdf/` 和 `config/` 到包的 share 目录 |

---

## 4. URDF 相对第 10 天多了什么

### 4.1 每个 link 的惯性

| link | 质量 |
| --- | ---: |
| `base_link` | 2.0 kg |
| `link1` | 1.0 kg |
| `link2` | 0.8 kg |
| `tool0` | 0.1 kg |
| `camera_link` | 0.2 kg |

`inertial` 的 `<origin>` 要和质心位置一致，例如立柱的质心在高度的一半处。

### 4.2 世界固定关节

```xml
<link name="world"/>

<joint name="world_joint" type="fixed">
  <parent link="world"/>
  <child link="base_link"/>
  <origin xyz="0 0 0" rpy="0 0 0"/>
</joint>
```

这样底座固定，只有 `joint1`、`joint2` 会动。

### 4.3 ros2_control 硬件接口

```xml
<ros2_control name="GazeboSystem" type="system">
  <hardware>
    <plugin>gazebo_ros2_control/GazeboSystem</plugin>
  </hardware>

  <joint name="joint1">
    <command_interface name="position"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
  </joint>

  <joint name="joint2">
    <command_interface name="position"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
  </joint>
</ros2_control>
```

含义：

```text
command_interface：控制器能写 position
state_interface：控制器/广播器能读 position、velocity
```

### 4.4 Gazebo 插件

```xml
<gazebo>
  <plugin filename="libgazebo_ros2_control.so" name="gazebo_ros2_control">
    <parameters>$(find arm_description)/config/controllers.yaml</parameters>
  </plugin>
</gazebo>
```

它负责把 Gazebo 的关节包装成 `ros2_control` 硬件接口。

---

## 5. 复现步骤

### 5.1 安装依赖

```bash
sudo apt install -y ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros2-control ros-humble-ros2-control ros-humble-ros2-controllers
```

### 5.2 把包放进工作空间

如果 `~/ros2_ws/src/arm_description` 已经存在（第 10 天的版本），用下面两条命令覆盖文件，避免复制成嵌套目录：

```bash
mkdir -p ~/ros2_ws/src/arm_description
```

```bash
cp -r project12_ros2_gazebo/ros2_ws/src/arm_description/. ~/ros2_ws/src/arm_description/
```

### 5.3 编译

```bash
cd ~/ros2_ws
```

```bash
source /opt/ros/humble/setup.bash
```

```bash
colcon build --symlink-install --packages-select arm_description
```

```bash
source install/setup.bash
```

### 5.4 启动 Gazebo

```bash
ros2 launch gazebo_ros gazebo.launch.py
```

### 5.5 启动 robot_state_publisher

新开终端。

```bash
source /opt/ros/humble/setup.bash
```

```bash
source ~/ros2_ws/install/setup.bash
```

```bash
ros2 run robot_state_publisher robot_state_publisher --ros-args -p use_sim_time:=true -p robot_description:="$(xacro ~/ros2_ws/src/arm_description/urdf/simple_arm.urdf.xacro)"
```

### 5.6 生成模型

新开终端。

```bash
source /opt/ros/humble/setup.bash
```

```bash
source ~/ros2_ws/install/setup.bash
```

```bash
ros2 run gazebo_ros spawn_entity.py -topic robot_description -entity simple_arm -x 0 -y 0 -z 0.0 -timeout 60.0
```

### 5.7 启动控制器

先启动状态广播：

```bash
ros2 run controller_manager spawner joint_state_broadcaster
```

再启动轨迹控制器：

```bash
ros2 run controller_manager spawner joint_trajectory_controller
```

检查：

```bash
ros2 control list_controllers
```

应该看到两个都 active。

### 5.8 发轨迹

```bash
ros2 action send_goal /joint_trajectory_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "
trajectory:
  joint_names: [joint1, joint2]
  points:
    - positions: [0.0, 0.0]
      time_from_start: {sec: 0, nanosec: 0}
    - positions: [0.8, -0.5]
      time_from_start: {sec: 3, nanosec: 0}
"
```

成功后终点应为：

```text
joint1 ≈ 0.8
joint2 ≈ -0.5
```

### 5.9 RViz 同步验证

```bash
rviz2 --ros-args -p use_sim_time:=true
```

RViz 设置：

```text
Fixed Frame: base_link
Add → RobotModel
Description Topic: /robot_description
Durability Policy: Transient Local
Add → TF
Show Names / Show Axes
Marker Scale: 0.3
Distance: 1.5
```

再发一条不同轨迹，确认 Gazebo 和 RViz 同步运动。

---

## 6. 实验结果

### 6.1 模型成功生成到 Gazebo

![Gazebo 中的 simple_arm](assets/gazebo.png)

这一张对应：

```text
ros2 node list        → /gazebo, /robot_state_publisher
ros2 service list     → /spawn_entity
spawn_entity.py       → Successfully spawned entity [simple_arm]
```

Gazebo 终端中也能看到 `GazeboSystem` 硬件接口完成初始化、配置和激活。

### 6.2 轨迹执行成功

![轨迹执行与控制输出](assets/control.png)

这张图里包含：

```text
joint_trajectory_controller: Goal reached, success!
Goal finished with status: SUCCEEDED
/joint_states:
  joint1 ≈ 0.7999999
  joint2 ≈ -0.4999973
```

说明从 action goal → controller → hardware interface → Gazebo 的整条命令链已经打通。

### 6.3 Gazebo 与 RViz 同步

![Gazebo 与 RViz 同步](assets/gazebo_rviz.png)

RViz 里能看到机械臂和 Gazebo 中的姿态一致，说明：

```text
Gazebo 实际关节状态
→ joint_state_broadcaster
→ /joint_states
→ robot_state_publisher
→ /tf
→ RViz
```

这条状态链也是通的。

---

## 7. 踩过的坑

### 7.1 `xacro` 没输出不是失败

```bash
xacro simple_arm.urdf.xacro > /tmp/simple_arm.urdf
```

没有输出、退出码为 0，就是成功，内容已经写进 `/tmp/simple_arm.urdf`。

### 7.2 `spawn_entity.py` 一直等 `/robot_description`

说明 RSP 没启动，或者没有正确发布：

```text
Waiting for entity xml on robot_description
```

检查：

```bash
ros2 topic list | grep robot_description
```

### 7.3 一直等 `/spawn_entity`

说明 Gazebo 的 ROS factory 插件没加载，或者连接到了残留 Gazebo 实例。

检查：

```bash
ros2 node list
```

正常应该看到 `/gazebo`：

```text
/gazebo
/robot_state_publisher
```

没有 `/gazebo` 就清掉 Gazebo 进程，重新：

```bash
ros2 launch gazebo_ros gazebo.launch.py
```

### 7.4 只有一个控制器 active

`spawner` 一次只启动一个控制器：

```bash
ros2 run controller_manager spawner joint_state_broadcaster
ros2 run controller_manager spawner joint_trajectory_controller
```

两条都要执行，`controllers.yaml` 只是声明。

### 7.5 `/dynamic_joint_states` 是什么

除了 `/joint_states`，还会看到：

```text
/dynamic_joint_states
```

这是 ros2_control 的更通用关节状态格式，不是错误。TF 和 RViz 使用 `/joint_states`。

### 7.6 `effort: .nan`

Gazebo 硬件接口没有提供 effort 状态接口，所以：

```text
effort: [nan, nan]
```

位置控制不需要它，不是错误。

### 7.7 控制器更新周期警告

```text
Desired controller update period (0.01 s) is slower than
the gazebo simulation period (0.001 s).
```

控制器 100 Hz，Gazebo 1000 Hz，属于正常警告。

### 7.8 RViz 配置要保存

RViz 关闭时如果提示保存：

```text
Save changes to /home/wsq/.rviz2/default.rviz?
```

建议点 **Save**，下次直接恢复 Fixed Frame、RobotModel 和 TF。

---

## 8. 和真机、VLA 的关系

| 今天学的 | 真机/VLA 中的用途 |
| --- | --- |
| `ros2_control` | 同一套控制器可以切换到真实硬件接口 |
| `joint_trajectory_controller` | 真机底层关节轨迹执行 |
| `joint_state_broadcaster` | 编码器状态进入 ROS2 |
| Gazebo 物理仿真 | 真机前的控制和安全验证 |
| `use_sim_time` | 仿真和真机时间系统切换 |
| 硬件抽象 | VLA 高层动作最终落到真实关节命令 |

以后换真机时，大致的替换是：

```text
GazeboSystem 插件
        ↓ 替换
厂商硬件接口插件 / ros2_control 驱动
```

控制器、轨迹 action、`/joint_states` 和 TF 结构尽量保持不变。

VLA 输出的是高层动作或动作块，最终仍然要经过：

```text
动作 → 轨迹/控制器 → ros2_control → 真实关节
```

---

## 9. 下一步

今天完成了“物理仿真 + 控制器驱动”。

接下来进入：

```text
MoveIt2
```

目标：

- Panda/简化机械臂的运动规划；
- 避障、碰撞检测、轨迹执行；
- 把今天的 `joint_trajectory_controller` 接到 MoveIt2 的执行层。
