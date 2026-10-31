# 项目 11：TF2、URDF 与 RViz

> 状态：✅ 已跑通。
>
> 时间：2026.10.30 ｜ 第 10 天 ｜ ROS2 周 Day 2
>
> 环境：Ubuntu 22.04.5 + ROS2 Humble（VMware 虚拟机）
>
> 目标：从“节点能通信”推进到“机器人知道自己每个零件在哪”。

第 9 天解决的是**节点之间怎么传消息**；今天解决的是**消息里的位置到底在哪个坐标系**。

今天的机械臂不再靠 PyBullet 引擎直接摆姿态，而是：

```text
URDF/xacro 描述机器人结构
+ /joint_states 提供关节角
+ robot_state_publisher 做正运动学
= TF2 中一棵实时更新的坐标树
```

---

## 1. 今天搭的机器人

一个三连杆简化机械臂：

```text
base_link
  └── joint1（revolute，绕 z）
        link1
          └── joint2（revolute，绕 y）
                link2
                  └── tool_joint（fixed）
                        tool0
                          └── camera_joint（fixed）
                                camera_link
                                  └── camera_optical_joint（fixed）
                                        camera_optical_frame
```

外观：

- 灰色底座；
- 蓝色立柱；
- 橙色横臂；
- 红色末端小球 `tool0`；
- 绿色相机 `camera_link`；
- `camera_optical_frame` 没有实体，只表示相机光学坐标系。

---

## 2. 代码结构

```text
project11_ros2_tf/
├── README.md
├── assets/
│   ├── tf.png
│   ├── rviz_model.png
│   └── tf_tree.png
└── ros2_ws/
    └── src/
        └── arm_description/
            ├── package.xml
            ├── CMakeLists.txt
            ├── launch/
            │   └── display.launch.py
            └── urdf/
                └── simple_arm.urdf.xacro
```

| 文件 | 作用 |
| --- | --- |
| `package.xml` | 声明 `robot_state_publisher`、`joint_state_publisher_gui`、`xacro`、`rviz2` 等依赖 |
| `CMakeLists.txt` | 把 `urdf/` 和 `launch/` 安装到包的 share 目录 |
| `simple_arm.urdf.xacro` | 机器人结构：link、joint、origin、axis、limit、材质 |
| `display.launch.py` | 一次启动 RSP、滑条窗口和 RViz |

---

## 3. 复现步骤

### 3.1 安装依赖

```bash
sudo apt install -y ros-humble-xacro ros-humble-joint-state-publisher-gui ros-humble-robot-state-publisher ros-humble-rviz2
```

### 3.2 编译工作空间

```bash
cd project11_ros2_tf/ros2_ws
```

```bash
source /opt/ros/humble/setup.bash
```

```bash
colcon build --symlink-install
```

```bash
source install/setup.bash
```

### 3.3 启动

```bash
ros2 launch arm_description display.launch.py
```

会启动：

- `robot_state_publisher`
- `joint_state_publisher_gui`
- `rviz2`

### 3.4 首次打开 RViz 要手动配置

我们的 launch 没有附带 `.rviz` 配置文件，所以 RViz 第一次打开是空场景；这不是启动失败。

按顺序操作：

1. **Global Options → Fixed Frame** 改成 `base_link`；
2. 左下角 **Add → RobotModel → OK**；
3. 再 **Add → TF → OK**；
4. 展开 TF，勾选 **Show Names** 和 **Show Axes**；
5. **Marker Scale** 调到 `0.3`；
6. 右侧 **Views → Current View**：
   - `Distance` 改成 `1.5`；
   - `Focal Point Z` 改成 `0.3`。

如果 RobotModel 显示不出来，检查：

```text
Description Source: Topic
Description Topic: /robot_description
Durability Policy: Transient Local
```

`robot_state_publisher` 发布 `/robot_description` 时用的是 **Transient Local** QoS；RViz 如果保持默认的 `Volatile`，就会错过启动时只发布一次的 URDF。

### 3.5 查询相机位姿

```bash
ros2 run tf2_ros tf2_echo base_link camera_link
```

两个关节都为 0 时，大约是：

```text
Translation: [0.380, 0.000, 0.510]
Rotation: in Quaternion [0.000, 0.000, 0.000, 1.000]
```

拖动 `joint1` 或 `joint2`，translation 和 rotation 会实时变化。

### 3.6 查询相机光学坐标系

```bash
ros2 run tf2_ros tf2_echo base_link camera_optical_frame
```

translation 和 `camera_link` 完全一样，但 rotation 不同：

```text
Rotation: in RPY (radian) [-1.571, 0.000, -1.571]
Rotation: in RPY (degree) [-90.000, 0.000, -90.000]
```

### 3.7 查看完整 TF 树

```bash
ros2 run tf2_tools view_frames
```

等待约 5 秒，当前目录会生成 `frames_*.pdf`：

```bash
xdg-open frames_*.pdf
```

如果提示找不到 `dot`：

```bash
sudo apt install -y graphviz
```

---

## 4. 实验截图

### 4.1 最小静态 TF：先手动造一条边

![静态 TF 与 view_frames](assets/tf.png)

这一步没有 URDF，只用 `static_transform_publisher` 手动发布：

```text
base_link → camera_link
```

终端里 `tf2_echo` 查到：

```text
Translation: [0.200, 0.000, 0.300]
Rotation: identity
```

右下角的 `view_frames` PDF 只有两个节点，说明 TF 可以只有一条边。

### 4.2 RViz 中的完整机械臂

![RViz 中的 simple_arm 与 TF](assets/rviz_model.png)

RViz 里同时验证了三件事：

- RobotModel 成功订阅 `/robot_description`；
- TF 显示里能看到 `base_link → link1 → link2 → tool0 → camera_link → camera_optical_frame`；
- `tf2_echo` 输出相机在 base 下的 translation 和旋转。

### 4.3 完整 TF 树

![完整 TF 树](assets/tf_tree.png)

`view_frames` 生成的树：

```text
base_link
  └── link1
        └── link2
              └── tool0
                    └── camera_link
                          └── camera_optical_frame
```

前两段是动态关节，后面三段是固定关节。

---

## 5. 4×4 齐次变换矩阵

TF 树里的每一条边，都可以看成一个 4×4 矩阵：

```text
T = [ R   t ]
    [ 0   1 ]
```

- `R`：3×3 旋转矩阵；
- `t`：3×1 平移向量；
- `0`：`[0 0 0]`；
- 右下角：`1`。

`tf2_echo base_link camera_optical_frame` 的实际输出：

```text
At time 1793354463.198479610
- Translation: [0.380, 0.000, 0.510]
- Rotation: in Quaternion (xyzw) [-0.500, 0.500, -0.500, 0.500]
- Rotation: in RPY (radian) [-1.571, 0.000, -1.571]
- Rotation: in RPY (degree) [-90.000, 0.000, -90.000]
- Matrix:
  0.000  0.000  1.000  0.380
 -1.000  0.000  0.000  0.000
 -0.000 -1.000  0.000  0.510
  0.000  0.000  0.000  1.000
```

### 5.1 怎么读这个矩阵

前三列是相机光学坐标系的三个轴，在 `base_link` 中的方向：

```text
光学 x 轴 → base 的 -y
光学 y 轴 → base 的 -z
光学 z 轴 → base 的 +x
```

最后一列是平移：

```text
相机原点在 base 的 (0.380, 0.000, 0.510) 米处
```

### 5.2 矩阵作用在点上

相机坐标里的点 `(x_o, y_o, z_o)` 转到 base：

```text
x_b = z_o + 0.380
y_b = -x_o
z_b = -y_o + 0.510
```

例如相机正前方 1 米：

```text
p_optical = (0, 0, 1)
p_base    = (1.380, 0, 0.510)
```

这就是 RGB-D 反投影之后，把目标点从相机坐标系转到机器人坐标系的步骤。

### 5.3 四元数、RPY、矩阵

上面三种旋转表示说的是同一个姿态：

```text
Quaternion (xyzw)：[-0.500, 0.500, -0.500, 0.500]
RPY：[-90°, 0°, -90°]
Matrix：左上角 3×3
```

四元数适合存储和插值，RPY 适合人看，矩阵适合做乘法。

---

## 6. `tf2_echo` 到底是谁相对谁？

规则：

```text
tf2_echo A B
= B 的原点在 A 坐标系中的位置
= B 相对 A 的位姿
```

例如：

```bash
ros2 run tf2_ros tf2_echo base_link camera_link
```

是相机相对底座。

```bash
ros2 run tf2_ros tf2_echo tool0 camera_link
```

是相机相对手腕。

```bash
ros2 run tf2_ros tf2_echo camera_link base_link
```

是底座相对相机，也就是反过来查。

translation 永远是**相对坐标**，不是脱离参考系的绝对坐标。所谓“世界坐标”，也只是相对 `world` 或 `map` 这个 frame。

---

## 7. 为什么拖关节时是这种变化？

`joint1` 绕 z 轴，是水平面内的 yaw：

```text
x、y 变化
z 不变
```

`joint2` 绕 y 轴，是 x-z 平面内的 pitch：

```text
z 变化最明显
x 也会变化
```

因为横臂在 x 方向的投影是 `臂长 × cos(q2)`，q2 一变，投影就变。

按这份 URDF 的尺寸，相机位置近似为：

```text
r = 0.38 * cos(q2) + 0.05 * sin(q2)

x = cos(q1) * r
y = sin(q1) * r
z = 0.46 - 0.38 * sin(q2) + 0.05 * cos(q2)
```

所以不是“x/y 只归 joint1，z 只归 joint2”，而是两个关节通过整条运动链共同影响三个坐标。

---

## 8. 三个容易混淆的点

### 8.1 `camera_link` 和 `camera_optical_frame` 平移一样

因为它们原点重合：

```xml
<origin xyz="0 0 0" rpy="-1.5707963 0 -1.5707963"/>
```

位置一样，姿态不同：

```text
camera_link：ROS 机体坐标系，x 前、y 左、z 上
camera_optical_frame：光学坐标系，z 前、x 右、y 下
```

### 8.2 固定关节的“局部不变”和“全局变化”

```bash
ros2 run tf2_ros tf2_echo tool0 camera_link
```

不管怎么拖关节，translation 都是：

```text
[0.030, 0.000, 0.050]
```

因为相机焊在手腕上。

但：

```bash
ros2 run tf2_ros tf2_echo base_link camera_link
```

会随关节变化，因为中间隔着动态关节。

### 8.3 `view_frames` 的 PDF 不会动

PDF 只是**拓扑图**，展示谁是谁的父节点；它不显示实时位置，也不会随关节运动。要看实时运动，用 RViz 的 TF 显示或 `tf2_echo`。

---

## 9. 踩过的坑

### 9.1 `file not found: 'xacro'`

launch 里用 `Command(["xacro ", xacro_file])` 调用了 `xacro` 可执行文件，但系统没装 `ros-humble-xacro`。

```text
[ERROR] [launch]: Caught exception in launch:
file not found: [Errno 2] No such file or directory: 'xacro'
```

解决：

```bash
sudo apt install -y ros-humble-xacro
```

### 9.2 RViz 的 Fixed Frame 默认是 `map`

我们的树根是 `base_link`，没有 `map`，所以左上角报：

```text
Fixed Frame [map] does not exist
```

把 `Fixed Frame` 改成 `base_link`。

### 9.3 RobotModel 报 `Error subscribing: Empty topic name`

只把 `Description Source` 设成 `Topic` 还不够，还要把 `Description Topic` 明确填成：

```text
/robot_description
```

否则 RViz 会尝试订阅一个空话题。

### 9.4 QoS：`Volatile` 收不到启动时发布的 URDF

`robot_state_publisher` 用 **Transient Local** 发布 `/robot_description`。RViz 是后启动的，如果 QoS 用 `Volatile`，就会错过那一条消息。

设置：

```text
Durability Policy: Transient Local
Reliability Policy: Reliable
```

### 9.5 RViz 里看不到机器人

除了显示项没添加，还有一个常见原因：视角太远。

机械臂只有约 0.5 米，而 RViz 默认 `Distance` 可能是 10 米。把 `Distance` 调到 1.5，`Focal Point Z` 调到 0.3。

### 9.6 相机 frame 看起来“没动”

`camera_link` 是末端的小绿盒，`camera_optical_frame` 和它原点重合、没有实体，只显示旋转过的坐标轴。放大末端、打开 TF 的 `Show Names`/`Show Axes` 才能看清。

---

## 10. 和真机、VLA 的关系

| 今天学的 | 真机/VLA 中的用途 |
| --- | --- |
| URDF/xacro | 描述真实机械臂、夹爪、相机安装位置 |
| `robot_state_publisher` | 用编码器关节角计算所有 link 的实时位姿 |
| TF2 | 查询相机、末端、底座之间的变换 |
| `camera_optical_frame` | 真实相机驱动和手眼标定的标准坐标约定 |
| `tf2_echo` | 排查“相机看到的目标到底在机器人哪个位置” |

真实相机装在夹爪上时：

```text
camera → hand：固定，来自手眼标定
hand → base：随关节运动变化
```

VLA 模型看到的是图像坐标系，输出的动作要在机器人坐标系执行，中间必须靠 TF 做翻译。多传感器频率不一致时，TF 的时间戳和插值也会参与对齐。

---

## 11. 下一步

今天只做了运动和坐标，没有物理。

下一步进入：

```text
Gazebo + ros2_control
```

目标：

- 在 Gazebo 里生成机械臂；
- 用 `ros2_control` 配置关节控制器；
- 让关节轨迹控制器真正驱动仿真关节；
- 理解“控制器接口”和“硬件接口”的抽象。

> 第 11 天已完成这部分，见 [`../project12_ros2_gazebo/README.md`](../project12_ros2_gazebo/README.md)。
