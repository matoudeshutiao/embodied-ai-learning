# 2026-11-22 Day 32 原始记录

## 今天的目标

- 验证把生产检测器从 bbox 中心改成顶面 3D 质心后的实际效果。
- 检查相机内参是否真的是剩余误差的来源。
- 开始 eye-in-hand：加腕部相机，准备手眼标定。
- 如果条件允许，完成第一次手眼标定实验。

## 3 次 A3 回归

生产检测器改成顶面 3D 质心（2 mm 深度带）后，跑 3 次 A3：

```text
trial 1: vision 4.6 mm, place 4.4 mm
trial 2: vision 4.5 mm, place 4.3 mm
trial 3: vision 4.0 mm, place 3.8 mm
3/3 全链路成功
```

对比改前：

```text
vision: 6.1 → 4.4 mm
place:  5.9 → 4.2 mm
```

说明新检测器在真实抓放链路中生效，和静态对比的 B 方法 4.08 mm 一致。

## K 检查：内参没有问题

读 Gazebo 实际发布的相机内参：

```text
/camera/camera_info
/camera/depth/camera_info
```

两者一致：

```text
fx = fy = 277.1273455955935
cx = 160.5
cy = 120.5
```

理论值：

```text
fx = (320/2) / tan(1.047198/2) ≈ 277.127
```

结论：**内参是对的**，之前“乘 1.024 后误差降到 1 mm”只是对单侧小区间的过拟合，
不能证明存在 2.4% 的内参比例错误。

## 相机另一侧的静态对比

把方块放到相机另一侧（x 0.62–0.68，y -0.12 ~ -0.04），重跑 10 个位置：

```text
        原区域 (x<0.55)        新区域 (x>0.55)
A       6.45 mm, 径向 +6.42    2.48 mm, 径向 +2.17
B2      4.08 mm, 径向 +3.99    3.57 mm, 径向 -0.56
C       8.31 mm, 径向 +8.24    3.60 mm, 径向 +1.74
```

关键：

- B 的径向误差符号翻转，径向缩放校正失效（校正后仍 3.5 mm）；
- 方法排名变化：原区域 B 最好，新区域 A 最好；
- 误差随视角变化，不是稳定的内参比例误差；
- 320×240 下 1 像素 ≈ 2.7 mm，2.5–4 mm 约等于 1–1.5 像素，已接近像素/边缘分割下限。

## 腕部相机 Stage 1

新增：

```text
panda_hand → wrist_camera_link 固定关节
wrist_camera_link → wrist_camera_optical_frame 静态 TF
libgazebo_ros_camera 深度相机
preserveFixedJoint 防止 Gazebo 合并 link
```

验证：

```text
/wrist_camera/image_raw 约 5.7 Hz
wrist_camera_link TF 存在
pre-grasp 能看到红方块
```

RViz 新增腕部相机图像显示。

## 安装位置迭代

### 1. x=0.06, z=-0.04, pitch=-1.5708

- 方块在画面上方，顶面接近上边缘；
- 单帧探测：`translation 3.72 mm, normal 151.85°, reprojection 4.71 px`；
- 角点 v=5，顶面可能被图像边缘裁切。

### 2. 上仰 17°（pitch=-1.87）

- 方块进入画面中间；
- 但顶面不再与图像平面平行，±2 mm 深度带只圈到一条等深线附近的窄带：
  `top area=564 px²`，约 17×54 px；
- PnP：`translation 18.75 mm / normal 110.78° / reprojection 4.76 px`，不可信。

### 3. 回退倾斜，x=0.045, z=-0.04

- `top area=1302 px²`；
- `translation 2.57 mm`，但 `normal 102.30° / reprojection 3.86 px`；
- 画面上方有一小条手/手指遮挡，四边形变成梯形。

### 4. 沿手背后退，x=0.06, z=-0.06

- `top area=1094 px²`；
- `translation 19.81 mm / normal 100.25° / reprojection 6.67 px`；
- 但这次 pre-grasp 姿态仍由俯视视觉生成，手臂遮挡使目标偏了约 23 mm，
  不是公平的安装位置对比。

## 今天得到的三个关键认识

### 1. 顶面深度带要求“不倾斜”

±2 mm 深度带假设顶面与图像平面平行。相机一倾斜，顶面深度不再恒定，
深度带只取到一条等深线附近的窄带，PnP 随之失效。
如果不改检测算法，腕部相机必须保持不倾斜。

### 2. 遮挡有两个方向

- 腕部相机：手/手指会切掉画面的上沿，顶面变成梯形；
- 俯视相机：机械臂抬高后挡在相机和方块之间，`panda_move_to_target`
  在 0.30 m 时会拿到偏 23 mm 的目标。

所以标定姿态不能依赖俯视视觉；至少要保证每个姿态里相机能完整看到目标。

### 3. 方形顶面不适合完整手眼标定

方块顶面是正方形，PnP 有 4 个 90° 旋转等价的解，yaw 不可观测。
完整的手眼旋转标定需要**非对称目标**：

```text
ArUco / 棋盘格
或给方块顶面加一个不对称标记
```

## 关于“用真值”的边界

| 用法 | 影响 | 能不能算仿真 |
|---|---|---|
| 用 `/model_states` 选标定姿态 | 小，等价于实验员挑位姿 | 可以 |
| 在求解里用方块真值算目标世界位置 | 大，把“手眼标定”变成“已知目标平移标定” | 不行 |

今天写的 `wrist_hand_eye_translation.py` 属于第二种：
它在求解里用了 `target_base` 真值，只能作为 known-target 平移诊断，
不能当 `cv2.calibrateHandEye` 的纯 AX=XB 结果。

## 今天的决定

```text
手眼标定：暂缓
原因：目标可见性、遮挡、方形顶面的 yaw 二义性
下一步：先保证标定姿态里目标完整可见，
        再用非对称目标（ArUco/棋盘格）做纯 AX=XB
```

## 用户原话

> 不行，那你继续修改，精度必须至少可以做手眼标定，我今天要实现手眼标定的初次实验

> 现在主要问题是不要倾斜，以及不要遮挡对吗

> 影响大吗？影响大我们就还是先回去，继续调那个相机位置，
> 今天先不进行手眼标定。我还是希望尽可能仿真

> 行今天先这样吧明天再看看，写day32的日志

## 提交

```text
a277e4c feat: use top-face centroid in production detector
3f53d5f feat: add wrist camera stage 1
e3015b8 feat: add wrist target PnP probe
c88cdbf fix: tilt wrist camera and select IPPE normal
475e1b8 docs: record wrist PnP limitation
3136c7a fix: revert wrist tilt and reduce side offset
b43eda9 fix: move wrist camera back to clear hand
1adda62 feat: add wrist hand-eye translation calibration
```

## 还没做

```text
目标完整可见的标定姿态
非对称标定目标（ArUco/棋盘格或顶面标记）
纯 AX=XB 手眼标定
腕部检测 + 接近过程闭环
物理摩擦抓取
真机
```
