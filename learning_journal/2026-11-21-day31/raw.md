# 2026-11-21 Day 31 原始记录

## 今天的目标

- 完成 A3 随机位置抓放 20 次，统计全链路成功率。
- 把试验结果自动整理成报告并推送到 GitHub，避免手工贴日志。
- 量化视觉误差来源，决定下一步做 eye-in-hand 还是先改检测器。

## A3：20 次随机位置抓放

命令：

```bash
python3 ~/embodied-ai-learning/project16_ros2_vla_bridge/scripts/panda_pick_trials.py \
  --trials 20 --seed 0
```

范围：x ∈ [0.42, 0.48]，y ∈ [-0.18, -0.12]，±3 cm；放置点 `(0.40, 0.08, 0.375)`。

结果：

```text
全链路成功     20/20
抓取成功       20/20
视觉误差       mean 6.1 mm, max 7.5 mm
放置误差 xy    mean 5.9 mm, max 7.3 mm
放置误差 z     mean 1.0 mm
视觉-放置相关  r = 0.995
```

异常：

```text
第 5 次命令位置  (0.4486, -0.1450)
实际位置        (0.4570, -0.1610)
偏差            约 18 mm
```

机器人仍然成功，因为视觉跟随的是实际方块，不是命令位置。
这条异常说明“命令位置”和“实际位置”必须分开记录。

报告：

- [experiments/a3/20261121_201637/report.md](../../project16_ros2_vla_bridge/experiments/a3/20261121_201637/report.md)
- 提交：`d82362c experiment: publish A3 20261121_201637 (20/20)`

## 报告自动化

新增：

```text
scripts/panda_pick_trials.py    # 试验执行
scripts/publish_experiment.py   # 生成 report.md + git commit/push
experiments/a3/<run_id>/        # report.md / summary.csv / meta.json / logs
```

试验时加 `--publish`：

```bash
python3 project16_ros2_vla_bridge/scripts/panda_pick_trials.py \
  --trials 20 --seed 0 --publish
```

推送第一次失败：

```text
remote: Invalid username or token.
Password authentication is not supported for Git operations.
```

原因：GitHub 不支持账号密码，必须用 Personal Access Token。
在虚拟机配置 `credential.helper store`，用 fine-grained token
（Contents: Read and write）作为密码后推送成功。

## 静态视觉估计器对比

脚本：

```text
scripts/vision_estimator_compare.py
```

不动机器人，把方块传送到 10 个已知位置，比较三种中心估计：

```text
A  整个红色轮廓的 bbox 中心 + 深度中值
B  顶面深度过滤后的 3D 质心
C  整个轮廓的矩质心 + 深度中值
```

结果：

```text
A  mean 6.45 mm, max 7.69 mm, 径向 6.42 mm
B  mean 4.18 mm, max 4.84 mm, 径向 4.10 mm
C  mean 8.31 mm, max 9.54 mm, 径向 8.24 mm
```

B 比 A 下降约 35%，说明 bbox 把可见侧面算进去，
中心被拉向图像中心，确实是误差的一部分。

## 顶面阈值扫描与径向缩放

对 B 再测 1/2/4/8 mm 四种深度阈值，并给每种方法拟合一个纯径向缩放 s：

```text
          原始误差      径向缩放 s     校正后误差
A         6.45 mm       1.0388         0.71 mm
B1 (1mm)  4.08 mm       1.0236         0.97 mm
B2 (2mm)  4.08 mm       1.0236         0.97 mm
B4 (4mm)  4.18 mm       1.0243         0.85 mm
B8 (8mm)  4.38 mm       1.0250         1.06 mm
C         8.31 mm       1.0502         1.07 mm
```

结论：

- 1 mm 和 2 mm 阈值结果相同，说明侧面顶部残留影响很小，**阈值不是主因**。
- 每种方法乘上 ~1.02–1.04 的径向缩放后，误差都降到 0.7–1.1 mm。
- 说明剩余误差是**纯径向比例误差**（内参/投影），不是随机噪声。
- 0.7–1.1 mm 已小于 1 个像素（320×240 下约 2.7 mm/px），
  接近分辨率和量化下限。

## 生产检测器落地与次日验证

`panda_color_detector.py` 默认改为：

```text
estimator = top
top_depth_margin = 0.002 m
min_top_points = 20
```

debug 图中红方块的黄点 = 顶面 3D 质心；
可用 `-p estimator:=bbox` 切回旧方法。

次日（2026-11-22）跑 3 次 A3 回归：

```text
trial 1: vision 4.6 mm, place 4.4 mm
trial 2: vision 4.5 mm, place 4.3 mm
trial 3: vision 4.0 mm, place 3.8 mm
3/3 全链路成功
```

和静态对比 B 的 4.08 mm 一致，说明新方法在真实抓放链路中生效。

## 相机窗口

`panda_moveit_gazebo.launch.py` 新增第二个 RViz：

```text
窗口 1：MoveIt MotionPlanning
窗口 2：Camera Raw + /vision/debug_image + target marker + TF
```

参数：

```text
start_camera_view:=false    # 关闭相机窗口
```

两个窗口不是重复：MoveIt 窗口负责规划和碰撞体，相机窗口负责图像和检测。
RobotModel/TF 两个窗口都有，只是为了让相机窗口也能看出空间关系。

## 用户原话

> 你可以写一个脚本每次让虚拟机把实验报告传到GitHub然后你去访问看报告，
> 不然我这样好麻烦，就专门搞一个文件夹

> 所以我解决了那个bbox的误差吗？用的什么方法通俗易懂解释一下。

> 为啥有两个rviz页面，一个有相机一个没有，是不是搞重复了？

## 提交

```text
e84af35 feat(vla_bridge): add A3 random-position trial runner
6901704 fix(vla_bridge): handle Humble SetEntityState response fields
19d7605 feat: add A3 experiment report publishing
d82362c experiment: publish A3 20261121_201637 (20/20)
55d1b7c feat: add static vision estimator comparison
1593320 feat: open camera RViz with Gazebo demo
4d5c1a0 feat: sweep top-face margins and fit radial scale
a277e4c feat: use top-face centroid in production detector
```

## 还没做

```text
相机内参标定：修正 ~2.4% 的径向比例误差
eye-in-hand：解决遮挡和接近过程中的闭环修正
±5 cm 更大范围试验：找失败边界
物理摩擦抓取：仍然依赖 link attach
真机：未开始
```
