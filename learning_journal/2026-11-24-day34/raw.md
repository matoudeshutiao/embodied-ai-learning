# 2026-11-24 Day 34 原始记录

## 今天的目标

- 冻结 Day 33 的手眼标定结果。
- 用腕部相机检测红方块，验证 base 坐标误差。
- 做 PBVS 闭环接近，5 个起点收敛。
- 今天不接 `panda_pick`，Step 3 留到下一轮。

## Step 0：冻结标定

结果存到 `config/wrist_hand_eye.json`，方法是 all/PARK：

```text
对 URDF 真值：3.26 mm / 0.309°
板中心绝对验证：mean 2.07 mm，max 2.31 mm
```

感知使用：

```text
T_base_cam = T_base_link8 · T_link8_cam_calib
```

URDF 里的相机安装没有改；标定只在软件里用。

## Step 1：腕部相机检测红方块

测试位姿用 `/model_states` 真值选（只选姿态），
方块坐标由腕部相机 + 手眼标定计算。

方法：

```text
HSV 红色分割
+ 深度反投影
+ 红色点云 RANSAC 顶面平面
+ 顶面质心 - 2.5 cm · 法向
```

实测 3 帧：

```text
mean 2.90 mm, max 2.91 mm
```

结果里的系统性偏差约 2.3 mm 在 z 方向。

## Step 2：PBVS 闭环，第一版失败

第一版 0.20 m、alpha 0.6、2 mm 容差：

```text
converged 1/5
truth error mean/max 6.02/10.09 mm
```

失败现象：

- 某些 IK 位姿下平面内点只剩 7–8k，红色点云被切；
- 顶面中心 y 方向偏 7 mm 左右，闭环被坏检测带偏；
- 一次 MoveIt 修正失败；
- 2 mm 容差比检测/控制噪声还小。

## 修复

```text
1. 质量门：inliers / red >= 85%，red >= 3000，
   normal_z >= 0.98，cube z 在 0.36–0.39
2. 闭环起点 0.20 → 0.30 m：方块更小，IK 分支变化时不容易被切
3. 容差 2.0 → 2.5 mm
4. alpha 0.6 → 0.7
5. MoveIt 失败 +5 mm z 重试一次，并打印错误码
```

## Step 2：最终结果

```text
converged 5/5
truth error mean/max 2.41/2.61 mm
iterations [2, 3, 3, 3, 3]
```

| start | 起点偏移 | 迭代 | 最终 hand_err | 对真值误差 |
|---|---|---|---|---|
| 0 | (0,0) | 2 | 1.89 mm | 2.61 mm |
| 1 | (+2,0) cm | 3 | 2.24 mm | 2.39 mm |
| 2 | (-2,0) cm | 3 | 2.11 mm | 2.33 mm |
| 3 | (0,+2) cm | 3 | 1.32 mm | 2.47 mm |
| 4 | (0,-2) cm | 3 | 2.28 mm | 2.28 mm |

结论：

- 闭环收敛到视觉目标（hand_err <2.5 mm）；
- 最终对真值误差 2.41 mm，和 Step 1 的系统偏差一致；
- 0.30 m 起步 + 质量门后，坏检测没有再带偏闭环。

## 用到的边界

```text
真值：选初始起点、最终评估
控制：腕部相机 + 手眼标定，不使用真值
```

## 下一步

- Step 3：接进 `panda_pick`，闭环修 x/y 后再下降到 0.20 m 抓取；
- 5 次抓放验证；
- 可选：录第一次手眼标定 + 腕部闭环 demo。

## Demo 建议

值得录，但不需要录完整的 15 姿态采样。

建议 60–90 秒：

```text
1. 腕部相机看到板子，机械臂走 5–8 个标定姿态（8x 加速）
2. 终端显示 15/15 和 PARK 3.26 mm / 0.309°
3. 腕部相机检测红方块 + 5/5 闭环接近
4. 终端显示 truth mean/max 2.41/2.61 mm
```

标注“姿态采样已加速”，不要拼接不同次的结果。

## Demo 录制（实际执行）

视频（未剪辑，直接上传 B 站）：

- https://www.bilibili.com/video/BV1Qgyfz1EJd/

本次录制的是“已有标定 + 腕部闭环”的系统验证，**没有重跑 15 姿态 AX=XB**。
画面内容：

```text
单帧 ChArUco 检测
→ Day 33 已冻结的 15/15、PARK 3.26 mm / 0.309%
→ 腕部相机检测红方块
→ PBVS 5 个起点
```

采集时间：`wrist_servo_20261124_170830`。

复现结果：

```text
converged 5/5
truth error mean/max 3.41/6.11 mm
iterations [2, 3, 4, 2, 4]
```

| start | 迭代 | 最终 hand_err | 对真值误差 |
|---|---|---|---|
| 0 | 2 | 0.47 mm | 2.48 mm |
| 1 | 3 | 2.06 mm | 3.34 mm |
| 2 | 4 | 1.84 mm | 2.46 mm |
| 3 | 2 | 1.34 mm | 6.11 mm |
| 4 | 4 | 1.63 mm | 2.76 mm |

和首次接受结果的区别：

```text
首次：5/5，2.41/2.61 mm，[2, 3, 3, 3, 3]
Demo：5/5，3.41/6.11 mm，[2, 3, 4, 2, 4]
```

start 3 的检测变差：`inliers=4740`，红色点云顶面被部分切掉，
但 `inliers/red` 仍过 85% 门槛，所以质量门放行；闭环收敛到偏掉的视觉目标，
对真值 6.11 mm。说明当前质量门只有比例约束，没有绝对 inlier 下限/时间一致性。

Demo 没有改代码、没有改标定，也没有覆盖 `config/wrist_hand_eye.json`；
正式结果仍以首次 2.41/2.61 mm 为准。

## Step 3：腕部视觉抓放 5/5

时间：2026-11-24 23:39

流程：

```text
0.30 m PBVS 修正 x/y
→ 下降到 0.20 m 并做一致性校验
→ 保持 0.30 m 的 x/y
→ 笛卡尔下降到抓取位
→ 夹爪闭合、attach、抬升、放置
```

5 次随机起点结果：

```text
full success: 5/5 (100%)
grasp success: 5/5 (100%)
```

| trial | cube (x, y) | approach_err | verify_diff | place_err xy/z |
|---|---|---|---|---|
| 1 | (0.4707, -0.1345) | 0.33 mm | 0.40 mm | 0.8 / 1.0 mm |
| 2 | (0.4452, -0.1645) | 1.58 mm | 0.31 mm | 0.9 / 1.0 mm |
| 3 | (0.4507, -0.1557) | 1.90 mm | 1.24 mm | 1.6 / 1.0 mm |
| 4 | (0.4670, -0.1618) | 2.00 mm | 0.33 mm | 0.9 / 1.0 mm |
| 5 | (0.4486, -0.1450) | 1.49 mm | 0.47 mm | 1.0 / 1.0 mm |

汇总：

```text
wrist approach error: mean 1.46 mm, max 2.00 mm
0.20 m verify diff: mean 0.55 mm, max 1.24 mm
place error xy: mean 1.0 mm, max 1.6 mm
```

## Step 3 遇到的两个问题

### 1. 0.20 m 检测低质量

第一次 approach-only 在 0.20 m 报 low-quality，兜底用了 0.30 m 目标。

原因：启动时夹爪没有先张开，0.20 m 视角里手指挡住方块。
修复：wrist_pick 在 PBVS 前先张开夹爪，并打印 `finger_positions`。
修复后 0.20 m `inliers` 约 11.6k，`diff_xy` 0.2–0.5 mm。

### 2. panda_home 穿桌子还报成功

原 `panda_home` 直接发 `FollowJointTrajectory`，绕过 MoveIt，
没有碰撞检查；从放置点回 home 时扫过桌子，但控制器返回 code=0。

改成 MoveIt JointConstraint 规划后，又暴露出零位自碰撞：

```text
panda_link5 和 panda_link7 自碰撞
Start state appears to be in collision
OMPL 无法初始化起始树
```

最终处理：

```text
优先 MoveIt 规划 home
规划失败时，用直接关节轨迹做一次恢复
恢复只用于“起始态卡在自碰撞”的 bootstrap
```

从放置点回 home 时，MoveIt 正常成功：

```text
home result: code=0, moveit_code=1, ok=True
```

没有触发 `recovery=direct`，也没有再穿桌子。

## Step 3 边界

```text
真值：随机起点、重置方块、最终评估
控制：腕部相机 + 手眼标定
```

## Step 3 产物

```text
output/a3_20261124_233909/summary.csv
output/a3_20261124_233909/trial_01.log ... trial_05.log
```

## 用户原话

> 第一个，然后从这里开始算day34，
> 你可以先列个今天的计划，用到的使用方法，最终需要实现什么

> 先写日志吧，以及你觉得我们后面有必要录一下
> 第一次手眼标定的demo吗

> 录完了，还是之前那一版没有录手眼标定，懒得剪辑直接传b站了
> https://www.bilibili.com/video/BV1Qgyfz1EJd/
