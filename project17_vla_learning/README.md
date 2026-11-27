# 项目 17：VLA / Video Action Model 入门

> 状态：2026-11-25 开始
> 当前阶段：video action model 入门
> 时间预算：每天 30–60 分钟，到 2026 年 12 月底
> 设备：RTX 4060 Laptop 8GB

## 这个项目要做什么

不是从零训练 VLA，也不是复现 LingBot-VA / mimic-video 的完整模型。

目标是先用小算力走通概念链路：

```text
视频 backbone
→ 未来帧 / latent 表示
→ IDM / action decoder
→ 动作
→ 执行后用真实观测回灌
```

最终交付：

```text
1. 一页 pipeline 图，能讲清 LingBot-VA 和 mimic-video
2. 一个 GPU 可跑的 toy IDM：相邻帧预测动作
3. 一个 frozen encoder + action decoder 小实验
4. 一页差距分析：toy 和真模型差在数据、算力、闭环
5. 5 分钟讲解 + README
```

## 目录

```text
project17_vla_learning/
├── README.md
├── PLAN.md         # 近一个月计划
├── notes/          # 每周笔记、问题、pipeline 图
├── references/     # 论文和官方仓库入口
├── toy_idm/        # CPU/GPU 小实验
└── assets/         # 截图、图、视频封面
```

## 近一个月计划

详细版见 [PLAN.md](PLAN.md)。

| 周 | 日期 | 时间 | 交付 |
|---|---|---:|---|
| 第 1 周 | 11.25–12.01 | 3–4 h | `notes/01_pipeline.md` |
| 第 2 周 | 12.02–12.08 | 3–5 h | `notes/02_basics.md` |
| 第 3 周 | 12.09–12.15 | 3–4 h | `notes/03_code_reading.md` |
| 第 4 周 | 12.16–12.22 | 4–6 h | `toy_idm/` 代码与结果 |
| 第 5 周 | 12.23–12.31 | 2–4 h | README、5 分钟讲解、差距分析 |

## 两个核心参考

- [LingBot-VA](references/01_lingbot_va.md)
- [mimic-video](references/02_mimic_video.md)

## 边界

```text
本机 8GB：只跑 toy、小视频预测、冻结小 encoder
LingBot-VA / mimic-video 本体：需要 24GB 级 GPU 或云服务器
不在本机硬刚完整模型
```

## 和 project16 的关系

`project16_ros2_vla_bridge` 解决的是机器人与执行链路：

```text
ROS2 → MoveIt2 → 夹爪 → 手眼标定 → 视觉抓放
```

`project17_vla_learning` 解决的是学习型策略：

```text
视频/图像观测 → 动作模型 → 策略评估
```

以后可以把 project17 训练出的 action decoder 接到 project16 的仿真抓取环境里。
