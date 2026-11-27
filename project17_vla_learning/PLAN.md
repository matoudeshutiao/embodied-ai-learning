# 近一个月计划：VLA / Video Action Model 入门

> 周期：2026-11-25 → 2026-12-31
> 预算：每天 30–60 分钟，总计约 15–25 小时

## 一个月目标

用一个 8GB 游戏本，走通 video action model 的概念链路，不硬刚完整大模型。

最终交付：

```text
1. 一页 pipeline 图：LingBot-VA vs mimic-video
2. 一个 GPU toy IDM：相邻帧预测动作
3. 一个 frozen encoder + action decoder 小实验
4. 一页差距分析：toy 和真模型的差距
5. 5 分钟讲解 + README
```

验收标准：

```text
能不看稿解释：
- 为什么 video action model 要先预测未来
- IDM 和 action decoder 的区别
- LingBot-VA 和 mimic-video 的主要区别
- 为什么它们需要 24GB 级显存
```

## 每周计划

### 第 1 周：11.25–12.01

时间：3–4 h

任务：

```text
读 LingBot-VA、mimic-video 的官方入口
画一页数据流图
回答 notes/01_pipeline.md 的 5 个问题
```

交付：

```text
notes/01_pipeline.md
assets/pipeline.png
```

### 第 2 周：12.02–12.08

时间：3–5 h

任务：

```text
补 video diffusion / flow matching 基础
理解 latent video、时序一致性
理解 inverse dynamics model
```

交付：

```text
notes/02_basics.md
```

### 第 3 周：12.09–12.15

时间：3–4 h

任务：

```text
读官方代码结构
看数据格式：LeRobot、action 维度、相机输入
定位 video backbone、action decoder、IDM
```

交付：

```text
notes/03_code_reading.md
```

### 第 4 周：12.16–12.22

时间：4–6 h

任务：

```text
生成合成 2D 方块视频
训练 toy IDM：相邻帧 → (dx, dy)
对比单帧输入 vs 相邻帧输入
可选：冻结 ResNet18 提特征，只训练 action decoder
```

交付：

```text
toy_idm/ 代码
loss 曲线和误差表
```

### 第 5 周：12.23–12.31

时间：2–4 h

任务：

```text
整理 README
录 5 分钟讲解
写差距分析：数据、算力、闭环
```

交付：

```text
project17_vla_learning/README.md
notes/05_gap.md
```

## 每周检查点

```text
第 1 周：能画出完整数据流
第 2 周：能解释 flow matching / IDM
第 3 周：能指出代码里每个模块的位置
第 4 周：toy IDM 能训练和评测
第 5 周：能讲清 toy 和真模型差在哪
```

## 明确不做

```text
不在 8GB 本机跑 LingBot-VA / mimic-video 本体
不训练视频扩散模型
不从零训练 VLA
不采 100 条真机数据
不承诺复现 SOTA
```

## 如果时间不够

优先保住：

```text
第 1 周 pipeline
第 4 周 toy IDM
第 5 周总结
```

第 2、3 周可以压缩成阅读笔记，但不能跳过。

## 如果寒假能全职

可以提前做第 4 周，但验收标准不变：

```text
代码可跑、结果可复现、结论有边界
```
