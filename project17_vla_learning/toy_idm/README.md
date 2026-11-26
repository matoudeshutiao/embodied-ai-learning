# Toy IDM：相邻帧预测动作

> 状态：待实现
> 目标：用最小的 GPU 实验对应 video action model 里的 inverse dynamics model。

## 任务

```text
输入：frame_t, frame_t+1（32×32 或 64×64）
输出：动作 (dx, dy)
```

数据由脚本生成：

```text
红色方块在画面里随机速度移动
动作 = 方块位移
```

## 最小实验

```text
1. 生成 200 条短轨迹
2. 划分 train / val
3. 小 CNN + MLP 预测 (dx, dy)
4. 对比：单帧输入 vs 相邻帧输入
```

## 验收

```text
相邻帧版本的动作误差低于单帧版本
训练和评测在 RTX 4060 8GB 上可跑
有 loss 曲线和误差表
```

## 不做什么

```text
不训练视频扩散
不加载 LingBot-VA / mimic-video 本体
不接 Gazebo
```
