# 第 1 周：LingBot-VA vs mimic-video pipeline

> 状态：Claude 先整理的参考版，待用户核对/改写。
> 产出：一页数据流图 + 下面 5 个问题。

## 数据流图

```text
LingBot-VA：
图像 + 语言 + 状态
        ↓
自回归视频-动作 backbone
        ↓
下一帧 + 动作交错预测
        ↓
IDM / action head 解出动作
        ↓
执行 → 真实观测替换 KV-cache → 继续闭环
```

```text
mimic-video：
图像 + 语言 + 状态
        ↓
预训练视频 backbone（冻结或 LoRA）
        ↓
中间 latent 特征
        ↓
flow-matching action decoder
        ↓
动作
        ↓
执行后重新观测/推理
```

关键差异：

```text
LingBot-VA：真的预测未来画面，再把画面变成动作
mimic-video：不生成完整像素视频，直接用视频 backbone 的 latent 出动作
```

## 5 个问题

### 1. 为什么 video action model 要先预测未来？

因为只看当前帧，模型只知道“现在长什么样”，不知道“动作会导致什么”。

先预测未来帧或未来 latent，相当于让模型学一个近似的物理/任务动力学：

```text
现在这样 → 做这个动作 → 下一步会变成什么样
```

这样有利于长程任务、遮挡、物体状态变化和闭环纠错。  
人话：先脑补“下一步画面”，再决定“这一步动作”。

### 2. IDM 和 action decoder 的区别是什么？

**IDM（inverse dynamics model）**：

```text
看状态变化，反推动作
(frame_t, frame_t+1) → action
```

它回答的是：“从这一帧到下一帧，机器人做了什么？”

**action decoder**：

```text
看视频 backbone 的 latent 特征，直接回归/生成动作
latent → action
```

它不一定要显式比较两帧，而是把视频模型学到的表示映射成控制量。

```text
IDM：先有状态转移，再反推动作
action decoder：直接从视觉表示生成动作
```

mimic-video 更接近“latent → flow-matching action decoder”；
LingBot-VA 的流程里明确有视频预测 + IDM 的角色。

### 3. LingBot-VA 为什么需要 KV-cache？

因为它是自回归视频-动作模型：

```text
第 1 步预测 → 第 2 步要用第 1 步的结果
第 2 步预测 → 第 3 步要用前两步的结果
```

KV-cache 缓存历史帧/动作的注意力键值，避免每一步都从头重算。

作用：

```text
1. 支持长程记忆
2. 降低自回归推理成本
3. 支持“预测未来 + 异步执行”
4. 执行后用真实观测替换缓存，避免一直基于想象漂移
```

人话：它把“刚才发生了什么”记在缓存里，不用每次重新看完整历史。

### 4. mimic-video 为什么可以不做完整像素生成？

因为它不需要把未来画面真正渲染出来。

它用的是视频 backbone 中间的 latent 表示，再训练一个轻量 action decoder：

```text
视频 backbone latent → action decoder → action
```

好处：

```text
1. 不用视频 decoder 生成像素，省算力
2. 不受像素生成模糊/伪影影响
3. action decoder 参数量相对小
4. 视频 backbone 可以冻结或只做 LoRA
```

人话：它不要求模型“画出未来”，只要求模型“理解未来”，然后直接从理解里出动作。

### 5. 这两个模型各自最吃算力的部分是哪一块？

共同瓶颈：

```text
视频 backbone
```

包括：

```text
高分辨率视频 token
时间维 attention
扩散/flow matching 迭代
KV-cache / 长上下文
训练数据和 batch
```

区别：

```text
LingBot-VA：
视频生成 + 自回归推理 + KV-cache，显存需求高
官方说明约 18–24GB 级别

mimic-video：
视频 backbone 仍然重，但省掉完整像素生成；
action decoder 相对轻，但 backbone 和训练仍不是 8GB 能舒服跑的
```

8GB 4060 的定位：

```text
只做 toy、小视频预测、冻结小 encoder、latent/action decoder 小实验
不跑完整 LingBot-VA / mimic-video
```

## 对比表

| 维度 | LingBot-VA | mimic-video |
|---|---|---|
| 视频表示 | 自回归预测未来帧 + 动作 | 预训练视频 backbone 的中间 latent |
| 动作生成 | 视频预测后由 IDM/action 头解动作 | flow-matching action decoder 直接吃 latent |
| 是否生成完整视频 | 是，预测未来画面 | 否，避免完整像素生成 |
| 闭环方式 | 执行后用真实观测替换 KV-cache | 执行后新观测重新进入 backbone/推理（细节待读代码） |
| 主要算力瓶颈 | 视频生成 + 自回归 + KV-cache | 视频 backbone + latent/action decoder 训练 |
| 8GB 能否跑 | 不能跑完整模型 | 不能舒服跑完整模型 |

## 我的结论

```text
LingBot-VA 更像“先想象未来画面，再反推动作”
mimic-video 更像“借视频模型的latent理解未来，直接解码动作”
两者都在解决同一个问题：不要只做当前帧到动作的映射。
```

待用户核对：

```text
1. 用自己的话重写 5 个答案
2. 标出仍然不懂的术语
3. 画出自己版本的一页数据流图
```
