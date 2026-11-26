# 第 1 周：LingBot-VA vs mimic-video pipeline

> 状态：待填写
> 产出：一页数据流图 + 下面 5 个问题

## 数据流图

```text
图像 / 语言 / 状态
        ↓
  视频 backbone
        ↓
未来帧 / latent 表示
        ↓
 IDM / action decoder
        ↓
       动作
        ↓
执行 → 真实观测回灌
```

把上面替换成你自己的版本，并补上：

- LingBot-VA 在哪一步预测下一帧？
- mimic-video 的 action decoder 吃哪一层 latent？
- 两者的闭环方式分别是什么？

## 5 个问题

1. 为什么 video action model 要先预测未来？
2. IDM 和 action decoder 的区别是什么？
3. LingBot-VA 为什么需要 KV-cache？
4. mimic-video 为什么可以不做完整像素生成？
5. 这两个模型各自最吃算力的部分是哪一块？

## 对比表

| 维度 | LingBot-VA | mimic-video |
|---|---|---|
| 视频表示 |  |  |
| 动作生成 |  |  |
| 是否生成完整视频 |  |  |
| 闭环方式 |  |  |
| 主要算力瓶颈 |  |  |
| 8GB 能否跑 |  |  |

## 我的结论

-
