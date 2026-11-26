# LingBot-VA

## 官方入口

- GitHub：https://github.com/Robbyant/lingbot-va
- 项目页：https://technology.robbyant.com/lingbot-va
- 模型：https://huggingface.co/robbyant/lingbot-va-base
- 2.0 项目页：https://technology.robbyant.com/lingbot-va-v2

## 一句话

自回归视频-动作世界模型：预测下一帧和对应动作，执行后用真实观测刷新 KV-cache，形成闭环。

## 阅读时只看这五点

1. 输入是什么：当前图像、语言、机器人状态？
2. 视频和动作怎么在同一个序列里交错？
3. IDM 在哪一步把视频表示变成动作？
4. KV-cache 和异步执行解决什么问题？
5. 为什么需要 18–24GB 显存？

## 本机结论

RTX 4060 8GB 不跑本体，只读代码和论文。

## 相关笔记

- [第 1 周 pipeline](../notes/01_pipeline.md)
