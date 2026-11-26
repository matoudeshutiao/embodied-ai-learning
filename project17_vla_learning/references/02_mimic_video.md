# mimic-video

## 官方入口

- 官方 GitHub：https://github.com/mimic-video/mimic-video
- 项目页：https://mimic-video.github.io/
- arXiv：https://arxiv.org/abs/2512.15692
- Hugging Face Papers：https://huggingface.co/papers/2512.15692

> `lucidrains/mimic-video` 是社区实现，不是官方仓库。

## 一句话

用预训练视频模型提供视觉/物理先验，冻结视频 backbone，只训练一个 flow-matching action decoder，把中间 latent 变成机器人动作。

## 阅读时只看这五点

1. 为什么它可以不生成完整像素视频？
2. action decoder 吃的是哪一层 latent？
3. 视频 backbone 冻结后，训练成本降在哪里？
4. 它和“先生成视频再反推动作”的 IDM 路线有什么区别？
5. 真机/仿真评测看什么指标？

## 本机结论

视频 backbone 仍然偏重，8GB 只做小规模 latent/action decoder 实验。

## 相关笔记

- [第 1 周 pipeline](../notes/01_pipeline.md)
