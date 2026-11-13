# 指令输入接口 v0

> 状态：预留接口，尚未实现。
>
> 目的：以后加语音模块时，只替换输入源，不改抓放任务代码。

## 话题

```text
/policy/instruction
类型：std_msgs/String
```

v0 先用字符串，JSON 格式：

```json
{
  "action": "pick_and_place",
  "object": "red_cube",
  "target": "blue_box"
}
```

也可以更简单：

```text
pick red cube into blue box
```

先由规则解析，后续再换 LLM/VLA。

## 数据流

```text
文本输入 / ASR 语音
→ /policy/instruction
→ task_policy
→ 目标物体和放置位置
→ MoveIt2 / action
→ safety_filter
→ safe_executor
→ JTC
```

语音模块只负责：

```text
麦克风 → 文本 → /policy/instruction
```

它不直接发关节轨迹，也不绕过安全层。

## 延迟与抢占

```text
ASR：0.5–3 s
LLM：0.5–5 s
策略频率：5–10 Hz
控制器：100–1000 Hz
```

规则：

```text
1. 指令异步到达，不阻塞控制循环
2. 新指令默认“最新优先”
3. 执行中收到新指令，先取消/重规划
4. 语音解析失败时保持当前任务或安全停止
5. 任何情况下都不能跳过 safety_filter
```

## 后续实现

```text
1. 先做文本指令发布节点
2. 再做 ASR 节点发布同一个话题
3. 需要开放式语言时再加 LLM 解析
4. 最后才考虑端到端 VLA
```
