# aBot — 从数字孪生到真实人形机器人

> 一个把「想法」变成「会动的机器人」的项目。用 Blender 构建人形数字孪生，探索机器人的软件与智能（运动控制 / 感知 / LLM 智能），目标是一台桌面级小型人形智能机器人。

## 🎯 北极星目标

**开发一台人形机器人**。当前阶段用 **Blender 构建数字孪生**（演示真实机器人形态），并在此之上探索机器人的软件与智能部分：

- **运动控制**：骨骼、关节、动作（9 个关键动作：relax / tpose / apose / idle / wave / walk / nod / look / run）
- **感知**：相机 + IMU（P3）
- **智能大脑**：LLM 自然语言 → 动作契约 → 执行（P4）

项目同时是 **亲子共学** 的对象（为 8 岁孩子设计的低门槛参与与学习手册）。

> 注：「为 AGENT 提供长期记忆系统」是 ArchGraph 项目的北极星，不是 aBot 的；aBot 将 ArchGraph 长期记忆作为机器人「大脑」记忆层的外部协同能力复用。

## 🗺️ 方案与路线

核心思路：**单一事实源 + 契约不变、后端扩三**。

- **单一事实源**：程序化脚本 `build_humanoid.py` 同时生成 Blender 模型（.blend）、物理模型（MJCF）与真机关节配置，避免多份模型漂移。
- **契约不变**：twin-control 提供统一动作契约（pose / motion / FK / state / stop / health）。
- **后端扩三**：同一契约可切换 Blender（渲染）/ MuJoCo（物理）/ 真机（hardware_adapter）。

三阶段开发过程：

1. **数字孪生**：Blender 角色 + 动作控制接口，键盘可控制基本动作（已完成）
2. **MCP 接口**：将 twin-control 封装为 MCP Server，Agent 通过 MCP 控制角色（已完成）
3. **真实物理 ROBOT**：组装真机 + MCP 接口，Agent 运行在电脑上控制真机（规划中）

路线图：**P0 数字孪生建模 → P1 实时操控 → P2 物理与运动（MuJoCo）→ P3 感知 → P4 智能大脑 → P5 硬件对齐**。

> 安全原则：LLM 只允许调用已验收的动作契约白名单，禁止直出关节角；真机执行前先上安全联锁。

## ✨ 当前状态（2026-09）

- ✅ **chibi 人形数字孪生**：程序化建模，19 根骨骼，9 个关键动作烘焙为 joint-only 骨骼关键帧（GUI 即播）
- ✅ **实时操控**：twin_server + TwinClient（HTTP 127.0.0.1:8123），pose/motion/FK/state 全链路
- ✅ **键盘控制**：`keyboard_control.py` 键盘实时控制基本动作
- ✅ **MCP 接口**：`twin_mcp_server.py` 封装为 MCP，Agent 可调用控制角色
- ✅ **aBot 人格 Agent**：数字孪生是它的身体，可直接对话让它动起来
- ✅ **AI 团队协作**：项目总管 / 视觉分析员 / 机器人3D建模员 / 技术洞察团队，配合意图图（ArchGraph）长期记忆与验收测试体系

## 🚀 快速开始

### 1. 启动数字孪生（GUI 模式，可看到动作）

```powershell
blender --python scripts/blender_humanoid/twin_server.py
```

### 2. 用键盘控制

```powershell
python scripts/blender_humanoid/keyboard_control.py
# 空格=idle 1=relax 2=tpose 3=apose 4=wave 5=nod 6=look 7=walk 8=run 0=stop h=帮助 q=退出
```

### 3. 用 Agent / MCP 控制（需重启 opencode 加载 `twin-control` MCP）

重启 opencode 后，Agent 可调用 `motion(name="wave")`、`pose(name="relax")`、`fk(bone="head", x=0.3, y=0, z=0)` 等工具控制角色。

### 4. 与 aBot 人格对话

重启 opencode 后切换到 `abot` 主 Agent，直接跟它说话，它会通过 MCP 控制自己的「身体」。

## 📁 目录结构

```
assets/humanoid/          数字孪生模型、渲染图、动作证据图
scripts/blender_humanoid/ 建模 / 操控 / 键盘 / MCP / 渲染脚本
tests/acceptance/         可执行验收测试（GIVEN-WHEN-THEN）
design/KG/                ArchGraph 意图图（长期记忆与架构）
docs/learning/            亲子共学手册
docs/booklist-robot-ai/   公众号文章与购买落地页
.opencode/agent/          opencode 人格/角色 Agent 定义
```

## 🧪 质量与验收

- 每个功能都有**可执行验收测试**（`tests/acceptance/test_*.py`），全部通过后才交付（当前 16 个用例全绿）。
- 架构与记忆登记在 ArchGraph 意图图（`design/KG/SystemArchitecture.json`），支持跨会话记忆与验收回归。

## 🤖 用 AI 辅助开发

项目由多个 AI 角色协作完成，每个角色是独立 opencode Agent（有身份、模型与记忆）：

| 角色 | 模型 | 职责 |
|---|---|---|
| 项目总管 | Qwen3.8 Max | 全局目标、长期记忆、角色协调、验收把关 |
| 视觉分析员 | Qwen3.8 Max（多模态） | 看图复核渲染与动作 |
| 机器人 3D 建模员 | Qwen3.8 Max（多模态） | 建模、蒙皮、动作、键盘/MCP 实现 |
| 技术洞察团队（5 角色） | Max / Flash / DeepSeek | 雷达侦察、四环评估、实验验证、报告 |

**AI 辅助方式**：意图图记忆、验收测试优先、委派与复核闭环（建模 → 视觉复核 → 修正）、联网调研（DashScope WebSearch）做选型。

## 🧭 路线图与下一步

- **P2 物理与运动**：接入 MuJoCo（已选型 Adopt），步态/平衡/碰撞仿真
- **阶段③真实 ROBOT**：组装桌面小型人形机器人（参考技术洞察：SG90→STS3215/Dynamixel、PCA9685、PC 外挂主控）
- **P3 感知 / P4 智能大脑 / P5 硬件对齐**

---

*aBot 项目 ｜ 2026-09-01 ｜ 与 AI 团队一起打造*
