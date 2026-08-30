# aBot 项目交接总结

> 生成日期：2026-08-30 ｜ 工作区：`d:\Projects\aBot` ｜ 角色：项目总管 `project-overseer-001`

## 1. 项目概况

- **项目**：aBot — 人形机器人（早期阶段）
- **北极星/愿景**：开发人形机器人；当前用 **Blender 构建数字孪生**演示真实机器人形态，并在此之上探索机器人的**软件与智能**（运动控制 / 感知 / LLM 智能）
- **与 ArchGraph 的关系**：独立项目。「为 AGENT 提供长期记忆系统」是 **ArchGraph** 的北极星，**不是 aBot 的**；aBot 可将 ArchGraph 长期记忆作为外部协同能力复用（机器人"大脑"的记忆层），但不属 aBot 自身目标

## 2. 已完成工作

### P0 — 数字孪生模型（commit `7e3f92b`）
- 程序化构建人形 3D 角色（Blender 5.1 + bpy，基元建模 + 细分曲面 + 平滑着色 + 材质 + 19 根骨骼骨架）
- 产物：`assets/humanoid/humanoid.blend`、`preview_front.png`、`preview_3quarter.png`
- 脚本：`scripts/blender_humanoid/build_humanoid.py`

### P1 — 可驱动数字孪生：本机 Agent 实时操控（commits `c5615ee`, `2f843d4`）
> 需求变更：不要视频，改为「本机 Agent 实时操控 Blender 里的模型」
- **操控内核** `humanoid_control.py`：静态姿势（relax/tpose/apose）+ 时间动作（idle/wave/walk/nod/look）+ 原始 FK 骨骼驱动
- **控制服务** `twin_server.py`：在 Blender 内运行，HTTP `127.0.0.1:8123`；GUI / 无头双模式
- **Agent 客户端** `twin_client.py`（纯标准库，零依赖）+ 演示 `demo_agent.py`
- **API**：`GET /health` `/state`；`POST /pose` `/motion` `/bones` `/stop`
- **已验证**：pose / motion / FK 直驱 / 状态回读全链路通过（骨骼角度级精度）
- 证明图：`driven_tpose.png` / `driven_wave.png` / `driven_relax.png`

## 3. 意图图状态（`design/KG/SystemArchitecture.json`）

- **7 元素 / 3 关系 / 5 视图**，语义生命周期 `Aligned`，`validateSystemArchitecture` 通过
- 元素：
  - `project-overseer-001`（Business Actor 项目总管）
  - `abot-vision-001`（Goal — aBot 北极星）
  - `humanoid-model-001`（Artifact — 数字孪生模型）
  - `humanoid-build-001`（Business Process — 建模与操控流程）
  - `twin-control-001`（Application Service — 本地操控服务，Association → humanoid-model-001）
  - 模板 `1240` / `1249`（Grouping）
- 视图：`170 SystemArchitecture`（顶层）、`169/174/176`（模板）、`overseer-ltm-001`（项目总管长期记忆）

## 4. 如何运行

```powershell
# 1) 启动数字孪生服务（GUI 模式可实时看到模型动作）
blender --python scripts/blender_humanoid/twin_server.py
#    （无头测试：blender -b -P scripts/blender_humanoid/twin_server.py）

# 2) Agent 操控（同机任意进程）
python scripts/blender_humanoid/demo_agent.py
# 或代码： from twin_client import TwinClient; twin = TwinClient(); twin.set_pose("tpose") ...
```

## 5. 关键经验 / 坑（已记入记忆）

| 坑 | 原因 | 解决 |
|---|---|---|
| 无内置人形生成器 | Blender 5.1 无 makehuman 类插件 | bpy 程序化基元建模 |
| 引擎标识 | `BLENDER_EEVEE_NEXT` 不存在 | 用 `BLENDER_EEVEE` |
| 骨热权重失败 | 多基元重叠网格非流形 | `fix_weights()` 距离衰减兜底 |
| 无头 timers 不触发 | 无主事件循环 | 主循环手动调 `drive_once()` |
| `file_format="FFMPEG"` 被拒 | 该 build 无 ffmpeg 支持 | 系统 ffmpeg 9.0（PATH）单独编码 |
| 关系类型校验 | ArchiMate 3.2 矩阵限制（Application Service 不能 Serving Artifact） | 改用 Association |
| 项目边界 | aBot ≠ ArchGraph（北极星不同） | 已纠正图内污染，记忆已记录 |

## 6. 下一步（路线）

- **P2 物理与运动**：接入 MuJoCo / 物理引擎做真实物理（步态 / 平衡 / 碰撞），Blender 作渲染
- **P3 感知**：Blender 虚拟相机 → 渲染摄像头视角 → 视觉输入
- **P4 智能大脑**：Agent（LLM + ArchGraph 长期记忆）→ 自然语言指令 → 规划 → 通过 `TwinClient` 驱动动作
- **P5 硬件对齐**：模型形态贴近未来真机（关节 / 执行器 / 传感器）
