# 源码与参数的权威性分级（Source authority）

> 本文回答一个问题：**同一个量（位姿、内外参、速度、世界常量）有多份记录时，以哪一份为准。**
> 项目所有者 2026-10-24 的明确要求：以最近的真机复跑环境为准；但**所有历史源码必须保留**，
> 不允许整批抹除，方便后续抽调和复盘。

## 1. 分级定义

| 级别 | 含义 | 使用规则 |
|---|---|---|
| **A：权威** | 2026-10-22 ~ 2026-10-24 在真机 CS625 上实测、可追溯到命令与输出 | 任何与之冲突的旧值都以 A 为准 |
| **B：可复用，需复核** | 师兄工程里经过实机验证的接口（话题名、服务名、消息类型、attach 机制） | 可以直接用；但接口版本变化（Humble→Jazzy）时要复核 |
| **C：不可信，仅保留复盘** | 早期仿真环境参数、旧相机内外参、旧场景位姿、未经真机验证的估计值 | **不删除**，就地标注；不得用于真机运动 |

## 2. A 级清单（以真机实测为准）

| 内容 | 存放位置 | 依据 |
|---|---|---|
| 手眼外参 `T_tcp_cam` | `config/handeye/tcp_to_camera.json` | 2026-10-23 标定 + 2026-10-24 ICP 复核 |
| 相机内参（fx 931.47 / fy 931.60 / cx 648.70 / cy 449.32 @1280×960） | `config/real/camera_intrinsics.json` | 2026-10-24 实机读数 |
| 插槽固定位姿 `slot_origin(+q)`、槽内工件 `box_in_slot` | `config/tasks/common.yaml`（源自 `config/unload_real.yaml`） | 师兄 RVS 标定 + 机械固定 |
| 工件桌位姿（模型原点，精观测位 3 帧共识） | `config/tasks/common.yaml` 注释 + `docs/agent_memory.md §121` | 2026-10-24 10:15，3 帧离散 1.0 mm |
| 工件几何中心偏移 `[-0.208, 0.066, -0.011]` | `scripts/icp_pose_once.py` / `model_environment.py` | CAD 模型属性 |
| 可复用位姿（baseline / transit / staging / pre_insert / insert / pre_grasp / grasp / place_*） | `config/tasks/common.yaml` | 2026-10-22 真机跑通 |
| 观测包络 / 工作空间盒 / 夹爪间距下限 0.38 m | `scripts/capture_dataset.py` 常量 | 2026-10-24 真机碰撞后实测规则 |
| 逐阶段速度档案（safe） | `config/tasks/load_safe.yaml` 等 | 见下方 §4 的"证明值"表 |
| 环境模型（表格 z、体素、位姿） | `runtime/environment/<ts>/model.json` | 每轮重新生成 |
| 机器人能否运动的判据 | `docs/agent_memory.md §121` | safety_mode=1 / robot_mode=7 / sockets=True 三样同时成立 |

## 3. B 级清单（师兄接口，只读引用）

| 接口 | 位置 | 用途 |
|---|---|---|
| `tcp_bridge`（`/tcp_server`） | `~/cs625_legacy_underlay_jazzy` | 与 RVS/上位机的目标位姿交换 |
| `environment_collision_generator`（`/collision_object_publisher`） | 同上 | 插槽/工件的 attach 碰撞体（`touch_links`） |
| `cs625_compliant_placement` | 同上 | 柔性放置节点 |
| `cs625_state_monitor`（`/cs625/raw_state`） | 同上 | 机器人原始状态 |

规则（ADR-004）：**不修改、不复制、不删除**师兄代码；整合只通过话题/服务接口。

## 4. C 级清单（不可信，仅保留）

| 内容 | 位置 | 为什么不可信 |
|---|---|---|
| 旧仿真世界/场景参数 | `src/cs625_simulation/**` | 建库时的环境，与当前真机桌面/插槽/幕布不符 |
| 旧相机内外参 | `src/cs625_simulation/config/**` | 非本台相机实测 |
| 旧场景位姿 / 旧场景模型 | `config/scenes/**`、`docs/evidence/**` 的旧 figure | 未经当前真机复核 |
| `config/pick_place_real.yaml` 中 `pre_grasp`/`grasp` 的阶段速度 0.08/0.04 | 该文件本身 | **旧 runner 忽略这两个阶段档案**，实际执行是 0.02/0.01；新配置已改为实际值 |
| 早期 `p7_*` 合同/门禁参数 | `src/cs625_bringup/config/p7_*` | 仿真阶段产物，真机口径未重标定 |

**保留约定**：C 级内容**一条都不删**。需要标注时，在数值旁加注释
`# UNRELIABLE: superseded by 2026-10-24 real measurement`，或让新值以新文件形式并存。
归档 / 移动 / 删除任何文档都必须先取得所有者书面批准（AGENTS.md「文档归档」）。

## 5. 冲突解决顺序

```text
真机实测（A）  >  师兄已验证接口（B）  >  设计文档  >  旧仿真参数（C）  >  推断
```

若同一量出现两份 A 级记录（例如两次不同日期的真机测量），以**较新的测量**为准，
并把旧值以注释形式保留在该文件里，注明日期与来源。

## 6. 速度值的"证明值"（防止校验器比实践更严或更松）

`scripts/run_pick_and_place.py` 的 `validate()` 会对逐阶段速度做硬校验。上限不是拍脑袋定的，
而是"这台机器上真跑过的最大值"：

| 阶段类 | 证明过的最大值 | 出处 |
|---|---|---|
| 巡航（transit / baseline） | 0.08 | 2026-10-22 真机 |
| 一般装配（staging / lift / pre_insert / withdraw） | 0.05（staging）/ 0.03（pre_insert、withdraw、lift） | 2026-10-22 真机 |
| 槽内紧间隙（insert / straighten / d2c / d4 / d5_extract） | **0.005**，且禁止"快巡航+慢尾段" | 2026-10-22 真机（单边间隙约 1 mm） |
| 效率模式巡航 | 0.12（**新设，待真机验证**） | 所有者 2026-10-24 批准 |

`d2b_entrance_tilted`（空夹爪进入槽口）**故意不算紧间隙**：它在真机上以 0.03 跑过，
校验器不应比实践更严。
