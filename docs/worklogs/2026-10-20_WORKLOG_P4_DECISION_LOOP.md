---
title: "2026-10-20 WORKLOG：P4 最小决策闭环（失败码 + 有界重试，只规划）"
date: 2026-10-20
type: WORKLOG
status: done（决策逻辑 PASS；状态入口待接入）
---

# P4：最小决策闭环

## 一、目标与范围

在 P3 已验证的“RVS 位姿 → MoveIt 规划”链路上，加一个**最小自动决策层**：

- 把视觉失败（`NG,POSE` 或等待位姿超时）映射为 `VISION_NO_MATCH`；
- 规划失败映射为 `PLAN_FAILED`；
- 每次失败消耗一次重试，超过 `max_retries` 后 `ABORT`，失败码 `RETRY_EXHAUSTED`；
- 只调用规划服务，**绝不调用执行服务**；不连接真机、不产生运动。

本轮不新增 package、不改师兄的 `tcp_server`、不做 NBV。

## 二、数据流

```text
RVS/TCP 9999
  → tcp_server
  → /box_pre_grasp_tcp_pose（有效位姿）        → P4 决策节点 → /cs625/plan_to_pose_srv（只规划）
  → /rvs/pose_status（可选，NG 状态；本轮由测试发布器模拟）
                                              → /p4/state（状态 JSON）
                                              → /p4/reobserve_request（请求重新观测）
```

## 三、改动文件

| 文件 | 作用 |
|---|---|
| `test/p4_decision_logic.py` | 纯决策状态机：失败码、重试计数、最终结论 |
| `test/test_p4_decision_logic.py` | 5 个单元测试 |
| `test/p4_decision_demo.py` | ROS 2 演示节点：订阅位姿/状态，调用规划服务，输出结果 JSON |

未改 `tcp_bridge`、未改 MoveIt 配置、未改 `cs625_task_orchestrator`。

## 四、失败码与重试语义

```text
VISION_NO_MATCH  视觉没匹配（NG,POSE 或等待位姿超时）
PLAN_FAILED      规划服务返回 success=False
RETRY_EXHAUSTED  重试次数用尽，最终 ABORT
SUCCESS          规划成功（未执行）
```

`max_retries=2` 表示最多再重新观测 2 次；每次失败进入 `RETRY` 并发布
`/p4/reobserve_request`（内容 `REOBSERVE`），供后续接 RVS 的 T3/T1 触发。

## 五、运行证据（只规划，未执行）

### A. 失败 → 重试 → 成功

命令（节选）：

```bash
python3 test/p4_decision_demo.py --pose-timeout 6 --max-retries 3 \
    --result-file /tmp/p4_success.json
# 超时后，经 TCP 9999 发送有效 S3：S3,-500,0,300,180,0,0
```

结果：

```text
VISION_NO_MATCH → RETRY 1/3
收到位姿 → PLAN_FAILED → RETRY 2/3
再次收到位姿 → 规划成功（planning_time≈0.0117 s）
decision=SUCCESS, attempts=2, retries=2, planning_calls=2,
failures=[VISION_NO_MATCH, PLAN_FAILED], executed=false
```

### B. 重试耗尽

```bash
python3 test/p4_decision_demo.py --pose-timeout 3 --max-retries 1 \
    --result-file /tmp/p4_abort.json
```

结果：

```text
超时 → VISION_NO_MATCH → RETRY 1/1
再次超时 → VISION_NO_MATCH → ABORT
decision=ABORT, retries=1, failures=[VISION_NO_MATCH, VISION_NO_MATCH],
final_code=RETRY_EXHAUSTED, planning_calls=0, executed=false
```

### C. `NG,POSE` 立即映射（不靠超时）

```bash
python3 test/p4_decision_demo.py --pose-timeout 30 --max-retries 1 \
    --result-file /tmp/p4_ng.json
# 向 /rvs/pose_status 发布两次 NG,POSE,0,0,0,0,0
```

结果：

```text
收到 NG → VISION_NO_MATCH → RETRY 1/1
再次 NG → VISION_NO_MATCH → ABORT
last_event=VISION_STATUS, final_code=RETRY_EXHAUSTED, executed=false
```

证据目录：`~/cs625_legacy_snapshot_20261020/p4_evidence_20261020/`
（三组 JSON + 日志、单测输出、契约检查输出）。

## 六、验证结果

```text
python3 -m pytest -q test/test_p4_decision_logic.py   → 5 passed
python3 test/contract_checks.py                       → Phase 0–6 static contract checks: PASS
```

## 七、限制与下一步

1. **真实 `NG` 状态入口尚未接入**：当前 `tcp_server` 丢弃 `NG,POSE`，所以真实 RVS
   的视觉失败在决策层表现为“等待位姿超时”（同样映射为 `VISION_NO_MATCH`）。
   要让 C 场景在真实 RVS 上成立，需要给 `tcp_server` 加一个只读状态话题，或做一层
   薄入口；这会改动师兄冻结代码，需所有者决定。
2. **决策逻辑尚未并入 `cs625_task_orchestrator`**：当前是 `test/` 下的演示工具；
   等接口稳定后再决定是否提升为正式节点，避免提前造抽象。
3. **重试动作尚未接 RVS**：`/p4/reobserve_request` 目前只发布字符串；后续可接
   RVS 命令端口 7000 的 `T3/T1` 触发，实现真正的“重观测”。
4. **规划器有随机失败**：本轮 A 场景第一次规划失败、第二次成功；这正是必须有
   有界重试的运行时证据，不是 bug。
5. **无真机、无运动**：本轮只用零位 `/joint_states` 桩 + MoveIt 规划；
   `execute:=false`，未调用 `/cs625/execute_last_plan`。

## 八、能力层与关键链状态

```text
Capability layer
- environment and dependencies : PASS（Jazzy + vendor underlay + legacy underlay）
- robot model and simulation   : PASS（零位 joint_states 桩；未用 Gazebo）
- kinematics, control, planning: PASS（/cs625/plan_to_pose_srv 规划成功）
- vision, hand-eye, TF         : PARTIAL（P3 相机在线，但 ICP 未匹配；P4 未使用视觉）
- active perception / NBV      : NOT STARTED
- real hardware and safety     : NOT STARTED（无真机、无运动）

Acceptance target
- objective   : NG/超时 → VISION_NO_MATCH；有界重试；只规划
- pass criteria: A 成功、B 重试耗尽、C NG 立即映射；executed=false
- evidence    : 三组 JSON/日志 + 5 个单测 + 契约检查 PASS

Critical-chain status
- URDF -> Gazebo entity             : NOT INVOLVED
- ros2_control -> joint_states      : PASS（零位桩，非真实驱动）
- base_link -> camera optical TF    : NOT INVOLVED
- RGB-D -> normalized sensor topics : NOT INVOLVED
- point cloud -> MoveIt scene       : NOT INVOLVED
- RVS pose -> MoveIt plan           : PASS（有效位姿规划成功）
- NG -> decision layer              : PASS（/rvs/pose_status 逻辑验证；
                                       真实 tcp_server 尚未发布该状态）
- NBV decision -> robot execution   : NOT STARTED

Sim/real alignment
- reused common core : 否（本轮是 test/ 演示工具）
- sim entry point    : 无
- real entry point   : 无
- real execution     : 不允许；未调用执行服务
- execute/confirmation gate : execute=false（默认）

Intentionally not done
- 未改师兄 tcp_server 的状态发布（需所有者决定）
- 未并入 cs625_task_orchestrator
- 未接 RVS 7000 的 T3/T1 重观测命令
- 未做 NBV/主动视觉
- 未连接真机、未执行运动
```
