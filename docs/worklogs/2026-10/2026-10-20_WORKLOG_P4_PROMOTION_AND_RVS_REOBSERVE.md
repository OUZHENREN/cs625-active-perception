---
title: "2026-10-20 WORKLOG：P4 决策逻辑提升为正式节点 + RVS 重观测"
date: 2026-10-20
type: WORKLOG
status: done（节点 PASS；RVS 重观测 PASS；真实 NG 仍走超时兜底）
---

# P4 提升：正式节点 + RVS 重观测

## 一、目标

把上一轮 `test/` 里的 P4 演示工具变成正式节点：

1. 决策逻辑与节点迁入 `cs625_task_orchestrator`；
2. 节点入口 `ros2 run cs625_task_orchestrator p4_decision_node`；
3. 重试时向 RVS 命令端口 7000 发送 `T3/T1,<机器人位姿>`，触发真实重观测；
4. 不修改师兄的 `tcp_server`，真实 `NG` 暂时仍以“等待位姿超时”进入决策层。

## 二、数据流

```text
/box_pre_grasp_tcp_pose（有效位姿） ─┐
/rvs/pose_status（NG 状态，可选）   ─┴→ p4_decision_node
        → /cs625/plan_to_pose_srv（只规划）
        → /p4/state（JSON）
        → /p4/reobserve_request（JSON）
        → 可选：TCP 7000 向 RVS 发送 T3/T1,<pose>
```

## 三、改动文件

| 文件 | 变化 |
|---|---|
| `src/cs625_task_orchestrator/cs625_task_orchestrator/p4_decision_logic.py` | 从 `test/` 迁入（纯逻辑，内容不变） |
| `src/cs625_task_orchestrator/cs625_task_orchestrator/p4_decision_node.py` | 新增正式节点 |
| `src/cs625_task_orchestrator/tests/test_p4_decision_logic.py` | 从 `test/` 迁入，改为包内导入 |
| `src/cs625_task_orchestrator/setup.py` | 新增 `p4_decision_node` 入口 |
| `src/cs625_task_orchestrator/package.xml` | 新增 `geometry_msgs` 依赖 |
| `test/p4_decision_demo.py` | 删除（功能已由正式节点覆盖，不保留重复脚本） |
| `test/p4_decision_logic.py`、`test/test_p4_decision_logic.py` | 移走，不在 `test/` 保留副本 |

## 四、节点参数

```text
pose_topic         默认 /box_pre_grasp_tcp_pose
status_topic       默认 /rvs/pose_status
plan_service       默认 /cs625/plan_to_pose_srv
pose_timeout_sec   默认 5.0
max_retries        默认 2
state_topic        默认 /p4/state
reobserve_topic    默认 /p4/reobserve_request
result_file        默认空（不写文件；测试时指定）
rvs_host           默认空（禁用 RVS 命令）
rvs_port           默认 7000
rvs_command        默认 T3
rvs_pose           默认空；需 6 个数 x,y,z,rx,ry,rz（mm/deg）
rvs_timeout_sec    默认 2.0
```

## 五、运行证据

### 1. 包测试

```bash
python3 -m pytest -q src/cs625_task_orchestrator/tests
# 35 passed（原有 30 + 新增 5）
```

> 说明：`colcon test --packages-select cs625_task_orchestrator` 调用的是
> `python3 -m unittest`，收集不到本仓库的 pytest 风格测试，返回 code 5；
> 这是测试运行器配置问题，不是本轮功能失败。功能验证用 pytest 直接执行。

### 2. 正式节点：重试耗尽（不启用 RVS）

```bash
ros2 run cs625_task_orchestrator p4_decision_node --ros-args \
  -p pose_timeout_sec:=3.0 -p max_retries:=1 \
  -p result_file:=/tmp/p4_formal_abort.json
```

结果：`VISION_NO_MATCH → RETRY 1/1 → 超时 → ABORT`；
`executed=false`，`planning_calls=0`。

### 3. 正式节点：重试时真实发送 RVS 命令

```bash
ros2 run cs625_task_orchestrator p4_decision_node --ros-args \
  -p pose_timeout_sec:=4.0 -p max_retries:=1 \
  -p rvs_host:=127.0.0.1 -p rvs_port:=7000 -p rvs_command:=T3 \
  -p 'rvs_pose:=419.966,353.329,732.167,-166.891,55.656,-122.934' \
  -p result_file:=/tmp/p4_formal_rvs.json
```

证据：

```text
节点日志：已向 RVS 发送重观测命令: T3,419.966,353.329,732.167,-166.891,55.656,-122.934
RVS 日志（16:10:37）：T3_TCPServerReceive 接收到: T3,419.966,...
RVS 日志（16:10:45）：CommonTCPClientNode::Process() > send: NG,POSE,0,0,0,0,0
节点随后第二次超时 → ABORT
```

说明：RVS 的 `NG` 仍被 `tcp_server` 丢弃，节点靠超时收敛；重观测动作本身已真实送达 RVS。

### 4. 正式节点：规划成功（不启用 RVS）

```bash
ros2 run cs625_task_orchestrator p4_decision_node --ros-args \
  -p pose_timeout_sec:=5.0 -p max_retries:=3 \
  -p result_file:=/tmp/p4_formal_success.json
# 期间通过 TCP 9999 发送 S3,-500,0,300,180,0,0
```

结果：`3 次规划尝试，2 次 PLAN_FAILED，最后规划成功（0.0130 s）`；
`decision=SUCCESS`，`executed=false`。

证据目录：`~/cs625_legacy_snapshot_20261020/p4_evidence_20261020/`
（`p4_formal_*.json/.log`、`p4_package_pytest.txt`、`rvs_reobserve_20261020.log`）。

## 六、限制

1. **`rvs_pose` 是静态参数**：目前用“上一次真实拍照位姿”作为重观测位姿；
   真机接入时应从机器人当前 TCP 位姿（FK 或控制器话题）实时获取，否则手眼转换会错。
2. **真实 `NG` 仍走超时兜底**：未修改 `tcp_server`；如需立即映射，需要加只读
   `/rvs/pose_status` 发布（3 行，但会改动师兄冻结代码，需所有者决定）。
3. **节点依赖师兄规划服务**：`/cs625/plan_to_pose_srv` 来自
   `~/cs625_legacy_underlay_jazzy`，运行前必须 source；`package.xml` 未声明该依赖，
   避免仓库构建依赖外部 underlay。
4. **无真机、无运动**：只调用规划服务；`executed=false`；
   未调用 `/cs625/execute_last_plan`。
5. **RVS 视觉仍未匹配**：`rvsGroup` 触发后会返回 `NG`；真实有效 S3 需要相机在
   拍摄位、场景中有可匹配的 box/slot。

## 七、能力层与关键链状态

```text
Capability layer
- environment and dependencies : PASS（Jazzy + vendor underlay + legacy underlay）
- robot model and simulation   : PASS（零位 joint_states 桩；未用 Gazebo）
- kinematics, control, planning: PASS（规划服务调用成功）
- vision, hand-eye, TF         : PARTIAL（RVS 相机在线，ICP 未匹配；P4 未使用视觉）
- active perception / NBV      : NOT STARTED
- real hardware and safety     : NOT STARTED（无真机、无运动）

Critical-chain status
- ros2_control -> joint_states      : PASS（零位桩，非真实驱动）
- RVS pose -> MoveIt plan           : PASS
- NG -> decision layer              : PASS（逻辑）；真实入口仍走超时
- decision -> RVS reobserve         : PASS（T3 命令真实送达 RVS 7000）
- plan -> execution                 : NOT STARTED（禁止执行）

Sim/real alignment
- reused common core : 是（cs625_task_orchestrator 包）
- sim entry point    : 无
- real entry point   : 无
- real execution     : 不允许；execute=false
- execute/confirmation gate : 默认关闭

Intentionally not done
- 未改师兄 tcp_server 的 NG 状态发布
- 未接机器人当前 TCP 位姿（rvs_pose 仍为静态参数）
- 未做 NBV/主动视觉
- 未连接真机、未执行运动
```

## 八、下一步

唯一推荐任务：**把重观测位姿改成实时值**——从 `/joint_states` + FK（或控制器话题）
取当前 TCP 位姿，替换静态 `rvs_pose`，然后在相机位于拍摄位时跑一次真实
`NG → T3 重观测 → 有效 S3 → 规划` 闭环。若你同意改 `tcp_server`，再顺手加
`/rvs/pose_status` 让 `NG` 立即映射。
