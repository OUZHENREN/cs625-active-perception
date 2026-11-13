# 仓库结构（2026-11-07）

> 取代归档的 `project_layout.md`。本文只描述**现在**的结构。

## 1. ROS 2 包（src/，12 个）

| 包 | 职责 |
|---|---|
| `cs625_ap_interfaces` | 消息/服务/动作定义 |
| `cs625_ap_description` | 相机与工具扩展（含 `my_end_effector_link`） |
| `cs625_sensor_adapter` | 相机输入归一化（品牌话题不出本包） |
| `cs625_target_perception` | 目标位姿与定位质量接口 |
| `cs625_view_generation` | 候选视角生成 |
| `cs625_view_evaluation` | 策略打分与 episode 协调 |
| `cs625_motion_adapter` | TF / IK / MoveIt 规划与执行适配 |
| `cs625_task_orchestrator` | 任务级证据与抓取流程 |
| `cs625_experiment_tools` | episode 汇总与实验指标 |
| `cs625_simulation` | Gazebo 世界/传感器/真值 |
| `cs625_bringup` | launch 组合与 profile |
| `cs625_rviz_panel` | RViz 停靠面板（六模式时代的 UI，已冻结） |

## 2. 脚本分层（scripts/）

```text
入口层
  cs625_panel.py                 两模式操作面板（安全 / 效率）
  resume_teaching.sh             一键续跑（重启驱动栈 → 激活控制器 → --resume）
  recover_driver_stack.sh        重启驱动栈并等待就绪
  recover_control.sh             轻量恢复（resend + 速度滑条）

示教链路（scripts/teaching/）
  teaching_sequence.py           0–18 步主链
  teaching_debug.py              效率模式（分阶段/改参/归档）
  plan_joint_locked.py           IK + 分支锁 + 速度契约的规划器
  （注：分段执行与插入脚本在 scripts/ 根目录，不在 teaching/ 下）
scripts/execute_assembly_chunks.py   分段执行 + 力矩看门狗
scripts/compliant_insert.py          插入段 effort 监督
  assembly_search.py             绕 TCP Y 的装配搜索
  straighten_in_slot.py          槽内分步转平 + 自由转动空间测量
  rvs_pose_client.py / rvs_fitness.py / pose_library.py / pose_math.py
  adjust_tcp.py / verify_pose.py / set_observation_pose.py / promote_baseline.py
  record_slot_reference.py       人工对齐后反算槽位姿

环境与自检
  model_environment.py / build_sim_world.py / apply_environment_collision.py
  control_health.py / preflight_motion.py / check_docs.py

实验与归档
  experiment_recorder.py / archive_experiment.py / sync_worklog.sh
```

## 3. 数据目录（runtime/，不入版本控制）

```text
environment/latest/model.json     环境模型（门禁要求 <2 h）
rvs_teaching/*.json               RVS 采集 T1/T2/T3（含 fitness）
slot_reference_verified.json      已验证槽位姿（插入权威）
pose_library.json                 位姿库
teaching_run_state.json           最后成功阶段（供 --resume）
debug_runs/<stamp>/               效率模式每次运行的完整归档
logs/                             驱动栈与运行日志
```

## 4. 不可改动

```text
~/cs625_legacy_underlay_jazzy/            师兄的只读栈（冻结，参考用）
~/cs625_legacy_sim_snapshot_20261024/     旧仿真快照
docs/archive/                             历史文档
```

> 来源：由归档原件 `project_layout.md` 更新而来（包边界沿用，脚本层与数据目录按现状重写）。
