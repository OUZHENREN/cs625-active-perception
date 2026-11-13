# 归档：P7 旧仿真 NBV 代码（2026-11-13）

> 归档 = **只读保留，不是删除**。本目录里的一切都不再被当前流程使用，
> 只用于复盘与溯源。**不要照着这里的脚本执行命令。**

## 归档原因（所有者 2026-11-13 指令）

P7 是"前六模式"时代的五道门仿真流水线（P7.1 传感器门 → P7.2 位姿估计 →
P7.3 几何 NBV → P7.4 MoveIt NBV → P7.5 附着抓取）。它的 NBV 打分、番茄遮挡
世界与配套证据件全部建立在**旧的 placeholder 相机与 YCB 番茄场景**上，与
2026-10-24 起"由真机实测生成仿真"的路线不再一致，所有者确认**不可用**，
要求归档。

真机侧结论（2026-11-13 交接文档）也收窄了 NBV：自动换视角改为**方案 A
（预教站位 + 按偏转角自动选站）**，旧 P7 的"几何打分搜索换位"不再演进。

## 归档内容与原始路径

| 归档位置 | 原路径 | 内容 |
|---|---|---|
| `test/` | `test/` | P7.1–P7.5 门禁脚本、NBV 打分/筛选、抓取生成、执行回执采集、`run_p7_*.sh` 启动器、`build_p7_contact_monitor.sh`、`p7_gazebo_contact_monitor.cpp`、对应 `test_p7_*.py` 测试 |
| `test_data/p7_3_severe_v5_target_from_p7_2.json` | `test/data/` | P7.3 严重遮挡层目标位姿夹具 |
| `src_cs625_bringup/config/` | `src/cs625_bringup/config/` | P7 观察/NBV 初始与落位关节配置 17 份（p7_1/p7_2/p7_3 各层 + `p7_static_grasp_sim.yaml`） |
| `src_cs625_bringup/tests/test_p7_gripper_config.py` | 同上 | 检验旧 P7 附着块的测试（块已从 xacro 移除） |
| `src_cs625_motion_adapter/` | `src/cs625_motion_adapter/` | `p7_arm_motion_adapter.py`、`test/p7_capture_reachable_candidates.py`、`test/p7_ik_link_probe.py`、`tests/test_p7_arm_motion_adapter.py` |
| `src_cs625_simulation/worlds/` | `src/cs625_simulation/worlds/` | 8 个旧番茄世界（light/occlusion v1–v5 + 相机轴诊断）——**已被 `scripts/build_sim_world.py` 生成的真机实测世界取代** |
| `src_cs625_task_orchestrator/p7_grasp_geometry.py` | `src/cs625_task_orchestrator/cs625_task_orchestrator/` | 旧抓取几何 |
| `src_cs625_task_orchestrator/test_p7_capture_tools.py`、`test_p7_grasp_geometry.py` | `src/cs625_task_orchestrator/tests/` | 对应测试 |

## 改名保留（仍在用，不归档）

| 新路径 | 原路径 | 为什么保留 |
|---|---|---|
| `src/cs625_task_orchestrator/cs625_task_orchestrator/gazebo_pose.py` | `p7_gazebo_pose.py` | `scripts/run_pick_and_place.py` 仿真档用 `parse_gz_model_pose` 读工件真值位姿 |
| `src/cs625_task_orchestrator/tests/test_gazebo_pose.py` | `test_p7_gazebo_pose.py` | 同上 |
| `src/cs625_bringup/config/cs625_safe_initial_positions.yaml` | `p7_safe_initial_positions.yaml` | `scripts/insertion_configuration_sweep.py` 的 IK 种子 |

## 保留不动的 P7 执行回执工具（仍被插入序验证链使用）

`run_insertion_sequence.sh` / `assemble_insertion_episode.py` 与
`test/contract_checks.py` 的必需文件清单仍在用这些件，因此**未归档**：

- `src/cs625_task_orchestrator/cs625_task_orchestrator/p7_{episode_contract,episode_recorder,gripper_adapter,attachment_adapter}.py` 及其 `tests/test_p7_{episode_contract,gripper_adapter,attachment_adapter}.py`（`setup.py` 仍挂着 recorder/gripper/attachment 三个控制台入口）
- `test/p7_execute_pose_capture.py`、`test/p7_capture_gazebo_contacts.py`、`test/p7_attachment_smoke_publisher.py`
- `test/data/p7_smoke_evidence.{json,yaml}`、`test/data/p7_gripper_close_command.json`

后续建议（需所有者决定）：把这些执行件改名为非 P7 名称并迁入六模式命名体系。

## 同步清理的共享文件（2026-11-13 同一提交）

- `src/cs625_ap_description/urdf/cs625_active_perception.urdf.xacro`：删除死 P7
  附着块（`p7_attachment_enabled` 参数 + `/p7/attachment/*` DetachableJoint，
  child 是旧世界才有的 `target_object`）；删除仅 P7 代码使用的
  `gripper_base_link → p7_grasp_center_link` 固定关节（仿真专用 frame）。
  当前仿真的夹持附着由 `scripts/sim_robot_description.py` 注入的
  `/cs625/carry/*` DetachableJoint（child `shielding_module`）承担，不受影响。
- `src/cs625_bringup/launch/sim_control.launch.py`：不再读 `CS625_P7_ATTACHMENT_ENABLED`、不再传 `p7_attachment_enabled`。
- `sim_base.launch.py` / `sim_active_localization.launch.py` / `sim_control.launch.py` 的 `initial_detach_topic` 默认值改为现行 `/cs625/carry/detach`。
- `sim_task_scene.launch.py` 提示语、`cs625_moveit.rviz` 的 `p7_grasp_center_link` 显示项同步更新。
- `test/contract_checks.py`：P7.1 初始位姿配置改为从本归档目录读取；提示文案更新。

## 受影响的反向链接（已同步修正）

- `test/contract_checks.py`（读取路径与提示）
- `test/README.md`、`docs/reference/simulation.md`（MoveIt touch_links 段）
- `docs/archive/README.md`（本表）
- `scripts/insertion_configuration_sweep.py`、`scripts/run_pick_and_place.py`（改名模块引用）

## 可逆性

全部文件经 `git mv` 移动，历史完整；恢复 = `git mv` 回原路径并撤销共享文件
改动（见同一提交）。历史文档 `docs/archive/2026-10-24-pre-six-modes/`（P7 文档）
保持原样、未动。

## 附：P7 运行时证据（同日一并归档）

所有者 2026-11-13 指示：`docs/evidence/` 里大段的 P7 仿真证据（129 个
`p7_*` 目录，五道门各轮 capture/诊断/门总结，约 1100 个文件）同样归档，
移入本目录的 `evidence/` 子目录，只读保留、仅供溯源，不再作为当前验收证据
引用。`docs/evidence/README.md` 的 P7 运行索引已改为指向本处；
`minimum_showcase_matrix_20260906_r1` 等非 P7 证据留在原位。
