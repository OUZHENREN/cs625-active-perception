# 归档区

> 归档 = **只读保留**，不是删除。这里的文档不再维护，只用于复盘与溯源。

## 2026-10-24-pre-six-modes/

2026-10-24 当天的工作把系统从"P4/P5/P7 分阶段验证"推进到"六类任务模式 + 强制环境建模 +
仿真与真机 1:1"，因此把此前的阶段性文档整体归档。归档时点：

- 六类模式（load/unload/full × safe/efficient）在 2026-10-24 落地；
- 环境模型门禁成为每个流程的前置条件；
- 仿真不再手工搭建，改为由真机实测生成（`scripts/build_sim_world.py`）；
- 旧的仿真世界与旧场景配置另存到 `~/cs625_legacy_sim_snapshot_20261024/`。

| 归档文件 | 原用途 | 为什么归档 |
|---|---|---|
| `README.md` | 旧的项目说明（P0–P1 视角，写着"真机未连接"） | 已被仓库根的新 `README.md` 取代 |
| `simulation.md` | 旧仿真世界的使用说明 | 旧世界已停用，新说明见 `docs/simulation.md` |
| `HANDOFF_2026-08-03_PHASE_0_1_SIM_RUNTIME_BLOCKED.md` | Phase 0/1 仿真运行阻塞的交接 | 阶段已完成 |
| `phase1_review_2026-08-04.md` | Phase 1 评审 | 阶段已完成 |
| `p5_experiment_protocol.md` / `minimum_showcase_experiment.md` | 仿真实验协议 | 被 `experiment_data_protocol.md` 与六类模式取代 |
| `p6_on_site_runbook.md` / `p6_simulation_temporary_acceptance.md` | 真机就绪与临时验收 | 真机流程已改为 `run_task.py` 六模式 |
| `p7_2_pose_estimator_handoff_contract.md` / `p7_3_geometry_nbv_contract.md` | P7 位姿估计/几何 NBV 的接口契约 | 接口已并入 `docs/interfaces.md` |
| `p7_attachment_assisted_grasp_protocol.md` / `p7_five_gate_protocol.md` | 五道门禁与附着抓取协议 | 门禁概念并入 `AGENTS.md` 与 `model_environment.py` |
| `p7_simulation_temporary_acceptance_2026-08-31.md` | 仿真临时验收记录 | 历史证据 |
| `diagnostics/` | 早期诊断脚本与截图（2.1 MB） | 一次性诊断，已被新工具取代 |

## 规则

- 归档**必须经所有者书面批准**，且要给出：源路径、目标路径、原因、被谁取代、
 受影响的反向链接、是否可逆（见 `AGENTS.md`「文档归档与取代」）。
- 不新建第二个归档目录；归档按 `YYYY-MM-DD-<主题>/` 命名。
- 归档文档里的路径可能已失效；**不要照着归档文档执行命令**。
- 需要复活某份归档内容时，复制出来改，不要就地编辑。
