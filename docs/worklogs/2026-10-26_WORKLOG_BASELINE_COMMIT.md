# 2026-10-26 WORKLOG — 真机仍然不可达；把前两轮的成果提交成干净基线

> 接 `2026-10-26_HANDOFF_REAL_RETEST.md`。本轮**没有启动 Gazebo、没有真机运动**：
> 所有者选择的第一件事是"先把压着的工作树提交成基线"。

## 1. 一句话结果

真机 `192.168.1.200` 不可达（不是 WSL 网络问题），因此交接文档 §9 的真机重测无法开工；
把工作树里积压的 **22 个改动文件 + 14 个新文件**按功能分成 4 次提交入库，工作树现在是干净的，
`pytest 230 passed`、`contract_checks Phase 0–6 PASS`。

## 2. 真机状态（可复现的判断依据）

```bash
ping -c 2 -W 1 192.168.1.200            # → 2 packets transmitted, 0 received, 100% packet loss
ip -brief addr                          # → eth1 192.168.1.100/24 UP（就在机器人那段网里）
for i in $(seq 1 254); do ping -c1 -W1 192.168.1.$i & done; wait
                                        # → 只有 192.168.1.100（本机）与 192.168.1.120 应答
ip neigh show dev eth1 | grep 192.168.1.200   # → INCOMPLETE（ARP 都没学上）
```

结论：网卡、网段、路由都正常，机器人侧没有应答 ⇒ 机器人未上电 / 网线未接 / IP 被改过。
**这不是驱动或 ROS 的问题，先解决物理连接再谈启动。**

## 3. 提交前审查（做了什么、结论）

| 检查 | 命令 | 结果 |
|---|---|---|
| 离线测试 | `python3 -m pytest test/ -q` | **230 passed**（11.10 s） |
| 契约静态检查 | `python3 test/contract_checks.py` | **Phase 0–6 PASS** |
| 空白/冲突标记 | `git diff --check` | 干净（exit 0） |
| 密钥/本机绝对路径 | 对全量 diff + 新文件 grep `/home/oy`、`192.168.`、`password|token|secret` | 只有 `docs/evidence/experiment_index.md` 里的归档路径（既有约定，非本次引入） |
| 新增大文件的依据 | grep `tool0_vision_body|gripper_arm_left|gripper_arm_right` | 被 `scripts/sim_robot_description.py` 的物理夹爪注入路径引用（默认关闭的 `CS625_SIM_GRIPPER=0`） |
| 生成物是否手工改过 | 用 `build_sim_world.py` 重生成到 `/tmp` 再 diff | 只差"数据来源"注释行（见 §5.1），生成器与生成物一致 |

## 4. 提交清单（4 次，逻辑分组）

| 提交 | 内容 | 关键文件 |
|---|---|---|
| `dd78dec` feat(sim) | 仿真归位到厂商参考模型；`DetachableJoint` 表达"夹住"；桌面 xy 用显式标注的近似值 | `scripts/sim_robot_description.py`、`scripts/sim_carry_workpiece.py`、`sim_*`/`real_moveit` launch、`config/cell_tables.yaml`、世界/场景生成物、3 个 CAD 拆分网格、`test_sim_*.py` |
| `1f073bd` feat(tools) | 只读观测工具：环境点云发布、任务实时状态页 | `scripts/publish_environment_point_cloud.py`、`scripts/live_status_page.py`、`environment_point_cloud.rviz`、README 一节 |
| `c2b8d54` chore(vscode) | DSH Web 任务在 `bash -lc` 下找不到 nvm 的 node/dsh | `.vscode/tasks.json` |
| `7373708` docs(worklog) | 353 mm TCP、仿真参考模型复盘、真机重测交接 | `docs/worklogs/2026-10-25_WORKLOG_TCP_FRAME_353MM.md` 等 3 份 |

`git status --short` → 空。**注意：提交只是"存档"，没有改任何运行逻辑**；
上面 4 次提交的内容上一轮已经跑过并记录，本轮没有新增运行态结论。

## 5. 本轮发现、但**没有**擅自处理的两件事

### 5.1 生成物里的"数据来源"指向旧环境模型（几何没变）

```
tracked : runtime/environment/20261025_091250/model.json
latest  : runtime/environment/20261026_030037/model.json
```

用当前模型重生成到 `/tmp/regen_world.sdf` / `/tmp/regen_scene.yaml` 再 diff，
**只有这两行注释和三处路径字符串不同，几何/位姿/尺寸逐字相同**。
所以这是"出处标注过期"，不是对齐误差。按工程规矩（生成物只能由生成器改）它该重生成一次刷新出处，
但那会改动两个已入库的生成文件——留给所有者决定，本轮不动。

### 5.2 `scripts/live_status_page.py` 还没有文档入口

它是有用的（真机/仿真实跑时能一直开着看阶段进度 + 六关节 + 日志尾巴），
但目前只在脚本自己的 docstring 里写了用法，README 与 `.vscode/tasks.json` 都没有入口。
下一轮顺手补一行 README 或加一个任务编号即可，不需要它单独立项。

## 6. Capability layer

本轮**没有运行态变化**，以下状态沿用 `2026-10-26_WORKLOG_SIM_REFERENCE_MODEL_AND_ATTACH.md`
的实测结论，未在本轮复验：

```
- environment and dependencies      PASS      ROS Jazzy 链 source_dev_env.sh --verify 全绿
- robot model and simulation        PARTIAL   仿真描述已归位厂商 CAD；insert 未通过（12 mm 夹持关系误差 vs 1.2 mm 槽隙）
- kinematics, control and planning  PARTIAL   仿真 15 阶段里 14 个 PASS；真机 --plan-only 上一轮未跑（不可达）
- vision, hand-eye calibration/TF   PARTIAL   厂商描述的相机是占位（640×480/15 Hz）；真机 hand-eye 参数未在本轮核验
- active perception / NBV           NOT STARTED
- real-hardware integration/safety  BLOCKED   真机不可达，四项健康检查无法执行
```

## 7. Acceptance target

- 本轮目标：工作树干净、离线检查全绿、成果可复现。
- 达成：`git status` 空、`pytest 230 passed`、`contract_checks PASS`。
- 未达成（不在本轮范围）：真机 load 重测（机器人不可达）。
- 用了 Gazebo：**否**（本轮未启动）。假硬件：否。**真机运动：否。**

## 8. Critical-chain status（本轮未推进，状态沿用并注明依据）

```
URDF -> Gazebo entity              PASS      2026-10-26 仿真：厂商 CAD 实体生成成功，工具碰撞为闭合态代理
ros2_control -> joint_states        PARTIAL   2026-10-26 末次启动 joint_state_broadcaster 竞态未配置（记录在案，未修）
base_link -> camera optical TF      PARTIAL   厂商描述缺 camera_*_optical_frame（已知代价，未在本轮处理）
RGB-D -> normalized sensor topics   PARTIAL   厂商相机为 640×480/15 Hz 占位
point cloud -> MoveIt planning scene NOT STARTED 本轮未验（scene applier 存在但未跑）
NBV decision -> robot execution     NOT STARTED
```

## 9. Sim/real alignment

```
共用应用核心     是（run_task.py + run_pick_and_place.py；本轮提交未改真机路径逻辑）
仿真入口        sim_cell.launch.py（默认厂商描述）
真机入口        real_moveit.launch.py（默认已改成厂商描述）
真机执行        未允许（不可达）
门禁状态        runtime/environment/latest = 20261026_030037，age > 2 h，下次真机开工必须现场重采
```

## 10. Intentionally not done

- 未重生成世界/场景文件（§5.1，改动生成物需要所有者拍板）。
- 未给 `live_status_page.py` 补 README/任务入口（§5.2）。
- 未改 `config/tasks/*.yaml` 的位姿/速度、未改厂商包、未改工件质量/重力。
- 未启动 Gazebo（本轮不需要，且会长跑占满 CPU）。
- 未把上一轮的交接/复盘文档归档（工具链会自动同步到 Obsidian，归档属于所有者决定）。

## 11. 下一轮唯一推荐任务

**机器人上电、`ping 192.168.1.200` 通之后，按 `2026-10-26_HANDOFF_REAL_RETEST.md` §3 顺序做真机开工**：
四项健康检查全 True → 现场重采环境模型（否则门禁 exit code 2）→ `--plan-only` 看 19 段是否全 PLANNED
→ 再决定是否 `--confirm --allow-release` 真动。
在此之前不要在仿真里继续改判据——仿真能验的（可达性、观测前缀、搬运）已经验完，
剩下的"真机夹紧 + 1 mm 间隙插入"只能在真机上验。
