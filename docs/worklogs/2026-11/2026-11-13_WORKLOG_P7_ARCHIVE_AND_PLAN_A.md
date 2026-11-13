# 2026-11-13 工作日志：P7 代码归档 + 方案 A（预教站位自动选站）实现

> 交接来源：`docs/worklogs/2026-11/2026-11-13_HANDOFF_VISION_MILESTONE.md`（下一段对话先读它）。
> 本轮环境：真机已关机（所有者下班），全部验证**离线**完成；**没有发生任何真机运动**。

## 0.1 复盘补记（22:2x，应所有者要求复查）

复查结论：**本轮自己的改动全部真实通过；复查过程中发现并修复了一串"本轮之前就存在"
的合同检查失败**（与归档无关），并如实记录：

```text
已复核并再次通过（fresh 重跑）：
· pytest test/               275 collected / 275 passed / 0 错误 0 跳过
· 包级测试（orchestrator/bringup/motion_adapter tests）  29 passed
· test/contract_checks.py 独立运行  PASS（带与不带 ROS 环境各一次）
· xacro 展开 exit 0，注入后恰好 1 个 /cs625/carry DetachableJoint
· --auto-station 决策（真机测量文件）与初测一致
· git：HEAD=main=origin/main=429097f（复查后追加 2 提交），工作树干净，
  推送内容无 runtime/、无敏感文件
· 全仓库悬空引用终扫：0 命中

本轮之前就坏、复查时发现并修复（提交 f57e6a2）：
① contract_checks 的 required_files 还指着 2026-11-07 文档重组前的 7 个旧路径
   （docs/*.md → docs/reference/ 或 docs/archive/2026-11-07-pre-reorg/）——
   该检查自 11-07 起就一直是红的，与今晚归档无关。
② start_real_stack.sh / recover_driver_stack.sh 里有 192.168.* 兜底字面量
   （违反 AGENTS.md"禁止硬编码 IP"）。两脚本本就读 ~/.cs625_local.env，
   改为：值放本地 env（已补 CS625_HOST_IP=192.168.1.100），脚本缺失即报错。
③ 字面量扫描范围修正：docs/reference（真机实测记录）/ docs/manuals（现场报告）
   按 worklog/archive 同理由豁免；代码里的禁令不变。

数量口径说明：交接文档记"296 passed"。当前 275 = 296 − 归档掉的 P7 用例
（24 个 test 函数 + 2 个参数化展开）＋ 新增 9 个，差 4 个用例无法在 worktree
里精确复现（ROS 环境解析到主仓库 install，历史口径不可完全重建）；
该差异只影响历史数字，不影响当前套件全绿、零错误的事实。

仍未验证（如实）：方案 A 的执行路径（plan_pose_ik 换位 + 精测闭环）需要真机；
Gazebo 运行时（carry attach 行为）只做了静态展开验证，未实际启动；colcon build
未在归档后重跑（包结构只动了 setup.py 入口清单，语法已验证）。
```

## 0. 本轮解决的两个问题

```text
①（所有者指令）P7 旧仿真 NBV 代码全部归档 —— 旧五道门流水线（番茄世界、几何 NBV、
   门禁脚本/配置）不可用，移入 docs/archive/2026-11-13-p7-nbv-legacy-code/。
②（交接文档 §6 下一步）实现方案 A：预教站位 + 按偏转角自动选站 —— nbv:workpiece
   阶段不再硬编码 0° 视角，改为"粗观测偏转角 → 查表选站 → 与 0° 相对视角推导
   交叉核对 → plan_pose_ik 带安全闸执行；查不到/核对不过/需大幅改构型一律原地精测"。
```

## 1. P7 归档的精确范围（动了哪些文件）

### 1.1 移入 `docs/archive/2026-11-13-p7-nbv-legacy-code/`（git mv，历史完整）

```text
test/                    58 个 p7 文件（P7.1–P7.5 门禁脚本、NBV 打分/筛选、抓取生成、
                         执行回执采集、run_p7_*.sh、test_p7_*.py 测试）
test/data/p7_3_severe_v5_target_from_p7_2.json
src/cs625_bringup/config/ 17 份 p7_*.yaml（观察/NBV 初始与落位关节配置 + static_grasp）
src/cs625_bringup/tests/test_p7_gripper_config.py（测旧附着块，块已删）
src/cs625_motion_adapter/ p7_arm_motion_adapter.py + 3 个配套脚本/测试
src/cs625_simulation/worlds/ 9 个旧番茄世界（已被 build_sim_world.py 生成的世界取代）
src/cs625_task_orchestrator/ p7_grasp_geometry.py + 2 个配套测试
```

归档目录内有完整 README（源路径、原因、保留/改名清单、可逆性）。

### 1.2 改名保留（当前仍在用，不归档）

```text
p7_gazebo_pose.py            → cs625_task_orchestrator/gazebo_pose.py
                               （run_pick_and_place.py 仿真档用它读工件真值位姿）
tests/test_p7_gazebo_pose.py → tests/test_gazebo_pose.py
p7_safe_initial_positions.yaml → cs625_bringup/config/cs625_safe_initial_positions.yaml
                               （insertion_configuration_sweep.py 的 IK 种子）
```

### 1.3 保留不动的 P7 执行回执工具（仍被插入序验证链使用）

`run_insertion_sequence.sh` / `assemble_insertion_episode.py` / `test/contract_checks.py`
必需文件清单仍在用：`p7_{episode_contract,episode_recorder,gripper_adapter,attachment_adapter}.py`
及其测试、`test/p7_{execute_pose_capture,capture_gazebo_contacts,attachment_smoke_publisher}.py`、
`test/data/p7_{smoke_evidence.yaml,smoke_evidence.json,gripper_close_command.json}`。
**后续建议（需所有者决定）**：把这些执行件改名迁入六模式命名体系。

### 1.4 共享文件清理（死 P7 附着链）

```text
URDF xacro：删除 p7_attachment_enabled 参数与 /p7/attachment/* DetachableJoint 块
            （child 是旧世界才有的 target_object，当前世界为 shielding_module）；
            删除仅 P7 代码使用的固定关节 gripper_base_link → p7_grasp_center_link
            （仿真专用 frame，SRDF 无引用，六模式不用）。
            当前仿真的夹持附着 = sim_robot_description.py 注入的 /cs625/carry/*，
            不受影响（已用 xacro 展开 + 注入链验证）。
launch：sim_control 不再读 CS625_P7_ATTACHMENT_ENABLED、不再传 p7_attachment_enabled；
        sim_base / sim_active_localization / sim_control 的 initial_detach_topic
        默认值改为现行 /cs625/carry/detach；sim_task_scene 提示语更新；sim_cell 注释更新。
config：cs625_moveit.rviz 移除 4 处 p7_grasp_center_link 显示项。
test/contract_checks.py：P7.1 初始位姿配置改为从归档目录读取；提示文案更新。
docs：test/README.md 重写；docs/reference/simulation.md touch_links 段更新；
      docs/archive/README.md 增行。
```

## 2. 方案 A 实现（改了哪些文件）

```text
scripts/nbv_views.py
  + pose_from_payload() / yaw_degrees_between()（yaw 符号约定，见 §3 验证）
  + station_plan()：选站 + 0° 相对视角推导交叉核对 + 一致性门（纯数学，不碰 ROS）
  ~ select_known_view()：容差缺省时取查表 tolerance_deg
scripts/nbv_from_reference.py
  + --auto-station 模式：决策打印 → 站位绝对位姿 → plan_pose_ik.py 执行（全套安全闸）
    退出码语义：0 = 决定并执行成功；6 = 拒绝自动换位（无站位/工件已移动/需大幅改构型）
scripts/run_pick_and_place.py
  ~ nbv:workpiece 阶段：改用 --auto-station；plan-only 时跑纯数学选站 dry-run 并打印决定
config/tasks/common.yaml
  + workpiece_views 的 '+11' 与 '+11_left8' 标注 family: taught（所有者手摆的独立站位，
    不与 0° 推导做一致性门）；其余站位默认 relative_zero 族
test/test_nbv_station_selection.py（新，9 个用例）
  覆盖：yaw 符号约定、查表选站（命中/越界/非数字 key 跳过）、与推导一致 → 放行、
  无站位覆盖 → 拒绝、工件移动 >40 mm → 拒绝（relative_zero 门）、taught 族不设门、
  缺 0° 基准 → 报错、真机测量文件回归（skipif 保护）。
```

安全语义（与交接文档 §5 红线一致）：**查不到站位 / 推导与存储差 >40 mm / 需要大幅改
构型 → 一律不自动换位，继续原地精测**；执行只走 `plan_pose_ik.py`（workspace_bounds +
腕部/单关节限幅 + 抽样碰撞 + 当前状态接触硬闸）。没有任何视角搜索或转腕动作。

## 3. 关键验证（全部离线，真机未开）

```bash
source scripts/source_dev_env.sh --full
python3 -m pytest test/ -q                    # 275 passed（归档前 296，减少的是归档掉的 P7 测试）
python3 scripts/run_task.py --scope load --mode efficient --world sim --dry-run   # exit 0，相位表含 nbv:workpiece
python3 scripts/nbv_from_reference.py --auto-station \
    --reference-measurement runtime/measurements/workpiece_zero.json \
    --current-measurement runtime/measurements/workpiece_coarse.json
```

yaw 符号约定（用今天真机留下的测量文件核对）：
```text
workpiece_minus30.json → yaw −29.57°（measure_object 报 −29.6°）
workpiece_minus45.json → −42.42°（报 −42.6°）
workpiece_coarse.json  → +11.26°（交接文档报 +11.2°）   ⇒ 约定正确
```

选站实测（真机测量文件，dry 决策）：
```text
+11.3° → 选 '+11'（taught 族，放行；推导差 180 mm 属正常，不同视角族）
−29.6° → 选 '−30'（relative_zero 族，推导与存储仅差 0.9 mm / 0.0° —— 数学自洽）
−42.4° → 选 '−45'（差 2.6° 在容差内，推导差 4.3 mm）
+90°（合成）→ 无站位覆盖 → exit 6 拒绝
−30° 但工件平移 64 mm（合成）→ 一致性门触发（157 mm）→ exit 6 拒绝
```

xacro 展开对照（git HEAD 旧版 vs 新版）：差异恰好等于预期改动（P7 附着块、
p7_grasp_center_link 移除 + 注释更新），其余逐行一致；
`sim_robot_description.py` 注入链现在恰好注入 1 个 `/cs625/carry/*` DetachableJoint。

## 4. 能力层与状态（诚实）

```text
Capability layer: vision、手眼标定与 TF / active perception（方案 A 基线的代码层）
  环境与依赖      PASS（离线：pytest、xacro、dry-run）
  机器人模型/仿真  PASS（xacro 展开 + 附着注入链验证；未跑 Gazebo）
  真机运动        NOT EXECUTED —— 真机已关机，方案 A 的执行路径从未在真机跑过
Sim/real alignment: 共用同一份 workpiece_views 与选站数学；执行分别走
  plan_pose_ik（真机）。方案 A 本身 = 主动视觉研究里的 fixed/predefined 基线策略。
Intentionally not done（交接文档 §6 其余项）:
  · 真机验证方案 A 的移动+精测闭环（需要真机 + 所有者在场）；
  · 规划场景无环境（桌子/夹具对 MoveIt 不可见）——自动换位前必须先修好；
  · 精观测"最多重试一次"（需要相机，真机）；
  · +11_left8 的自动偏好选择（目前只会选数字 key 的 +11；两个站位都实测通过）。
```

## 5. 复现命令（下一段对话）

```bash
source scripts/source_dev_env.sh --full
python3 -m pytest test/ -q                                  # 275 passed
python3 scripts/nbv_from_reference.py --auto-station \
    --reference-measurement runtime/measurements/workpiece_zero.json \
    --current-measurement runtime/measurements/workpiece_coarse.json   # 纯数学决定，无运动
# 真机在场时：
python3 scripts/run_task.py --scope load --mode efficient --world real --confirm \
    --until measure:workpiece_fine
```
