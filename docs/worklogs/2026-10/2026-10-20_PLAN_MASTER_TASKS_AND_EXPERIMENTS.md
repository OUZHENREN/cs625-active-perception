---
title: "2026-10-20 CS625 课题总方案 · 当前任务总览 · 实验方案设计（证据分级 v1.0）"
date: 2026-10-20
version: v1.0
status: 待所有者确认（确认前不归档、不删除任何旧版）
author: 欧阳志海（哈尔滨工业大学 0855 机械专硕）
evidence_policy: "A=仓库可验证；B=一手工作日志；C=待验证假设；D=目标值/计划值"
supersedes: 无。本版与《2026-09-18 开题答辩后研究路线调整与最新正式版》《2026-09-19 任务总览与仿真实验设计 v2》《2026-10-19 破局实操手册》并存，职责不同。
tags: [CS625, 总方案, 任务总览, 实验设计, 证据分级, 2026-10-20]
---

# 2026-10-20 CS625 课题总方案 · 当前任务总览 · 实验方案设计（v1.0）

> **这份文档为什么存在**
> 10-19 的两份计划（破局总方案 v4.0、仿真破局实操手册）方向是对的，但把"计划推演"写成了"仓库事实"：有的命令会启动两套仿真，有的适配器并不存在，有的估计器不能用于新对象。本版只做三件事：**把事实和假设分开、把下一步收敛到仓库真实卡点、把实验设计写成可预注册的协议**。
> 每条结论都带证据级别：**[A]** 仓库可验证（文件/命令/commit）；**[B]** 一手工作日志；**[C]** 待验证假设；**[D]** 目标值/计划值。凡标 C/D 的内容，不得写进论文结论句。

---

## 0. 使用规则与本版纠错

### 0.1 使用规则

1. 每周只认**一个硬产出**（一张表、一份 CSV、一次可复现的 episode），不认"忙了一周"。
2. 任何命令先用 `source scripts/source_dev_env.sh` 进入唯一环境；不在两个发行版之间混用。
3. 本版不覆盖：求职/选调/小论文排版（各有独立文档）；本版只覆盖 **CS625 课题的工程、实验与论文数据**。
4. **本版不归档任何旧版**。是否归档旧文档，只由所有者本人书面确认。

### 0.2 本版对 10-19 版本的 7 处纠错

| # | 10-19 版本的说法 | 仓库事实与证据 | 本版修正 |
|---|---|---|---|
| 1 | 手册 Level 1：先起 `sim_base` 再起 `sim_task_scene` | `sim_task_scene` 内部已 include `sim_base`；照做会起两套 Gazebo+MoveIt **[A]** `src/cs625_bringup/launch/sim_task_scene.launch.py:79-90`、`README.md:70-79` | 只起一条 `sim_task_scene` |
| 2 | 数据流含 `Gazebo contact → cs625_force_adapter` | 11 个包里**没有** force adapter；9-19 v2 §2.6 只是"新增"提议 **[A]** `src/` 包清单 | 接触先用 Gazebo contact topic 直采；force adapter 列为待实现 |
| 3 | "直接调用**现成**的 `p7_attachment_adapter` 把弹仓刚性附着" | DetachableJoint **硬编码** `child_model=target_object`、`child_link=target_link`；任务世界模型叫 `shielding_module`，README 明确要求用 `CS625_P7_ATTACHMENT_ENABLED=false` 关掉它 **[A]** `src/cs625_ap_description/urdf/cs625_active_perception.urdf.xacro:161-170`、`README.md:80-90` | 附着方案列为决策 D1，需 1 处参数化改动后才可用 |
| 4 | "45/45 缺少 goal_time: 5.0，已在 d79713b 修复" | `d79713b` 正是**加入** 5.0 的 commit；9-19 v2 §1.2 明确说这是高置信度**假设**（证据目录无 config 快照）；当前值 **15.0**（`913864c`） **[A]** `src/cs625_bringup/config/sim_controllers.yaml:43-50` | 45/45 保持"遗留链路、不再复跑"；结论只写成工程解释 |
| 5 | Level 3："把 `p7_2_estimate_textured_pose.py` 的配准逻辑封装" | P7.2 估计器是**纹理物体** SIFT+PnP，输入必须 `--model-obj` + `--texture`；新对象 `shielding_module` 是无纹理 STL **[A]** `test/p7_2_estimate_textured_pose.py --help`、`src/cs625_simulation/assets/cs625_task/shielding_module/model.sdf` | 新对象定位必须新建 ICP 管线（W3），不能"节点化旧脚本" |
| 6 | "O2 用 30~50 seeds 正式对比" | 9-19 v2 §9.2 已算出：n=50 只能检出 ≈19–23 pp（配对 McNemar, 80% power） **[B]** | 正式主比较 n=50/条件；30 只够 Pilot |
| 7 | "4 周产出第 2、3、4 章 4 表 8 图" | 手册自身 Level 4 写"第 4–8 周"；点名的产出是 3 表 7 图，且**没有任何 Level 对应第 4 章力觉** **[A]** 手册全文 | 4 周只承诺 G2+首个 episode；第 4 章口径见 D3 |

> 补充：本版发现的两个**此前没人写出**的编排缺陷，见 §4.1 F3/F4——它们会让"跑通插槽 episode"在修好接触 topic 之后**仍然拿不到可解释的判定**。

---

## 1. 事实基线（截至 2026-10-20，仓库 HEAD `18ee409`）

### 1.1 已验证通过的（A 级）

| 能力 | 证据 | 边界 |
|---|---|---|
| URDF / Gazebo / ros2_control / MoveIt 仿真底座 | `README.md:10-18` | 仿真 |
| 任务场景：19.0 kg 屏蔽片模块 + 12.965 kg 楔形插槽架 + 0920 外撑夹爪入库，12 项契约测试 | `cs625_task_scene.yaml`、`test/contract_checks.py`、2026-09-19 工作日志 **[B]** | 几何/质量/惯量；不含抓取可靠性 |
| P7.1–P7.4 `PASS/FROZEN`；P7.5 单场景 `PASS` | `docs/evidence/p7_*`；README | 番茄罐（YCB 纹理物体）；不是新对象、不是正式矩阵 |
| P4 45/45 矩阵 | `docs/evidence/minimum_showcase_matrix_20260906_r1/` | 已废弃的 `sim_view_executor` 遗留链路，仅作工程解释 |
| 插槽任务执行编排与三分判定契约存在 | `test/run_insertion_sequence.sh`、`test/assemble_insertion_episode.py`、`insertion_episode_contract.py` | 编排**从未产出过 `episode.json`** |

### 1.2 当前唯一技术卡点（A 级，按修复顺序）

```text
① 接触 topic 不广播
   两个 SDF 把 <topic> 写在了 <sensor> 层；gz-sim 的 contact 传感器只认 <contact><topic>。
   对照实验（本机 gz-sim 8.11.0）：contact 层 topic 生效，sensor 层 topic 被忽略，
   落到默认 /world/<world>/model/<model>/link/<link>/sensor/<name>/contact。
   [A] shielding_module/model.sdf:76-83、cs625_insertion_scene.sdf:74-78
   证据：7 次尝试的 contacts.log 全部报 "Gazebo contact topics are not advertised"。

② Gazebo 分区没对齐
   sim_base 用 cs625_<时间戳> 私有分区；另开终端的分区为空。
   证据：全部腿回执 runtime.gz_partition = ""，而 gz topic -l 看不到仿真会话。
   [A] sim_base.launch.py:211、CS625_GZ_PARTITION

③ 夹爪两条腿没有回执（本版新发现）
   契约 REQUIRED_LEGS 含 close_arms / open_arms；而 runner 的 command_arms 只发 topic + sleep，
   不写 legs/close_arms.json、legs/open_arms.json。装配器会把它们记为 unrecorded，
   判成 OTHER_FAILURE（UPSTREAM_LEG_FAILED）——即使其余 6 条腿全成功。
   [A] insertion_episode_contract.py:33-41、test/run_insertion_sequence.sh:129-137

④ 接触采集窗口会漏掉下降段（本版新发现）
   runner 在第一条腿之前就启动 90 s 采集，最后又 kill 掉进程：episode 短于 90 s 时
   contacts.json 根本不会写出；长于 90 s 时下降段可能落在窗口之外。
   [A] test/run_insertion_sequence.sh:140-160、p7_capture_gazebo_contacts.py:51
```

7 次真实尝试（`~/cs625_insertion_episodes/20260922_*`）的腿数据（A）：

| 尝试 | 结果 |
|---|---|
| 133952 | pregrasp `EXECUTION_FAILED`（goal_time 5 s）；insert_entry 成功；insert_seated 笛卡尔仅 69.7% |
| 134432 | insert_entry 成功；insert_seated 仅 37.3% |
| 135501 | `MOVEIT_UNAVAILABLE`（仿真已停） |
| 135851 | 多条腿在 MoveIt 层失败，起点构型已被前一次运行带偏 |

> 结论：**在 ①②③④ 修好之前，"插槽卡死"这个结论没有证据可依**；它现在只是一个符合几何事实的**预期**（C→待验证）。

### 1.3 未实现 / 未接线的（A 级）

| 项 | 现状 | 影响 |
|---|---|---|
| 附着关节（module ↔ tool0） | 插件绑 `target_object/target_link`，任务场景默认关闭 | 19 kg 模块只能靠零预紧摩擦，仿真里可能掉件；手册的"附着后抬升"目前不可执行 |
| force adapter | 不存在；只有 Gazebo contact topic 与 P7 contact monitor | 第 4 章"接触状态机"可以先仿真，但 `/sensors/force/wrench` 是未来接口 |
| 新对象位姿估计 | `cs625_target_perception` 只有 `target_pose_relay.py`；P7.2 估计器是纹理物体专用；仓库无 ICP/Open3D 实现 | 第 3 章"固定视角定位"是**新建管线**，不是把旧脚本包一层 |
| 真机 R0–R4 | `docs/real_hardware_readiness.md` 仍写 "Physical robot/operator required" | 真机 F/T 与力控结论暂不可写；第 4 章只能先做方法+仿真/离线 |

### 1.4 近一个月的缺口（B 级）

9/21–10/16 主要在小论文（*Industrial Robot* 初稿、10 图 4 表、7 处矛盾诊断），仿真主线停摆；这是 10-19 焦虑的来源，不是能力缺失。小论文里的公差预算数字（最不利叠加 1.374 mm、3σ=1.105 mm、捕获窗口 1.804 mm）是 **B 级**中间结论，进大论文前必须重新核对口径并标注来源。

---

## 2. 三个需要你拍板的决策（C → 决策后转 A）

### D1 附着方案（决定 W2 数据有没有意义）

| 选项 | 做法 | 代价 | 后果 |
|---|---|---|---|
| D1-A 参数化附着（推荐） | xacro 增加两个 arg（`attachment_child_model/link`），任务场景传 `shielding_module/shielding_module_link`，保留 `detached` 初始状态；抓取到位后发 attach | ~1 小时 + 1 次回归 | 抬升/搬运稳定；论文写"附着辅助的几何抓取"，**不得写"摩擦抓取成功"** |
| D1-B 纯摩擦 | 保持 attachment 关闭 | 0 | 19 kg 零预紧在 Gazebo 可能掉件；只能把"掉件"作为限制如实记录，抓取可靠性不能作为结论 |
| D1-C 只做 MoveIt 附着 | planning scene 里附着，物理不附着 | 0 | 视觉/规划数据可用，物理抬升不可用；第 5 章任务级数据缺一半 |

**推荐 D1-A；决策期限 2026-10-25。** 在决策前，W1 的 episode 只当工程证据，不当论文数据。

### D2 插装口径（决定第 3/4 章边界）

- D2-A（推荐）：近期实验止于**预插入位姿**（insert_entry 成功）；insert_seated 的 0.04 mm 卡死作为**记录在案的真实约束**，写进第 4 章动机与第 5 章失败模式，不进入第 3 章成功率。
- D2-B：把插装纳入第 3 章主实验（需要柔顺/力控，当前不可做）。
- D2-C：给插槽加导向倒角绕过卡死（改变真实工装，不建议）。

**推荐 D2-A；决策期限 2026-10-25。** 仓库现有的 `PRECISION_FIT_JAM` 判定与 D2-A 一致。

### D3 力觉口径（决定第 4 章怎么写）

- 现行工作假设（B→C）：CS625 只有关节电流估计，`/ft_data` 大概率从未跑通（三处 `sensor_name` 不一致）。
- 近期写法：第 4 章写"方法 + 仿真接触证据 + 师兄历史数据离线复算"，**不写"真机力控实验"**。
- 现场判定：按 9-19 v2 §2.4 的 7 步流程（10 分钟）执行一次，结论写入 `docs/real_hardware_readiness.md`；未执行前保持 C。

**决策期限：下次能接触真机时第一件事。**

---

## 3. 总方案（到 2027-06，只保留可验收的门）

### 3.1 论文六章与素材来源

| 章节 | 已有素材 | 缺什么 | 由哪个实验补 |
|---|---|---|---|
| 第 1 章 绪论 | 开题报告、文献库、小论文初稿 | 新对象背景段落 | 无实验，写作 |
| 第 2 章 系统设计与容差 | 模型/惯量/楔形槽几何 **[A]**；夹爪结构与 S_f、ANSYS 计划 **[B/D]**；公差预算 1.374/1.105/1.804 **[B]** | ANSYS 云图、GD&T 图、5 布局 IK/规划基线数据 | W2 表 2-4；周五结构轨道 |
| 第 3 章 任务约束主动重观测 | P7.2/P7.3/P7.4 冻结方法 **[A]** | 新对象定位管线、遮挡场景、S0/S1/S2 数据 | W3–W4 + 12 月 Pilot |
| 第 4 章 接触/力觉 | 卡死约束的几何事实 **[A]**；师兄历史力矩数据 **[B]** | F/T 判定、接触状态机仿真、真题数据 | 2027-01 起；真机条件具备才做 R0–R4 |
| 第 5 章 系统实验 | P7.5 单场景闭环 **[A]**；统计协议 **[B]** | 正式矩阵、可靠性统计 | 2027-02 |
| 第 6 章 总结 | — | — | 写作 |

### 3.2 能力门（取代 9-19 v2 的 Gate 0–M5，改成可验证的证据）

| 门 | 目标 | 验收证据 | 当前状态 |
|---|---|---|---|
| G1 感知输入（任务场景） | `/sensors/camera/*` 四路有效、TF 到 `camera_depth_optical_frame` | `ros2 topic hz` 记录 + 1 帧快照 + tf2_echo | 未在本场景验证（C） |
| G2 已知位姿机械基线 | 预抓取→抓取→抬升→转运→预插入 6 腿全部 `MOTION_SUCCEEDED` | 3 个连续 episode 的 `legs/*.json` | 未通过（A：7 次全失败） |
| G3 固定视角点云定位 | 不用 GT 的位姿估计 + 质量门禁 | 10 组冻结观测上的 e_t/e_R 表 | 未开始（A：无实现） |
| G4 单次主动重观测 | 初视点拒止→选点→移动→新帧→定位→抓取，1 次 | 1 个 episode + 候选 CSV | 未开始 |
| G5 Pilot 批量化 | 10 连续 episode 无人值守 + 自动 CSV | 汇总 CSV + manifest | 未开始 |

### 3.3 月度里程碑（2026-10 → 2027-06）

| 月份 | 唯一硬产出 | 条件/风险 |
|---|---|---|
| 2026-10 下 | G2 首个可解释 episode + 5 布局基线数据（表 2-4 初版） | D1/D2 需先拍板 |
| 2026-11 | G3 离线定位管线 + 遮挡场景可重复摆放 + 质量门禁初标定 | Open3D/ICP 依赖需先验证 |
| 2026-12 | G4 单次重观测闭环 + Pilot 10 集演练（只作工程验收） | 依赖 G3 |
| 2027-01 | Pilot 90 集 + 实验协议冻结 | 依赖 G5 自动化 |
| 2027-02 | O2 正式 150 集 + 消融 100 集；第 3 章初稿 | 依赖 Pilot 阈值 |
| 2027-03 | 第 4 章方法稿 + 接触状态机仿真；真机 R0–R2（若可去现场） | 真机不可控 |
| 2027-04 | 真机 R3–R4 或退化为"仿真+历史数据"口径；第 4/5 章数据冻结 | 同上 |
| 2027-05–06 | 大论文初稿 80%（第 1–5 章） | — |

> 机械结构/ANSYS/GD&T/专利是与上面并行的第二条轨道，各月占用周五上午+下午，不写进本计划的验收门；避免又变成"计划里什么都有、实际一样没落地"。

---

## 4. 当前任务总览（4 周，2026-10-20 → 2026-11-15）

### 4.1 第 1 周（10/20–10/25）：把"能判定的 episode"做出来

> 本周只解决一个问题：**一次完整运行能产出有意义的 `episode.json`**。不追求 PASS/JAM，先追求"判定可信"。

**F1 修接触 topic（2 个文件，4 行）** —— 需你批准后执行
- `shielding_module/model.sdf`：把 `<topic>/cs625_task/contacts/module</topic>` 移入 `<contact>`；
- `cs625_insertion_scene.sdf`：地面 contact 同样处理；
- 验证：起场景后 `gz topic -l | grep cs625_task/contacts` 应看到 module 与 ground 两条；若看到 `/world/.../sensor/.../contact`，说明还没改对。

**F2 钉住 Gazebo 分区（不改代码，改操作纪律）**
- 终端 A：`export CS625_GZ_PARTITION=cs625_ins_$(date +%m%d_%H%M)`，再 `ros2 launch ... sim_task_scene.launch.py`；
- 终端 B：`export GZ_PARTITION=<A 日志里的 ign_partition 值>`、`export IGN_PARTITION=$GZ_PARTITION`；
- 可选加固：runner 里加 4 行 fail-fast（未设 `GZ_PARTITION` 就 exit 2）。

**F3 让夹爪腿留下回执（约 10 行）**
- 把 runner 的 `command_arms` 改成调用 `python3 test/p7_execute_gripper_capture.py --command-id ... --target-position-m ... --output "$episode_dir/legs/<label>.json"`；
- 装配器只读 `receipt.success/code`，schema 不同不影响；
- 回归：`python3 -m pytest test/test_insertion_episode_contract.py -q`。

**F4 接触窗口覆盖下降段（约 5 行）**
- 把接触采集挪到 `insert_seated` 之前启动，`--duration-sec 60`，结束后 `wait`（不要 kill）；
- 验收：`contacts.json` 存在且 `duration_sec` 覆盖下降段。

**F5 首次运行与判读**
```bash
cd ~/projects/cs625-active-perception
source scripts/source_dev_env.sh --verify
python3 test/contract_checks.py
python3 -m pytest test/test_task_insertion_sequence.py test/test_insertion_episode_contract.py test/test_grasp_template.py -q
```
终端 A：
```bash
source scripts/source_dev_env.sh
export CS625_GZ_PARTITION=cs625_ins_$(date +%m%d_%H%M)
CS625_P7_ATTACHMENT_ENABLED=false ros2 launch cs625_bringup sim_task_scene.launch.py
```
终端 B（与 A 同一个分区名）：
```bash
source scripts/source_dev_env.sh
export GZ_PARTITION=<与 A 的 ign_partition 相同>
export IGN_PARTITION=$GZ_PARTITION
gz topic -l | grep cs625_task/contacts      # 期望两行
bash test/run_p7_adapter.sh arm              # 保持前台运行
# 另一个终端：bash test/run_p7_adapter.sh gripper
CS625_P7_SIMULATION_EXECUTION=1 bash test/run_insertion_sequence.sh
```
判读：
```bash
ep=$(ls -dt ~/cs625_insertion_episodes/*/ | head -1)
python3 -c "import json,sys;d=json.load(open(sys.argv[1]));v=d['verdict'];print(v['verdict'],v['failure_codes'],v['first_failed_leg'],v['failed_legs'])" "$ep/episode.json"
```
- `verdict` 三种含义见契约文件；`other_failure` 时看 `failed_legs`；
- 若 `UPSTREAM_LEG_FAILED` 且 `failed_legs=['close_arms','open_arms']` → F3 没生效；
- 若 `CONTACT_EVIDENCE_MISSING` / 没 contacts.json → F1/F2/F4 之一没生效；
- 若 `PRECISION_FIT_JAM` → 预期结果，把 `episode.json` 存进 `docs/evidence/insertion_episode_<date>_r1/`。

**本周产出**：一次完整 episode 目录（legs/contacts/episode.json）+ 一份 1 页失败定位记录。**本周不做**：力控、NBV、批量、改工装。

### 4.2 第 2 周（10/26–11/01）：GT 基线 5 布局 → 表 2-4 原始数据

前置：D1 已决策。若选 D1-A，先做参数化并跑 1 次回归；若选 D1-B，则表 2-4 只填 IK/规划列，抬升列标注"摩擦方案，不作可靠性结论"。

布局（D 级，执行前记录进协议）：

| 布局 | 模块初始位姿 | 目的 |
|---|---|---|
| P0 | 当前记录值 | 基准 |
| P1 | 世界 Y +50 mm | 横向可达性 |
| P2 | 世界 Y −50 mm | 横向可达性 |
| P3 | 绕 Z +8° | 姿态鲁棒性 |
| P4 | 绕 Z −8° | 姿态鲁棒性 |

每条布局的命令顺序（缺一步都会让世界与配置漂移）：
```bash
python3 scripts/sync_task_world.py --print
python3 scripts/sync_task_world.py --set module --position <x> <y> <z> --rpy-rad <r> <p> <yaw>
python3 scripts/sync_task_world.py                 # 把配置写回世界
python3 scripts/sync_task_world.py --check         # 应无漂移
python3 test/contract_checks.py
python3 scripts/insertion_sequence.py --write      # 重算 6 个路点
python3 scripts/insertion_sequence.py --check
```
然后按 4.1 起场景跑 episode（每个布局 3 次，每次重启仿真保证起始构型一致）。

**产出**：`docs/evidence/gt_baseline_2026102x_r1/` 下 5 布局 × 3 次 × `legs/*.json`，加一个聚合脚本 `scripts/summarize_insertion_legs.py`（新写，约 80 行）输出 `baseline.csv` 与表 2-4：
- 每腿：`code`、`planning_time_sec`、`execution_time_sec`、`fk_position_error_m`、`fk_orientation_error_rad`、`cartesian_path_fraction`、`start_state_joint_positions_rad`；
- 由关节角与限位算**最小关节裕度**（角限位来自 `p7_safe_initial_positions.yaml` 同源配置）；
- 环境最小间隙：**暂不写**（需要新测距脚本，列 C，11 月补）。

**验收**：表 2-4 的每一格都有 CSV 行可追溯；无一行来自"估计"。

### 4.3 第 3 周（11/02–11/08）：新对象点云定位最小闭环（离线优先）

为什么不能复用 P7.2：它是纹理物体 SIFT+PnP，输入要 OBJ+纹理；模块是 STL、无纹理。[A]

1. 冻结观测集：写 `scripts/capture_sensor_snapshot.py`（订阅 `/sensors/camera/color/image`、`depth/image`、`depth/camera_info`、`points`，落盘一帧 + 时间戳 + TF 快照），在不同视角/布局下采 10 组，存 `docs/evidence/localization_views_2026110x_r1/`。
2. 依赖决策：`python3 -c "import numpy, scipy"`；Open3D 未安装（A）——先验证 `pip install open3d` 是否可用（需你批准）；不可用则用 numpy/scipy 写最小 ICP。
3. 写 `scripts/module_pose_icp.py`（新，约 150 行）：
   - ROI 裁剪（只在扶手/外轮廓附近的点）→ 体素降采样 → 由 GT 位姿 seed 的 point-to-point ICP → 输出 `T_est`、`fitness`、`e_t`、`e_R`、`visible_ratio_grasp_roi`；
   - **GT 只用于评价，不进入抓取位姿**（这条写进脚本头注释）。
4. 先在 10 组冻结观测上跑通，得到 e_t/e_R 分布；再决定是否加 FPFH/多 seed 全局初始化。

**验收**：10 组观测的定位误差表 + 一版可复现的脚本；**不验收成功率**。

### 4.4 第 4 周（11/09–11/15）：质量门禁标定 + G3 判定 + 协议冻结

1. 用第 3 周数据标定门禁阈值：`fitness` 与 `visible_ratio_grasp_roi` 各画分布，取使"误接收/误拒止"都可接受的阈值；阈值写进 `config/common` 或 `config/sim`，不硬编码。
2. 用 2 个未参与标定的布局做留出验证；G3 判定：通过/不通过要写清依据。
3. 把 §5 的实验设计冻结成 `docs/experiment_protocol_v1.md`（预注册）：阈值、种子、指标、失败码、统计方法、停止规则。
4. 复盘：如果 G3 未通过，不进入 G4；把阻塞写清楚，而不是"再跑跑看"。

**四周总产出**：可判定的 episode 1 份 + 表 2-4 数据 + 定位误差表 + 冻结的协议草稿。**不承诺**论文级正式矩阵。

---

## 5. 实验方案设计 v1.0（预注册草稿）

### 5.1 研究问题

- **RQ1**：重遮挡 O2 下，任务约束主动重观测 S2 相对 S0（固定视角）、S1（预设第二视点）是否提高最终任务成功率？
- **RQ2**：S2 的收益来自哪一项——可见性增益、运动代价、抓取衔接代价？（消融）
- **RQ3**：0.04 mm 精密配合在位置控制下是否必然卡死；卡死是否总伴随接触？（第 4 章动机）

### 5.2 对象、场景与遮挡

- 对象：19.0 kg 屏蔽片模块；工装：12.62°/5.004° 楔形插槽架；夹爪：0920 外撑式（零预紧仿真口径）。
- 任务定义（D2-A）：**取件 → 抬升 → 转运 → 预插入** 为第 3 章任务；`insert_seated` 只作第 4 章约束证据，不计入第 3 章成功率。
- 遮挡物：需新增 1 个静态圆柱/方柱模型（当前任务世界没有遮挡物 [A]），位置由参数给出（3 档），随 `cs625_task_scene.yaml` 派生；**遮挡等级不用"感觉"定义**，而用初始视角下实测的 `visible_ratio_grasp_roi` 分档：O0 >0.65、O1 0.35–0.65、O2 <0.35（阈值来自 9-19 v2 的提案 **[B]**，由 Pilot 标定后冻结 **[C]**）。

### 5.3 因子、策略与配对

| 因子 | 水平 |
|---|---|
| 遮挡 | O0 / O1 / O2 |
| 布局 | P0 / P1 / P2（P3/P4 只做鲁棒性补充） |
| 策略 | S0 固定视角 / S1 预设第二视点 / S2 任务约束主动重观测 |

- 三策略共用同一候选生成器、同一硬约束过滤（FOV→工作空间→IK→关节限位→自碰撞/环境碰撞→MoveIt 可达）与同一质量门禁；
- 每个 scene seed 预生成一次，三策略配对使用；失败结果不删除；
- 初视点判据用**冻结后的**质量门禁，不用真值。

### 5.4 指标（全部可从 episode 记录导出）

| 指标 | 定义 | 数据来源 |
|---|---|---|
| SR_task | 6 条位姿腿 + 2 条夹爪腿全部 `MOTION_SUCCEEDED`，模块到预插入位 | `episode.json` |
| SR_local | 定位质量门禁通过（无重观测/重观测后） | `quality` 记录 |
| RR | 初视点拒止的 episode 中，重观测后通过门禁的比例 | episode 汇总 |
| e_t / e_R | 估计位姿相对 GT 的平移/旋转误差（GT 只评价） | `estimate.json` |
| C_recover | 重观测的额外路径长度与时间 | 轨迹/回执 |
| 接触 | 下降段 contact 记录（有/无、次数、首触时间） | `contacts.json` |
| 失败码 | 现有契约码 + 新增 `LOCALIZATION_REJECTED` 等 | `failure_codes.csv` |

### 5.5 数据契约（先定，再跑）

```text
docs/evidence/<topic>_<yyyymmdd>_r<N>/
  manifest.json        git_head、config_sha256{文件:hash}、ros_distro、gz_partition、host、wall_time_sec、seeds_sha256
  episodes.csv         每个 episode 一行：seed、occlusion、layout、strategy、verdict、failure_codes、views、e_t、e_R、RR
  episodes/<id>/       与运行目录同构（legs/、contacts.json、episode.json）
  candidates.csv       S2 的候选与评分（用于图 3-5/3-6）
```
- 没有 `manifest.json` 的证据不进论文；
- 大体积原始点云留在本地归档，仓库只放汇总与小 JSON。

### 5.6 样本量与统计（来自 9-19 v2 §9.2，B 级）

| 阶段 | 规模 | 用途 |
|---|---|---|
| Pilot | 3 遮挡 × 3 策略 × 10 seeds = 90 | 定阈值、方差、失败分布；**不做确证性结论** |
| 正式主比较 | O2 × 3 策略 × 50 配对 seeds = 150 | 确证性比较（主终点） |
| 消融 | 2 个变体 × 50 seeds = 100 | A1 去掉抓取衔接代价、A2 去掉运动代价 |

- 主终点：O2 的 SR_task 配对差异；Wilson 95% CI 描述；配对 McNemar 检验；≤3 个确证性检验，Holm 校正；
- 最小可检出差异：n=50 时约 19–23 pp（80% power, α=0.05 双侧）；**因此不允许用 30 seeds 下"显著提升"的结论**；
- 停止规则：Pilot 后若 O2 下 S0 与 S2 差异 <10 pp，先查门禁与场景，再决定是否扩大样本（不做事后挑选）。

### 5.7 实验门（与 §3.2 对应）

| 门 | 通过标准 |
|---|---|
| G2 | 3 个连续 episode 的 6 条位姿腿全部成功；模块到预插入位；无非预期碰撞 |
| G3 | 10 组冻结观测中 ≥8 组 e_t ≤ 5 mm 且 e_R ≤ 2°（阈值先按此，Pilot 前可调，调整必须留记录） |
| G4 | 1 次完整"拒止→选点→移动→新帧→定位→抓取"闭环，所有失败有 failure code |
| G5 | 10 个连续 episode 无人值守，自动汇总 CSV |

### 5.8 论文图表映射

| 产出 | 位置 |
|---|---|
| 表 2-4 五布局 IK/规划/误差 | 第 2 章 |
| 蒙特卡洛公差图（1.374/1.105 vs 1.804） | 第 2 章（数值来源需复核，B） |
| 表 3-1 S0/S1/S2 × O0/O1/O2、95% CI + McNemar | 第 3 章 |
| 表 3-2 消融 | 第 3 章 |
| 图 3-5~3-8 候选分布/过滤漏斗/误差箱线/代价柱状 | 第 3 章 |
| 卡死接触时间线 + 失败模式表 | 第 4/5 章 |

---

## 6. 风险与备选

| 风险 | 触发信号 | 备选 |
|---|---|---|
| 19 kg 掉件 | 抬升后模块位置突变、contact 无记录 | D1-A 附着；或论文只写"几何抓取"，不写"抓取可靠性" |
| 定位精度不足 | e_t > 10 mm | 缩小 ROI、加多 seed、先做"已知初始位姿的局部 ICP"，把全局初始化放到 11 月下旬 |
| 遮挡阈值不稳定 | O1/O2 分档重叠 | 改为按遮挡物几何位置定义，`visible_ratio` 只作描述 |
| 时间不够 | G2 两周内不通过 | 正式矩阵缩为 O2-only 50×3；Pilot 减半；**不改统计方法** |
| 依赖装不上 | Open3D pip 失败 | numpy/scipy 最小 ICP；仍不行则第 3 章定位部分退为"离线数据集验证"，且如实写明 |
| 金库 9p 掉挂载 | `No such device` | `wsl.exe -u root -e bash -c "umount -l /mnt/g; mount -t drvfs G: /mnt/g"`（2026-10-20 实测可用） |

---

## 7. 一页命令卡（每天开机只跑这 5 条）

```bash
cd ~/projects/cs625-active-perception
source scripts/source_dev_env.sh --verify
python3 test/contract_checks.py
python3 -m pytest test/test_task_insertion_sequence.py test/test_insertion_episode_contract.py test/test_grasp_template.py -q
git log --oneline -1
```
> 全绿才动手改代码。任何一条红，当天就修它，不进入下一步。

---

## 8. 与旧文档的关系

| 文档 | 本版如何使用 |
|---|---|
| 《2026-09-18 开题答辩后研究路线调整_最新正式版》 | 论文骨架与六章结构以它为准；本版只补"已证实/待验证"标记 |
| 《2026-09-19 任务总览与仿真实验设计 v2》 | 统计与证据纪律以它为准；日期/10 月中旬目标作废，§5–§9 抽为协议 |
| 《2026-10-19 破局实操手册》 | 四级闯关的"执行节奏"可用；4 处硬错误按 §0.2 修正后使用 |
| 《2026-10-19 破局总方案 v4.0》 | 已从归档还原，仅作历史参考；月度/求职轨道不由本版替代 |
| 《2026-09-18 原版归档》 | 已从归档还原，写"研究内容"细节时参考 |

---

## 9. 变更记录

| 版本 | 日期 | 变更 |
|---|---|---|
| v1.0 | 2026-10-20 | 首版：事实基线 + 2 个新发现的编排缺陷 + 3 个决策 + 4 周任务 + 实验协议草稿；不归档任何旧版 |
