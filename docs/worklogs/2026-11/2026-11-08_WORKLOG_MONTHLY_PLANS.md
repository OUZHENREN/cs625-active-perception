# 2026-11-08 WORKLOG 计划改为按月 + 十一月工作计划落地

> 相关：`docs/plans/2026-11.md`（本月计划）、`docs/plans/2026-10.md`（上月主计划，原
> `2026-11-03_PLAN_CS625_FOCUS_AND_ROADMAP.md` 改名而来）
> 真机使用：无。本轮只改文档与一个镜像脚本的判断条件，未启动 ROS、未移动机械臂、未占用相机。
> 仿真使用：无。

## 1. 本轮要解决的问题

所有者提出两件事：
1. 外部评审（对《2026-10-30 课题总体规划与实施方案_完整版》）的意见是否成立、计划要不要改；
2. 计划档案改成**逐月一份**：旧的「⭐PLAN CS625 课题聚焦与实施路线（活文档）」按月份改名，
   另外单独写一份十一月工作计划。

## 2. 外部评审意见的核对结果（对着仓库证据，不采信记忆）

| 评审意见 | 结论 | 依据 |
|---|---|---|
| 交叉偏振消高光物理正确 | ✅ 成立 | 光学事实，无需争论 |
| 点云密度 83→412 点/cm² "实测过就可信" | ⚠️ **不能采信** | 全仓库检索 `偏振` 只命中计划文字，**没有任何安装/实测记录**；该数字来源未标注，不能当本工位实测写进论文 |
| 粗-精分层合理 | ✅ 成立 | 现有 `scripts/icp_pose_once.py` 已证明精配准可跑通（numpy + kd-tree，非 PCL） |
| 180° 翻转消歧命中痛点 | ✅ 成立 | 仓库记录"扶手面朝向机械臂才稳（±45° 窗口）"，翻转解在黑盒里无法被发现 |
| NBV 只有标题、没有实现 | ✅ 成立（但结论要改） | 总规划 §3.6 代码确实无 NBV；**但仓库已有骨架** `src/cs625_view_generation` + `src/cs625_view_evaluation/.../policy.py` 的 `coverage_nbv`，只是评分为 `sin(俯仰角)` 代理量 → 本月做"评分接口"，不做完整 NBV |
| C++ 工程与现有 Python 栈冲突 | ✅ 成立 | 整条已验证示教链是 66 个 Python 脚本 + 12 个薄包；重写等于把已验证的东西清零 |
| 深度帧率不支持实时视觉伺服 | ✅ 成立 | 实测深度 0.79 fps（1280×960 有效点≈0，实际解算用 640×480）；彩色 23.1–23.5 fps |
| 工件可能不是 7.0 kg 光学镜 | ❌ **不成立** | `docs/manuals/2026-11-03_MANUAL_REAL_MACHINE_EXPERIENCE.md` 第 6 条记"承载 6.8 kg 工件"，与 7.0 kg 差 3%。真正缺的是**含扶手的 CAD 模型**（仓库只有简化 `box_collision.STL`） |
| 视觉与力控两条线不要混 | ⚠️ 部分成立 | 总规划 §2.2.2 已写"视触时序解耦"，属读得快；但叙述确实容易混淆，计划里分开排期 |

评审**漏掉**的三条（决定本月能不能开工）：
1. 手眼链两套口径未验明（`T_tcp_cam` vs `T_flange_cam` 到槽心差 0.16 m，
   `config/handeye/tcp_to_camera.json` 与 URDF 不一致）；
2. 偏振片是否真的装在相机上，仓库无记录；
3. 插槽权威值时效（`runtime/slot_reference_verified.json` 为 11-04，当天 R1 差 33 mm）。

结论：**不改方向，改顺序**——本月先做"数据入口 + 手眼验明 + 粗测量"，NBV 的完整实现推到
十二月（与仓库"关键链未验收不推进 NBV"的规则一致）。

## 3. 本轮改动的文件

```text
R  docs/plans/2026-11-03_PLAN_CS625_FOCUS_AND_ROADMAP.md -> docs/plans/2026-10.md
   （git mv；正文未改，只在头部加"十月主计划 + 月度命名规则"说明）
A  docs/plans/2026-11.md                          新增：十一月工作计划（三件交付 + 条件性插入收尾）
M  docs/README.md                                 索引加"本月做什么"一行；目录树与文档规则加月度命名
M  docs/worklogs/2026-11/2026-11-03_WORKLOG_OBSERVATION_BASELINES_AND_ACCURACY.md
                                                  旧计划链接改指 2026-10.md，并注明改名事实
M  scripts/sync_worklog.sh                        文件名断言放行 YYYY-MM.md（月度计划用）
A  docs/worklogs/2026-11/2026-11-08_WORKLOG_MONTHLY_PLANS.md   本文件
```

工作日志快照 `docs/worklogs/2026-11/2026-11-03_PLAN_CS625_FOCUS_AND_ROADMAP.md` **保持原样**
（它是 11-03 的历史记录，按当时状态保留，不改名、不删除）。

## 4. 执行的命令与结果

```bash
git mv docs/plans/2026-11-03_PLAN_CS625_FOCUS_AND_ROADMAP.md docs/plans/2026-10.md   # PASS
python3 scripts/check_docs.py                                                        # PASS（见 §5）
scripts/sync_worklog.sh docs/plans/2026-10.md --to 01_研究规划                        # 见 §6
scripts/sync_worklog.sh docs/plans/2026-11.md --to 01_研究规划                        # 见 §6
scripts/sync_worklog.sh docs/worklogs/2026-11/2026-11-08_WORKLOG_MONTHLY_PLANS.md     # 见 §6
```

## 5. 验证结果

```text
python3 scripts/check_docs.py → ALL DOC CHECKS PASSED
  · manual 29 节（章节号完整、索引一致）
  · 引用路径 49 条全部存在（首次跑出 1 条 FAIL：`scripts/vision_capture.py` 尚未创建；
    改成"scripts/ 下待建的 vision_capture.py"后通过——文档不得引用尚不存在的文件）
  · agent_memory 13 条规则无重复

source scripts/source_dev_env.sh --full && python3 -m pytest test/ -q
  → 293 passed in 10.57s
  （不 source 环境直接跑会 3 个用例 collection error：No module named 'rclpy'，
    这是环境问题，不是代码问题）
```

## 6. Obsidian 镜像

```text
docs/plans/2026-10.md                                        → 01_研究规划/2026-10.md            ✅
docs/plans/2026-11.md                                        → 01_研究规划/2026-11.md            ✅
docs/worklogs/2026-11/2026-11-08_WORKLOG_MONTHLY_PLANS.md    → 工作日志/2026-11-08_...md         ✅（本文件）
```

`scripts/sync_worklog.sh` 原来只接受 `YYYY-MM-DD_` 开头的文件名，月度计划 `2026-11.md`
会被它直接拒绝；本轮把断言放宽到 `YYYY-MM-DD_` 或 `YYYY-MM.` 两种形状（两行改动 + 注释），
镜像才能走原来的校验链路（复制 + SHA256 比对），而不是手工 `cp`。

## 7. 剩余限制与下一步

```text
· 十一月计划里的 scripts/vision_capture.py 尚不存在，是本月第一个要新建的文件；
  在那之前先用 scripts/capture_dataset.py 观察现有落盘逻辑，避免出现第二套格式。
· 手眼链验明、偏振片确认都还没做——它们是 W1 的任务，不是已完成事项。
· 插入段收尾是【条件性】交付，D1–D3 在 11-22 前达标才做，否则顺延十二月与力控一起。
· Obsidian 的 01_研究规划 里仍留有旧名 2026-11-03_PLAN_CS625_FOCUS_AND_ROADMAP.md，
  删除属于所有者决策，未擅自删除。
```

## 8. 能力层与关键链（本轮无变化，如实记录）

```text
Capability layer：environment / robot model / kinematics / vision / active perception /
                  real hardware —— 本轮均未推进，只做计划文档。
Critical-chain status（沿用 2026-11-07 判定，本轮未改）：
  URDF -> Gazebo entity            PASS（仿真）
  ros2_control -> joint_states     PASS（真机）
  RGB-D -> normalized topics       PARTIAL（彩色可用，深度 0.79 fps）
  base_link -> camera optical TF   NOT ACCEPTED（手眼两套口径差 0.16 m）
  point cloud -> MoveIt scene      NOT ACCEPTED
  NBV decision -> robot execution  NOT STARTED
Sim/real alignment：共用 common core；本轮未触碰，real execution 未发生。
Intentionally not done：NBV 实现、偏振效果结论、C++ 重写、力控/RL/ANSYS；未修改任何 vendor 代码。
```
