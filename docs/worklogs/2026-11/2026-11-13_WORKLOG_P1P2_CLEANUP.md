# 2026-11-13 工作日志：P1/P2 清理批次（NBV 资产、AGENTS 主线、memory 拆分、提交前检查、缺口 fallback）

> 起因：外部审计文件 `workspace_assessment_20261114.md` 提出 6 类问题。我逐条核实后
> 采纳有效项（P1 三项 + P2 三项），分派 4 个子智能体并行执行、写作用域互不重叠，
> 我负责缺口 fallback、交叉修正、验证与提交。
> 所有者决定：**本轮及前几轮跨午夜的工作都计入 2026-11-13**。
> 本轮**没有真机运动**，没有启动 Gazebo/相机。

## 1. P1 —— 低风险、直接改善真机安全与论文可复现性

### 1.1 归档旧侧移 NBV 策略
`scripts/nbv_lateral.py` → `docs/archive/2026-11-13-scripts-legacy/six-mode/nbv_lateral.py`
（`git mv`，内容零改动；方案 A 落地后无调用者）。归档 README 的 six-mode 表补一行，
写明原路径、原因、现行替代 `nbv_from_reference.py --auto-station`。

### 1.2 `nbv_score.py` 标注"仅作库"
只改文件头：说明它现在只被 `nbv_from_reference.py` 当库用（`read_tcp()` 等），
命令行评分入口 2026-11-13 退役、现行改看 `nbv_views.py`。正文未删。

### 1.3 每个已验证站位加 `validated_command`
`config/tasks/common.yaml` 的 7 个站位（0/+11/+11_left8/−16/−23/−30/−45）各加一条
**可复现该站位精度的完整命令**，取自现行 `config/tasks/load_efficient.yaml` 的
`workpiece_fine`：`measure_object.py --preset workpiece --stage fine --groups N --prior ...`。
`-45` 用 `--groups 4`（其 1.3 mm 备注来自 4 组那批），且该站位 `validated: false`。
→ 论文里报"0.14–0.5 mm"时，读者能照着同一条命令复现。

### 1.4 AGENTS.md 订正（真机主线 + 安全规则）
- `View-planning order` 拆成两节：**真机主线 = 方案 A**（粗观测偏转角 → 查已验证站位表
  容差 12° → 与 0° 相对视角推导交叉核对 >40 mm 拒绝 → `plan_pose_ik` 安全闸 → 换位 →
  带先验精测；并写明 `--auto-station` 只打印、`--execute` 才动、taught 族免核对、
  三种失败一律不自动换位）；**搜索式流程 = 仿真/研究专用**，并写明真机不用的物理原因
  （相机在 TCP 后方约 0.3 m、腕部 ±45° 电缆、桌面/夹具干涉）。
- `Prohibited patterns` 新增三条硬安全规则（原只在 agent_memory）：不忽略接触对/不屏蔽
  碰撞报警；当前状态已接触禁止移动；禁止翻转构型且大幅改构型前打印 6 关节角并经所有者
  同意。`|wrist_3|` 与 J6 是同一关节，已统一为单一阈值 170°。

## 2. P2 —— 结构性整理

### 2.1 `agent_memory.md` 拆两层（所有者已批准）
| 文件 | 行数 | 内容 |
|---|---|---|
| `docs/agent_memory.md` | **183**（原 2570） | 铁律/工作方式、安全红线、当前权威值、相机与视角约束、口径更正索引 |
| `docs/agent_memory_history.md` | 2587 | 原全文逐字存档（138 个 `##` 标题一一对应，可回溯事故经过） |

拆分后每轮开工只需读 183 行，权威值不再被埋在第 2540 行。history 与原文件逐字一致
（已用 `git show HEAD:` 比对）。新增条目仍写 `agent_memory.md`，history 只读。

### 2.2 提交前检查（新增，所有者已批准）
`scripts/pre_commit_check.sh`：自己 source 环境 → pytest → check_docs → contract_checks，
fail-fast，任一步失败即报"失败：步骤 i/N"；`--fast` 跳过 pytest（只改文档时用）。
`.vscode/tasks.json` 增加第 9 个任务「⑨ 提交前检查 · 三连」。
本轮提交前已用它跑完整三连：**283 passed → 文档一致 → 治理合同 PASS**。

### 2.3 覆盖率缺口 fallback（原先只有"拒绝"没有"然后"）
`nbv_from_reference.py --auto-station` 选不到可用站位时，现在会：
1. 说清原因：最近站位是哪个、差多少度、**是否有站位因未过精度门被跳过**；
2. 打印三步处置：原地精测（默认）→ 测不稳则人工摆位 → 新角度实测通过后并入
   `workpiece_views`；
3. 把缺口**落盘** `runtime/measurements/station_gap_<时间戳_微秒>.json`，含推导出的
   目标 TCP/四元数、最近站位、被跳过的站位、当前测量文件——下次人工摆到该角度时可
   直接实测并补进表。
`nbv_views.station_plan()` 相应新增 `nearest_key / nearest_error_deg /
skipped_unvalidated` 三个诊断字段。

## 3. 交叉修正（子智能体留下、由我处理）

```text
① config/tasks/common.yaml：-45 的 validated_command 值里被追加了中文注解
   （导致命令不能直接复制执行）→ 去掉注解（说明本来就在 note 里）。
② scripts/nbv_views.py 的旧 docstring 仍写"--execute 交给 nbv_lateral 执行"
   → 改为指向 nbv_from_reference.py --auto-station，并注明 nbv_lateral 已归档。
③ agent_memory 拆分后，AGENTS.md 与真机手册里"见 agent_memory §133–135 / §138/§139"
   指向的编号已移到 history → 改为"见 agent_memory.md 安全红线/当前权威值，
   原始条目在 agent_memory_history.md §xxx"。
④ docs/README.md：索引与目录树补 agent_memory_history.md，并说明"历史条目不必每轮读"。
```

## 4. 验证

```bash
source scripts/source_dev_env.sh --full
bash scripts/pre_commit_check.sh        # 283 passed → ALL DOC CHECKS PASSED → PASS（exit 0）
python3 -m pytest test/ -q              # 283 passed
# 缺口 fallback 两条路径（纯数学，不动臂）
python3 scripts/nbv_from_reference.py --auto-station \
  --reference-measurement runtime/measurements/workpiece_zero.json \
  --current-measurement runtime/measurements/workpiece_minus45.json   # 拒绝 + 落盘缺口
```

- `-42.4°` 现在输出：最近站位 −45°（在容差内但未通过精度门）→ 拒绝 + 记录缺口文件；
- `-30°` 仍正常选站（回归未受影响）；
- `validated_command` 的 7 条命令参数（`--preset/--stage/--groups/--prior`）已核对
  `measure_object.py --help` 真实存在。

## 5. 能力层与状态

```text
Capability layer: 无能力变化（整理 + 安全闸收紧）
  环境与依赖      PASS（283 测试 + 文档 + 合同，一条命令可复现）
  NBV 选站        PASS（新增"未验证站位不得选中"与缺口落盘；真机未跑）
  真机            NOT RUN（本轮无真机运动）
Intentionally not done:
  · E1/E2/E3 论文实验采集脚本（需所有者确认新增脚本）；
  · 仿真世界/配置重生成（等六维力+法兰的新 URDF 到位后统一做）；
  · agent_memory_history.md 未再压缩（保留全文以便回溯事故经过）。
```

## 6. 复现命令

```bash
source scripts/source_dev_env.sh --full
bash scripts/pre_commit_check.sh
git log --oneline -1 --stat
```

## 7. 收尾（2026-11-14 01:4x）：覆盖范围扩大到 −45..+25，移动门 40 → 200 mm

所有者当天最后两个决定：

```text
① 覆盖范围从 −42..+23 扩到 −45..+25：
   config/tasks/common.yaml  tolerance_deg 12 → 15（覆盖 [−45, +26]），
   并新增 near_tolerance_deg: 12 —— 12° 以内仍做严格核对，12..15° 判 station_far
   （可用 + 告警，最终由精观测质量门兜底）。
② "工件被移动"的判定门从 40 mm 放宽到 200 mm：
   相机绕工件转 12–15° 本身就有 90–120 mm 的几何位移，40 mm 会误判远站位；
   --family-gate-mm 默认 40 → 200，nbv_views.station_plan 默认同步。
   工件真被搬走 200 mm 以上仍判 station_moved 并拒绝自动换位。
```

实现细节（`scripts/nbv_views.py`）：
- 门对**所有** relative_zero 站位统一适用（远站位不再跳过），`station_far` 只作告警标签；
- 容差比较加 `1e-6` 余量（修 −45.0° 因浮点被判越界）；
- 返回值新增 `near_tolerance_deg / far_by_deg`，`station_moved` 仍优先于 `station_far`。

测试（`test/test_nbv_station_selection.py`，11 passed）：
- 工件搬走 300 mm → 仍判 station_moved；
- 工件移动 60 mm → 放行（对应所有者新门）；
- 远站位（偏差 14°，介于 near 12 与 tol 15）→ station_far 且选中该站位。

文档同步：AGENTS.md（tolerance 15 / 严格核对 12 / 门 200 mm）、论文计划（2.4 节口径）。

**所有者明天（11-14 白天）计划**：上真机逐项测"工件边界 + 粗观测/精观测在该边界是否稳定"，
据此再定 tolerance / 门的最终值。本轮没有真机运动。

