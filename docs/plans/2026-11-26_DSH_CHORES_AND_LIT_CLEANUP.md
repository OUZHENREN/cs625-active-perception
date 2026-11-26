# 文献收尾与日常工程杂活清单（2026-11-26，供后续执行）

> [!NOTE] 接管与执行进度（2026-11-26）
> 已直接接管并完成以下全部离线任务（第 1、3、4、5、6 项）：
> - 第 1 项已完成：`N7KRDULX`（24 条批注）与 `JKP9YS3R`（5 条批注）、`SKHVI4C5`、`SP4IE6N4`、`CPQNLDLY` 均已完成 T1 精读，`92KPMV94` 已补全印刷页码（`p.171–180`）与章节，并修正 `RESEARCH_GAPS.md` 空白一与 `citation_matrix.py`。
> - 第 2 项已确认库内现状：Zotero 全库 `3,411` 条中 Gage R&R / MSA PDF 为 `0` 篇（仅存无附件标准 `ZIL5I4QK`），待外部导入 AIAG MSA 第 4 版与 ISO 9283:1998 PDF 后即可写入 T1 笔记。
> - 第 3–4 项已完成：`literature_index.py` / `citation_matrix.py` 年份与章节正则修复，`NOTES_INDEX.md`（93 篇）、`CITATION_MATRIX.md`（19 节，仅 `3.6` 诚实缺口）、`lit_triage`（782 篇已全量排除 93 篇 T1）与 `THESIS_OUTLINE_V2.md`（含 `EXPERIMENT_DESIGN_FROM_LITERATURE.md` 与 `force_control_interface.md` 口径对齐）均已更新。
> - 第 5–6 项离线部分已完成：`config/tasks/full_efficient.yaml`、`run_pick_and_place.py`、`experiment_recorder.py`、`archive_experiment.py`、`ft_sensor_record.py` 与对应单测已落地；对 `measure_20261125_191925` 离线核查证实插槽应保留 `prior_register`（最大中心散布 0.365 mm），不宜强套 `anchor_on_corner`（跳变 ~795 mm、覆盖率跌至 0）。
> - 仅剩需宿主写权限 `/mnt/g`、完整 `/opt/ros/jazzy` 或真实硬件的条件项：第 7 项仿真搭建、第 8 项 Obsidian/Zotero 回写与 ROS 全量验证、第 9 项真机窗口检查及相机面板重启出图复核。

> 目标：在不动主线框架的前提下，集中清理复核后遗留的文献收口、元数据修复、离线仿真验证与真机前置准备项。
> 适用范围：与 [docs/TODO.md](../TODO.md) 并行对照执行，每完成一项回 TODO 勾选并附命令实测结果。

## 一、文献收口与 Zotero 回写（可离线立刻处理）

### 1. 补读手眼标定高批注文献 `N7KRDULX` 并复核 3.1 节
- **现状问题**：`runtime/lit_triage/all.json` 中已确认存在 `[2022] An overview of hand-eye calibration`，对应 `itemKey` 为 `N7KRDULX`（`line: B_视觉测量与标定`，`hits=13`），且为所有者高频批注文献；此前排查时写入的 `AUJMWY5D` 在 `all.json` 中不存在，导致该篇至今未出现在 [docs/literature/notes/](../literature/notes/) 中。
- **执行动作**：
  1. 在 Zotero 库确认 `N7KRDULX` 的 PDF 附件路径及所有者批注内容，按 `docs/literature/templates/PAPER_NOTE_TEMPLATE.md` 完成 T1 精读笔记并写入 `docs/literature/notes/N7KRDULX_<短标题>.md`。
  2. 重点核实该综述对手眼标定模型求解（如 Tsai-Lenz、对偶四元数、全局最优解）与残差评价指标（方程残差 vs 重投影误差）的表述，补充或更新 [docs/literature/RESEARCH_GAPS.md](../literature/RESEARCH_GAPS.md) 空白一。
  3. 检查是否有必要更新 `scripts/citation_matrix.py` 的 `3.1 手眼标定与残差评价` `MANUAL_OVERRIDE`（或保留 `RGBUCYKU` / `8G2H64GR` / `T8IJCCH3` 三篇原始求解文献、将 `N7KRDULX` 作为综述方法依据列入），随后重跑 `citation_matrix.py` 与 `outline_inject_refs.py`。
- **完成标志**：`docs/literature/notes/N7KRDULX_*.md` 落地且通过 `python3 scripts/check_docs.py`。

### 2. 补充第 3.6 节测量系统能力评价（R&R / GRR）文献
- **现状问题**：在 `runtime/lit_triage/all.json` 全部 797 条记录中检索 `R&R` / `GRR` / `Gage` / `测量系统分析` / `再现性`，命中数为 0；目前 [docs/literature/CITATION_MATRIX.md](../literature/CITATION_MATRIX.md) 第 3.6 节仍为 0 篇候选。
- **执行动作**：
  1. 向 Zotero 库补充检索并入库至少 3 篇可直接引用的标准或论文（建议组合：AIAG MSA 第 4 版测量系统分析参考手册、`Gage R&R` 在机械臂三维视觉测量或在线检测中的工程应用论文 2 篇）。
  2. 完成 3 篇 T1 精读笔记写入 `docs/literature/notes/`，重点记录 `tolerance = 2 mm` 口径下的 `%GRR` 判定阈值（如 10% / 30%）与 `NDC`（Distinct Categories）最小要求、重复测量组数与操作/姿态因子划分。
  3. 重跑 `python3 scripts/citation_matrix.py` 与 `python3 scripts/outline_inject_refs.py`，确认 3.6 节候选数达到 3 篇。
- **完成标志**：`CITATION_MATRIX.md` 全 19 节候选不足数降至 0，并同步更新纲要附录 A。

### 3. 重跑分诊脚本并统一 Zotero 子笔记清单回写
- **现状问题**：当前 [runtime/lit_triage/summary.md](../../runtime/lit_triage/summary.md) 仍记录排除 78 篇 T1 文献，而 `docs/literature/notes/` 已增至 88 篇（新增 `5A9MHS5X`、`94THW73P`、`GEKIMA8P`、`CW6SUARC` 等 10 篇）；同时 `zotero_notes_manifest.json` 需包含新增笔记及待补的 `N7KRDULX`。
- **执行动作**：
  1. 完成第 1–2 项新笔记后，重跑分诊统计：`python3 scripts/lit_triage.py --db "/mnt/g/我的云端硬盘/Zotero/zotero.sqlite"`，核对 `runtime/lit_triage/summary.md` 已排除 T1 数量与 `notes/` 文件数一致。
  2. 重新生成并导出 Zotero 回写清单：`python3 scripts/zotero_import_notes.py --db "/mnt/g/我的云端硬盘/Zotero/zotero.sqlite" --emit-manifest`。
  3. 提醒所有者在 Zotero 开发者菜单 Run JavaScript 粘贴运行 `scripts/zotero_import.js`，一次性回写新增精读子笔记。
- **完成标志**：`summary.md` 排除数与 `notes/*.md` 数量一致，manifest 导出成功且 Zotero 完成导入无报错。

### 4. 核对单篇元数据与自动排序小节合理性
- **现状问题**：
  - `BJPEZ2IP_基于视觉目标位姿估计综述.md` 元数据注明 PDF 为无年份双栏手稿（仅标注作者生年 1995），需回 Zotero 核实真实发表/入库年份并补写进第 1 节元数据行。
  - `3.3 可达性判定`、`4.1 误差源辨识`、`5.1 KWR75 通信与驱动`、`5.2 重力补偿与零漂标定`、`3.2 粗-精分层位姿解算` 目前走关键词自动排序；部分新精读文献（如 `IDNHNTKH` 光学模块转运 6R 避障规划、`EZPX9E2H` 装配误差链与数字孪生、`CSA73ZR4` 状态空间误差预测、`VM5LEYKR` 位置闭环柔顺与虚拟接触面、`G57M7BNI` 六维柔顺补偿实验）可按实际相关性复核是否写入 `MANUAL_OVERRIDE`。
- **执行动作**：回 Zotero 确认 `BJPEZ2IP` 年份后补全笔记；逐节检查 [docs/literature/CITATION_MATRIX.md](../literature/CITATION_MATRIX.md) 自动入选篇目，若有明显跨界噪声篇目，补入 `scripts/citation_matrix.py` 的 `MANUAL_OVERRIDE` 并附简短注释。
- **完成标志**：`CITATION_MATRIX.md` 无明显跨领域凑数引文，所有条目均带准确出版年份与页码论断。

---

## 二、离线代码与数据记录补全（不占真机）

### 5. 补全实验记录的数据通道字段（对应 TODO 第三节）
- **当前缺口**：自动化重测与真机耗时、力-位对齐分析还缺几项结构化字段（见 `docs/TODO.md` 第三节与 `六i` 台账）。
- **执行动作**：
  1. 测量自动重测记录 `attempts` 与每轮失败原因，写入测量结果 JSON 与 `phases.json`。
  2. 将 `phases.json` 中每阶段的 `plan_secs` 与 `exec_secs` 拆开记录，并在 `timeseries.csv` 标记规划起点与执行起点。
  3. 补上 KWR75 六维力与机械臂关节时间戳的 wall 时钟偏移同步校准参数，写入力传感日志元数据。
  4. 为安全与环境检查拒绝触发补充失败模式原因记录，便于后续论文第 6 章统计帕累托分布。
- **验收标准**：跑一次干跑或仿真闭环，检查输出的 `phases.json`、`timeseries.csv` 与归档摘要均能直接读出上述字段。

### 6. 梳理 `full` 模式与插槽角点锚定离线验证（对应 TODO 六c）
- **执行动作**：
  1. 核对 `phases_load` / `phases_unload` 中 `observe:workpiece_coarse`、`observe:tables`、`observe:slot_fine` 的配置顺序，确认与当前真机主线示教/站位流程一致，不再引用朝墙的旧扫桌位。
  2. 基于历史录制点云数据，离线验证插槽测量角点锚定方案及三条边组重复性指标（目标：≤ 2 mm）。
  3. 重启相机面板进程后，对照确认修复后的订阅线程代码在实机面板中实际生效并正常出图。

### 7. 搭建 371 mm 简化接触插入离线仿真测试（等待法兰期间推进）
- **现状背景**：KWR75 法兰尚未发加工前，真机力控与手眼重标定无法开工；第 5.5 节已定稿采用变阻尼强化学习（固定 M 与 K，在线调节阻尼缩放 B_t）。
- **执行动作**：在现有 Gazebo / 物理仿真环境中先搭建仅保留关键间隙几何特征的矩形模块—深槽简化接触模型，跑通位置型导纳外环 + 变阻尼系数接口闭环与基线策略训练通路，先解决动作钳位范围与接触状态奖励函数设计，待法兰与真机延迟探针数据到位后再标定真实参数。

---

## 三、宿主环境全量校验与真机窗口准备

### 8. 在完整 ROS 2 Jazzy 宿主终端执行 Obsidian 同步与提交前全检
- **现状说明**：受限无 ROS 环境下外部盘（`/mnt/g`）只读且无 `/opt/ros/jazzy`，本轮对 [NOTES_INDEX.md](../literature/NOTES_INDEX.md)、[CITATION_MATRIX.md](../literature/CITATION_MATRIX.md) 与 [2026-11-26_THESIS_OUTLINE_V2.md](2026-11-26_THESIS_OUTLINE_V2.md) 的修复均已写入仓库，但尚未镜像到 Obsidian。
- **执行动作**：
  ```bash
  # 1) 镜像文献索引与引用矩阵/纲要至 Obsidian（G 盘 9p 需纯内容写入）
  python3 scripts/literature_index.py --to-obsidian
  python3 -c '
  import pathlib
  v = pathlib.Path("/mnt/g/我的云端硬盘/Obsidian_Vault/OY/Do be do be do/04-课题与项目")
  (v / "01_研究规划/2026-11-26_THESIS_OUTLINE_V2.md").write_bytes(pathlib.Path("docs/plans/2026-11-26_THESIS_OUTLINE_V2.md").read_bytes())
  (v / "01_研究规划/CITATION_MATRIX.md").write_bytes(pathlib.Path("docs/literature/CITATION_MATRIX.md").read_bytes())
  (v / "04_文献与研究问题/CITATION_MATRIX.md").write_bytes(pathlib.Path("docs/literature/CITATION_MATRIX.md").read_bytes())
  '

  # 2) 加载完整 ROS 环境并跑提交前全量校验
  source scripts/source_dev_env.sh --full
  bash scripts/pre_commit_check.sh
  ```
- **完成标志**：Obsidian 目录文件字节大小与仓库一致，且 `check_docs.py`、`contract_checks.py` 与全量 `pytest test/ -q` 全部退出码为 0。

### 9. 下一次真机窗口前置检查顺序（参照 TODO 第一、二节）
1. 执行开工四项检查与夹爪状态核对（`init=5 / position_mm=0`）；若手动调整过示教器，先运行 2° 探针校准。
2. 完成插入位姿标定（`record_insert_poses.py --name ...` -> `--derive` 确认往返误差 0.000 mm -> `--promote`），接入 `reference_insert_chain()` 后先 `--plan-only` 复核整条链。
3. 在工装台架上单独通电接通 KWR75（Windows COM3 -> `usbipd`），运行 `ft_sensor_node.py --self-test` 与 10 s 零漂基线采集，再用 `scripts/ft_latency_probe.py` 实测轨迹 goal 到关节启动与停机的真实延迟，以确定后续停走节拍选型（10 Hz 或分步决策）。
