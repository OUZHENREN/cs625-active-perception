# 待办（当前周）

> **维护规则（所有者 2026-11-26 定）**
> · 本文件是**唯一待办入口**，独立于工作日志：工作日志记"发生了什么"，这里记"要做什么"。
> · 每周用 `python3 scripts/todo_week.py --rollover` 归档到 `docs/todos/`，本文件只留未完成项。
> · 标记：`- [ ]` 未完成 ｜ `- [x]` 本周已完成 ｜ `- [!]` 阻塞（必须写清阻塞原因）
> · **日期归属**：连续跨零点的工作（晚上做到凌晨）一律归到**开始那一天**。
> · **需求即时登记（所有者 2026-11-26）**：所有者提出任何需求，我**当场**在下面新增一条，
>   再动手做；这样不会被后续上下文挤掉。登记时写清"要什么、验收标准"。

**本周**：2026-11-24（周一）～ 2026-11-30（周日）

## 一、下一次真机窗口（优先级最高，建议顺序）

- [ ] 1. 开工四项门 + 夹爪状态（`gripper_link.sh --status` 需 `init=5 / position_mm=0`）；示教器动过则加 `ensure_robot_ready --force` + 2° 探针
- [ ] 2. **插入位姿标定**（约 15 min，手册第 69 条）：所有者手动摆 insert / straighten / pre_insert / withdraw → `record_insert_poses.py --name ...` → `--derive`（往返须 0.000 mm）→ 确认后 `--promote`
- [ ] 3. 把标定接进 `reference_insert_chain()`（改读 `insert_calibration`）→ `--plan-only` 复核整条插入链
- [ ] 4. 效率模式收尾（约 40 min）：工件重测（coarse→选站→fine）→ **pre_grasp / grasp / grip 接触三段** → `model:slot` 端到端串联
- [ ] 5. 每阶段记录 `secs`，把固定开销新数值补进手册第 54 条

## 二、六维力（KWR75，代码已就绪，卡硬件）

- [x] 协议解析 + ROS 节点 + 全速率记录 + 出图（2026-11-25 完成，离线自检通过）
- [!] **装机**：阻塞于**法兰加工**（未发加工）→ 加工到位前 URDF/TCP/手眼都不能动
- [ ] 加工到位后：传感器安装 → URDF 加 FT frame → 更新 TCP/工具链 → **手眼重标定** → `wrench` 接入 recorder → 基线采集（零漂/噪声）
- [ ] 台架熟悉（不装机）：通电后 `ft_sensor_node.py --self-test` → 跑节点 → `ft_sensor_record.py --duration 10 --plot`，看零漂与噪声

## 三、数据通道待补（代码与单测已落地，真机采集待实机运行验收）

- [ ] ① 夹爪位置/电流（Windows 串口桥，20 Hz）——接触的间接证据
- [x] ② **测量尝试次数与失败原因**（`run_pick_and_place.py` `measure`/`observe` 均按 `MEASURE_ATTEMPTS=3` 记录并回写测量 JSON、`record[-1]["check"]` 及 `phases.json`）
- [x] ③ 规划耗时 vs 执行耗时拆开（`phases.json` `timings` + `timeseries.csv` `:exec` 标记 + `archive_experiment.py` `README.md` 展示）
- [x] ④ 传感器与关节的时钟对齐偏差（`ft_sensor_record.py` 同步监听 `/joint_states` 并输出 `.meta.json` 对齐统计；待通电同步采集实测）
- [x] ⑤ 安全闸/环境门触发记录（`classify_failure_mode` + `failure_mode` / `gate_reason` 写入运行记录与 `phases.json`）

## 四、视觉实验计划需更新（"漏项二"：`docs/plans/2026-11-13_PLAN_NBV_CHAPTER.md` 仍可用，但）

- [ ] ① 站位数量 7 → **10**，E4 可达性表重出
- [ ] ② 工作量重估：实测 2.5–3 min/次（不是 1 min）；E1 建议先做 7 角度 × n=2（≈2 h）
- [ ] ③ E5 必须记录尝试次数
- [ ] ④ 论文禁止把插槽写成"绝对定位精度 0.64 mm"（面内弱约束，手册 68），只能写**重复性**
- [ ] 暂缓（所有者 2026-11-14 决定）：O1 自遮挡、O2 外部遮挡

## 五、对比实验升级（"漏项三"，所有者认为原方案太简单，暂不推进）

> 方案见 `docs/literature/2026-11-26_EXPERIMENT_DESIGN_FROM_LITERATURE.md`（依据 Zotero 199 篇学位论文）

- [ ] M1 误差链建模与精度分配（**离线可做**，GUM + 蒙特卡洛 + 公差分配表）
- [ ] M2 杠杆臂敏感度实验（真机约 1 h）：Δx = d·Δθ，深度 0/100/200/371 mm
- [ ] M3 测量系统 R&R（真机约 1.2 h）：%GRR / NDC，tolerance 取 2 mm 间隙
- [ ] M4 六维力接触状态识别与柔顺接管（**阻塞于 M4 前置：装机**）
- [ ] M5 装配可靠度与失败模式帕累托（离线，用历史归档）
- [ ] 原 A1/A2/A3 三臂对比降级为 M1 的输入支撑，不单独作主线

## 六、论文与出图

- [ ] 论文第 2 章（遮挡目标定位与观测位选择）按 M1–M3 重构主线
- [ ] 出图入口：`~/cs625_experiment_archive/<id>/timeseries.csv` + `images/`（见 `docs/reference/data_map.md` 第 4 节）

## 六b、文献与论文（2026-11-26 新增）

- [x] 连上 Zotero 库（`G:\我的云端硬盘\Zotero`，3476 条 / 199 篇学位论文）并抽出方法与依据
- [x] 大论文纲要早期三版（V2/V3/V4）已归档：`docs/plans/archive/2026-11-26_thesis_outline_drafts/`
- [x] 同类硕博论文章法抽样（6 篇，Zotero PDF）：结论 5–6 章是常态，8 章过多
- [x] **大论文纲要 V2（现行）**：`docs/plans/2026-11-26_THESIS_OUTLINE_V2.md`
      —— 6 章（按同类硕博论文模板）；现状当基线；NBV = 站位先验 + 受约束局部优化
- [x] 已确认口径：题目暂用 A、手腕 **±150°**、允许侧移
- [ ] 待确认：① 第 3 章手眼标定方法对比做到几种解法 ② 力控降级触发时间（建议法兰 >6 周）③ ANSYS 是否纳入
- [ ] **新增研究增量：局部视角优化（I2）** —— 离线光线投射评分器（CAD+环境点云预测可见性）
      → 站位附近 roll/pitch/侧移候选 → plan_pose_ik 执行少数候选 → 与纯站位对照
      验收：同一工件位置，覆盖率/重复性提升且拒绝率可接受（真机约 4 h，可离线先做评分器）
- [ ] 待确认：论文题目/定位（保留 10-30 版题目 + 倒序写作）与写作主战场（第 2、3 章先行）
- [x] 交叉偏振已停用（所有者确认）→ 第 2 章按"已停用+原因"写，不写成已落地
- [x] 补：Zotero 里真正批注过的 3 个 PDF 已查明并完成 T1 全文精读（`N7KRDULX` 24 条、`JKP9YS3R` 5 条、`5PHN9L8R` 1 条）
- [ ] 10-30 版总方案**不覆盖不改写**（Obsidian 原件保持），v2 只做纲要对照

## 六j、插槽位姿标定 + 分段插入（所有者 2026-11-26 定，优先）

- [ ] **① 插槽位姿标定重做（角点锚定）**：现状——单次测量重复性 0.4 mm 很稳，但绝对位姿
      面内不确定（两条配准路径差 66 mm；推导 vs 手摆插入位差 28 mm；所有者观察 ~40 mm）。
      方案：三平面求交出角点 → 2 角点 + 1 法向构造坐标系（首选）；受约束 ICP 冻结面内 3 DOF（备选）。
      验收：连续 3 组 ≤2 mm/0.1°；隔 1 天再测 ≤5 mm。
- [ ] **② 分段插入（必做，不是可选）**：所有者明确"不分段基本插不进去"。
      444 mm 拆成 20–40 mm 段（首段 50 mm）；0–80 mm 视觉复核，80 mm 后力复核；
      超差回退 1 段不回起点。量化依据：0.5° 在 444 mm 处 3.9 mm（>2 mm 间隙），
      分段到 20–40 mm 后只产生 0.17–0.35 mm。
      待办：新增"按深度切段 + 每段复核"的插入执行路径。
- [x] 不采纳（所有者 2026-11-26）：暂不改 `reference_insert_chain()`；暂不看师兄数据
      （确认他每次重测 slot）

## 六f、六维力量程校核（2026-11-26 已定稿：**够用**，我方两次估算都错了）

- [x] 传感器额定（KWR75D，手册 p.11）：Fx,Fy **500 N**｜Fz **700 N**｜Mx,My,Mz **18 N·m**｜过载 300%
- [x] 需求侧（**真实 CAD 质心** + 当前抓取模式 + 7 kg）：
      体素化 5 mm → 占用 50793 体素、实体体积 6349 cm³（若 7 kg 等效密度 1103 kg/m³，说明是空心件）
      质心在 CAD 系 [-0.2059, 0.0688, -0.0114]；相对 TCP 仅 (2.8, 42.2, **-190.8**) mm
      → **水平力臂只有 42 mm**（质心几乎在抓取点正下方），静态弯矩 **2.9 N·m**（额定 18 → **0.16×**）
      → 轴向力 69 N（含夹爪 4.58 kg 共 114 N，Fz 700 → 余量 **6.2×**）
- [x] **结论：KWR75D 量程完全够，可以按原计划装机、力矩通道也应记录**
- [x] 记录我两次估算错误（已写进手册第 71 条）：
      ① 第一次把 flange→TCP 的 **353 mm 当成水平力臂**（那是竖直方向，不产生弯矩）→ 高估 2.3×
      ② 第二次用 **bbox 中心当质心**（真实质心用体素化算，水平偏移只有 42 mm 而非 331 mm）→ 又高估 8×
- [!] 实测关节力矩**不能**用于反推量程（原因见手册第 71 条）：驱动上报的 effort 量级不自洽
      （腕关节中位 0.2–0.6 N·m，而仅夹爪 4.58 kg 在 0.1 m 力臂就该有 ~4.5 N·m），
      且同一批数据出现过 730 N·m 离群值 → 只能用模型算，不能用它校核量程

## 六i、评审意见（另一智能体 2026-11-26）处理台账

- [x] 事实性修正（已逐条溯源验证属实后修改）：
      ① 78 篇笔记全中文、英文整批漏读 → 补 lit-en（7 篇英文）+ lit-cn-top（3 篇 Top-10 漏读），进行中
      ② `nbv_views.py` 实测 z-buffer 打分无分辨力（0.2704/0.2704/0.2715）→ 纲要 3.4 评分函数改为
         ICP 信息矩阵/Fisher 信息 + 高光避让 + 关节增量代价
      ③ 手腕口径修正：绝对 ±150° / 单次相对 45° / 执行层 25° / J4 建议 ±120°
      ④ 引用矩阵改为"论文自身内容（§1–§7）+ 否定句过滤 + 人工复核覆盖表"；4.5 R&R 诚实标 0 候选
      ⑤ 附录 A 与纲要 17 节对齐；根 README 同步（411 测试、13 任务、传感器已到货卡法兰）
- [x] 四项决策已由所有者批准并落地（"按照你的建议来"）：
      ① 5.5 = **变阻尼 RL**（学 B，不动 M/K）；"位移残差"降为备选对照
      ② 题目定稿《面向遮挡光学元件精密装配的可达观测规划与视触柔顺控制研究》
         （AGENTS / 纲要 / 根 README 已同步）
      ③ 第 4 章扩为 4.4 柔度变形（ANSYS 互证）+ 4.5 卡阻/楔紧（Whitney）；R&R 移到 3.6
      ④ 流程落地：AGENTS 六模式压缩为指针 + 删 Capability-gate 模板 + 手腕口径修正；
         pre-commit hook 已装（`scripts/pre-commit.hook` + `scripts/install_hooks.sh`）；
         根 README 已瘦身为"定位+上手+红线"并指向 docs/README
- [x] **取数前数据通道补全（评审④，代码与离线单测已完成）**：
      ① 测量自动重测记 `attempts` 与失败原因（写入 measurements JSON、运行日志与 `phases.json`）
      ② 每阶段 `plan_secs` 与 `exec_secs` 分离（`phases.json` `timings` 与 `timeseries.csv` `:exec` 标记）
      ③ KWR75 与关节时间戳对齐（`ft_sensor_record.py` 输出 `.meta.json` 对齐统计，待真机通电实测）
      验收：已由单测覆盖字段生成；真机一次完整运行后归档 README 将实测呈现上述字段 

## 六h、三条文献空白 → 我们的增量点（见 `docs/literature/RESEARCH_GAPS.md`）

- [ ] ① 手眼标定**残差**（mm/°）没人报 → 标定后摆 15–20 个位姿报残差 + 留出点误差
- [ ] ② 接触状态**误判率**没人报 → 力数据人工标注 → 混淆矩阵 + 误判率 + 误判后果
- [ ] ③ **停走节拍**只有 5UFZI35T 一篇支持 10 Hz → 先用 `ft_latency_probe.py` 实测 goal 延迟再定

## 六g、第五章强化学习收敛记录（已归并六i，2026-11-26 定稿）

- [x] 5.5 节已按所有者批准归并定稿为 **变阻尼 RL**（冻结 M/K、只在线调节正定阻尼 B_t；位移残差仅作可选对照组）
- [x] `THESIS_OUTLINE_V2.md` 5.5 与 `CITATION_MATRIX.md` 5.5（`KZ3CTUGL` / `GEKIMA8P` / `CW6SUARC`）已对齐落地；`EXPERIMENT_DESIGN_FROM_LITERATURE.md` 与 `force_control_interface.md` 已同步口径
- [ ] 待办：法兰加工及延时代价实测到位后，再在仿真/真机开展 5.5 变阻尼 RL 训练与 5.6 三档对照

## 六e、力控架构决策：阻抗 vs 导纳（所有者 2026-11-26 提出，待定稿）

- [ ] 决策前必须核实的三条事实（不能拍脑袋）：
      ① 我方 ros2_control 只暴露 **position** 命令接口（`src/cs625_bringup/config/sim_controllers.yaml`
         的 `command_interfaces: [position]`；真机配置同样），**没有 effort/torque 命令接口**
         → 力矩型阻抗控制目前无通路
      ② 厂商控制器**自带力控功能**（CS625 手册 3.6.6：四种力控坐标系、力保护、`get_tcp_force()`、
         进入力控前需 ≥0.02 s 延时），30004 RTSI 可高频交换关节角/速度/**TCP 实时力矩**
         → 但只确认了"读取力矩"，**没有确认能否下发力矩指令**
      ③ 现有执行器是**整条轨迹下发**（execute_assembly_chunks），不是 100 Hz 位置流
         → 位置型导纳需要先建"流式位置下发"通路（100 Hz 级）
- [ ] 初步判断（待证据定稿）：**导纳（位置型，外层闭环）** 与现有安全契约兼容性最好
      （速度上限 0.10 rad/s、25° 分段、看门狗、工作空间框都能继续生效）；
      厂商内置力控作为**对照基线**；力矩型阻抗暂不作为主线（接口未确认）
- [x] 力控接口核查（`docs/reference/force_control_interface.md`）：
      · 厂商 SDK 有 `startForceMode/endForceMode`（EliteDriver.hpp:304），limits 可限速
      · RTSI 已提供内部 TCP 力估计（`getAcutalTCPForce`），驱动以 `tcp_fts_sensor` 发布
      · **但我方 ROS 驱动不暴露力控指令**；能否启用需示教器/SDK 探针实测，不能假设
      · 决策：主线 = 自建位置型导纳（KWR75 供力）；厂商力控 = 待验证基线；直接 writeServoj = 不采用
- [x] 延迟探针工具：`scripts/ft_latency_probe.py`（测 goal→运动延迟，决定 10 Hz 还是 20–30 Hz 切片）
- [ ] 真机待做：① 示教器看力控菜单 ② 读 tcp_fts_sensor 按末端看响应 ③ 测 goal 延迟
- [ ] 文献依据：C 批（力觉柔顺）6 篇的"阻抗/导纳"使用与参数整定做法，读完后写进引用矩阵

## 六d、文献精读流水线（所有者 2026-11-26 新需求，进行中）

- [x] 建成工作清单：Zotero 全库 956 条（thesis/journal/conference），**828 条有 PDF**，
      按四大方向分类 → `runtime/literature_worklist.json`
- [x] 首批 24 篇（四方向各 6，互不重复）已派 4 个子智能体精读 → `runtime/literature_batches.json`
- [x] 笔记模板（依据所有者 9-19 阅读提示词压缩）：`docs/literature/templates/PAPER_NOTE_TEMPLATE.md`
- [x] Zotero 回写通道：本地 API 不可达 → 改用 JS 运行器 `scripts/zotero_import.js` + `--emit-manifest`
- [x] 工具：`scripts/literature_index.py`（汇总索引，可 `--to-obsidian` 复制）、
      `scripts/zotero_import_notes.py`（子笔记回写 Zotero，默认 dry-run + 备份 + 拒绝在 Zotero 运行时写入）
- [x] **T1 精读增至 93 篇**（含 `N7KRDULX`、`JKP9YS3R`、`SKHVI4C5`、`SP4IE6N4`、`CPQNLDLY`）→ `docs/literature/notes/`；`NOTES_INDEX.md` 已修复年份与 5 类结构并重生成
- [x] **T2 通读登记更新至 782 条**（`scripts/lit_triage.py` → `runtime/lit_triage/`，已严格排除全部 93 篇 T1）
- [x] **引用矩阵** `docs/literature/CITATION_MATRIX.md`：19 节矩阵已重生成，18 节候选 ≥3 篇；`3.6 R&R` 经 Zotero 3,411 条全库检索为 0 篇（仅存无附件标准 `ZIL5I4QK` ISO 9283:1998），待库外补入 `AIAG MSA 第 4 版 + ISO 9283 原文`
- [x] **纲要注入**：纲要 V2 附录 A 已重生成（19 节、缺口仅 3.6）；注意同步至 Obsidian（宿主写 G 盘）
- [ ] **Zotero 回写与 Obsidian 镜像（需在宿主环境或 G 盘可写权限下执行）**：重跑 `--emit-manifest` 覆盖 93 篇并在 Zotero Run JavaScript 执行 `scripts/zotero_import.js`；同步更新后的 `NOTES_INDEX.md`、`CITATION_MATRIX.md`、`THESIS_OUTLINE_V2.md` 至 Obsidian
- [ ] T3：正文引用到的 T2 文献按需升精读（等论文正文动笔时做）
- [ ] 更新阅读提示词 v3：把本课题四条线（装校装备/视觉测量标定/力觉柔顺/误差可靠）与
      我们的实测口径（0.42 mm/0.078°、插槽面内弱约束、371 mm 插入深度）写进固定背景

## 六c、遗留缺口（本轮盘点新增，2026-11-26）

- [x] `full` scope 配置梳理：`config/tasks/full_efficient.yaml` `phases_load` 已同步 `phases` / `load_efficient.yaml`，`phases_unload` 已补齐 `observe:slot_fine` 与 `observe:tables` 对齐 `unload_efficient.yaml`（`test_full_efficient_step_phases_aligned_with_mainline` 已覆盖）
- [ ] 相机面板修复后**重启面板复核出图**（所有者已口头确认"修复好了"，但代码在重启前生效与否需复核）
- [ ] 六维力只在**离线自检**通过，真机（通电 + COM3 → usbipd）从未跑过
- [x] 插槽测量离线核查（`measure_20261125_191925`）：`coarse_pose.py` 无 `--preset slot`，`measure_object.py --preset slot` 现有 `prior_register` 实测最大中心散布 **0.365 mm**（`slot_latest.json` `0.41 mm / 0.171°`）；若对 745 mm 非立方体槽套用 `anchor_on_corner`，交点锚定使位置跳变 ~795 mm、覆盖率跌至 0.000（散布 1.409 mm），证实应维持 `prior_register` 路线

## 七、本周已完成

- [x] 一键回粗观测位（`goto_workpiece_coarse.py`，真机 8.21 s / 到位 0.2 mm）
- [x] 一键回停车位（所有者："极其干净利落"）
- [x] 工件粗测/精测在工件移动 138 mm 后仍成功（0.98 / 0.978）
- [x] 插槽观测链 + 历史基线比对（0.5 mm / 0.07°）
- [x] 相机面板"无图像"真因修复（一个进程一个订阅线程）
- [x] 测量失败自动重测（3 次）
- [x] 环境模型可用（测量数据集补 metadata + 单位换算）
- [x] 插入链统一到 11-04 参考（`reference_insert_chain()`）
- [x] 六维力协议 + 节点 + 记录出图（离线验证）
- [x] 数据地图 `docs/reference/data_map.md` + `scripts/data_report.py`
- [x] 待办体系（本文件）+ 归档脚本
