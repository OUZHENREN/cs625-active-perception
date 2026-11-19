# 2026-11-19 工作日志：执行安全硬闸、面板重构、开工/收工接入相机

> 本轮分两段：先按所有者要求做**计划**（`docs/plans/2026-11-19_PLAN_PANEL_AND_STARTUP.md`，
> 三份只读审计支撑），所有者确认三项决定后开始实现。
> **本轮没有动机械臂**：所有验证都是离线/离屏/只读命令。

## 1. 执行安全硬闸（最高优先，起因是 11-18 的速度失控）

### 1.1 事故复查

```text
现象（所有者）：速度极快、完全超出边界。
数据：计划本身最大关节速度 0.138 rad/s，×time-scale 3.0 后 0.046 rad/s（2.6°/s），总时长 18 s。
     ⇒ 计划并不快。
但机械臂最终停在"2 倍指令位移"处：
     J1 45.1 → 128.7（目标 86.0）   约 2.1×
     J5 96.8 → −0.1（目标 49.2）    约 2.0×
     J6 88.5 → 156.8（目标 121.9）  约 2.0×
结论：问题不是"计划快"，而是**执行器对行程没有任何上限，且完全信任计划时间戳**。
```

### 1.2 现在的硬闸（代码强制，非参数）

```text
scripts/execute_assembly_chunks.py
  · govern_timestamps()：不信任计划时间戳，按 MAX_JOINT_SPEED_RAD_S = 0.05 rad/s
    （≈2.9°/s）重新配时；只会更慢（max(1.0, time_scale)），执行前打印治理后时长与最大速度。
  · joint_deltas() + MAX_JOINT_DELTA_RAD = 0.44 rad（25°）：超限计划直接
    REFUSED_LARGE_MOVE，什么都不发；要放行必须打印 6 个关节角并经所有者同意。
  · 治理后仍超速 → REFUSED_SPEED。
scripts/plan_pose_ik.py
  · 同一道 25° 闸：超限直接拒绝写计划（exit 8），坏计划根本产不出来。
```

**用 11-18 真实计划验证**：`wrist_2 = 47.6° > 25°` → 会被拒绝；若强制放行，6.0 s → **49.9 s**、
最大速度 **1.0°/s**。

**规则固化**：`AGENTS.md` 新增 `Execution safety contract` 章节；`docs/agent_memory.md`
加硬闸说明；`test/test_execute_assembly_speed_guard.py` 6 条测试（纯函数，不连 ROS）。

## 2. 面板崩溃（所有者决定：删掉"界面"行）

```text
根因：_refresh_keys 写 key_labels["界面"]，该键从未注册（9bc0b0c 加入时漏了元组）。
     每 500 ms 一条 KeyError；异常从 _tick 逃出后 _refresh_session（分支/提交）与
     _refresh_captures（最近采集）永远执行不到 ⇒ 那两项一直是"—"、采集表一直空。
处理：删掉该行；所有写入改走 _set_key()（缺键忽略，不再抛异常）。
验证：离屏 3 s @3600×2000，stderr 0 字节（之前每 500 ms 一条回溯）。
```

## 3. 开工①/收工⑧ 接入相机（所有者要求 1）

```text
camera_mode.py
  · 新增只读 health [--expect modeling]：pid + launch 身份校验 → rclpy 图 API 数
    color/depth/camera_info 发布者 → 收一帧 CameraInfo 校验分辨率 → 固定一行输出，
    rc 0/1/2。不启停、不重启、不写 pid。
  · 旧的 status 在相机没起时实测 37 s 且仍返回 0，不能当开工判据。
  · CS625_CAMERA_IP 检查移出 import，stop/status/health 不再因缺 IP 直接退出。
start_real_stack.sh
  · 修三个"假装成功"：activate / preflight 的退出码必须检查；门顺序改
    activate → ensure_robot_ready → preflight；"已在运行"判据改用 control_health 整体 rc。
  · 相机段（--camera auto|on|off，默认 auto）：健康则复用；检测到 RVS_AI_Center 则跳过
    并说明（RVS 与 ROS 相机互斥）；否则 timeout 150 启动 modeling 并 health 复核；
    失败打印 4 条可照做命令并 exit 5。
  · --check-only 也报告相机；结尾一行汇总（驱动 / 四项门 / 相机）。
stop_all_drivers.sh
  · 相机改为 runtime/camera.pid 优先 + 进程身份校验（comm 与 cmdline），停后清 pid；
    收尾报告"相机驱动 / pid 文件"两行；硬编码 IP 改为从 ~/.cs625_local.env 读。
.vscode/tasks.json
  · ①/⑧ 不再无条件打印"完成"，按退出码分别提示并 exit $code。
```

**验收（只读路径，未启停真实相机）**：

```text
python3 scripts/camera_mode.py health   → rc=1，reason=stale_pid（明确原因）
bash scripts/start_real_stack.sh --check-only → 驱动与相机两段状态都打印
bash scripts/stop_all_drivers.sh --dry-run    → 只报告不执行，pid 文件 mtime 不变
```

## 4. 效率页面板重构（所有者要求 2）

```text
左栏 400 → 1200（日志视口 388 → 1188，3.06×）；日志自动换行 + 横向滚动条关闭；
阶段 3 列 / 速度 4 组每行 / 参数 1 行（框高 287/132/99 → 191/78/51）。

相机：fit 模式默认（KeepAspectRatio，只缩小不放大）；1:1 模式保持原像素但滚动条 AsNeeded；
      2560 图不再把窗口顶到 3560×2140；相机列 min 660 / max 1314 + 伸缩。

视觉流程 + 实验数据：删掉重复的 QGroupBox 标题；删掉每 2 s 的 resizeColumnsToContents；
数据框移到流程下方；流程/数据行高按 QFontMetrics（46/29 px）；缺失字段显示"—"。

底行：206 → 320；3D 卡固定 640、运行记录占其余；时间列显示 HHMMSS + 全名 tooltip。

采集刷新：500 ms → 2 s TTL 缓存（它扫约 190 个 json；崩溃修好后它才真正开始跑）。
```

**实测（3600×2000）**：左栏 1200；日志 1188×1544；相机 1280×960 无裁切；流程表高 261；
底行 3D 640 + 运行记录 1731；tail 底部约 519 px 为动态伸缩区。

## 5. 一个诚实报告的未接线项

```text
面板的"速度覆盖 / 重复次数 / 跳过位姿门 / 跳过静置"四个控件**仍不影响命令**：
run_task.py 与 run_pick_and_place.py 都没有对应参数（--speed-override / --repeat /
--no-gates / --no-settle 不存在）。硬拼会让 run_task 以 "unrecognized arguments" 启动失败。
本轮做法：加 tooltip 说明"暂不生效"，运行时在日志里 warn 列出被忽略的设置。
要真正生效需要扩展 run_task.py（下一轮，需所有者确认是加参数还是先禁用控件）。
```

## 6. 验证

```bash
python3 -m pytest test/ -q                                  # 291 passed
python3 scripts/check_docs.py                               # ALL DOC CHECKS PASSED
python3 -m pytest test/ -q -k contract                       # 9 passed
QT_QPA_PLATFORM=offscreen timeout 4 python3 scripts/cs625_panel.py --no-probe --size 3600x2000 2>/tmp/e; wc -c /tmp/e   # 0
QT_QPA_PLATFORM=offscreen timeout 4 python3 scripts/cs625_panel.py --no-probe --scale 1.5 2>/tmp/e2; wc -c /tmp/e2      # 0
QT_QPA_PLATFORM=offscreen timeout 6 python3 scripts/cs625_panel.py --smoke /tmp/safe.png                               # 安全页正常
python3 scripts/camera_mode.py health; echo rc=$?            # 相机没起 → rc=1
```

## 7. 能力层与状态

```text
Capability layer: 人机界面 / 开工收工 / 执行安全
  执行速度与行程      PASS（硬闸 + 6 条离线测试；未在真机跑过运动）
  面板稳定性          PASS（离屏双档 stderr 0；安全页 smoke 正常）
  面板布局            PASS（4K 实测；<2516 px 宽会横向压缩，1080p 本就不支持）
  相机启停接入        PARTIAL（只读路径已验证；"相机健康 rc=0"与真实 modeling 启动待真机验证）
  面板运行参数        NOT WIRED（4 个控件无对应 CLI 参数，已在日志中 warn）
  真机运动            NOT RUN（本轮未动机械臂）
Intentionally not done:
  · 观测失败策略（所有者要求放到后面商量）；
  · run_task.py 的 --resume / resume_task.sh（计划 P1，下一轮）；
  · 相机接入的真实启动验证（需要相机可启动时由所有者执行一次）。
```

## 8. 补：效率入口的"重挂 + 断点续跑"（所有者 2026-11-19 追加）

```text
背景：示教链有 resume_teaching.sh（重启驱动栈 + --resume），但面板效率页用的
     run_task.py 完全没有——失败只能从头重来。这就是"能力退化感"的技术根因。

实现（三处）：
① run_pick_and_place.py：新增 --start-index（按 0-based 序号跳过已完成阶段），
   并在阶段日志里记录 index —— 必须按序号，因为 load_efficient 里
   observe:slot_coarse 出现两次，按名字会定位到错误阶段。
② run_task.py：新增 --resume，读 runtime/run_task_resume.json（失败时自动写入：
   scope/mode/world/step/失败阶段/start_index/总数），校验 scope/mode/world 一致后
   透传 --start-index；成功跑完自动删除续跑点；每次续跑前用 ensure_robot_ready --force。
   同时把 experiment_recorder 的停止/归档移进 finally —— 失败提前 return 时它以前会
   一直跑且不归档。
③ 新增 scripts/resume_task.sh（对称 resume_teaching.sh，**人工执行不自动**）：
   读状态 → 打印计划 → 夹爪夹持检查（恢复点在 grasp 之后时）→ ensure_robot_ready --force
   → 可选 recover_driver_stack.sh --timeout 120 → run_task.py --resume。
   符合"急停后人工确认安全再恢复、不自动续跑"的铁律。
   .vscode/tasks.json 增加 ⑩ 续跑 · 效率入口（共 10 个任务）。

离线验证：
  · 单测 9 条（状态校验、重名阶段按 index、日志解析、runner_command 透传）；
  · 合成续跑状态 + 机器人离线 → 脚本正确拒绝（safety_mode=7 不是 NORMAL），
    未执行任何运动。
```

## 9. 观测策略：摆放约束 + 两平面回退缺陷（所有者 2026-11-19 追加）

### 9.1 摆放约束（先做，0 风险）

```text
配置：站位 0 / −16 / −30 / +11 / +11_left8 各加 reference_measurement，指向该站位
     验证时的工件测量文件；−23 与 −45 没有可信标定数据，如实留空（检查会打印"跳过"）。
判据：工件相对该站位参考位置的 平移 ≤30 mm、yaw ≤3.5°
      （依据 capture 审计的投影模型：yaw 差 3.3° 时第三面要稳定出 3 个面需 ≤30 mm）。
行为：选站后打印偏差；超限则 exit 7 并给出可照做提示（摆回参考位置，或换覆盖当前偏角的站位）。
边界：比较加 1e-6 epsilon（算出的 3.500000000000011 不该判超限）。

用 11-18 真实数据验证：−16 站位、工件偏离参考位置 58.7 mm → REFUSED（rc=7）。
以前这条路径会先把臂移过去、再在精观测阶段失败。
```

### 9.2 两平面回退缺陷（软件根因）

```text
现象：11-18 三组测量彼此一致，但扶手分只有 0.134/0.18/0.19 → 质量门判 0 组可信。
根因：两平面回退的初值可能偏 180°（正方形截面的镜像分支），剔点循环用它删掉
     17 549 点、其中 84% 落在扶手区；随后在同一份被删空的云上算扶手分。
修复：几何拟合仍剔点，但**对称性判别一律用未剔点的原始云**。
验证：用 11-18 归档数据重跑三组 → 扶手 0.882/0.884/0.898，valid=True（之前 False）。
```

### 9.3 缺一轴显式化

```text
只有两个可见面时，沿对象轴的位置由模型先验给出、没有观测约束，不能当 1 mm 级结果。
现在这类组显式判为不可信，拒绝理由写成「缺一轴（沿对象轴 N 无观测约束，需换视角）」，
不再一律叫"退化"。判定函数提到模块级并加了 4 条测试。
```

### 9.4 本轮验证

```bash
python3 -m pytest test/ -q          # 309 passed（新增 9 条：摆放 5 + 可信度 4）
python3 scripts/check_docs.py       # ALL DOC CHECKS PASSED
python3 scripts/nbv_from_reference.py --auto-station \
  --reference-measurement runtime/measurements/workpiece_zero.json \
  --current-measurement runtime/measurements/workpiece_coarse.json
# → 摆放检查 58.7 mm / 3.50°，REFUSED，rc=7
```

### 9.5 未做

```text
· 补偿位姿（跟随工件平移）：J4 抱闸未修，且 11-18 已证明大改构型风险高 → 修好后再做；
· 选站加"平移一致性"排序（−23 站位 derived−stored 只差 10.9 mm）：留待下一轮，
  需要先 plan-only 验证 −23 在当前臂位下的 IK。
```

## 10. 选站：按"与当前工件的匹配度"排序（所有者 2026-11-19 追加）

### 10.1 为什么改

```text
11-18 选站只按偏转角最近：−16（差 3.46°）以 0.09° 击败 −23（差 3.54°）。
但用"站位存储位姿 vs 当前推导位姿"一算：
    −16 差 80.2 mm（工件已离开它验证时的位置）→ 第三面只剩 2 个面
    −23 只差 10.9 mm
yaw 误差几乎相同时，应该选"与当前这件工件更匹配"的站位。
```

### 10.2 改法

```text
station_plan 先算推导位姿，再对容差内候选算 |stored_tcp − derived_tcp|（delta_mm），
按 delta_mm 升序 + yaw 误差次键排序；排序结果写进 plan["ranked"] 供打印/诊断。
特殊处理：family == taught（+11 / +11_left8，所有者手摆）不用 delta 排序——它们的
视角本来就不是由 0° 推导来的，delta 对它们不是质量指标；改为按偏转角误差排，
偏转角差 ≤5° 时优先选用（保留原先"手摆站位更贴合该偏角"的意图）。
摆放检查不过的候选排到最后。
```

### 10.3 验证

```text
真实数据（11-18）：选站结果由 −16（80.2 mm）变为 −23（10.9 mm），与 capture 审计建议一致。
plan-only 复核 −23 可达性（不动臂）：
  IK 有解、轨迹抽样无接触，但需要 shoulder_pan 36.4° / wrist_2 36.4° / wrist_3 32.4°
  → 被 25° 行程硬闸拒绝（REFUSED_LARGE_MOVE）。
  结论：−23 选得对，但执行必须拆段；这是下一步。
测试：新增 4 条（排序偏好更小 delta、ranked 诊断、摆放差计算、无参考返回 None），
      全套 313 passed。
```

### 10.4 下一步

```text
· 把"到 −23 站位"的移动拆成每段 ≤25° 的两段（先肩转、再腕部），每段执行后复核；
· 或者改用 MoveIt OMPL 规划（plan_pose_once）走一条完整轨迹；
· 然后才在真机上验证 −23 站位的精观测质量（预期第三面余量是 −16 的 2.2 倍）。
```

## 11. 构型连续性 + 超限自动分段（所有者 2026-11-19 两条要求）

### 11.1 构型连续性（肩/肘/腕分支）

```text
原则：同一个 TCP 位姿通常有 8 组 IK 解（2×2×2 分支），末端位置一样但整条手臂的排布不同。
     跨分支切换会让手臂在关节空间里大幅重排（可能撞，也会让相机姿态跳变）。
做法：用肩转 / 肘 / 腕2 的符号组成构型标志，先比"是否同分支"，再比整臂变化、腕部变化。
输出示例（−23 站位实测）：
     IK 解：整臂变化 2.49 rad，腕部变化 36.4°
     构型分支：当前 (1, -1, 1) → 目标 (1, -1, 1)（同分支，连续）
```

### 11.2 超限自动分段

```text
要求：超过 25° 行程硬闸、但终点仍在绝对角度区间内 → 程序自己分段，不要让人手工拆。
做法（scripts/execute_assembly_chunks.py）：
  · split_plan()：先把关节空间折线按"半步长"加密，再贪心切成每段 ≤25° 的连续段；
  · 相邻段共享边界点、每段时间戳从 0 起、终点与原计划一致；
  · 逐段执行，速度治理按段生效；
  · --no-auto-split 恢复旧的"直接拒绝"；--allow-large-move 表示"不分段、一次下发"（须所有者同意）。
plan_pose_ik 不再拒绝，只打印行程与预计段数。

验证（−23 真实计划）：
  36.4° > 25° → 自动分 2 段：第 1 段 17 点/4.0 s/最大 24.3°，第 2 段 9 点/2.0 s/最大 12.1°
  连续性通过、终点一致。
测试：新增 5 条分段测试，全套 318 passed。
```

### 11.3 现在的执行链

```text
plan_pose_ik（选同分支解 + 打印行程/段数）
  → 写单份计划文件
  → execute_assembly_chunks（超限自动分段 → 速度治理 → 逐段下发 → 逐段容差校验）
```

### 11.4 仍未在真机验证

```text
· 自动分段只在离线用真实计划验证过段划分；真机执行需所有者在场；
· −23 站位的精观测质量（预期第三面余量是 −16 的 2.2 倍）尚未实测。
```

## 12. 真机执行：自动分段跑通，选站改为按实测覆盖率（所有者 2026-11-19）

### 12.1 真机结果

```text
第一次（选站 −23，yaw 只差 1.2°）：
  换位：构型同分支 (1,-1,1)，自动分 2 段，到位复核无接触 ✅（首次真机跑通自动分段）
  精观测：3 组出位姿、3–4 面、扶手 0.87–0.91、重复性 0.1 mm
        但 coverage 0.927 / 0.882 < 0.95 → REFUSE

第二次（手动规划到 −16，yaw 差 5.8°）：
  换位：自动分 2 段，两段 SUCCESSFUL，末态偏差 0.0006 rad（0.03°）
  精观测：verdict OK，coverage 0.9725，fitness 0.986/0.968/0.977，
         扶手 0.911/0.917/0.925，重复性 0.24 mm / 0.053°
```

结论：**"偏转角更近"不等于"测得更好"**。−23 差 1.2° 却测不过，−16 差 5.8° 却测得很好。

### 12.2 选站判据改为实测覆盖率

```text
config/tasks/common.yaml：0/−16/−30/+11/+11_left8 各加 empirical_coverage
  （0.991 / 0.967 / 0.972 / 0.967 / 0.967，来自各自验证时的那次测量）
  −23 与 −45 留空（没有可信测量）
排序：容差内 → 实测覆盖率降序 → 偏转角误差 → stored−derived delta
      没有实测覆盖率的站位排在有数据的后面，按偏转角兜底（保持旧行为）
plan["ranked"] 带上 coverage，面板/日志能看到"为什么选它"
```

### 12.3 当前摆放与摆放检查的冲突（待所有者决定）

```text
当前工件中心 (0.6615, 0.657)，相对 −16 参考位置差 40 mm / 5.8°，
相对 −30 参考位置差 47 mm / 7.7° —— 都超过 30 mm / 3.5° 的摆放门。
所以自动选站会选中 −30（覆盖率最高）但随即被摆放检查拒绝（rc=7）。
而用 --view-key -16 显式指定时不做摆放检查，实测 0.9725 通过。
⇒ 说明 30 mm 阈值偏保守（40 mm 实测仍能过）；所有者计划接下来每 5° 精拍并严格控制
   摆放偏移，届时该冲突自然消失。是否把阈值放宽到 50–60 mm，等所有者决定。
```

### 12.4 本轮修掉的三个真机 bug（已单独提交 8960d94）

```text
① execute_assembly_chunks 把 glob 返回的字符串当 Path 用 → 第一步就崩（臂没动）；
② ensure_robot_ready 只等 2 s 读状态，DDS 发现慢时读到 None → 误判"安全模式异常"；
③ run_task 提前返回时 finally 里 KeyError: 'recorder'。
另：auto_station 以前只转发规划器最后一行，现在把构型分支/行程/自动分段都转发了。
```

### 12.5 下一步（所有者计划）

```text
+30° … −45° 每 5° 精拍一遍，严格控制工件摆放的平面位置偏移，
用这批数据重新填 empirical_coverage（届时每个站位都有实测值，选站判据才真正数据驱动）。
```

## 13. 精拍开始：站位 tier、登记工具、0° 实测登记（所有者 2026-11-19 晚）

### 13.1 站位优先级 tier（新拍优先，旧站位兜底）

```text
所有者定：新拍的那批示教位姿若站得住，就成为首选；现有站位降为第二选择。
实现（config/tasks/common.yaml + scripts/nbv_views.py）：
  · 条目可带 tier（1 = 新拍首选；缺省 2 = 现有兜底）与 yaw_deg（新站位与已有角度
    撞名时用它指定设计偏角，例如 '0@20261119'）；
  · 排序：tier 升序 → 有实测覆盖率优先 → 覆盖率高优先 → 偏角误差 → stored−derived delta；
  · 决策日志同时打印 tier、实测覆盖率与候选排序，选站理由可见。
```

### 13.2 登记工具（不再手写 YAML）

```text
scripts/add_station_from_measurement.py：
  读精拍测量 JSON → 校验（verdict=OK、覆盖率 ≥0.95、测得偏角与 --yaw 差 ≤1.5°）
  → 生成条目（tcp/quat 取当前机器人位姿，reference_measurement 指向该 JSON，
     写 empirical_coverage / tier / yaw_deg / workpiece_pose）
  → 默认只打印；--write 写入并回读校验。
用法（键以负号开头时必须用等号）：
  python3 scripts/add_station_from_measurement.py \
      --measurement runtime/measurements/workpiece_at_0_fine.json \
      --key '0@20261119' --yaw 0.0 --write
  # 负角：--key=-20@20261120
```

### 13.3 0° 精拍实录（今天只拍 0°）

```text
1) 所有者把工件绕**标记角点**转到 0° 并摆好；
2) 反控端口曾掉线（ESTAB 0 条）→ ensure_robot_ready --force → 恢复 3 条；
3) 移动到原 0° 站位 (0.63546, 0.32805, 0.46654)：
   构型同分支 (1,-1,1)，自动分 2 段，ALL_ASSEMBLY_CHUNKS_OK，末态偏差 0.0006 rad；
4) 粗观测三组位姿不一致（>2 mm，未采纳）→ 直接做无先验精观测：
   verdict OK，coverage 0.954，3/3 组可信，3–4 个面，扶手 0.87–0.90，
   重复性 0.30 mm / 0.022°，测得 yaw = 0.0°（与目标一致）；
5) 登记为 tier-1 站位 '0@20261119'（reference_measurement 指向该次测量）。
```

### 13.4 选站验证

```text
当前工件位姿下：
  选中站位 0@20261119（tier 1，差 0.0°，实测覆盖率 0.954）
  候选排序：0@20261119(t1,cov 0.954,0.0°) > 0(t2,cov 0.991,0.0°) > +11(t2,cov 0.967,11.0°)
→ tier 1 正确压过覆盖率更高的旧站位（所有者要的"新拍优先"）。
→ 摆放检查：工件相对该站位参考位置 0.0 mm / 0.00°，在容差内。
```

### 13.5 今天没做 / 明天继续

```text
· 只有 0° 一个新站位；其余角度（+30…−45 每 5°）明天拍；
· 0° 的 0.954 略低于旧站位历史 0.991，所有者计划"之后进行微调"——微调后重新拍、
  重新登记（同 key 覆盖即可）；
· 拍完每个角度后用同一个工具登记，选站判据将逐步完全由实测数据驱动。
```

### 13.6 验证命令

```bash
python3 -m pytest test/ -q            # 331 passed
python3 scripts/add_station_from_measurement.py \
  --measurement runtime/measurements/workpiece_at_0_fine.json --key '0@20261119' --yaw 0.0   # dry-run
python3 scripts/nbv_from_reference.py --auto-station \
  --reference-measurement runtime/measurements/workpiece_zero.json \
  --current-measurement runtime/measurements/workpiece_at_0_fine.json
# 预期：选中 0@20261119（tier 1）
```

