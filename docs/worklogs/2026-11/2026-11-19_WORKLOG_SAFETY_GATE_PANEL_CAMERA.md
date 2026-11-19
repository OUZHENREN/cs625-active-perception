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
