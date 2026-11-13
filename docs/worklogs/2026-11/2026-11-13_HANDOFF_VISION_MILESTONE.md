# 交接文档：视觉模式里程碑（2026-11-13 结束时的状态）

> 给下一段对话用。先读这一页，再看 `2026-11-13_WORKLOG_NBV_RELATIVE_VIEW.md` 的细节。

## 1. 一句话现状

```text
测量链（粗观测 → 精观测，带先验）已在真机稳定达到 0.14–0.5 mm / ≤0.2°，
并接进效率模式流程；"自动换视角（NBV）"未能成功，已按所有者决定收窄为
"预教站位 + 按偏转角自动选站"（方案 A），尚未实现。
```

## 2. 已经可用（可直接依赖）

| 能力 | 入口 | 实测 |
|---|---|---|
| 工件粗观测（带先验） | `measure_object.py --stage coarse --prior auto --auto-release` | 覆盖率 0.98，yaw 检出，3 组一致 |
| 工件精观测（带先验） | `measure_object.py --stage fine --prior <coarse.json>` | **0.14–0.46 mm / 0.04–0.10°** |
| 插槽精观测 | `measure_object.py --preset slot --stage fine` | **0.64 mm / 0.063°** |
| 相对视角推导（数学） | `nbv_from_reference.py`（dry-run） | 六个角度全部给出正确目标位姿 |
| 安全闸 | `plan_pose_ik.py` / `plan_pose_once.py` / `move_tcp_linear.py` | 越界/碰撞/已接触 一律拒绝（见 §5） |
| 效率流程观测前缀 | `run_task.py --scope load --mode efficient --until measure:workpiece_fine` | 跑通，整段约 1 分钟 |

## 3. 关键配置（改这些就能调行为）

```text
config/tasks/common.yaml
  workpiece_zero_quat           yaw=0° 基准（实测四元数）
  workpiece_views{...}          已验证视角表：0 / +11 / +11_left8 / −16 / −23 / −30 / −45
  observe.workpiece_coarse_efficient   效率模式起点 = 工件粗观测基准点位
  efficient_start_pose          同上（带用途备注）
  observe.workpiece_coarse      示教模式的固定粗观测位（未改动）
  workspace_bounds              TCP 安全作业空间 x[0.3,1.1] y[0.1,0.9] z[0.05,0.95]
  workpiece_focus               [0.85, 0.65, 0.13, 0.30] 球裁

config/tasks/{load,full}_efficient.yaml
  phases 开头：observe:workpiece_coarse_efficient → measure:workpiece_coarse
              → nbv:workpiece（当前会安全拒绝）→ measure:workpiece_fine → …
  observe_checks：
    workpiece_coarse  = measure_object --stage coarse --prior auto --auto-release --prior-max-age-min 30
    workpiece_fine    = measure_object --stage fine --groups 3 --prior runtime/measurements/workpiece_coarse.json
```

## 4. 今天的关键认知（避免重走弯路）

```text
① 相机在 TCP 后方约 0.3 m ⇒ 转腕 = 把相机甩出大圆弧：换视角必须平移，不许转腕。
② 多起点 ICP 会把同一张图解成两个位姿（对称翻转）：必须用【粗位姿做先验】筛解。
③ 覆盖率指标曾经在测"裁剪干不干净"，已改为"剔背景后的命中率"。
④ 关节限位 ≠ 位置安全：必须同时限制 TCP 作业空间。
⑤ 规划场景里没有桌子/夹具（get_planning_scene 返回 0 个对象），所以"无碰撞"只代表不自撞。
⑥ 记录基准位姿前必须连续多次读取确认静止（曾把 z 差 10 cm 的旧值当新值）。
```

## 5. 安全红线（写在 docs/agent_memory.md 132–138 条）

```text
· 绝不屏蔽碰撞报警（134）
· 当前状态已接触 → 一律不动（135）
· 大幅改构型必须先打印 6 个关节角并经所有者同意（135）
· 关节建议区间（136）+ TCP 作业空间（137 附带）+ 速度上限 0.03
· 抓取前不看 TCP 碰撞；抓取后才看附着工件碰撞（132）
```

## 6. 尚未完成 / 已知失败模式

```text
· 自动 NBV：推导出的位姿需要大改姿态 → 危险；纯平移目标越界 → 被拒。
  下一步方案 A：预教 3–5 个站位（按偏转角区间），NBV 只做"选站"。
· 抓取 / 插入：从未在真机验证（只跑过 plan-only）。
· 规划场景无环境：桌子/夹具对 MoveIt 不可见（apply_real_cell_scene.py 报告发布成功，
  但查询仍为 0）——真机自动运动前必须修好。
· 精观测偶发不稳定：某组落错分支导致"可信组 < 2"，带先验可缓解大部分。
```

## 7. 复现命令（按顺序）

```bash
source scripts/source_dev_env.sh --full
python3 scripts/ensure_robot_ready.py                       # safety=1 robot=7
python3 scripts/apply_real_cell_scene.py                    # 场景（目前不生效，见 §6）
python3 scripts/run_task.py --scope load --mode efficient --world real --confirm \
        --until measure:workpiece_fine                      # 观测前缀
python3 -m pytest test/ -q                                  # 296 passed
```

## 8. 对后续"主动视觉实验"的影响（所有者关心的问题）

```text
影响很小，而且是有利的：

· 方案 A（预教站位 + 自动选站）本身就是主动视觉研究里的【基线策略】之一
  （fixed / predefined views）。有了它，NBV 才有对比对象，实验才成立。
· 今天的成果（测量链、质量门、先验消歧、相对视角推导）都是 NBV 研究的基础设施，
  不依赖"自动换位"是否成功。
· 真正的限制在硬件：相机后置 0.3 m + 线缆限位，使"任意位姿自由移动"不可行。
  因此论文里的"连续 NBV"建议放在仿真里做（Gazebo 可用），真机做【离散站位选择】的验证，
  两者用同一套候选生成与评分接口（这正是仓库既有的 view_planning 分层设计）。

结论：方案 A 不阻碍研究，反而提供了可复现的基线与安全边界。
```
