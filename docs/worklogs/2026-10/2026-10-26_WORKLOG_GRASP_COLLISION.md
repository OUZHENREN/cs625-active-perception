# 2026-10-26 WORKLOG — 抓取撞击事故复盘：静态抓取位姿 + 夹爪未校准

> 事故：20:16 真机 `load/efficient` 在 `grasp` 接近段撞倒工件，所有者触发急停。
> 本轮**只做取证、代码修复与恢复步骤**；事故后没有发出任何运动指令。

## 1. 时间线（来自日志与 phase_log）

```text
20:00:03  observe:workpiece_coarse/fine   ok（fine 的稳定性检查 PASS）
20:03–20:11 observe:tables / slot_coarse / slot_fine / baseline  全 ok，模型门禁 PASS
20:13:31  pre_grasp   ok=True，实测 TCP 与目标差 1.5 mm，verify PASS
20:13:40  gripper_init ok=True（**走了"init==5 就跳过"的旧路径，没有真正校准**）
20:13:53  grip        ok=True，报告 50.0 mm
20:14:37  grasp       chunk executor rc=6 GOAL_TOLERANCE_VIOLATED
                      TCP 停在 (0.8086,0.5582,-0.065)，目标 (0.8316,0.5918,-0.0583)
                      → 距目标还差 41 mm，在工件处被挡住
20:16     所有者急停；runner 已在 grasp 失败处自行中止（没有继续后续阶段）
```

事故后状态：`safety_mode=1`、`robot_mode=5 (idle)`、`arm_controller active`、
`/joint_states` 正常；recorder 已停止（id 20261026_195932，438 张图已存）。

## 2. 根因（按证据排序）

### R1（系统性）抓取位姿是 10-24 的静态常量，工件位姿从不重测
- `git log -p config/tasks/common.yaml`：`pre_grasp/grasp` 自 10-24 引入后**未改动**。
- `scripts/model_environment.py:90` 的 `WORKPIECE_POSE` 是常量；命令行默认值就是它，
  门禁把"常量"和"上一版模型的同一个常量"比较，所以 workpiece delta **恒为 0**。
- 结果：工件只要被移动/重新摆放，系统看不出来；`pre_grasp/grasp` 仍然照着 10-24 的位置走。

### R2（直接触发）夹爪没有真正校准，开度未被物理验证
- 旧 `ensure_gripper_initialized()` 只要桥返回 `init==5` 就跳过校准；
  `init==5` 会跨急停/断电保留，不能证明手指已回零。
- `grip` 只相信桥报告的位置（50.0 mm），没有物理校验；面板开度还有归零偏移（交接 §6）。
- 外撑夹爪的支撑臂是"随开度向前伸出"的：开度不对，工具的有效长度就不对，
  接近段会在到达抓取点之前先碰到工件——与"差 41 mm 被挡住"一致。

### R3（今天引入的放大因素）关节目标容差从 0.02 收紧到 0.002 rad
- 上午把它改成 0.002 是为了让实测 TCP 与规划终点对齐（否则差 21 mm，验收过不了）。
- 但这也意味着：**现在机械臂会精确执行旧的（可能已偏的）标称位姿**，而 10-22 那次成功运行时
  规划允许最多 1.15° 的终点偏差，反而"歪打正着"地避开了偏移。
- 结论：容差收紧本身是对的，但它要求位姿来源必须准确——因此 R1 必须一起解决。

### R4（缺失的保护）没有接触前的漂移守卫，也没有被挡住的早期检测
- 接近段整段 73 点一次执行（7.1 s），被挡住后一直推到控制器超时才失败；
- 没有"实测工件位姿 vs 抓取位姿假设"的前置比较。

## 3. 本轮修复（已提交，未发运动指令）

| 文件 | 改动 |
|---|---|
| `scripts/grasp_from_measurement.py`（新） | 从实测工件位姿推导 `pre_grasp/grasp`：`grasp = T · p_part`，接近偏移在**零件系**里保存；参考位姿代入可**精确复现**现有 [0.8316,0.5918,-0.0583]；`--measure` 用相机测，`--check` 漂移 >5 mm/1° 时退出 1 并打印修正后的 YAML |
| `config/tasks/common.yaml` | `workpiece_fine` 的检查链追加漂移守卫：工件被移动超过阈值 → 流程在**抓取之前**停下 |
| `scripts/run_pick_and_place.py` | ①夹爪**每次运行都校准**（不再相信 init==5）；②接触阶段（grasp/insert/straighten/d5_extract）至少分 4 段执行；③每段结束后实测关节与计划终点比较，差 >0.05 rad 判定"被挡住"，立即停、不重规划、不继续推 |
| `test/test_grasp_from_measurement.py`（新） | 6 个测试：公式复现、零件系抓取点、平移/旋转传播、漂移阈值、`--check` 拒绝 |

验证：`pytest` 261 passed，`contract_checks` PASS。**本轮没有发任何运动指令。**

## 4. 恢复步骤（必须按顺序，且需要所有者操作）

1. **先复位软件栈，再清急停**（10-24 急停复盘结论：驱动半死时话题会停在旧值）：
   停掉当前 launch（SIGINT）→ 重跑 A1 起驱动/MoveIt → A2 激活控制器 → 四项检查。
2. **把工件放回标称站立位置**（原来被撞倒的位置），人不要站在臂的作业半径内。
3. **测量并核对工件位姿**（臂停在精观测位，不动）：
   ```bash
   python3 scripts/grasp_from_measurement.py --measure
   ```
   - 若 drift ≤ 5 mm / 1°：现有 `pre_grasp/grasp` 仍可用；
   - 若超限：把脚本打印的 `pre_grasp/grasp/grasp_q` 更新到 `config/tasks/common.yaml`，
     再重新 dry-run / plan-only。
4. **夹爪实测**（静止时）：`status` 记录 init 与实际开度；发一次 `init` 后确认
   `init==5`，再 `grip 50` 确认物理开度（卡尺/目测），把偏移量记下来。
5. 先跑 `B plan-only`，再只跑 `pre_grasp,gripper_init,grip,grasp` 四段的 `--confirm` 小步验证，
   **不要直接跑完整 19 段**。

## 5. Capability layer

```
- environment and dependencies      PASS
- robot model and simulation        PARTIAL  仿真 insert 未通过
- kinematics, control and planning  PASS     规划/执行链可用；接触安全判据本轮加强
- vision, hand-eye calibration/TF   PARTIAL  相机链路通；工件位姿未接入任务闭环（本轮补了工具+守卫）
- active perception / NBV           NOT STARTED
- real-hardware integration/safety  BLOCKED  急停后需复位；抓取位姿来源必须改为实测
```

## 6. 下一轮唯一推荐任务

**先恢复现场并按第 4 节测量**；拿到实测工件位姿后，用
`grasp_from_measurement.py` 生成位姿并更新配置，再用"四段小步"重试抓取。
在此之前**不要再跑完整 execute**。
