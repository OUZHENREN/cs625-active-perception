# 真机工位参考

> 本文合并归档原件 `real_run_reference.md`（真机轨迹记录索引）与 `real_hardware_readiness.md`（真机就绪门），并按当前实际开工流程重写检查项。真机是 Elite CS625；**2026-10-22 已完整跑通过一次全流程**，主干关节轨迹都录下来了（取出放回段未录全，见 §5）——仿真遇到问题先拿这些记录对比，不要重新猜参数。

## 1. 工位与坐标约定

| 项目 | 值 | 出处 |
|---|---|---|
| 插槽夹具位姿 `slot_origin`（夹具顶面高度） | z = **0.443245 m**（≈0.4432），xy = 1.241629, 0.181709 | 两份原件未记录；见 `sim_scene_from_real_runs.md` |
| 插槽夹具朝向 | quat (-0.001629, 0.004253, 0.003384, 0.999984) | 同上 |
| 工件桌顶面 z | −0.0910 m（点云拟合） | 同上 |
| 插槽桌顶面 z | −0.0407 m（点云拟合） | 同上 |
| `base_link` 坐标轴 | **两份原件未记录**；仓库其他文档记为 **+Z 竖直向上、右手系（ROS REP-103）** | `docs/manuals/2026-11-03_MANUAL_REAL_MACHINE_EXPERIENCE.md` 第 15 条、观测基线工作日志 |
| 角度正方向 | **两份原件未记录**；仓库其他文档记为绕 `base_link` **+Z 右手定则、俯视逆时针为正**（+X→+Y） | 同上 |
| 相机与手眼外参 | **两份原件未记录** | 手眼链未验明，见 `simulation.md` §4.2 |
| 工具 TCP / 法兰关系 | **两份原件未记录** | 见 `simulation.md` §3.7 |
| 工作空间边界、关节限位 | **两份原件未记录** | 限位：肘 ±180、其余 ±360；手拖后经验上先拨回 ±180° 内再作业 |

两份原件没有坐标轴/角度约定，上表标注出处的数值取自本仓库其他已归档文档，未做任何推算。工件位姿是任务变量，每次任务重测；夹具与桌面是恒定量。

> 来源：归档原件 real_run_reference.md、real_hardware_readiness.md（两份原件未记录坐标约定，表中数值另注出处）

## 2. 真机轨迹记录索引

记录位置 `~/cs625_legacy_snapshot_20261020/runs/<run>/`；工具 `python3 scripts/replay_real_run.py --list`（共 **49 条**轨迹）／`--run <run>` 原样回放。这些记录速度慢，但**动作与顺序是对的**（所有者确认）。

### 2.1 完整流程那次（`20261022_full_task_e2e/meta.json`）

```text
S2/S3 re-observed after model change
pre_grasp (0.7677, 0.4903, -0.0595) reached
gripper init at pre_grasp; contact width 52.2 mm; hold 52 mm
grasp feed -> 0.829, 0.588, -0.058; workpiece follows (owner confirmed)
lift 15.5 cm total; withdraw 120 mm
transit 0.95, 0.45, 0.35
pre_insert 1.0996, -0.0125, 0.3773 (tilted -10 deg) via staging point
insert straight (tilted) -> 1.157, -0.013, 0.032
rotate to straight (ry 9.5 deg -> -0.49 deg)
release: retract fingers 51.4 -> 30 -> 20 -> init 0
pure translation withdrawal 40+40+50+50 mm (no rotation)
return to slot baseline 1.039, -0.017, 0.740
```

三条**必须照抄**的经验（写在同一个 meta 的 `notes` / `safety_events` 里）：

1. **`slot collision removed during insertion`** —— 真机插入时插槽碰撞体是**从规划场景里去掉**的（所以仿真的 `NO_COLLISION_CHECK_PHASES` 不是绕过检查）；
2. **`workpiece collision via senior attach`** —— 抓取后工件**以 attach 跟随工具**，不留在场景里当碰撞体，之后的规划不该再和工件做碰撞检查；
3. **`controller recovery required after goal_time aborts`** —— 真机也出现过 `goal_time` 中止，需要恢复控制器；仿真里 `GOAL_TOLERANCE_VIOLATED` 本身不算异常，**异常的是"物理上过不去"（力矩饱和、RTF 崩塌）**。

### 2.2 按阶段的轨迹（`--list` 全量 49 条，这里只列主干）

| 阶段 | run / 文件 | 点数 | 时长 | 终点关节 (pan, lift, elbow, w1, w2, w3) |
|---|---|---|---|---|
| S2 候选视角 | `20261022_s2_candidate/s2_candidate_plan.json` | 106 | 10.46 s | 0.37, -1.20, -2.32, -0.32, 2.14, 0.97 |
| pre_grasp（修正版） | `20261022_pre_grasp_exec/pre_grasp_plan.json` | 115 | 11.36 s | 0.72, -2.02, -2.00, -0.67, **1.58**, 1.29 |
| pre_grasp（分段） | 同 run `pre_grasp_chunk1..5.json` | 24×5 | 2.2–2.3 s | 逐段 |
| grasp（关节） | `20261022_grasp_joint_partial/grasp_joint_plan.json` | 33 | 3.17 s | 0.75, -2.14, -1.82, -0.78, 1.57, 1.32 |
| grasp（笛卡尔） | `20261022_grasp_partial/grasp_cartesian_plan.json` | 26 | 2.45 s | 0.76, -2.10, -1.87, -0.74, 1.58, 1.33 |
| 撤离 | `20261021_retreat_from_grasp/retreat_plan.json` | 29 | 2.77 s | 0.87, -2.19, -1.69, -0.83, 1.58, 1.69 |
| 搬运 | `20261022_transit_selection/transit_from_grasp.json` | 763 | 76.14 s | 0.58, -1.91, -1.38, -1.41, 1.58, 1.12 |
| pre_insert | `20261022_transit_selection/preinsert_from_transit.json` | 168 | 16.69 s | 0.15, -1.86, -1.46, -1.19, 1.54, 1.75 |
| 接近（分段） | `20261021_approach_chunked_success/chunk1..4.json` | 20×4 | ~1.9 s | 逐段 |

**共同点**：这些轨迹的 `wrist_2` 全都在 **1.53~1.62 rad（≈90°）**，真机长期工作在球腕奇异位形上——"奇异导致跟踪失败"的说法因此站不住脚，真正的问题是物理模型（见 `simulation.md` §3.4）。

### 2.3 用法与判定

```bash
python3 scripts/replay_real_run.py --list
ROS_DOMAIN_ID=72 python3 scripts/replay_real_run.py --run 20261022_pre_grasp_exec --speed 1.5
ROS_DOMAIN_ID=72 python3 scripts/replay_real_run.py --files \
  20261022_pre_grasp_exec/pre_grasp_plan.json \
  20261022_grasp_joint_partial/grasp_joint_plan.json \
  20261021_retreat_from_grasp/retreat_plan.json \
  20261022_transit_selection/transit_from_grasp.json \
  20261022_transit_selection/preinsert_from_transit.json
```

判定标准：**每条都 `ALL_ASSEMBLY_CHUNKS_OK`、末端误差 ≤0.05 rad、RTF 不塌**。任何一条在仿真里过不去，都是仿真模型的问题（而不是任务定义的问题）。

> 来源：归档原件 real_run_reference.md

## 3. 开工就绪检查

### 3.1 启动侧默认值

`real_base.launch.py` 默认 `launch_driver:=false`、`robot_ip:=` 空、`activate_joint_controller:=false`、`execute:=false`，不启动 active-perception 节点；`real_preflight` 没有 `ActionClient` 或轨迹下发路径，`execute=true` 时返回 `REAL_EXECUTION_REFUSED`。真机执行器只在 `execute=true`、确认放行、`r4_authorized=true`、operator 与 approval ID 非空、两个 MoveIt scaling 均 >0 且 **≤0.10** 时才接受任务，并在下发前用 MoveIt 重新规划。

### 3.2 每次上真机前的五项检查

| # | 检查 | 合格判据 |
|---|---|---|
| 1 | 四项门 | `safety_mode=1`（NORMAL）、`robot_mode=7`（RUNNING）、`arm_controller active`、`control_health.py` 的 sockets/controller/joint_states 三项全 True |
| 2 | 反控通道 | `ss -tn \| grep -E ":5000[134] "` → **3 条 ESTAB**（50001/50003/50004） |
| 3 | 夹爪 | `/cs625/gripper/status` 的 `position_mm` 合理（0 = 闭合空载，**≥40 = 张紧夹持**）；电流回传不可信，抓持判定只看 `position_mm` |
| 4 | 视觉服务 | `RVS_AI_Center` 在跑（**管理员方式**启动），RVS 界面无「TCP 等待连接超时」，端口 2024 OPEN |
| 5 | 相机归属 | RVS 与 ROS 相机驱动**互斥**，不能同时占用相机（二选一） |

```bash
source scripts/source_dev_env.sh --full
ros2 topic echo /io_and_status_controller/safety_mode --once   # 1 = NORMAL
ros2 topic echo /io_and_status_controller/robot_mode --once    # 7 = RUNNING
ros2 control list_controllers --controller-manager /eli_ros2_control_node   # arm_controller active
python3 scripts/control_health.py                              # exit 0 = 三项全 True
ss -tn | grep -E ":5000[134] "                                 # 3 条 ESTAB
tasklist.exe | grep -i "RVS_AI_Center"                         # Windows 侧
```

注意：`safety_mode` / `robot_mode` 是 latched（TRANSIENT_LOCAL）、只在变化时发布，volatile 订阅会永远读不到当前值；**驱动"半死"时会继续发布旧的 safety_mode**，所以四项门全绿不等于连接活着，必须同时看反控 3 条 ESTAB 与 `control_health.py`。四项门全绿、反控也在，但机械臂接受目标后 **10 s 不动** → `NO_MOTION` 多为机器人侧 `external_control` 程序失活，唯一解药是重启驱动栈（`scripts/recover_driver_stack.sh`），重发脚本与 dashboard play 无效。

归档原件的 R0–R4 分级（安全默认值 → 驱动连接 → 相机/TF → plan-only → 低速确认运动）与上面的实际流程不冲突，但已被后者取代；R4 的"现场复核 + 显式确认"仍是硬要求。

> 来源：归档原件 real_hardware_readiness.md（命令与判据来自本手册第 25–28 条及通用开工清单）

## 4. 网络与主机侧排障

工位网络（2026-11-05/07 实测）：机器人 **192.168.1.200**（示教器静态 IP），主机 **192.168.1.100**（Windows 以太网 + WSL 镜像网络）。以下数值只作现场事实记录，代码里仍不得硬编码 IP（实际值放 `~/.cs625_local.env`）。

| 现象 | 真因 | 处置 |
|---|---|---|
| 反控端口为 0、示教器显示"未连接到服务端" | 机器人网口 **76% 丢包**（50 个 ping 只回 12 个）；同网段 ping 相机 0% 丢包 | **换交换机端口** → 0% 丢包 |
| 丢包已降到 0%，反控端口仍为 0 | 驱动与机器人的**主端口连接已在丢包期间死掉**，网络恢复不会自动恢复 | SIGINT 停 launch 进程组 → 重新 launch（或 `scripts/recover_driver_stack.sh`），端口随即恢复 3 条 |
| ping 正常、我们侧端口在监听，但 `ss` 里连 SYN_RECV 都没有 | Windows 以太网被判为**"公用网络"**，防火墙丢弃 50001/50003/50004 入站 | 改专用网络或放行这三个端口 |
| 视觉与 ROS 相机争用 | RVS 与 ROS 相机驱动**互斥** | 确认相机归属，二者只留一个 |

排障顺序（别从最上层猜）：

```text
① ping -c 50 -i 0.2 192.168.1.200    # 丢包率是第一判据
② ping 相机做同网段对照              # 立刻区分"网络"与"对端"
③ 看驱动日志有无 PrimaryPort ... End of file 刷屏
④ 以上都正常，才查"程序是否在跑"（RVS_AI_Center / external_control）
```

Windows 主机侧（管理员 PowerShell）：

```powershell
Set-NetConnectionProfile -InterfaceAlias "以太网" -NetworkCategory Private
New-NetFirewallRule -DisplayName "CS625 reverse control" -Direction Inbound -Protocol TCP `
  -LocalPort 50001,50003,50004 -Action Allow
```

补充：反控通道由**机器人主动拨号到本机**，本侧只能监听与等待；端口为 0 时不要反复重启 RVS 或相机，按上表顺序查。WSL 用镜像网络时，Windows 以太网的网络类别与防火墙规则同样决定 50001/50003/50004 能否入站。

> 来源：本节为新增排障章节（2026-11-05/07 现场记录）；归档原件 real_hardware_readiness.md 不含网络内容

## 5. 未验证项

| # | 项目 | 状态 | 说明 |
|---|---|---|---|
| 1 | 取出流程的真机关节轨迹 | 未录到 | 10-22/10-23 只推进到 `d1 → d2c → d3(抓取 50mm) → d4(转倾斜) → d5 抽出 ~130 mm`，"放回工件桌"是所有者手动完成；取出段只能按 `config/unload_real.yaml` 的固定位姿链重新规划，不能原样回放 |
| 2 | 手眼链口径（`T_tcp_cam` vs `T_flange_cam`） | 未验证 | 两条链的相机位置差约 0.16 m，需真机一次性验明（拿真实 S0 图把槽心投影，落在槽口中心的链才对），见 `simulation.md` §4.2 |
| 3 | 夹爪 `position_mm` ↔ 物理跨距 | 未标定 | 没有 0/10/20/30/40/50 mm 六点卡尺表，所以"position_mm 合理"只能定性判断 |
| 4 | 夹持力与闭合/张开速度 | 未测 | 0.1 A 限流只知电流、不知 N；速度未测（仿真侧 `effort=80`、`velocity=0.02` 是假定） |
| 5 | 两指是否机械强制同步 | 未单独验证 | 真机为单驱动、机械同步，但未单独取证 |
| 6 | 坐标约定与工作空间边界 | 两份原件未记录 | `base_link` 轴向、角度正方向、手眼、TCP、工作空间边界均未写入两份原件；可用的替代记录见 §1 |

> 来源：归档原件 real_run_reference.md、real_hardware_readiness.md
