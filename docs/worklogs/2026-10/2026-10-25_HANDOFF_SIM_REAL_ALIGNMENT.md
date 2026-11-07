# 2026-10-25 HANDOFF — 仿真与真机对齐（自包含交接）

> 给「没有任何上下文的新对话」看。读完这一份能接手；细节入口见 §12。
> 本交接写于上下文过长、需要换会话时；**所有结论都有出处或命令可复现**。

---

## 1. 一句话现状

**装入/取出/全流程六类模式的编排与配置已就绪，真机侧因机器人不可达无法实动；
仿真侧今天第一次真正跑起来并把"跑不完"的根因（工件碰撞体是实心块）修掉了，
`load/efficient` 在仿真里已能跑过观测前缀与搬运段（13/19），剩 3 段仍未通过。**

## 2. 现场状态（开工前先核这三条）

```text
真机 192.168.1.200       No route to host（整晚不可达，未做任何真机运动）
真机相机                 已停止（进程无 percipio_camera；物理供电无法软件切断，需所有者断 PoE/适配器）
仿真                     当前在 ROS_DOMAIN_ID=72（子智能体实例）；其余域请用 71/73 避免撞
本机 Windows 会话        解锁（录屏依赖它；锁屏时抓不到画面）
```

## 3. 今天完成并验证过的修复（每条都有证据）

| # | 问题 | 根因 | 验证 |
|---|---|---|---|
| 1 | 五个观测阶段全 `PLAN_FAILED`，日志 `nearest solution 2.6~3.1 rad … refusing a branch flip` | `ik()` 拿 `/joint_states` **原始数组**当 seed，却按 `JOINTS` 命名；Gazebo 按字母序发 8 个关节（含两指） | `test/test_ik_seed.py` 5 个用例；仿真里观测段全过 |
| 2 | 规划成功但 `ok=False`（`chunk N starts far from the current state`） | 执行子进程期间没人 spin，`/joint_states` 缓存停滞后 | `fresh_joints()`：等"调用之后到达"的样本 |
| 3 | 失败一律显示 `PLAN_FAILED` | `detail() if ok else 'PLAN_FAILED'` 把原因覆盖 | 直接输出 `detail()`＋执行器末尾 4 行＋终端关节误差 |
| 4 | 估算器在仿真里一直等图 | `icp_pose_once.py` 依赖真机才有的 `/tcp_pose_broadcaster/pose` | 加 TF 回退（`base_link → my_end_effector_link`） |
| 5 | 仿真启动卡在控制链检查 | 走了真机 `control_health.py`（等硬件）；改 CLI 后又超时 | 直接探测**轨迹 action server** + `/joint_states` |
| 6 | 仿真位姿校验必然失败 | 校验用真机照片训练的 YOLO；Gazebo 渲染无贴图 → `no mask` | 仿真里降级为 **advisory**（打印原因、不阻断）；真机仍阻断 |
| 7 | 仿真起始位姿是 vendor 全零，第一个观测就换 IK 分支 | 没把真机起始位姿交给 Gazebo | `cs625_bringup/config/sim_initial_positions.yaml`（= 真机实测起始位姿），作为 launch 默认 |
| 8 | 仿真运行会驱动**真夹爪** | `gripper()` 无条件走 Windows PowerShell | 按 world 分流：仿真发 `/gripper_controller/commands`，真机才走 HIBTOT |
| 9 | 相机模式钩子会在仿真运行时重启**真机相机**（真机画面混进仿真域，污染录像） | `run_task.py` 钩子没判断 world | 限定 `--world real`；另加录制器的来源自检 |
| 10 | **工件碰撞体是实心块**，夹爪进不去扶手凹槽 → 力矩顶满、RTF 崩塌、超时 | `shielding_module/model.sdf` 的 `<collision name="envelope">` 0.4546×0.1310×0.4650 实心盒 | 拆成 `body`＋`rail` 两块，中间 223.8 mm 凹槽；真机轨迹回放零误差（见 §5） |
| 11 | Qt 窗口**标题栏**中文变方框 | WSLg 的 Weston 用发行版字体画标题栏，`/mnt/wslg/distro/usr/share/fonts` 无中文字形 | 顶层标题改 ASCII，中文留在窗内；弹窗标题同改；子进程加 `PYTHONIOENCODING=utf-8` |
| 12 | 演示视频"打不开"（只有 0:00 播放器） | 用 OpenCV 写的 `mp4v`（MPEG-4 Part 2），Chromium 不支持 | 转 **VP9 WebM**；本机 OpenCV 无 H.264 编码器（`avc1` 打不开） |

**两条被否掉的方案（别再试）**：
1. **`<mesh>` 当工件碰撞体** —— dartsim 不支持 SDF mesh 碰撞（日志 `Mesh construction from an SDF has not been implemented yet for dartsim`），
   碰撞体被**静默建成空**：轨迹"通过"但 effort 恒为 0，等于删掉工件。
2. **给扶手凹槽加底板** —— 工具本体宽 294.6 mm、跨越工件 ±y 向外扫，加底板后 6/23 状态撞 `groove_floor`；去掉后 0/46。

## 4. 仍然坏着的三件事（按优先级）

1. **`load/efficient` 仿真只有 13/19**：`pre_grasp`（规划出 813 点 81 s 的长轨迹但执行失败）、
   `grasp`、`staging`（PLAN_FAILED）、`pre_insert`（leg 重试）未通过。
   注意这**不再是**"物理卡死"（工件凹槽已修）：更像是"从当前位置出发的规划绕路/耗时过长"，
   以及执行段与规划段的状态衔接。**下一步先看 `pre_grasp` 为什么规划出 813 点**（对比真机
   `20261022_pre_grasp_exec/pre_grasp_plan.json` 只有 115 点 11.36 s）。
2. **dartsim 不支持 mesh 碰撞** → 机器人 9 个 link、夹爪本体、夹具、两张桌子在 Gazebo 里
   **都没有碰撞体**，只有 box/cylinder 这类 primitive 有效。所以"仿真验证间隙/碰撞安全"目前失真。
   建议单开一轮用 box/cylinder 近似补齐，回归判据 = §5 的真机轨迹回放零误差。
3. **坐标链有一处不一致**：`config/handeye/tcp_to_camera.json` 仍存 `T_flange_tcp=(-0.046,0,0.353)`（师兄旧工具），
   而 URDF 现在挂 0920 工具（法兰系导出，尖端 +207 mm）。两条相机链在 baseline 位姿下
   到槽心距离分别是 **0.645 m** 与 **0.300 m**，光轴夹角 14.9° 与 25.7°。
   **必须用真图验明**：把槽心用两条链分别投影到一张真实 S0 图，落在槽口中心的那条才对。

## 5. 决定性证据：真机轨迹在修好的仿真里零误差回放

```text
pre_grasp   11.36 s  status=4 error=0  终端误差 0.000 rad  RTF=1.000  effort 全 0
transit     76.14 s  status=4 error=0  终端误差 0.000 rad  RTF=1.110
            wrist_2 从 0.366 扫到 1.584（穿过 π/2），六关节 path ratio 全 1.00、no stall
pre_insert  16.69 s  status=4 error=0  终端误差 0.000 rad  RTF=1.000
（改前：RTF 0.239，pan/lift 730 N·m(100%)、elbow 353、三个腕 100，写死卡住）
```

→ 结论：**只要动作是"真机做过的"，仿真现在跟得住**；"腕部奇异导致跟踪失败"是错误结论
（真机全程 wrist_2≈90°）。

## 6. 权威依据：仿真场景用真机记录来定义

**必读**：[`docs/sim_scene_from_real_runs.md`](../sim_scene_from_real_runs.md) —— 每条都带出处。
核心数字（不要再用别处的值）：

```text
中转点（装入/取出共用）  transit = 0.95, 0.45, 0.55
                         transit_q = [-0.4835, 0.8753, 0.0075, 0.0018]（沿用抓取朝向）
                         原始定义是 z=0.35，带工件时 code 99999 无解，抬到 0.55 后成功
插槽侧固定链             pre_insert 1.0996,-0.0125,0.3773（-10° 倾角）
                         entrance_q [-0.004,0.997,-0.001,0.083]
                         insert_tcp 1.1574,-0.0132,0.0321 / insert_q [-0.003,1.000,-0.002,-0.004]
                         staging 1.0748,-0.0122,0.5252 ; baseline 1.044,-0.011,0.740
观测基准位               S0 插槽 1.044,-0.011,0.726 ; S2 工件 0.615353,0.354540,0.574506
                         S3 抓取 0.652989,0.703460,0.355241
插槽几何                 slot_origin 1.241629,0.181709,0.443245
夹爪（三个数别混）       开度 50 mm（硬件）；连杆比 0.8514 → 两臂跨距 204.00→246.57 mm
                         URDF 关节 = 单侧垫块位移 0→21.285 mm
扶手凹槽                 两扶手内侧 233 mm；grasp TCP 在零件系 (-0.3235,0.0829,0.1793) m
速度基线（真机实测）     插入/抽出 planning 0.005–0.01、time_scale 3.0–5.0、每段 10–20 mm、停 1.0–1.5 s
                         装入非装配段 planning 0.01、每段 40–50 mm、停 1 s
铁律                     释放后先纯平移 ≥150 mm 再旋转；带件进出插槽必走 staging；
                         中止后只重新规划不重放旧轨迹；规划请求不要填 start_state
```

## 7. 真机轨迹资产与回放工具

```text
记录目录  ~/cs625_legacy_snapshot_20261020/runs/    43 次运行、49 条关节轨迹
索引      docs/real_run_reference.md
工具      python3 scripts/replay_real_run.py --list
          python3 scripts/replay_real_run.py --run 20261022_pre_grasp_exec --speed 1.5
          python3 scripts/replay_real_run.py --files a.json b.json ...
判据      每条 ALL_ASSEMBLY_CHUNKS_OK、终端误差 ≤0.05 rad、RTF 不塌
```

完整流程那次：`20261022_full_task_e2e/meta.json`；取出那次的进度在
`docs/worklogs/2026-10-22_HANDOFF_EVENING_UNLOAD.md`（到"工件完整抽出、移动到工件桌正上方"，
**放回工件桌是所有者手动完成**，所以**取出没有关节轨迹可回放**，只能用固定位姿链重新规划）。

## 8. 新增工具（都是今天写的）

```text
scripts/replay_real_run.py        回放真机录制轨迹（判定仿真是否跟得住）
scripts/record_sim_run.py         录制"只含仿真画面"的任务视频（来源自检 + 饱和度过滤 + 烧 SIM 标签）
scripts/record_sim_screen.sh      全场景录屏（Windows 桌面/按窗口抓帧 → ffmpeg H.264）
scripts/file_viewer.py            只读文件查看器：浏览器看视频/图片/日志（http://localhost:8090/）
scripts/fit_fixture_to_cloud.py   夹具网格 vs 相机点云配准（判据用覆盖度，不用 bbox）
scripts/verify_fixture_clearance.py 严格碰撞逐段验证（开碰撞检查 + 接触点）
```

## 9. 六类模式与配置（现状）

```bash
python3 scripts/run_task.py --scope {load|unload|full} --mode {safe|efficient} --world {real|sim} [--confirm]
```

- 只规划回归：19/19、19/19、38/38、38/38、57/57、57/57（`full` 要 `--handover auto` 才会跑完取出段）；
- 仿真执行：观测前缀 + 搬运段已过，剩 §4 的三项；
- 环境模型闸门：`runtime/environment/latest.json` 指向的模型 **超过 2 h 直接 REFUSE**。
  **离线重建即可，不需要相机**：`python3 scripts/model_environment.py`。

## 10. 需要所有者提供 / 决定

1. **机器人可用了先做**：`control_health.py` 四项 → 环境建模 → `load/safe` 预演+只规划 →
   再只做 pre_insert→insert 入口 100 mm 低速实动（验证 1 mm 间隙是否真接触）。
2. **EFG80 `position_mm` ↔ 两臂跨距 的标定表**（0/10/20/30/40/50 mm 六点，卡尺量）——
   没有它，夹爪中间段只能按线性假设。
3. **夹爪分件质量/质心**（基座/单臂/电机）、PS800-E1 + 支架重量。
4. **是否批准"给臂/夹具/桌子补 box 近似碰撞体"**（§4 第 2 条，会改变现有已通过的链，需单开一轮）。

## 11. 下一步唯一推荐任务（按顺序）

1. **查 `pre_grasp` 为什么在仿真里规划出 813 点 / 81 s**（真机同段只有 115 点 / 11.36 s）：
   大概率是起始位姿不同导致 OMPL 绕路；对照法是先 `replay_real_run.py` 把手臂摆到真机
   pre_grasp 起点，再规划同一目标。目标：`load/efficient` 仿真 19/19。
2. 19/19 后：起**带 GUI 的 Gazebo**，用 `scripts/record_sim_screen.sh` 录一段全场景视频
   （录的是整机+夹具+工件的动作，不是相机视角）。
3. 然后才是 §4 第 2 条（补碰撞体）。

## 12. 文档入口

```text
docs/sim_scene_from_real_runs.md     仿真场景的权威定义（本文 §6 的来源）
docs/real_run_reference.md           49 条真机轨迹索引 + 三条必须照抄的经验
docs/sim_fidelity.md                 仿真保真度：已对齐/未验证/未建模，含 §3.2 工件碰撞体根因
docs/simulation.md                   从实测生成世界的流程与坑（install 树过期等）
docs/file_viewer.md                  浏览器查看器用法
docs/worklogs/2026-10-25_WORKLOG_SIM_EXECUTION_FIX.md   今天的工作日志
AGENTS.md                            工程规则（必须读）
```

## Capability layer

```text
- environment and dependencies      PASS      ROS Jazzy 链、相机双模式、ffmpeg 已装
- robot model and simulation        PARTIAL   工件凹槽已修；臂/夹具/桌子在 Gazebo 里仍无碰撞体
- kinematics, control and planning  PASS      六模式只规划全过；真机轨迹回放零误差
- vision, hand-eye calibration/TF   PARTIAL   相机内参已对齐；坐标链有 353/207 mm 待验明
- active perception / NBV           NOT STARTED
- real-hardware integration/safety  BLOCKED   机器人不可达
```

---

## 13. 仿真控制器排查线（sim-control-debug）的最终交接

> 这一节是子智能体在交接时的自述，补上 §4 第 1 条的诊断细节。

### 13.1 已确认并验证

```text
改动文件：只有 src/cs625_simulation/assets/cs625_task/shielding_module/model.sdf
         （实心 envelope → body + rail 两盒，凹槽 223.8 mm、沿 part +y 贯通；
          contact sensor 改挂 body，另加 module_rail）
已入库：commit 791b767；工作区干净
回放验证：pre_grasp 11.36 s / transit 76.14 s（wrist_2 0.366→1.584 过 π/2）/ pre_insert 16.69 s
         全部 status=4 error=0、终端误差 0.000 rad、RTF 1.00/1.11/1.00、六关节 ratio 1.00 无 stall
MoveIt 复检：真机轨迹 46 个实测状态 invalid 0（改前从 t=8.4 s 起报 tool/finger vs 工件）
```

### 13.2 `load/efficient` 仿真 13/19 的失败细节

```text
pre_grasp  813 点 / 81.2 s，GOAL_TOLERANCE 0.234 / 0.197 / 0.100 rad
grasp      GOAL_TOLERANCE 0.250 rad
staging    PLAN_FAILED
pre_insert 在 leg 重试中被交接清场中断（无结论）
（修前同一批阶段误差 1.25~1.49 rad，现在小 5~6 倍，但没根治）
```

### 13.3 对 §4 第 1 条的**关键新结论**（下一会话优先看这条）

```text
813 点/81 s 本身不是异常：那是 baseline → pre_grasp 的整段关节空间规划（v=0.03）+ time_scale×1.6，
                            点数是规划器自适应采样出来的。
真正的异常是：**终点落在另一个 IK 分支**
             规划末点 pan 0.14 / wrist_3 1.70
             真机录制段末点 pan 0.718 / wrist_3 1.29
原因：MoveIt 场景里**没有工件**（真机也故意不发布，因为夹爪要接触工件），
      所以换分支不受任何约束。
残余卡滞最可能的原因：body 盒仍被 tool 扫到（盒近似表达不了真实壳体）。
```

**因此下一会话的第一件事**应为：让规划**锁在真机那条分支上**——
最省事的做法是直接用真机录制轨迹的**终点关节值**作为 joint goal（而不是 pose goal），
或者把录制轨迹的中间点作为 waypoint 约束；然后再看残余接触。

### 13.4 尚未做 / 待决定

```text
· 19/19
· dartsim mesh 碰撞补齐（臂 9 个 link + 夹爪本体 + 夹具 + 两张桌子全部无碰撞体）
· 撑开接触力（仿真夹爪是刚性 tool + 21 mm 手指行程，没有真实外撑机构，取不到）
· 坐标链 353 vs 207 mm 的验明
```

### 13.5 临时文件（都在 /tmp，可删）

```text
/tmp/wrist_repro.py                复现 + 测量工具（RTF、终端误差、逐关节偏差）
/tmp/mkchunk.py                    由真机轨迹生成 chunk
/tmp/simdbg/                       日志与 CSV，含三条通过证据：
                                   f2_real_pg.csv / f2_real_transit.csv / f2_real_preinsert.csv
/tmp/simdbg/real_*.json            真机轨迹副本
runtime/environment/20261025_070113/   离线重建模型（gate PASS，delta 0.00 mm，latest 已指向）
```
