# 仿真环境（从真机实测生成）

> 2026-10-24 起，仿真不再是"另画一个世界"，而是**从真机实测数据生成**：世界几何来自 `model.json` 与 `common.yaml`，旧的手工世界全部归档、只读。本文合并归档原件 `simulation.md`（总述）、`sim_scene_from_real_runs.md`（用真机记录定义场景）、`sim_fidelity.md`（保真度实测）。

## 1. 定位与原则

旧仿真世界（`cs625_insertion_scene.sdf`）与真机根本不是同一个场景：

| 项目 | 真机实测 | 旧手搭世界 |
|---|---|---|
| 工件 | (0.503, 0.635)，桌面 z=−0.091 | (−0.510, −0.810)，**没有桌面模型** |
| 插槽 | (1.242, 0.182)，夹具底面 z=−0.041 | (−0.850, 0.490)，悬空 z=0.273 |
| 方位差 | — | 工件差 ~174°、插槽差 ~142°（几乎在对面） |

在那种世界里"规划通过"对真机没有意义。四条原则：① **几何由实测生成**（桌子高度/范围、夹具位姿来自测量，不靠猜、不手画）；② **工件位姿允许变化**（任务变量，每次任务重新测），其余必须对齐真机；③ 生成物（`cs625_real_cell.sdf`、`cs625_real_cell_scene.yaml`）由生成器拥有，**禁止手改**，要改就改生成器并重新生成；④ 旧世界只读保留在 `~/cs625_legacy_sim_snapshot_20261024/`（含 SHA256SUMS）与 `docs/archive/2026-10-24-pre-six-modes/`，新代码不得 launch 或 import。

> 来源：归档原件 simulation.md

## 2. 场景生成流程

### 2.1 数据流（唯一入口）

```text
runtime/environment/<ts>/model.json      两张桌子的 z 与 xy 范围（点云拟合）
config/tasks/common.yaml                 夹具位姿 slot_origin(+q)、工件位姿、手眼、内参
        │
        ▼  scripts/build_sim_world.py
src/cs625_simulation/worlds/cs625_real_cell.sdf        地面 + 两张实测桌面 + 夹具 + 工件
config/cs625_real_cell_scene.yaml                      生成的位姿（勿手改）
        │
        ▼  scripts/apply_real_cell_scene.py
MoveIt 规划场景碰撞体（默认只发夹具；--include-tables / --include-module 按需）
```

重新生成（真机换桌/挪夹具之后必须重跑）：

```bash
source scripts/source_dev_env.sh --full
python3 scripts/model_environment.py --dataset ~/cs625_yolo_seg/datasets/2026-10-25_raw --out runtime/environment
python3 scripts/build_sim_world.py --out-world src/cs625_simulation/worlds/cs625_real_cell.sdf --out-config config/cs625_real_cell_scene.yaml --report /tmp/sim_fit_report.json --figure /tmp/sim_world_topview.png
```

生成器打印拟合表：每个模型的**包围盒中心**对实测参考点的偏差，偏差 >20 mm 会显式警告。

### 2.2 必须由实测固定的量

| 量 | 值 | 备注 |
|---|---|---|
| 插槽绝对位姿 | `slot_origin = 1.241629, 0.181709, 0.443245`，quat `(-0.001629, 0.004253, 0.003384, 0.999984)` | 模型原点 = `slot_origin`（RVS 标定语义，不是包围盒中心） |
| 工件桌顶面 z | −0.0910 m（点云拟合） | 碰撞体用体素 `--z-min -0.13 --z-max -0.07` |
| 插槽桌顶面 z | −0.0407 m（点云拟合） | 同上 |
| S0 插槽观测基准位 | 1.044, −0.011, 0.726 | 采样基准 |
| S2 工件观测位 | 0.615353, 0.354540, 0.574506 | 采样基准 |
| S3 工件抓取位 | 0.652989, 0.703460, 0.355241 | 采样基准 |

每次任务重新测的只有**工件初位姿**（每抓一次都不一样），其余是恒定量。**坐标口径（10-20 日志的第一条经验）**：法兰 → 工具 TCP 为 (-46, 0, 353) mm（师兄工具实测）；工件模型原点 → 几何中心偏移 (10.5, -215, -105.2) mm（box.ply 尺寸 131 x 430 x 222 mm；T_b_tcp 平移 (-4, -215, -411) mm → T_center_tcp ≈ (-14.5, 0, -305.8) mm）。这两条分别造成过 **353 mm** 和 **约 16 cm** 的系统偏差，且两次都被先误判成"视觉算错/手眼标定错"。URDF 现状（2026-10-24 起）：工具换成 夹持机构0920.STL（法兰系导出，X ±147.285、Z -7.0..+207.0 mm，flange→my_end_effector_link 零偏移），但 `config/handeye/tcp_to_camera.json` 仍保存 T_flange_tcp = (-0.046, 0, 0.353)，而 URDF 里 camera_color_optical 距 eef 只有 (0.0970, 0.0422, 0.0975) m（约 0.14 m），与 T_tcp_cam 平移 0.296 m 相差约 0.16 m。两链数值对比（基准位姿 1.044,-0.011,0.726、四元数 [-0.059, 0.938, 0.060, 0.336]、槽心 1.241629,0.181709,0.443245）：`T_base_tcp @ T_tcp_cam` → 相机 0.769, -0.018, 0.834，到槽心 0.645 m、光轴夹角 14.9°；`T_base_tcp @ T_flange_cam` → 1.024, 0.039, 0.592，到槽心 0.300 m、25.7°。这条口径已于 **2026-11-10 验明**：正确链路是 `T_base_tcp @ T_tcp_cam`，两姿态一致性 0.19 mm / 0.71 mm（误差上界 ≈2.7 mm）；"0.16 m" 是法兰系安装位与 TCP 系 `T_tcp_cam` 的记账差，不是标定错（见 `pose_conventions.md` §4.1、`agent_memory.md` §131）。

### 2.3 中转点与插槽位姿链（装入/取出共用）

```text
transit       = 0.95, 0.45, 0.55            ← 现在生效的值（12 个候选点里选出）
transit 朝向  = [-0.4835, 0.8753, 0.0075, 0.0018]   ← 沿用抓取朝向
原始定义      = 0.95, 0.45, 0.35  → 带工件规划 code 99999（无解）；抬高到 0.55 → 212 点、真机 4 段成功
pre_insert    = 1.0996, -0.0125, 0.3773     ← 沿倾斜轴回退 350 mm
entrance_q    = [-0.004, 0.997, -0.001, 0.083]   ← -10° 倾角（进/出槽用）
insert_tcp    = 1.1574, -0.0132, 0.0321     ← 坐入位（= 取出时的抓取位）
insert_q      = [-0.003, 1.000, -0.002, -0.004]  ← 转正后
staging       = 1.0748, -0.0122, 0.5252     ← 插槽外侧安全接近点
baseline      = 1.044, -0.011, 0.740        ← 插槽观测基准位
```

约束（12 个候选点的选出条件）：工件桌与插槽桌之间、高于桌面 ≥0.30 m、IK 可达、无碰撞、能到 pre_insert；两套流程共用（`config/pick_place_real.yaml` 与 `config/unload_real.yaml`）。

### 2.4 碰撞处理（真机试过五条路，结论明确，仿真照抄）

```text
1. attach + touch_links（师兄做法）        → 可用（位姿查询显示 identity 但实际有效）
2. 手工下发 ACM                            → 破坏 SRDF 默认豁免，不用
3. 世界系实心 mesh 跟随 TCP                → 与腕部自碰撞
4. 点云体素模型                            → 几何真实，仍可能自碰撞
5. 体素 + 挖空夹爪通道                     → 能过但模型失真，**禁止使用**
正解：师兄 attach 节点 + 插入段移除插槽碰撞体
```

- 真机做插入时**插槽碰撞体从规划场景里移除**（`slot collision removed during insertion`），所以 `NO_COLLISION_CHECK_PHASES` 不是取巧，是照抄真机做法；工件抓取后**以 attach 跟随工具**，不留在场景里当碰撞体；
- MoveIt **不检查世界↔世界**，"把工件当世界盒子校验"永远无碰撞——错误方法，校验一律用 `check_state_validity`；
- 真实教训：接近插槽用"简化碰撞网格（无扶手）"导致余量不足、差点碰撞 → 碰撞体必须**带扶手**。

### 2.5 真机速度与分段（比当前配置更快）

| 场景 | 真机实测 |
|---|---|
| 装入（非装配段） | planning 0.01，time_scale 2.0–3.0，每段 40–50 mm，停 1 s |
| 抽出（装配段） | planning 0.01，time_scale 5.0，每段 **10 mm**，停 **1.5 s** |
| 插入/抽出通用基准 | planning 0.005–0.01、time_scale 3.0–5.0、每段 10–20 mm、停 1.0–1.5 s |
| 体感 | 快慢主要取决于**停顿次数**，不只是 mm/s |

当前紧间隙段固定 0.005 m/s 偏保守但方向正确；**分段长度与停顿次数**应与上表对齐。

### 2.6 从真机复盘搬来的其它实测约束

```text
草稿桌面 bbox（曾进碰撞场景）：workpiece_table center 0.60,0.925,-0.15 size 0.90,0.75,0.10
                              slot_table      center 1.175,0.225,-0.11 size 0.55,0.45,0.10
去掉两张草稿桌子后 pre-grasp→grasp 规划 fraction 从 0.72 变成 1.0 → draft table collision boxes are the blocker
  （现在用点云拟合值，但仍须按实测校准；绿色是碰撞体、视觉 GLB 才有完整扶手 = 当时急停的原因）
环境扫描：RVS E1 逐视角 PCD → 7 个视角合并（1 cm 降采样，29,484 点）→ 体素化桌面碰撞体；未做自点过滤、覆盖率未验收
长轨迹：120 mm 拆 2 段（每段 1.9 s）feedback 误差 0；12.4 s 整段出现执行层超调
关节顺序：不能按数组下标当 standard 顺序；IK seed 曾因此整段错位，必须按 name 映射
夹爪硬件：HITBOT EFG80，COM9，115200，从机 1；Modbus 位置/速度/电流 寄存器 2/4/6，状态 64/65/66；
          Windows TCP 桥 127.0.0.1:7500，ROS 服务 /cs625/gripper/{status,open,close}；HITBOT Studio 与桥不能同占 COM9
模块质量 19.0 kg；插槽架 12.965 kg；楔形槽倾角 5.004° / 12.62°；精密配合 0.04 mm（insert_seated 曾卡死）
工件顶面 ≈0.30 m、几何中心 ≈0.199 m（以机械臂台面为基准）
S0 插槽精拍摄位 648.004, 855.880, 70.552 (mm)  rpy 96.156, 178.308, 89.594
S2 工件精拍摄位 717.457, 575.399, 577.137 (mm) rpy 13.004, 124.455, 63.714
RVS T3 要【法兰】值，发 TCP 值会整差 353 mm（已修：state_to_flange_values）
MoveIt plan_to_pose 用 OMPL 只约束终点 → 会先越过再回伸（超调）
MoveIt execute_last_plan 返回 false ≠ 运动失败（等待 6.4 s < 控制器 12.9 s）
```

### 2.7 生成后起仿真

```bash
source scripts/source_dev_env.sh --full
ros2 launch cs625_bringup sim_cell.launch.py            # 默认带 RViz
ros2 launch cs625_bringup sim_cell.launch.py headless:=true launch_rviz:=false
```

它启动 Gazebo（新世界）+ ros2_control + MoveIt + RViz，**不自动跑任务**。控制器在负载高时可能 60–90 s 才齐全，等 `/joint_trajectory_controller/follow_joint_trajectory` 出现再跑。`--world sim` 只把控制动作名换成 `/joint_trajectory_controller/follow_joint_trajectory`（`config/tasks/world_sim.yaml`），位姿/阶段/速度与真机完全相同——这就是"1:1"。

**install 树里的资产会过期**：新增 `assets/cs625_real_cell/slot_fixture` 后若没重装，Gazebo 报 `model://cs625_real_cell/slot_fixture` 找不到、**gz server 直接退出**（看起来像世界文件坏了）。改了 assets 或 world 之后必须重装：

```bash
colcon build --symlink-install --packages-select cs625_simulation cs625_bringup cs625_ap_description
ls install/cs625_simulation/share/cs625_simulation/assets/cs625_real_cell/slot_fixture/   # 确认资产已在 install 中
```

### 2.8 五条必须搬进仿真的铁律

```text
1. 释放后：先初始化手指 → 纯平移沿插入轴离开 ≥150 mm → 才允许旋转（曾槽内旋转导致急停）
2. 中止后只允许重新规划，禁止重放旧轨迹；规划请求不要填 start_state
3. 执行器首点保护：与当前位置偏差 >0.03 rad 拒绝执行
4. 带工件进出插槽一律走 staging 高位，不允许中途点直达
5. 释放夹爪必须当次授权；单个动作 ≤60 s，60 s 无位移立即排查
```

> 来源：归档原件 sim_scene_from_real_runs.md（起仿真/资产部分来自 simulation.md）

## 3. 保真度实测

> 目的：仿真是为了**在真机之前提前暴露阻碍**，每一项都必须是"能测的数字"，不是"看起来像"。只写实测对比；没有数据的一律标 `UNKNOWN / 未验证`，不猜。最近更新：2026-10-24 夜。

### 3.1 结论摘要

| 对象 | 真机 vs 仿真 | 结论 |
|---|---|---|
| 两张桌面 | 顶面 z 差值 **0.00 mm**（由实测点云生成） | PASS |
| 插槽夹具 | 碰撞网格与相机实测点云：中位 **4.95 mm**、p90 17.2 mm、73.6% 表面在 10 mm 内 | PASS（≈5 mm 级） |
| 工件 | 网格 vs 点云中位 12.45 mm（点云含桌面背景，未严格配准） | PARTIAL |
| 相机内参 | 1280×960，fx 931.47 / fy 931.60 / cx 648.70 / cy 449.32；仿真实测与真机逐个差 ≤0.005 px（<0.001%） | PASS |
| 相机帧率 | 真机 0.79 fps；配置 1 Hz，空闲 1.00–1.02 s，被抢占时 70 s 平均 0.28 Hz | PARTIAL |
| 相机安装位 | `camera_link`=深度眼，`camera_color_xyz`=彩色眼 24.4 mm 基线；渲染在彩色眼 | PARTIAL |
| 夹爪行程/几何 | **夹爪开度 50 mm（硬件事实）**；跨距 204.00→246.57 mm（比值 0.8514）；URDF 建模的是**单侧垫块位移 21.285 mm** | PARTIAL |
| 动力学/接触/力 | 无 F/T、无摩擦模型、无真实伺服 | NOT MODELED |

**一句话**：几何（桌子、夹具、工件、相机朝向）已用实测对齐到"厘米以内、夹具 5 mm"，但**动力学、夹爪、力/接触、相机运行时行为**尚未对齐；仿真能预测"流程与几何上的阻碍"，不能预测"力学上的阻碍"。

### 3.2 怎么测的（可复现）

```bash
source scripts/source_dev_env.sh --full
python3 scripts/fit_fixture_to_cloud.py --json /tmp/fixture_fit.json   # 夹具配准
python3 scripts/build_sim_world.py --out-world src/cs625_simulation/worlds/cs625_real_cell.sdf --out-config config/cs625_real_cell_scene.yaml   # 桌面
python3 scripts/apply_real_cell_scene.py                               # 规划场景（默认只发夹具）
```

夹具基线：中位 4.95 mm、p90 17.16 mm、<5 mm 50.3%、<10 mm 73.6%、<20 mm 93.3% → PASS。ICP 会沿长轴滑移 104 mm（近对称楔形），脚本显式提示"对称滑移、忽略"，不作为修正。

**夹具为什么可信（也说明先前的误判）**：早前用**包围盒尺寸**比较 RVS 网格与单视角重建点云，结论是"差 455 mm，不是同一个东西"，那是错的——单视角点云有遮挡，包围盒天然偏小。改用"模型原点 = `slot_origin`"+"该位置下网格表面被实测点云覆盖的比例"判定：10 mm 内 73.6%、20 mm 内 93.3%。旧仿真夹具（`cs625_task/slot_fixture`）是另一套模型：槽宽 447.5 mm vs 真机 341.6 mm、插入路径只有 2/9 层贯通，**已弃用但保留**。

### 3.3 最重要的限制：槽内紧间隙阶段关闭碰撞检查

```text
TIGHT_CLEARANCE_PHASES       insert / straighten / d2c / d4 / d5_extract  → 速度 ≤0.005 m/s
NO_COLLISION_CHECK_PHASES    = 上面这些 + d2b_entrance_tilted             → 规划不做碰撞检查
```

原因：真机插槽单边间隙约 **1 mm**，比碰撞网格误差还小。这些阶段若照常做碰撞检查，真机每天都能完成的动作会**全部被判成碰撞**。因此 **"六模式 plan-only 全过"只证明运动学可达，不证明夹具不挡路。** 需要严格碰撞验证时用 `scripts/verify_fixture_clearance.py`（`avoid_collisions=True` + `check_state_validity` 接触点）。

2026-10-24 夜在隔离域（`ROS_DOMAIN_ID=65`、应用描述、夹具碰撞体已发布）跑 5 个插入相关阶段：`insert`、`straighten`、`d2b_entrance_tilted`、`d2c_straighten`、`d4_tilt_for_removal` 的 **fraction 全部 0.000**，contact 全部是 `real_cell_slot_fixture <-> my_end_effector_link`。

**5 个阶段、5 对接触，全部是 `real_cell_slot_fixture ↔ my_end_effector_link`；没有一次是机械臂连杆（base_link / shoulder_* / elbow / wrist_*）碰到夹具。** 结论：机械臂本体接近路径干净；被判碰撞的全部是**工具网格进入插槽**（真机单边间隙 1 mm，本来就靠槽壁导向、物理上接触），FCL 检查这个区域永远报碰撞、信息量为零。所以 `NO_COLLISION_CHECK_PHASES` 是有实验依据的正确设计。前提：夹具几何来自**单视角扫描**（中位 4.95 mm），工具网格来自 CAD；"工具与槽壁在 5 mm 级内是否真的相交"，仿真给不出可信答案，只有真机能给。接触点位置（供以后核对）：低处 `z≈0.102 m`、高处 `z≈0.365 m`，都在槽口/槽道范围内。

### 3.4 工件碰撞体：从"实心块"改成"扶手凹槽"（2026-10-25 修复）

**现象**：仿真里 `pre_grasp` 起所有执行段 `GOAL_TOLERANCE_VIOLATED`，终端误差集中在单个腕关节。**根因**：`shielding_module/model.sdf` 碰撞体是实心包络盒 `0.4546 x 0.13099 x 0.4650 m`，而真机是**两条支撑臂插进两根扶手之间 233 mm 宽的凹槽、再向外撑**。盒子把凹槽填死：

```text
grasp TCP 在零件系 = (-0.3235, 0.0829, 0.1793) m，离扶手凹槽底面 26.8 mm → 落在实心盒内部
实测后果：肩/腕力矩顶到 URDF 上限（730 / 100 N·m），RTF 从 1.06 掉到 0.089~0.239，
          wrist_1 传递力矩甚至到 3985 N·m（限值 100），控制器按仿真时间耗尽 goal_time 才 abort
```

**修复**（只改这一个文件）：拆成 `body` box `0.2258 x 0.1310 x 0.4650` + `rail` box `0.0050 x 0.1310 x 0.4650`，中间留 **223.8 mm** 自由跨度（实测 233 mm，差值是壁厚余量），凹槽沿 part +y 贯通；接触传感器分别挂到 `body`（原 topic）与新增 `rail`（`/cs625_task/contacts/module_rail`）。

**验证**（真机录制轨迹原样回放）：

```text
pre_grasp   11.36 s  status=4 error=0  终端误差 0.000 rad  RTF=1.000
transit     76.14 s  status=4 error=0  终端误差 0.000 rad  RTF=1.110
            （wrist_2 从 0.366 扫到 1.584，穿过 pi/2 奇异，六关节 path ratio 全 1.00、no stall）
pre_insert  16.69 s  status=4 error=0  终端误差 0.000 rad  RTF=1.000
MoveIt check_state_validity：46 个状态 0 invalid（改前从 t=8.4 s 起报工具撞工件）
力矩：改前 730/730/353/100/100/100 N·m → 改后全 0
```

**两条被否掉的方案**（别再试）：1）**`<mesh>` 用 CAD 真实几何** —— dartsim **不支持 SDF mesh 碰撞**（`Mesh construction from an SDF has not been implemented yet for dartsim`）：碰撞体会被**静默建成空**，轨迹"通过"但所有关节 effort 恒为 0，等于把工件删了；2）**给凹槽加底板** —— 工具本体宽 294.6 mm、跨越 part ±y 面向外扫，加底板后 6/23 个状态撞 `groove_floor`；去掉底板后 0/46。抓取只撑扶手内侧，底板只增加假碰撞。

### 3.5 由 dartsim 限制暴露的既有缺陷：Gazebo 里机械臂和夹具没有碰撞体

URDF 里 9 个 link 的 mesh 碰撞全部没被创建（`base_link_inertia`、`shoulder_link`、`upperarm_link`、`forearm_link`、`wrist_1/2/3_link`、`my_end_effector` 的 collision）；`slot_fixture` 与两张桌子也是 mesh → 同样为空。结论：

> **现在 Gazebo 里的碰撞只对 box/cylinder 这类 primitive 有效**（例如夹爪手指）。机械臂本体、夹爪本体、夹具、桌子在物理引擎里都没有碰撞体。因此"仿真验证间隙/碰撞安全"目前是失真的，必须单开一轮用 box/cylinder 近似补齐，并以"真机轨迹回放零误差"作为回归判据（补齐会改变现有已通过的链，不能顺手做）。

### 3.6 相机：已对齐的与未验证的

| 项目 | 真机 | 仿真（当前代码） | 说明 |
|---|---|---|---|
| 分辨率与水平 FOV | 1280×960；68.98°（2·atan(640/931.47)） | 1280×960；`camera_horizontal_fov_deg=68.98` → 1.20393 rad | 由真机内参反算，不用 vendor 的 60°；K 由 `<lens><intrinsics>` 直接给 |
| 帧率 | 0.79 fps（深度流瓶颈） | `camera_update_rate=1.0` | 用 1 Hz 复现"观测要等新帧" |
| 深度噪声 | 金属件 3~4 mm 散布 + 约 25% 空洞 | Gaussian σ=3 mm；**空洞不模拟** | 空洞是渲染限制，不伪造 |
| 相机位姿 | `T_tcp_cam`（彩色光心，距 TCP 0.2958 m） | `camera_link`=深度眼，彩色眼由 `camera_color_xyz` 偏置 24.4 mm；**渲染已移到彩色眼** | 见下 |

**为什么渲染要放到彩色眼**：应用用 `T_tcp_cam` 反投影彩色像素，而真机深度是配准到彩色的（`depth_registration_enable:=true`）。Gazebo 的 rgbd_camera 只能在一个位姿渲染两路，放彩色眼才与真机发布的图像一致；代价是仿真深度也来自彩色眼（真机是两路独立后配准），此项标 PARTIAL。

**运行时复核（2026-10-24 22:05–22:15，`ROS_DOMAIN_ID=63` 隔离，未与真机相机抢 `/camera/*`）**：

```bash
ROS_DOMAIN_ID=63 CS625_GZ_PARTITION=cs625_cam_align ros2 launch cs625_bringup sim_cell.launch.py headless:=true launch_rviz:=false
ros2 topic echo /camera/color/camera_info --once      # 对照 fx/fy/cx/cy
ros2 topic hz /camera/color/image_raw --window 10     # 帧率
```

| 项目 | 真机（A 级） | 仿真实测 | 差 | 结论 |
|---|---|---|---|---|
| 分辨率 | 1280×960 | 1280×960 | 0 | PASS |
| fx | 931.4731 | 931.4700 | −0.003 px (−0.0003%) | PASS |
| fy | 931.5953 | 931.6000 | +0.005 px (+0.0005%) | PASS |
| cx | 648.6953 | 648.7000 | +0.005 px (+0.0007%) | PASS |
| cy | 449.3193 | 449.3200 | +0.001 px (+0.0002%) | PASS |
| 水平 / 垂直 FOV | 68.98° / 54.52° | 68.98° / 54.52° | 0 | PASS |
| 帧率 | 0.79 fps | 配置 1 Hz；70 s 实测 19 帧 = **0.279 Hz**，帧间隔 min/中位 **1.00 / 2.02 s** | — | PARTIAL |
| 深度噪声 σ | 3~4 mm（ICP 中位残差） | 实测 **3.01 mm**（两帧差分 ÷√2） | +0.01 mm | PASS |
| 深度有效比例 | ~75%（金属件视角） | 40.45%（59.5% 像素超出 3 m clip） | 不可直接比 | PARTIAL |
| 深度编码 | uint16 mm | 32FC1 米 | 单位不同 | FAIL（差异） |
| P 矩阵 | 未读到（驱动当晚离线） | `[931.554,0,640; 0,931.554,480]` | — | PARTIAL |

K 为什么能精确：Harmonic 的 `<camera><lens><intrinsics>` 会直接写进 `sensor_msgs/CameraInfo`；只写 `<horizontal_fov>` 时 cx/cy 会退化成图像中心 640/480（cy 差 30.7 px = 6.8%）。实测原始输出：

```text
k: [931.47, 0.0, 648.7, 0.0, 931.6, 449.32, 0.0, 0.0, 1.0]
p: [931.5540447642347, 0.0, 640.0, 0.0, 0.0, 931.5540447642345, 480.0, 0.0, 0.0, 0.0, 1.0, 0.0]
```

帧率 PARTIAL 的原因：1 Hz 是配置值，`update_rate` 确实生效（空闲时帧间隔 1.00–1.02 s）；本机同时跑 4 个 Gazebo（别的会话）+ MoveIt + llvmpipe 软件渲染（load average ≈22 / 24 核），渲染线程被抢占时出现 4–13 s 停顿，70 s 平均被拉低到 0.28 Hz。不是参数问题，稳定 1 Hz 要在空闲机器上复测。

深度一帧统计（`/camera/depth/image_raw`）：1280×960、32FC1、有效 40.45%（497023/1228800）、中位 **0.863 m**、p5/p95 0.531/2.302 m、min/max 0.4975/2.9759 m（clip 0.2–3.0 m 内）。有效比例低于真机 75% 的原因是视野里 59.5% 像素超过 3 m，不是噪声；仿真也**没有**金属反射造成的深度失效（`<noise>` 只加高斯，不产生空洞）。**单位不同**：真机 uint16 mm，仿真桥接后 32FC1 米，`sensor_adapter` 原样转发，下游按 mm 假设会差 1000 倍。

**安装位（`parent_frame → child_frame`）**：URDF 的 `tool0 → camera_link` 不是 `T_tcp_cam` 本身，因为 `tool0` 就是法兰、不是控制器 TCP：

```text
tool0 -> camera_link            (0.096754, 0.017803, 0.097328) m   = 深度眼
tool0 -> camera_color_optical   (0.096950, 0.042212, 0.097540) m   = T_flange_cam_color_optical
T_tcp_cam = inv(T_flange_tcp) @ T_tool0_cam_color                  |t| = 0.295764 m
平移差 0.000 mm，旋转差 3.8e-5 deg，与 config/handeye/tcp_to_camera.json 数值一致
```

注意：把渲染传感器挪到彩色眼时 `<pose>` **只能平移**，不能带 `color_rpy`：那是光学系旋转，直接当传感器姿态会让相机歪 90°（传感器 +X 变成 camera_link −Y）。另外，**真机相机与仿真相机默认都在 ROS_DOMAIN_ID=0**，会同时发布 `/camera/*`；验证仿真必须用独立 `ROS_DOMAIN_ID`（例如 63）并配 `GZ_PARTITION`。

### 3.7 夹爪与末端

真机是**外撑式**（两臂向外顶扶手内壁），仿真是两指沿 X 向外平移的简化板（机理对、连杆简化，PARTIAL）。

| 项目 | 真机 | 仿真（本次改后） | 结论 |
|---|---|---|---|
| 支撑臂截面 | 55 × 58 × 146 mm（CAD 量得） | `box 0.055 0.058 0.146` | PASS |
| 接触面（外侧面）跨距 | **246.57 mm**（SolidWorks 两平行面） | 关节上限 0.021285 m → **246.57 mm** | PASS（`measured_arm_span_m: 0.24657`） |
| 零预紧抓取跨距 | 244.00 mm（扶手内间距，故意不用 2.57 mm 预紧） | 关节 0.020 → **244.00 mm** | PASS |
| 收拢（张开前）跨距 | **未实测** | 关节 0 → 204.00 mm | UNKNOWN（204 = 244 − 2×20 是推导值） |
| 单指行程 | 未实测（EFG80 上位命令 0..80 mm 是**驱动器坐标**） | 0 → 21.285 mm（跨距变化 42.57 mm） | UNKNOWN（缺标定表） |
| 抓取宽度 `50.0 mm` | `grip_width_mm: 50.0` = EFG 最大张开（所有者确认） | 端点标定：50 mm → 246.57 mm 跨距；中间段按线性假设 | PARTIAL（端点） |
| 质量（夹爪本体） | **4.580 kg 实测**（夹持机构0920 STL，SolidWorks） | `my_end_effector_link` 4.580 kg（未改）+ 手指/基座 0.140 kg | PARTIAL（重复记账 3.1%） |
| 质量（相机） | 无 datasheet | 0.010 + 0.050 = 0.060 kg（ESTIMATED） | UNKNOWN |
| 质心/惯量 | 本体实测 CoM (−0.10, 14.08, 109.56) mm；手指/相机未测；本体为 SolidWorks 真张量 | 手指/基座/相机用实心长方体估算（ESTIMATED） | PARTIAL / UNKNOWN |
| 夹持力/闭合速度 | 0.1 A 限流（未标定 N）；速度未测 | `effort="80"`；`velocity="0.02" m/s`（均未验证） | UNKNOWN |
| 控制接口 | PowerShell `init/open/close/grip(width,current)` | `position` 关节 + `/gripper_controller/commands` | PARTIAL（语义不同） |
| 两指同步 | 单驱动、机械同步（**未单独验证**） | URDF `<mimic>`，gz_ros2_control 实测两侧同步 | PASS（仿真侧） |

换算关系（对这一简化板精确）：

```text
外侧面跨距 span_mm(j) = 204.0 + 2000.0 * j          (j 单位 m)
反解         j(span_mm) = (span_mm - 204.0) / 2000.0
两指内壁净开度 = span_mm - 110.0 mm                 (每臂 55 mm 厚)
```

2026-10-24 在 `sim_cell.launch.py headless:=true` 起来 90 s 后读 `/joint_states`，再依次下发 `/gripper_controller/commands`：

| 下发命令 | `gripper_right_finger_joint` | `gripper_left_finger_joint` | 换算外撑跨距 | 说明 |
|---|---:|---:|---:|---|
| 无（初始） | 0.000 m = 0.000 mm | 0.000 m = 0.000 mm | **204.000 mm** | 收拢态 |
| 0.020 m | 0.020 m = 20.000 mm | 0.020 m = 20.000 mm | **244.000 mm** | 零预紧抓取态 |
| 0.021285 m | 0.021285 m = 21.285 mm | 0.021285 m = 21.285 mm | **246.570 mm** | CAD 实测跨距（新上限） |
| 0.050 m（字面 50 mm） | 0.021285（被上限截断） | 0.021285（被上限截断） | **246.570 mm** | 50 mm 无法到达 |

TF 交叉验证（`tf2_echo gripper_right_finger_link gripper_left_finger_link`，0.021285 时）：`Translation: [-0.247, 0.000, 0.000]` → 两指原点相距 246.57 mm（tf2_echo 只打印 3 位小数）。两指同步证据（`gz_ros2_control` 日志）：物理引擎**不做** mimic 约束（`the chosen physics engine does not support mimic constraints`），但 `gz_ros2_control` 在接口层做了，所以只给右手下发时左手确实跟随（上表两列相等就是实测结果）。

**"50 mm 抓取值"的换算（所有者确认：EFG 最大张开 50 mm）**：夹持宽度 50 mm + 电流 0.1 A（50 mm 已完全顶住；52 mm 起电源 ~1 A 硬顶），init 约 10 s、必须等到 init==5 才能发下一条命令；无自锁（保持夹力=持续通电），归零有偏移（显示开度≠真实开度）。配置的 `grip_width_mm: 50.0` 进的是 `hitbot_cmd.ps1 -Cmd grip -Value 50`，即 **EFG80 驱动器坐标**。归零后有偏移误差，显示位置与实际开度不完全一致；寄存器 70 电流反馈不可信（以电源面板为准）；实测对照全是驱动器读数：`52.0→52.0 state=0`、`52.5→卡在 52.231`、`53.0→卡在 52.231`、`54.0→卡在 51.77`——没有一条伴随物理跨距测量。所以**不存在** `position_mm → span_mm` 的映射；硬当跨距代入反解会得到 `j = (50 − 204)/2000 = −0.077 m`，比 `lower = 0` 还低，**URDF 不可达**（下发 0.050 m 被截断在 0.021285）。端点齐了之后才有确定换算：

```text
span(width) = 204.00 + (246.57 - 204.00) * width / 50.0     [mm]
finger_joint(width) = (span(width) - 204.00) / 2 / 1000    [m] = 0.0004257 * width
50 mm  -> 21.285 mm/指 -> 跨距 246.57 mm
25 mm  -> 10.643 mm/指
 0 mm  ->  0      mm   -> 跨距 204.00 mm
```

实现在 `scripts/run_pick_and_place.py::finger_position_m()`（纯函数，有 6 个单测），数值唯一来源是 `config/tasks/common.yaml` 的 `gripper_max_opening_mm / gripper_span_min_mm / gripper_span_max_mm`。仿真实测（`/gripper_controller/commands` → `/joint_states`）：0/25/50 mm 分别得到 `0 / 10.64 / 21.285 mm` 的手指关节，与上式一致 → PASS。**仍然是假设的部分**：中间段按**线性**插值（内部楔形/连杆未必线性）；要变成事实需要 0/10/20/30/40/50 mm 六点卡尺实测的跨距表，在那之前这一项标 **PARTIAL（端点标定）**。同时消掉一个安全隐患：仿真运行时夹爪命令**不再走 PowerShell**（那会驱动真夹爪），而是发 `/gripper_controller/commands`；真机才走 HITBOT 桥。

**几何一致性**：手指碰撞/可视几何 = `box 0.055 × 0.058 × 0.146 m`，与 SolidWorks 量得的两条支撑臂截面一致（PASS），但只是**接触面 + 张开方向 + 截面**的承载板，**不是真实连杆**。真机工具网格 `gripper_0920.stl` 本身已包含这两条臂，所以仿真手指碰撞盒在 `my_end_effector_link` 网格内部；SRDF 因此关掉这几对 `disable_collisions`，结果是**手指与工件的接触判定完全依赖 attach/ACM，不是碰撞几何**——任何"仿真抓取成功/失败"的说法都要一起讲这一点。真机手指的接触贴片/橡胶垫/倒角没有照片与尺寸 → **未验证**。

**已改的数值（可复核，全部 ESTIMATED，不是实测）**：

```text
cs625_parallel_gripper.xacro
  gripper_right/left_finger_joint  limit upper: 0.020 -> 0.021285  (改为 CAD 实测跨距)
  gripper_base_link  mass: 0.001 -> 0.100 kg (90 × 90 × 50 mm 包络)  inertia 1e-9 -> (8.83e-5, 8.83e-5, 1.35e-4)
  each finger        mass: 0.001 -> 0.020 kg (55 × 58 × 146 mm)      inertia 1e-9 -> (4.11e-5, 4.06e-5, 1.07e-5)
cs625_camera_extension.xacro
  camera_mount_link  mass: 0.01 -> 0.010 kg (60 × 60 × 20 mm)  inertia 1e-6 -> (3.33e-6, 6.00e-6, 3.33e-6)
  camera_link        mass: 0.05 -> 0.050 kg (30 × 80 × 30 mm)  inertia 1e-5 -> (3.04e-5, 7.50e-6, 3.04e-5)
```

**为什么手指不是"看起来合理"的 1.2 kg**：整块 55×58×146 mm 实心铝在密度 2.677 g/cm³ 下是 **1.246 kg/臂**，但 `my_end_effector_link` 上的 **4.580 kg 实测质量已经包含这两条臂**；按物理估算填进去会变成"7.4 kg 的末端"（实测 4.580 kg 的 1.6 倍），对重力补偿/加减速的伤害比 1 g 占位更大。所以选择"小值记账"，**代价是重复记账 0.140 kg（实测的 3.1%）**，并在注释里写明正确做法是等 CAD 分件质量属性把 4.580 kg 拆到各 link。相机同理：真机相机是**工具网格的一部分**，0.060 kg 与 `camera_kg_placeholder: 0.06` 一致，但真实 PS800-E1 应更重 → **这份 0.06 kg 是下限，不是实测**。

**MoveIt 侧**：规划组是 `chain base_link → my_end_effector_link`，`tool0 → gripper_base_link → *_finger_link` 都是它的子链，所以手指关节**不属于** `cs625_arm`（这正是要的）。SRDF 里没有 `<end_effector>`，夹爪相关只有六条 `disable_collisions`；`touch_links` 由运行时补：六模式用 `scripts/update_grasped_box.py` 把 9 个机器人 link 的 ACM 置为允许；历史上的 P7 抓取路径用的是 `p7_grasp_center_link` 等 frame，该 frame 已于 2026-11-13 随 P7 代码归档从 URDF 移除（见 `docs/archive/2026-11-13-p7-nbv-legacy-code/`）。结论：**抓取后 attach 的"允许接触"覆盖手指，但它是脚本里的运行时行为，不是 SRDF 声明** → 换入口（纯 MoveIt 手写 `AttachedCollisionObject`）就会漏；建议在 SRDF 补 `<end_effector>`。

**TCP / 工具坐标系口径（已查明）**：

```text
工具网格 gripper_0920.stl  bbox z ∈ [-0.007, +0.207] m，宽轴 x = 294.6 mm，窄轴 y = 85 mm
URDF: flange --(xyz=0)--> tool0 --> 夹爪；flange --(−0.046,0,0.353)--> my_end_effector_link（TCP，兄弟节点）
SRDF tip = my_end_effector_link；/tcp_pose_broadcaster/pose 发布的就是它的位姿
→ 所有任务位姿（pre_grasp / insert / baseline …）都是【法兰位姿】，不存在 207 mm 换算错误
pre_insert 工具网格与夹具顶梁的接触点 (1.2494, 0.0022, 0.3653)
vendor 描述的 camera_link 占位盒（3×5×2 cm）在 insert 路径上会撞夹具 → 应用描述已删除该盒
```

### 3.8 完全没建模的部分

```text
· 动力学：连杆惯量、摩擦、接触力 —— URDF 的末端质量/质心仍是占位值
· 夹爪：真机是 HITBOT 固件（PowerShell 控制，带夹持力/电流/行程），仿真是 URDF 位置关节
· 力/力矩：仿真没有 F/T 传感器，实验摘要里 fx..tz 永远是"未采集"
· 相机：冻结/掉线、真实金属反射造成的深度失效、曝光与运动模糊
· 控制：125 Hz 环路抖动、无实时内核、保护停止与恢复行为
· 线缆拖拽、底座刚性
```

> 来源：归档原件 sim_fidelity.md（碰撞策略与位姿来自 sim_scene_from_real_runs.md）

## 4. 已知差异与未验证项

### 4.1 已知差异

| 项目 | 现状 |
|---|---|
| 夹具几何 | 已换成真机 RVS 几何（`assets/cs625_real_cell/slot_fixture`，60k 面、0.2×0.745×0.508 m），模型原点 = `slot_origin`；旧仿真夹具保留但不再使用 |
| 桌面与默认策略 | 两张实测桌面的盒子在 xy 上有 0.144 m² 重叠；桌面高度厘米级精度（帧间位姿残差 ±2~3 cm），抓取姿态距桌面只有 3 cm，把桌子当实心盒会产生"假碰撞"，所以默认只发夹具，`--include-tables` / `--include-module` 按需 |
| 工件 | 用实测 CAD 位姿；包围盒底面比工件桌顶面低 14.8 mm（模型系与 CAD 系差异），在 Gazebo 当动力学体生成需另抬 14.8 mm |
| 相机与力口径 | 真机 uint16 mm vs 仿真 32FC1 米（按 mm 假设差 1000 倍）；仿真渲染在彩色眼（真机两路独立后配准）→ PARTIAL；仿真机械臂没有 F/T 传感器，`wrench` 与 effort 列为空（实验摘要写"未采集"） |
| 夹爪与物理 | 两指直动简化板、非真实连杆，接触判定靠 attach/ACM；Gazebo 里机械臂/夹爪本体/夹具/桌子都没有 mesh 碰撞体（只有 primitive 有效） |

### 4.2 未验证 / UNKNOWN 汇总

| # | 项目 | 状态 | 缺什么才能消除 |
|---|---|---|---|
| 1 | 收拢态跨距（仿真 `lower = 0` 对应 204 mm 是否成立） | UNKNOWN | 回零后两臂外侧面卡尺实测 |
| 2 | 单指行程 / 抓取宽度中间段 | UNKNOWN / PARTIAL | EFG80 `position_mm` ↔ 外撑跨距标定表（0/10/20/30/40/50 mm 六点） |
| 3 | 夹爪分件质量、质心、惯量 | UNKNOWN | 基座 + 单臂 + 驱动电机的 CAD 质量属性 |
| 4 | 相机质量 0.060 kg（下限） | UNKNOWN | PS800-E1 与支架 datasheet |
| 5 | 夹持力与闭合/张开速度 | UNKNOWN | 测力/测速数据；URDF `effort=80`、`velocity=0.02` 未验证 |
| 6 | 真机两指是否机械强制同步 | UNKNOWN | 单电机/双电机确认（仿真 mimic 是否合理） |
| 7 | 手指接触贴片/橡胶垫/倒角 | 未验证 | 照片 + 贴片厚度/材料（是否改变有效跨距） |
| 8 | 工具与槽壁在 5 mm 级内是否真的相交；`pre_insert` 处工具网格与夹具顶梁的接触 | 未验证 | 真机实测（夹具几何来自单视角扫描）；"应用描述 + 单一份手眼"下重跑 `verify_fixture_clearance.py` |
| 9 | 动力学/接触/力/真实伺服 | NOT MODELED | 无 F/T、无摩擦模型；URDF 末端质量/质心占位 |
| 10 | 相机稳定 1 Hz、深度有效比例、P 矩阵、深度空洞、相机安装位 | PARTIAL / 未模拟 | 空闲机器复测；驱动在线时读 P；`<noise>` 只加高斯不产生空洞；rgbd 只能单一位姿渲染两路 |
| 11 | Gazebo 间隙/碰撞安全验证 | 失真 | 需单开一轮 box/cylinder 近似补齐，以真机轨迹回放零误差回归 |
| 12 | attach `touch_links` | PARTIAL | SRDF 无 `<end_effector>`，靠脚本运行时补 |
| 13 | `prepare_gazebo_model.py` 起不来；`joint_trajectory_controller` 未激活 | 未修 / 未查 | 实体用的是上次生成的 `/tmp/cs625_active_perception_gazebo_model.urdf` → **实体落后于源码**；控制器 `Failed to activate` 原因未查（可能与本机并发 3~4 个 Gazebo 有关） |
| 14 | 环境扫描没有机器人自点过滤 | 未验收 | RVS E1 7 视角合并 29,484 点，覆盖率未验收 |
| 15 | 取出流程的真机关节轨迹 | 未录到 | 只推进到 d5 抽出 ~130 mm，放回工件桌是所有者手动完成；取出段只能按固定位姿链重新规划 |
| 16 | 手眼链口径（`T_tcp_cam` vs `T_flange_cam`） | **已验明（2026-11-10）** | 正确链路 = `T_base_tcp @ T_tcp_cam`（见 `pose_conventions.md` §4.1、`agent_memory.md` §131）；两条链到槽心 0.645 m / 14.9° vs 0.300 m / 25.7° 的差异即那 0.16 m 记账差 |

> 来源：归档原件 sim_fidelity.md（第 14/15/16 条来自 sim_scene_from_real_runs.md）

## 5. 何时必须重新生成

```text
① 真机工位/夹具几何变了（换桌子、挪夹具、换插槽/扶手、扶手形状或桌面高度变化）→ 重新生成 SDF 与 scene yaml，重跑 model_environment + build_sim_world；禁止手改生成物
② model.json 超过 2 h，或 gate 拒绝（槽 >1 mm/0.5°、工件 >3 mm/1°、<20k 点）→ 拒绝运动，先重建
③ 生成器报告的包围盒中心偏差 >20 mm → 先查测量/拟合，再决定是否接受
④ 改了生成器或新增 assets → 重建并重装 cs625_simulation / cs625_bringup / cs625_ap_description
   （否则 install 树资产过期，gz server 会因 model:// 找不到而直接退出）
⑤ 任务只换工件位姿时：不必重建世界，但每次任务必须重测工件初位姿
```

> 来源：归档原件 simulation.md（阈值与失效模式来自 sim_fidelity.md、sim_scene_from_real_runs.md）
