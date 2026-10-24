# 仿真保真度报告（Simulation fidelity）

> 目的：这份仿真是为了**在真机之前提前暴露阻碍**，所以每一项都必须是"能测的数字"，
> 不是"看起来像"。本文只写**实测对比**；没有数据的一律标 `UNKNOWN / 未验证`，不猜。
> 最近更新：2026-10-24 夜。真机当晚网络不可达（`No route to host`），所以没有真机运动。

## 1. 结论摘要

| 对象 | 真机 vs 仿真 | 结论 |
|---|---|---|
| 两张桌面 | 顶面 z 差值 **0.00 mm**（由实测点云生成） | PASS |
| 插槽夹具 | 碰撞网格与相机实测点云：中位 **4.95 mm**、p90 17.2 mm、73.6% 表面在 10 mm 内 | PASS（≈5 mm 级） |
| 工件 | 网格 vs 点云中位 12.45 mm（点云含桌面背景，未严格配准） | PARTIAL |
| 相机内参 | 1280×960，fx 931.47 / fy 931.60 / cx 648.70 / cy 449.32 | 仿真实测 K 与真机逐个差 ≤0.005 px（<0.001%） | PASS（§4 运行时复核） |
| 相机帧率 | 0.79 fps（1280×960 彩色+深度） | 配置 1 Hz；空闲时间片下帧间隔 1.00–1.02 s，本机被别的会话抢占时 70 s 平均 0.28 Hz | PARTIAL（§4） |
| 相机安装位 | 用 RVS 手眼链：`camera_link` = 深度眼，`camera_color_xyz` = 彩色眼 24.4 mm 基线；传感器渲染在**彩色眼** | PARTIAL（见 §4） |
| 夹爪行程/几何 | 行程已对齐到 CAD 实测的 246.57 mm 外撑跨距，质量改为**标注估算值**（不是实测） | PARTIAL（见 §8；`position_mm`↔跨距仍缺标定） |
| 动力学/接触/力 | 无 F/T、无摩擦模型、无真实伺服 | NOT MODELED |
| 槽内 1 mm 间隙 | 规划时**关闭碰撞检查**（几何导向） | 见 §3 的重要限制 |

**一句话**：几何（桌子、夹具、工件、相机朝向）已经用实测数据对齐到"厘米以内、夹具 5 mm"，
但**动力学、夹爪、力/接触、相机运行时行为**还没有对齐；仿真能预测"流程与几何上的阻碍"，
不能预测"力学上的阻碍"。

## 2. 怎么测的（可复现）

```bash
cd ~/projects/cs625-active-perception
source scripts/source_dev_env.sh --full

# 2.1 夹具：把碰撞网格与相机实测点云做配准（判据用"标定位置下的覆盖度"，不是 ICP 最优解）
python3 scripts/fit_fixture_to_cloud.py --json /tmp/fixture_fit.json
#   基线：中位 4.95 mm、p90 17.16 mm、<5mm 50.3%、<10mm 73.6%、<20mm 93.3%   → PASS
#   ICP 会沿长轴滑移 104 mm（近对称楔形），脚本会显式提示"对称滑移、忽略"，不作为修正

# 2.2 桌面：直接用实测模型生成，差值 0.00 mm
python3 scripts/build_sim_world.py --out-world src/cs625_simulation/worlds/cs625_real_cell.sdf \
    --out-config config/cs625_real_cell_scene.yaml

# 2.3 规划场景：把真机夹具几何作为碰撞体发布进 MoveIt
python3 scripts/apply_real_cell_scene.py            # 默认只发夹具
```

### 2.1 夹具为什么可信（也说明我先前的误判）

早先我用**包围盒尺寸**比较 RVS 网格与单视角重建点云，结论是"差 455 mm，不是同一个东西"。
那是错的：单视角点云有遮挡，包围盒天然偏小。改用两件事判定：

1. **模型原点 = `slot_origin`**（RVS 标定语义，不是包围盒中心）；
2. 该位置下**网格表面被实测点云覆盖的比例**：10 mm 内 73.6%、20 mm 内 93.3%。

结论：它是真机夹具，位置也对；旧仿真夹具（`cs625_task/slot_fixture`）是另一套模型——
槽宽 447.5 mm vs 真机 341.6 mm、插入路径只有 2/9 层贯通，**已弃用但保留**。

## 3. 最重要的限制：槽内紧间隙阶段的碰撞检查是关的

`scripts/run_pick_and_place.py` 把规则分成两类：

```text
TIGHT_CLEARANCE_PHASES       insert / straighten / d2c / d4 / d5_extract  → 速度 ≤0.005 m/s
NO_COLLISION_CHECK_PHASES    = 上面这些 + d2b_entrance_tilted             → 规划不做碰撞检查
```

原因：真机插槽单边间隙约 **1 mm**，比碰撞网格误差还小。若这些阶段照常做碰撞检查，
真机每天都能完成的动作会被判成碰撞。**因此 "六模式 plan-only 全过" 只证明运动学可达，
不证明夹具不挡路。** 需要严格碰撞验证时用 `scripts/verify_fixture_clearance.py`
（`avoid_collisions=True` + `check_state_validity` 接触点）。

## 4. 相机：已对齐的与未验证的

| 项目 | 真机 | 仿真（当前代码） | 说明 |
|---|---|---|---|
| 分辨率 | 1280×960 | 1280×960 | xacro + launch 默认已改 |
| 水平 FOV | 68.98°（2·atan(640/931.47)） | `camera_horizontal_fov_deg=68.98` → 1.20393 rad | 由真机内参反算，不再用 vendor 的 60°；K 由 `<lens><intrinsics>` 直接给 |
| 帧率 | 0.79 fps（深度流是瓶颈） | `camera_update_rate=1.0` | 用 1 Hz 复现"观测要等新帧" |
| 深度噪声 | 金属件 3~4 mm 散布 + 约 25% 空洞 | Gaussian σ=3 mm；**空洞不模拟** | 空洞是渲染限制，不伪造 |
| 相机位姿 | `T_tcp_cam`（彩色光心，距 TCP 0.2958 m） | `camera_link`=深度眼（`camera_extrinsics_sim.yaml`），彩色眼由 `camera_color_xyz` 偏置 24.4 mm；**渲染传感器移到彩色眼** | 见下 |

**为什么渲染要放到彩色眼**：应用用 `T_tcp_cam` 反投影彩色像素，而真机深度是配准到彩色的
（`depth_registration_enable:=true`）。Gazebo 的 rgbd_camera 只能在一个位姿渲染两路，
放在彩色眼才与真机发布的图像一致；代价是仿真深度也来自彩色眼（真机是两路独立后配准），
这一项标 PARTIAL。

**运行时复核（2026-10-24 22:05–22:15，`ROS_DOMAIN_ID=63` 隔离，未与真机相机抢 `/camera/*`）**：

```bash
ROS_DOMAIN_ID=63 CS625_GZ_PARTITION=cs625_cam_align \
  ros2 launch cs625_bringup sim_cell.launch.py headless:=true launch_rviz:=false
# 等 90 s 后
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
| 帧率 | 0.79 fps | 配置 1 Hz；70 s 实测 19 帧 = **0.279 Hz**，帧间隔 min/中位 **1.00 / 2.02 s** | — | PARTIAL（见下） |
| 深度噪声 σ | 3~4 mm（ICP 中位残差） | 实测 **3.01 mm**（两帧差分 ÷√2） | +0.01 mm | PASS |
| 深度有效比例 | ~75%（金属件视角） | 40.45%（59.5% 像素超出 3 m clip） | 不可直接比 | PARTIAL |
| 深度编码 | uint16 mm | 32FC1 米 | 单位不同 | FAIL（差异，见下） |
| P 矩阵 | 未读到（驱动当晚离线） | `[931.554,0,640; 0,931.554,480]` | — | PARTIAL |

K 为什么能精确：Harmonic 的 `<camera><lens><intrinsics>` 会直接写进
`sensor_msgs/CameraInfo`；只写 `<horizontal_fov>` 时 cx/cy 会退化成图像中心 640/480
（cy 差 30.7 px = 6.8%）。`/camera/color/camera_info --once` 的原始输出：

```text
k: [931.47, 0.0, 648.7, 0.0, 931.6, 449.32, 0.0, 0.0, 1.0]
p: [931.5540447642347, 0.0, 640.0, 0.0, 0.0, 931.5540447642345, 480.0, 0.0, 0.0, 0.0, 1.0, 0.0]
```

帧率 PARTIAL 的原因：1 Hz 是配置值，`update_rate` 确实生效（有空闲 CPU 时帧间隔就是
1.00–1.02 s）；本机同时跑着 4 个 Gazebo（别的会话）+ MoveIt + llvmpipe 软件渲染
（load average ≈22 / 24 核），渲染线程被抢占时出现 4–13 s 停顿，70 s 平均被拉低到 0.28 Hz。
这不是参数问题，稳定 1 Hz 要在空闲机器上复测。

深度一帧统计（`/camera/depth/image_raw`）：1280×960、32FC1、有效 40.45%（497023/1228800）、
中位 **0.863 m**、p5/p95 0.531/2.302 m、min/max 0.4975/2.9759 m（clip 0.2–3.0 m 内）。
有效比例低于真机 75% 的原因不是噪声，而是视野里 59.5% 像素超过 3 m；同时仿真**没有**
金属反射造成的深度失效（`<noise>` 只加高斯，不产生空洞）。**单位不同**：真机深度是
uint16 mm，仿真桥接后是 32FC1 米，`sensor_adapter` 原样转发，下游按 mm 假设会差 1000 倍。

**安装位（`parent_frame → child_frame`）**：URDF 里的 `tool0 → camera_link` 不是
`T_tcp_cam` 本身，因为 `tool0` 就是法兰、不是控制器 TCP；两者关系已核算：

```text
tool0 -> camera_link            (0.096754, 0.017803, 0.097328) m   = 深度眼
tool0 -> camera_color_optical   (0.096950, 0.042212, 0.097540) m   = T_flange_cam_color_optical
T_tcp_cam = inv(T_flange_tcp) @ T_tool0_cam_color                  |t| = 0.295764 m
平移差 0.000 mm，旋转差 3.8e-5 deg，与 config/handeye/tcp_to_camera.json 数值一致
```

注意：把渲染传感器挪到彩色眼时 `<pose>` **只能平移**，不能带 `color_rpy`：那是光学系
旋转，直接当传感器姿态会让相机歪 90°（传感器 +X 变成 camera_link −Y）。

> 注意：**真机相机与仿真相机默认都在 ROS_DOMAIN_ID=0**，会同时发布 `/camera/*`。
> 验证仿真相机时必须用独立 `ROS_DOMAIN_ID`（例如 63）并配 `GZ_PARTITION`。

## 5. 完全没建模的部分（不要指望仿真能预测）

```text
· 动力学：连杆惯量、摩擦、接触力 —— URDF 的末端质量/质心仍是占位值（见 §6）
· 夹爪：真机是 HITBOT 固件（PowerShell 控制，带夹持力/电流/行程），仿真是 URDF 位置关节
· 力/力矩：仿真没有 F/T 传感器，实验摘要里 fx..tz 永远是"未采集"
· 相机：冻结/掉线、真实金属反射造成的深度失效、曝光与运动模糊
· 控制：125 Hz 环路抖动、无实时内核、保护停止与恢复行为
· 线缆拖拽、底座刚性
```

## 6. 需要所有者提供的数据（否则无法继续对齐）

| 缺什么 | 用在哪 | 影响 |
|---|---|---|
| 夹爪与相机支架的**真实质量与质心** | URDF `<inertial>` + 示教器 Payload | 重力补偿、加减速抖动（"不流畅感"的头号嫌疑） |
| 夹爪**行程**（mm）与手指几何 | URDF 关节限位 + 碰撞几何 | "夹住没夹住"在仿真里是否有意义 |
| 真实套筒/夹具的**接触刚度**（可选） | 接触模型 | 插入时的力学行为 |

## 7. TCP / 工具坐标系口径（已查明）

严格碰撞检查下 `pre_insert` 会撞夹具顶梁（接触点 (1.2494, 0.0022, 0.3653)），
一度怀疑是"夹爪尖 vs 法兰"差了 207 mm。离线核算后的结论：

```text
工具网格 gripper_0920.stl   bbox  z ∈ [-0.007, +0.207]  m
  → 网格原点在法兰，工具向 +z 伸出 207 mm；宽轴 x = 294.6 mm，窄轴 y = 85 mm
URDF 链路  flange --(xyz=0)--> my_end_effector_link --(xyz=0)--> tool0 --> 夹爪
  → my_end_effector_link == 法兰（不是夹爪尖）
SRDF tip   my_end_effector_link
/tcp_pose_broadcaster/pose 发布的就是 this link 的位姿
  → 所有任务位姿（pre_grasp / insert / baseline …）都是【法兰位姿】
```

**所以不存在 207 mm 的换算错误**：任务位姿、MoveIt 规划、手眼外参三者的"TCP"是同一个
（法兰）。把位姿"抬高 0.207 m"的尝试是把口径改错了，不能采纳。

剩下两件事与夹具无关、属于机器人模型：

1. **vendor 描述的 `camera_link` 占位盒**（3×5×2 cm）在 insert 路径上会撞夹具；
   应用描述 `cs625_ap_description` 已删掉这个盒子。**真机 MoveIt 必须用应用描述**，
   否则规划器会看到一只不存在的相机盒。
2. `pre_insert` 处工具网格与夹具顶梁的接触，需要在"应用描述 + 单一份手眼"下重跑
   `scripts/verify_fixture_clearance.py` 才能定性；在此之前不要下"插入不可行"的结论。

---

## 8. 夹爪与末端（2026-10-24 追加，只改描述文件，未动真机）

### 8.1 一句话

真夹爪是**外撑式撑紧机构**：两条支撑臂插进工件两扶手之间，向外顶住扶手**内壁**
（`config` 里的 `grasp.style: expanding_brace`）；HITBOT EFG80 只是它的电动驱动。
仿真是同一接触面、同一张开方向的**简化板**，不是真实连杆。本次把关节行程对齐到
CAD 实测的 **246.57 mm 外撑跨距**，把 1 g 占位质量换成**明确标注 ESTIMATED 的估算值**，
并量出了"关节值 → 外撑跨距"的换算。**仍然对不齐的是**：配置里 `grip_width_mm: 50.0`
是 HITBOT 的驱动器坐标，没有任何数据能把它换算成物理跨距，所以 50 mm 在仿真里
**不可表达**（换算出的关节值是 −0.077 m，超出下限 0）。

### 8.2 项目对照表

| 项目 | 真机 | 仿真（本次改后） | 依据 | 结论 |
|---|---|---|---|---|
| 机构类型 | 外撑式（两臂向外顶扶手内壁） | 两指沿 X 向外平移的简化板 | 工作日志 2026-09-19 §9.4；`cs625_task_scene.yaml` `grasp.style` | PARTIAL（机理对，连杆简化） |
| 支撑臂截面 | 55 × 58 × 146 mm（CAD 量得） | `box 0.055 0.058 0.146` | SolidWorks 导出，2026-09-20 commit | PASS |
| 接触面（外侧面）跨距 | **246.57 mm**（SolidWorks 两平行面） | 关节到上限 0.021285 m → **246.57 mm** | `measured_arm_span_m: 0.24657` | PASS（本次对齐） |
| 零预紧抓取跨距 | 244.00 mm（扶手内间距，故意不用 2.57 mm 预紧） | 关节 0.020 → **244.00 mm** | `handle_inner_span_m` + 主人决定 `none_zero_preload` | PASS |
| 收拢（张开前）跨距 | **未实测** | 关节 0 → 204.00 mm | 204 = 244 − 2×20（推导值，不是量出来的） | UNKNOWN |
| 单指行程 | 未实测（EFG80 上位命令 0..80 mm 是**驱动器坐标**） | 0 → 21.285 mm（跨距变化 42.57 mm） | SolidWorks 跨距 − 接触跨距 = 2.57 mm 预紧 | UNKNOWN（缺标定表） |
| 抓取宽度 `50.0 mm` | `grip_width_mm: 50.0`、`grip_current: 0.1`（`config/tasks/common.yaml`） | **无法换算**（见 8.4） | `docs/agent_memory.md` §75/§76 | FAIL / 缺数据 |
| 质量（夹爪本体） | **4.580 kg 实测**（夹持机构0920 STL，SolidWorks） | 挂 `my_end_effector_link` 4.580 kg（未改） + 手指/基座 0.140 kg | 2026-09-19 §9.3 | PARTIAL（重复记账 3.1%，见 8.5） |
| 质量（相机） | 无 datasheet | 0.010 + 0.050 = 0.060 kg（ESTIMATED） | 与 `camera_kg_placeholder: 0.06` 一致 | UNKNOWN |
| 质心 | 夹爪本体实测 CoM (−0.10, 14.08, 109.56) mm；手指/相机未测 | 手指 CoM 在板中心、相机 CoM 在原点（ESTIMATED） | SolidWorks（本体）/ 假设（手指、相机） | PARTIAL |
| 惯量 | 本体为 SolidWorks 真张量；其余未知 | 手指/基座/相机用实心长方体估算（ESTIMATED） | 无 CAD 分件质量属性 | UNKNOWN |
| 夹持力 | 0.1 A 限流（未标定 N） | `effort="80"`（出厂假定，未验证） | 无测力数据 | UNKNOWN |
| 闭合速度 | 未测 | `velocity="0.02" m/s`（未验证） | 无测速数据 | UNKNOWN |
| 控制接口 | PowerShell `init/open/close/grip(width,current)` | `position` 关节 + `/gripper_controller/commands` | `scripts/run_pick_and_place.py` vs `sim_controllers.yaml` | PARTIAL（语义不同：宽度/电流 vs 位置） |
| 两指同步 | 单驱动、机械同步（**未单独验证**） | URDF `<mimic>`，gz_ros2_control 实测两侧同步 | 见 8.3 实测 | PASS（仿真侧） |
| MoveIt 规划组 | — | 手指关节**不在** `cs625_arm` 组 | SRDF `chain base_link → my_end_effector_link` | PASS |
| attach `touch_links` | — | SRDF 里**没有** `<end_effector>`；运行时有等价机制 | 见 8.6 | PARTIAL |

### 8.3 实测：仿真里的关节值 ↔ 毫米跨距

换算关系（对这一简化板是精确的）：

```text
外侧面跨距 span_mm(j) = 204.0 + 2000.0 * j          (j 单位 m)
反解         j(span_mm) = (span_mm - 204.0) / 2000.0
两指内壁净开度 = span_mm - 110.0 mm                 (每臂 55 mm 厚)
```

在 `sim_cell.launch.py headless:=true` 起来后 90 s 读 `/joint_states`，再依次下发
`/gripper_controller/commands`：

| 下发命令 | `gripper_right_finger_joint` | `gripper_left_finger_joint` | 换算外撑跨距 | 说明 |
|---|---:|---:|---:|---|
| 无（初始） | 0.000 m = 0.000 mm | 0.000 m = 0.000 mm | **204.000 mm** | 收拢态 |
| 0.020 m | 0.020 m = 20.000 mm | 0.020 m = 20.000 mm | **244.000 mm** | 零预紧抓取态 = 扶手内间距 |
| 0.021285 m | 0.021285 m = 21.285 mm | 0.021285 m = 21.285 mm | **246.570 mm** | CAD 实测跨距（本次新上限） |
| 0.050 m（字面 50 mm） | 0.021285（被上限截断） | 0.021285（被上限截断） | **246.570 mm** | 50 mm 无法到达，见 8.4 |

TF 交叉验证（`tf2_echo gripper_right_finger_link gripper_left_finger_link`，0.021285 时）：
`Translation: [-0.247, 0.000, 0.000]` → 两指坐标原点相距 246.57 mm（tf2_echo 只打印 3 位小数）。

两指同步的证据（`gz_ros2_control` 日志）：

```text
[gz_ros2_control]: Joint 'gripper_left_finger_joint'is mimicking joint 'gripper_right_finger_joint' with multiplier: 1 and offset: 0
[Err] [Physics.cc:1808] Attempting to create a mimic constraint for joint [gripper_left_finger_joint]
      but the chosen physics engine does not support mimic constraints, so no constraint will be created.
```

即：**物理引擎不做 mimic 约束，但 gz_ros2_control 在接口层做了**，所以只给右手下发时
左手确实跟随（上表两列相等就是实测结果）。这一点以前没有证据，本次补上。

### 8.4 "50 mm 抓取值"自检：仿真给不出 50 mm（结论 FAIL/缺数据）

- 配置要的是 `grip_width_mm: 50.0`（`config/tasks/common.yaml`），它进的是
  `hitbot_cmd.ps1 -Cmd grip -Value 50`，即 **EFG80 的驱动器坐标**。
- `docs/agent_memory.md` §76 明确记录：**"归零后有偏移误差，显示位置与实际开度不完全一致"**，
  且寄存器 70 的电流反馈不可信。§75 的实测对照是 `52.0→52.0 state=0`、`52.5→卡在 52.231`、
  `53.0→卡在 52.231`、`54.0→卡在 51.77`——全是**驱动器读数**，没有一条伴随物理跨距测量。
- 因此**不存在** `position_mm → span_mm` 的映射。若硬把它当跨距代入 8.3 的反解：
  `j = (50 − 204)/2000 = −0.077 m`，比 `lower = 0` 还低，**URDF 不可达**；
  下发 0.050 m 时实测被截断在 0.021285（上表最后一行）。
- 结论：**仿真的 50 mm 抓取 = 无法验证**。既不能说"仿真张开到 50 mm"，也不能说
  "50 mm 对应 244 mm 跨距"——那两句话都会是编造。

### 8.5 几何一致性

- 手指碰撞/可视几何 = `box 0.055 × 0.058 × 0.146 m`，与 SolidWorks 量得的两条支撑臂截面一致
  （`cs625_task_scene.yaml` 的 `measured_arm_span_m` 同源数据）。**比对：PASS（截面）**。
- 但这只是**接触面 + 张开方向 + 截面**的承载板，**不是真实连杆**：真实机构由内部斜面/连杆把
  EFG80 的直线行程变成两臂外撑，仿真用两根直动指代替。**PARALLEL：简化**。
- 真机手指的**接触贴片/橡胶垫/倒角**没有照片与尺寸 → **未验证**（缺失清单 §8.7 第 5 条）。
- 真机工具网格 `gripper_0920.stl` **本身已经包含这两条臂**，所以仿真里手指的碰撞盒在
  `my_end_effector_link` 的网格内部；SRDF 因此把这几对 `disable_collisions` 关掉（见 8.6），
  结果是**手指与工件的接触判定完全依赖 attach/ACM，不是碰撞几何**。这点必须在任何
  "仿真抓取成功/失败"的说法里一起讲。

### 8.6 已改的数值（可复核）

```text
src/cs625_ap_description/urdf/cs625_parallel_gripper.xacro
  gripper_right_finger_joint  limit upper: 0.020    -> 0.021285   (改为 CAD 实测跨距)
  gripper_left_finger_joint   limit upper: 0.020    -> 0.021285   (mimic 跟随，同步改)
  gripper_base_link      mass: 0.001 -> 0.100 kg   inertia 1e-9 -> (8.83e-5, 8.83e-5, 1.35e-4)
  gripper_{left,right}_finger_link mass: 0.001 -> 0.020 kg  inertia 1e-9 -> (4.11e-5, 4.06e-5, 1.07e-5)

src/cs625_ap_description/urdf/cs625_camera_extension.xacro
  camera_mount_link  mass: 0.01 -> 0.010 kg  inertia 1e-6 -> (3.33e-6, 6.00e-6, 3.33e-6)
  camera_link        mass: 0.05 -> 0.050 kg  inertia 1e-5 -> (3.04e-5, 7.50e-6, 3.04e-5)
```

**这些质量/惯量全部是 ESTIMATED，不是实测**，文件里逐处写了 `ESTIMATED, needs CAD mass/CoG`。
估算依据与"为什么故意取小"：

| link | 值 | 依据 | 性质 |
|---|---:|---|---|
| `gripper_base_link` | 0.100 kg | 90×90×50 mm 安装座包络、实心长方体惯量 | 估算（记账用） |
| 每根手指 | 0.020 kg | 同上包络 55×58×146 mm | 估算（记账用） |
| `camera_mount_link` | 0.010 kg | 60×60×20 mm 支架板 | 估算 |
| `camera_link` | 0.050 kg | 30×80×30 mm 深度模块 | 估算（偏低） |

**为什么手指不是"看起来合理"的 1.2 kg**：整块 55×58×146 mm 的实心铝在实测密度
2.677 g/cm³ 下是 **1.246 kg/臂**，但 `my_end_effector_link` 上的 **4.580 kg 实测质量已经包含这两条臂**。
把它们按物理估算填进去会变成"7.4 kg 的末端"（实测 4.580 kg 的 1.6 倍），
对重力补偿/加减速的伤害比 1 g 占位更大。所以这里选择"小值记账"，
**代价是重复记账 0.140 kg（实测的 3.1%）**，并在注释里写明正确做法是等 CAD 分件质量属性
把 4.580 kg 拆到各 link（那要改 `cs625_active_perception.urdf.xacro`，本次不在授权范围内）。

相机同理：真机的相机是**工具网格的一部分**（`cs625_camera_extension.xacro` 注释），
0.060 kg 与 `config/cs625_task_scene.yaml` 的 `camera_kg_placeholder: 0.06` 一致，
但真实的 PS800-E1 应该更重 → **这份 0.06 kg 是下限，不是实测**。

### 8.7 MoveIt 侧现状（`src/cs625_bringup/config/cs625_active_perception.srdf`）

1. **夹爪关节不在规划组里**（这正是要的，避免规划时多自由度）：

```xml
<group name="cs625_arm">
  <chain base_link="base_link" tip_link="my_end_effector_link"/>
</group>
```

考点：chain 的 tip 是 `my_end_effector_link`，而 `tool0 → gripper_base_link → *_finger_link`
都是它的子链，所以 `gripper_left_finger_joint` / `gripper_right_finger_joint` **不属于**
`cs625_arm`。P7 的夹爪指令走 `/gripper_controller`，不经 MoveIt 规划器。

2. **SRDF 里没有 `<end_effector>`，因此没有 `touch_links`**。夹爪相关的只有六条
   `disable_collisions`：

```xml
<disable_collisions link1="gripper_base_link" link2="gripper_left_finger_link"  reason="Adjacent"/>
<disable_collisions link1="gripper_base_link" link2="gripper_right_finger_link" reason="Adjacent"/>
<disable_collisions link1="gripper_left_finger_link" link2="gripper_right_finger_link" reason="Never"/>
<disable_collisions link1="my_end_effector_link" link2="gripper_base_link" reason="Adjacent"/>
<disable_collisions link1="my_end_effector_link" link2="gripper_left_finger_link" reason="Never"/>
<disable_collisions link1="my_end_effector_link" link2="gripper_right_finger_link" reason="Never"/>
```

3. **`touch_links` 在运行时补，覆盖了手指**：
   - P7 抓取：`test/p7_apply_fixture_scene.py` 的 `attached_target_proxy()` 给出
     `touch_links = [p7_grasp_center_link, gripper_left_finger_link, gripper_right_finger_link]`
     → **两指都覆盖**。
   - 六模式：`scripts/update_grasped_box.py` 不用 `touch_links`，而是把
     `box_collision` 与 9 个机器人 link（含两根手指）的 ACM 全部置为允许 → 等价效果。
   结论：**抓取后 attach 的"允许接触"是覆盖手指的**，但它是脚本里的运行时行为，
   不是 SRDF 声明 → 换一个入口（例如纯 MoveIt 的 `AttachedCollisionObject` 手写）
   就会漏。建议（不在本次授权内）在 SRDF 里补 `<end_effector>`。

### 8.8 仍然缺失的真机数据（按重要性）

| # | 缺什么 | 用来消掉哪条 UNKNOWN |
|---|---|---|
| 1 | **EFG80 `position_mm` ↔ 外撑跨距的标定表**（至少 0/10/20/30/40/50 mm 六点，用卡尺量两臂外侧面） | 50 mm 抓取值、单指行程、`lower` 上限 |
| 2 | 收拢态跨距（回零后两臂外侧面距离） | 仿真 `lower = 0` 对应 204 mm 是否成立 |
| 3 | 夹爪分件质量/质心（基座 + 单臂 + 驱动电机的 CAD 质量属性） | 0.140 kg 重复记账、手指 CoM |
| 4 | PS800-E1 相机与支架的 datasheet 重量 | 0.060 kg 是否可信 |
| 5 | 手指接触面照片 + 贴片厚度/材料 | 碰撞盒外的橡胶垫是否改变有效跨距 |
| 6 | 0.1 A 下的实际夹持力（N，每臂） | URDF `effort=80` |
| 7 | 闭合/张开速度（mm/s） | URDF `velocity=0.02` |
| 8 | 两指是否机械强制同步（单电机还是双电机） | 仿真 mimic 是否合理 |

### 8.9 复现命令（本次实际跑过的）

```bash
cd ~/projects/cs625-active-perception
source scripts/source_dev_env.sh --full

# 1) 改完描述包必须重建（symlink-install 也要跑，确保 install 里有新文件）
colcon build --symlink-install --packages-select cs625_ap_description
# 2) 起仿真（用独立 ROS_DOMAIN_ID，避免和真机/别的会话的 /joint_states 混在一起）
ROS_DOMAIN_ID=79 ros2 launch cs625_bringup sim_cell.launch.py headless:=true launch_rviz:=false
# 3) 等 90 s，读关节
ros2 topic echo /joint_states --once          # 找 gripper_{left,right}_finger_joint
# 4) 下发 20 mm / 21.285 mm / 50 mm 并复读
ros2 topic pub --once /gripper_controller/commands \
    std_msgs/msg/Float64MultiArray "{data: [0.02]}"
# 5) 跨距交叉验证
ros2 run tf2_ros tf2_echo gripper_right_finger_link gripper_left_finger_link
# 6) 停：给 launch 进程发 SIGINT（不要 pkill 通配）
kill -INT <launch_pid>
```

### 8.10 本次验证时观测到的两个无关异常（供后续复查，未修）

1. **`prepare_gazebo_model.py` 起不来**（2026-10-24 22:0x 的 `sim_cell.launch.py`）：

```text
xacro: error: expected exactly one input file as argument
argv 里出现 'prefix:=safety_limits:=true'，说明 `cs_type:=` 与它的值被拼成了两个 argv 项
```

后果：Gazebo 实体用的是**上一次成功生成**的 `/tmp/cs625_active_perception_gazebo_model.urdf`，
不是当次源码展开的结果。本次已核对那个文件里 `upper=0.021285`、mimic 都在，
所以夹爪证据有效；但这是一个**实体落后于源码**的保真度风险，需要单独修。

2. **`joint_trajectory_controller` 未激活**（同一次实例）：

```text
[spawner_joint_trajectory_controller]: Failed to activate controller : joint_trajectory_controller
```

夹爪控制器不受影响（本节的测量全部有效），但手臂轨迹执行在该实例里不可用。
原因未查（可能与本机并发起了 3~4 个 Gazebo 实例有关），需要单独复查。
