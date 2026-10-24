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
| 相机内参 | 已改为真机 1280×960、水平 FOV 68.9°（由 fx 931.47 反算） | 代码已改，**运行时未复核**（本机 Gazebo 反复被终止） |
| 相机帧率 | 已设 1 Hz（真机实测 0.79 fps） | 代码已改，运行时未复核 |
| 相机安装位 | 用 RVS 手眼链：`camera_link` = 深度眼，`camera_color_xyz` = 彩色眼 24.4 mm 基线；传感器渲染在**彩色眼** | PARTIAL（见 §4） |
| 夹爪行程/几何 | URDF 里质量是占位值（0.001 kg 等） | FAIL（未对齐） |
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
| 水平 FOV | 68.9°（fx 931.47） | 1.20274 rad = 68.9° | 由真机内参反算，不再用 vendor 的 60° |
| 帧率 | 0.79 fps（深度流是瓶颈） | `camera_update_rate=1.0` | 用 1 Hz 复现"观测要等新帧" |
| 深度噪声 | 金属件 3~4 mm 散布 + 约 25% 空洞 | Gaussian σ=3 mm；**空洞不模拟** | 空洞是渲染限制，不伪造 |
| 相机位姿 | `T_tcp_cam`（彩色光心，距 TCP 0.2958 m） | `camera_link`=深度眼（`camera_extrinsics_sim.yaml`），彩色眼由 `camera_color_xyz` 偏置 24.4 mm；**渲染传感器移到彩色眼** | 见下 |

**为什么渲染要放到彩色眼**：应用用 `T_tcp_cam` 反投影彩色像素，而真机深度是配准到彩色的
（`depth_registration_enable:=true`）。Gazebo 的 rgbd_camera 只能在一个位姿渲染两路，
放在彩色眼才与真机发布的图像一致；代价是仿真深度也来自彩色眼（真机是两路独立后配准），
这一项标 PARTIAL。

**未验证**：本次会话里 Gazebo 实例反复被（并发的其他会话/清理动作）终止，
`/camera/color/camera_info` 的 K 矩阵与实测帧率没有在稳定实例上复核。复核方法：

```bash
ROS_DOMAIN_ID=63 GZ_PARTITION=cs625sim \
  ros2 launch cs625_bringup sim_cell.launch.py headless:=true launch_rviz:=false
# 等 90 s 后
ros2 topic echo /camera/color/camera_info --once      # 期望 fx≈931.5 fy≈931.6 cx≈648.7 cy≈449.3
ros2 topic hz /camera/color/image_raw                 # 期望 ≈1 Hz
```

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

## 7. 仍未解决的一个疑点（需要真机数据确认）

夹具碰撞体发布后，严格碰撞检查下 `pre_insert` 会撞夹具顶梁（接触点 (1.2494, 0.0022, 0.3653)）。
初步分析指向 **TCP/工具坐标系约定**：任务位姿是按"夹爪尖"给的，而 MoveIt 的
`my_end_effector_link` 与 `tool0` 之间还有 0.353 m 的偏移（`T_FLANGE_TCP`）。
真机上这些位姿跑通过，说明至少有一个环节的定义与仿真里假设的不同。
**这个疑点没解决之前，不要声称"仿真里插入不可行"或"真机插入有问题"。**
