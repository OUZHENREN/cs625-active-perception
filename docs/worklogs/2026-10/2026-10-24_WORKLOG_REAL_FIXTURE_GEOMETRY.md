# 2026-10-24 WORKLOG — 用真机几何替换插槽夹具，并验证插入链（真机未动）

> 本轮目标：判定哪份夹具几何是真机那只；把它 1:1 接进 Gazebo/MoveIt；
> 在夹具碰撞体存在时验证 6 个插入相关阶段。**没有动真机，没有 git 提交，没有 pkill。**
>
> 一句话结论：舍入误差 0 的候选是 RVS `slot_pointcloud.STL`（96,760 面、毫米），
> 已转成 60,000 面、米制的 `slot_fixture_collision.stl`。旧仿真夹具的坐标系
> 与它相差 194 mm（槽口中心 vs 插入路径），这正是今天 `insert` 撞实心壁的原因。
> 换上以后，plan-only 的 6 个阶段全 ok；但**严格碰撞检查**（`avoid_collisions=True`）
> 暴露的是**机器人模型问题**，不是夹具几何问题：
>
> - 正在跑的 vendor 描述：`camera_link` 的 3×5×2 cm 占位盒在 z≈0.026 撞夹具；
> - 应用描述（真机工具 `gripper_0920.stl`）：294.6 mm 宽的工具被摆成沿世界 x，
>   撞夹具顶梁 —— TCP/工具坐标系约定问题（任务位姿是夹爪尖，`my_end_effector_link` 是法兰）。
>
> 这两条都不在本任务“夹具几何”范围内，需要单独的机器人模型/位姿口径修正。

---

## 1. 判定：哪份几何是真机夹具

命令：`python3 scripts/analyze_fixture_geometry.py`

真机点云重建复用 `scripts/build_scene_model.py` 的 `load_metadata` / `frame_to_base`
（没有第二套深度投影代码）。空槽批次 `slot*` 逐帧测量后取中位数：多帧直接合并会把
槽腔"糊住"（实测槽宽从 457 mm 掉到 341 mm），所以按帧测。

```text
真机 RGB-D 重建（slot* 空槽批次，逐帧测量后取中位数）
  帧数/点数     : 16 帧 / 2787618 点
  可见 bbox     : 267 × 772 × 510 mm  (x/y/z，逐帧 bbox 并集)
  槽宽中位数     : 450.0 mm  （5/9 层贯通）
  逐层         : z=0.38:--, z=0.33:--, z=0.29:448.0, z=0.25:448.0, z=0.20:450.0,
                 z=0.16:453.0, z=0.12:468.0, z=0.08:--, z=0.03:--

候选                                     格式/单位         顶点/面          bbox x×y×z (mm)  槽宽中位 (mm)  贯通层  槽口偏离路径 (mm)
-------------------------------------  ------------  ------------  ---------------  ---------  ---  -----------
RVS STL (slot_pointcloud.STL)          ascii STL mm  290280/96760  200×745×508      457.0      6/9  1
RVS PLY (slot_pointcloud20260127.ply)  binary PLY m  300000/0      153×474×435      n/a        -    -
legacy slot_collision.STL              ascii STL mm  1080/360      200×745×491      457.0      6/9  1
old sim slot_fixture.stl               binary STL m  27018/9006    232×745×506      447.5      2/9  194
```

结论（脚本原始输出）：

```text
结论
  选用: rvs_mesh
    ✓ rvs_mesh: bbox y/z ok, 槽宽 457.0 mm (真机 450.0 mm), 槽口对中
    ~ legacy: 跌破保真度的碰撞代理（360 面），源自 rvs_mesh（数值判据也过，但楔形/底面细节已被削掉）
    ✗ rvs_ply: 与 rvs_mesh 是同一扫描表面（点到面 p99 2.6 mm），但只是单视角点云、没有三角面，
               不能当碰撞网格；作为“同一物体”的交叉证据保留
    ✗ old_sim: 插入路径只有 2/9 层是贯通槽（多数高度不在槽内）；槽口中心偏离插入路径 194 mm
```

逐条解释：

| 候选 | 是什么 | 为什么不是（或不是首选） |
|---|---|---|
| `slot_pointcloud.STL` | 真机夹具，RVS 模型系，96,760 面，毫米 | **选用**。槽宽 0.457 m、槽口中心离插入路径 1 mm、bbox 与真机重建一致 |
| `slot_pointcloud20260127.ply` | 同一物体的单视角点云（300k 点） | 与 STL 的表面距离 p99 = **2.6 mm**，证明是同一扫描；但只有表面点、没有面，且有遮挡，bbox 不能当完整 AABB（0.153×0.474×0.435） |
| `slot_collision.STL` | 同一几何的 360 面碰撞代理 | 数值判据也过，但底部裁掉 16.8 mm、楔形槽被拉直成 0.457 直槽；不能代表“楔形”的真实几何，只做交叉验证 |
| `slot_fixture.stl`（旧仿真） | 另一套 SolidWorks 模型/坐标系 | bbox 差 32 mm（x 0.232 vs 0.200）、槽宽 0.445 vs 0.457、**槽口中心偏 194 mm**；按 bbox 中心摆到 `slot_origin` 后槽在 y≈+0.18，插入路径在 y≈−0.013 |

关于 bbox 判据的诚实说明：真机 RGB-D 重建是**可见表面**，x 方向受遮挡；
多帧配准还有 1–2 cm 抖动（单帧 bbox 205×570×468，逐帧并集 267×772×510）。
所以 bbox 只能当“一致性包络”，真正有区分度的是**槽宽**和**槽口是否落在插入路径上**。
`grid`/`bbox` 判据在这份数据上无法单独区分 RVS 与旧网格（两者都落在包络内），
是槽宽（7 mm vs 10 mm）和槽口中心（1 mm vs 194 mm）把旧网格排除掉的。

---

## 2. 生成的真机碰撞网格

命令：`python3 scripts/build_fixture_collision_mesh.py`

```text
输入    : .../RobotVisionSuite/runtime/Model/slot_pointcloud.STL  (ascii STL, 96760 面, 毫米)
输出    : src/cs625_simulation/assets/cs625_real_cell/slot_fixture/meshes/slot_fixture_collision.stl
输出    : 60000 面 / 180000 顶点 / 2.86 MiB / 米制
bbox    : [-0.1773, -0.5655, -0.5077] -> [0.0227, 0.1795, -0.0] m
尺寸    : 0.2000 × 0.7450 × 0.5077 m
槽宽    : 原始 0.457010 m -> 抽稀 0.457010 m (差 0.00 mm)
自检    : bbox 保持 True；槽宽保持 True；面数 ≤ 80000 True
```

坐标系：**RVS 模型系**（与 `slot_origin_q` 标定同一个系），模型原点在夹具顶面/前角，
**不是零件中心**。新增模型：

```text
src/cs625_simulation/assets/cs625_real_cell/slot_fixture/model.sdf
src/cs625_simulation/assets/cs625_real_cell/slot_fixture/model.config
src/cs625_simulation/assets/cs625_real_cell/slot_fixture/meshes/slot_fixture_collision.stl
```

`model.sdf` 是 static + mesh collision + mesh visual；注释里写了 bbox 供
`declared_bbox_from_comment()` 交叉校验（与 STL 最大差 0.021 mm）。

---

## 3. 接入世界与 MoveIt 场景

命令：

```bash
python3 scripts/build_sim_world.py \
  --out-world src/cs625_simulation/worlds/cs625_real_cell.sdf \
  --out-config config/cs625_real_cell_scene.yaml \
  --report /tmp/sim_fit_report.json --figure /tmp/sim_world_topview.png
```

改动只有夹具这条 include（桌子与工件逻辑未动）：

```diff
- FIXTURE_URI = "model://cs625_task/slot_fixture"
+ FIXTURE_URI = "model://cs625_real_cell/slot_fixture"
- fixture_mesh = .../cs625_task/slot_fixture/meshes/slot_fixture.stl
+ fixture_mesh = .../cs625_real_cell/slot_fixture/meshes/slot_fixture_collision.stl
- fixture_pose = seat_bbox_on_plane(bbox, slot_origin[:2], slot_table_top, yaw)
+ fixture_pose = {"xyz": slot_origin, "rpy": [0, 0, yaw]}
```

拟合报告（原始输出摘录）：

```text
拟合报告（aligned point = 模型 origin 或 bbox 中心，vs 实测参考点）
model            aligned point world [m]               measured reference [m]                reference      dev [m]  axes  alignment
------------------------------------------------------------------------------------------------------------------------------------
slot_fixture     (  1.241629,  0.181709,  0.443245)    (  1.241629,  0.181709,  0.443245)    slot_origin     0.0000   xyz  model_origin
shielding_module (  0.712576,  0.597211,  0.130014)    (  0.720593,  0.618232,  0.133546)    cad_centre      0.0228   xyz  measured_cad_pose

对齐结论:
  夹具 slot_fixture：采用 **模型 origin = slot_origin** 对齐（不是 bbox 中心）。
    生成位姿：xyz=[1.241629, 0.181709, 0.443245] rpy=[0, 0, 0.006754]
    bbox 中心偏离 slot_origin xy 207.9 mm、z 253.9 mm（坐标系的定义，不是误差）。
    夹具 bbox 底面在插槽桌顶面以下 23.7 mm（底部法兰）。
```

为什么不是 bbox 中心：`slot_origin` 是 RobotVisionSuite 对**模型 origin** 的标定值。
模型 origin 离 bbox 中心 (−0.076, −0.194, −0.254) m；按 bbox 中心摆，楔形槽口会落到
世界 y≈+0.18 —— 正是旧夹具的做法，也正是 194 mm 偏差的来源。
底部法兰比桌面低 23.7 mm 也保留：模型 origin 压在 `slot_origin` 时顶面 z=0.4432，
RGB-D 实测顶面 ≈0.441、槽宽 0.457 vs 实测 448–468 mm；为了“贴桌面”整体抬 23.7 mm，
顶面和槽宽就都对不上了。

`apply_real_cell_scene.py` **不需要改**：它本来就解析 scene yaml 里的
`fixture.model_uri`，并优先选文件名带 `collision` 的网格。

```text
$ python3 scripts/apply_real_cell_scene.py --dry-run
scene config : .../config/cs625_real_cell_scene.yaml
world frame  : world
  real_cell_slot_fixture       mesh  xyz=[1.2416, 0.1817, 0.4432] .../cs625_real_cell/slot_fixture/meshes/slot_fixture_collision.stl
  cross-check: workpiece_table: top_z config -0.0910 vs model -0.0910 (Δ 0.0 mm)
  cross-check: slot_table: top_z config -0.0407 vs model -0.0407 (Δ 0.0 mm)
dry run: nothing published
```

---

## 4. 规划验证：plan-only 的 6 个阶段

命令（load 流程，原始输出节选）：

```bash
python3 scripts/run_pick_and_place.py --config runtime/effective_load_safe_load.yaml \
  --plan-only --start-joints "0.559681835159803,-0.886110693244363,-2.6479421412927606,\
0.004946539382850723,1.8827091289653233,1.4481534658974855"
```

```text
[pre_insert]     ok=True points=102 dur=9.9s legs=2 band=None
[insert]         ok=True points=631 dur=62.9s legs=1 band=None
[straighten]     ok=True points=401 dur=40.0s legs=1 band=None
[withdraw]       ok=True points=160 dur=15.6s legs=3 band=(0.0, 80.0)
```

unload 流程（从 pre_insert 的 IK 起，跳过已在位的 d1/d2/d2a）：

```text
[d2b_entrance_tilted] ok=True plan-only v=0.03 points=205 dur=20.2s legs=2
[d2c_straighten]      ok=True plan-only v=0.005 points=303 dur=30.1s legs=1
[d4_tilt_for_removal] ok=True plan-only v=0.005 points=0 dur=0.0s legs=0
[d5_extract_1]        ok=True plan-only v=0.005 points=285 dur=28.4s legs=1
[d5_extract_2..18]    ok=True （每步 23–39 点）
[d6a_lift]            ok=False plan failed（不在本轮 6 个目标阶段内）
```

**但 plan-only 不能当“碰撞下可行”的证据**，有两个硬编码原因（未修改 runner）：

1. `run_pick_and_place.py` 的 `NO_COLLISION_CHECK_PHASES = TIGHT_CLEARANCE_PHASES | {"d2b_entrance_tilted"}`，
   而 `plan_cartesian(..., avoid_collisions=not _no_collision_check(name))`（916 行）。
   也就是说 insert / straighten / d2b / d2c / d4 / d5 **全部关闭碰撞检查**。
   实验验证：在插入路径 (1.13, −0.013, 0.20) 放一个 10 cm 的探针 box，insert 仍然 ok=True。
2. `d4_tilt_for_removal` 的 legs=0：`build_cartesian_legs` 只看**位置**差，不看姿态差；
   纯旋转段（straighten / d2c / d4）在 plan-only 里可能被当成“已满足”而真空通过。

---

## 5. 严格碰撞检查：到底谁撞了哪里

新增 `scripts/verify_fixture_clearance.py`：同一条 TCP 链，但强制
`compute_cartesian_path(avoid_collisions=True)` + `check_state_validity`，
失败时二分到第一个碰撞状态并取 contacts。

### 5.1 用正在跑的 vendor 描述（`eli_cs_robot_description`）

```text
[insert]  FAIL fraction=0.6068
  最后合法 TCP [1.1347, -0.0129, 0.1678]；第一个碰撞 TCP [1.1498, -0.0126, 0.0929]
  接触 real_cell_slot_fixture <-> camera_link @ [1.1206, 0.0119, 0.0263] (world)
```

`camera_link` 是 vendor 描述里的 3×5×2 cm 占位盒（joint `camera_joint`，
xyz="0.05 0 0.05"）；应用描述 `cs625_camera_extension.xacro` 已经明确把它删掉
（“camera is part of the real tool mesh”）。它在这条链上比夹具几何更早挡住。
这不是夹具问题：接触点在夹具**前下部**，而槽腔在 z≤0.26 的 6 层里都是贯通的。

### 5.2 用应用描述（`cs625_ap_description`，真机工具网格）

```text
规划场景里的世界碰撞体: real_cell_slot_fixture(60000 面)
pre_insert IK ok；该状态 valid=False
  接触: real_cell_slot_fixture <-> my_end_effector_link @ [1.2494, 0.0022, 0.3653]
[insert] FAIL fraction=0.0000
  接触 real_cell_slot_fixture <-> my_end_effector_link @ [1.1225, 0.0367, 0.1028]
```

把任务位姿按“夹爪尖 → 法兰”抬高 0.207 m（`gripper_0920.stl` 的 +Z 长 0.207 m）后：

```text
pre_insert IK ok；valid=True
[insert] FAIL fraction=0.1966
  最后合法 TCP [1.0767, -0.0124, 0.5136]；第一个碰撞 TCP [1.0767, -0.0124, 0.5135]
  接触 real_cell_slot_fixture <-> my_end_effector_link @ [1.2427, 0.0167, 0.409] (world)
```

工具 294.6 mm 的**宽轴（X）**被摆成沿世界 x，而夹具 x 向只有 0.2 m；顶梁在
z≈0.41 处被工具的 +X 端撞到。这不是夹具几何问题，是**工具/TCP 坐标系约定**：
任务位姿是夹爪尖（真实控制器 TCP），MoveIt 的 `my_end_effector_link` 是法兰；
并且真实工具宽轴在插入时应沿 y（夹具长轴 0.745 m），而不是 x。

### 5.3 夹具本身是可行的（几何证据）

- 槽宽 0.457 m，槽腔沿 y 贯通 6/9 层，槽口中心离插入路径 1 mm；
- 工件截面（`shielding_module.stl` 中段）宽 0.441–0.445 m，0.457 − 0.445 ≈ **12 mm 总间隙**
  （单边 ~6 mm；与日志“单边约 1 mm”同量级，差异来自扫描网格与测量口径）；
- 旧夹具在插入路径上只有 2/9 层贯通、槽口偏 194 mm —— 那才是 insert 失败的原因。

---

## 6. 单测

`test/test_fixture_geometry.py`（20 个纯函数测试）+ 更新后的 `test/test_sim_world_fit.py`：

```text
$ source scripts/source_dev_env.sh --full
$ python3 -m pytest test/ -q
205 passed in 15.77s
```

覆盖：ASCII/二进制 STL 解析、PLY body offset、mm→m、bbox、合成槽道 `through_gap`、
点云直方图槽宽（含飞点抑制）、插入线采样、profile 汇总、RVS placement、
夹爪尖→法兰换算、已提交碰撞网格的米制/面数/bbox/文件大小。

---

## 7. 修改/新增文件

| 文件 | 状态 | 作用 |
|---|---|---|
| `scripts/analyze_fixture_geometry.py` | 新增 | 四份候选 + 真机重建的 bbox/槽宽判定 |
| `scripts/build_fixture_collision_mesh.py` | 新增 | mm→m、抽稀到 60k 面、自检槽宽不变 |
| `scripts/verify_fixture_clearance.py` | 新增 | 强制碰撞检查的 6 段插入链验证 |
| `src/cs625_simulation/assets/cs625_real_cell/slot_fixture/**` | 新增 | 真机夹具模型 + 2.86 MiB 碰撞网格 |
| `scripts/build_sim_world.py` | 修改 | 夹具 URI / mesh / 位姿规则（模型 origin = slot_origin） |
| `src/cs625_simulation/worlds/cs625_real_cell.sdf` | 重新生成 | 夹具 include 指向新模型 |
| `config/cs625_real_cell_scene.yaml` | 重新生成 | `fixture.model_uri/pose_world` 用新值 |
| `test/test_sim_world_fit.py` | 修改 | 夹具断言改为“模型 origin 落在 slot_origin” |
| `test/test_fixture_geometry.py` | 新增 | 20 个纯函数测试 |
| 旧 `cs625_task/slot_fixture/**` | **未动** | 保留，仍被其他引用使用 |

未改：`scripts/run_task.py`、`scripts/run_pick_and_place.py`、`config/tasks/*`、
真机、任何 vendor 代码；没有 git 提交；没有 pkill。

---

## 8. Capability layer

```text
Capability layer
- environment and dependencies      : PASS（ROS Jazzy + venv 的 numpy/scipy/open3d）
- robot model and simulation        : BLOCKED（vendor camera_link 占位盒 / 应用描述的工具宽轴约定）
- kinematics, control and planning  : PASS（plan-only 6 段全 ok，IK 可达）
- vision, hand-eye calibration and TF: NOT TOUCHED（只读用了标定与帧位姿）
- active perception / NBV           : NOT STARTED（本任务不涉及）
- real-hardware integration/safety  : NOT TOUCHED（真机不可达，全程未发 action goal）

Acceptance target
- objective      : 真机夹具 1:1 接入 + 插入链在碰撞检查下可行
- pass criteria  : 几何判定明确；网格 ≤80k 面；6 段在碰撞体存在时规划通过
- runtime evidence: analyze/build/apply 命令输出、plan-only 原始日志、
                    verify_fixture_clearance 的 fraction + contact

Critical-chain status
- URDF -> Gazebo entity             : PASS（新 model.sdf 被 world include，verify 时场景有 60000 面对象）
- ros2_control -> joint_states      : NOT USED（plan-only / 服务调用，未执行轨迹）
- base_link -> camera optical TF    : PASS（沿用既有 TF，未改）
- RGB-D -> normalized sensor topics : NOT USED（离线读数据集）
- point cloud -> MoveIt planning scene: PASS（apply_real_cell_scene 发布 60k 面 mesh）
- NBV decision -> robot execution   : NOT STARTED

Sim/real alignment
- reused common core  : build_scene_model 的投影/位姿函数被分析脚本复用
- sim entry point     : build_sim_world.py -> world + scene yaml
- real entry point    : 未使用（真机不可达）
- real execution      : 不允许（本任务禁止）
- execute/confirmation: 全程未执行
```

---

## 9. 限制与下一步

1. **plan-only 不能证明碰撞可行**：6 个紧间隙阶段在 runner 里是
   `avoid_collisions=False`，必须用 `verify_fixture_clearance.py` 或给 runner 加
   “严格模式”才能验收。本轮没有改 runner（硬约束）。
2. **严格检查的 blocker 在机器人模型，不在夹具**：
   - vendor 描述的 `camera_link` 占位盒应删除（应用描述已经删了）；
   - 应用描述的工具宽轴与任务位姿口径（夹爪尖 vs 法兰、宽轴 x vs y）需要统一；
   - 这两项都要改 description/SRDF 或位姿配置，超出“只做夹具几何”的范围。
3. **规划环境有多个 move_group**（域 0 / 47 / 77，部分在自动重启），
   `/compute_cartesian_path` / `/check_state_validity` 由哪个实例回答不确定。
   本轮截图来自固定域（47，应用描述；vendor 那次来自域 0），建议后续验收固定一个域。
4. 真机点云重建的 bbox 只能当包络（遮挡 + 1–2 cm 配准抖动），槽宽是逐帧测的；
   “1 mm 单边间隙”与实测 6 mm 单边的差异需要真机复测确认（可能与工件实际外廓/倒角有关）。
5. 下一步唯一推荐：**先把工具/TCP 口径和 camera_link 占位盒修正掉**，然后重跑
   `verify_fixture_clearance.py`；夹具几何这条线已经可以冻结。
