# 2026-11-02 PLAN 环境碰撞接入方案（参照师兄 environment_collision_generator）

> 专项目标：把实测环境几何灌进真机 MoveIt 规划场景，替换当前的"空场景 + 离线包络门"。
> 依据：通读师兄 `environment_collision_generator` / `environment_point_cloud_publisher`
> 与本仓库 `model_environment.py` / `apply_real_cell_scene.py` /
> `build_environment_collision.py` / `check_tool_envelope.py`。
> 状态：方案待所有者批准后实施。

## 1. 现状对照

### 师兄管线

```text
PCD 目录(时间戳子目录)
  → environment_point_cloud_publisher  （合并/体素降采样 0.01/统计+半径离群剔除）
  → /environment/point_cloud (base_link, 1 Hz, 自动重扫)
  → environment_collision_generator
      · 体素化环境云：0.05 m、≤500 盒、≥1 点/体素
      · 减去 slot 网格区域、减去 box 网格区域（各 +15 mm padding）
      · env_voxel_* 盒 → /planning_scene (world 系)
```

### 本仓库已有等价物

| 师兄部件 | 本仓库等价物 | 状态 |
|---|---|---|
| point_cloud_publisher | `scripts/model_environment.py`（离线）→ `runtime/environment/<ts>/scene.ply` + model.json；`scripts/publish_environment_point_cloud.py` → `/environment/point_cloud` | ✅ 已有 |
| 体素化 | `model_environment.py::voxel_collision_bodies()`：0.02 m 边、2.5-D 盒、**顶面与桌面齐平、盒体悬在表面下方**（不占用桌面上方自由空间）→ 存进 model.json `collisions` | ✅ 已有（离线） |
| 旧运行时发布 | `scripts/build_environment_collision.py`（PCD→规划场景，无减除、无订阅） | 🔶 旧版，缺减除 |
| 夹具网格 | `apply_real_cell_scene.py` 发布 fixture 网格 | ✅ 已有 |
| 离线几何门 | `check_tool_envelope.py` / `check_carried_clearance.py` | ✅ 已有（过渡方案） |

### 缺口（本次专项）

**缺"model.json `collisions` → 规划场景"的运行时发布器，且带师兄的"减除"能力：**

1. 减除**插槽区域**：夹具网格已由 apply_real_cell_scene 单独发布，环境盒在夹具区域会双重计数；
2. 减除**工件区域**：工件不发布（抓取需要空间），环境盒若覆盖工件脚印会挡住抓取/抬升——这正是师兄 `enable_box_subtraction` 的目的；
3. 减除尺寸需要 padding（师兄用 15 mm）。

## 2. 方案：新建 `scripts/apply_environment_collision.py`

```text
输入：
  runtime/environment/latest/model.json        # collisions（2.5-D 体素盒，base 系）
  config/cs625_real_cell_scene.yaml            # fixture 网格 bbox / module bbox（减除区）
  --workpiece-xyz/--workpiece-quat             # 运行时实测 S3（新鲜工件位姿，减除区中心）
  --padding 0.015                              # 沿用师兄 15 mm

处理：
  1. 读 model.json 的 collisions（顶面齐平桌面的 2.5-D 盒）
  2. 减除区 A：夹具——用 config fixture bbox（模型系）+ 位姿变换到 world，扩 padding
  3. 减除区 B：工件——用 config module bbox + 实测 S3 变换到 world，扩 padding
  4. 与两个减除区 AABB 相交的环境盒 → 丢弃（保证夹具/工件上方与周围自由）
  5. 剩余盒 env_* + 夹具网格 → ApplyPlanningScene（ADD；旧 env_* 先 REMOVE）
  6. 打印：发布盒数 / 减除盒数 / 与 apply_real_cell_scene 的交叉核对

输出：/planning_scene（world 系）
CLI：--dry-run / --remove / --keep-workpiece（不抓取时保留工件区）
```

数据流：

```text
model.json(collisions) + fixture bbox + S3(工件) + padding
        → apply_environment_collision.py
        → ApplyPlanningScene(world)
        → MoveIt 规划场景 = 环境体素盒(减除后) + 夹具网格
```

## 3. 接入示教链路

```text
T3 拿到新鲜 S3 之后、step 5 预抓取之前：
  python3 scripts/apply_environment_collision.py --workpiece-xyz <S3> --workpiece-quat <S3>
（场景刷新后才规划 pre_grasp/grasp；抓取成功后工件 attach 属专项 2，先留钩子）
```

## 4. 必须先回答的风险（10-27 的真实教训）

`2026-10-27_WORKLOG_RVS_TEACHING_CHAIN.md` 记录：**"桌子灌入后抓取位规划失败（实测历史位姿也失败），当前做法是清空场景只做抓取"**。

所以本方案的验收标准不是"场景灌进去了"，而是：

```text
灌入场景（含减除）后：
  plan-only pre_grasp 规划成功 ？   plan-only grasp 规划成功 ？
  plan-only lift/transit 规划成功 ？
```

三条全部成功才允许在真机启用；若抓取段仍规划失败，退路是**分段启用**：
抓取段保持空场景 + 离线包络门（现状），转运/插入段启用环境场景。这正是"减除"要解决的
问题——减除区要足够大，让抓取走廊自由，又不至于把真实桌子挖穿。

## 5. 决策点（需要所有者拍板）

1. 减除 padding 用师兄的 15 mm，还是按工件半尺寸 + 安全余量（建议 15 mm 起步，实测调）？
2. 抓取段规划若在灌入场景后仍失败：接受"分段启用"退路，还是继续扩大减除区？
3. 环境盒只建模桌面 band（桌子立面/桌腿不在内）——接受 2.5-D 近似，还是补桌腿盒？

## 6. 验证阶梯

```text
1. python3 scripts/apply_environment_collision.py --dry-run      # 打印盒数/减除数，不动
2. 发布后 ros2 service call /get_planning_scene 核对对象数
3. plan-only：pre_grasp / grasp / lift / transit 四条轨迹全部规划成功（不动臂）
4. 真机执行前，四条轨迹再过 check_tool_envelope.py 交叉验证
```

## 7. 产出清单

- 新增 `scripts/apply_environment_collision.py`（≈150 行，复用 model.json 与 config）
- `scripts/teaching/teaching_sequence.py`：T3 后加一次场景刷新调用（1 行级）
- 测试：`test/` 加纯逻辑测试（减除 AABB 判断、盒过滤），不含 ROS
- 文档：本 PLAN + worklog 实施记录

## 8. 实施记录与验收结果（2026-11-02 晚，已按所有者三项决策执行）

```text
已实现
  scripts/apply_environment_collision.py    model.json collisions(1476 盒) → ApplyPlanningScene(world)
                                            fixture/workpiece 减除区(15 mm XY padding)、--dry-run/--remove/
                                            --keep-workpiece；ApplyPlanningScene: success 实测
  test/test_apply_environment_collision.py  5 个纯逻辑测试（AABB/旋转/相交/过滤）5/5 通过
  teaching_sequence.py                      接入点按验收结果落位（见下）

验收（真机 MoveIt，不动臂）
  A 场景灌入后 grasp 规划：NO_COLLISION_FREE_IK  ❌  ← 10-27 教训重现
  B 场景灌入后 lift 规划：  plan 99999            ❌
  C 阴性对照（TCP z=-0.20 桌面下）：plan 99999   ✅ 场景确实在拦

根因（A 的失败）
  /check_state_validity 显示：env_voxel_slot_table_* 与 camera_link 相交 8–20 mm。
  camera_link 是【厂商占位盒】（vendor cs625.urdf.xacro："Camera positioned 5cm in front
  of the EEF"）——抓取姿态下这个假相机盒伸到 TCP 前方 5 cm（桌面带内），而真机相机
  挂在工具体上（T_flange_cam 实测，TCP 后方 ~250 mm）。即：是厂商占位几何导致的假碰撞。

决策落地（所有者已批：接受分段启用退路）
  · 抓取段（4b）：只清空场景（--remove），靠离线包络门 + 分段 + verify 保护
  · 转运段（9b，抬到安全高度后）：apply --keep-workpiece（工件已离开桌面，不再减除）
  · 遗留选项（未动）：把 real_base.launch 的描述切回仓库 wrapper
    cs625_active_perception.urdf.xacro（启动默认值，含实测相机外参与 SolidWorks 工具体），
    可消除 vendor camera_link 假碰撞——需所有者确认后再改。
```

## 9. 定向修模型 + 验收通过（2026-11-02 夜，回答"能不能达到师兄效果"）

所有者质疑 wrapper 描述的准确性后，先逐行对比（见正文第 1 节结论），再按
"wrapper 好零件 + 去掉已知错误"定向修，最终**全场景验收通过**。

```text
改动（src/cs625_ap_description/）
1. cs625_parallel_gripper.xacro
   · 左右手指 link 的 <collision> 移除（保留视觉）——77 mm 占位实体不进规划
2. 同一文件：Gazebo 夹爪 ros2_control 块改为 sim 标志门控
   （此前无条件内嵌 gz_ros2_control/GazeboSimSystem，真机驱动加载即崩溃——
   这就是 10-26 交接改用 vendor 描述的真正原因）
3. cs625_active_perception.urdf.xacro：调用处传 sim 标志

验证
  xacro 真机模式展开：GazeboSimSystem = 0（驱动不再崩）
  xacro 仿真模式展开：GazeboSimSystem = 1（仿真不受影响）
  重建 + 重启栈加载 wrapper → 驱动存活、四项门全绿

验收（场景=1476 桌面体素盒 + 新模型）
  pre_grasp  规划 code 1，6 点    ✅
  grasp      规划 code 1，77 点   ✅（昨天 NO_COLLISION_FREE_IK）
  lift       规划 code 1，66 点   ✅（昨天 99999）
  阴性对照（TCP z=-0.20 桌面下）99999  ✅ 场景仍在拦

结论
  昨天失败的唯一元凶 = 厂商 camera_link 占位盒（EEF 前 5 cm）。
  wrapper 相机（实测手眼，与 RVS DepthToRobotTCP 四位小数一致）替代后全绿。
  teaching_sequence 改回【全场景启用】：4b 用新鲜 S3 减除工件后灌场景；
  9b 分段启用块删除。携带段仍由离线 carried-box 门保护（专项 2 未落地）。
```

## 10. 遗留与后续

```text
· 本专项收尾：环境场景全流程启用（真机执行前仍需按验证阶梯复核一次）
· 专项 2（携带工件建模）：抓取成功后把工件 attach 进规划场景（参照
  collision_object_publisher / attached_box_visual_publisher）
· 专项 3（力控装配）：插入段力反馈（参照 cs625_tcp_force_probe /
  cs625_compliant_placement）
```

## 11. 专项 2（携带工件建模）实施记录（2026-11-02 夜）

参照师兄 `collision_object_publisher` / `attached_box_visual_publisher`，做动作式
实现（不引入事件总线）：

```text
新增 scripts/attach_workpiece.py
  --attach   工件网格(shielding_module)以 inv(T_b_tcp) 位姿 attach 到 my_end_effector_link
             touch_links=[my_end_effector_link]（与师兄约定一致，位姿由 pose_math 单一来源）
  --detach [--place-xyz/--place-quat]  卸下；可选把工件按 T_world_box=T_world_tcp@inv(T_b_tcp)
             放回世界场景（释放位=插入整平 TCP）
  --reset    同时清掉 attached 与世界里的两份副本（含中断残留）
  --dry-run
新增 test/test_attach_workpiece.py  3 个纯逻辑测试（attach 位姿=inv(T_b_tcp)、
  释放位姿合成、与 pose_math 的抓取往返一致）
teaching_sequence.py 接入：步骤 7 抓住后 --attach（失败 WARN，离线携带门兜底）；
  步骤 16 释放前 --detach --place（插入整平位姿）

实机验证发现并修掉一个 MoveIt 陷阱：
  · ApplyPlanningScene 的 diff 里，attached REMOVE 必须同时置
    scene.robot_state.is_diff = True —— 否则 detach 静默不生效
    （服务返回 success 但对象仍在）。
  · 对不存在 id 的 REMOVE 会让服务返回 success=False 并可能拒绝整个 diff
    —— 改为先查场景、只对存在的 id 发 REMOVE。
  · 中断残留：attached id 可能以世界对象形式残留 —— reset/attach/detach
    一律同时清 world 与 attached 两份。
实机往返验证（不动臂）：attach→attached=1；detach+place→world=[shielding_module_placed]；
reset→attached=0、world=0。16 个测试全过。
```
