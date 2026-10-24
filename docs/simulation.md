# 仿真：与真机 1:1 对齐的场景

> 2026-10-24 起，仿真不再是"另画一个世界"，而是**从真机实测数据生成**。
> 旧的、手工搭建的仿真世界全部归档，不再使用。

## 1. 为什么要重做

旧仿真世界（`cs625_insertion_scene.sdf`）与真机完全不是同一个场景：

| | 真机实测 | 旧仿真世界 |
|---|---|---|
| 工件 | (0.503, 0.635)，桌面 z=−0.091 | (−0.510, −0.810)，**没有桌面模型** |
| 插槽 | (1.242, 0.182)，夹具底面 z=−0.041 | (−0.850, 0.490)，悬空 z=0.273 |
| 方位差 | — | 工件差 ~174°、插槽差 ~142°（几乎在对面） |

在那种世界里"规划通过"对真机没有任何意义。所以：**几何由实测生成，工件位姿允许变化，其余对齐真机。**

## 2. 场景怎么生成

```text
runtime/environment/<ts>/model.json      两张桌子的 z 与 xy 范围（点云拟合）
config/tasks/common.yaml                 夹具位姿 slot_origin(+q)、工件位姿、手眼、内参
        │
        ▼  scripts/build_sim_world.py
src/cs625_simulation/worlds/cs625_real_cell.sdf        地面 + 两张实测桌面 + 夹具 + 工件
config/cs625_real_cell_scene.yaml                      生成的位姿（勿手改）
        │
        ▼  scripts/apply_real_cell_scene.py
MoveIt 规划场景里的碰撞体（默认只发夹具；桌子与工件按需 --include-tables / --include-module）
```

重新生成（真机换了桌子/挪了夹具之后必须重跑）：

```bash
source scripts/source_dev_env.sh --full
python3 scripts/model_environment.py --dataset ~/cs625_yolo_seg/datasets/2026-10-25_raw \
    --out runtime/environment
python3 scripts/build_sim_world.py \
    --out-world src/cs625_simulation/worlds/cs625_real_cell.sdf \
    --out-config config/cs625_real_cell_scene.yaml \
    --report /tmp/sim_fit_report.json --figure /tmp/sim_world_topview.png
```

生成器会打印一张拟合表：每个模型的**包围盒中心**对实测参考点的偏差。偏差 >20 mm 会显式警告。

## 3. 起仿真

```bash
source scripts/source_dev_env.sh --full
ros2 launch cs625_bringup sim_cell.launch.py            # 默认带 RViz
ros2 launch cs625_bringup sim_cell.launch.py headless:=true launch_rviz:=false
```

它启动：Gazebo（新世界）+ ros2_control + MoveIt + RViz。**不自动跑任务**，只打印下一条该跑的命令。
控制器在负载高时可能 60–90 s 才齐全，等 `/joint_trajectory_controller/follow_joint_trajectory`
出现再跑任务。

跑六类模式（不动真机）：

```bash
python3 scripts/run_task.py --scope load --mode safe --world sim --dry-run
python3 scripts/run_task.py --scope load --mode safe --world sim --confirm
```

`--world sim` 只做一件事：把控制动作名从 `/arm_controller/follow_joint_trajectory` 换成
`/joint_trajectory_controller/follow_joint_trajectory`（见 `config/tasks/world_sim.yaml`）。
位姿、阶段、速度与真机**完全相同**——这正是"1:1"的含义。

## 4. 为什么默认不把桌子放进碰撞场景

- 桌面高度是**厘米级**精度（帧间位姿残差 ±2~3 cm），而抓取姿态距桌面只有 3 cm；把桌子当实心盒子会
  产生"假碰撞"，把真机上能做的抓取判成不可行；
- 真机的既有做法（师兄 `collision_object_publisher`）也只发插槽夹具 + 抓取后的工件；
- 因此默认只发夹具；需要验证桌面碰撞时用 `--include-tables`，需要验证工件避让时用
  `--include-module`（注意：夹爪必须能碰到工件才能抓，所以工件作为实心体时抓取必然不可行）。

## 4.1 槽内紧间隙阶段的碰撞策略（重要）

真机插槽的单边间隙约 **1 mm**，比碰撞网格的误差还小。把真机网格放进规划场景后，
`insert` / `straighten` / `d2c` / `d4` / `d5_extract` / `d2b_entrance_tilted`
如果照常做碰撞检查，会**全部被判成碰撞**——但真机每天都能做这些动作，因为槽壁本身在导向。

因此 `scripts/run_pick_and_place.py` 把两类规则分开：

```text
TIGHT_CLEARANCE_PHASES      速度上限 0.005 m/s（哪些阶段要慢）
NO_COLLISION_CHECK_PHASES   = 上面这些 + d2b_entrance_tilted（哪些阶段不做碰撞检查）
```

- 只有这些阶段关闭碰撞检查（`plan_cartesian(avoid_collisions=False)`），其余阶段一律开启；
- `d2b` 进槽口时是空夹爪、已证明可用 0.03 m/s，所以它只关碰撞检查、不降速；
- 这条策略必须写进报告：仿真里"插入可行"是**几何导向下**的可行，不是"规划和碰撞检查都通过"。

## 5. 已知差异与限制

| 项目 | 现状 |
|---|---|
| 夹具几何 | 已换成真机 RVS 几何（`assets/cs625_real_cell/slot_fixture`，60k 面、0.2×0.745×0.508 m），模型原点 = `slot_origin`（RVS 标定语义）。旧仿真夹具（`cs625_task/slot_fixture`）保留但不再使用 |
| 相机 | 仿真相机走 URDF 的 eye-in-hand 宏；内参按实测 fx/fy 反算视场 |
| 桌面 | 两张实测桌面的盒子在 xy 上有 0.144 m² 重叠，重叠区两个静态盒子叠放 |
| 工件 | 直接使用实测 CAD 位姿；其包围盒底面比工件桌顶面低 14.8 mm（模型系与 CAD 系差异），
若要在 Gazebo 里当动力学体生成需另抬 14.8 mm |
| 力矩/电流 | 仿真机械臂没有 F/T 传感器，`wrench` 与 effort 列为空（实验摘要写"未采集"） |

## 6. 旧世界在哪

```text
~/cs625_legacy_sim_snapshot_20261024/     旧 SDF / 旧场景 yaml / 旧 P4-P7 位姿（只读 + SHA256SUMS）
docs/archive/2026-10-24-pre-six-modes/    旧仿真说明文档
```

新代码不得 launch 或 import 这些文件。它们只用于复盘与对照。
