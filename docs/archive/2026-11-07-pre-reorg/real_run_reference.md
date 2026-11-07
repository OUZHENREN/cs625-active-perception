# 真机轨迹记录索引（仿真的唯一基准）

> 真机在 **2026-10-22 已经完整跑通过一次全流程**，每一步的关节轨迹都被录下来了。
> 仿真里遇到问题时，**第一件事是拿这些记录对比**，而不是重新规划、重新猜参数。
> 这些记录的速度慢、但**动作与顺序是对的**（所有者确认）。
>
> 位置：`~/cs625_legacy_snapshot_20261020/runs/<run>/`
> 工具：`python3 scripts/replay_real_run.py --list`（49 条轨迹）／`--run <run>` 原样回放

## 1. 完整流程那次的记录（`20261022_full_task_e2e/meta.json`）

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

1. **`slot collision removed during insertion`** —— 真机做插入时，插槽碰撞体是**从规划场景里去掉**的。
   这解释了为什么我们的 `NO_COLLISION_CHECK_PHASES` 是对的，不是"绕过检查"。
2. **`workpiece collision via senior attach`** —— 抓取后工件是**以 attach 的方式跟随工具**，
   不是留在场景里当碰撞体。所以抓取之后的规划根本不该再和工件做碰撞检查。
3. **`controller recovery required after goal_time aborts`** —— 真机也出现过 `goal_time` 中止，
   需要恢复控制器。所以仿真里出现 `GOAL_TOLERANCE_VIOLATED` 本身不算异常，
   **异常的是"物理上过不去"（力矩饱和、RTF 崩塌）**。

## 2. 按阶段可用的轨迹（`--list` 全量 49 条，这里只列主干）

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

**共同点**：这些轨迹的 `wrist_2` 全都在 **1.53~1.62 rad（≈90°）**，也就是真机长期工作在球腕奇异位形上——
仿真里"奇异导致跟踪失败"的说法因此站不住脚，真正的问题是物理模型（见 `docs/sim_fidelity.md` §3.2）。

## 3. 用法

```bash
# 看有哪些
python3 scripts/replay_real_run.py --list

# 原样回放真机的 pre_grasp（仿真里慢一点更稳）
ROS_DOMAIN_ID=72 python3 scripts/replay_real_run.py --run 20261022_pre_grasp_exec --speed 1.5

# 回放一段完整主干（按顺序给文件）
ROS_DOMAIN_ID=72 python3 scripts/replay_real_run.py --files \
  20261022_pre_grasp_exec/pre_grasp_plan.json \
  20261022_grasp_joint_partial/grasp_joint_plan.json \
  20261021_retreat_from_grasp/retreat_plan.json \
  20261022_transit_selection/transit_from_grasp.json \
  20261022_transit_selection/preinsert_from_transit.json
```

判定标准：**每条都 `ALL_ASSEMBLY_CHUNKS_OK`、末端误差 ≤0.05 rad、RTF 不塌**。
任何一条在仿真里过不去，都是仿真模型的问题（而不是任务定义的问题）。
