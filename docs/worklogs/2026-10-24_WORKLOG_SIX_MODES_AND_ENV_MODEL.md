# 2026-10-24 WORKLOG — 六类模式落地、强制环境建模、全流程只规划验证（机器臂未动）

> 本轮目标（所有者当日指定）：**先把"不需要移动真机机械臂"的事全部做完** —— 只用相机与规划。
> 结果：六类模式（3 scope × 2 mode）全部落地并**只规划跑通**；环境建模与门禁可用；
> 效率模式取消分段慢速；真机未被移动一次（有证据）。阻塞项只有一条：**机器人网络不可达**。

---

## 1. 一句话现状

```text
六类模式     load/unload/full × safe/efficient 全部 dry-run + plan-only 通过（共 51 段/全流程）
臂是否动过   没有。plan-only 全程 joint_states 保持 [0,0,0,0,0,0]（fake cell），未发任何 action goal
环境模型     runtime/environment/20261024_030734/model.json（93 帧、612k 点、3222 体素、门禁四种拒绝已验证）
效率模式     取消分段（chunks=1）+ 取消 dwell + 巡航 0.12；规划时长 load 从 77.9 s 降到 49.1 s（63%）
阻塞         机器人 192.168.1.200 No route to host（相机 192.168.1.120 正常）→ 真机段全部暂缓
```

---

## 2. 本轮新建/修改的文件

| 文件 | 状态 | 作用 |
|---|---|---|
| `config/tasks/common.yaml` | 新增 | 世界常量 + 全部可复用位姿（唯一权威来源）+ `environment` 门禁阈值 + `verification` 验收阈值 |
| `config/tasks/{load,unload,full}_{safe,efficient}.yaml` | 新增 6 份 | 互相独立的六类模式：阶段表 + 逐阶段速度档案 + 释放/确认策略 |
| `scripts/run_task.py` | 新增 | 单一入口 `--scope {load|unload|full} --mode {safe|efficient}`；合并配置 → 加载环境模型 → 过门禁 → 调 runner |
| `scripts/model_environment.py` | 新增 | 离线环境建模（复用 `build_scene_model.py` 的点云/平面函数）+ `evaluate_gate()` 纯函数门禁 |
| `scripts/run_pick_and_place.py` | 修改 | 补 4 个洞（见 §4）+ `--plan-only` + `--start-joints` + 失败原因可见 |
| `test/test_task_modes.py` | 新增 | 15 个离线单测（模式解析 / 配置校验 / 门禁四种拒绝 / plan-only 不发 goal） |
| `docs/source_authority.md` | 新增 | 源码与参数的权威性分级（A/B/C）+ 速度"证明值"表 |
| `docs/interfaces.md` | 追加 §8 | 任务层入口的数据流（不进 ROS 图，不加新接口） |
| `.gitignore` | 修改 | 追加 `runtime/`（生成物不入库） |

**没有新增 ROS 包，没有改动 11 个包的边界，没有删除任何源码。**

---

## 3. 六类模式：只规划验证结果（本轮核心证据）

命令模板（不需要机器人；`--start-joints` 用精观测位实测关节，chaîne 从真实位姿开始）：

```bash
python3 scripts/run_task.py --scope <scope> --mode <mode> --plan-only \
  --start-joints 0.5597,-0.8861,-2.6480,0.0049,1.8827,1.4482
```

| scope | mode | 结果 | 规划运动时长 | 说明 |
|---|---|---|---:|---|
| load | safe | 13/13 段规划成功 | 77.9 s | 分段 3~7、dwell 1.0、time_scale 2.0 |
| load | efficient | 13/13 | **49.1 s** | 单段、无 dwell、巡航 0.12 → 安全模式的 63% |
| unload | safe | 38/38 | 288.8 s | 槽内 0.005 m/s 主导 |
| unload | efficient | 38/38 | 285.6 s | **几乎无差别**（见 §5 解释） |
| full | safe | 51/51 | 365.6 s | load + unload |
| full | efficient | 51/51 | 334.6 s | load + unload |

> 表里的秒数是**规划器给出的轨迹时长**（not executed）。safe 模式执行时还要乘 `time_scale 2.0`
> 并加 dwell，所以真机耗时约为表值的 2 倍；efficient 模式 `time_scale 1.0`、无 dwell，表值≈实际。

**"臂没动"的证据**：plan-only 结束后 `/joint_states` 仍是 `[0,0,0,0,0,0]`（本轮用 fake hardware 起控制栈，
初始值全 0，没有任何命令写入）；全程只调用了 MoveIt 的四个规划服务
（`/plan_kinematic_path`、`/compute_ik`、`/compute_cartesian_path`、`/compute_fk`），没有向
`/arm_controller/follow_joint_trajectory` 发过 goal。

**为什么能用 fake hardware 跑**：机器人不可达，但规划只需要"机器人描述 + 一个起始状态"。
`--start-joints` 让 plan-only 直接以真实位姿的关节角为起点、逐段串联（每段以上一段的规划终点为
`start_state`），因此**这条验证路径完全不依赖机器人是否在线**。

---

## 4. 给 runner 补的 4 个洞（把经验真正写进代码）

| 洞 | 原来 | 现在 |
|---|---|---|
| `pre_grasp` / `grasp` 忽略阶段速度档案 | 写死 `velocity_scale` / `assembly_velocity_scale` 和 6/4 段 | 走 `profile_for()`，与其它阶段一致 |
| 速度上限是全局常量，逐阶段速度不校验 | 只查两个全局标量 → 档案里可以写任意速度 | 上限按模式（safe 0.08 / efficient 0.12，装配 0.03），**逐阶段**校验；`load_unload_config` 也走同一套 |
| `full` 衔接点未定义 | 无 | `run_task.py` 定义 load→unload，默认 `--handover confirm` 停下等人工确认 |
| 阶段验收不统一 | 只有人工跑 `verify_pose.py` | 装配相关阶段后自动调 `verify_pose.py`（阈值来自 `verification` 段） |
| （附）失败原因不可见 | 只打印 `ok=False` | 打印 `IK_FAILED` / `PLAN_FAILED` / `CARTESIAN_PLAN_FAILED` / `points=N dur=Xs` |

**槽内紧间隙规则**（写死，任何模式都不放宽）：
`insert / straighten / d2c_straighten / d4_tilt_for_removal / d5_extract` 速度 ≤ **0.005 m/s**，
且禁止"快巡航 + 慢尾段"复合曲线。`d2b_entrance_tilted`（空夹爪进槽口）**不算紧间隙**——
它在真机上以 0.03 跑过，校验器不能比实践更严。

---

## 5. 为什么 unload 的"效率模式"几乎没有提速（重要结论）

| 段 | 距离 | 速度 | 时长 |
|---|---:|---:|---:|
| d2b_entrance_tilted | 350 mm | 0.005（紧间隙，两模式相同） | 70 s |
| d5_extract ×18 步 | 350 mm | 0.005（紧间隙，两模式相同） | 70 s |
| d2c / d4 转动 | 9.3° ×2 | 0.005 | ≈6 s |

unload 的绝大部分时间花在**槽内**，而槽内速度被安全规则钉死在 0.005 —— 这正是
"单边间隙约 1 mm"的物理约束，不是调度策略问题。**要缩短 unload，只能改机械/工装间隙或改抓取方式，
不能靠调速度。** 效率模式的收益只在 load 与巡航段体现（load: 77.9 → 49.1 s）。

---

## 6. 环境建模与门禁（每次流程前强制）

```text
输入     已有 181 帧数据集里的 93 帧（每帧带 4×4 TCP 矩阵 + 彩色 + 深度）
点云     4.42 M → 612 k（voxel 0.01 + 统计滤波）
工件桌   z = −0.0910 m，xy x[0.260,1.180] y[0.280,1.080]，1450 体素块
插槽桌   z = −0.0407 m，xy x[0.780,1.700] y[−0.280,0.640]，1772 体素块
插槽     xyz [1.241629, 0.181709, 0.443245]  quat [−0.001629, 0.004253, 0.003384, 0.999984]
工件     xyz [0.5029, 0.6351, 0.1254]  quat [0.1910, 0.9816, 0.0019, −0.0068]，中心 [0.7206, 0.6182, 0.1335]
门禁     插槽 >1 mm/>0.5°、工件 >3 mm/>1°、点数 <20 k、模型 >2 h → 拒绝（四种都已用假数据验证）
产物     runtime/environment/<ts>/{model.json, scene.ply} + latest 软链 + latest.json
```

已知限制（沿用交付说明）：桌面 z 是**厘米级**（帧间位姿残差 ±2~3 cm，存在"幽灵台面"）；
`slot_table` 取的是插槽原点下方 0.48 m 的工作台面，不是工装台板；`scene.ply` 无颜色。
这些不影响门禁判断（门禁用的是位姿偏差与点数），但**不能当作毫米级碰撞体直接发布**。

---

## 7. 流畅性排查（按所有者给的清单逐条核对，均未动机械臂）

| 检查项 | 结论 | 证据 |
|---|---|---|
| 时间参数化是 IPTP 还是 TOTG | **已经是 TOTG**，不需要切换 | `ros2 param get /move_group ompl.response_adapters` → `AddTimeOptimalParameterization` |
| Jerk 约束 | **没有**。TOTG 只约束速度/加速度，不约束加加速度 | 同上；轨迹里无 jerk 限制项 |
| 笛卡尔步长 | 3 mm（建议 5~10 mm），比建议更细 | `scripts/run_pick_and_place.py` `req.max_step = 0.003` |
| 控制环频率 | 125 Hz（与 RTDE 一致） | 驱动启动日志 `update rate is 125 Hz` |
| 实时内核 | **没有**（WSL2 `6.18.33.2-microsoft-standard-WSL2`，PREEMPT_DYNAMIC） | `uname -a` |
| FIFO 实时调度 | **被拒绝**：`Could not enable FIFO RT scheduling policy: Operation not permitted` | 驱动启动日志 |
| 循环抖动测量工具 | `cyclictest` **未安装** | `which cyclictest` → 空 |
| 末端负载/质心/惯量 | **URDF 里是占位值**（相机 0.01 kg、机身 0.05 kg、手指 0.001 kg），质心全 0 | `src/cs625_ap_description/urdf/*.xacro` |
| 底座刚性/线束拖拽 | 需要现场检查，软件侧无法判断 | — |

**结论**：最可疑的两项是 ①无实时内核 + FIFO 调度被拒（125 Hz 环路的抖动源），
②URDF 末端负载/质心是占位值（重力补偿会偏）。这两项都不是"改 MoveIt 参数"能解决的。
本轮不擅自改这些（会改变真机动力学行为），等所有者决定。

---

## 8. 源码保留与权威性

- 新建 `docs/source_authority.md`：A 级（2026-10-22~24 真机实测）/ B 级（师兄接口）/ C 级（旧仿真参数，仅保留）
- 师兄代码 `~/cs625_legacy_underlay_jazzy` 未改一字；旧仿真参数一条未删
- `config/pick_place_real.yaml` / `config/unload_real.yaml` 保持原样（mtime 仍为 2026-10-23 12:39），作为已验证基线并存
- 归档/移动/删除任何文档仍需所有者书面批准

---

## 9. Capability layer

```text
- environment and dependencies      PASS    相机 0.8 FPS；伪硬件控制栈 125 Hz；门禁与建模脚本离线可跑
- robot model and simulation        PASS    URDF/RSP/规划场景可用；fake hardware 可替代机器人做规划验证
- kinematics, control and planning  PASS    51 段全流程规划成功（同一起始位姿，串行 start_state 链）
- vision, hand-eye calibration/TF   PASS    （上一轮）精观测位 3 帧共识 1.0 mm
- active perception / NBV           NOT STARTED  前置链路已验收
- real-hardware integration/safety  BLOCKED  机器人 No route to host；真机执行段全部暂缓
```

### Acceptance target（本轮）

```text
objective        在不移动机械臂的前提下，把六类模式与环境建模全部落地并验证
pass criteria    ① 6 组合 dry-run 全部通过 ② 6 组合 plan-only 全段规划成功 ③ 门禁四种拒绝成立 ④ 臂未动
runtime evidence 上表 6 行 plan-only 结果；joint_states 保持 [0,0,0,0,0,0]；
                 evaluate_gate 四种假模型均 REJECT；pytest 15 passed
verdict          PASS
```

### Critical-chain status

```text
URDF -> Gazebo entity               NOT APPLICABLE（本轮真机/伪硬件）
ros2_control -> joint_states        PASS（fake hardware 125 Hz）
base_link -> camera optical TF      PASS（上一轮验证）
RGB-D -> normalized sensor topics   PASS（相机在线）
point cloud -> MoveIt planning scene PARTIAL（碰撞体已生成但未发布；见 §6 限制）
NBV decision -> robot execution     BLOCKED（真机不可达）
```

### Intentionally not done

```text
· 真机执行 6 种模式（机器人不可达）
· 把 3222 个体素碰撞体发布进 MoveIt（厘米级精度，先不入场景，避免假碰撞挡住规划）
· 改末端负载/质心、装 RT 内核、装 cyclictest（属于真机动力学与系统层，需所有者决定）
· 动 unload 的槽内 0.005 m/s 规则（物理约束，不是调度问题）
```

---

## 10. 命令速查

```bash
cd /home/oy/projects/cs625-active-perception && source scripts/source_dev_env.sh --full

# 建模（离线，用已有数据集）
python3 scripts/model_environment.py --dataset ~/cs625_yolo_seg/datasets/2026-10-25_raw --out runtime/environment
# 只查门禁
python3 scripts/model_environment.py --out runtime/environment --verify-only

# 六类模式：预演（打印阶段表/速度/模型时间戳，不规划不动作）
python3 scripts/run_task.py --scope load --mode efficient --dry-run

# 六类模式：只规划（不需要机器人；从实测位姿串联）
python3 scripts/run_task.py --scope full --mode efficient --plan-only --handover auto \
  --start-joints 0.5597,-0.8861,-2.6480,0.0049,1.8827,1.4482

# 真机执行（机器人可达后才有意义）
python3 scripts/run_task.py --scope load --mode safe --confirm --allow-release

# 单测
python3 -m pytest test/test_task_modes.py -q      # 15 passed
```

---

## 11. 卡点（需要所有者）

```text
机器人 192.168.1.200 不可达：ping 100% loss，30001/29999 报 No route to host；
相机 192.168.1.120 正常。软件侧无可修之处 —— 需要确认机器人是否上电 / 网线 / IP 是否变化。
恢复后最快的验证顺序：real_base.launch.py → 激活 arm_controller → real_moveit.launch.py
→ control_health.py 三样全 True → 重新建模 → run_task --scope load --mode safe --confirm
```

---

## 12. 纯相机侧检查（本轮最后一项，未动臂）

```text
相机帧率     0.756 FPS（正常区间 0.75~0.8）
一帧检测     1 个实例：slot conf 0.78，box [354,297,929,813]
画面内容     工件（弹仓）立放在木桌上，居中、完整入画（与 10:15 精观测位同一视角）
```

**新发现（需要记录）**：**同一视角、同一物体，10:0x 被判为 `workpiece` 0.70，11:3x 被判为 `slot` 0.78。**
即两类模型在精观测近距视角上存在**类别混淆**。影响：

- 精观测阶段若用"取 workpiece 掩膜"的方式选目标，会拿不到目标（或拿错类）；
- 当前 `icp_pose_once.py` 用的是"最大类实例 + 覆盖率门禁"，还不会直接崩，但目标选择逻辑
  不应假定类别名一定正确；
- 下次真机联调前，建议先用当前视角批量跑 20 帧统计两类混淆率，再决定是补样本还是加"按几何
  （CAD 投影）复核类别"的后处理。

> 备注：`/tmp` 在本轮期间被系统清空过一次（当时的 ROS 栈与 `/tmp/cs625_cal` 一起消失），
> 所以这条检查是临时内联脚本跑的，没有留下仓库内脚本。
