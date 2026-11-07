# 2026-10-26 WORKLOG — 真机开工：把"起不来"变成看得见的失败（启动链重构第一段）

> 接 `2026-10-26_HANDOFF_REAL_RETEST.md`。所有者把硬件全部上电后要求"检查任务逻辑漏洞并重构"。
> 本轮**启动了真机驱动 + MoveIt、激活了 arm_controller、在真机栈上跑了 3 个阶段的 plan-only**；
> **没有发过任何轨迹，机械臂没有动**。Gazebo 未启动。

## 1. 一句话结果

任务"起不来"不是单一 bug，而是**三层叠加**：① 这台机器上根本没有 ROS 栈（已由本轮补起驱动+MoveIt）；
② `control_health.py` 把 `inactive` 判成 active，导致"控制器没激活"这道门永远通过；③
`run_pick_and_place.py` 在 MoveIt 服务上 `while not wait: pass` 死等、不打印，冷启动看起来就是"没反应"。
三者都已修好并实测：现在缺 MoveIt 时 21 s 内打印缺哪三个服务并非零退出；控制器没激活时健康检查立刻报错
并打印激活命令。

## 2. 运行时事实（本轮实测）

```text
机器人 192.168.1.200   ping 通；TCP 30001/30004 开放
相机   192.168.1.120   ping 通，但 ROS 侧没有 percipio_camera 节点
夹爪   PowerShell 桥 C:\Users\Public\hitbot_cmd.ps1 存在；status → init=5, position_mm=0
驱动   ros2 launch ... real_base.launch.py（launch_driver:=true）→ dashboard "Connect to robot success"
MoveIt real_moveit.launch.py → "You can start planning now!"
控制器 arm_controller 原本 inactive → 本轮 switch_controllers --activate 成功
四项   safety_mode=1 / robot_mode=7 / arm_controller active / control_health 三 True
点云   /sensors/camera/points 存在但 Publisher count = 0（只有 move_group 在订阅）
场景   /get_planning_scene → world.collision_objects = []（夹具/桌面从未加进规划场景）
plan-only  observe:workpiece_coarse / baseline / pre_grasp 三段 ok=True，exit 0
```

## 3. 改了什么（按"先让失败可见，再谈冗余"）

| 文件 | 改动 | 为什么 |
|---|---|---|
| `scripts/control_health.py` | 控制器状态改成按列精确比较（`controller_state()`）；不再用 `"active" in line`；控制器管理器/控制器名走 `CS625_*` 环境变量（默认不变）；恢复脚本改成绝对路径 + 保留输出 | **实测**：`arm_controller inactive` 时旧代码仍返回 `controller: True`、exit 0，于是 runner 把目标发给一个不能执行的控制器。这是"任务起不来"最隐蔽的一层 |
| `scripts/run_pick_and_place.py` | ① MoveIt 四个服务改成 20 s 上限，超时列出缺哪几个并 `SystemExit`；② `ensure_control_ready` 增加 `safety_mode==1 && robot_mode==7`（AGENTS 四项只查过两项）；③ load 的 `gripper_init`/`grip` 改成等 `init==5` / `grip_and_wait`；④ load 的 `release` 加 `--allow-release` 门槛并等 `init==5`；⑤ `verify()` 超差现在把阶段置 `ok=False` 并中止流程；⑥ unload 分支补 `node.world=ucfg.world`（仿真 unload 之前会真的去调真机夹爪桥）；⑦ `chunk_count()` 修正分段下标（3 点 6 段以前会重放运动）；⑧ 子进程路径改绝对；⑨ 删除死代码 `_ik_legacy`/`validity`/`skipped` 与无用的 `/check_state_validity` 客户端；⑩ `--phases` 拼错直接报错；⑪ 空记录不再算成功 | 每条都有具体失败模式（静默挂起 / 假通过 / 危险动作序列） |
| `scripts/run_task.py` | `--dry-run` 门禁 REJECT 时返回 2（与 AGENTS/交接一致）；打印模型 `age`；phase runner 用 `cwd=REPO` 启动 | 交接 §9 第 3 步就是"dry-run 看门禁"，以前永远 exit 0 |
| `src/cs625_bringup/launch/real_base.launch.py` | `launch_moveit` 默认 `true → false` | 文档让人另开终端起 MoveIt，默认值又自动起一份；实测 `/move_group` 出现 4 个同名节点，规划结果取决于谁应答 |
| `test/test_task_runner_safety.py` | 新增 8 个纯 Python 回归测试 | 锁住本轮修的判据（inactive/active、chunk 下标、chunks 校验、joint 容差、grip_current、dry-run 退出码） |

## 4. 验证（命令与结果）

```bash
python3 -m pytest test/ -q          # 238 passed（230 + 新增 8）
python3 test/contract_checks.py    # Phase 0–6 PASS

# ① 控制器没激活时健康检查必须失败（旧代码通过）
python3 scripts/control_health.py
# health: {'sockets': True, 'controller': False, 'joint_states': True}
#   arm_controller is not active (...); activate it with: ros2 control switch_controllers ...
# exit=1
ros2 control switch_controllers --controller-manager /eli_ros2_control_node \
  --activate arm_controller --strict          # Successfully switched controllers!
python3 scripts/control_health.py             # 三 True，exit=0

# ② 没有 MoveIt 时必须在有限时间内报缺什么（旧代码永远挂着、零输出）
ROS_DOMAIN_ID=99 timeout 45 python3 scripts/run_pick_and_place.py \
  --config runtime/effective_load_efficient.yaml --plan-only --log /tmp/plan.json
# MoveIt services are not available: /compute_ik, /plan_kinematic_path, /compute_cartesian_path
#   waited 20 s. ... Nothing was commanded; fix the stack and re-run.
# exit=1，耗时 21.2 s

# ③ 门禁 REJECT 时 dry-run 必须非零（旧代码 exit 0）
python3 scripts/run_task.py --scope load --mode efficient --world real --dry-run
# environment gate : REJECT（模型 6.76 h）
# dry run gate verdict: REJECT -- a real run would refuse here (exit code 2)
# exit=2

# ④ 真机栈上的规划链路（不动臂）
python3 scripts/run_pick_and_place.py --config runtime/effective_load_efficient.yaml \
  --plan-only --phases "observe:workpiece_coarse,baseline,pre_grasp" --log /tmp/plan_real.json
# 三段 ok=True，exit=0
```

## 5. 仍然挡着"一次完整真机任务"的三件事（本轮未做，已定位）

1. **环境模型过期且"新鲜度"判据是构建时间**：`runtime/environment/latest` = 10-26 03:00，已 6.8 h；
   而 `model_environment.py` 的 `created_at` 是"生成模型的时间"，默认 `--dataset` 还写死
   `2026-10-25_raw`。也就是说"用上周点云重建一次"就能把门禁刷成 PASS。真正合规的做法是在现场
   采当天数据（需要臂按观测位姿运动，属真机动作，需所有者当次授权）。
2. **规划场景从未应用**：`/get_planning_scene` 的 `collision_objects` 是空的；`apply_real_cell_scene.py`
   默认只发 1 个夹具、不等确认、没 move_group 也打印成功。所以刚才的 `--plan-only ok=True`
   只证明"在空世界里可达"，不证明"避开了桌子和夹具"。
3. **相机归一化链路没起**：`real_base.launch.py` 的 `launch_sensor_adapter` 默认 false，且
   `src/cs625_bringup/config/real.yaml` 的四个 `input_*_topic` 还是空字符串；`/sensors/camera/points`
   目前 publisher=0。没有它，observe 阶段的 YOLO/ICP 检查不可能通过。

## 6. Capability layer

```
- environment and dependencies      PASS      ROS Jazzy 链 --verify 全绿；真驱动连接成功
- robot model and simulation        PARTIAL   真机描述/驱动 OK；仿真 insert 仍未通过（见 10-26 复盘）
- kinematics, control and planning  PASS(真机) plan-only 3 段成功；完整 19 段未跑（门禁+场景未就绪）
- vision, hand-eye calibration/TF   BLOCKED   相机 ROS 驱动未起；归一化话题 publisher=0
- active perception / NBV           NOT STARTED
- real-hardware integration/safety  PARTIAL   四项健康检查齐了；真机执行仍未开始（模型/场景/相机未就绪）
```

## 7. Acceptance target

- 目标：真机 `--confirm` 之前，所有"不满足前提"的情况都必须**立刻、指名**失败。
- 达成：控制器未激活、MoveIt 未起、门禁 REJECT、`--phases` 拼错、夹爪 init 未完成 —— 五种情况现在都会
  以非零退出并打印原因（各有命令证据）。
- 未达成：一次完整真机 load（缺第 5 节的模型/场景/相机）。
- 用了真机：是（驱动连接、控制器激活、plan-only 规划）。**真机运动：否。** Gazebo：否。

## 8. Critical-chain status

```
URDF -> Gazebo entity              PASS(仿真) 厂商 CAD 实体（10-26 早）
ros2_control -> joint_states        PASS       真机驱动 joint_state_broadcaster active
base_link -> camera optical TF      BLOCKED    真机树没有 camera_*_optical_frame（厂商描述缺）
RGB-D -> normalized sensor topics   BLOCKED    /sensors/camera/points publisher=0
point cloud -> MoveIt planning scene NOT STARTED collision_objects=[]
NBV decision -> robot execution     NOT STARTED
```

## 9. Sim/real alignment

```
共用应用核心     是（本轮改动同时服务两边；仿真路径未改变判据）
仿真入口        sim_cell.launch.py（未启动）
真机入口        real_base.launch.py + real_moveit.launch.py（本轮实测可用）
真机执行        未允许（模型/场景/相机未就绪）
门禁状态        REJECT（模型 6.8 h，需现场重测）
```

## 10. Intentionally not done

- 未改 `config/tasks/*.yaml` 的位姿/速度；未改厂商包。
- 未合并 `Config`/`UnloadConfig` 的重复加载与 `run_unload`/load 的重复执行体（第二轮重构，需要专门测试）。
- 未修 unload 的 `d3d_pull_check` 恒真、`d5_extract` 无单步进展检查、纯旋转阶段被当"已到位"（审计 #8/#9/#26）。
- 未处理 `real_base` 里 `force_torque_sensor_broadcaster` 必然激活失败（该机器人没有力传感器，属厂商 launch 的固定噪声）。
- 未把夹爪桥路径 `C:\Users\Public\hitbot_cmd.ps1` 移进配置（PS1 不在仓库）。
- 未新增任何"gate/contract/preflight"脚本：本轮只修既有判据与错误路径。

## 11. 下一轮唯一推荐任务

**按下面顺序补齐真机最后一个缺口，然后再谈 `--confirm`**：
① `python3 scripts/apply_real_cell_scene.py`（把夹具/桌面加进规划场景，`/get_planning_scene` 能看到）；
② 起相机（`python3 scripts/camera_mode.py fast`）并把实测的 percipio 话题名填进
`src/cs625_bringup/config/real.yaml`，用 `launch_sensor_adapter:=true` 起适配器，确认
`/sensors/camera/points` 有 publisher 且 3 s 内有帧；
③ 现场重采环境模型（需所有者授权运动），`--dry-run` 应为 exit 0；
④ 再 `--plan-only` 跑完整 19 段，最后才 `--confirm`。
