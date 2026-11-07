# 2026-10-26 HANDOFF — 下一段对话：真机自动化重测

> 给「下一段对话」的自包含交接。目标：**在真机上再次跑通 load 流程（最好六模式），
> 记录关键数据，回填仿真场景。** 本文件只讲真机；仿真的失败复盘在
> `2026-10-26_WORKLOG_SIM_REFERENCE_MODEL_AND_ATTACH.md`（续 2）。

---

## 0. 一句话结论（先给）

- 真机流程的**代码与配置没有被本轮仿真工作破坏**：`contract_checks` Phase 0–6 PASS、
  `pytest` 230 passed、`run_task --world real --dry-run` 阶段表正确、控制动作名正确
  （`/arm_controller/follow_joint_trajectory`）、安全默认值正确（`execute=false` /
  `require_confirmation=true`）。
- 真机当前**网络不可达**（`ping 192.168.1.200` 100% 丢包），本轮只能离线核验，**不能真动**。
- 开工前唯一要补的：**在真机现场重新采环境模型**（当前模型已 >2 h，门禁会 REJECT）。

---

## 1. 为什么仿真没做好（客观，见复盘文件）

一句话：**参考模型和坐标系后来都对，但"夹爪抓取"这个动作我从头到尾没有真正实现，
且没有尽早坦白。** 具体三条：

1. 厂商 `eli_cs_robot_description` **没有任何夹爪关节**（全仓库 grep 过，prismatic=0）。
   我所谓的"抓取"一直是运动学替代——先用 `set_pose` 位姿写、后改成 `DetachableJoint`
   刚体焊住，从来不是"两侧张开撑住工件扶手内壁"。
2. 真机夹爪是 **HITBOT EFG80 外撑式**：电机开度 0→50 mm 经连杆变成两条支撑臂**外撑**
   （跨距 204→246.57 mm）**并向前伸出**够到工件扶手。这个"臂怎么随电机开度前伸/外扩"
   的**关节运动学不在任何 ROS 描述里**：厂商 `tool0_vision.STL` 只画了收拢态的两条臂
   （在 TCP 后方 203~349 mm），没有关节、没有连杆参数。所以"物理夹住 19 kg 工件"这件事，
   没有额外输入（真机夹爪连杆参数或实测）就做不出**诚实**的仿真——这是硬缺口，不是我
   再调几个参数能补的。
3. 我反复改参考模型（wrapper↔vendor）、反复"修"一个没坏的东西，把问题越搅越乱；"146 mm
   支架"是我的误判（你说得对，现实没有支架，146 mm 是支撑臂向前伸出的距离，不是缺一块实体）。

**能做 / 不能做的边界**：仿真能诚实验证的是「可达性 + 观测前缀 + 搬运轨迹 + 工件跟随
判据」；不能诚实验证的是「真机夹紧 + 1 mm 间隙插入」——那只能上真机。

---

## 2. 真机开工前状态（离线已核验 ✓）

```text
环境链            source scripts/source_dev_env.sh --verify → full chain intact（ROS Jazzy + underlay + overlay）
参考模型          real_moveit.launch.py 默认已改为 eli_cs_robot_description / cs625.urdf.xacro（与真机一直用的那份一致）
SRDF              已回退本轮的"物理夹爪豁免"残留，回到 10-25 TCP 修正后的状态（合法）
安全默认          real_preflight.launch.py: execute=false, require_confirmation=true
控制动作          真机 /arm_controller/follow_joint_trajectory；仿真 /joint_trajectory_controller/...（world_sim.yaml 只叠加 sim）
测试              230 passed；contract_checks Phase 0–6 PASS
```

**本轮改动对真机路径的净影响 = 零**（`run_pick_and_place.py` 里所有新逻辑都挂在
`cfg.world == "sim"` 下；`goal_joint_tolerance` 真机默认仍是 0.02；`real_moveit` 只改了
description 默认值）。

---

## 3. 真机启动顺序（照抄，来自 10-23 handoff + 10-24 E-stop 复盘）

```bash
cd <仓库根目录>
source scripts/source_dev_env.sh --full
source "$HOME/.cs625_local.env"

# ① 驱动底座（控制器保持 inactive，不会自己动）
ros2 launch cs625_bringup real_base.launch.py launch_driver:=true \
  robot_ip:="${CS625_ROBOT_IP:?}" \
  description_package:=eli_cs_robot_description description_file:=cs625.urdf.xacro \
  headless_mode:=true launch_moveit:=false launch_rviz:=false \
  activate_joint_controller:=false execute:=false require_confirmation:=true

# ② MoveIt（另开终端；要观察场景就把 launch_rviz 改 true）
ros2 launch cs625_bringup real_moveit.launch.py \
  description_package:=eli_cs_robot_description description_file:=cs625.urdf.xacro \
  moveit_config_package:=elite_cs625_moveit_config \
  semantic_package:=cs625_bringup semantic_file:=config/cs625_active_perception.srdf \
  launch_rviz:=true

# ③ 激活 arm_controller（若 inactive）
ros2 control switch_controllers --controller-manager /eli_ros2_control_node \
  --activate arm_controller --strict

# ④ 师兄视觉/碰撞组件（源码在只读工作区）
#   tcp_server  / collision_object_publisher  / cs625_state_monitor

# ⑤ 健康检查：四项全过才许动
python3 scripts/control_health.py            # sockets / controller / joint_states 全 True
ros2 topic echo /io_and_status_controller/safety_mode --once   # 要 1 (NORMAL)
ros2 topic echo /io_and_status_controller/robot_mode   --once   # 要 7 (RUNNING)
```

**四项缺一不可**：`safety_mode=1`、`robot_mode=7`、`arm_controller active`、
`control_health.py` 三 True。E-stop 复盘的血泪教训：**话题上有数据 ≠ 数据是当前的**，
驱动半死时 safety_mode/robot_mode 会停在旧值；急停复位先重启软件栈，不是先动示教器。

---

## 4. 环境模型（强制前置，否则门禁 REJECT）

真机运行前必须在**真机现场**重新采一次环境模型（当前模型已 >2 h）：

```bash
# 先采集（用当天的相机数据，例如 ~/cs625_yolo_seg/datasets/<当天>/）
python3 scripts/model_environment.py --dataset ~/cs625_yolo_seg/datasets/<当天> \
    --out runtime/environment
# 然后每次 run_task 前会打印 model 路径 / age / 偏差，门禁 REJECT 时 exit code = 2
python3 scripts/run_task.py --scope load --mode efficient --world real --dry-run
```

`--dry-run` 只打印裁决不动机械臂；`--plan-only` 只做可达性规划不动；`--confirm` 才真动。

---

## 5. 跑哪些（先 load 后六模式）

```bash
# 先只规划、不真动（验证 MoveIt/IK/场景）
python3 scripts/run_task.py --scope load --mode efficient --world real --plan-only

# 真动（每次都要所有者当次授权；真机默认每段都会实测验收）
python3 scripts/run_task.py --scope load --mode efficient --world real \
    --confirm --allow-release
```

关键位姿（`config/tasks/common.yaml`，已与真机跑通记录对齐）：

```text
pre_grasp [0.7677, 0.4903, -0.0595]     grasp [0.8316, 0.5918, -0.0583]
transit   [0.95,   0.45,   0.55  ]       staging [1.0748, -0.0122, 0.5252]
pre_insert[1.0996,-0.0125, 0.3773](-10°) insert  [1.1574, -0.0132, 0.0321]
baseline  [1.044, -0.011,  0.726 ]
夹持      开度 50 mm + 电流 0.1 A（所有者实测：50 mm 已顶住，52 mm 起 ~1 A 硬顶）
槽位      slot_origin [1.241629, 0.181709, 0.443245]（机械固定）
```

---

## 6. 真机已知坑（照做，别踩）

1. **夹爪**：init ~10 s，必须等 `init==5` 再发下一条；开度显示有归零偏移，以电源面板电流为准；
   保持夹力 = 持续通电会有电流声（正常）；**释放夹爪必须所有者当次授权**。
2. **速度滑块**：10-22 卡住一次是"compliant 节点把速度滑块留在 10%"，恢复 = deactivate/activate
   arm_controller + 速度滑块回 1.0。
3. **工件几何**：工件沿工具 z 轴横向伸出 ~0.41 m、包围盒 114×441×465 mm；靠近插槽桌先横移
   让开 y≤0.40 再下降，否则刮插槽桌触发急停。
4. **碰撞语义**：工件碰撞走师兄 attach（`collision_object_publisher`），校验用
   `check_state_validity`，不要读场景位姿做几何判断（读回来永远是 0）。
5. **大图丢帧**：1280×960 BGR8 ~3.7 MiB 超过 Fast DDS 共享内存段，best-effort 下丢 60–80%；
   要图就把相机切 modeling（1280×960，0.79 fps）或用 fast（640×480 仅彩色 23 fps）。

---

## 7. 真机跑通后要记录的数据（回填仿真用）

每次 `--confirm` 会 `--no-record` 与否都会写 `runtime/phase_log_<scope>.json`；
`--confirm`（不带 --no-record）会自动起 20 Hz recorder 并存 `runtime/experiments/<id>/`。
额外手动记：

```text
① 每个阶段结束的实测 TCP（phase_log 的 tcp 字段）与目标差
② 夹爪：init 后 position_mm、grip 命令开度、电源面板电流、是否真撑住（workpiece_follows）
③ 四个安全量（safety_mode / robot_mode / arm_controller / control_health）
④ 相机外参/内参如果重标定，回填 config/handeye + camera_extrinsics_sim.yaml
⑤ 真机实际夹爪几何（两条支撑臂在开度 0 和 50 时，相对法兰的位置/跨距）——
   这是唯一能补上"仿真物理夹爪"缺口的输入
```

---

## 8. 回填仿真的可行性（诚实）

| 项 | 能回填吗 | 说明 |
|---|---|---|
| 环境模型 / 世界 / 夹具几何 | 能 | `model_environment.py` + `build_sim_world.py` 已是从真机实测生成，重跑即可 |
| 阶段位姿 / 速度 | 能 | `common.yaml` 已与真机一致，真机再验一遍后无需改 |
| 工件跟随 + 入槽判据 | 能（已在 sim 侧实现） | 但依赖"附着"成立 |
| **物理夹爪（撑住 19 kg 工件）** | **只有拿到第 7.⑤ 的夹爪连杆数据才能** | 缺"臂随开度前伸/外扩"的关节运动学，仿真只能做运动学替代 |

**结论**：仿真场景的"几何/位姿/世界"缺陷能修（重跑生成器即可）；"夹爪抓取"缺陷需要真机的
夹爪连杆实测，否则只能用 DetachableJoint 运动学替代，且必须明确标注"不代表物理夹紧"。

---

## 9. 下一段对话建议的开工清单（按序）

1. 开机 + 确认机器人可 ping（`ping 192.168.1.200`）。
2. §3 启动顺序 + §5 的四项健康检查。
3. §4 采当天环境模型 + `--dry-run` 看门禁。
4. `--plan-only` 验一遍 load（19 段全 PLANNED）。
5. `--confirm --allow-release` 真跑 load（所有者现场）。
6. 记录 §7 数据，尤其夹爪连杆第 ⑤ 条。
7. 回填仿真：重跑 `build_sim_world.py`；夹爪数据到手后评估物理夹爪是否可做。
