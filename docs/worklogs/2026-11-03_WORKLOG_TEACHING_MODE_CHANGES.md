# 2026-11-03 WORKLOG 示教模式变更总览（复验用）

> 用途：一页看懂示教链路现在"是什么、和 10-27 那版差在哪、哪些规则是踩坑换来的"。
> 复验时按 §4 的命令逐条跑即可。

## 1. 一条链路的当前形态

```text
RVS(Windows)                     WSL 侧                                         机器人
-----------                      ------                                         ------
T0/T2 → 视角位姿 S0/S2  ──9999──▶ tcp_server ──▶ rvs_pose_client（多帧≥3）
   (法兰系！见 §2)                                    │ 有效性+新鲜度校验
T1/T3 → 目标位姿 R1/S3                              ▼
   (物体系，pose_math 换算)              teaching_sequence 19 步
                                              │
                        ┌─────────────────────┼──────────────────────┐
                        ▼                     ▼                      ▼
              plan_joint_locked        check_tool_envelope      execute_assembly_chunks
              (无碰撞+多seed+锁分支)    (工具/携带盒 vs 实测桌面)  (分段/慢速/守护/自恢复)
                                              │
                                              ▼
                                       arm_controller → 机器人
                       夹爪：gripper.py（Windows 桥，position_mm 为准）
```

## 2. 踩坑换来的规则（最重要，别再犯）

### 2.1 RVS 的 S0/S2 是【法兰位姿】，不是 TCP（所有者 2026-11-03 裁定）

```text
RVS 手眼标定 DepthToRobotTCP 以法兰为参照；
直接把 S0/S2 当 TCP 用 → 整体偏 353 mm（沿工具轴）。
转换：TCP = 法兰位置 + R_flange × (-0.046, 0, 0.353)
落地：config/tasks/common.yaml 的 rvs_view_pose_frame: "flange"；
      teaching_sequence.view_pose_to_tcp() 在步骤 2/4 自动转换。
注意：R1（插槽）与 S3（工件）是【物体位姿】，走 pose_math，不受此规则影响。
```

### 2.2 抓取接近用【垂直下降】，不用水平 +y

```text
10-27 那版沿工具 x 轴水平进入（pre_grasp 在 grasp +120 mm），只在工件 yaw 63.2°
下验证过。2026-11-03 工件 yaw 94.7°，同一动作在第 4/6 段触发保护停止
（停在 y=0.546，差 48.5 mm）——所有者诊断：x 偏约 5 mm 让闭合夹爪蹭到扶手。
改为：先到【抓取位正上方 120 mm】，再纯垂直下降。
  · 实测 xy 漂移 1.8/1.9 mm（真垂直），到位 0.1988 mm / 0.0312°
  · 下降前用 FK 逐点核对手臂连杆与工件 AABB：前臂 610 mm、wrist_1 84 mm、
    wrist_2 59 mm、upperarm 759 mm，均在外；只有工具本体按设计进入包围盒
  · 结果：抓取 → 抬升 150 mm → 抬到安全高度 0.525 m 全部一次通过
```

### 2.3 effort 是姿态相关的，绝对值和不能跨姿态比较

```text
2026-11-03 实测：撤离 15 mm 后 Σ|effort| 反而从 16.8 升到 17.4（重力矩变了）。
接触判定必须用【慢自适应基线的残差】，不能拿"当前值 vs 某个固定基线"比。
基线（预抓取位、工具朝下、桌面以上 45 mm）：静止 16.001±0.022，慢速运动 ±0.14。
```

### 2.4 反控通道会静默掉线

```text
症状：控制器接受目标、状态话题全绿，但臂不动 / NO_MOTION。
判据：50001/50003/50004 没有 ESTAB。
恢复：ros2 service call /io_and_status_controller/resend_external_script \
        std_srvs/srv/Trigger   （2026-11-03 两次都用这一条救回）
自动化：execute_assembly_chunks 已内置 —— NO_MOTION 且 safety=1/robot=7 时自动重发
        一次并返回 10，调用方可重试该段。
```

### 2.5 手动微动之后必须复验 TCP

```text
2026-11-03：两次 wrist_3 微动（±4°/±5°）把 J6 从 32.7° 带到 44.9°，
姿态偏 12.09°（位置只差 4.3 mm）。张紧式夹爪歪 12° 去夹 244 mm 跨距的扶手 = 撞倒工件。
规则：进抓取区前必须过 5b 门（1.5 mm / 0.10°）。
```

### 2.6 门限有物理下限（0.5–1.0 mm）

```text
IK→FK 残差实测 0.003 mm（不是 IK 精度问题）；
真因是控制器关节跟随误差 0.0005–0.0008 rad × 353 mm 杆臂 = 0.5–1.0 mm。
所以抓取门设 1.5 mm / 0.10°（旧 0.5 mm 会长期振荡）。
插入门 (13b) 仍是 0.1 mm —— 低于该下限，实插前需要确认放宽到 0.5 mm 左右。
```

## 3. 本轮（11-03）新增/修改清单

| 文件 | 变更 | 目的 |
|---|---|---|
| `scripts/teaching/plan_joint_locked.py` | IK 改"纯运动学 + 显式 validity"；多种子（当前+4 个已验证姿态）；双规则（|q|>180° / 单次>120°）；**绕圈改用关节总行程>200° 判定**（原按计划时长会被低速误判） | 拒绝绕圈分支，同时不误杀合法慢规划 |
| `scripts/teaching/nearest_feasible_view.py`（新） | 保留请求的相机距离，扫视角邻域，经手眼转 TCP，返回最近的"可达+无碰撞+仍瞄准目标"站位 | RVS 视觉位姿不可采纳时的自动替代 |
| `scripts/teaching/adjust_tcp.py`（新） | 迭代 IK→规划→分段慢速→实测，误差助推闭环（--boost） | 把 TCP 收敛到门限内（含 12° 滚转修正） |
| `scripts/teaching/teaching_sequence.py` | view_pose_to_tcp（S0/S2 法兰→TCP）；步骤 4 不可采纳时自动找替代站位；5b/6b 门 1.5 mm/0.10°；aim_target 放宽到"最新可用 T3" | 规则落地 |
| `scripts/execute_assembly_chunks.py` | time-scale<1 拒绝；**NO_MOTION 时自动重发 external_control 并返回 10** | 反控掉线自恢复 |
| `scripts/compliant_insert.py` | 剔除非有限/荒谬 effort 样本；周期打印 fast/slow/残差；100 Hz 监督 | 力控监测健壮性 |
| `scripts/teaching/gripper.py` | `current_a` 溢出/NaN 判为 None，抓持只看 position_mm | 坏遥测防护 |
| `src/cs625_ap_description/urdf/cs625_active_perception.urdf.xacro` | 夹爪宏**只在仿真实例化**（含 p7_grasp_center_link） | 消除 "Missing gripper_right_finger_joint" 状态不完整 |
| `config/tasks/common.yaml` | 新增 rvs_view_pose_frame: "flange" 及其依据 | 规则固化 |

## 4. 复验命令（按顺序）

```bash
cd /home/oy/projects/cs625-active-perception && source scripts/source_dev_env.sh --full

# 1) 描述：真机 6 个关节、无手指关节
ros2 run xacro xacro src/cs625_ap_description/urdf/cs625_active_perception.urdf.xacro \
  | grep -c 'type="revolute"\|type="prismatic"'          # 期望 6

# 2) 规划器：不可站/绕圈必须拒，合法位姿必须过
python3 scripts/teaching/plan_joint_locked.py 0.7844 0.2928 0.5723 \
  -0.5935 0.6472 0.2667 0.3971 --velocity 0.05 --output /tmp/x.json   # 非 0

# 3) 法兰→TCP 转换
python3 -c "import sys; sys.path.insert(0,'scripts/teaching');
import importlib.util as u; s=u.spec_from_file_location('t','scripts/teaching/teaching_sequence.py');
m=u.module_from_spec(s); s.loader.exec_module(m);
print(m.view_pose_to_tcp([0.7925,0.2724,0.5711],[-0.7068,0.5185,0.3390,0.3416]))"

# 4) 全量测试
python3 -m pytest test/ -q                                # 期望 287 passed

# 5) 反控通道状态
ss -tn | grep -E ":5000[134] " | wc -l                    # 期望 3
```

## 5. 当前未闭环

```text
· 真机描述改动【尚未在运行中的栈上生效】——需要重启驱动栈才加载新描述
  （当前臂正夹持工件停在 0.525 m，建议等工件放下后再重启并验证
   "Missing gripper_right_finger_joint" 消失）
· S0/S2 按法兰解释后，步骤 2/4 的站位会与今天不同 → 下次观测需实测验证 T1/T3 仍能匹配
· 插入门 0.1 mm 低于伺服下限，待所有者确认放宽
· 转运/插入段尚未实跑（本轮只到"抬到安全高度"）
```
