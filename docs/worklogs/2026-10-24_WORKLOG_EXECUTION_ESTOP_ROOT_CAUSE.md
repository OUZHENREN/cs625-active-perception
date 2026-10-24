# 2026-10-24 WORKLOG — 粗观测→精观测跑通：驱动栈陈旧导致"执行超时"，精观测位使位姿收敛到 1 mm

> 接 `docs/worklogs/2026-10-24_HANDOFF_NEXT_SESSION_YOLO_AND_PIPELINE.md` §7「下一步唯一推荐任务」。
> 本文修正了本轮中途的一个误判（见 §3），最终结果是**链路跑通**：臂从粗观测位移动到精观测位，
> 并在精观测位得到 3 帧一致、位置离散 1.0 mm、转角离散 0.285° 的位姿。

---

## 1. 一句话现状

**驱动栈重启后，"执行超时"消失：臂正常移动到精观测位，精观测位下的 ICP 位姿 3/3 帧一致
（位置离散 1.0 mm，转角 0.285°，覆盖率 84%），STABILITY: PASS。**

```text
机器人      safety_mode = 1 (NORMAL)      robot_mode = 7 (RUNNING)
驱动        sockets / arm_controller / joint_states 三项全 True
臂位姿      精观测位  TCP (0.5084, 0.2969, 0.5626)   （粗观测位是 (0.580, 0.350, 0.704)）
工件位姿    xyz (0.5029, 0.6351, 0.1254)  quat (0.1910, 0.9816, 0.0019, -0.0068)
工件几何中心 (0.7206, 0.6182, 0.1335)     TCP→工件中心 0.577 m
运动       已发生（真机，MoveIt 规划 + 分段执行，全程无碰撞）
```

---

## 2. 本轮真正做完的事（按时间顺序）

```text
1. 诊断：上一段执行的 TIMEOUT 不是"等得不够久"（证据见 §4）
2. 恢复相机：相机进程活着但没有图像 → 重启，恢复 0.799 FPS
3. 清理孤儿：3 个 10-22/10-23 遗留的 eli_components_loader 一直占着机器人 30001/29999
4. 重启驱动栈 + 激活 arm_controller + 重启 MoveIt → 机器人状态从 7/3 变成 1/7，sockets True
5. 用库里现成的 capture_dataset.py 算精观测位（look-at + 距离/俯仰/方位 + 三重门禁）
6. 执行移动：粗观测位 → 精观测位（1 个目标位姿，4 段执行，全部 ALL_ASSEMBLY_CHUNKS_OK）
7. 精观测位拍帧 + 3 帧共识位姿 → STABILITY: PASS
8. 用 CAD 内点投影目视复核位姿 → 内点云贴合工件正面
```

---

## 3. 一个必须记录的误判（以及为什么它值得记）

**中途我根据 `safety_mode=7 (ROBOT_EMERGENCY_STOP)` + `robot_mode=3 (POWER_OFF)` 判断
"机器人被急停了，必须有人在示教器上复位"。所有者在关键点上纠正：先重启软件栈。**

重启驱动栈之后，同一组话题立刻变成 `safety_mode=1 (NORMAL)` / `robot_mode=7 (RUNNING)`，
而且 `arm_controller` 从 inactive 变成 active、reverse-control 三个端口全部建立。

**教训**：驱动半死时，`io_and_status_controller` 仍然在发布，但发布的是**陈旧值**。
"话题上有数据"不等于"数据是当前的"。判断机器人能不能动，必须三样一起看：

```text
① safety_mode / robot_mode（要 1 / 7）
② arm_controller 是否 active
③ control_health.py 的 sockets 是否 True（反向控制端口 50001/50003/50004）
③ 才是真正说明"上位机与机器人已经握上手"的那一项；②③ 任一为 False 时，①通常是陈旧的。
```

这也解释了昨晚的 `TIMEOUT`：控制器接受了 goal，但机器人没有在执行；客户端除了超时别无办法。
（昨晚那一刻的确切触发点无法从现有日志 100% 定死，但"驱动/控制器不在工作状态"与现象一致。）

---

## 4. "不是超时太短"的实测证据

`/tmp/c2f_0.json`（昨晚最后一次执行写出的第一段轨迹，09:27）：

```text
点数 15   规划时长 1.4 s   执行器 time_scale=2.0 → 2.8 s
首点 q [0.737, -1.020, -2.557, 0.278, 1.637, 1.558]
末点 q [0.820, -1.070, -2.598, 0.330, 1.565, 1.591]   最大跨度 0.083 rad
```

执行器等待窗口 `duration + 20 = 22.8 s` 后打印 `TIMEOUT`；执行前后 `/joint_states`
六个关节读数**完全相同**。2.8 s 的轨迹在 22.8 s 内一步没走 → 不是超时窗口的问题。

---

## 5. 本轮代码改动（1 个文件）

`scripts/execute_assembly_chunks.py` —— 让"goal 被接受但结果不来"能自报原因：

| 改动 | 作用 |
|---|---|
| 新增 `--no-motion-window`（默认 10 s） | 轨迹要求 >0.02 rad 运动、而 10 s 内关节没动 → 立刻 `NO_MOTION` 并打印机器人状态，返回 8（纯保持段不误报） |
| 等待窗口 `duration+20` → `duration*2+40` | 控制器自己的放弃点是 `2*duration+25`，客户端必须晚于它，否则永远看不到控制器判决 |
| 失败时打印 `safety_mode=N(名) robot_mode=N(名)` | 把"急停/断电/未激活"这类原因直接写出来 |
| 结果打印错误码名字 | 区分"到了但不准"（GOAL_TOLERANCE_VIOLATED）与"根本没跟上"（PATH_TOLERANCE_VIOLATED） |
| `NO_MOTION` 时先 cancel goal | 避免残留目标压住下一条轨迹 |

> 数据流未变：`chunk*.json → execute_assembly_chunks.py → /arm_controller/follow_joint_trajectory → 下位机`，
> 同时旁听 `/joint_states`、`safety_mode`、`robot_mode` 作为判据。

> 本轮还曾给 `scripts/coarse_to_fine_pose.py` 加过一个夹爪间距门禁，后来**已撤回**：
> 库里 `capture_dataset.py` 早就有 `MIN_TCP_CLEARANCE = 0.38` + 工作盒 + 规划三重门禁，
> 应当复用而不是在第二个脚本里重写同一套规则。

---

## 6. 精观测位的算法（复用库里现成的，没有新写）

```text
scripts/capture_dataset.py::camera_target(part, distance, azimuth, elevation, T_tcp_cam, T_current_tcp)
  ① 从"工件 → 基座方向"出发的水平方向，绕 Z 转 azimuth
  ② 再按 elevation 抬起（默认只允许 ±35° / ±40°）
  ③ camera_position = part + direction × distance（距离指【相机】到工件，不是 TCP）
  ④ look-at：光轴 +Z 指向工件中心
  ⑤ T_tcp = T_cam @ inv(T_tcp_cam)      ← 相机在 TCP 后方 0.296 m，必须这样换算
门禁：工作盒 x∈[0.35,1.10] y∈[0.25,1.05] z∈[0.25,0.80]
      TCP→工件中心 ≥ 0.38 m（MIN_TCP_CLEARANCE）
      真实规划一次，检查终点构型落在观测包络内
```

本轮 **dry-run 扫了 15 个候选位姿**（az −20…+20，el 10…30，相机距 0.68 m），只有 4 个通过：

```text
az+10_el+20  TCP (0.506, 0.301, 0.563)  间距 0.45 m   OK   ← 采用（最接近所有者指定的"俯角 22°、方位 +15°"）
az+20_el+20  TCP (0.561, 0.272, 0.563)  间距 0.45 m   OK
az+10_el+30  TCP (0.540, 0.343, 0.620)  间距 0.45 m   OK
az+20_el+30  TCP (0.587, 0.319, 0.620)  间距 0.45 m   OK
其余：2 个超出工作盒、5 个规划失败、4 个终点构型超出观测包络
```

---

## 7. 关键结果：精观测位让位姿从"翻支"变"收敛"

同一台臂、同一个工件、同一个算法，只换观察位：

| 观察位 | 3 帧位置离散 | 转角离散 | 判决 |
|---|---:|---:|---|
| 粗观测位 TCP (0.580,0.350,0.704)，相机距工件 0.72 m，工件被画面下边缘裁掉 | **79.23 mm** | 0.707° | STABILITY: FAIL |
| **精观测位 TCP (0.508,0.297,0.563)，相机距工件 0.68 m，俯角 20°，look-at 居中** | **1.00 mm** | 0.285° | **STABILITY: PASS** |

精观测位 3 帧原始数据：

```text
frame 1  xyz=[0.5029, 0.6349, 0.1248]  median=3.4 mm  coverage=84%
frame 2  xyz=[0.5030, 0.6348, 0.1249]  median=3.4 mm  coverage=84%
frame 3  xyz=[0.5026, 0.6357, 0.1251]  median=3.4 mm  coverage=84%
consensus 3/3   cluster xyz [0.5028, 0.6351, 0.1249]  spread 1.00 mm   STABILITY: PASS
```

最终位姿（`--dump-projection` 目视复核，内点云贴合工件正面）：

```text
xyz   (0.5029, 0.6351, 0.1254)  m
quat  (0.1910, 0.9816, 0.0019, -0.0068)   [x,y,z,w]
几何中心 (0.7206, 0.6182, 0.1335)          = xyz + R(quat)·[-0.208, 0.066, -0.011]
```

> 与交接文档 §3.1 记录的粗定位 `(0.499,0.641,0.127)` 相差 6 mm 以内 —— 跨会话一致，可信。
> 粗观测位之所以翻支：工件在画面下边缘被裁掉（掩膜贴到第 960 行），可见表面不足，
> 正面的四重对称就把解拉到了另一支。**这正是"精观测位"存在的理由，本轮用数据证明了它。**

---

## 8. Capability layer

```text
- environment and dependencies      PASS    相机 0.799 FPS；驱动/相机/camport 工作区三处环境确认
- robot model and simulation        PASS    URDF、robot_state_publisher、TCP 广播正常
- kinematics, control and planning  PASS    规划 + 分段执行成功（4 段全部 ALL_ASSEMBLY_CHUNKS_OK）
- vision, hand-eye calibration/TF   PASS    精观测位下 3/3 帧一致，位置离散 1.0 mm
- active perception / NBV           NOT STARTED（前置链路刚验收，可以开始）
- real-hardware integration/safety  PASS    间距 0.577 m（>0.38 门禁），无碰撞，真机运动已发生
```

### Acceptance target（本轮）

```text
objective        从粗观测位移动到精观测位，并在精观测位推理出具体位姿
pass criteria    ① 有真实运动 ② 位姿位置离散 ≤2 mm、转角离散 ≤0.5°
runtime evidence capture_dataset 输出 "1 frames, 0 failures"；TCP 由 (0.580,0.350,0.704) 变为 (0.508,0.297,0.563)
                 check_pose_stability 输出 position spread 1.00 mm / rotation 0.285° / 3-3 frames agree / PASS
verdict          PASS
```

### Critical-chain status

```text
URDF -> Gazebo entity               NOT APPLICABLE（本轮真机）
ros2_control -> joint_states        PASS  /joint_states 125 Hz
base_link -> camera optical TF      PASS  相机位姿 = TCP × 手眼外参（config/handeye/tcp_to_camera.json）
RGB-D -> normalized sensor topics   PASS  /camera/color|depth/image_raw 有帧（0.8 FPS）
point cloud -> MoveIt planning scene NOT CHECKED（本轮的规划只用目标位姿约束，未依赖场景点云）
NBV decision -> robot execution     PARTIAL 观测位→执行链路已通；NBV 策略本身未接入
```

### Sim/real alignment

```text
reused common core       capture_dataset.py / plan_pose_once.py / execute_assembly_chunks.py / icp_pose_once.py
sim entry point          本轮未跑
real entry point         real_base.launch.py（launch_driver:=true headless_mode:=true）+ real_moveit.launch.py
real execution allowed   是（safety_mode=1, robot_mode=7, arm_controller active, sockets True）
execute/confirmation     execute:=false require_confirmation:=true（launch 默认未改；本轮由所有者口头确认后执行）
vendor code modified     no
```

### Intentionally not done

```text
· 未改覆盖率门禁（70%）：精观测位下 3 帧都是 84%，门禁不是瓶颈；粗观测位的翻支应靠"走到精观测位"解决，而不是放/收门禁
· 未把 capture_dataset.py 的 PART_REFERENCE（0.6804,0.7163,0.1240）更新为本轮实测值 —— 它只用于生成候选列表，等所有者确认
· 未做任务分段试跑 A/B/C/D
· 未接 NBV
```

---

## 9. 命令速查（本轮实际用过，顺序即执行顺序）

```bash
cd /home/oy/projects/cs625-active-perception
source scripts/source_dev_env.sh --full

# 1) 停旧栈（SIGINT 无效时再 TERM）
kill -INT <driver_launch_pid> <moveit_launch_pid>

# 2) 起驱动（headless）+ 激活控制器 + 起 MoveIt
ros2 launch cs625_bringup real_base.launch.py launch_driver:=true robot_ip:="${CS625_ROBOT_IP}" \
  description_package:=eli_cs_robot_description description_file:=cs625.urdf.xacro \
  headless_mode:=true launch_moveit:=false launch_rviz:=false \
  activate_joint_controller:=false execute:=false require_confirmation:=true
ros2 control switch_controllers --controller-manager /eli_ros2_control_node --activate arm_controller --strict
ros2 launch cs625_bringup real_moveit.launch.py description_package:=eli_cs_robot_description \
  description_file:=cs625.urdf.xacro moveit_config_package=elite_cs625_moveit_config \
  semantic_package:=cs625_bringup semantic_file:=config/cs625_active_perception.srdf launch_rviz:=false

# 3) 健康三样（缺一不可）
ros2 topic echo /io_and_status_controller/safety_mode --once   # 要 1
ros2 topic echo /io_and_status_controller/robot_mode   --once   # 要 7
python3 scripts/control_health.py                              # 要 sockets/controller/joint_states 全 True

# 4) 相机（必须在 camport 环境里）
source ~/cs625_camport_ws/install/setup.bash
ros2 launch percipio_camera percipio_camera.launch.py device_ip:=192.168.1.120 \
  color_resolution:=1280x960 depth_resolution:=1280x960 depth_enable:=true \
  depth_registration_enable:=true point_cloud_enable:=false laser_power:=100 \
  depth_qos:=SENSOR_DATA color_qos:=SENSOR_DATA

# 5) 精观测位候选（只算不动）
python3 scripts/capture_dataset.py --dry-run --out /tmp/fine_probe --tag probe \
  --center 0.697 0.597 0.284 --distance 0.68 --azimuths -20 -10 0 10 20 --elevations 10 20 30 \
  --part-xyz 0.905 0.531 0.295 --part-quat 0 0 0 1

# 6) 移动到精观测位 + 拍帧
python3 scripts/capture_dataset.py --out /tmp/fine_obs --tag fine \
  --center 0.697 0.597 0.284 --distance 0.68 --azimuths 10 --elevations 20 \
  --part-xyz 0.905 0.531 0.295 --part-quat 0 0 0 1 --skip-framing --settle 3.0

# 7) 精观测位推理位姿
python3 scripts/check_pose_stability.py --weights ~/cs625_yolo_seg/runs/segment/runs/segment/box_slot_1280/weights/best.pt \
  --template ~/cs625_rvs_archive/models_20261023/弹仓_装配导出.STL --repeat 3
python3 scripts/icp_pose_once.py --weights <同上> --template <同上> --voxel 0.002 --dump-projection /tmp/fine_projection.png
```

---

## 10. 本次踩过 / 复现的坑

```text
1. 话题上有数据 ≠ 数据是当前的。驱动半死时 safety_mode/robot_mode 会一直是旧值（本轮误判的根源）。
2. 判断"能不能动"要三样一起看：状态 1/7 + arm_controller active + control_health sockets True。
3. 重启驱动后 arm_controller 默认 inactive，必须手动 switch_controllers --activate。
4. 相机进程活着不代表相机活着：看 /camera/color/image_raw 有没有帧；SIGINT 停不掉时要 TERM/KILL。
5. percipio_camera 在 ~/cs625_camport_ws，不在仓库 overlay。
6. 相机在 TCP 后方 0.296 m：`--distance` 指相机到工件，换算 TCP 必须用 T_cam @ inv(T_tcp_cam)。
7. 粗观测位裁掉工件下半部 → 四重对称翻支（79 mm）；精观测位居中后立刻收敛（1 mm）。
8. 1 帧"PASS"没有意义，3 帧一致才是判据（`check_pose_stability.py`）。
```

---

## 11. 下一步唯一推荐任务

**在精观测位用共识位姿做一次"取位姿→预抓取位"的规划（只规划不执行），
确认位姿是可以直接喂给抓取环节的；然后再做任务分段试跑 A 段（pre_grasp）。**

理由：位姿已经稳定（1 mm），下一个未知量是"这个位姿能否转成可执行的抓取位姿"，
这是任务链路的第一环，且可以只规划不执行，风险最低。
