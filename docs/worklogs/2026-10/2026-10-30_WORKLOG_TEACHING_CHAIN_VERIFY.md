# 2026-10-30 WORKLOG 示教链路复跑验证（真机）

> 接 `2026-10-27_WORKLOG_RVS_TEACHING_CHAIN.md`。所有者要求：在师兄源码基础上重跑
> 已存在的示教模式，重点是**验证已有模式能不能跑通**，先只做观测前缀（第 0–4 步）。
> 本轮**真机运动 9 次**（含 1 次手动示教器回摆），无碰撞、无急停、无过载。
> 结论：**观测前缀（第 0–4 步）已完整跑通**——T0/T1/T2/T3 全部拿到有效回传；
> 过程中修掉 2 处代码缺陷、定位了"位姿目标选错关节分支"问题、并发现 RVS 当天响应极慢。

## 1. 一句话结果

示教链路的位姿通路已打通并实测出结论边界：插槽（T0→S0→T1→R1）通了，工件（T2→S2→T3→S3）
一次都没通。查出的不是 ROS 侧 bug，而是三件独立的事：① 驱动栈半死 2 天；
② 捕获路径与 T0/T2 返回值两处代码缺陷（已修/已改）；③ RVS 侧工件 3D 匹配当天不可用。

## 1b. 20:20–20:31 补测：T2/T3 通过，观测前缀闭环

前面的"T2 全失败"发生在 17:08–17:47。所有者 20:22 要求把臂送回工件粗观测点重试，
结果变了——不是链路变好，而是**之前的失败里混了一次"RVS 只是慢"**：

```text
20:20:53  T2 发出 → 立即 NG
20:24:42  S2 到达 (713.148, 519.068, 563.878) / rpy(11.4,124.6,80.7)   ← 合理拍摄位
20:25:12  又一个 NG
```

S2 比客户端 180 s 超时晚到约 50 s，所以客户端报 `NO_ANSWER`，而 `tcp_server` 日志里其实是成功的。
这条 S2 已按 `tcp_server` 收到的原始报文还原存档：`runtime/rvs_teaching/T2_20261030_latched.json`
（`/box_precision_view_tcp_pose` 是 VOLATILE 发布，无法回读历史消息）。

随后（全部用"当前构型做 IK 种子 + 关节空间规划"的方式移动）：

```text
→ S2 (0.7131,0.5191,0.5639)   75 点/7.34 s  执行 OK (0.0014 rad)  FK 最低 0.582 m
T3 → S3 = (0.7108, 0.8745, 0.3558)  quat (0.0061,-0.0012,0.5239,0.8517)   18.7 s
```

由 S3 反算（`pose_math`）：

```text
grasp     = (0.8991, 0.7788, -0.0571)
pre_grasp = (0.8450, 0.6717, -0.0581)
```

与 10-27 的 grasp (0.7882, 0.6623, -0.0576) 对比：**z 差 0.5 mm、姿态相近（yaw 63.2° vs 66.4°），
水平位置差 161 mm**——与所有者所说的"工件在桌上但位置变了"完全一致。

插槽链（来自今天 T1 的 R1）反算：`insert = (1.1701,-0.0094,0.0314)`、
`pre_insert = (1.1118,-0.0023,0.3765)`，与第 14 步常量差约 13 mm / 4 mm。

**结论修正**：T2 不是"系统性失败"，而是"RVS 响应时间从 11 s 恶化到 3–4 min，且失败时回 NG"。
客户端 180 s 超时对今天的状态不够；建议把 `rvs_pose_client.py` 的默认 `--timeout` 提到 300 s
以上，并把 `tcp_server` 日志里的 S*/R* 行作为兜底来源。

## 2. 现场恢复（开工前不是可动状态）

开工时的客观状态：

```text
safety_mode = 7 (ROBOT_EMERGENCY_STOP)   robot_mode = 3 (POWER_OFF)
50001/50003/50004 只有 LISTEN，0 条 ESTAB  → 机器人侧没有跑 external_control
dashboard 服务：'socket disconnected: Connection reset by peer' / 'Broken pipe'
launch.log：eli_components_loader 自 10-28 20:03 起每 10 ms 刷
            'Primary port receive package head had expection: End of file'
            至 10-30 16:46 累计 560,898 行
```

所有者确认"急停没按、机器人已上电"→ 判定为**半死驱动**（AGENTS.md 明确警告的模式），
按 handoff 先重启软件栈：

```bash
kill -INT -<launch pgid>            # 等价 Ctrl-C，不用 pkill/wildcard
ros2 launch cs625_bringup real_base.launch.py launch_driver:=true \
  robot_ip:=192.168.1.200 description_package:=eli_cs_robot_description \
  description_file:=cs625.urdf.xacro headless_mode:=true launch_moveit:=true \
  launch_rviz:=false launch_sensor_adapter:=true activate_joint_controller:=false \
  execute:=false require_confirmation:=true
python3 scripts/control_health.py --activate
```

重启后四项门全过：`safety_mode=1`、`robot_mode=7`、`arm_controller=active`、
`sockets/controller/joint_states=True`；50001/50003/50004 与 30004/29999 全部 ESTAB。

顺带清掉 4 个 10-27 遗留的孤儿节点（父进程已是 init，跑了 2 天 22 小时）：
2× `controller_stopper_node` + 2× `rgbd_sensor_adapter`——它们与主栈重复发布同名话题，
是 `ros2 node list` 一直报"nodes share an exact name"的来源。

## 3. 观测前缀实测（第 0–4 步）

所有运动均为位姿目标 + 一次下发执行，执行前用 `/compute_fk` 逐点算 TCP 高度（空规划场景下
MoveIt 不校验桌面，这是本链路当前唯一的碰撞防线）。

```text
1  slot_coarse  规划 171 点/16.97 s → 执行偏差 0.0015 rad → T0 ✅
                S0 = (1.0841, 0.0490, 0.6145)，3.9 s
2  slot_fine(常量) 规划 53 点 → 执行 OK (0.0019 rad) → T1 ❌ NG,POSE,0,0,0,0,0
2' S0(RVS 要求) 规划 155 点 → 执行 OK (0.0014 rad) → T1 ✅
                R1 = (1.2542, 0.1927, 0.4391)，3.6 s，姿态≈单位四元数
3  workpiece_coarse 规划 274 点/27.28 s → 执行 OK (0.0016 rad) → T2 ❌
                105 s 内 3 次 NG，最后吐出 S2=(0.9605,1.1045,-0.0666)
                换更宽站位 (0.5013,0.2532,0.7873) 重试 → 1 次 NG 后静默
                回到六模式粗观测位后再试 → 180 s 无回传（1 次 NG）
4  workpiece_fine → T3  未做（S2 不可用）
```

### T2 为什么判死

```text
S2 = (0.9219, 1.1184, -0.0771) / (0.9605, 1.1045, -0.0666)
按"工件位姿"解释 → 抓取位 z = -0.371 m（桌面下 37 cm，物理不存在）
按"拍摄位姿"解释 → 同样在桌下
```

对照校验：同一套 `pose_math` 反算 10-27 的 S3，得 (0.7882, 0.6623, -0.0576)，
与 worklog 记录的 (0.788, 0.662, -0.058) 完全一致 → **换算没问题，是 RVS 当天匹配失败**。
所有者现场确认：工件在相机画面里，但没有被识别框住。

## 4. 代码改动（两处，均未提交）

| 文件 | 改动 | 依据 |
|---|---|---|
| `scripts/teaching/rvs_pose_client.py` | `REPO = parents[2]`（原 `parent.parent`） | 原写法把捕获 JSON 写到 `scripts/runtime/rvs_teaching/`，而 `teaching_sequence.latest_capture()` 只读仓库根 `runtime/rvs_teaching/` → 第 5 步必然报 "missing T3 capture"。修正后今天的 T1/T2 已正确落盘 |
| `scripts/teaching/teaching_sequence.py` | 第 2/4 步改用 T0/T2 返回的实时 S0/S2（原来用写死的 `slot_fine` / `workpiece_fine` 常量） | 实测：站在固定 `slot_fine` 上 T1 必 NG，站到 T0 给的 S0 上 3.6 s 成功。脚本注释本来就写 "move to that pose"，代码与注释不一致 |

## 5. 构型分支问题（本轮最重要的认知）

修正口径：位姿目标复现不了当初那套动作。

```text
                  J1 shoulder_pan    J2          J3 elbow      TCP
本轮中途实际       2.398 rad 137.4°  -0.090     -2.243        (0.5865,0.3473,0.7037)
六模式期望         0.752 rad  43.1°  -1.001     -2.553        (0.5886,0.3482,0.7075)
```

同一个 TCP 位姿有多个关节解。`plan_pose_once.py` 只给位姿约束，MoveIt 会在**当前构型附近**
选 IK 分支；今天臂的起始构型 J1 已经是 ~137°，于是每一步都延续了这个分支——相机位置是对的
（所以 T0/T1 能成），但整条臂的姿态与六模式完全不同，所有者一眼看出"尤其是 J1"。

处置：所有者用示教器把臂从 J1=137° 回摆到 63°，本轮再按位姿目标补到 **46.1°**
（参考 43.1°），TCP 误差 3.8 mm / 1.01°，整条臂回到正确分支。回摆前后的关节值已存档：
`runtime/rvs_teaching/posture_before_jogback_20261030.json`。

推断：这条链路要可复现，目标必须是**关节目标**或**用期望构型做 IK 种子**，而不是位姿目标。

## 6. 顺带量到的两个插入相关数字

```text
R1 反算插入终点   (1.1701, -0.0094, 0.0314)     第 14 步写死常量 (1.1574, -0.0132, 0.0321)
                  x 向差约 13 mm —— 对 0.005 m/s、1 mm 级间隙的插入是致命的
pre_insert 反算   (1.1117, -0.0023, 0.3764)     写死常量 (1.0996, -0.0125, 0.3773)
```

这也是 10-27 过载根因的另一半（此前只归因于工件被转过 7.71°）。

## 7. 验证方式与结果

```text
python3 scripts/control_health.py                    三 True，exit 0
ros2 topic echo /io_and_status_controller/safety_mode --once   mode: 1
ros2 topic echo /io_and_status_controller/robot_mode   --once   mode: 7
ss -tn | grep 192.168.1.200                          50001/50003/50004/30004/29999 全 ESTAB
python3 scripts/plan_pose_once.py ...                code 1（6 次，全部成功）
python3 scripts/execute_assembly_chunks.py ...       ALL_ASSEMBLY_CHUNKS_OK（7 次）
```

运动共 7 次，全部 `status=4 SUCCESSFUL`，到位偏差 ≤0.0019 rad；每次执行前 FK 高度检查
最低 0.637 m（baseline 段 0.708 m）。真机运动：是。Gazebo：否。

## 8. Capability layer

```
- environment and dependencies      PASS      --verify 全绿；师兄 tcp_server 以 PYTHONPATH 启动，未混 overlay
- robot model and simulation        PARTIAL   真机描述/驱动 OK；仿真侧本轮未触碰
- kinematics, control and planning  PASS      规划 6 次、执行 7 次全部成功；构型分支问题已定位
- vision, hand-eye calibration/TF   BLOCKED   插槽通路通；工件通路 5 次全失败（RVS 侧识别）
- active perception / NBV           NOT STARTED
- real-hardware integration/safety  PARTIAL   四项门齐、空场景下 FK 高度自检；插槽/工件均未真抓
```

## 9. Critical-chain status

```
URDF -> Gazebo entity                NOT APPLICABLE（本轮未跑仿真）
ros2_control -> joint_states         PASS      125 Hz，重启后无 overrun 报错
base_link -> camera optical TF       BLOCKED   依赖 RVS 内部手眼；ROS 侧无 camera_*_optical_frame
RGB-D -> normalized sensor topics    BLOCKED   相机被 RVS 占用（互斥），/sensors/camera/status connected=false
point cloud -> MoveIt planning scene NOT STARTED  world.collision_objects = []（10-27 既有做法）
RVS pose -> robot execution          PARTIAL    T0/T1 通；T2 起不通
```

## 10. Sim/real alignment

```
共用应用核心     否——本节只动了 scripts/teaching/，六模式与仿真代码零改动
sim entry        未使用
real entry       scripts/teaching/teaching_sequence.py / run_teaching_chain.py
real execution   允许（四项门通过），本轮实际执行 7 次运动
gate state       空规划场景 + FK 高度自检，无桌面碰撞校验（历史做法，风险已知）
```

## 11. 明确没做的

- 第 4 步 T3、第 5–18 步（预抓取、抓取、抬升、transit、staging、旋转、插入、整平、释放、
  退出、baseline 之外的链路动作）——因为 S2 不可用，无从开始；
- 没有修改师兄源码（只以 PYTHONPATH 方式运行其 `tcp_server`），没有搬动租户 overlay；
- 没有提交 git（`scripts/teaching/` 整体仍未跟踪）；
- 没有解决"位姿目标 vs 关节目标"的复现问题，只定位并记录。

## 12. 下一步（唯一推荐）

先在 RVS 侧解决工件识别（2D 是否框住 / 3D 模型与当前工件是否匹配 / 工件姿态），
再回到第 3 步。同时建议把示教目标从"位姿目标"改为"关节目标或期望构型种子"，
否则即使 T2 通了，动作序列也无法复现 10-27 那套已验证姿态。

## 13. 证据位置

```text
runtime/rvs_teaching/T0_20261030_170111.json       S0
runtime/rvs_teaching/T1_20261030_170545.json       R1
runtime/rvs_teaching/T2_20261030_170831.json       第一个坏 S2
runtime/rvs_teaching/posture_before_jogback_20261030.json   回摆前构型
~/.ros/log/2026-10-30-16-54-39-.../launch.log      第一次重启的完整日志
~/.ros/log/2026-10-30-17-47-02-.../launch.log      回摆后重启的日志
tcp_server 原始往来（含被丢弃的 NG）             本轮后台作业 bash-c0be7d70bbff418e 的 stderr
```

## 14. 复盘后的程序修正（20:42，所有者确认"固定/动态"规则后）

所有者规则：**两个粗采样位姿固定；两个精采样位姿（S0/S2）与抓取位姿（S3 换算）动态，
必须 RVS 给出。** 据此补齐三处：

```text
scripts/teaching/teaching_sequence.py
  · 删除 POSE 中已无引用的固定 slot_fine / workpiece_fine 常量（防止未来误用）
  · rvs() 内部超时 140 s → 300 s（子进程 220 s → 360 s）
  · 头部新增 2026-10-30 设计规则；grasp 注释标注插入链 13 mm 未决问题
scripts/teaching/rvs_pose_client.py
  · 默认 --timeout 150 s → 300 s（10-30 实测 T2 回传 229 s）
scripts/teaching/run_teaching_chain.py
  · capture_pose 超时 140 s → 300 s
scripts/teaching/README.md
  · 新增"固定与动态"一节；19 步表更新为 S0/S2 动态语义
```

**未决、未动**（超出观测前缀范围，需所有者决策）：
1. 插入链（步骤 12–18）仍用固定常量；今天 R1 反算 insert = (1.1701,-0.0094,0.0314)
   比常量 (1.1574,-0.0132,0.0321) 偏 13 mm——与"插槽固定"（10-27 所有者确认）矛盾。
2. 关节目标精确到位（≤0.42°）但实测 TCP 与名义粗采样位姿差 8.6 mm——world↔base 偏移、
   真实 FK 与 URDF 差异、TCP 定义差异三者未甄别（本轮探针未读到 TF 静态树，结论未定）。
3. latest_capture() 只按 mtime 取最新捕获，无有效性校验（今天 17:08/17:30 的无效 S2
   z<0 若在最新位置会被静默使用）。

## 15. 晚场完整复跑（20:53–21:09，所有者要求"再跑一遍前四步"）

全部用"当前构型 IK 种子 + 关节空间规划"（临时工具 /tmp/plan_joint_goal.py，
FK 高度检查 /tmp/fk_height_check.py，均未入库），运动 4 次全部成功。

```text
1 slot_coarse  执行 OK (0.0018 rad)
   T0 第 1 次 → S0 x=4.41 m（垃圾）；第 2 次 → x=4.46 m（垃圾）；
   第 3 次 → S0 (1.0844, 0.0490, 0.6149)  与下午 S0 差 0.4 mm   ← RVS 又"先瞎后好"
2 → S0         执行 OK (0.0015 rad)
   T1 → R1 (1.2561, 0.1960, 0.4427) 4.4 s   与下午 R1 差 ~5 mm
3 workpiece_coarse  第一次 IK 又跳回 137° 分支（被拦），改为直接以所有者参考构型
   [0.752,-1.001,-2.553,0.251,1.619,1.568] 做关节目标 → 执行 OK (0.0018 rad)
   T2 → 281.1 s → S2 (0.7838, 0.3205, 0.5691)
4 新 S2 不可达：IK 唯一解（J1=5.7° 族）经 /check_state_validity 判定自碰撞（1 contact）；
   avoid_collisions IK 只有极端解（J1=157.8°, J5=326.1°）；关节空间规划 99999；
   位姿目标给出 734 点翻腕路径（J2=255°）→ 全部拒绝
   → 退回下午验证过的 S2 (0.7131, 0.5191, 0.5639) 触发 T3
   T3 → S3 (0.7191, 0.6696, 0.3539) yaw 73.9°，21.7 s
```

**两个新事实**：
1. RVS 给的精拍摄位姿（S2）**不保证可达/无自碰撞**——这次给的 S2 在 MoveIt 模型里自碰撞。
   示教程序今后必须在"移到 S0/S2"前先做 IK+有效性检查，不能拿到就执行。
2. 今晚 T3 的 S3 与下午差 **205 mm（y 向）**、yaw 差 10.7°。两次之间没有碰过工件，
   原因未定：工件被移动过、或"站位与 RVS 最新 T2 期望不符"导致匹配劣化。
   **抓取位以下午干净流程的结果为准：grasp (0.8991, 0.7788, -0.0571)**。

验证：师兄 tcp_server 对下午 S3 独立算出 box_grasp_tcp_pose = (0.899, 0.779, -0.057)，
与我方 pose_math 差 ≤1 mm（tcp_server 日志 20:30:26 行）。

结束状态：臂在 S2 (0.718, 0.518, 0.565)，safety=1 / robot=7 / 三 True，栈保持运行。

## 16. 三个新行为 + 抓取位规划（21:15–21:25，所有者指示）

所有者指示：① 插槽位姿几乎固定 → 加入"对比往日坐标"行为；② RVS 一次触发难成功 →
至少 3 帧对比、最多 10 次尝试；③ 工件是动过的 → 用最新 S3 规划到抓取位。

```text
scripts/teaching/rvs_pose_client.py
  · --min-frames 3 / --max-attempts 10（默认值即此策略）
  · 帧有效性过滤：位置在工作空间盒内（x∈[-0.6,1.8] y∈[-1.2,1.6] z∈[-0.9,1.6]）、
    四元数模长 0.99–1.01；垃圾帧存档（_aNN 后缀）但不计入
  · 3 帧之间打印最大 pairwise 偏差（位置 mm / 姿态 deg）
scripts/teaching/teaching_sequence.py
  · rvs() 传 --min-frames 3 --max-attempts 10（子进程超时 3600 s）
  · 新增 check_slot_history()：T1 后把新 R1 与六模式参考 + 历史 R1 对比
    六模式参考 = 10-27 已证插入常量 (1.1574,-0.0132,0.0321) 以 Ry(180°)
    （= diag(-1,1,-1) = POSE["straight"] 的四元数）反推 → 槽姿 (1.2394, 0.1798, 0.4446)
    历史文件 runtime/rvs_teaching/slot_history.json（已播种今天两条 R1）
    阈值：>30 mm 或 >3° 才 WARN；只报告不中断
    实测：两条 R1 与参考差 20.4 / 23.3 mm、0.9–1.3° → PASS
scripts/teaching/README.md  新增"RVS 多触发与插槽对比"一节
```

**途中修正**：参考反推最初误用 Rx(180°)（差 437 mm/180°），改为 Ry(180°) 后与
今天两条 R1 差 ~20 mm——与之前报告的插入常量 13 mm 偏差自洽。

**抓取位规划（只规划、未执行）**：

```text
多帧 T3（新策略实跑）：3/3 帧有效，耗时 17–21 s/帧，帧间偏差 0.2 mm / 0.04°
  共识 S3 = (0.7190, 0.6695, 0.3541)  quat (0.0075,-0.0019,0.6012,0.7991)
  抓取位   grasp     = (0.9221, 0.6120, -0.0590)  quat (-0.6012,0.7991,0.0075,0.0019)
           pre_grasp = (0.8888, 0.4968, -0.0605)
规划（当前构型 IK 种子 + 关节空间）：111 点 / 10.96 s（0.05 缩放）
  终点关节 (41.4, -127.2, -93.6, -48.7, 90.6, 56.6)°  干净，无翻腕
  /check_state_validity：valid，0 接触
  FK 终点 TCP (0.9152, 0.6184, -0.0571) vs 目标 9.6 mm / 0.92°（IK 容差内）
  路径 TCP 高度 0.565 → -0.057（末端下降为抓取本身，工件扶手在 base z≈-0.058，
  与 10-22/10-27 实测一致）
未执行；等所有者审阅后决定。

## 17. 事故记录：预抓取下降段撞到工件桌（21:26）

所有者反馈：速度太快，臂直接撞到桌面；随后所有者确认"无接触残留、无损伤"，
并已手动把臂撤回高位。机器人当前 safety_mode=7（急停）/ robot_mode=3（断电），
控制通道断开；臂停在安全高度（TCP z≈0.43）。

```text
直接原因（两条，都是我的错）：
1. --time-scale 语义用反：execute_assembly_chunks.py:184 是 scaled = point["t"] * time_scale，
   >1 才是慢。预抓取执行时传了 0.5（以为半速），实际是 2 倍速；且规划用的速度缩放
   是 0.05（程序规定 pre_grasp 为 0.02）→ 实际约 5 倍于程序速度。
2. 模型 FK 与实机 TCP 在工具下翻姿态失配：/tcp_pose_broadcaster 发布的是控制器
   （示教器 tool_data）TCP 状态接口（vendor URDF 中 <sensor name="tcp_pose"> 只有
   state_interface，无 link），不是从 URDF 算的 FK；MoveIt 规划的是
   my_end_effector_link（flange+(-0.046,0,0.353)）。工具朝下时两者差 5–19 mm，
   事故中段（所有者半途撤臂时读取）同关节下两者 z 相差约 630 mm。下降深度因此
   与模型预测完全不同，加上空规划场景（world.collision_objects=[]），没有任何碰撞防线。
```

```text
已落地的修正：
1. execute_assembly_chunks.py：--time-scale < 1.0 直接 REFUSED（守护 + 帮助文本写明
   语义与事故来源）；全仓现有调用全部 ≥1.0，测试 8/8 通过。
2. scripts/teaching/README.md 已知限制新增两条：
   ① time-scale 语义；② 在查清控制器 TCP 与 URDF TCP 关系并把工件桌灌进规划场景前，
   【禁止】下降到 z<0.2。
```

```text
下一步调查（动臂前必须完成）：
1. 读控制器 tool_data / 示教器 TCP 设定，确认控制器 TCP 与
   my_end_effector_link（-0.046, 0, 0.353）是否一致；在"工具下翻"姿态下用
   实测关节对比 tcp_pose_broadcaster 与 MoveIt FK 的差。
2. 用实测桌子几何重建规划场景（10-27 已知桌面模型差 12–65 mm 待重做），
   下降段必须有碰撞模型。
3. 预抓取/抓取下降改为：慢速 + 分段下降 + 每段用【控制器 TCP】对位校验。
在以上三项完成前，示教序列执行到第 4 步（T3）为止。

## 18. 工具真实长度查明（2026-10-31，所有者提出质疑后）

所有者指出：TCP 到法兰是 353 mm，但**工具真实长度未必是 353 mm**。三份独立证据：

```text
① 控制器/URDF       flange→TCP = 353 mm；2026-10-31 12:12 实测 tcp_pose_broadcaster
                    与 MoveIt FK 差 0.0 mm（同时采样 5 组），模型 TCP 可信
② 厂商碰撞网格       tool0_collision.STL（挂在 my_end_effector_link 即 TCP 系）
                    z ∈ [-0.3595, -0.146] m，x ∈ [-0.004, 0.173]，y ∈ ±0.120
                    → 法兰在 TCP 后方 353 mm，工具实体前沿仅到 TCP 后方 146 mm
③ 仓库实测夹爪       cs625_parallel_gripper.xacro（注明 "measured, not invented"）
                    法兰系 arm Z ∈ [4.0, 150.0] mm → 工具实体前沿 = 法兰前方 150 mm
```

**结论**：工具静止实体长度 = 法兰前方 **150 mm**（采纳保守值，已写入
`config/tasks/common.yaml` 的 `tool_body_forward_from_flange_mm`）；TCP 在 353 mm；
差额 146–203 mm 由夹爪张开时支撑臂前伸补足，**该连杆运动学无模型**。

### 同轮查明的真正碰撞原因（推翻 §17 的假设）

所有者现场确认：撞的是**夹爪下方的电机保护座**，撞在**插槽桌面**上。

```text
slot_table    x∈[1.01, 1.70]   y∈[-0.45, 0.40]   top_z = -0.106
workpiece_tbl x∈[0.30, 0.99]   y∈[0.30, 0.90]    top_z = -0.103
预抓取 TCP (0.8888, 0.4968, -0.0605) —— TCP 全程在工件桌上方
但工具横向包络（TCP 系 x 最大 +0.173、y ±0.120）外缘可达 (1.06, 0.376)
→ 正好进入插槽桌足迹 (x≥1.01, y≤0.40)
```

**教训**：只看 TCP 的 FK 高度检查无法防止此类碰撞；电机保护座不在任何网格里。
因此新增安全规则（写入 config/tasks/common.yaml 的 `tool_safety_note`）：
**任何 TCP z 低于 (桌面 + 0.15) 的路径，必须保证整个工具包络避开两张桌子的足迹，
不能只校验 TCP 点。**

§17 中"模型 TCP 与实机不一致（630 mm）"的结论**作废**——那是拿 21:27 的 TCP
去比 21:37 的关节（中间所有者手动撤过臂），并非同时刻采样。

## 19. 保护座找到 + 分段下降实装 + 复验通过（2026-10-31 下午）

### 保护座在模型里（纠正 §17/§18 的错误）

所有者指出电机保护座就在模型里。查明：本仓库真机描述
`cs625_active_perception.urdf.xacro` 的 tool0 链接，其【碰撞几何】就是 SolidWorks
导出的完整工具刚体 `meshes/tool/gripper_0920.stl`（法兰坐标系，单位米）：

```text
X 对称 -147.285 .. +147.285（宽轴 294.6 mm，装配时绕工具轴 +90°）
Y ±42.5 mm；Z -7.0（安装面后方定位凸台）.. +207.0 mm
→ 电机保护座是该刚体的一部分（所有者确认："它和夹爪是一部分"）
```

§17 说"保护座不在任何网格里"是**错误**的——当时只量了厂商的 tool0_collision.STL
（师兄那份 177×240×214 的对象），没查仓库自己的 SolidWorks 网格。
`config/tasks/common.yaml` 工具几何块已改写为权威值（tool_body_forward_from_flange_mm=207、
半宽 147.3 mm），`scripts/check_tool_envelope.py` 的包络改为两网格的并集并注明保护座归属。

### 分段下降（B）实装进示教程序

`teaching_sequence.py` 新增 `SEGMENTED = {"pre_grasp": 4, "grasp": 6}` 与
`execute_plan(path, ..., segments)`：下降段先过 `check_tool_envelope.py`（margin 20 mm），
再把轨迹切成 N 段、**每段从 t=0 重新计时**、逐段执行 + 到位校验，任一段失败即停。
途中修掉一个自己引入的 bug：分段保留原始绝对时间戳，第 3 段起点 t≈8 s ×2 倍缩放后
控制器"等待 16 s"，执行器 10 s 无动作窗口误报 NO_MOTION——已改为每段减掉首点时间。

### 复验：速度假设成立

同一条预抓取轨迹（目标 (0.8888, 0.4968, -0.0605)，含保护座在内的权威包络检查
最窄间隙 174 mm），以 time_scale 2.0 分 4 段慢速执行：

```text
段 1..4 全部 ALL_ASSEMBLY_CHUNKS_OK，末段到位偏差 0.0002–0.0003 rad
最终 TCP (0.8843, 0.4933, -0.0696)，桌面以上 36 mm，全程无接触
```

对比昨天：同目标、同区域，2 倍速一次下发撞到插槽桌面；今天慢速分段执行无接触。
**结论：2026-10-30 事故的直接原因是速度过快（time-scale 语义用反 + 0.05 规划缩放），
规划路径本身含保护座在内离两张桌子 174–189 mm，无模型接触。**

遗留：名义预抓取位与实际到位差 10.8 mm / 0.68°（关节到位 0.0003 rad）——
与之前观测位 5–19 mm 同源的 world/base 或 IK 容差问题，抓取前需用 verify_pose
门做一次 TCP 级微调确认。

## 20. 2026-11-02 续跑：TCP 微调、S2 不可站的根因、反控通道丢失

### TCP 微调（所有者要求"调整 TCP"）

迭代 IK + 关节空间规划（容差 0.0005 rad）+ 分段慢速执行，把 TCP 从 18.0 mm / 1.61°
调到 0.26 mm / 0.04°。过程中发现：前两次迭代规划器给出 9 点和 1 点的"原地"轨迹（IK 解
与当前关节几乎相同），第三次才收敛——**关节目标无法消除 TCP 级残差**，必须迭代。

### RVS 指定的 S2 为什么站不了（根因）

```text
S2 位姿 IK 用 5 个不同种子探测，全部收敛到同一组关节：
  J = 3.8, -89.2, -113.5, -42.4, 143.3, 21.4   (J5 拱到 143°)
/check_state_validity：forearm_link <-> my_end_effector_link 重叠 5.3 mm
手腕翻转解 J4+180/J5取反/J6+180：姿态对（0.04°）但位置差 255.7 mm
  → 因为 TCP 不在法兰轴线上（-0.046, 0, 0.353），翻转会把横向偏移翻到另一侧
```

即：该位姿**唯一解就是会自碰撞的解**。而"以前能站"是因为师兄的厂商描述用的是
177×240×214 mm 的 tool0_collision.STL，本仓库换成 SolidWorks 真机刚体
gripper_0920.stl（294.6×85×214 mm，含电机保护座）后干涉才显现。
旁证：S2 在 10-31 与 11-02 两天几乎一致（0.7856/0.2928/0.5729 与 0.7844/0.2928/0.5723），
而工件已移动——**S2 很可能是 RVS 工程里存下的固定拍照位**，不是按当前工件算的。

### 绕行方案与今日观测

改用 10-30 验证过的 S2 站位 (0.7131, 0.5191, 0.5639)（IK 无碰撞、valid=True、0 接触；
相机到工件 0.493 m、视线偏差 7.4°）补拍：

```text
T2 三帧：S2 = (0.7844, 0.2928, 0.5723)，spread 1.4 mm / 0.22°
T3 三帧：S3 = (0.6146, 0.5763, 0.3542) yaw 94.5°，spread 0.2 mm / 0.04°
→ grasp     = (0.8269, 0.5944, -0.0579) quat (0.7346,-0.6785,-0.0062,-0.0026)
→ pre_grasp = (0.8364, 0.4748, -0.0594)
与 10-30 的抓取位相差 97 mm（工件确实移动过）
```

### 新发现：反控通道会静默丢失（重要运维教训）

现象：控制器接受目标、`status=4` 之前无任何报错，但臂**完全不动**；连 0.05 rad 的
微动也不执行。排查发现 **50001/50003/50004 只有 LISTEN、没有 ESTAB**——机器人侧的
`external_control` 脚本连接掉了（长时间空闲后掉线），而状态流仍正常，所以
`safety_mode=1 / robot_mode=7 / speed_scaling=100` 全绿，极具迷惑性。

恢复：`ros2 service call /io_and_status_controller/resend_external_script std_srvs/srv/Trigger`
→ 三条 ESTAB 立刻恢复 → 微动测试通过。
**今后：任何"动作不执行"的排查，第一项就是查反控三端口的 ESTAB，而不是看模式话题。**

### 当前状态

```text
臂在预抓取位 (0.8361, 0.4749, -0.0596)，与目标差 0.35 mm / 0.04°，桌面以上 46 mm
夹爪闭合 (position_mm=0)，四项门全绿，抓取位在 +y 方向 120 mm 外
下一步：第 6 步（闭合进入抓取位）→ 第 7 步（张开 50 mm 张紧式抓取）
```

## 21. 审计 + 7 条漏洞修补（2026-11-02 晚）

审计文档：`docs/worklogs/2026-11-02_REVIEW_TEACHING_MODE_AUDIT.md`（开发程度 / 与师兄
对比 / 漏洞清单）。同轮按严重度修补全部 7 条：

```text
1  S0/S2 无可达门、位姿目标会翻腕
   → 新增 scripts/teaching/plan_joint_locked.py：IK(avoid_collisions=True, 当前关节种子)
     + 关节空间规划（--goal-tol 可收紧到 0.0003）。move() 全部改走它；
     无碰撞 IK 直接拒绝（exit 2）。实测：不可达 RVS S2 → NO_COLLISION_FREE_IK 拒绝；
     baseline 正常规划成功。
2  latest_capture 无校验
   → 新增 validate_capture()：工作空间盒 + 四元数模长 + 按命令的 z 合理域（T3 限 z>-0.35）
     + 新鲜度 ≤6 h；从最新往回找第一个合法捕获，全不合法则 None + 原因。
3  抓取进入前无 TCP 门
   → 步骤 5b/6b 新增 verify_pose 门（0.5 mm / 0.05°，verify_pose.py 增加
     --pos-tol-mm/--rot-tol-deg CLI）；13b 保持 0.1 mm 插入门。
4  抬升/携带段无包络
   → SEGMENTED 扩到 lift(2)/transit(2)/staging(2)；check_tool_envelope.py 新增
     --carried（并入 check_carried_clearance 的携带盒 T_LINK_BOX + BOX bbox）。
     设计决定：携带盒只约束 transit/staging（横向搬运）；lift 只做 tool-only 包络，
     因为抬离段起始时工件本就压在桌上（几何门会永远拒绝它），且 lift 是工件自身
     脚印内的竖直运动、由分段+到位校验保护。实测：合成 lift 轨迹 tool-only 187 mm 通过；
     横向低空路径被 --carried 正确拦截（证明该门有效）。
5  R1 采了不用
   → 插入链改为实时 R1 → insert_from_slot 计算 insert/rotated/pre_insert；
     无新鲜 T1 时回退六模式常量并打印 WARN。步骤 12-15/17 全部改走该链。
6  spread 只打印不判据
   → rvs_pose_client 对 T1/T3 加帧间上限 30 mm / 3°，超限 SPREAD_TOO_LARGE exit 4；
     S0/S2 不限（RVS 每次重算拍摄位，合理变化）。
7  低危项
   → 步骤 0 失败打印 WARN；grip 后 grip_still_holding() 校验（≥40 mm 才 lift）；
     run_teaching_chain 默认 grasp/s3-quat 改为 None，未显式传参时打印"10-27 旧值"告警。
```

验证：全文件语法通过；dry-run 19 步正常；pytest 8/8；新规划器对不可达位姿 exit 2、
对可达位姿 exit 0；--carried 对横向低空路径正确拦截。

## 22. 下一专项（参照师兄，另行进行）

1. 环境碰撞：参照 environment_collision_generator / environment_point_cloud_publisher
   把实测桌面+夹具灌入 MoveIt 场景（先处理"厘米级精度误报"）。
2. 携带工件：参照 collision_object_publisher / attached_box_visual_publisher，
   抓取成功后把工件 attach 到规划场景的 TCP。
3. 力控装配：参照 cs625_tcp_force_probe / cs625_compliant_placement 为插入段补
   力/力矩反馈与柔顺。

## 23. 环境碰撞专项落地（2026-11-02 晚）

按 2026-11-02_PLAN_ENV_COLLISION_INTEGRATION.md 实施：新增
apply_environment_collision.py（1476 个 2.5-D 桌面体素盒 → 规划场景，夹具/工件减除区
15 mm；dry-run/remove/keep-workpiece），5 个纯逻辑测试通过；真机 ApplyPlanningScene
success。验收：**场景灌入后 grasp/lift 规划失败**（10-27 教训重现），根因是厂商
camera_link 占位盒在抓取姿态伸入插槽桌面带 8–20 mm（假碰撞）。按所有者批准的
"分段启用"退路落位：抓取段保持空场景（4b 只清空），转运段在抬升后启用场景
（9b，--keep-workpiece）。实验后场景已还原为空（0 对象）。遗留选项：描述切换回
仓库 wrapper（含实测相机外参），待所有者确认。

## 24. 真机描述定向修复 + 环境场景全流程启用（2026-11-02 夜）

所有者质疑 wrapper URDF 不准确后：逐行对比确认基础臂链/TCP 与 vendor 完全一致，
差异仅在工具+相机子树。定向修三处：①手指 link 碰撞移除（77 mm 占位，真机无此实体）；
②Gazebo 夹爪 ros2_control 块加 sim 门控（此前无条件内嵌导致真机驱动加载即崩溃——
10-26 改用 vendor 描述的真正原因）；③调用处传参。验收：新模型 + 1476 桌面体素盒场景下
pre_grasp/grasp/lift 规划全绿（昨天 grasp NO_COLLISION_FREE_IK），阴性对照仍被拦；
元凶确认为厂商 camera_link 占位盒。teaching_sequence 改回全场景启用（4b 灌场景）。
实验后场景已清空还原。详见 2026-11-02_PLAN_ENV_COLLISION_INTEGRATION.md §9。

## 25. 专项 2（携带工件建模）落地（2026-11-02 夜）

参照师兄 collision_object_publisher 实现 attach_workpiece.py（attach/detach+place/reset），
接入示教序列 7b/15b；修掉两个 MoveIt 陷阱（robot_state.is_diff、不存在 id 的 REMOVE），
实机往返验证通过，16 测试全过。详见 2026-11-02_PLAN_ENV_COLLISION_INTEGRATION.md §11。
下一步：专项 3（力控装配，参照 cs625_tcp_force_probe / cs625_compliant_placement）——
传感器未装前，插入段继续用 0.005 m/s + fault-poll 过渡。

## 26. 收工 + 专项 3 方案（2026-11-02 深夜）

所有者关闭全部硬件（机械臂/夹爪/相机）后，WSL 侧遗留进程已全部 SIGINT 干净退出。
专项 3 通读完成：师兄力控 = 关节力矩接触检测（Σ|τ| 残差滤波，0.8/2.0 两档阈值）
+ 50 Hz 状态机（APPROACH 0.30 / SOFT 0.10 / CONTACT_HOLD 0.05）+ set_speed_slider 节流
——关键结论：师兄的接触检测【不依赖六维力传感器】，我方 /joint_states effort 即可复现；
但其节流通道（speed_slider）对我方 ros2_control 无效，等价物是分段执行 + 每段降速
+ 力矩监督取消。方案（两期：A 无传感器过渡 / B 传感器升级）见
docs/worklogs/2026-11-02_PLAN_FORCE_CONTROL_INTEGRATION.md。
