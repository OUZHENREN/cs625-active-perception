# 机械臂状态码速查（真机调试用）

> 建立：2026-11-12。来源：`ros2 interface show eli_common_interface/msg/SafetyMode|RobotMode`，
> 以及 2026-11-11 的一次真实事故（松开急停按钮后驱动仍报 `safety_mode=7`，导致
> 力矩通道读数为 0、所有接触检测失效）。

## 1. 怎么看

```text
两个话题都是 transient_local（latched），普通订阅收不到历史那条，必须显式声明 durability：

  import rclpy
  from rclpy.node import Node
  from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
  from eli_common_interface.msg import SafetyMode, RobotMode
  qos = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                   reliability=ReliabilityPolicy.RELIABLE,
                   durability=DurabilityPolicy.TRANSIENT_LOCAL)

  /io_and_status_controller/safety_mode  → int8 mode
  /io_and_status_controller/robot_mode   → int8 mode

最快的命令行等价写法：
  ros2 topic echo /io_and_status_controller/safety_mode --once
  ros2 topic echo /io_and_status_controller/robot_mode --once
```

## 2. safety_mode（安全模式）

| 值 | 名称 | 含义 | 怎么办 |
|---|---|---|---|
| -2 | UNKNOWN | 还没读到 | 等 1–2 s 再读；驱动可能没起来 |
| **1** | **NORMAL** | **正常，可运动** | 只有这个值才允许跑流程 |
| 2 | REDUCED | 降速模式（安全限速） | 可运动但速度受限，检查是否切换了模式 |
| 3 | PROTECTIVE_STOP | 保护停止（撞到/力矩超限） | 示教器复位；查碰撞、查负载 |
| 4 | RECOVERY | 恢复中 | 等它回到 1 |
| 5 | SAFEGUARD_STOP | 安全防护停止（光幕/门） | 排查安全装置后复位 |
| 6 | SYSTEM_EMERGENCY_STOP | 系统级急停 | 复位急停按钮后重启驱动栈 |
| **7** | **ROBOT_EMERGENCY_STOP** | **本体急停** | **松开急停按钮 → 示教器复位 → 若仍为 7 则重启驱动栈** |
| 8 | VIOLATION | 违反安全限制 | 查安全配置/关节限位 |
| 9 | FAULT | 故障 | 记录代码，联系厂商 |
| 10 | VALIDATE_JOINT_ID | 关节 ID 校验 | 重启控制器 |
| 11 | UNDEFINED_SAFETY_MODE | 未定义 | 重启驱动栈 |
| 12 | AUTOMATIC_MODE_SAFEGUARD_STOP | 自动模式安全停止 | 切回手动/复位 |
| 13 | SYSTEM_THREE_POSITION_ENABLING_STOP | 系统三位使能停止 | 检查使能开关 |
| 14 | TP_THREE_POSITION_ENABLING_STOP | 示教器三位使能停止 | 示教器使能键按下/复位 |

## 3. robot_mode（机器人模式）

| 值 | 名称 | 含义 | 怎么办 |
|---|---|---|---|
| -2 | UNKNOWN | 尚未读到 | 等 1–2 s |
| -1 | NO_CONTROLLER | 找不到控制器 | 机器人是否上电/网线是否通 |
| 0 | DISCONNECTED | 已断开 | 查网络、重启驱动 |
| 1 | CONFIRM_SAFETY | 需要确认安全 | 示教器上确认 |
| 2 | BOOTING | 启动中 | 等 |
| 3 | POWER_OFF | 未上电 | 上电 |
| 4 | POWER_ON | 已上电但未使能 | 使能（示教器/服务） |
| 5 | IDLE | 空闲（未使能运行） | 正常空闲；要运动需要使能到 7 |
| 6 | BACKDRIVE | 拖动示教/无力矩 | 退出拖动模式 |
| **7** | **RUNNING** | **运行中（可运动）** | 正常 |
| 8 | UPDATING_FIRMWARE | 固件升级 | 等 |
| 9 | WAITING_CALIBRATION | 等待标定 | 先完成标定 |

## 4. 常见组合与含义

```text
(1, 7)  正常可动                        ← 开工要看到这个
(1, 5)  安全正常但未使能                  ← 需要使能
(7, 7)  急停 + 控制器仍在运行             ← 急停按钮被按下；松开后要先复位
(7, 5)  急停 + 空闲                      ← 同上，且未使能
(3, 7)  保护停止                         ← 撞到/力矩超限；示教器复位
(-2, -2) 读不到                          ← 驱动栈没起或话题名不对
```

## 5. 与执行器的关系（重要）

```text
· EFFORT_DEAD：力矩通道读数为 0（阈值 <1.0 N·m）时，execute_assembly_chunks 拒绝执行。
  急停态下驱动不再上报有效力矩，所以"急停 → 力矩死流 → 执行器拒绝"是一条因果链。
· ESTOP_OR_PROTECTIVE_STOP：运动途中 safety_mode 变成 3/6/7 时，执行器立刻取消轨迹并返回。
  2026-11-11 的真机运行就是被这条拦下的（比继续走安全得多）。
· 开工脚本 scripts/start_real_stack.sh 现在会在健康检查之后读 safety_mode/robot_mode，
  safety≠1 时直接告警并停止后续动作。
```

## 6. 一句话排查顺序

```text
读 (safety, robot) → 若 (7,*)：松开急停+复位；仍 7 → 重启驱动栈
                  → 若 (1,5)：使能到 RUNNING
                  → 若 (-2,-2)：驱动栈没起
                  → 都正常再看 preflight（力矩流是否活）与 /joint_states
```
