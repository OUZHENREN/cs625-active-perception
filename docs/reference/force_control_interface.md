# 力控接口清单与路线决策（2026-11-26 核查，全部有据可查）

> 起因：所有者问"阻抗还是导纳"，并要求把缺口补上。
> 本文只写**核查事实**与**决策依据**；不写猜测。每条事实给出文件与行号。

## 1. 事实一：厂商 SDK 具备完整力控能力（这是之前漏掉的）

`~/elite-sdk-1.2.0/include/Elite/EliteDriver.hpp`

| 能力 | 接口 | 说明 |
|---|---|---|
| 力控模式开关 | `enableForceMode(reference_frame, selection_vector, wrench, mode, limits)` | **直接在控制器内做力控**：给定期望 wrench、柔性轴选择向量、力坐标系、**速度限幅 limits[Vx,Vy,Vz,ωx,ωy,ωz]** |
| 力控坐标系 | `enum class ForceMode {FIX, POINT, MOTION, TCP}` | 固定系 / 点位系 / 运动方向系 / TCP 系 |
| 负载与质心 | `setPayload(double mass, const vector3d_t& cog)` | 给控制器设置末端负载质量与**质心（相对法兰）**，内部做重力补偿 |
| 力传感器清零 | `zeroFTSensor()` | 把当前值存为参考，后续测量减去该参考（tare） |
| 读实时力 | `get_tcp_force()`（脚本侧） | 与 KWR75 相互独立，可交叉校验 |

**注意**：`limits` 参数让我们把力控模式下的**笛卡尔速度**压到很小（例如 0.005 m/s），这与我们
"装配段 0.005 m/s"的口径一致——不是无限制的力控。

## 2. 事实二：底层流式位置接口**已经在用**（不是缺的环节）

`~/cs625_underlay_jazzy/src/eli_cs_robot_driver/src/hardware_interface.cpp:568`
```cpp
eli_driver_->writeServoj(position_commands_, recv_timeout_ * 1000, false, false);
```
- `servoj_time = 0.008 s`（**125 Hz**）、`servoj_gain = 300`、`servoj_lookahead_time = 0.1`
  （`hardware_interface.cpp:361–391`，默认值见 `EliteDriver.hpp:51–58`）
- 也就是说：ros2_control 的 position 命令**在每个控制周期都以 servoj 流式下发**，
  不是"整条轨迹一次性丢给控制器"。

**由此修正我之前的判断**：缺的不是"流式位置下发"，缺的是
**"新的目标点能多快被接受"**——即 action goal 的更新延迟。文献里 `RCMLJ2ED` 实测
位置型阻抗的真实上限是**重发 goal 的延迟（4 s）**，不是算法频率；而 `5UFZI35T` 用
**100 ms 停走节拍（10 Hz）**就完成了 0.3 mm 单边间隙、250 mm 深孔装配。
→ 所以"10 Hz 切片 + 短轨迹 goal"在原理上就够用，**不需要改造架构**，只需实测延迟。

## 3. 三条路线与决策

| 路线 | 做法 | 优点 | 代价/风险 | 结论 |
|---|---|---|---|---|
| **A 厂商力控模式** | `enableForceMode` 设期望 wrench + 柔性轴 + 速度限幅 | 控制器内闭环、响应快、成熟 | 绕过我们 `execute_assembly_chunks` 的关节速度治理器；需自己加看门狗（力阈值、关节区间、超时） | **作为基线对照**（论文第 5 章的 baseline） |
| **B 自建导纳（10–50 Hz 切片）** | 读 `/ft_sensor/wrench` → 导纳律算 Δx → 发短 `FollowJointTrajectory` goal | **算法是我们自己的**（论文主体，可加残差 RL）；安全治理器全程生效 | 受 goal 更新延迟限制，需先实测 | **主线方案**（符合"本文方法 vs 基线"的章节结构） |
| **C 直接 writeServoj 流式** | 自己起节点按 125 Hz 下发 servoj | 延迟最低 | **绕过 ros2_control 与我们全部安全闸**，违反执行安全契约 | **不采用** |

**决策**：主线走 **B（自建位置型导纳）**，A 作为**对照基线**，C 不采用。
理由：论文必须有自己的算法贡献；而厂商力控恰好提供了"同一台机器上的现成基线"，
这比引用别人的论文更有说服力。

## 4. 关键问题：厂商力控能不能用在我们这台"不自带力传感器"的 CS625 上？

**先说结论：能力在控制器里，但"是否授权启用"必须问机器；而且即使能用，也不该当主路线。**

支持"能用"的证据（代码级）：
```text
① RTSI 提供 getAcutalTCPForce()（RtsiIOInterface.hpp:210），
   官方驱动已把它读出来并作为 tcp_fts_sensor 状态接口发布
   （hardware_interface.cpp:141–151 发布 force.x/y/z、torque.x/y/z，:474 每周期更新）
   → 控制器内部**有 TCP 力估计**（基于关节力矩），所以"没装外置力传感器"不等于"无力控"
② SDK 真实签名（EliteDriver.hpp:304–306）：
     bool startForceMode(const vector6d_t& reference_frame,
                         const vector6int32_t& selection_vector,
                         const vector6d_t& wrench, const ForceMode& mode,
                         const vector6d_t& limits);
     bool endForceMode();
   其中 limits = [Vx,Vy,Vz,ωx,ωy,ωz] 是**速度限幅**，与我们装配段 0.005 m/s 口径一致
```

不确定性（必须实测，不能假设）：
```text
· 手册 3.6.6 描述的是控制器能力，**没有写是否选配/授权**；我们这台是否启用未知
· 基于关节力矩的 TCP 力估计精度远低于外置传感器（摩擦、模型误差，量级十几 N），
  对比 KWR75 的准度 0.5 %FS（≈2.5 N / 3.5 N·m）→ **它不能替代 KWR75**
· 我们的 ROS 驱动**没有暴露力控指令**（只有 tcp_fts_sensor 只读接口），
  所以启用它只能走 SDK（C++）或示教器，不能走我们的 ROS 栈
```

验证三步（成本递增，且都不发运动指令）：
```text
① 示教器：看"力控"菜单是否存在/可选（0 成本，最快）
② ROS 侧：读 tcp_fts_sensor 六个分量，手动按一下末端看是否有响应
   （`ros2 topic echo` 不行，这是 ros2_control 状态接口；可用现有控制器状态话题或写 10 行节点读）
③ SDK 探针：按上面的真实签名写一个最小程序调用 startForceMode（选择向量全 0、wrench 全 0、
   limits 取 0.005），看返回 true/false，随即 endForceMode()
   —— 注意 startForceMode 需要驱动已加载控制脚本，需参考 eli_cs_robot_driver 的 script 文件
```

**判定**：
| 场景 | 结论 |
|---|---|
| 若 ① 示教器里没有力控菜单 | 该选项未授权 → **厂商力控不可用**，直接走自建导纳 |
| 若 ① 有菜单但 ③ 返回 false | 同上 |
| 若 ③ 返回 true | **可用，但只作基线**：精度靠关节力矩估计，不如 KWR75；且绕过我们的治理器 |

**主路线不变**：自建位置型导纳（KWR75 供力，ROS 层闭环，安全治理器全程生效）。
厂商力控是"同一台机器上的现成基线"，能验证就用，不能验证就删掉这一栏。

## 5. 还缺的一件事：实测 goal 更新延迟

必须实测两个数（真机就绪时第一件事，脚本已备 `scripts/ft_latency_probe.py`）：
```text
① goal 下发 → 关节开始变化 的延迟（决定导纳最高能跑多少 Hz）
② 力超阈值 → 实际停止 的延迟（决定安全阈值要留多少余量）
```
判据：
- 若 ① ≤ 30 ms → 20–30 Hz 切片可行；
- 若 ① 在 30–120 ms → 10 Hz 停走节拍（按 5UFZI35T 的经验仍可完成 0.3 mm 间隙装配）；
- 若 ① > 300 ms → 切片方案不可行，只能退回 A（厂商力控）。

## 6. 与论文结构的关系

```text
第 5 章 力控：基线 = 厂商力控模式（A）；本文方法 = 自建导纳 + 残差补偿（B）
             对照指标：插入成功率、力峰值、分段用时、位置/角度残差
这样"基线 vs 方法"是同一台机器人、同一工件、同一间隙下的对照，证据强度最高。
```
