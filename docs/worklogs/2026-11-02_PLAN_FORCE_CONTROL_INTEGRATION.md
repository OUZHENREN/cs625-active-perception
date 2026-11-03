# 2026-11-02 PLAN 力控装配接入方案（参照师兄 cs625_compliant_placement）

> 专项目标：为插入段（示教序列步骤 14/15）补接触/卡死检测与降速保护，替代
> 当前"0.005 m/s + fault-poll"的被动防护。
> 依据：通读师兄 `cs625_tcp_force_probe`（76 行）与 `cs625_compliant_placement`
> （controller 119 / detector 73 / node 301 行，C++）。
> 状态：硬件已全部关停（机械臂/夹爪/相机），本方案为离线文档，待开工后实施。

## 1. 师兄实现拆解（源码结论）

```text
cs625_tcp_force_probe（76 行）
  /force_torque_sensor_broadcaster/wrench -> Float64MultiArray /cs625/tcp_force
  纯桥接；依赖六维力传感器激活（我方当前激活失败：ft_sensor/force.x 不可用）

cs625_compliant_placement（状态机 50 Hz）
  输入  cs625_state_monitor/CS625State：
        · joint_torques[6]  ← 接触检测用的是【关节力矩】，不是六维力传感器！
        · actual_tcp_x/y/z_mm、actual_rot_*_deg
  检测  接触检测器（contact_detector.cpp）：
        tau_metric = Σ|τ_i|
        baseline：慢自适应（α=0.01）；filtered：快滤波（α=0.2）
        residual = max(0, filtered - baseline)
        contact  = residual ≥ 0.8；safety = residual ≥ 2.0（阈值单位=力矩和）
  状态机（placement_controller.cpp）：
        IDLE → APPROACH(0.30) / SOFT_APPROACH(0.10) → contact → CONTACT_HOLD(0.05)
        → 距目标 ≤3 mm → DONE；safety → ABORTED
  节流通道  /io_and_status_controller/set_speed_slider —— 只对机器人【内部运动】
        生效（师兄执行链是 robot_commander 60000 口 + 机器人本体），
        对我方 ros2_control arm_controller 无效（其日志明确 "No scaling interface set"）
  接口  服务 start_compliant_placement(target mm/tolerance/开关) / stop / get_state
  缺口  卡死判定参数存在（progress_window 0.3 s、epsilon 0.5 mm、jam_confirm 10），
        但 controller 源码里从未置位 jam_detected —— 师兄的卡死判定疑似未完成
```

## 2. 与我方执行链的差异（照搬不可行的两点）

```text
1. 节流通道不同：师兄=机器人 speed_slider；我方=ros2_control 轨迹执行。
   我方等价物 = 现有 SEGMENTED 分段执行 + 每段 time_scale 调整 + 力矩监督取消。
2. 力矩来源不同：师兄走自建 cs625_state_monitor；我方 /joint_states 的 effort
   字段已由 joint_state_broadcaster 发布（实测有效值存在）——今天就能拿到 Σ|effort|，
   无需等六维力传感器。
```

## 3. 方案（分两期）

### A 期：无传感器过渡（现在即可实施）

新增 `scripts/compliant_insert.py`（监督式，复用现有分段执行器）：

```text
输入  目标 TCP(实时 R1 链的 insert_tilted)、tolerance 3 mm、阈值参数
流程
  0  静止基线：采集 /joint_states effort 1 s，Σ|effort| 均值 → baseline
  1  按 SEGMENTED 分段执行插入轨迹（首段 time_scale 2.0 起步）
  2  每段执行中 50 Hz 采样 effort：
       residual = max(0, 滤波(α=0.2) - 慢基线(α=0.01))
       residual ≥ contact 阈值 → 下一段 time_scale 加倍（等价 SOFT→CONTACT_HOLD）
       residual ≥ safety 阈值 → 立即 cancel 当前 goal + 沿插入轴反向退回 20 mm + ABORT
  3  每段到位校验（复用 verify 门）+ TCP 距目标 ≤3 mm → DONE
输出  与 execute_assembly_chunks 同风格的结果行 + /cs625/compliant_insert/state 话题(可选)
```

接入：teaching_sequence 步骤 14 改调本脚本（保持 --allow-release 语义不变）。
safety 阈值初值与单位需开工后实测标定（见 §5 决策点）。

### B 期：六维力传感器到货后升级

```text
接触信号源  Σ|effort| → F/T 力/力矩残差（CS625State.actual_tcp_wrench，
            消息结构已预留 has_actual_tcp_wrench 标志）
阈值        按真传感器数据重标定（接触/安全两档）
状态机      不变——A 期验证过的监督框架直接换数据源
```

## 4. 验收阶梯

```text
1. 离线：单元测试（残差滤波、状态机转移、阈值触发）——纯逻辑可全离线
2. 开工后：无工件空跑下降，记录 effort 噪声基线，定 contact/safety 阈值
3. 有工件实插：正常插入不触发 safety（≤DONE）
4. 故障复现：把工件偏转 7.71°（10-27 过载条件）再插——必须在 robot FAULT 之前
   由 safety 检测取消并退回，而不是打到 safety_mode=9
```

## 5. 决策点（待所有者）

1. safety 触发后的动作：仅停（cancel goal）还是自动沿插入轴反退 20 mm？
   （10-27 教训：退出要沿原路反向，不重新规划——方案按"反向退回"设计）
2. contact 触发后的降速策略：沿用师兄的 0.10 → 0.05 两级，还是只做一级？
3. 阈值初值：开工后先跑空载基线，再由所有者拍板接触/安全两档数值。

## 6. 相关事实（避免返工）

- 六维力传感器【不是】本方案的前置：师兄的接触检测本身就是关节力矩；传感器
  只是 B 期的信号升级。
- 插入目标姿态已由"实时 R1 链"修正（13 mm 常量偏差，10-31 已改）；工件偏转导致
  的姿态失配仍是 open question——接触/卡死检测是兜底保护，不是对齐解法。
- 10-27 的过载根因（固定插入位 + 工件被转 7.71°）在 A 期验收第 4 条直接复现验证。
