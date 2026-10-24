---
title: "2026-10-20 WORKLOG：P3 RVS→WSL 位姿通道联调（真实 RVS）"
date: 2026-10-20
type: WORKLOG
status: done（传输 PASS；视觉匹配 NOT ACCEPTED）
---

# P3：RVS → WSL 位姿通道联调

## 一、本轮结论

1. **RVS → WSL 的 TCP 位姿通道已用真实 RVS 验证通过**：RVS 命令服务（7000）接收
   `T1/T2/T3` 命令，采集相机帧、加载手眼标定，并向 `127.0.0.1:9999` 发出报文；
   WSL 侧 `tcp_server` 收到报文（本轮为 `NG,POSE,0,0,0,0,0`）。
2. **视觉匹配 NOT ACCEPTED**：三组命令的 ICP 都报 `没找到匹配坐标`，因为当前机器人
   不在拍摄位、相机画面里没有可匹配的 box/slot。RVS 因此发送 `NG,POSE,0,...`，
   `tcp_server` 按"未知类型"记录（不以 S/R 开头）。
3. **规划链路（只规划）此前已通过**：用一条有效 S3 位姿，`/cs625/plan_to_pose_srv`
   规划到派生 pre-grasp / grasp 成功（约 0.02 s），未执行。
4. 结论：P3 的"通道"目标完成；"真实视觉位姿"被场景/相机位姿阻塞，属于 P4/P5 的
   前置条件，不是 TCP 问题。

## 二、关键事实

```text
RVS 版本与工程    : RobotVisionSuite Win 1.8.410
                  : CS625_SlotBox_VisualGuidance_RVS20260627-01_SENDTEST.rvs
RVS 命令端口      : 7000（TCPServerConnectionResource，auto_start=1）
RVS 位姿端口      : 9999（CommonTCPClientNode → tcp_server）
命令格式          : T1/T2/T3,x_mm,y_mm,z_mm,rx_deg,ry_deg,rz_deg（逗号分隔）
本轮收到          : T3、T1、T2 均被 TCPServerReceive 识别
RVS 相机          : TyCameraResource device_online: true
视觉结果          : rvs_icp: 没找到匹配坐标 → send: NG,POSE,0,0,0,0,0
tcp_server 行为   : 收到 'NG,POSE,0,0,0,0,0'，警告 Unknown msg type 'NG'
```

证据：`~/cs625_legacy_snapshot_20261020/p3_evidence_20261020/rvs_seam_20261020.log`
与 `tcp_server_p3.log`，以及两张 RVS 截图。

## 三、为联调做的事

1. 生成只改地址的副本 `..._WSL.rvs`：16 处旧 VM 局域网地址 → `127.0.0.1`，原工程未动。
2. 为调试组准备 `..._SENDTEST.rvs`：给 `rvsGroup_0/1` 的发送节点加了固定字符串
   EmitNode（原始 R1/S3 报文），以绕过空输入覆盖；本轮的 T1/T2/T3 测试仍走真实视觉组。
3. Windows UI 自动化：最小化/置前 RVS、截图、读取工具栏（运行/停止/单次运行）、
   启动工程；从 WSL 通过 PowerShell 向 RVS 7000 端口发送 T1/T2/T3 命令。
4. 归档证据；未连接机器人、未执行任何运动。

## 四、能力层与关键链状态

```text
Capability layer
- environment and dependencies : PASS（legacy underlay + RVS 1.8.410）
- robot model and simulation   : NOT INVOLVED（无 Gazebo/机器人）
- kinematics, control, planning: PASS（plan_to_pose 只规划）
- vision, hand-eye, TF         : PARTIAL（相机在线、手眼标定加载；ICP 未匹配）
- active perception / NBV      : NOT STARTED
- real hardware and safety     : NOT STARTED（无真机、无运动）

Critical-chain status
- RVS 命令 → RVS 组           : PASS（T1/T2/T3 被识别）
- RVS 位姿 → WSL tcp_server   : PASS（收到 NG,POSE）
- 有效位姿 → /target_pose     : NOT ACCEPTED（本轮无有效 S/R 位姿）
- /target_pose → MoveIt 规划  : PASS（此前有效位姿已验证）
- MoveIt 规划 → 执行          : NOT STARTED（未调用执行服务）
```

## 五、限制与下一步

1. **获取有效位姿需要相机在拍摄位**：T0/T2 先计算拍摄位，T1/T3 在拍摄位采集并匹配；
   当前机器人不在位，所以 ICP 失败。解决方式二选一：
   - 现场把机器人/相机摆到拍摄位（有人监督），再触发 T1/T3；
   - 或运行 `rvsGroup_0/1` 调试组（固定 R1/S3），验证"有效位姿 → 规划"。
2. **P4 必须处理 `NG,POSE`**：当前 `tcp_server` 丢弃；自动决策层应映射为
   `VISION_NO_MATCH`，触发重观测/重试，而不是继续规划。
3. 本轮未改仓库代码；`_SENDTEST.rvs` 与 `_WSL.rvs` 是外部副本，原工程未动。
4. 真机安全：本轮无真机连接、无运动；后续真机联调保持 `execute:=false` 与硬件急停。
