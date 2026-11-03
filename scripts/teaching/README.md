# 示教链路（RVS 给位姿）— 独立于六类模式

本目录是 **2026-10-27 在真机上跑通的那条程序**的完整记录（2026-10-30 复验：观测前缀
T0–T3 四步全部跑通，抓取位已由当天 S3 实时算出），用来给后续自动化做示教。
它与另外两套代码严格分开：

```text
师兄源码   ~/cs625_legacy_underlay_jazzy/        只读，不修改；通过 ROS 话题调用
六类模式   scripts/run_task.py + config/tasks/    不受本目录影响，本目录也不调用它
本示教链路 scripts/teaching/                      只依赖 MoveIt、驱动、夹爪桥
```

## 组成

```text
scripts/teaching/rvs_pose_client.py    触发 RVS 的 T0/T1/T2/T3，抓取回传位姿并存 JSON
scripts/teaching/gripper.py            夹爪：status / init / grip（张开到抓取宽度）
scripts/teaching/run_teaching_chain.py 按步执行：observe / pre-grasp / enter / grip / lift
```

## 数据流

```text
RVS(Windows)                        本链路(WSL)                         机器人
-----------                         ----------                          -----
命令口 7000  <-- tool0 位姿 <--     rvs_pose_client.flange_pose()
   |                                      |
   +--> tcp_server --> /box_target_pose(S3) /box_grasp_tcp_pose /box_pre_grasp_tcp_pose
   |                                      |
   |                                 MoveIt 规划 + 一次下发执行（末段降速、到位自检）
   |                                      |
   +--> 夹爪桥 hitbot_cmd.ps1 <--------   init / grip(50 mm，张紧式抓取)
```

## 坐标系约定（本机实测，务必记住）

```text
URDF:  flange -> tool0             = (0, 0, 0)
       flange -> my_end_effector_link(TCP) = (-0.046, 0, 0.353)

师兄 pose_sender_node 发给 RVS 的是 tool0 位姿；
/tcp_pose_broadcaster/pose 给的是 TCP；
本机 TF 有多棵树，base_link->tool0 解析不到，因此用 TCP 位姿 + 上面的固定偏移换算。

RVS 侧手眼 DepthToRobotTCP ≈ (0.0968, 0.0178, 0.0973) m / Rz(-90.5°)
我方 T_flange_cam           ≈ (0.0970, 0.0422, 0.0975) m / Rz(-90.7°)
=> RVS 以【法兰】为参照，不是 TCP。
```

## 抓取动作顺序（唯一被验证安全的顺序）

```text
1. pre-grasp   夹爪已 init（闭合，接近 0）
2. init        到位前执行，jaws 参考到 ~0
3. enter       夹爪闭合状态低速进入抓取位，实测偏差 < 5 mm
4. grip        张开到 grip_width_mm = 50 mm —— 这是张紧式抓取，撑住扶手
5. lift        抬升 0.15 m，这是"抓到没有"的唯一证据
```

**禁止**：夹爪张开时进入工件区域。2026-10-27 这样做撞倒扶手并触发急停。

## 已知限制

```text
· RVS 与 ROS 相机互斥：RVS 运行时必须先停 ROS 相机节点
· RVS_AI_Center 必须在跑（推理口 2024），否则 2D 节点报「TCP 等待连接超时」
· 双向控制链路要 3 条 ESTAB（50001/50003/50004）；急停后必须重连
· 规划场景当前清空了桌子，只做抓取；桌子模型与实测差 12–65 mm，待重做
· 未实现：插入段（预插入/插入/转正/释放）尚未纳入本目录
· 2026-10-30 事故：预抓取下降段撞到工件桌。两个原因——
  ① --time-scale 0.5 被误当作半速，实为倍速（执行器是把时间戳×该值，>1 才是慢）；
     execute_assembly_chunks.py 已加守护：<1.0 直接拒绝。
  ② /tcp_pose_broadcaster 发的是【控制器示教器 TCP】（硬件状态接口，非 URDF FK），
     与 MoveIt 规划的 my_end_effector_link 在"工具下翻"姿态的关系未验证
     （事故中段实测同关节下两者 z 相差 ~630 mm）。
     在查清二者关系 + 把工件桌灌进规划场景之前，【禁止】任何下降到 z<0.2 的运动。
```

## 姿态必须由 S3 反算

```text
R_grasp = R_S3 × diag(-1, 1, -1)
2026-10-27：工件相对 10-22 参考绕竖直轴转了 7.71°，fitness 0.91 属良好，
因此角度差不是匹配精度问题，而是工件真的被转过，必须用实时 S3。
```

## 用法

```bash
source scripts/source_dev_env.sh --full

# 1) 触发一次精细识别（前提：RVS 工程已打开并在运行，机器人停在精观测位）
python3 scripts/teaching/run_teaching_chain.py observe

# 2) 到预抓取位
python3 scripts/teaching/run_teaching_chain.py pre-grasp

# 3) init 夹爪 + 闭合状态进入抓取位
python3 scripts/teaching/run_teaching_chain.py enter

# 4) 张开到抓取宽度（撑住扶手）
python3 scripts/teaching/run_teaching_chain.py grip

# 5) 抬升验证
python3 scripts/teaching/run_teaching_chain.py lift
```

## 前置条件与已知限制

```text
· RVS 与 ROS 相机互斥：RVS 运行时必须先停 ROS 相机节点
· RVS_AI_Center 必须在跑（推理口 2024），否则 2D 节点报「TCP 等待连接超时」
· 双向控制链路要 3 条 ESTAB（50001/50003/50004）；急停后必须重连
· 规划场景当前清空了桌子，只做抓取；桌子模型与实测差 12–65 mm，待重做
· 未实现：插入段（预插入/插入/转正/释放）尚未纳入本目录
```


## 固定与动态（2026-10-30 所有者确认，附实测依据）

```text
固定（常量，config/tasks/common.yaml）：
  slot_coarse      (0.7802, -0.1458, 0.7986)
  workpiece_coarse (0.5886,  0.3482, 0.7075)

动态（必须 RVS 给，不能写死）：
  T0 → S0  插槽精拍摄站位      T1 → R1  插槽位姿
  T2 → S2  工件精拍摄站位      T3 → S3  工件位姿 → grasp/pre_grasp（pose_math 换算）

实测（2026-10-30）：站固定 slot_fine 常量 → T1 回 NG,POSE,0,0,0,0,0；
站 RVS 给的 S0 → T1 3.6 s 返回 R1。RVS 响应时间当天恶化到 3–4 min（T2 实测 229 s），
客户端超时已默认 300 s。
```

## RVS 多触发与插槽对比（2026-10-30 所有者确认）

```text
多触发：rvs_pose_client.py 每次采集至少 3 帧有效回传才停，最多触发 10 次（--min-frames 3
        --max-attempts 10）。有效性 = 位置在工作空间内 + 四元数模长 ≈1；
        垃圾帧（10-30 实测 T0 曾连续两次回 x=4.4 m）会被存档但不计入帧数；
        3 帧之间打印最大 pairwise 偏差（2026-10-30 T3 实测 0.2 mm / 0.04°）。
插槽对比：teaching_sequence.py 每次拿到 R1 后调用 check_slot_history()：
        与六模式参考（由 10-27 已证的插入常量反推，槽姿 ≈ (1.2394, 0.1798, 0.4446)、
        姿态单位四元数）及历史 R1（runtime/rvs_teaching/slot_history.json）对比，
        打印位置 mm / 姿态 deg 偏差；>30 mm 或 >3° 打印 WARN。只报告、不中断。
        10-30 实测：两条 R1 与参考差 20.4 / 23.3 mm，0.9–1.3°（插槽确实近乎固定）。
```

## 一次通过的完整程序（`teaching_sequence.py`）

所有失败过的动作都不在程序里，顺序固定为：

```text
0  环境建模（离线）
1  插槽粗观测 T0         固定粗采样位 → RVS 给 S0（精拍摄站位）
2  插槽精观测 T1         移到 S0（动态）→ RVS 给 R1        ← 之前漏掉
3  工件粗观测 T2         固定粗采样位 → RVS 给 S2（精拍摄站位）
4  工件精观测 T3         移到 S2（动态）→ RVS 给 S3
5  预抓取位              由 S3 实时换算（动态）
6  init + 闭合进入抓取位 （夹爪必须已闭合）
7  张开夹爪 50 mm（张紧式抓取）
8  抬升 0.15 m
9  抬到安全高度 0.53
10 transit（不可跳过）
11 staging
12 纯旋转 −10°（位置不变）
13 纯下降（姿态不变）
14 插入 0.005 m/s
15 整平 0.1 m/s
16 释放（必须 --allow-release）
17 纯平移退出
18 baseline
```

位姿换算（与师兄 tcp_server 一致，`pose_math.py` 已复现并比对通过）：

```text
grasp     = box_pose  × T_b_tcp        T_b_tcp = diag(-1,1,-1) + (-0.004, -0.215, -0.411)
pre_grasp = grasp     沿 TCP x +120 mm
insert    = slot_pose × T_s_tcp        T_s_tcp = diag(-1,1,-1) + (-0.082, -0.193, -0.4125)
rotated   = insert    × Ry(−10°)
pre_insert= rotated   沿 TCP z −350 mm
```

速度（所有者 2026-10-27 确认）：只有 `insert`（与卸载的 d5_extract）用 0.005 m/s；
`straighten` 可用 0.1 m/s；其余按 0.02–0.08 的接近速度。

```text
pre_grasp 0.02 / 1.0      grasp 0.01 / 1.5 (+尾段 30 mm 降速)
lift 0.03 / 1.5           transit 0.08 / 1.0
staging 0.05 / 1.5        pre_insert 0.03 / 1.5 (+尾段 60 mm 0.010)
insert 0.005 / 3.0        straighten 0.1 / 1.0
withdraw 0.03 / 1.5 (0–80 mm 带内降速)      baseline 0.08 / 1.0
```

用法：

```bash
python3 scripts/teaching/teaching_sequence.py                 # 只打印步骤
python3 scripts/teaching/teaching_sequence.py --confirm       # 实际执行（到 15 步停止）
python3 scripts/teaching/teaching_sequence.py --confirm --allow-release   # 含释放
```
