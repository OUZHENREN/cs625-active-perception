# 2026-10-27 WORKLOG RVS 示教链路（独立于六类模式）

## 目的

所有者要求：先用「确定能跑通」的 RVS + RVSAI 采集工件/插槽位姿，跑通抓取—插入示范，
把师兄那条线作为**示教**复现；ROS 侧定位后续再逐步替换。该示教链路**独立于
`scripts/run_task.py` 的六类模式**，作为前期自动化示教模式，边走边在工作区复验。

## 运行组成

```text
Windows 侧：
  RobotVisionSuite（工程 CS625_SlotBox_VisualGuidance_RVS20260627-01_WSL.rvs）
  命令口 7000；RVS_AI_Center 推理口 2024
WSL 侧：
  tcp_bridge/tcp_server（师兄件，读取 RVS 回传并发布 ROS 位姿话题）
  scripts/rvs_pose_client.py（本轮新增：触发 T0/T1/T2/T3 并记录位姿）
  机器人驱动 + MoveIt（cs625_bringup real_base.launch.py）
```

## 已验证的关键事实

```text
1. RVS 与 ROS 相机互斥：RVS 运行时 ROS 相机节点必须停止。
2. 师兄 pose_sender_node 发给 RVS 的是 tool0 位姿（TF 查询 base_link→tool0）。
3. 本机 TF 存在多棵树，base_link→flange / tool0 无法解析
   → 改用 /tcp_pose_broadcaster/pose + URDF 固定偏移换算。
4. URDF：flange→tool0 = 0；flange→my_end_effector_link(TCP) = (-0.046, 0, 0.353)。
5. RVS 侧手眼 DepthToRobotTCP ≈ (0.0968, 0.0178, 0.0973) m，等价 Rz(-90.5°)
   与我方 T_flange_cam (0.0970, 0.0422, 0.0975) / Rz(-90.7°) 一致
   → **RVS 参照的是法兰系**，不是 TCP。
6. T2 约 11 s、T3 约 17 s；失败时回 'NG,POSE,0,0,0,0,0'（tcp_server 不转发 NG）。
7. 从已验证的精观测位触发 T3，抓取位 z = -0.058，与 10-22 实测 -0.0583 一致。
8. 今天的 S3 相对 10-22 记录绕竖直轴转了 7.71°（工件被真实转动），
   fitness 0.91 属良好 → 角度问题来自工件姿态，不是匹配精度。
   抓取姿态应由 R3 反算：R_grasp = R_S3 × diag(-1,1,-1)。
```

## 本轮事故与铁律（新增）

```text
事故：在预抓取位把夹爪张开到 27 mm 后直接朝抓取位推进，碰到扶手并触发急停。
铁律：进入抓取位之前，夹爪必须【已初始化且闭合到抓取宽度】；
      张开的夹爪绝不能进入工件区域。
教训：急停后控制器会话丢失，必须重连（recover_control.sh / resend）后才能再动。
```

## 执行层的改动（已验证）

```text
scripts/execute_assembly_chunks.py
  · 取消「长轨迹切多段」的用法：整条轨迹一次下发
  · 新增 --tail-fraction / --tail-scale：末段按比例放慢，保持路径平滑
  · 新增 --verify-final：执行后比对实测关节与末点，超差即失败
实测：一次下发 136 点轨迹，到位偏差 0.0002 rad。
```

## 当前未完成 / 下一步

```text
1. 确认「入库前闭合夹爪」的正确宽度与顺序（grip 命令 0→50 为张开方向，需现场确认）。
2. 规划场景：桌子灌入后抓取位规划失败（实测历史位姿也失败），
   当前做法是清空场景只做抓取；后续需按实测模型重生成场景并复核间隙。
3. 抓取成功后：抬升 15 cm → 中转 → 预插入 → 插入 → 转正（沿用师兄命令链）。
4. 该示教链路的脚本后续整理为独立于六类模式的入口。
```


## 程序已落成工作区独立目录（所有者要求）

```text
scripts/teaching/                       ← 示教链路专用，独立于六类模式与师兄源码
  rvs_pose_client.py                    触发 T0/T1/T2/T3，记录位姿 JSON
  gripper.py                            夹爪 status / init / grip（张开到抓取宽度）
  run_teaching_chain.py                 observe / pre-grasp / enter / grip / lift
  README.md                             坐标系约定、动作顺序、前置条件、已知限制
```

与另外两套代码的边界：

```text
师兄源码 ~/cs625_legacy_underlay_jazzy/  只读，不修改；仅通过 ROS 话题与 7000 端口交互
六类模式 scripts/run_task.py + config/tasks/  与本目录互不依赖
本目录   仅依赖 MoveIt、机器人驱动、夹爪 Windows 桥
```

## 抓取动作的正确顺序（2026-10-27 修正）

```text
关键修正：这是【张紧式抓取】——闭合状态进入，到位后【张开夹爪撑住扶手】。
之前到位后没有张开夹爪，所以没有抓住工件。

正确顺序：
  1. pre-grasp   夹爪已 init（接近 0）
  2. init        进入前执行
  3. enter       夹爪闭合，低速进入抓取位（实测偏差 0.0000 rad）
  4. grip        张开到 grip_width_mm = 50 mm（config/tasks/common.yaml 的既有值）
  5. lift        抬升 0.15 m，唯一能证明抓到的动作

禁止：夹爪张开时进入工件区域（当日因此撞倒扶手并触发急停）。
```

## 实测执行数据（本轮）

```text
预抓取位规划 26 点、抓取位 62–75 点、抬升 60 点
一次下发执行，到位偏差：预抓取 0.0001 rad、抓取 0.0000 rad、抬升 0.0000 rad
急停后 recover_control.sh + resend 可恢复 3 条端口
```


## 一次通过的完整程序（2026-10-27 晚定稿）

所有者要求：最终示教程序必须一次通过，因此**所有失败过的动作都不写入程序**：

```text
失败动作（已排除）                      对应修正
夹爪张开时上升                          必须先 init（闭合）再动
跳过 transit 直奔 staging               顺序固定：抬升→transit→staging
边旋转边下降                            先用纯旋转到位，再纯下降
到位后不张开夹爪                        到位后张开 50 mm 才算抓住
```

补齐所有者指出遗漏的三项（参照师兄源码）：

```text
· 插槽粗观测位   config/tasks/common.yaml: slot_coarse  [0.7802,-0.1458,0.7986]
· 插槽精观测位   config/tasks/common.yaml: slot_fine    [0.7036,-0.0945,0.8146]
· 环境建模       scripts/model_environment.py（离线，不动机器人）
  对应师兄实现： environment_point_cloud_publisher + environment_collision_generator
  （launch 里 slot_mesh/box_mesh + enable_*_subtraction，world_frame=world）
```

新增文件：

```text
scripts/teaching/pose_math.py          复现师兄 tcp_server 的位姿换算
  grasp      = box_pose  × T_b_tcp   (diag(-1,1,-1) + (-0.004,-0.215,-0.411))
  pre_grasp  = grasp 沿 TCP x +120 mm
  insert     = slot_pose × T_s_tcp   (diag(-1,1,-1) + (-0.082,-0.193,-0.4125))
  rotated    = insert × Ry(-10°)
  pre_insert = rotated 沿 TCP z -350 mm
  校验：用 17:01 的 S3 复算得到 grasp (0.7882,0.6623,-0.0576)
        与师兄 tcp_server 打印的 (0.788,0.662,-0.058) 一致

scripts/teaching/teaching_sequence.py  19 个步骤的一次通过版本
  --confirm 执行；--allow-release 才做第 16 步释放
```

速度（所有者确认）：

```text
只有 insert（卸载的 d5_extract 同理）保持 0.005 m/s；
straighten 提高到 0.1 m/s；
其余：pre_grasp 0.02 / grasp 0.01 / lift 0.03 / transit 0.08 /
      staging 0.05 / pre_insert 0.03 / withdraw 0.03 / baseline 0.08
```


## 插入段过载与三处修正（2026-10-27 晚）

```text
现象：插入到 z≈0.253 时 safety_mode=9(FAULT) 越插越紧触发过载/碰撞保护
根因：工件在桌上被转过 7.71°（抓取姿态已按 S3 修正），
      但插入仍用固定的 pre_insert/insert 坐标 + 固定插入姿态
      → 夹持姿态与插槽入口方向不匹配，越插越挤

确认（所有者）：
  1. 插槽是固定位姿（slot_origin 常量 × T_s_tcp），不是变量
  2. 故障/急停/过载的发现要从 2-3 分钟压缩到 20 秒内
  3. 退出要沿原路反方向，不能重新规划

修正：
  1. execute_assembly_chunks.py 增加 --fault-poll 0.2s 高频轮询 safety/robot 模式
  2. 增加 --max-wait，但发现"一视同仁 20s"会误杀长行程
     → 改为：结果等待按 duration+20s，max-wait 仅作可选硬顶
        （否则 272 点的退出轨迹会在 20s 被误判超时）
  3. 退出用反向同轴：沿插入方向纯抬升回 pre_insert 高度

待解决：
  · 插入姿态如何与固定插槽对齐（工件偏转 7.71° 的处理）
  · 是否需要先"转正工件"再插入，还是按插槽坐标系重新计算插入姿态
```
