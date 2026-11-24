# 2026-11-24 工作日志：效率模式抓取链首次打通 + 全链路验证

> 本日目标（所有者定）：验证离线视觉流程（不依赖大模型）能稳定识别工件位姿并到达预抓取位；
> 插槽位姿沿用既有示教位姿与拍摄步骤。
>
> **结果：抓取链首次端到端打通**——粗观测 → 选站 → 精测 → 推导抓取点 → 预抓取位 → 下降到位，
> 全程实测通过，到位精度 0.01 mm。但**不做完整搬运**（J4 抱闸未修复）。

## 0. 时间线

| 时间 | 做了什么 | 结果 |
|---|---|---|
| 11:5x | 开工三项验证（时钟/控制链/站位表） | 全部通过；10 个 tier-1 站位在位 |
| 12:0x | 粗测工件 yaw −9.2°，跑选站 | **发现选站排序 bug**：选中差 9.4° 的站位，正确的差 0.4° 的排第 3 |
| 12:1x | 修 `nbv_views.rank_key`：`blocked` 没进排序键 | 修完正确选中 −9@20261120 |
| 12:2x–13:0x | 预抓取位推导反复失败（IK 无解 / 奇异 / 翻转） | 逐一排查，见 §2 |
| 13:0x | 发现 RVS 无法作真值（fitness 0.672、请求位姿不可达） | 放弃 RVS 路线 |
| 13:3x | 所有者手工摆出并验证抓取位姿 | 记录 TCP/关节/工件位姿 |
| 14:4x | 算出 `grasp_in_workpiece_frame`，往返误差 0.000 mm | 抓取点定义确定 |
| 15:0x | 放宽肩部限位 −110→−130、TCP 框 z 下限 0.05→−0.08 | 抓取位可规划 |
| 15:1x–15:3x | **完整链路验证**（8 步） | 全部通过，见 §1 |
| 15:5x | 修正预抓取位：外移 10 mm（−Y）+ 接近段减速 50% | 间隙 8.1→17.2 mm |
| 16:0x | 效率模式 `grip_targets` 改用验证值 + 相对变换 | 与 config 差 0.31 mm |
| 16:1x | 夹爪缓慢退出工件（半速 1.9°/s） | 到位精确 |

## 1. 完整链路验证结果（本日核心）

| 步骤 | 指标 | 结果 |
|---|---|---|
| ① 移到粗观测位 | 4 段 | 全部 SUCCESSFUL |
| ② 粗测工件 | yaw / 覆盖率 | **−7.7° / 0.896**，2/3 组 |
| ③ 选站 | 站位 / 偏差 / 摆放检查 | `-9@20261120`，差 1.1°，7.7 mm / 1.11° ✓ |
| ④ 移到站位 | 2 段 | 全部 SUCCESSFUL |
| ⑤ 精测 | 覆盖率 / 重复性 | **0.97** / **0.1 mm、0.045°** |
| ⑥ 推导抓取点 | 与 config 差 | **0.31 mm** |
| ⑦ 到预抓取位 | 到工件点云间隙 | **17.2 mm**（外移前 8.1 mm） |
| ⑧ 下降到抓取位 | 速度 / 到位 | **1.9°/s（半速）**，精确 |

## 2. 本日踩的坑与真因

### 2.1 选站排序漏了摆放检查（已修）

```text
现象：yaw −9.2° 时选中 0@20261120（差 9.4°，覆盖率 0.975），
      而正确的 −9@20261120（差 0.4°，覆盖率 0.968）排第 3，随后被摆放检查拒绝
真因：nbv_views.rank_key 算了 blocked 却没放进排序键
修复：排序键改为 (tier, has_coverage, blocked, -coverage, error_deg, delta_mm)
```

### 2.2 `workpiece_reference` 过时 144.6 mm（已修）

```text
反推 config 抓取常数隐含的工件位置 = (0.6417, 0.6755, 0.1244)
  与当前实测 (0.6455, 0.6313, 0.1203) 差 44.6 mm
  与旧 reference (0.5029, 0.6351, 0.1254) 差 144.6 mm
→ 抓取常数是在"接近当前位置"标定的，reference 记错了位置
   今天所有基于旧 reference 的推导都偏了 14 cm
修复：reference 更新为验证抓取时测到的工件位姿
```

### 2.3 示教链的 `T_B_TCP` 与我们的测量约定不兼容（已绕开）

```text
teaching_task_poses() / grasp_from_box() 内部用 T_B_TCP = diag(-1,1,-1) + (-0.004,-0.215,-0.411)
属于 RVS 的 "box 系"，与 measure_object 输出的 CAD 系差 673 mm
→ 抓取点推导不能经过示教链，改用实测相对变换
```

### 2.4 真实桌面在 z = −0.11，不是 +0.12（认知纠正）

```text
我曾判断"pre_grasp z=0.09 低于桌面、常数有误"——错的
桌面实测 z = −0.11（机器人底座高于桌面），所以：
  grasp z = −0.058 → 桌面上 5 cm ✓
  pre_grasp z = +0.092 → 桌面上 20 cm ✓
判据：先看 [table] 桌面在 base z=... 再判断高度是否合理
```

### 2.5 预抓取位必须侧移 10 mm（所有者指出，已修）

```text
纯垂直下降会让夹爪电机保护座蹭工件表面，下降走不完
方向要靠实测判断：+Y 最近工件点 0.8 mm（会撞）、−Y 17.2 mm（安全）
最终 pre_grasp = grasp + (0, −0.010, +0.150)
```

### 2.6 规划场景里没有工件和桌子（未解决，已记录）

```text
/get_planning_scene 返回 world.collision_objects = 空
→ 所有"碰撞校验：无接触"只覆盖自碰撞，对工件/桌面无效
→ 贴工件的动作改用点云最近距离判断（<5 mm 视为风险）
```

### 2.7 保护停止后反控通道静默失效（已记录）

```text
现象：ESTAB 仍显示 3 条连接，但机器人不执行任何指令（PATH_TOLERANCE_VIOLATED）
修复：ensure_robot_ready --force 重发外部控制脚本（重发前确认记忆偏差 ≤2°）
本日出现 2 次，均在急停/保护停止之后
```

### 2.8 关节限位比真机验证过的姿态更严（已放宽）

```text
肩部：−110 → −130（所有者验证的抓取位姿是 −124.52）
肘部：−100 → −60（真机验证过 −107.2 / −101.7 / −91.9）
TCP 框 z：0.05 → −0.08（桌面 −0.11，抓取位 z=−0.060 需更低）
```

## 3. 本日新增/修改的文件

```text
config/tasks/common.yaml
  · grasp / grasp_q / pre_grasp / workpiece_reference 全部更新为验证值
  · 新增 grasp_in_workpiece_frame（相对变换，往返 0.000 mm）
  · 新增 approach_slowdown（末段 40% 半速、下降段全程半速）
  · workspace_bounds z 下限 0.05 → −0.08

scripts/nbv_views.py          选站排序加入 blocked
scripts/plan_pose_ik.py       肩部 −130、肘部 −60、--allow-outside-tcp-box、已验证姿态做 IK 种子
scripts/run_pick_and_place.py grip_targets 改用验证值 + grasp_in_workpiece_frame
scripts/teaching/rvs_pose_client.py  RVS 地址改为 CS625_RVS_HOST 环境变量
scripts/sync_command_position.py     零位移轨迹优先（不释放伺服）

docs/manuals/…EXPERIENCE.md   新增第 40–46 条
docs/agent_memory.md          同步权威值与三条新规则
```

## 4. 仍然存在的缺口

```text
① 规划场景缺工件/桌子 → 碰撞检查不完整（手册第 45 条）
② J4 抱闸未修复 → 不做完整搬运（抬升/转运/插入）
③ RVS 在当前工件位置 fitness 仅 0.672 → 不能作为真值来源
④ 效率模式的完整搬运链（抬升→转运→预插入→插入）尚未验证
```

## 5. 验证命令

```bash
python3 -m pytest test/ -q                    # 365 passed
python3 scripts/check_docs.py                 # ALL DOC CHECKS PASSED

# 抓取点往返验证（应为 0.000 mm）
python3 -c "
import yaml, numpy as np
from scipy.spatial.transform import Rotation as R
c = yaml.safe_load(open('config/tasks/common.yaml'))
r = c['workpiece_reference']; g = c['grasp_in_workpiece_frame']
T = np.eye(4); T[:3,:3] = R.from_quat(r['quat']).as_matrix(); T[:3,3] = r['xyz']
G = np.eye(4); G[:3,:3] = R.from_quat(g['quaternion']).as_matrix(); G[:3,3] = g['translation']
print('往返误差 mm:', round(float(np.linalg.norm((T@G)[:3,3] - np.array(c['grasp'])))*1000, 3))
"
```

## 6. 19:00–19:36 晚班：撤出 + 效率模式拍摄复验（4K 面板同一条命令）

### 19:05 撤出前先核对"机械臂到底在哪"

```text
起因：要给"撤出"定路线，必须先确认抓取姿态离工件多远。
  ros2 run tf2_ros tf2_echo base_link tool0  → (0.864, 0.493, 0.292)
  /tcp_pose_broadcaster/pose --once          → (0.8594, 0.5396, −0.0605)
两者差 353 mm，差点按"已经在 29 cm 高处"下结论。

核对：/compute_fk 分别算 tool0 与 my_end_effector_link
  tool0                → (0.86404, 0.49318, 0.29241)
  my_end_effector_link → (0.85935, 0.53956, −0.06051)  ← 与 broadcaster 完全一致
结论：末端是 my_end_effector_link，tool0 是占位 link（= tool_flange_to_tcp_mm 353 mm）。
     当时机械臂确实**还贴在抓取位**（z=−0.0605），撤出还没做。
→ 手册第 50 条；agent_memory 增加"TCP 位姿口径"一行。
```

### 19:10–19:20 缓慢撤出（夹爪闭合、工件未抬起）

```text
全部走 scripts/move_tcp_linear.py（笛卡尔直线，点云间隙闸在线），半速 time_scale 2.0：
  ① (0.8594, 0.5396, −0.0605) → (0.8594, 0.4896, −0.0605)  −Y 50 mm   间隙 29.4 → 54.6 mm
  ② 同上 → (0.8594, 0.4896,  0.0895)                        +Z 150 mm  间隙 54.5 → 56.7 mm
  ③ 同上 → (0.8594, 0.4896,  0.4500)                        +Z 360 mm  间隙 61.3 → 151.9 mm
三段后 TCP 在 (0.8594, 0.4896, 0.4500)，离工件点云 151.9 mm。
撤出后立刻抓一帧相机画面复核：红色标记仍平放在木板上、工件未被带走、夹爪未夹持。
```

### 19:20–19:24 到观测位的重构（5 段，所有者已同意）

```text
从抓取构型到 observe.workpiece_coarse 需要 wrist_1 走 101.7°（>90° 翻转阈值），
按规则先打印六个关节角并向所有者确认，得到"同意，分 5 段执行"。
  plan_pose_ik --allow-flip → 同分支 (1,-1,1)→(1,-1,1)，25 点轨迹抽样无接触
  execute_assembly_chunks --time-scale 2.0 → 5 段全部 SUCCESSFUL，到位误差 <0.01 mm
到达 (0.5886, 0.3482, 0.7075) 后才看清问题：**画面大半是墙**（见 19:29 一节）。
```

### 19:29 粗观测位搞错了：config 里有两个"工件粗观测位"

```text
用 observe.workpiece_coarse (0.5886, 0.3482, 0.7075) 拍粗观测：
  相机对着房间，工件只在画面最下沿露一点；
  measure_object 报"离群组 >2 mm 已剔除 → 紧密簇不足 2 组"，拒收。

回推今天每一次成功粗观测的站位（T_base_cam × inv(T_tcp_cam)）：
  eff_flow_coarse / eff2_coarse / eff3_coarse → 全部是 TCP (0.6238, 0.3737, 0.5076)
  对应 config 的 observe.workpiece_coarse_efficient = efficient_start_pose
  （= 效率模式起点，2026-11-13 所有者确认）
而 observe.workpiece_coarse 与 observe.tables 完全相同，是示教链路（RVS）的桌面扫描位。
→ 手册第 52 条。
换到 (0.6238, 0.37375, 0.50762) 重测（1 段，12.9° 最大行程，到位 1e-6）：
  工件中心 (850.5, 554.5, 125.9) mm，yaw +9.2°，覆盖率 0.871，重复性 0.64 mm / 1.23°，verdict OK
  [table] 报文同时确认：桌面在 base z = −0.111 m
手跑选站：+13@20261120（tier1，覆盖 0.985，差 4.0°，摆放检查 31.1 mm / 3.98°，限 50 mm / 5.0° → 通过）
```

### 19:32–19:35 改用 4K 面板效率模式同一条命令跑拍摄复验

```text
所有者指出应走"效率模式"（面板效率页 / teaching_debug 之外的正式入口是 run_task.py），
于是用面板效率页 _build_command 的同一串参数：
  python3 scripts/run_task.py --scope load --mode efficient --world real \
      --until measure:workpiece_fine --confirm        （--until = 面板勾选阶段后的截断）

结果（四段全 ok，实验已归档）：
  observe:workpiece_coarse_efficient  3 点、0.2 s（本来就在起点）
  measure:workpiece_coarse            **先验新鲜（4.4 分钟 < 30）→ 不重新测量**
  nbv:workpiece                       到位复核：无接触 ✅（站姿与 eff3_fine 完全同一站）
  measure:workpiece_fine              实拍：覆盖率 0.973、yaw 10.0°、
                                      中心 (852.4, 552.3, 126.5) mm、重复性 0.1 mm / 0.083°、verdict OK
  精测位姿 T_base_cam 与 eff3_fine 逐元素相同（差 1e-5）→ 选站落在 +13@20261120

实验归档：~/cs625_experiment_archive/20261124_193256_real_load_efficient
         （2123 行时序、12 张图像、索引已追加）
当前 TCP：(0.74711, 0.18444, 0.59582)，即 +13@20261120 站位。

**必须写清楚的一点**：这次"拍摄验证"的粗观测阶段**没有实拍**——它复用了 19:29 的先验。
要看真·全链路拍摄，需要把 measure 阶段的 --prior-max-age-min 改成 0 再跑一次。
→ 手册第 53 条；agent_memory 增加"效率模式粗观测的先验"一行。
```

### 19:36 收尾状态

```text
· 机械臂已离开工件（间隙 151.9 mm 起，现在停在 +13@20261120 站位）
· 工件未被移动、未被带走：粗测 (850.5, 554.5) 与精测 (852.4, 552.3) 差 2.9 mm
· 本轮新增手册条目：50（tool0 陷阱）、51（平移后腕部重解）、52（两个粗观测位）、53（先验顶替实拍）
· 仍缺：完整搬运（J4 抱闸）、规划场景缺工件/桌子
```
