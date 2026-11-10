# 2026-11-10 WORKLOG D1 数据入口：vision_capture.py 与首次干跑

> 相关：`docs/plans/2026-11.md` D1；`docs/manuals/debug/camera.md`
> 真机使用：相机（真机，已启动）。**未发机械臂指令、未启动夹爪、未发生任何运动。**
> 仿真使用：无。

## 1. 本轮要解决的问题

D1 的第一件交付：把"相机给的数据"按固定格式、可复现地存下来，并当场给出重复性——
后面粗测量/精配准的每个结论都建立在这批数据上。

## 2. 新增文件

```text
A scripts/vision_capture.py   按组采 RGB-D → 落盘 depth.npy / cloud.npy / color.png / meta.json
                              → 组内逐像素时间重复性 + 汇总 summary/metrics
```

- 订阅用 **reliable QoS**（大图 best-effort 会被静默丢弃，昨天踩过）。
- 只收新帧（按 `header.stamp` 去重）。
- 不动机械臂；组间停顿留给人工摆放工件。
- 每次运行的 csv/summary 带 tag 后缀，同一 `--out` 下可以采多个朝向而不互相覆盖。

## 3. 首次干跑（当前姿态，3 组 × 5 帧）

```bash
python3 scripts/vision_capture.py --groups 3 --frames 5 --tag dryrun --delay 1.0
```

```text
[00] 有效  76.0%  中位 1.355 m  点距 1.491 mm  重复性 1.497 mm
[01] 有效  76.1%  中位 1.355 m  点距 1.488 mm  重复性 1.497 mm
[02] 有效  76.0%  中位 1.355 m  点距 1.493 mm  重复性 1.549 mm
组内重复性中位数 1.497 mm（验收 ≤ 2 mm）
```

单组 meta 关键字段：`valid_ratio 0.7604 / median_m 1.355 / repeatability_mm 1.497 /
repeat_p95_mm 5.528 / spacing_mm 1.491 / flying 0.0305 / hole_ratio 0.2358`。

结论：**当前姿态满足 D1 验收线**（重复性 1.5 mm ≤ 2 mm）。p95 5.5 mm 说明边缘/背景处抖动偏大，
统计时应按工件区域取，而不是整幅图。

## 4. 现场状态（开工前检查）

```text
相机：在线。pid 166428，modeling 1280×960，发布者 1，0.8 fps
机械臂 ROS 驱动栈：不在线（ros2 node list 只有 /camera/camera、/camera/camera_container）
  → 现在读不到 TCP/TF、也不能规划移动；若只需"停着采"不受影响
夹爪：未启动
```

## 5. 观测位判断

当前相机姿态的画面：工件斜视占画面约 35–40%，扶手可见，深度有效 76%、点距 1.49 mm。
与 `config/tasks/common.yaml` 里 `observe.workpiece_fine`（az≈25°、el≈30°、投影面积 37%）的
描述一致 → **当前姿态可以直接当精观测位用**，不需要先动机械臂。

## 6. 下一步

5 个朝向 × 4 组（每组 5 帧），每次用一个 tag：

```bash
OUT=runtime/vision_dataset/$(date +%Y%m%d_%H%M)_orient5
python3 scripts/vision_capture.py --groups 4 --frames 5 --tag rot0   --out $OUT --delay 2
# 工件转 +30° 后重复 --tag rotp30；+60° → rotp60；+90° → rotp90；-30° → rotm30
```

## 7. 11:3x 开工检查（失败）+ 三维度问题回答

### 7.1 机械臂驱动栈仍未启动

```text
ros2 node list          → 只有 /camera/* 与 /cs625_camera_view、/control_health_activate
ros2 topic list         → 没有 /joint_states、没有 /tcp_pose_broadcaster/pose
pgrep cs625_bringup     → 无进程
runtime/logs/driver_*.log → 不存在（说明 launch 根本没跑起来）
python3 scripts/control_health.py → sockets False / controller False / joint_states False
ping 192.168.1.200      → 通（0.8 ms）
```

结论：机器人网络可达，但驱动栈没在跑 → **六轴位姿记录做不了**（`scripts/record_pose.py` 已就绪，
驱动起来一条命令即可写入 `docs/reference/observation_poses_recorded.yaml`）。

### 7.2 粗精配准开发的前置条件核对

```text
数据：runtime/vision_dataset/20261110_1115_orient5（20 组，重复性 1.60 mm）  ✅
内参：/camera/color/camera_info = config/real/camera_intrinsics.json        ✅
模板：~/cs625_rvs_archive/models_20261023/弹仓_装配导出.STL（13.8 MB）      ✅
库：Open3D 0.20.0（含 GICP）+ scipy 1.11.4                                   ✅
手眼：不需要（本月全部在相机系内验收）                                       ✅
→ 可以开始 D2（粗测量）
```

### 7.3 六模式偏差的历史根因（按量级排序，全部有文档出处）

| 量级 | 原因 | 出处 |
|---|---|---|
| **353 mm** | RVS 的 T1/T3 是**法兰**位姿，我们发了**工具 TCP**（差 -46,0,353 mm） | `2026-10-20_WORKLOG_P4_REALTIME_TCP_POSE.md` §16 |
| **~160 / 106 mm** | RVS 返回**几何中心**，而 `T_b_tcp` 按 box.ply **模型原点**写；中心偏移 (10.5,-215,-105.2) mm | 同上 §15 |
| **~100 mm** | ICP"稳定但错"：近正方形面四重对称，自洽的错误解 | `docs/agent_memory.md`（稳定性≠正确性） |
| **35 mm** | 复用 74 小时前的旧 R1 插槽位姿 → 2026-11-03 三次碰撞 | `2026-11-03_WORKLOG_COLLISION_INCIDENT_AND_MOTION_GATES.md` |
| **33 mm** | RVS R1 本身的跨会话漂移（系统偏置，单次测量不可靠） | 同上 / 交接文档 |
| **20 mm** | 规划终点关节容差 0.02 rad ≈ 1.15°（0.6 m 力臂） | `config/tasks/common.yaml` 注释 |
| **13 mm** | 示教与六模式两套插入常量并存、会漂移 | `2026-11-02_WORKLOG_IMPROVED_AND_REMAINING.md` #16 |
| **毫米级** | 附着碰撞体用简化 `box_collision.STL`（无扶手），低估真实外廓 | `docs/agent_memory.md` |

**共同模式：几乎全部是坐标口径 / 参考系 / 旧常量复用，而不是算法精度不足。**
这也是"总是偏"的原因——每一处口径错误都是几十到几百毫米的**系统偏差**，
不消除就一直在，且会被下一个环节继承。

## 8. 11:4x 驱动栈为什么没起来（根因）与修复

### 8.1 根因：VS Code 任务里的 pgrep 匹配到了自己

任务「① 开工 · 真机」里这一行：

```bash
if pgrep -f "ros2 launch cs625_bringup" >/dev/null; then echo "[开工] 驱动栈已在运行"; else ...启动...; fi
```

这个 `bash -c` 的命令行里**本身就包含** `ros2 launch cs625_bringup`（else 分支的启动命令），
所以 `pgrep -f` 匹配到了正在执行任务的这个 shell 自己 → 判定"已在运行" → **跳过启动**，
然后直接 `sleep 45; control_health --activate`，自然什么也没激活。

证据：`runtime/logs/driver_*.log` 今天**一个都没有**（nohup 的重定向会先建文件，说明启动分支根本没执行）；
在本机复现：`bash -c 'pgrep -af "ros2 launch cs625_bringup"'` 的输出就是它自己。

### 8.2 修复（第一次改错了，第二次才对）

第一次把判断改成 `pgrep -f "/opt/ros/jazzy/bin/ros2 launch cs625_bringup"` —— **还是自匹配**：
这个字符串本身也写在任务的命令行里（pgrep 的参数），所以照样匹配到自己。
实测：`bash -c 'pgrep -af "/opt/ros/jazzy/bin/ros2 launch cs625_bringup"'` 输出两行，
一行是真进程、一行是执行 pgrep 的 shell。

最终改成 **括号法**，让模式串本身不出现在会匹配的文本里：

```bash
pgrep -f "[r]os2 launch cs625_bringup"
```

命令行里出现的是 `[r]os2`（带方括号），正则 `[r]os2` 匹配的是 `ros2`，所以任务自己的
shell 不会命中，只有真正的 launch 进程会命中。

### 8.2.1 冷启动验证（本轮实测）

```text
1) 先 SIGINT 停掉手动起的栈 → 节点只剩相机，无残留进程
2) 用任务①的原始命令冷启动：
     [开工] 已在后台启动驱动栈
     activated: arm_controller: active
     health: sockets=True controller=True joint_states=True
     PREFLIGHT_OK
3) 再执行一次任务①：
     activated: arm_controller is already active
     PREFLIGHT_OK
     → 没有起第二个栈（launch 进程数 = 1，日志文件没增加）
```

### 8.3 手动起栈（本轮验证）

第一次手动起失败，是我自己笔误：写成 `description_file=...`（应为 `description_file:=...`），
launch 报 `malformed launch argument`。改对后：

```text
driver_114542.log：控制器逐个加载/激活，tcp_pose_broadcaster 正常
ros2 node list：/cs625、/eli_ros2_control_node、/arm_controller、/dashboard_client、
                /io_and_status_controller、/force_torque_sensor_broadcaster …
ros2 topic list：/joint_states、/tcp_pose_broadcaster/pose 都在
control_health --activate → arm_controller: active（sockets/controller/joint_states 全 True）
preflight_motion.py → PREFLIGHT_OK（effort 流 n=374、Σ|effort|=10.28）
```

### 8.4 位姿已记录

```yaml
name: workpiece_coarse_20261110
joints: elbow -2.172060 / shoulder_lift -0.825561 / shoulder_pan 0.768012 /
        wrist_1 -0.658295 / wrist_2 1.566055 / wrist_3 1.655411   (rad)
tcp: xyz [0.562996, 0.297247, 0.786888]  quat_xyzw [0.288288, -0.814162, -0.206552, -0.459746]
dataset: runtime/vision_dataset/20261110_1115_orient5
```

写入 `docs/reference/observation_poses_recorded.yaml`（只追加，不改任何模式配置）。

## 9. 12:0x CAD 模型盘点 + D2 粗测量 v0（20 组实跑）

### 9.1 CAD 模型：不缺，仓库外有一份精确的

```text
~/cs625_rvs_archive/models_20261023/
  弹仓_装配导出.STL        276642 面  外形 455 × 131 × 465 mm   ← 精确件（装配导出）
  弹仓.STL                  63228 面  同尺寸，降采样版
  弹仓_aligned_senior.sTL   63228 面  师兄对齐过的坐标系版本
  弹仓_simplified_5mm*.stl          5 mm 简化版（碰撞用）

~/cs625_legacy_underlay_jazzy/src/tcp_bridge/meshes/
  box_collision.STL / slot_collision.STL / tool0.STL   ← 只有简化碰撞体，不是 CAD
```

结论：**精确 CAD 在 `~/cs625_rvs_archive/models_20261023/弹仓_装配导出.STL`**（14 MB，
RVS 的模型目录，不在 git 仓库里）。尺寸 455 × 131 × 465 mm，与 `common.yaml` 里
"441×465 mm 近正方形面"的记录一致。师兄仓库里只有简化碰撞体。

### 9.2 D2 粗测量 v0 实现

```text
scripts/coarse_pose.py
  1. 图像空间抠出工件：深度窗 0.55–1.05 m 内最大连通域（桌面/幕布先被排除）
  2. 工件点云上逐个 RANSAC 平面分割（3 mm 内点，最多 8 个面）
  3. 平面去重：法向相似度 >0.92 视为同一个面的碎片，只留内点最多的
  4. 挑"两两正交（|n·n|<0.2）+ 内点最多"的三面 → Gram-Schmidt → 三平面 SVD 求交
  5. 输出角点（相机系，mm）与正交误差、条件数
```

### 9.3 实测结果（20 组）

| 朝向 | 成功 | 角点重复性（x/y/z 极差） | 正交中位 | 最小面内点 |
|---|---|---|---|---|
| rot0 | 4/4 | 6.6 / 3.9 / 6.6 mm | 0.101 | 3616 |
| rotm30 | 4/4 | **2.0 / 0.6 / 0.9 mm** | 0.062 | 8404 |
| rotp30 | **0/4** | — | — | 只有 2 个不同面 |
| rotp60 | **0/4** | — | — | 3 个面，但没有一组满足正交 |
| rotp90 | 4/4 | 8.0 / 2.1 / 4.4 mm | 0.033 | 8424 |

### 9.4 分析

1. **重复性最好的朝向（rotm30）角点极差 2 mm**，说明三平面法本身可用；
   差的朝向（rot0/rotp90）第三面点数少（3.6k），交点噪声大。
2. **rotp30/rotp60 失败是几何原因，不是 bug**：工件转到这两个角度时，
   第三个面几乎侧视（只剩 2 个可用面），三平面求交必然退化。
   这与手册记录的"扶手面朝向机械臂时最稳（±45° 窗口）"一致。
3. 下一步两条路：
   · 两平面的朝向用**叉积**补齐第三轴，位置用 CAD 尺寸约束 → 退化姿态也能出粗位姿；
   · 或用 CAD 模板做一次低精度 ICP 当粗位姿（相当于把 D3 前移半步）。

## 10. 12:1x 所有者追问：三条口径到底敲定了没有？

如实回答：**一条已敲定、一条半敲定、一条之前没敲定（现在补上）**。

| 问题 | 结论 | 证据 |
|---|---|---|
| 内参对应哪个分辨率 | ✅ **已敲定：1280×960** | 实机 `camera_info`、数据集 `meta.json`、`config/real/camera_intrinsics.json` 三者 K 完全一致（fx 931.473 / fy 931.595 / cx 648.695 / cy 449.319），D 全 0；深度 16UC1 已配准到彩色，共用同一套 K |
| 模板 STL 的单位与原点 | 🟡 **单位=米已确认；原点偏移已量化，但原点的物理语义没有文档** | 顶点坐标 0.45 量级 → 米；外形 455×131×465 mm；bbox 中心相对 STL 原点 = (-208.5, +43.5, -7.5) mm；含扶手；网格不闭合 |
| 位姿是模型原点还是几何中心 | ⚠️ **之前没敲定**（D2 v0 输出的是三平面交点，是中间量） | 现已定义为 `T_camera_object`（对象系 = STL 坐标系），中心换算公式写进文档 |

新增 [docs/reference/pose_conventions.md](../reference/pose_conventions.md)：相机系、内参按分辨率、
对象系与模板、位姿定义、以及四条禁止事项（别混分辨率、别把中心当原点、别混两个 STL、
别把 RVS 的中心位姿直接当 `T_camera_object`）。

这也解释了为什么现在还不能谈毫米级：**位姿定义刚定下来，D2 的角点还没变成位姿**。
D2 v1 要做的是"三法向 → 旋转、bbox 角点 → 平移"的映射。

## 11. 12:3x–13:1x D2 v1：三平面 → T_camera_object（用 CAD 形状消歧）

### 11.1 做了什么

```text
coarse_pose.py 新增：
  estimate_pose()    三平面法向 → 旋转；交点 ↔ CAD bbox 角点 → 平移（穷举 48 种指派）
  cad_register()     用 CAD（含扶手）做 ICP，把"近立方体 24 种等价解"压成一个
  reject_with_pose() 用当前位姿把"不像工件"的点剔掉（rotp90 的连通域和桌面连成一片）
  → 采点/平面/位姿三步迭代两轮
```

### 11.2 结果（20 组，13 组成功）

| 朝向 | 成功 | 覆盖率(fitness) | RMSE | 平移重复性 | 旋转重复性 |
|---|---|---|---|---|---|
| rot0 | 4/4 | 0.978 | 4.1 mm | **14.0 mm** | **5.7°** |
| rotm30 | 4/4 | 0.973 | 4.2 mm | **1.7 mm** | **0.94°** |
| rotp90 | 4/4 | 0.970 | 4.2 mm | **4.5 mm** | **1.27°** |
| rotp30 | 1/4 | 0.211 | 4.6 mm | — | — |
| rotp60 | 0/4 | — | — | — | — |

### 11.3 判读

```text
· 三平面法本身稳定：三个主朝向的角点/位姿都能闭合；
· CAD 消歧有效：覆盖率 0.97，说明位姿把观测点云几乎全部映射到模型表面上；
· RMSE ~4.2 mm 是"观测点到 CAD 表面"的距离，落在 D2 的 2–5 mm 目标区间；
· rot0 重复性差（14 mm / 5.7°）：四次 ICP 收敛到略有差异的姿态，需要更细的 ICP
  （更小阈值/更多迭代/多尺度）或固定模板采样；
· rotp30/rotp60 仍是几何问题：只有两个面可见 → 三平面退化（不是算法 bug）。
```

## 12. 13:0x–13:4x D2 v2：多尺度 ICP + 两平面定向搜索 + 置信门槛

### 12.1 三处改进

```text
① 模板采样确定性：Open3D 的 sample_points_uniformly 不吃 seed，改成按三角面面积
   自己采样（固定 seed=7），消除"同一组数据两次跑出不同结果"的来源
② 多尺度 ICP：对应距离 30 → 15 → 8 mm 逐级收敛，避免停在近立方体的浅局部极小
③ 两平面定向搜索（guided_two_plane）：只有两个可见面时，两个面固定两条轴、
   第三条用叉积，沿自由方向扫平移，用"观测点落在 CAD 附近的比例"粗筛后再 ICP
④ 置信门槛 POSE_MIN_COVERAGE = 0.70：覆盖率低于它的位姿标"不可信"，不当作结果
```

### 12.2 结果（20 组）

| 朝向 | 有效组 | 覆盖率 | RMSE | 平移重复性 | 旋转重复性 |
|---|---|---|---|---|---|
| rot0 | 4/4 | 0.988 | 3.7 mm | 3.1 mm | 3.80° |
| rotm30 | 4/4 | 0.973 | 3.5 mm | 2.1 mm | 0.92° |
| rotp30 | 4/4 | 0.810 | 4.1 mm | 7.6 mm | 1.25° |
| rotp60 | **0/4** | 0.53 | 4.2 mm | — | — |
| rotp90 | 4/4 | 0.980 | 3.6 mm | 4.7 mm | 2.11° |

### 12.3 判读

```text
· 20 组全部能出位姿，16 组通过置信门槛（4/5 朝向）；
· 平移重复性 2.1–7.6 mm（D2 目标 ≤5 mm：rot0/rotm30/rotp90 达标，rotp30 略超）；
· 旋转重复性 0.9–3.8°（D2 目标 ≤2°：rotm30/rotp30 达标，rot0/rotp90 略超）；
· rotp60 覆盖率只有 0.53 → 位姿不可信：该朝向工件只呈现一个大面 + 凹槽面，
  两平面也不是真正互相垂直的面，定向搜索选错了组合；需要"有效性筛面"而不是硬算。
```

### 12.4 下一步（D2 v3）

```text
1. rotp60：在选三/两平面之前先做"面有效性"检查（面积、平面度、是否与已知工件面尺寸匹配），
   不可靠的面不参与；
2. rot0/rotp90 的旋转重复性从 3.8°/2.1° 往 2° 以内收：ICP 收敛后再做一步
   "平面重拟合 + 位姿约束"（用平面法向修正旋转）；
3. 把位姿输出接进 derive/task_pose 的前置：相机系位姿 → （手眼）→ base 系。
```

## 13. 13:5x–14:2x D2 v3：面有效性筛 + 兜底路径剔点迭代 → **五个朝向全部可信**

### 13.1 改动

```text
① filter_planes()：只保留"像工件外表面"的面（内点 ≥1500、RMS ≤3 mm、质心离工件中心 ≤0.40 m）
   —— 针对 rotp60 里夹具大平面混进候选的问题
② 兜底路径（两平面）也做"用位姿剔点 → 重跑"的迭代，覆盖率按剔除后的点计算
   —— 之前 rotp60 覆盖率只有 0.53，是因为"工件区"里有 20 万点、其中一半是桌面/夹具
③ 试过但撤掉的：refine_rotation（用平面法向拧正姿态）——实测把 rot0 从 3.1 mm 变差到 7.9 mm，
   说明平面法向的噪声比 ICP 的统计误差更大，不该用它做最后一步
```

### 13.2 结果（20 组，全部通过置信门槛）

| 朝向 | 有效 | 覆盖率 | RMSE | 平移重复性 | 旋转重复性 |
|---|---|---|---|---|---|
| rot0 | 4/4 | 0.987 | 3.7 mm | 11.1 mm | 3.67° |
| rotm30 | 4/4 | 0.974 | 3.5 mm | **2.6 mm** | **0.93°** |
| rotp30 | 4/4 | 0.877 | 4.1 mm | 25.2 mm | 1.33° |
| **rotp60** | **4/4** | **0.870** | 4.2 mm | 6.2 mm | 2.21° |
| rotp90 | 4/4 | 0.980 | 3.6 mm | 4.2 mm | 2.01° |

### 13.3 判读

```text
· 五个朝向全部出位姿、且通过置信门槛（覆盖率 0.87–0.99）—— D2 的"5 朝向全部成功"达成
· RMSE 3.5–4.2 mm，落在 D2 的 2–5 mm 目标内
· 重复性：rotm30 2.6 mm/0.93° 最好；rotp90 4.2 mm/2.01° 基本达标；
  rot0 11.1 mm、rotp30 25.2 mm 仍偏大 —— 静态场景下重复采集的差异来自传感器噪声把 ICP
  推进不同的浅局部极小，说明这些朝向下"位姿没有被数据强约束"，需要额外约束（见下）
```

### 13.4 D2 v4 待办

```text
1. 位置约束：用三平面交点（角点）作为位置锚点，ICP 只负责姿态微调——角点重复性实测 2–7 mm，
   比 ICP 平移稳定；
2. 对 rotp30 这类"两面主导"的朝向，沿缺失轴的位置本来就没有观测约束，应显式标出
   "该轴不确定"而不是假装算出精确平移；
3. 位姿输出接 task pose 推导入口。
```
