# 2026-10-23 HANDOFF — 下个对话开工包

> 给「没有任何上下文的新对话」看的自包含交接。
> 读这一份就能接手；细节入口见文末 §10。

---

## 1. 一句话现状

真机上装入流程已跑通，取出流程已跑到「工件从插槽完整抽出、移动到工件桌正上方」；
工件目前由所有者**手动放回工件桌**，机械臂停在安全位。
下一步不是继续搬件，而是：**落六类模式代码 + 强制环境建模 + 三轮调试**。

---

## 2. 现场物理状态（2026-10-23 收尾）

```text
TCP          1.0309, 0.6861, 0.4398
夹爪          position=0 mm（全闭合、空夹），init=5
safety_mode   1（正常）    robot_mode 7（运行）
工件          已手动放回工件桌面
机器人型号    Elite CS625，RVS 视觉，HITBOT EFG80 电爪
```

**开工前必须先确认这三项**（急停后尤其如此）：`safety_mode=1`、`robot_mode=7`、`arm_controller active`。

---

## 3. 启动顺序（照抄即可）

```bash
cd <仓库根目录>
source scripts/source_dev_env.sh --full

# ① 驱动
source "$HOME/.cs625_local.env"
ros2 launch cs625_bringup real_base.launch.py launch_driver:=true \
  robot_ip:="${CS625_ROBOT_IP:?}" \
  description_package:=eli_cs_robot_description description_file:=cs625.urdf.xacro \
  headless_mode:=true launch_moveit:=false launch_rviz:=false \
  activate_joint_controller:=false execute:=false require_confirmation:=true

# ② MoveIt（另开终端；观察场景时把 launch_rviz 改 true）
ros2 launch cs625_bringup real_moveit.launch.py \
  description_package:=eli_cs_robot_description description_file:=cs625.urdf.xacro \
  moveit_config_package:=elite_cs625_moveit_config \
  semantic_package:=cs625_bringup semantic_file:=config/cs625_active_perception.srdf \
  launch_rviz:=true

# ③ 激活控制器（若 arm_controller 是 inactive）
ros2 control switch_controllers --controller-manager /eli_ros2_control_node \
  --activate arm_controller --strict

# ④ 视觉与碰撞相关节点（师兄组件，源码保留在只读工作区）
#   tcp_server              位姿链（S2/S3/R1/insert/pre_insert）
#   collision_object_publisher  工件 attach + 插槽碰撞体
#   cs625_compliant_placement   接触监督（可选）
#   cs625_state_monitor         原始状态（含关节力矩）

# ⑤ 健康检查（必须全 True）
python3 scripts/control_health.py

# ⑥ 当天环境建模（下次要实现的强制前置项；在此之前用现有体素脚本）
python3 scripts/build_environment_collision.py --pcd <当天点云>.pcd --apply
```

---

## 4. 关键事实（做任何动作前必须知道）

### 4.1 位姿链全部由师兄 `tcp_server` 提供，不要自写公式

```text
slot 原点基准        1.241629, 0.181709, 0.443245
                     q = (-0.0016288, 0.0042531, 0.0033840, 0.9999839)
insert TCP           1.1574, -0.0132, 0.0321
插入前 pre_insert     1.0996, -0.0125, 0.3773（-10° 倾角）
工件原始位姿          0.653422, 0.703592, 0.355183
抓取 TCP             0.8316, 0.5918, -0.0583
工件观测/抓取前       0.7677, 0.4903, -0.0595
中转点（带件）        0.95, 0.45, 0.55   ← 必须抬高，0.35 无解
插槽拍摄基准位        1.044, -0.011, 0.726
```

### 4.2 工件的真实几何（最容易搞错）

```text
工件是沿「工具 z 轴」横向伸出约 0.41 m，不是竖直悬垂。
T_link_box ≈ (-0.009, 0.215, -0.411)，绕 y 转 180°。
工件包围盒 ≈ 114 × 441 × 465 mm。
后果：靠近插槽桌时必须先横移让开 y≤0.40 的区域，再下降；
      垂直下降会刮到插槽桌（已发生过，触发急停）。
```

### 4.3 碰撞机制的正确理解（推翻过一次误判）

```text
· GetPlanningScene 读回的位姿一律是 0 → 这是“读取不可信”，不是场景失效
· 内部位姿正确：在工件位置放探针，能报 box_collision（attach）接触 0.02 m
· MoveIt 不检查「世界 ↔ 世界」，所以“世界盒子表示工件”做校验永远无碰撞（错方法）
· 结论：工件碰撞走师兄 attach（collision_object_publisher + touch_links）
        校验用 check_state_validity；不要读场景位姿做几何判断
```

### 4.4 夹爪（HITBOT EFG80）

```text
· 夹持宽度 50 mm + 电流 0.1 A（所有者实测：50 mm 已完全顶住；52 mm 起电源显示 ~1 A 硬顶）
· 归零有偏移，显示开度 ≠ 真实开度
· 寄存器 70 的电流反馈不可信；以电源面板电流为准
· init 约 10 s，必须等 init==5 才能发下一条命令
· 无自锁：保持夹力 = 持续通电 = 会有电流声（正常）
· 释放夹爪必须所有者当次授权（发生过未授权松开、工件悬空险些坠落）
```

### 4.5 速度策略（所有者已确认的偏好）

```text
· 只在「接近工件抓取」和「工件在插槽内移动」时低速
· 其余阶段允许不均匀速度：长段快、末端接近慢
· 装配段（插入/转正/抽出/倾角准备）间隙仅左右各约 1 mm
  → 默认全程匀速低速；只有真机验证过才允许 `--allow-uneven-insert`
· 插入/抽出实测基准：planning 0.005–0.01、time_scale 3.0–5.0、每段 10–20 mm、停 1.0–1.5 s
```

---

## 5. 必须遵守的铁律

```text
1. 释放夹爪：必须所有者当次明确同意（代码闸门 --allow-release）
2. 中止后只允许「重新规划」，禁止重放旧轨迹（曾导致来回抽搐）
3. 位姿读取必须取新帧（用缓存帧会反复走同一步）
4. 控制链不正常就恢复：bash scripts/recover_control.sh
   （严重时重启驱动栈；急停后 safety_mode 仍非 1 也要重启栈）
5. 单次工具调用 ≤60 s；60 s 内无位移立即排查，不要等
6. 装配段不允许大幅度旋转掠过插槽；释放后先纯平移离开 ≥150 mm 再旋转
7. 带工件进出插槽一律走 staging 高位中转；不允许从中途点直接规划到插槽
8. 规划「对准」类动作必须用实测验收（scripts/verify_pose.py），
   MoveIt 位姿目标容差 0.05 rad ≈ 2.9°，会假成功
9. 归档/移动/删除任何文档需所有者书面批准
```

---

## 6. 现有代码地图

### 编排与执行

```text
scripts/run_pick_and_place.py        装入 13 阶段 / 取出 18 阶段编排
                                     --flow pick_place|unload --confirm
                                     --skip/--until --allow-release --allow-uneven-insert
scripts/execute_assembly_chunks.py   分段执行器（首点保护 0.03 rad、行缓冲）
scripts/move_tcp_linear.py           直线移动（等新帧，防重复同一步）
scripts/verify_pose.py               实测位姿验收（位置/姿态残差 + PASS/FAIL）
scripts/control_health.py            秒级健康检查 + --fix 自动恢复
scripts/recover_control.sh           resend + 控制器切换 + 滑块 100%
```

### 环境与碰撞

```text
scripts/build_environment_collision.py   点云体素化为桌面碰撞体（--z-min/-0.13 --z-max/-0.07）
scripts/check_carried_clearance.py       几何余量（注意：世界-世界不参与 MoveIt 检查，仅作参考）
scripts/validate_carried_pose.py         attach 存在性/位姿验证试验脚本
scripts/visual_mesh_publisher.py         可视化模型发布
scripts/shims/trimesh.py                 让师兄节点缺 trimesh 时也能跑
```

### 配置（现有，待重构为六类模式）

```text
config/pick_place_real.yaml   装入参数 + phase_speeds
config/unload_real.yaml       取出参数 + phase_speeds + 固定位姿链
```

### 可视化面板

```text
scripts/cs625_ui.py   PyQt5 面板：关节、TCP、RVS 位姿表（10 行）、
                      夹爪控制（初始化/夹紧 50mm/张开/收拢）、事件日志、RVS 图片
                      图片扫描 ImageSave_*/*/color.png
```

### 师兄只读工作区（保留，不修改）

```text
~/cs625_legacy_underlay_jazzy/src/
  tcp_bridge/                     tcp_server、collision_object_publisher、meshes
  cs625_compliant_placement/      接触监督
  cs625_state_monitor/            原始状态（30001 第三连接，9 Hz，含关节力矩）
  environment_collision_generator/ 点云体素化 + 插槽扣除
  cs625_task_manager/             E0–E9 / D0–D9 状态机
  cs625_trajectory_tools/         力补偿笛卡尔
```

---

## 7. 下次要做的三件事（按顺序）

### ① 强制环境建模（含「常量 / 变量」区分）

新增 `scripts/model_environment.py`，产出 `runtime/environment/<ts>/model.json`。

**必须区分「恒定量」与「每任务变量」：**

```text
恒定量（每次都要验证没变）：
  插槽绝对位姿 slot_origin + quat（T1 实测）
  工件桌 / 插槽桌 碰撞体（体素）
  → 阈值：插槽偏差 >1 mm 或 >0.5° 拒绝

每任务变量（每次任务重新确立）：
  工件初位姿 workpiece_pose_initial（T3/S3 实测）
  → 每次抓取面对的工件位姿都不同，必须当场测量，不能沿用旧值
  → 阈值：与本次视觉置信度/点数门槛挂钩，测不到就拒绝开始
```

**放置位的推导规则（取出任务的关键）**

```text
取出任务不重新定义放置点，而是以「参考工件位姿」为基准：
  place_TCP = workpiece_pose_ref × T_b_tcp

workpiece_pose_ref 的来源优先级：
  1) 本次全流程任务开始时测得的工件初位姿
  2) 上一次「抓取并装入」任务记录中的工件初位姿
  3) 都没有 → 拒绝执行取出任务（不允许用固定常数硬编码）

因此需要一条任务记录链：
  runtime/tasks/<task_id>.json 里保存
    task_id / scope / 开始时工件初位姿 / 插槽基准 / 建模时间戳 / 关联的上一条任务
```

### ①b 三类 scope 对建模数据的依赖

| scope | 需要的建模数据 | 放置位来源 |
|---|---|---|
| `load`（抓取+装入） | 恒定量（插槽/桌子）+ **本次工件初位姿** | 不涉及放置（放置在插槽） |
| `unload`（取出+放置） | 恒定量 + 参考工件位姿 | `workpiece_pose_ref × T_b_tcp`，ref 取自本次全流程或上一条抓取任务 |
| `full`（四段全流程） | 恒定量 + 本次工件初位姿 | 直接复用本次任务开始时测得的工件初位姿 |

即：**取出可以独立跑，但必须有可追溯的工件初位姿来源**；来源缺失就拒绝，而不是退化成固定坐标。

### ② 六类模式落地

```bash
python3 scripts/run_task.py --scope {load|unload|full} --mode {safe|efficient} [--confirm]
```

```text
config/tasks/common.yaml                     世界常量 + 可复用位姿（中转/预插入/插入/10° 转正）
config/tasks/{load,unload,full}_{safe,efficient}.yaml
```

六份配置彼此独立；执行原语复用 `run_pick_and_place.py` 中的
健康门 / 分段执行 / 失败重规划 / 实测验收。

### ③ 三轮调试

```text
第一次 可用性：6 类各 dry-run + 真机各 1 次；把经验全部写进代码；修 bug
第二次 效率  ：固化可复用位姿；非装配段提速；效率模式目标 ≤ 安全模式 60% 时间
第三次 验收  ：6 类各 ≥3 次，统计成功率/耗时，采集数据供论文实验
```

---

## 8. 已知坑（踩过的，别再踩）

| 现象 | 原因 | 处理 |
|---|---|---|
| 命令接受但不动 / TIMEOUT | 多次中止后控制器失效 | `recover_control.sh`；必要时重启驱动栈 |
| 急停后 `safety_mode` 仍 7 | 驱动未恢复 | 重启驱动栈（不是反复 resend） |
| 原地重复同一步 | 位姿读取用缓存帧 | 等新帧再返回 |
| 来回抽搐 | 失败后重放旧轨迹 | 只重新规划；执行器首点保护 |
| 规划“成功”但没对准 | 位姿目标容差 2.9° | 收紧容差 + `verify_pose.py` 实测 |
| 关节点位差 0.02 rad 就判到位 | 执行器容差过大 | 小步移动用 `--goal-tolerance 0.002` |
| COM9 被占用报错 | UI 轮询与命令行抢串口 | 助手已加串口重试；UI 轮询改为 4 s |
| 夹爪面板报编码错 | PowerShell 输出 GBK | UI 多编码兜底 + 助手强制 UTF-8 |
| UI 图片不更新 | 只扫 box 目录 | 已改扫 `ImageSave_*/*/color.png` |
| 带件到插槽无解（code 99999） | 低通道不足 | 先到 staging 高位，或抬高运输点到 z=0.55 |

---

## 9. 仓库状态

```text
· 未提交改动较多（脚本、文档、配置），所有者尚未决定提交范围
· python3 test/contract_checks.py → PASS
· 归档/移动文档需所有者书面批准
· 师兄源码与我方仿真源码都保留，旧仿真配置标记「不可信但不删除」
```

---

## 10. 文档入口

```text
docs/agent_memory.md                                   长期经验（第 1–100 条，含今日结论 97–100）
docs/worklogs/2026-10-23_HANDOFF_MODES_ARCHITECTURE.md 六类模式架构与当日交接
docs/worklogs/2026-10-22_HANDOFF_EVENING_UNLOAD.md     装入跑通与取出推进的交接
docs/worklogs/2026-10-22_WORKLOG_VOXEL_GRASP_FLOW.md   点云碰撞与抓取流程复盘
docs/dependencies.md                                   依赖与师兄组件对照
```

Obsidian 镜像：`G:\我的云端硬盘\Obsidian_Vault\OY\Do be do be do\04-课题与项目\02_CS625项目\工作日志`
（WSL 看不到 G 盘，**必须用 PowerShell 从 `\\wsl.localhost\...` 复制**）

### ④ 观测基准位与视角搜索（已确立，待实现）

```text
基准位（起点，不是固定值）
  S0 插槽观测位  1.044, -0.011, 0.726
  S2 工件观测位  0.615353, 0.354540, 0.574506
  S3 工件抓取位  0.652989, 0.703460, 0.355241

调整分工
  J1 / J2 / J3 → 大幅调整（站位、高度、距离）
  J4 / J5 / J6 → 小幅调整（末端姿态、遮挡改善）

优化目标
  RVS 返回分数 ↑
  插槽 / 工件在视野中的完整度 ↑

实现顺序
  J1 粗扫 → J2/J3 定高度距离 → J4/J5/J6 微调 → 记录(关节角/分数/完整度/时间)
  → 联合排序选最佳观测位 → 写回本次任务记录作为基准
```

### ⑤ RVS 的已知缺陷与可介入点

```text
现状
  命令响应慢（T3 约 20–40 s）、观测帧率低、内部算子写死无法改、急停后重启慢

我方可以做的（不需要改 RVS 内部）
  1. 适配层：包一层 ROS 2 节点对接 RVS 7000 命令服务
     统一「下发命令 → 轮询结果 → 带时间戳缓存 → 用 latched QoS 暴露」
     避免上层反复触发、避免把等待时间混进机械臂动作
  2. 缓存与复用：位姿带时间戳，超过新鲜度就拒绝；同任务内不重复触发同类观测
  3. 并行化：等待 RVS 期间不做机械臂运动；RVS_AI 常驻避免每次重启推理
  4. 超时策略：单次超时只重发一次，连续失败才报错，不无限重试

需要 RVS 之外的新通路（才能真正解决帧率与算子问题）
  5. 独立相机通路：用 ROS 2 相机驱动直接取 RGB-D，绕开 RVS 做连续观测与自研检测
     —— 这是提升帧率和掌控算子的唯一根本办法；需要相机型号/驱动权限
  6. 若不能接管相机：只能接受 RVS 的低帧率，用「多次静态采集 + 融合」缓解
```

---

## 11. 2026-10-23 晚间追加：RVS 绕开与 ROS 相机链路（重要）

### 11.1 RVS 与 ROS 的取流互斥

```text
PS800-E1 为 GigE 相机，RVS 与 ROS 节点不能同时取流。
现状：RVS 可停止（进程可被 PowerShell 结束），停止后 ROS 才能打开相机。
已确认：WSL 里广播发现不可靠，必须「按 IP 打开」。
```

### 11.2 ROS 直连相机的可用方案（已验证）

```text
驱动：percipioxyz/camport_ros2（自带 camport4 SDK），装在 ~/cs625_camport_ws
改动（仅该克隆仓库内，原件已备份）：
  · percipio_device.cpp：设备 id 形如 IP 时用 TYOpenDeviceWithIP，
    且不能提前 return（alive=true 与事件回调注册在函数尾部）
  · percipio_camera_node_driver.cpp：广播枚举失败时遍历以太网接口按 IP 打开
启动（640×480 彩色+深度，位姿用）：
  ros2 launch percipio_camera percipio_camera.launch.py \
    device_ip:=<相机IP> color_resolution:=640x480 depth_resolution:=640x480 \
    depth_enable:=true depth_registration_enable:=true point_cloud_enable:=false
实测：彩色约 24 fps（仅 RGB）；开启深度后 0.85 fps（深度硬件上限）
```

### 11.3 关键性能与分辨率结论

```text
· 仅开 RGB：23.5 fps（640×480）
· 开深度后：约 0.85 fps（RGB 与深度同帧组，被深度拖住）
· 1280×960 下深度有效点≈0（金属反光 + 高分辨率稀疏）→ 位姿必须用 640×480
· 巨型帧未开启（1500 正常、9000 全失败），但不是瓶颈
```

### 11.4 自训分割模型

```text
数据：RVS_AI_Center 的 box/ 目录 53 组（rgb.png + rgb.json，LabelMe 多边形）
      注意：RVS 训练分辨率是 1280×960，不是 320
转换：~/cs625_yolo_seg（YOLO-seg 格式，42 train / 11 val）
模型：yolov8n-seg，两版
      320 版 mAP50-95 掩膜 0.968
      640 版 mAP50-95 掩膜 0.984（推荐）
推理：CPU 27 ms/帧（640）
噪声：验证集同源，指标偏乐观
```

### 11.5 TF 双树问题（当前阻塞点之一）

```text
现象：/tf_static 有 4 个发布者；tool0 不在 TF 树里；
      相机帧与机器人树不连通（"not part of the same tree"）
临时解法（已实施）：绕开 TF
      T_base_camera = T_base_tcp × T_tcp_camera（常量）
标定：config/handeye/tcp_to_camera.json
      T_tcp_cam 平移 ≈ (0.050, 0.000, 0.050) m  （相机几乎就在 TCP 上）
长期解法：统一 TF 发布源（让 robot_state_publisher 带相机固定关节，
          关闭相机驱动自带 TF）
```

### 11.6 本轮新增脚本

```text
scripts/ros_rgb_observer.py     一期：实时 RGB 检测 + 视野评分（置信度/完整度/占比/帧率）
scripts/depth_pose_once.py      二期：掩膜→点云→质心/PCA 位姿；--repeat N 输出 std
scripts/icp_pose_once.py        ICP：模板(局部) ↔ 点云(相机) → 结果转基座
                                输出 fitness / inlier RMSE
scripts/coarse_to_fine_pose.py  粗→精流程（默认 dry-run）：
                                粗：掩膜中心 + 最近显著深度峰 → 目标点(基座)
                                精：算 standoff 位姿 → 计划/执行 → ICP
```

### 11.7 当前精度状态（明天唯一要攻的点）

```text
坐标链路已打通：TCP 话题 + 常量偏移 + 掩膜 + 反投影
但 dry-run 目标点仍偏：(-0.376, -1.054, 0.265)，期望 (0.6–0.9, 0.4–0.8, 0.1–0.4)
距离 2.02 m vs 实际约 1.5 m
判断：误差被"相机姿态 × 远距离"放大，属标定量级问题
下一步：用"已知靶点"做链路标定（5 分钟定位到具体环节）
```

### 11.8 掩膜粘连问题（未解，优先级高）

```text
即使工件占画面 75–80%，掩膜仍把底部桌面粘连进来
→ 点云尺寸 1.75 m（真实 0.45 m）
→ 质心法与 ICP 都因此失效（fitness 0.32 / 0.17）
修法优先级：
  1. 补样本重训（加"工件周围有桌面"的图，200–300 张）
  2. 精采样阶段（移动到更近视角，减少桌面占比）
  3. 后处理（尺寸过滤/3D 聚类，效果有限，已证明不能单独用）
```

### 11.9 RVS 的粗/精两级机制（复刻依据）

```text
视点位两级：粗观测位 → 精观测位（由粗结果推出）
命令两级  ：T2（任意位置→S2 粗）→ T3（在 S2 触发→S3 精）；插槽 T1 同理
算法两级  ：工件 modelRelSamplingDistance 30 mm / 插槽 15 mm，均开法向估计
已确认：所有者此前定义的 S0/S2/S3/R1 全部属于【粗采样基准位】
详见 docs/worklogs/2026-10-23_SPEC_RVS_OPERATOR_FLOW_AUDIT.md
```

### 11.10 模型资产

```text
~/cs625_rvs_archive/models_20261023/
  弹仓.STL                    用户高保真模型（米单位，未改）
  弹仓_装配导出.STL           高面数副本
  弹仓_aligned_senior.sTL     已做轴向交换 + 中心对齐（63,228 面）
  弹仓_simplified_5mm_v2.stl  5 mm 简化（9,466 面，尺寸/中心与源一致）
注意：体素量化简化会引入半体素偏差，ICP 模板建议用保形简化（quadric）
```

### 11.11 明天开工顺序（更新版）

```text
1. 启动驱动 + MoveIt（带 RViz），确认 safety_mode=1 / 端口 3 条 / 控制器 active
2. 启动相机节点（640×480，彩色+深度），确认 image/depth 有数据
3. 运行「已知靶点」标定：对准桌面已知点，反算基座坐标并与实测比对
4. 据标定结果修偏移或深度筛选，使 dry-run 目标点落回 (0.6–0.9, 0.4–0.8, …)
5. 再干跑 coarse_to_fine 的 --confirm 实跑，验证精采样闭环
6. 之后才是六类模式与三轮调试（第八节）
```
