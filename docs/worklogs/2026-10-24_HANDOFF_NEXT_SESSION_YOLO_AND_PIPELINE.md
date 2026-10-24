# 2026-10-24 HANDOFF — 数据采集、四类标注、YOLO 重训与验收（下一段对话起点）

> 本文档是 2026-10-24 夜间（00:00–09:30）全部工作的交接说明。
> 阅读顺序：§1 一句话现状 → §2 已完成 → §3 未完成/阻塞 → §4 数据与模型 → §5 基准位姿 → §6 文件清单 → §7 下一步唯一任务 → §8 命令速查。

---

## 1. 一句话现状

**数据采集与两类分割模型重训已完成并通过离线验收（mask mAP50 0.86，工件类 0.927），但"粗观测→精观测→取位姿"的自动运动链路在真机上仍未跑通（执行器反复 TIMEOUT）。**

```text
当前臂位姿   (0.5915, 0.3496, 0.7038)   ← 工件粗观测位
safety_mode  7（保护停止，需示教器复位）
驱动健康     sockets = False（需重启驱动栈）
相机         运行中（1280×960，~0.79 fps）
```

---

## 2. 已完成（有证据）

### 2.1 数据采集：181 帧，22 个批次

| 批次组 | 帧数 | 内容 |
|---|---:|---|
| workpiece_v2/v3/v4/v5/v6/v7/v9/v10/v11 | 58 | 工件在 **5 个不同摆放位置**，0.45–0.80 m，俯角 15–45° |
| workpiece_d065 / d080 | 19 | 早期位置，含几何复核合格帧 |
| box_in_slot / box_in_slot2 | 11 | **工件在插槽内**（取出任务场景），0.60/0.75 m |
| slot / slot_close / slot_eval2 | 16 | 空插槽，0.60/0.95 m，方位 −10…+30° |
| metal_table / tables | 6 | 插槽桌（金属台面）+ 木桌空景 |
| empty_scene / environment / workspace | 36 | 幕布、墙面、房间宽视角（负样本） |
| table_views / slot_eval / workpiece / workpiece_d45 | 35 | 早期无效或未入库（无 metadata，标注时会自动跳过） |

**关键性质**：每个有效帧都有 `metadata.jsonl` 记录完整 `tcp_matrix`（4×4），可做几何标注。
数据集根目录：`~/cs625_yolo_seg/datasets/2026-10-25_raw/`

### 2.2 采集脚本已内置五道守卫（这是本次最重要的工程成果）

`scripts/capture_dataset.py` 是**唯一采集入口**，缺一条即拒绝执行：

```text
① 工作空间盒     x∈[0.35,1.10]  y∈[0.25,1.05]  z∈[0.25,0.80]
② 观测包络       J1 −30…120   J2 −150…−30   J3 −165…−40
                 J4 −160…60   J5 15…150     J6 35…125
   （任务包络另存：J1 2…45  J2 −128…−88  J3 −130…−62  J4 −98…13  J5 86…95  J6 62…100）
③ 夹爪间距       TCP 到工件中心 ≥ 0.38 m（插槽可用 --min-clearance 降到 0.18）
④ 取景门禁       用已知位姿投影 CAD 包围盒：面积 ≥10%、中心偏移 ≤0.35、中心在画面内
⑤ 规划器预检     真实规划一次，检查**终点构型**在观测包络内
```

`--dry-run` 会先打印全部目标位姿的检查结论，**确认后才动臂**。

### 2.3 两类自动标注：93 帧

`scripts/auto_label_four_class.py --classes two`

| 类别 | 标注方式 | 帧数 |
|---|---|---:|
| 0 workpiece | CAD 表面采样点投影 + **深度一致性过滤** + 凸包 | 74 |
| 1 slot | 师兄点云模型 `slot_pointcloud20260127.ply`（局部系 + 插槽位姿平移）投影 | 31 |
| 负样本 | 空标签 | 2 |

**已目视验证**：工件轮廓贴合零件（含扶手、顶部倒角），插槽轮廓完整覆盖夹具框架与槽口。

### 2.4 YOLO 重训：完成 40/40 轮

```text
模型目录  ~/cs625_yolo_seg/runs/segment/runs/segment/box_slot_1280/
权重      weights/best.pt（7.1 MB）
基础      box_seg640/best.pt 微调
数据      78 训练 / 15 验证（两类）
imgsz     1280（与采集一致）
```

**A/B 对比（同一验证集，离线，可复现）**：

| 模型 | Mask P | Mask R | mAP50 | mAP50-95 |
|---|---:|---:|---:|---:|
| **新 box_slot_1280** | **0.907** | **0.858** | **0.861** | **0.599** |
| 旧 box_seg640 | 0.794 | 0.417 | 0.492 | 0.140 |

分类别看（新模型）：

```text
workpiece  P 0.995  R 0.917  mAP50 0.927  mAP50-95 0.681
slot       P 0.818  R 0.800  mAP50 0.795  mAP50-95 0.516
```

> 旧模型在验证集上只检出 12 个实例（完全漏掉 slot 类），新模型检出 17 个。

### 2.5 其他已交付

- `scripts/build_scene_model.py`：多帧 RGB-D 拼基座系点云 + 平面拟合（实测：工件桌 z≈−0.09，幕布 y≈1.35）
- `scripts/record_frames.py`：任务全程 1 Hz 录像（含 TCP + 抓取事件）
- `scripts/capture_environment.py`：环境宽视角采集（受安全工作盒约束）
- `config/real/camera_intrinsics.json`：fx 931.47 / fy 931.60 / cx 648.70 / cy 449.32 @1280×960
- `scripts/auto_label_frames.py`：单类工件几何标注（保留作为工具）

---

## 3. 未完成 / 阻塞（下一段对话的真实起点）

### 3.1 【阻塞】粗观测 → 精观测 → 取位姿 的运动链路

**目标流程**（所有者 09:1x 明确要求）：

```text
粗观测位拍一帧 → ICP 粗定位（得到工件大致位姿 + 几何中心）
  → 据此算精观测位（相机距工件 0.55 m、俯角 22°、look-at）
  → 规划并移动过去 → 在精观测位连拍 3 帧求共识位姿
```

**实际结果**：粗定位与精观测位计算都正确，**但移动阶段反复 `TIMEOUT`**：

```text
第 1 次：精观测距离 0.50 m → 夹爪间距 0.286 m（危险，未拦）
第 2 次：距离 0.72 m → 规划失败（不可达）
第 3 次：距离 0.55 m（间距 0.329 m）→ 规划成功 74 点，执行 chunk1 TIMEOUT
第 4 次（控制器重启后）：规划成功 85 点，执行 chunk1 又 TIMEOUT
```

**已确认的事实**：

- 粗定位可用：`coarse pose (0.499,0.641,0.127) coverage 86% median 3.3mm ICP_PASS`
- 工件几何中心：`(0.717, 0.624, 0.134)`
- 精观测位：`TCP (0.578, 0.449, 0.376)`，间距 0.329 m，规划器给出 74–85 点轨迹
- **卡点在执行，不在规划**

**下一步要查的（按顺序）**：

```text
① execute_assembly_chunks.py 的超时逻辑：duration + 20 s
   轨迹 85 点、time_scale 2.0 时 scaled_total 可能很短，导致 +20 s 不够
   → 建议把超时改成 max(duration+20, 45) 或按段数放大
② 控制器是否在上一段 goal 后进入拒绝状态（每次 TIMEOUT 后需 recover_control.sh）
③ 观察 arm_controller/controller_state 的 error 字段是否持续非零
```

### 3.2 【阻塞】四类标注中的桌面两类

- 木桌/金属桌的**平面标注失败**：高度判据分不开地板、幕布、墙面（实测标签覆盖整屏）
- 已降级为两类（工件 + 插槽），所有者已同意（方案 A）
- 若以后要桌面类：需要严谨的 RANSAC 平面 + 法向 + 连通域 + 已知桌面范围，或人工标注 20–30 帧

### 3.3 【未开始】

```text
· 完整任务分段试跑（A: pre_grasp / B: +grip,lift / C: +transit,staging / D: 全流程）
  脚本已就绪：scripts/run_pick_and_place.py --config config/pick_place_real.yaml --confirm --phases ...
  分段录像：scripts/record_frames.py
· 任务阶段训练数据（抓取/搬运/插入过程的图像）—— 0 张
· NBV 策略接入与评估（接口已规划，见 docs/worklogs/2026-10-24_PLAN_TASK_VIEWS.md）
· PPF 实时化（OpenCV CPU 版实测 2 分钟以上，已用 --ppf 开关隔离，默认关闭）
```

---

## 4. 数据与模型位置

```text
原始采集     ~/cs625_yolo_seg/datasets/2026-10-25_raw/         181 帧，22 批次
两类数据集   ~/cs625_yolo_seg/datasets/2026-10-25_yolo2/       93 帧（78/15）
             data.yaml: 0=workpiece  1=slot
四类数据集   ~/cs625_yolo_seg/datasets/2026-10-25_yolo4/       仅工件类可靠，勿用于训练
场景点云     ~/cs625_yolo_seg/datasets/scene_model/scene.ply   74,621 点（仅工件周围）
新模型       ~/cs625_yolo_seg/runs/segment/runs/segment/box_slot_1280/weights/best.pt
旧模型       ~/cs625_yolo_seg/runs/segment/runs/box_seg640/weights/best.pt（回滚用）
CAD 模板     ~/cs625_rvs_archive/models_20261023/弹仓_装配导出.STL
插槽点云     /mnt/d/Program Files (x86)/RobotVisionSuite/runtime/Model/slot_pointcloud20260127.ply
```

---

## 5. 基准位姿（全部已实测，见 `docs/agent_memory.md §118/§119`）

```text
工件粗观测位（发现/粗定位）
  TCP   (0.5886, 0.3482, 0.7075)
  姿态  (0.2954, −0.7023, −0.2539, −0.5958)
  J1 43.1  J2 −57.4  J3 −146.3  J4 14.4  J5 92.8  J6 89.8

工件精观测位（位姿解算）
  TCP   (0.813, 0.606, 0.436)      相机距工件 0.45 m

插槽粗观测位（评估槽口/发现）
  TCP   (0.7802, −0.1458, 0.7986)
  姿态  (0.1834, −0.8502, −0.1133, −0.4803)
  J1 −21.1  J2 −72.5  J3 −111.0  J4 −37.5  J5 128.5  J6 62.7

插槽精观测位（已按投影面积/居中择优）
  TCP   (0.788, 0.072, 0.759)      相机距插槽 0.95 m，俯角 20°，方位 +15°
  插槽投影面积 52.9%，完整入画，中心偏移 0.072
  （旧值 (0.718,0.029,0.801) 面积 36.6%/偏移 0.183，已弃用）

工件固定位姿（师兄标定，槽内为 box_in_slot）
  插槽本体  (1.2416, 0.1817, 0.4432)  quat (−0.0016, 0.0043, 0.0034, 0.99998)
  box_in_slot (1.1635, 0.2032, 0.4423)
```

---

## 6. 本次新增/修改的文件

| 文件 | 状态 | 说明 |
|---|---|---|
| `scripts/capture_dataset.py` | **新增** | 唯一采集入口，五道守卫 + `--dry-run` + `--min-clearance` + `--skip-framing` + 自动 recover |
| `scripts/auto_label_four_class.py` | **新增** | 几何标注（工件 CAD 投影 / 插槽点云投影），`--classes two/four` |
| `scripts/capture_environment.py` | **新增** | 环境宽视角采集 |
| `scripts/record_frames.py` | **新增** | 任务全程 1 Hz 录像 |
| `scripts/build_scene_model.py` | **新增** | 多帧拼点云 + 平面拟合 |
| `scripts/auto_label_frames.py` | 改写 | 单类工件几何标注（含深度一致性过滤） |
| `config/real/camera_intrinsics.json` | **新增** | 1280×960 内参 |
| `scripts/icp_pose_once.py` | 修改 | 加 `--ppf` 开关（默认关，CPU 版太慢）；锚点角块判别 |
| `scripts/check_pose_stability.py` | 修改 | 加覆盖率门禁（防 4 重对称错支） |

---

## 7. 下一步唯一推荐任务

**修通执行器超时，把"粗观测→精观测→取位姿"跑通并给出 3 帧共识数据。**

具体做法：

```bash
# 1) 复位保护停止（示教器）+ 重启驱动栈 + 激活控制器 + 重启 MoveIt
# 2) 修 scripts/execute_assembly_chunks.py 的超时
#    第 116 行: while ... time.time() - started < duration + 20
#    → 改为 < max(duration + 20, 60)
# 3) 跑 /tmp/cs625_cal/coarse_to_fine_verify.py（脚本已写好，只需改回距离 0.55）
# 4) 期望输出：position spread ≤ 2 mm，rotation spread ≤ 0.5 deg
```

跑通后再做任务分段试跑（A/B/C/D 四段）与任务阶段数据采集。

---

## 8. 命令速查

```bash
# 环境（每个 shell 必做）
cd /home/oy/projects/cs625-active-perception
source scripts/source_dev_env.sh --full

# 驱动 + MoveIt（重启后必须激活控制器！）
ros2 launch cs625_bringup real_base.launch.py launch_driver:=true \
  robot_ip:="${CS625_ROBOT_IP}" description_package:=eli_cs_robot_description \
  description_file:=cs625.urdf.xacro headless_mode:=true launch_moveit:=false \
  launch_rviz:=false activate_joint_controller:=false execute:=false require_confirmation:=true
ros2 control switch_controllers --controller-manager /eli_ros2_control_node \
  --activate arm_controller --strict
ros2 launch cs625_bringup real_moveit.launch.py \
  description_package:=eli_cs_robot_description description_file:=cs625.urdf.xacro \
  moveit_config_package:=elite_cs625_moveit_config semantic_package:=cs625_bringup \
  semantic_file:=config/cs625_active_perception.srdf launch_rviz:=false

# 相机
ros2 launch percipio_camera percipio_camera.launch.py device_ip:=192.168.1.120 \
  color_resolution:=1280x960 depth_resolution:=1280x960 depth_enable:=true \
  depth_registration_enable:=true point_cloud_enable:=false laser_power:=100 \
  depth_qos:=SENSOR_DATA color_qos:=SENSOR_DATA

# 健康检查 / 控制器恢复
python3 scripts/control_health.py
bash scripts/recover_control.sh

# 采集（先 dry-run 审查！）
python3 scripts/capture_dataset.py --dry-run --out ~/cs625_yolo_seg/datasets/2026-10-25_raw \
  --tag test --distance 0.70 --center X Y Z --part-xyz X Y Z --part-quat QX QY QZ QW \
  --azimuths -15 0 15 --elevations 20 30

# 标注 + 训练
python3 scripts/auto_label_four_class.py --classes two \
  --dataset ~/cs625_yolo_seg/datasets/2026-10-25_raw \
  --out ~/cs625_yolo_seg/datasets/2026-10-25_yolo2
cd ~/cs625_yolo_seg && yolo segment train \
  model=runs/segment/runs/box_seg640/weights/best.pt \
  data=datasets/2026-10-25_yolo2/data.yaml imgsz=1280 epochs=40 batch=4

# 验收（离线 A/B）
cd ~/cs625_yolo_seg && yolo segment val \
  model=runs/segment/runs/segment/box_slot_1280/weights/best.pt \
  data=datasets/2026-10-25_yolo2/data.yaml imgsz=1280 split=val
```

---

## 9. 本次踩过的坑（避免重复）

```text
1. 重启驱动后必须激活 arm_controller，否则臂无保持、下垂、执行被首点保护拒绝
2. 任何 goal 超时后控制器进入拒绝状态 → 必须 bash scripts/recover_control.sh
3. 长轨迹必须切更多段（77 s 轨迹切 4 段会超时，切 10 段全部通过）
4. 相机在 TCP 后方 0.296 m：相机距工件 0.45 m 时夹爪已插进工件（曾真机碰撞）
   → 工件观测必须 ≥0.68 m（间距 0.38 m）；所有者同意插槽可用 0.18–0.20 m
5. 取景必须瞄准【几何中心】而不是 ICP 输出的模型原点（角点）：
   center = t + R(quat) @ [−0.208, 0.066, −0.011]
6. 不能用 YOLO 判断"有没有拍到目标"——它会对幕布检测出 76% 掩膜
7. 相机设备会掉线但进程还在（ICP 无限等帧）→ 重启相机；numpy 2.x 会打崩 torch/scipy
8. RVS 启动：Start-Process 'C:\Users\Public\Desktop\RobotVisionSuite.lnk'
   然后在界面里 文件→打开 CS625_SlotBox_VisualGuidance_RVS20260627-01_WSL.rvs → 启动
   命令行传工程路径无效（会开成 new.rvs）
```

---

## 10. 安全提醒（真机）

```text
· 速度：velocity_scale ≤ 0.05，time_scale ≥ 2.0（这是本机"正常速度"）
· J1/J2/J3 保持基本角度不动，调视角只用 J4/J5/J6
· 任何单次关节变化 ≤ 30°，且必须过碰撞检查
· 禁止用 TCP 位姿目标做大范围重构（规划器会选绕 J1 转 160–200° 的路径，已两次擦碰）
· 大位移一律"垂直抬升 → 高位平移 → 下落"
· 当前 safety_mode=7，动臂前必须先复位
```
