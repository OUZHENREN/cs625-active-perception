# 2026-10-23 WORKLOG — 手眼标定同步 + 深度单位修正（真机、零运动标定）

> 起点：`docs/worklogs/2026-10-23_HANDOFF_NEXT_SESSION_START.md` §11.11 第 1–4 步。
> 终点：1.4 m 横向偏差定位到两个独立根因并修掉；coarse 干跑目标点落回期望区间。
> 本轮**没有发送任何机械臂运动指令**（只重启了相机节点）。

---

## 1. 一句话结论

偏差不是"相机姿态 × 远距离放大"，而是两处量级错误：**真机位姿链用了手搓外参而没用
RVS 手眼标定（差 135° / 0.34 m）**，以及**深度把设备计数当毫米发布（大了 4 倍）**。
修完后同一帧的 coarse 目标点从 `(-0.376, -1.054, 0.265)`（2.02 m）变成
`(0.851, 0.616, 0.212)`（0.48 m），落在期望区间 `(0.6–0.9, 0.4–0.8, 0.1–0.4)` 内。

---

## 2. 现场状态与设备

```text
机器人工况   safety_mode=1、robot_mode=7（未复核二次，因本轮不动臂）
臂位姿       TCP 0.7999, 0.5601, 0.5002   q = (-0.4146, 0.7772, 0.1589, 0.4458)
             来自 /tcp_pose_broadcaster/pose（frame: base，帧名 my_end_effector_link）
相机         Percipio PS800-E1，GigE，192.168.1.120，640x480 彩色 + 640x480 深度（配准开）
使用环境     真机（不是 fake hardware、不是 Gazebo）；RVS 未运行，ROS 独占相机
真实运动     无。仅停/起相机节点，机械臂保持不动
```

开工时发现的问题：**3 个 `percipio_camera` 容器同时在跑**（19:19 / 20:09 / 20:43 三次
launch 叠加），其中一个已收到 SIGINT 但进程仍在，`/camera/*` 有 3 个发布者、节点同名。
已按 PID 逐个 `kill -INT` → `kill -TERM`，然后单独起一份 640×480 的相机。

---

## 3. 根因 A：真机链没有用 RVS 手眼标定

### 3.1 事实

```text
RVS 手眼输出（存在于 <RVS>/runtime）：
  HandEyeTool.ini      [colorToRobot] 96.95 42.212 97.54 mm, 179.96 179.215 89.262 deg
                       [depthToRobot] 96.754 17.803 97.328 mm, -179.861 179.987 89.451 deg
  ColorToRobotTCP.txt  同一组数的 m / rad 编码
  DepthToRobotTCP.txt
  → 参考帧是【法兰】，标定帧是【相机光心光学帧】

仓库里已经有的推导：
  scripts/camera_extrinsics.py → src/cs625_bringup/config/camera_extrinsics_sim.yaml
  （只被仿真 launch 读，真机链没读）

真机链实际用的：
  config/handeye/tcp_to_camera.json —— 手写的“相机几乎在 TCP 上、45° 下倾”
  T_tcp_cam 平移 (0.050, 0.000, 0.050)

两者对比（当前臂位姿下计算）：
  平移差 0.34 m，旋转差 135.1°
```

### 3.2 为什么原来的值看起来"能用"

手写值的相机位置就在 TCP 旁边，视觉上"相机装在末端"，很容易被认为合理；但真正的
RVS 标定是相机在 TCP 后方 296 mm、光轴沿工具轴 +Z。用错之后，目标点的横向分量随
"相机到目标距离"被放大，正好表现为米级横向偏差。

### 3.3 修法

```text
T_base_camera = T_base_tcp @ T_tcp_cam
T_tcp_cam     = inv(T_flange_tcp) @ T_flange_cam_color_optical
T_flange_tcp  = (-0.046, 0, 0.353)   ← URDF 固定关节 flange→my_end_effector_link
                                       （也就是 TCP 广播器报告的帧）
```

新增 `scripts/sync_real_handeye.py`：从 RVS 三个文件推导，mm/deg 与 m/rad 互相校验
（不一致直接中止），写 `config/handeye/tcp_to_camera.json`，并支持 `--check` 防漂移。
文件里同时存 `T_flange_tcp` / `T_flange_cam_color_optical` / `T_tcp_cam` 和来源说明，
**不允许再手写**。

新增 `test/test_real_handeye_chain.py`：锁住三件事——`T_tcp_cam` 必须等于它声称的
组合、光轴必须大致沿法兰 +Z、相机到 TCP 必须约 296 mm（"相机贴在 TCP 上"的回归会
直接失败），并与仿真外参交叉比对（两边必须来自同一次标定）。

---

## 4. 根因 B：深度单位是 0.25 mm/计数，却按毫米发布

### 4.1 发现方式

新起的相机日志里有一行：

```text
[INFO] [percipio_device]: Depth scale unit: 0.25
```

对应源码 `percipio_camera_node.cpp:249` 的 `depth_scale_ = device_ptr_->getDepthValueScale();`
——读进来了，但 `publishDepthFrame()` 从未使用，直接把原始计数当 `16UC1` 毫米发出去。

### 4.2 用已知尺寸验证（与位姿无关的地面真值）

工件面 441 mm 是已知量，被测物自己的位姿不影响"尺寸"。同一帧、同一掩膜：

| 深度假设 | 反投影尺寸 (m) | 结论 |
|---|---|---|
| 1.00 mm/计数 | 2.331 × 1.763 × 1.733 | 不可能（真实 0.114×0.441×0.465） |
| 0.25 mm/计数 | 0.583 × 0.441 × 0.439 | 与已知 441 mm 面吻合 |

→ 深度确实大了 4 倍。这也推翻了 §108 里"掩膜像素查深度存在配准偏差"的判断：
**配准是对的**（彩色与深度内参完全相同、剪影对齐），错的是标尺。

### 4.3 修法

改在驱动里（唯一一处，消费者不用各自猜）：

```text
~/cs625_camport_ws/src/camport_ros2/.../percipio_camera_node.cpp  publishDepthFrame()
  Coord3D_C16 + depth_scale_ != 1 时：raw * depth_scale_ 四舍五入成毫米再发；
  0（无测量）保持 0。
percipio_camera_node.h：加一个复用的 depth_mm_buffer_。
```

这是该 fork 的第 3 处本地改动，已登记进 `docs/dependencies.md`（含前两处 IP 打开）。
重新编译：

```bash
source /opt/ros/jazzy/setup.bash
cd ~/cs625_camport_ws && colcon build --symlink-install --packages-select percipio_camera
# Finished <<< percipio_camera [1min 3s]
```

---

## 5. 零运动「已知靶点」标定

新增 `scripts/known_target_calibration.py`：抓一帧稳定的 RGB-D，用三个互相独立的地面
真值把链路拆开定位——

| 检查 | 地面真值 | 失败意味着什么 |
|---|---|---|
| 工件主轴长度 | 工件自身尺寸 114×441×465 mm | 深度标尺错（4 倍时会得到 1.7 m） |
| 桌面法向 | 桌面是水平面 | 旋转错（135° 外参错误时桌面倾斜 48°） |
| 桌面高度 | 历史实测桌面带 z = -0.13..-0.07 m | 平移错（TCP 偏置 / 手眼平移 / 相机重装） |

实测输出（2026-10-23 22:47，单帧）：

```text
camera in base     [0.586, 0.356, 0.518]   (camera->TCP 296 mm)
tcp in base        [0.8, 0.56, 0.5]
workpiece axes     [0.493, 0.406, 0.352] m  vs known 0.114/0.441/0.465
                   middle 0.406 m -> PASS
table plane        23745/94585 inliers  rms 1.2 mm
table normal base  [0.007, -0.012, 1.0]  tilt 0.8 deg -> PASS
table height       -0.103 m  expected -0.13..-0.07 m -> PASS
workpiece centroid [0.935, 0.736, 0.218] m (base frame)
RESULT: PASS
```

对照（修之前同样的量）：桌面法向倾角 47.9°、工件尺寸 1.76 m、目标点 2.02 m。

> 说明：桌面法向与高度是这一帧的**工作面**真值，与 §11.7 里记录的 "(0.6–0.9, 0.4–0.8,
> 0.1–0.4)" 是同一套坐标约定；工件当前实际位置由所有者摆放，本轮不假设它等于历史值。

重复性（同一臂位姿连测 3 帧）：

```text
工件质心   (0.935, 0.735, 0.219) / (0.935, 0.736, 0.218) / (0.935, 0.736, 0.218)
桌面高度   -0.103 / -0.104 / -0.102 m        桌面倾角 0.8° ×3
→ 静止帧之间的离散 ≤ 2 mm，说明偏差已不是随机量，剩下的都是可复现的系统量
```


---

## 6. §11.11 第 4 步验收：coarse 干跑

```bash
python3 scripts/coarse_to_fine_pose.py \
  --weights ~/cs625_yolo_seg/runs/segment/runs/box_seg640/weights/best.pt \
  --template ~/cs625_rvs_archive/models_20261023/弹仓_simplified_5mm_v2.stl
```

```text
coarse: u=330 v=277 z=0.48m area=0.52 pts=124572
camera pose from TCP x calibrated offset
target in base: [0.851, 0.616, 0.212]
current distance 0.48 m -> fine TCP [0.301, 0.076, 0.849]
dry run: pass --confirm to move
```

| | 修前（§11.7 记录） | 修后（本轮） |
|---|---|---|
| 目标点 | (-0.376, -1.054, 0.265) | (0.851, 0.616, 0.212) |
| 距离 | 2.02 m | 0.48 m |
| 是否落入 (0.6–0.9, 0.4–0.8, 0.1–0.4) | 否 | 是 |

---

## 7. 精确命令与结果

```bash
# 环境（必须，否则 contract_checks 会拒绝）
source scripts/source_dev_env.sh --full

# 相机（单独起一份 640x480，彩色+深度+配准）
source ~/cs625_camport_ws/install/setup.bash
ros2 launch percipio_camera percipio_camera.launch.py device_ip:=192.168.1.120 \
  color_resolution:=640x480 depth_resolution:=640x480 \
  depth_enable:=true depth_registration_enable:=true point_cloud_enable:=false

# 手眼同步（--check 防漂移）
python3 scripts/sync_real_handeye.py --rvs-dir "/mnt/d/Program Files (x86)/RobotVisionSuite/runtime"
python3 scripts/sync_real_handeye.py --rvs-dir "<同上>" --check

# 已知靶点标定
python3 scripts/known_target_calibration.py \
  --weights ~/cs625_yolo_seg/runs/segment/runs/box_seg640/weights/best.pt

# 测试
python3 -m pytest test/test_real_handeye_chain.py test/test_camera_extrinsics.py -q
# 15 passed
python3 test/contract_checks.py
# Phase 0–6 static contract checks: PASS
```

相机侧实测：

```text
depth topic     /camera/depth/image_raw  16UC1 640x480
镜头            color 与 depth 内参完全一致 fx=465.74 fy=465.80 cx=324.35 cy=224.66
帧率            彩色约 23.5 fps（只开 RGB）；带深度约 0.74 fps（深度硬件上限）
深度有效点      约 72%，中位 759 mm（修前中位 2351“mm”）
```

---

## 8. 环境注意事项（不是代码问题，但会让人误判成"相机没数据"）

```text
现象：相机 topic 在、camera_info 有数据，但 image/depth 一帧都收不到；
      报 RTPS_TRANSPORT_SHM "Failed to create segment ...: Permission denied"
原因：ROS 2 图像约 1 MB/帧，走 DDS 共享内存（/dev/shm）；
      受限文件策略下 /dev/shm 不可写，只能退到 UDP，
      而本机 60 KB 的 UDP 数据报也收不到（实测 1000 B 可、60000 B 丢）
结论：需要 /dev/shm 可写才能收图；普通商品脚本（rgb_observer 等）在正常终端里不受影响
```

已实测的最小复现：两个自有进程互发 640×480 `Image`，共享内存可用时收到，不可用时 0 帧。

---

## 8a. 文档镜像状态（未完成，需所有者处理）

```text
bash scripts/sync_worklog.sh docs/worklogs/2026-10-23_WORKLOG_HANDEYE_SYNC_AND_DEPTH_UNITS.md
→ ERROR: Obsidian work-log directory does not exist:
  /mnt/g/我的云端硬盘/Obsidian_Vault/OY/Do be do be do/04-课题与项目/项目/工作日志
WSL 里看不到 G 盘。按 §10 的说明需要用 PowerShell 从 \\wsl.localhost\... 复制，
或者挂载 G 盘后重跑该脚本。
```


---

## 9. 改动文件

```text
新增  scripts/sync_real_handeye.py             RVS 手眼 → 真机常量（带 --check）
新增  scripts/known_target_calibration.py      零运动已知靶点标定（PASS/FAIL）
新增  test/test_real_handeye_chain.py          外参组合/光轴/距离/仿真一致性回归
改写  config/handeye/tcp_to_camera.json        由手写值改为脚本生成（含来源与组合式）
文档  docs/dependencies.md                     camport_ros2 fork 第 3 处改动 + 使用方式
文档  docs/agent_memory.md                     §112 两个根因与验证方法
驱动  ~/cs625_camport_ws/src/camport_ros2（3 个文件）深度按 DepthScaleUnit 转毫米
```

未改动：`scripts/coarse_to_fine_pose.py`（读的是同一路径的 `T_tcp_cam`，数据修好后
无需改代码）、MoveIt/SRDF、任何运动脚本。

---

## 10. Capability layer

```text
环境与依赖        PASS    Jazzy + underlay + 本仓库 + camport overlay 链正常；
                         contract_checks 通过
机器人模型与仿真  不涉及   本轮不碰仿真
运动学/控制/规划  NOT RUN 本轮未运动；驱动与 MoveIt 仍在原会话运行
视觉/手眼/TF      PASS    已知靶点标定三项全过（尺寸/法向/高度）；
                         TF 双树仍未解，继续用常量绕开（§11.5 方案）
主动感知 / NBV    不涉及   前置的相机链路现在才算第一次可信
真机集成与安全    PASS    未发送运动指令；相机节点重启属传感器侧操作
```

### Acceptance target

```text
目标      在不动机械臂的前提下，定位并修掉 1.4 m 横向偏差的来源
通过标准  已知几何三项 PASS + coarse 干跑目标点落回 (0.6–0.9, 0.4–0.8, 0.1–0.4)
运行证据  本文件 §5、§6 的命令与输出；pytest 15 passed；contract_checks PASS
```

### Critical-chain status

```text
URDF → Gazebo entity              NOT RUN     本轮不涉及仿真
ros2_control → joint_states       PASS        驱动在跑，控制器 active（沿用开工状态）
base_link → camera optical TF      BLOCKED     TF 双树仍存在；本轮用常量链绕开
RGB-D → normalized sensor topics   PASS        640x480 彩色+深度，深度已换算成毫米
point cloud → MoveIt planning scene NOT RUN    未更新场景
NBV decision → robot execution     NOT STARTED 本轮不做
```

### Sim/real alignment

```text
复用公共核心      是：真机链与仿真外参现在同源于 RVS 标定（test 交叉比对）
仿真入口          sim_control.launch.py（读 camera_extrinsics_sim.yaml）
真机入口          known_target_calibration.py / coarse_to_fine_pose.py（读 config/handeye）
允许真机执行      否（默认 execute:=false, require_confirmation:=true）
执行/确认闸门     保持关闭；本轮 0 条运动指令
```

### Intentionally not done

```text
· §11.11 第 5 步（coarse_to_fine --confirm 实跑）——需要动臂，且算出的精观测位
  (0.301, 0.076, 0.849) 还没做可达性/碰撞检查，不能直接执行
· 掩膜粘连桌面（§11.8）未重训；本轮用"桌面平面剔除"兜底，只够做标定，不够做位姿
· TF 统一发布源（关掉相机驱动自带 TF、把相机固定关节接进 robot_state_publisher）未做
· 六类模式与三轮调试（§7）未开始
· 未修改任何 vendor 包以外的第三方代码；camport 改动均在本地 fork 并已登记
```

---

## 11. 本轮必须理解的 3 个知识点

1. **"手眼标定"要落到 ROS 的是组合式，不是一个数**：RVS 给的是
   `flange → 相机光心`，而位姿链的输入是 `base → TCP`，中间还差 URDF 的
   `flange → my_end_effector_link`。少算这一步，标定再准也会系统性偏。
2. **深度相机有两种"毫米"**：设备原始计数（这里 0.25 mm/计数）与 ROS 约定的毫米。
   驱动有 `DepthScaleUnit` 不用，就只能靠"已知尺寸"才能发现；而尺寸与位姿无关，
   所以它是检验深度标尺最干净的靶点。
3. **一个量级错误可以伪装成两个错误**：深度大 4 倍时，点云尺寸大 4 倍、位姿质心也
   偏，看起来像"配准错了"或"外参错了"。用三个互相独立的地面真值（尺寸 / 水平面
   法向 / 桌面高度）才能把标尺、旋转、平移分别钉住。

---

## 12. 所有者应该亲手运行的验证命令

```bash
# ① 相机有没有数据（应看到 0.7–1 fps，frame_id 为 camera_*_optical_frame）
source /opt/ros/jazzy/setup.bash
ros2 topic hz /camera/depth/image_raw

# ② 手眼常量有没有漂移（应打印 matches the calibration）
python3 scripts/sync_real_handeye.py \
  --rvs-dir "/mnt/d/Program Files (x86)/RobotVisionSuite/runtime" --check

# ③ 已知靶点标定（应 RESULT: PASS，桌面倾角 < 1°、高度约 -0.10 m）
python3 scripts/known_target_calibration.py \
  --weights ~/cs625_yolo_seg/runs/segment/runs/box_seg640/weights/best.pt
```

---

## 13. 下一步唯一推荐任务

**做 §11.11 第 5 步前的安全闸门**：把 coarse 推出的精观测位
`(0.301, 0.076, 0.849)` 先过一遍可达性 + 碰撞检查（`check_state_validity`，不允许
读规划场景位姿做几何判断——§4.3），确认后再用 `--confirm` 实跑一次
`coarse_to_fine_pose.py`，验证"粗→精"闭环。
