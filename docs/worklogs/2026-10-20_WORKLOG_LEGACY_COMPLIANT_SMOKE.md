---
title: "2026-10-20 WORKLOG：师兄柔顺放置节点的版本还原与 P2 运行时冒烟"
date: 2026-10-20
type: WORKLOG
status: done
---

# 师兄柔顺放置节点：版本还原 + P2 运行时冒烟（无机器人）

## 一、本轮结论

1. 发现 zip 内的 `cs625_compliant_placement` 是**旧版 REPORT_ONLY**（只有 GetState），
   带 Start/Stop 的新版只存在于 RVS 根目录的拍平副本；已按新版自己的 CMakeLists
   期望布局**重构（只搬文件位置、零逻辑修改）**并编译通过。
2. 用桩发布 `/cs625/raw_state` 做运行时冒烟：**8/8 通过**——初始状态、Start 接受、到位、
   软接近、接触保持、安全中止、Stop(abort)、GetState。
3. **关键发现（对论文第 4 章直接相关）**：这一版的**卡滞检测没有实现**——
   `PlacementController::update()` 从不置 `jam_detected`，节点也不声明 yaml 中的
   `progress_window_sec / progress_epsilon_mm / *_confirm_cycles`。
   所以第 4 章不能写"已有卡滞检测"，只能写"复用接触检测与安全中止，卡滞判据由本文补"。
4. 该节点是**监督器**而非执行器：只发布状态、按阶段请求
   `/io_and_status_controller/set_speed_slider`；运动仍由外部（RVS 插件 / MoveIt 客户端）执行。

## 二、版本还原

| 项 | 旧版（zip） | 新版（RVS 根目录，拍平） |
|---|---|---|
| 服务 | 仅 `get_compliant_placement_state` | Start / Stop / Get 三个 |
| 依赖 | 无 `eli_common_interface` | 依赖 `eli_common_interface`（速度滑块） |
| 配置 | 仅两个 alpha | 速度档、阈值、容差、jam 开关等 |
| 控制器 | 无 | `placement_controller.cpp`（阶段机） |
| 布局 | 标准包布局 | 文件全部拍平在目录根，CMakeLists 不可直接编译 |

重构方式：按新版 `CMakeLists.txt` 期望的布局放置
`src/`、`include/cs625_compliant_placement/`、`msg/`、`srv/`、`config/`、`launch/`；
**未改任何源码内容**。来源文件逐个 SHA-256 记录在
`~/cs625_legacy_snapshot_20261020/cs625_compliant_placement_RECONSTRUCTED.tsv`。

## 三、命令与结果

```bash
# 1. 重构后重编译
source /opt/ros/jazzy/setup.bash
source "$HOME/cs625_underlay_jazzy/install/setup.bash"
source "$HOME/cs625_legacy_underlay_jazzy/install/setup.bash"
cd "$HOME/cs625_legacy_underlay_jazzy"
colcon build --symlink-install --packages-select cs625_compliant_placement
# Finished <<< cs625_compliant_placement [9.90s]（stderr 仅 9p 时钟偏移告警）

# 2. 起节点
ros2 run cs625_compliant_placement compliant_placement_node \
  --ros-args --params-file "<install>/share/cs625_compliant_placement/config/compliant_placement.yaml" &

# 3. 冒烟
python3 "$HOME/cs625_legacy_underlay_jazzy/tools/compliant_smoke_test.py"
```

```text
=== P2 smoke result ===
PASS  initial_state
PASS  start_accepted
PASS  target_reached
PASS  soft_approach
PASS  contact_hold
PASS  safety_abort
PASS  stop_abort
PASS  get_state
total=8 failed=0
```

日志中的 `FastDDS SHM port` 告警与 `set_speed_slider service not ready` 均为预期环境噪声
（无 ros2_control IO 控制器），不影响状态逻辑。

## 四、能力边界（本轮确认）

```text
接触检测      : 已实现（六关节力矩绝对值之和的 EMA 残差；0.8 接触 / 2.0 安全）
软接近/保持   : 已实现（速度档 0.30 / 0.10 / 0.05）
到位判定      : 已实现（欧氏距离 ≤ 3 mm）
安全中止      : 已实现（残差超安全阈值 → ABORTED；停止活跃）
卡滞检测      : 未实现（jam_detected 永远 false；progress 参数未声明）
运动执行      : 不负责（监督器；外部规划器执行）
失败码/重试   : 无（这是本文自动决策层要补的）
```

## 五、改动文件

仓库内：

- 新增 `test/legacy_compliant_smoke.py`（P2 冒烟脚本；需要先 source 两个 underlay）；
- `docs/dependencies.md`：追加 P2 说明（版本还原、8/8、能力边界、卡滞缺口）；
- `docs/worklogs/2026-10-20_WORKLOG_LEGACY_UNDERLAY_SNAPSHOT.md`：追加 P2 补充；
- 本工作日志。

仓库外（WSL）：

- `~/cs625_legacy_underlay_jazzy/src/cs625_compliant_placement/`：替换为重构版；
- `~/cs625_legacy_underlay_jazzy/tools/compliant_smoke_test.py`；
- `~/cs625_legacy_snapshot_20261020/`：新增 `legacy_src_14pkgs_20261020_r2.tar.gz`、
  当前 `SHA256SUMS`（旧版保留为 `SHA256SUMS.as_extracted`）、
  `cs625_compliant_placement_RECONSTRUCTED.tsv`、`P2_SMOKE_20261020.txt`；
  `SOURCE_PROVENANCE.md` / `BUILD_STATUS.md` 追加 §10 与 P2 更新。

未改动：仓库 `src/`、任何 launch、仿真世界与配置；未起 Gazebo；未连机器人；无真实运动。

## 六、能力层与关键链状态

```text
Capability layer
- environment and dependencies : PASS（重构包重编译 9.9 s）
- robot model and simulation   : NOT INVOLVED
- kinematics, control, planning: RUNTIME NOT STARTED（节点不执行运动）
- vision, hand-eye, TF         : NOT INVOLVED
- active perception / NBV      : NOT STARTED
- real hardware and safety     : NOT STARTED（无真机；安全中止为软件模拟）

Critical-chain status
- URDF -> Gazebo entity        : NOT INVOLVED
- ros2_control -> joint_states : NOT INVOLVED（速度滑块服务缺失，仅告警）
- base_link -> camera optical  : NOT INVOLVED
- RGB-D -> normalized topics   : NOT INVOLVED
- point cloud -> planning scene: NOT STARTED（本轮未涉及）
- NBV decision -> execution    : NOT STARTED
```

## 七、仿真/真机对齐

- 复用共同核心：是（legacy underlay 的节点与消息）。
- 仿真入口：未变。
- 真机入口：未变；本轮未连真机。
- 真机执行允许：否；`execute:=false` 未改。
- 真机运动：无。

## 八、限制与下一步

1. 本轮是**软件在环**冒烟：桩状态、无运动、无 RVS、无 MoveIt 场景；阈值未用真机数据标定。
2. 卡滞检测需要补：progress-window（固定时间窗内 TCP 位移小于 ε）→ `jam_detected`；
   并把它接入失败码与有界重试。
3. 下一步 **P3**：Windows RVS → WSL TCP 位姿入口，只规划不执行；确认 `/target_pose`
   与 MoveIt 规划链路。
4. P4：在 reused 节点之上补自动决策层（失败码、有界重试、急停监控），并把
   `jam_detected` 的缺口补上。
