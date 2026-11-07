---
title: "2026-10-20 WORKLOG：冻结师兄自动化栈为 WSL 独立 underlay"
date: 2026-10-20
type: WORKLOG
status: done
---

# 冻结师兄自动化栈为 WSL 独立 underlay（P0 + 构建冒烟）

## 一、本轮结论

1. 按所有者决定 **"WSL 主线 + VM 兜底"**，师兄的 ROS 自动化栈已冻结为独立 underlay：
   `~/cs625_legacy_underlay_jazzy/`（14 个包），快照在
   `~/cs625_legacy_snapshot_20261020/`（tar + SHA256SUMS + 清单 + 来源说明 + 构建日志）。
2. 来源选 **2026-04-14 备份 zip**，不选 D 盘工作区：zip 是最后一个接口自洽、完整构建过的
   工作区，且包含 D 盘工作区缺失的 `cs625_compliant_placement` 与
   `cs625_tcp_force_probe`；D 盘工作区缺少这两包、把 compliant 包拍平、且状态消息更旧。
3. **14/14 包在 Jazzy 下零源码修改编译通过，总耗时 1 分 06 秒**；`cs625_compliant_placement`、
   `cs625_state_monitor`、`cs625_trajectory_tools`、`cs625_kinematics`、`tcp_bridge` 等
   可执行文件均已安装并可被 `ros2 pkg executables` 列出。
4. 这消除了"移植到 Jazzy 需要大量修补"的主要风险；剩余风险转向运行时（真机状态端口、
   RVS 位姿入口、MoveIt 场景与安全边界）。

## 二、来源与选择依据

| 项 | 值 |
|---|---|
| 源 zip | `/mnt/d/Program Files (x86)/RobotVisionSuite/runtime/Model/newfolder/elite_ros_ws20260414.zip` |
| SHA-256 | `d19cef1917bc2662fba615302dd792b866a321d9ba1fab5f4779b741ca8b5b18` |
| D 盘工作区 | `.../newfolder/elite_ros_ws`，HEAD `ad730f1`，`master`，364 个未提交改动，远端 `elite_robot_project_20260306`（GitHub 不存在） |
| 关键差异 | D 盘缺 `cs625_compliant_placement`、`cs625_tcp_force_probe`；`CS625State.msg` 缺 `has_actual_tcp_wrench` 与 TCP 力/力矩字段 |

选择规则：**接口自洽 + 可构建 + 完整**优先于"看起来更新"。zip 内构建日志显示
2026-04-13 22:15 已成功构建 compliant placement，因此以 zip 为唯一来源，D 盘工作区只作
交叉核对。

## 三、执行的命令（可复现）

```bash
# 1. 从 zip 只解出 14 个缺失包（不碰 build/install/log）
NF="/mnt/d/Program Files (x86)/RobotVisionSuite/runtime/Model/newfolder"
unzip -o -q "$NF/elite_ros_ws20260414.zip" \
  'elite_ros_ws/src/cs625_*/*' 'elite_ros_ws/src/eli_cs_robot_calibration/*' \
  'elite_ros_ws/src/environment_*/*' 'elite_ros_ws/src/model_visualizer/*' \
  'elite_ros_ws/src/pointcloud_workflow_interfaces/*' 'elite_ros_ws/src/tcp_bridge/*' \
  'elite_ros_ws/src/vision_bridge/*' 'elite_ros_ws/src/workspace_pointcloud_sampler/*' \
  -d /tmp/legacy_zip_src

# 2. 建立独立 underlay，清掉生成物
cp -a /tmp/legacy_zip_src/elite_ros_ws/src/. "$HOME/cs625_legacy_underlay_jazzy/src/"
find "$HOME/cs625_legacy_underlay_jazzy/src" -type d -name __pycache__ -prune -exec rm -rf {} +
find "$HOME/cs625_legacy_underlay_jazzy/src" -name '*.pyc' -delete

# 3. 快照 + 校验
tar czf "$HOME/cs625_legacy_snapshot_20261020/legacy_src_14pkgs_20261020.tar.gz" \
  -C "$HOME/cs625_legacy_underlay_jazzy" src
cd "$HOME/cs625_legacy_underlay_jazzy" && \
  find src -type f -print0 | sort -z | xargs -0 sha256sum \
  > "$HOME/cs625_legacy_snapshot_20261020/SHA256SUMS"

# 4. 构建（Jazzy）
source /opt/ros/jazzy/setup.bash
source "$HOME/cs625_underlay_jazzy/install/setup.bash"
cd "$HOME/cs625_legacy_underlay_jazzy"
colcon build --symlink-install

# 5. 验证
source "$HOME/cs625_legacy_underlay_jazzy/install/setup.bash"
ros2 pkg executables cs625_compliant_placement
ros2 pkg executables cs625_trajectory_tools
```

## 四、结果

```text
colcon build: Summary: 14 packages finished [1min 6s]
cs625_compliant_placement  0.64 s     cs625_state_monitor  0.45 s
cs625_trajectory_tools    37.0 s      cs625_kinematics    22.3 s
cs625_task_manager        20.3 s      tcp_bridge           1.48 s
environment_collision_generator 1min 06s   environment_point_cloud_publisher 1min 02s
...
ros2 pkg executables:
  cs625_compliant_placement  compliant_placement_node
  cs625_state_monitor        raw_state_receiver_node, state_csv_logger_node
  cs625_trajectory_tools     moveit_planner_client_node, display_trajectory_bridge_node, planned_tcp_path_node
  cs625_kinematics           ik_server_node, preview_joint_state_relay, preview_robot_description_publisher
  tcp_bridge                 robot_commander, tcp_server, pose_to_tf_broadcaster, collision_object_publisher, ...
```

## 五、改动的文件

仓库内（已通过静态契约检查）：

- 新增 `docs/decisions/ADR-004-legacy-senior-underlay.md`（环境决策与 underlay 规则）；
- `docs/dependencies.md`：新增 "Legacy automation underlay" 小节（来源、哈希、包清单、排除项、边界）；
- `docs/migration_from_legacy.md`：追加 2026-10-20 说明（Humble 行为历史、当前 Jazzy 主线）；
- 本工作日志。

仓库外（WSL）：

- `~/cs625_legacy_underlay_jazzy/src/` 14 个包 + `build/install/log`；
- `~/cs625_legacy_snapshot_20261020/`（tar、SHA256SUMS、PACKAGE_MANIFEST.tsv、SOURCE_PROVENANCE.md、BUILD_STATUS.md、build_20261020.log）。

未改动：本仓库 `src/`、任何 launch、任何测试、任何仿真世界与配置。

## 六、能力层与关键链状态

```text
Capability layer
- environment and dependencies : PASS（Jazzy 下 14/14 编译）
- robot model and simulation   : NOT INVOLVED（本轮未起仿真/未动模型）
- kinematics, control, planning: BUILD PASS；RUNTIME NOT STARTED（未连机器人）
- vision, hand-eye, TF         : BUILD PASS（tcp_bridge/vision_bridge）；RUNTIME NOT STARTED
- active perception / NBV      : NOT STARTED（与本轮无关）
- real hardware and safety     : NOT STARTED（本轮无真机、无运动）

Critical-chain status
- URDF -> Gazebo entity        : NOT INVOLVED
- ros2_control -> joint_states : NOT INVOLVED
- base_link -> camera optical  : NOT INVOLVED
- RGB-D -> normalized topics   : NOT INVOLVED
- point cloud -> planning scene: BUILD PASS（environment_* 包）；runtime NOT STARTED
- NBV decision -> execution    : NOT STARTED
```

## 七、仿真/真机对齐

- 复用共同核心：不涉及（本轮是 underlay 冻结，不是应用层）。
- 仿真入口：未变（`sim_*` 仍走现有仓库）。
- 真机入口：未变；师兄 VM 作为兜底参考。
- 真机执行允许：否。`execute:=false` 默认未变。
- 真机运动：无。

## 八、限制与未做

1. **未做运行时验证**：没有机器人、没有 RVS、没有起 Gazebo；只验证了编译与安装布局。
2. 师兄包的运行时依赖（状态端口 30001、MoveIt 场景、RVS TCP 位姿）尚未接入。
3. D 盘工作区仍是 364 个未提交改动的脏工作区，不作为来源；其 `0706` 之后的 RVS 工程
   （`20260626/20260627` 版本）只做了路径登记，未复制。
4. 代码版权属师兄：未经同意不得公开发布；论文引用与致谢待写。
5. `tcp_bridge/robot_commander` 直控路径未用于任何自动运动（安全规则）。

## 九、下一步（P2/P3）

1. **P2 运行时冒烟**：无机器人时用录包/桩发布 `/cs625/raw_state`，让
   `cs625_compliant_placement` 跑起来；验证三个服务（Start/Stop/Get）与状态消息。
2. **P3 RVS 缝**：Windows 侧发一个测试位姿到 WSL TCP 入口，验证 `/target_pose`
   与 MoveIt 规划（只规划、不执行）。
3. 两项通过后，再进入"自动决策 + 失败码 + 有界重试 + 急停监控"的 P4。

## 十、P2 补充（2026-10-20 15:06）：zip 内 compliant 是旧版，已重构并冒烟

- zip 内 `cs625_compliant_placement` 只有 `get_compliant_placement_state`（REPORT_ONLY），
  没有 Start/Stop；带 Start/Stop 的新版只存在于 RVS 根目录的拍平副本。
- 已按新版 CMakeLists 期望的布局重构（只搬位置、零逻辑修改），重编译通过；
  来源文件逐个哈希见快照 `cs625_compliant_placement_RECONSTRUCTED.tsv`。
- 运行时冒烟 8/8 通过（桩发布 `/cs625/raw_state`）：到位、软接近、接触保持、
  安全中止、Stop、GetState 全通；证据见快照 `P2_SMOKE_20261020.txt`。
- 关键发现：该版本**卡滞检测未实现**（`jam_detected` 永远 false；yaml 的 progress 参数
  节点未声明）；节点是监督器，不执行运动。详见 P2 工作日志。
