# 2026-09-16 工作日志：Gazebo 可视化开关、真机/仿真双底座、RViz 面板修复

- 日期：2026-09-16
- 仓库：`cs625-active-perception`
- 范围：仅仿真 + 真机**软件**底座；未连接真实机器人，未发生真实运动
- 本次提交：`998b62e`、`4ccbbce`、`528d357`（未 push）

## 一、起因

手工运行 Gazebo 仿真时出现两个现象：

1. Gazebo 里看不到机械臂；
2. RViz 不显示。

## 二、根因与修复

### 2.1 Gazebo 机械臂不可见 —— 设计行为，非故障

`sim_control.launch.py` 会调用 `prepare_gazebo_model.py`，把 URDF 里所有基于 mesh 的
`<visual>` 删除后再 `ros_gz_sim create`。运行日志：

```
Prepared Gazebo model at /tmp/cs625_active_perception_gazebo_model.urdf
  (removed 8 mesh visual elements).
```

目的是避免 WSL2 软件渲染路径下 RGB-D 渲染场景卡死。连杆、关节、碰撞、`ros2_control`、
相机都保留，只少了外观网格。

新增开关 `gazebo_visuals`（默认 `false`）：

- `prepare_gazebo_model.py` 增加 `--keep-visuals`；
- `sim_control.launch.py` 声明并透传；
- `sim_base.launch.py`、`sim_view_planning.launch.py` 暴露给用户；
- 用法：`ros2 launch cs625_bringup sim_base.launch.py headless:=false gazebo_visuals:=true launch_rviz:=true`。

### 2.2 RViz 不显示 —— 真 bug

`sim_base.launch.py` 给官方控制 launch 的 include 硬编码了 `"launch_rviz": "false"`。
ROS 2 的 `IncludeLaunchDescription` 会把被包含 launch **未声明**的参数写成**全局** launch
configuration，而 `sim_control.launch.py` 并不声明 `launch_rviz`，于是该 `false` 泄漏到全局，
12 秒后启动的 `sim_moveit.launch.py` 读到 `false`，RViz 节点被条件判断跳过 —— 即使显式传了
`launch_rviz:=true`。

修复：官方 include 不再传 `launch_rviz`；并把原先靠同一泄漏才存在的
`safety_limits` / `safety_pos_margin` / `safety_k_position` 显式声明。

### 2.3 RViz 面板报错（红色面板）

vendor `moveit.rviz` 引用了 8 个 `elite_*` 面板：

- `elite_dashboard_rviz_plugin/*`（7 个）：任何一个已审计版本里**都没有源码**；
- `elite_io_rviz_plugin/IOControlPanel`：师兄新版仓库里有源码，我编进了 underlay，但发现它的
  `package.xml` **缺少 `<build_type>ament_cmake</build_type>`**，colcon 按普通 CMake 包处理，
  生成的 `package.dsv` 没有 `local_setup.*` 条目，包**永远不进 `AMENT_PREFIX_PATH`**，
  `ros2 pkg prefix` 找不到，RViz 也就发现不了它。

处置：应用层新增 `cs625_bringup/config/cs625_moveit.rviz`（从 vendor 派生，剔除全部 `elite_*`
面板，保留 MoveIt MotionPlanning / Grid / TF / Marker / PoseArray / RobotModel），两个 MoveIt
入口通过 `moveit_rviz_config` 指向它。**未修改任何 vendor 文件。**

验证：`rviz2 -d <应用配置>` 启动后 `PluginlibFactory` 报错 0 条、`failed to load` 0 条。

## 三、真机 / 仿真双底座

按约定：只启动底座，主动感知另用 launch 叠加。

| 入口 | 启动内容 |
| --- | --- |
| `sim_base.launch.py` | Gazebo + `gz_ros2_control` + 应用描述 + MoveIt |
| `real_base.launch.py` | Elite driver + `ros2_control` + 应用描述 + MoveIt |

对齐方式：两边都复用 `elite_cs625_moveit_config` 的 SRDF / kinematics / joint limits / 规划管线，
都用应用 Xacro 构建规划模型，规划帧与 octomap 帧一致，**唯一差别是时钟**（仿真 `/clock`，
真机 wall time）。新增契约断言防止两者漂移。

### 3.1 underlay 补齐（工作区之外）

- 官方 Elite CS SDK：`Elite_Robots_CS_SDK` tag `v1.2.0` = `092f1595`（MIT），
  普通 CMake 构建后装到 `~/elite-sdk-1.2.0`；
- 师兄最新仓库 `elite_robot_project_20260305`（`main` = `5c003831`）：
  **只复制** `eli_cs_robot_driver`、`eli_cs_controllers` 两个包进
  `~/cs625_underlay_jazzy/src`，未覆盖任何已有包；
- 发现 `eli_common_interface` / `eli_dashboard_interface` 在 src 里但从未编译安装，
  用 `colcon build --packages-up-to` 一并补齐；
- 驱动以**裸库名**链接 SDK（`target_link_libraries(... elite-cs-series-sdk)`，而非其 imported
  target），因此 include / link / 运行期都需要 SDK 前缀。`scripts/source_dev_env.sh` 在检测到
  该前缀存在时导出 `CPLUS_INCLUDE_PATH` / `LIBRARY_PATH` / `LD_LIBRARY_PATH`。

### 3.2 过程中修掉的两个缺陷

1. `real_moveit` 的 `_resolve_share_file` 先拼 `"urdf/"` 再判断绝对路径，导致 `real_base` 传绝对
   路径时被当成相对名，MoveIt 加载到拼接错的描述，SRDF 夹爪 `disable_collisions` 全部失效
   （3 条 `gripper_* is not known to URDF`）。修复后为 0 条。
2. `real_base` 未声明 `initial_positions_file`，include 时报
   `launch configuration 'initial_positions_file' does not exist`。

## 四、验证记录

| 检查 | 结果 |
| --- | --- |
| `python3 test/contract_checks.py` | PASS |
| `git diff --check` | 通过 |
| `source_dev_env.sh --verify` | 四段验证全通过，打印 `elite sdk` 前缀 |
| 真机底座 fake-hardware 冒烟 | driver + RSP + MoveIt 组装成功，`move_group` 到达 `You can start planning now!`，0 warning |
| RViz 应用配置 | `PluginlibFactory` 报错 0 |
| 仿真 `gazebo_visuals:=true`（用户本机执行） | 保留 8 个网格；控制器全部 active；`/camera/image`、`/camera/depth_image`、`/camera/camera_info`、`/camera/points` 全部 advertised；`/sensors/camera/points` ≈ 1.8–2.4 Hz |

命令（关键几条）：

```bash
source scripts/source_dev_env.sh --verify
python3 test/contract_checks.py
ros2 launch cs625_bringup sim_base.launch.py headless:=true launch_rviz:=true
ros2 launch cs625_bringup real_base.launch.py launch_driver:=true \
  use_fake_hardware:=true robot_ip:=127.0.0.1 launch_moveit:=true launch_rviz:=false
```

## 五、能力层与限制

```text
Capability layer
- environment and dependencies   : Jazzy + underlay + overlay 通过；Elite SDK 1.2.0 已装
- robot model and simulation     : Gazebo 实体可保留网格；默认仍无网格
- kinematics, control, planning  : 仿真控制器 active；真机 MoveIt 底座组装通过（fake hardware）
- vision / hand-eye / TF         : 仿真相机四话题正常；真机相机未接
- active perception / NBV        : 未在本次范围内改动
- real hardware integration      : 仅软件底座，未连接真实机器人

Critical-chain status
- URDF -> Gazebo entity          : PASS（网格开关两种模式均可）
- ros2_control -> joint_states   : PASS（仿真，用户本机）
- base_link -> camera optical TF : PASS（仿真）
- RGB-D -> normalized topics     : PASS（仿真，约 2 Hz）
- point cloud -> MoveIt scene    : 未在本次重新验收
- NBV decision -> robot execution: NOT STARTED（未在本次范围内）
```

限制与未完成：

- **真机 R1 未验收**：无机器人连接，`real_base` 仅通过 fake-hardware 冒烟；控制器保持
  `activate_joint_controller:=false`、`execute:=false`。
- Agent 沙箱禁止写 `/dev/shm` 与 `~/.ros`，导致沙箱内 `gz_ros2_control` 收不到
  `robot_description`、控制器无法激活；用户本机无此问题（已用 `list_controllers` /
  `joint_states` / TF 证伪）。
- 保留网格后点云仅约 2 Hz（配置 10 Hz），软件渲染所致；录证据仍用默认无网格模型。
- `elite_io_rviz_plugin` 已编入 underlay 但不可被发现（缺 `build_type`），未被任何配置引用；
  若要启用需 fork 并补 `<build_type>ament_cmake</build_type>`（按 vendor 修改策略登记）。
- `move_group` 退出期在 `~rclcpp::Executor()` 处 segfault，属既有问题，与本次改动无关。
- 本次**未推送**远端。

## 六、变更文件

```text
新增  .repos/real.repos
新增  src/cs625_bringup/config/cs625_moveit.rviz
新增  src/cs625_bringup/launch/real_moveit.launch.py
修改  README.md
修改  docs/dependencies.md
修改  docs/real_hardware_readiness.md
修改  docs/simulation.md
修改  scripts/source_dev_env.sh
修改  src/cs625_bringup/launch/real_base.launch.py
修改  src/cs625_bringup/launch/sim_base.launch.py
修改  src/cs625_bringup/launch/sim_control.launch.py
修改  src/cs625_bringup/launch/sim_moveit.launch.py
修改  src/cs625_bringup/launch/sim_view_planning.launch.py
修改  src/cs625_bringup/scripts/prepare_gazebo_model.py
修改  test/contract_checks.py
本地  .vscode/tasks.json（仓库 .gitignore 忽略 .vscode/，未提交）
```

仓库外变更（工作区之外，未纳入版本控制）：

```text
~/elite-sdk-1.2.0/                                Elite CS SDK 1.2.0 安装前缀
~/cs625_underlay_jazzy/src/eli_cs_robot_driver    更新为 20260305 版本
~/cs625_underlay_jazzy/src/eli_cs_controllers     新增
~/cs625_underlay_jazzy/src/elite_io_rviz_plugin   新增（不可被发现，未引用）
~/cs625_underlay_jazzy/src/eli_common_interface       重新编译安装
~/cs625_underlay_jazzy/src/eli_dashboard_interface    重新编译安装
```

## 七、使用入口

VS Code：`Ctrl+Shift+P` → `Tasks: Run Task`；`Ctrl+Shift+B` 为默认构建任务。
真机控制器 IP 保存在仓库之外的 `~/.cs625_local.env`（`CS625_ROBOT_IP=...`），
任务 7 读取该文件，缺失则立即报错退出。
