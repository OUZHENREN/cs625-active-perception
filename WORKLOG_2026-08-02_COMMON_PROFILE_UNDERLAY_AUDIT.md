# 工作日志：common profile 与 underlay/NBV 复用审计

日期：2026-08-02

## 范围与边界

本轮继续执行 Phase 0–1，只修改现有五个包内的 description、bringup、sensor
profile、静态契约和文档。没有创建 P2–P6 包，没有搬运整个 NBV 工作区，也没有
修改师兄/厂商 underlay。

## 已完成的复用改进

- 新增 `cs625_bringup/launch/sensor_adapter.launch.py`，作为 sim/real 共用的
  normalized RGB-D 入口；两个 profile 不再各自创建适配器节点。
- `common.yaml` 只保存统一输出 topic、freshness 和 invalid-header 策略；
  `sim.yaml`、`real.yaml` 只区分仿真时钟和系统时钟。
- `real_base.launch.py` 直接组合既有
  `eli_cs_robot_driver/elite_control.launch.py`，不复制驱动实现。`robot_ip` 默认为空，
  `launch_driver=false`、`activate_joint_controller=false`、`execute=false`、
  `require_confirmation=true`。
- real profile 保留 `eli_cs_robot_description` 作为厂商 YAML 的所有者，同时把安装后的
  application wrapper 绝对路径作为 `description_file` 传入，避免复制关节、运动学、
  物理和视觉参数文件。
- application xacro 同时接受师兄仿真入口使用的 `prefix` 和真实驱动使用的
  `tf_prefix`，维持一份机器人应用描述。

## 本地资产审计结果

- 干净参考工作树：`experiment/real-nbv`，commit
  `ae7327e836419a617e9d446ee35c37b6680c1dc0`。
- `Ubuntu_Share/elite_ros_ws` 当前 commit
  `7f0c39bc3c159c22d9e50f39b9b364657df7be4f`，但含未提交 NBV/仿真修改，不能作为
  pinned underlay 整体导入。
- `vision_bridge` 是 TCP 目标位姿接收器，不是 PS800E1/Percipio RGB-D 驱动。
- 已登记可复用 API：`ViewpointSampler::generate_candidates`、
  `InformationGain`、`BaselineStrategies::select_next`、`IkSolver::solveIKAll`、
  `ExperimentLogger`；`NbvOrchestrator` 和 `nbv_pipeline.launch.py` 不整块迁移。
- 师兄 MoveIt `move_group.launch.py` 会从自身配置包重建 `robot_description`；它是否
  与 Gazebo/RSP 的 Eye-in-Hand wrapper 完全一致，仍是 S1 runtime gate。

## VM 验证与失败记录

用户在 Ubuntu 22.04 / ROS 2 Humble VM 运行：

```bash
bash scripts/doctor_humble.sh
bash scripts/build.sh
bash scripts/test.sh --event-handlers console_direct+
python3 test/contract_checks.py
```

结果：

- doctor：PASS，Ubuntu 22.04、Humble、ros2、colcon、rosdep、vcs、xacro 均存在；
- build：FAIL，colcon 不接受放在 `build` 子命令之后的 `--log-base`；
- test：FAIL，同一参数顺序错误；
- static contract：PASS。

根据 VM 的实际 CLI 证据，脚本已改为：

```text
colcon --log-base <local-log-dir> build ...
colcon --log-base <local-log-dir> test ...
```

并新增静态回归检查，强制全局 `--log-base` 位于子命令之前。

## VM 修复后复验结果

用户在同一 Ubuntu 22.04 / ROS 2 Humble VM 重新执行 build、test 和静态契约：

- build：PASS，五个 Phase 0–1 包全部完成，0 个失败；
- test：PASS，`cs625_sensor_adapter/tests/test_import.py` 共 1 项通过，0 个错误、
  0 个失败、0 个跳过；
- static contract：PASS。

因此当前可以确认源码构建、现有单元测试和静态架构契约通过。尚未确认 underlay 包发现、
Xacro 展开、TF 树、Gazebo/MoveIt 组合或真实硬件连接；这些仍属于后续 runtime gate。

## Underlay 发现结果

用户在只 source `/opt/ros/humble` 和当前应用 overlay 的环境中检查四个复用包，结果全部
`MISSING`。这只能说明当前应用环境没有加载 underlay，不代表共享目录中没有可复用构建。

宿主机审计发现 `Ubuntu_Share/elite_ros_ws/install` 已包含：

- `eli_cs_robot_description`；
- `eli_cs_robot_driver`；
- `eli_cs_robot_simulation_gz`；
- `elite_cs625_moveit_config`。

该 workspace 当前含未提交修改，因此只作为临时运行时验证来源，不能作为正式 pinned
依赖，也不能将其源码整体复制进本仓库。下一步在 VM 中仅 source 它的 install，再做
包发现、Xacro 展开和 launch 参数检查。

## 关于主动视觉库与既有 NBV 库的实际复用状态

本轮当前代码尚未源码级迁移 AIRLab-POLIMI active-vision 或既有 `cs625_nbv`。已完成
的是审计、API 阅读和职责映射，以及师兄 CS625 description/driver 的 underlay 组合；
不能把这些审计结果表述成算法已经接入。

原因是说明书和 ADR-002 明确要求 Phase 0–1 只保留五个基础包，禁止在官方 CS625/Humble
底座和 sim/real 契约尚未运行验收前迁移 NBV、感知、运动和编排代码。这是阶段门禁，不是
放弃复用。底座 runtime gate 通过后，实际迁移顺序为：

1. active-vision 的接口分层、点云和规划边界分别借鉴到后续 interfaces/scene-mapping/
   motion 边界；不导入其机器人专用实现；
2. 既有 `cs625_nbv/ViewpointSampler` 按职责迁入 `cs625_view_generation`；
3. `InformationGain`、`BaselineStrategies` 迁入 `cs625_view_evaluation`，统一接收已
   完成硬可达性过滤的候选集；
4. `NbvOrchestrator` 不整块复制，拆分为主动定位状态机；`ExperimentLogger` 只复用日志
   schema 和 provenance 字段。

因此当前状态应准确描述为“底座复用已开始，主动视觉/NBV 算法复用待 Phase 1 runtime
gate 后按职责迁移”，而不是“主动视觉/NBV 已经使用”。

## VM underlay 挂载复验

用户尝试 source `/mnt/hgfs/elite_ros_ws/install/setup.bash`，VM 返回
`No such file or directory`，随后四个 underlay 包仍全部 `MISSING`。这证明当前
VMware HGFS 挂载中没有宿主机的 `Ubuntu_Share/elite_ros_ws` 目录；不能据此判断该
underlay 的源码或 install 不存在。需要先把该目录作为第二个共享文件夹暴露给 VM，或
在 VM 的独立 underlay workspace 中重新构建；两种方案都不应把 underlay 源码复制到
当前 `elite_ros` Git 根目录。

## 运行边界

## Underlay 已挂载但 Jazzy 污染

VM 现在可以访问 `/mnt/hgfs/elite_ros_ws/install/setup.bash`，四个 underlay 包均能被
`ros2 pkg prefix` 找到；但 source 时出现：

```text
not found: "/opt/ros/jazzy/local_setup.bash"
```

这说明该 install 是在 Jazzy 环境生成的，不能作为 Humble underlay 使用。下一步必须
在 Ubuntu 22.04 / Humble 中从独立 underlay 源码重新构建，并把 build/install/log 放到
VM 本地 ext4 目录；当前 Git 仓库仍只保留应用源码，不能 source 这个 Jazzy install
来冒充 Humble 验证。

本轮没有启动 fake hardware、Gazebo 或真实硬件，没有执行 MoveIt 轨迹，也没有发生
真实机械臂运动。PS800E1 驱动、topic map、内参和手眼标定仍未得到可验证来源。

## 下一步 runtime gate

先检查师兄/厂商 underlay 包是否可发现，再展开 application xacro 和核对 launch 参数；
在这些只读检查通过之前，不启动 Gazebo、MoveIt、控制器或真实驱动。

## 干净参考树与第一批 NBV API 审计

宿主机的干净参考树 `.real_nbv_experiment_20260729` 当前为
`experiment/real-nbv`，commit `ae7327e836419a617e9d446ee35c37b6680c1dc0`，工作区
干净。它包含 `cs625_nbv`、`cs625_kinematics`、CS625 description/driver/simulation
和 MoveIt 配置，可作为独立 underlay/迁移来源；不应直接当作当前 Git 仓库的源码。

实际 API 审计确认：

- `ViewpointSampler::generate_candidates` 可保留 Fibonacci 半球采样和 look-at 数学，
  但必须把 `base_link`、相机 FOV、距离和半球轴改成 profile/接口参数，并处理边界样本数；
- `InformationGain` 可保留协方差、可见性、新颖度和 utility 计算，但当前直接依赖旧的
  `cs625_nbv/msg/ViewpointCandidate`，迁移时必须改为新候选/评分接口；
- 旧的 `score_candidates` 已假设候选先完成 `reachable` 过滤，这与课题规定的“先硬约束、
  后策略评分”一致，应作为接口不变量保留；
- 这些算法尚未复制或接入当前仓库，待 underlay runtime gate 通过后按职责进入后续
  `cs625_view_generation` 和 `cs625_view_evaluation` 包。

## Launch 参数门禁复验

用户在 Humble 中重建 `cs625_bringup` 后，以下两个只读参数检查均通过：

```text
ros2 launch cs625_bringup sim_base.launch.py --show-args
ros2 launch cs625_bringup real_base.launch.py --show-args
```

复验确认 sim/real 参数契约可被 Humble launch 加载；sim 与 real 均保持
`execute=false`、`require_confirmation=true`，real profile 的 `robot_ip` 默认为空、
`activate_joint_controller=false`。本机静态契约和 Python 语法检查也通过。此步骤没有
启动 Gazebo、控制器、MoveIt 或真实机械臂。

## Xacro 与 Humble launch 验证

用户在 Humble 下重新展开 application xacro：PASS，关键 frame 检查通过，生成 link 数为
17，包含 `world`、`base_link`、`tool0`、`camera_link` 和
`camera_depth_optical_frame`。

随后执行两个 launch 的 `--show-args` 时发现 Humble 的 `launch.actions` 不提供
`LogWarn`，导致 `sim_base.launch.py` 与 `real_base.launch.py` 在导入阶段失败。已将
这些非关键提示改为 Humble 支持的 `LogInfo`，并在消息文本中保留 `[WARN]` 标记；没有
改变启动条件、安全默认值或 underlay 组合逻辑。宿主机静态契约修复后：PASS。

## Xacro 运行时接口修复

用户在 Humble 下用已构建的 description underlay 展开 application xacro 时，师兄宏报错：

```text
Invalid parameter "simulation_controllers"
when instantiating macro: cs_robot
```

审计 `eli_cs_robot_description/urdf/cs_macro.xacro` 确认该宏支持
`sim_gazebo`、`sim_ignition` 和 `headless_mode`，但不接受
`simulation_controllers`。因此只从当前 application wrapper 的 `cs_robot` 调用中移除
这一无效传参；没有复制或修改师兄宏。顶层 `simulation_controllers` 参数仍保留，供后续
经过验证的仿真入口使用。宿主机静态契约检查修复后：PASS。

## Humble underlay 构建结果与 SDK 门禁

用户在 Humble 中从独立 artifact 根目录构建已有 underlay 源码：

- `eli_cs_robot_description`：PASS；
- `eli_common_interface`：PASS；
- `eli_dashboard_interface`：PASS；
- `eli_cs_robot_simulation_gz`：PASS；
- `elite_cs625_moveit_config`：PASS；
- `eli_cs_robot_driver`：FAIL，缺少
  `/opt/elite-sdk-custom/include/Elite/EliteDriver.hpp`；
- `eli_cs_controllers`：因 driver 失败而 ABORT。

因此 Humble underlay 的 description、仿真配置和 MoveIt 配置已经产生，但真实 driver
尚未通过。官方 Elite SDK 仓库确认支持 Ubuntu 22.04，并提供 PPA/源码安装路径；当前
师兄 driver CMake 固定使用 `/opt/elite-sdk-custom`，不能用伪造头文件、stub 库或修改
当前应用仓库来绕过该门禁。下一步应安装/构建官方 SDK 到该独立 underlay 前缀，再只
重建 driver 及其依赖。

## 师兄模型、相机模型与话题链部署

用户在 Humble VM 中验证 fixture world 启动成功：Gazebo 进程能够拉起，且安全的
`launch_official_sim:=false`、`launch_sensor_adapter:=false`、`camera_enabled:=false`
组合不会启动控制器或适配器。

本次按“复用 underlay + 应用包装”完成部署补强：

- `cs625_ap_description` 继续复用 `eli_cs_robot_description/urdf/cs_macro.xacro`，不复制
  师兄机械臂本体；仅保留 `my_end_effector_link`、稳定的 `tool0` 和 eye-in-hand 相机链；
- 相机模型补齐 `camera_color_optical_frame` 与 `camera_depth_optical_frame`，并沿用师兄
  `rgbd_camera` 传感器形态；Gazebo 源前缀固定兼容为 `/camera/*`；
- 应用 Xacro 顶层补回师兄 `libgz_ros2_control-system.so` 控制插件模式，并把
  `simulation_controllers` 放在插件层，而不是错误地传给不接受它的 `cs_robot` 宏；
- `cs625_bringup/config/sim_gz_bridge.yaml` 与 `sim_sensor_bridge.launch.py` 复用
  `ros_gz_bridge`，将 `/camera/image`、`/camera/depth_image`、`/camera/camera_info`、
  `/camera/points` 映射为 `/sim/camera/*`；
- 现有 `cs625_sensor_adapter` 继续负责唯一的统一边界，发布 `/sensors/camera/*`；
  `sensor_adapter.launch.py` 已修复为空命令行参数覆盖 profile YAML 的问题；
- real profile 的源话题保持空值，直到 PS800E1/Percipio 驱动在 Humble 下实际验证，未
  构造品牌驱动、stub 或虚假话题。

宿主机 `python3 test/contract_checks.py`：PASS；新增桥接、双 optical frame、控制插件
和 profile 话题映射均纳入静态契约。尚未宣称官方 MoveIt/Gazebo 控制链已在 VM 中通过；
下一步必须在 VM 重建五包并运行官方仿真 smoke test，检查 `/camera/*`、`/sim/camera/*`、
`/sensors/camera/*`、`/joint_states` 和 TF。

## 官方仿真首次运行门禁

用户在 Humble VM 执行官方仿真命令：

```text
timeout 60s ros2 launch cs625_bringup sim_base.launch.py \
  launch_fixture_world:=false launch_official_sim:=true \
  launch_sensor_adapter:=true camera_enabled:=true \
  launch_rviz:=false headless:=true
```

启动在解析师兄 `cs_sim_control.launch.py` 的控制器 spawner 时停止，明确错误为：

```text
package 'controller_manager' not found
```

随后执行 `ros2 control list_controllers` 也显示 `ros2 control` 子命令不存在；这是同一
个 Humble ros2_control 运行依赖缺失的结果。由于启动进程已经退出，`/joint_states`、TF
和 `/sensors/camera/status` 不存在并不能判定模型或相机失败。已记录为 VM 依赖门禁，要求
安装 `controller_manager`、`ros2_control`、`ros2_controllers`、`ros2controlcli` 和
`gz_ros2_control` 后重试。

## MoveIt 模型一致性修正

审计师兄 `elite_cs625_moveit_config/.setup_assistant` 发现其原始 MoveIt 入口会加载
`eli_cs_robot_description/urdf/cs625_for_moveit.urdf`，不会包含当前应用新增的相机安装座
和 optical frames。为避免控制模型和规划模型分裂，应用层新增了薄的
`cs625_bringup/launch/sim_moveit.launch.py`：复用师兄 SRDF、kinematics、joint limits、
trajectory controller 和 MoveIt 配置 builder，只将 `robot_description` 指向当前应用
Xacro，并让 MoveIt 点云更新器订阅统一 `/sensors/camera/points`。未修改或复制师兄
MoveIt/vendor 包；`sim_base.launch.py` 默认改为“师兄控制 launch + 应用 MoveIt launch”。

宿主机 Python 语法、静态契约和 `git diff --check` 均 PASS。上述应用 MoveIt 组合尚未在
VM 运行时验证，必须在补齐 ros2_control 依赖后重建并复验。

## 2026-08-03 官方仿真运行证据

用户补齐 ros2_control 后再次运行官方仿真。此次日志证明以下链路已经进入运行态：

- `robot_state_publisher` 成功启动并列出 `base_link`、`tool0`、`camera_mount_link`、
  `camera_link`、`camera_color_optical_frame`、`camera_depth_optical_frame`；
- `ros_gz_bridge` 成功创建 `/clock` 和四路相机桥接：`/camera/image`、
  `/camera/depth_image`、`/camera/camera_info`、`/camera/points` → `/sim/camera/*`；
- `ros_gz_sim create` 最终成功创建 `cs` 实体；
- MoveIt 成功加载 OMPL、CHOMP、Pilz 管线并进入 `You can start planning now!`。

同时捕获到三个真实缺陷：

1. MoveIt 报 `Semantic description is not specified for the same robot as the URDF`，原因是
   应用 URDF 根名为 `cs625_active_perception`，复用 SRDF 根名为 `cs625`；已将应用根名
   对齐为 `cs625`；
2. `cs625_sensor_adapter` 因覆盖 `rclpy.node.Node._publishers` 内部列表为字典而崩溃，
   已改为 `_stream_publishers`；
3. MoveIt 点云更新器报告 `occupancy_map_monitor/PointCloudOctomapUpdater` 未在当前
   plugin descriptions 中注册。当前配置类型与 Humble MoveIt 约定一致，需确认/安装
   `moveit_ros_occupancy_map_monitor` 二进制依赖后再判断是否还有配置问题。

控制器 spawner 仍等待 `/controller_manager/list_controllers`，本次附件没有给出
`gz_ros2_control` 的明确加载错误；下一次必须单独确认 `ros2 pkg prefix gz_ros2_control`
和 Gazebo 日志。模型、桥接和 MoveIt 已不再是“完全未启动”，但 ros2_control controller
active、适配器存活和统一输出话题仍未验收。

## 2026-08-03 00:02 运行门复盘与修正

用户在 Humble VM 的官方仿真命令中进一步验证了模型部署链：

- `robot_state_publisher` 已加载师兄 CS625 机械臂本体以及应用 eye-in-hand 相机链，日志列出
  `base_link`、`tool0`、`camera_mount_link`、`camera_link`、
  `camera_color_optical_frame`、`camera_depth_optical_frame`；
- `ros_gz_bridge` 已成功建立 `/camera/image`、`/camera/depth_image`、
  `/camera/camera_info`、`/camera/points` 到 `/sim/camera/*` 的四路桥接；
- `ros_gz_sim create` 最终成功创建 `cs` 实体，MoveIt 已加载 OMPL、CHOMP、Pilz 管线；
- 当前仍未验收 controller active、统一 `/sensors/camera/*` 输出和相机 TF，因为控制器服务与
  适配器进程分别还有阻塞/崩溃。

针对该次真实日志已完成以下最小修正：

1. 应用 Xacro 根名由 `cs625_active_perception` 对齐为师兄 SRDF 使用的 `cs625`，避免 MoveIt
   的 URDF/SRDF robot name 不一致；
2. `cs625_sensor_adapter` 不再覆盖 `rclpy.node.Node._publishers` 内部列表，改用
   `_stream_publishers` 保存流发布器；
3. 仿真默认控制器改用应用 `config/sim_controllers.yaml` 中的 Humble 标准
   `joint_state_broadcaster` 与 `joint_trajectory_controller`。师兄 `cs_controllers.yaml` 和
   `eli_cs_controllers` 仍保留为已安装 Elite SDK 后的显式 underlay 覆盖，不复制或改写师兄代码；
4. MoveIt 点云更新器仍需要 VM 安装/确认 `moveit_ros_occupancy_map_monitor` 二进制插件，不能
   通过删除点云配置来掩盖依赖缺失。

随后复核桥接 YAML 发现四路 Gazebo 相机桥接均声明 `SENSOR_DATA` QoS；适配器已同步使用
`qos_profile_sensor_data` 订阅和发布 color/depth/camera_info/points，避免“话题存在但 QoS
不兼容导致没有数据”的假通过。状态话题仍使用默认可靠 QoS。

## 2026-08-03 Phase 1 入口一致性修正

复核启动任务书后发现 `sim_active_localization.launch.py` 虽已存在，但默认值仍指向师兄
旧的 `cs_sim_moveit.launch.py` 和 SDK 绑定的 `cs_controllers.yaml`，会绕过当前应用的标准
Humble 仿真控制器配置。已将该薄入口对齐为：复用师兄 `cs_sim_control.launch.py`、使用
`cs625_bringup/config/sim_controllers.yaml`，并增加明确日志说明 `strategy:=disabled` 时不
启动目标感知或 NBV 算法。没有新增算法包，也没有复制或修改 vendor/师兄源码。

本轮修改后必须在 VM 重新 build 五个应用包，并确认 `ros2 pkg prefix controller_manager`、
`ros2 pkg prefix gz_ros2_control`、`ros2 pkg prefix moveit_ros_occupancy_map_monitor`；随后
重新运行官方仿真并检查 `ros2 control list_controllers`、`/joint_states`、
`/sensors/camera/status`、`/sensors/camera/points` 以及
`base_link -> camera_depth_optical_frame`。在这些运行时证据出现前，S1 运行门不标记为通过。

## 2026-08-03 Gazebo 实体创建根因与应用层修正

完整启动日志证明机械臂 Xacro 已正确生成，包含 `gz_ros2_control/GazeboSimSystem`、
`libgz_ros2_control-system.so`、应用 `sim_controllers.yaml` 和 eye-in-hand 相机链；真正阻塞点
是 `ros_gz_sim create` 在 Gazebo RGB-D 渲染系统完成初始化前，对
`/world/minimal_occlusion/create` 的请求超时。机械臂实体未生成，因此控制插件从未加载，
`/controller_manager` 不可能出现。

没有修改师兄 underlay。`sim_base.launch.py` 继续复用师兄 `cs_sim_control.launch.py`，仅增加
一次延迟 10 秒的应用层 spawn 重试。重试重新展开同一应用 Xacro、使用同一控制器 YAML，实体
名固定为 `cs` 且 `allow_renaming=false`；若师兄原始 spawn 已成功，重试不会生成重复机械臂。
该修正尚待 Humble VM 重建并验证 controller active、`/joint_states` 和相机统一话题。

## 2026-08-03 MoveIt 点云插件依赖校正

进一步核对 Humble MoveIt 包职责后确认：`moveit_ros_occupancy_map_monitor` 提供基础监视器，
配置中使用的 `occupancy_map_monitor/PointCloudOctomapUpdater` 动态插件由上游
`moveit_ros_perception` 提供。应用没有自行实现点云到 OctoMap 的替代代码，只在
`cs625_bringup/package.xml` 增加该上游运行依赖，并在静态契约中固定这项复用要求。

VM 下一轮需安装/确认 `ros-humble-moveit-ros-perception`，随后与延迟实体生成重试一起复测。
S1 仍以 controller active、`/joint_states`、统一相机输出和 camera TF 的运行证据为准。

## 2026-08-03 Humble VM 复测结果

用户已安装 `ros-humble-moveit-ros-perception`，五个应用包重新构建成功，
`Phase 0–1 static contract checks: PASS`，并确认 `moveit_ros_perception` 位于
`/opt/ros/humble`。

本次运行证据：

- `robot_state_publisher` 成功读取 `world`、`base_link`、`tool0`、
  `camera_mount_link`、`camera_link`、`camera_color_optical_frame` 和
  `camera_depth_optical_frame`；
- MoveIt 成功加载上游 `pointcloud_octomap_updater`，并监听
  `/sensors/camera/points`；
- `/sensors/camera/status` 收到深度数据，`received_count=509`；
- `/sensors/camera/points` 已有稳定输出，观测频率约 2–7 Hz；
- 延迟重试进程已执行实体创建，但请求仍出现超时；之后日志出现 `OK creation of entity`；
- `gz_ros2_control` 没有产生可响应的 `/controller_manager/list_controllers`，两个
  spawner 持续等待，`/joint_states` 因此尚未验收；
- 当前一次 `tf2_echo` 未建立 `base_link` 到 `camera_depth_optical_frame` 的连接，需在
  仿真进程保持运行时检查 `/tf`、`/tf_static` 和节点发现状态。

本轮不修改 URDF、相机话题或 MoveIt 配置；这些部分已有直接运行证据。下一步只针对
Gazebo 服务端插件日志、controller manager 和 TF 发布做诊断，仍不迁移 NBV/主动视觉算法。

## 2026-08-03 Message Filter 根因收敛与控制插件修正

用户再次观察到 MoveIt 对 `camera_depth_optical_frame` 的点云执行 Message Filter 丢帧。
复核后确认日志中目标帧引号前的空格属于 MoveIt 固有日志格式，并非配置中的尾随空格。
实际因果链为：`gz_ros2_control` 未创建 controller manager，导致没有 `/joint_states`，
六个转动关节 TF 不发布，因而相机末端子树无法连接到 `base_link/world`，最后表现为点云
消息队列持续塞满。

本轮针对该因果链做最小修正：保留师兄 CS625 的顶层 `gz_ros2_control` 架构和控制器 YAML，
在应用 Xacro 中显式指定 `robot_param=robot_description`、
`robot_param_node=robot_state_publisher`，并使用 Humble 的
`controller_manager_name` 元素；MoveIt 侧补回师兄配置中的
`octomap_frame=base_link` 与 `octomap_resolution=0.02`，点云来源仍为统一的
`/sensors/camera/points`。未新增控制器、点云算法或 NBV 代码。

用户重建后实体再次创建成功，但仍没有任何 `gz_ros2_control`/controller manager 初始化
日志，Message Filter 丢帧继续出现。这证明前述 Xacro 参数显式化仍不足：Gazebo 服务端没有
加载模型控制插件。针对 Humble/Fortress 的环境差异，`sim_base.launch.py` 现在通过 ament
索引解析已安装 `gz_ros2_control` 的真实前缀，启动前验证
`libgz_ros2_control-system.so` 存在，并将其目录同时前置到
`IGN_GAZEBO_SYSTEM_PLUGIN_PATH` 与 `GZ_SIM_SYSTEM_PLUGIN_PATH`。这样既保留师兄控制架构，
也避免 Fortress 使用旧变量时静默找不到插件；若二进制本身缺失，launch 会立即给出明确错误。

用户确认 `/opt/ros/humble/lib/libgz_ros2_control-system.so` 存在，且
`gz_hardware_plugins.xml` 已注册 `gz_ros2_control/GazeboSimSystem`，因此插件包和二进制
缺失已排除。随后附件中的生成 URDF 已包含上一轮 `robot_param`/
`controller_manager_name` 修正，但过滤后的启动日志没有出现最新 `sim_base.launch.py`
必定打印的 `gz_ros2_control_plugin=/opt/ros/humble/lib/libgz_ros2_control-system.so` 标记。
这表明该次运行尚未使用最新安装的 `cs625_bringup` 启动文件；插件搜索路径修正实际上还未
进入运行态。下一步先比较源码和 install-space 启动文件，再定向重建 bringup，避免继续用
旧安装副本重复验证。
### 2026-08-03：bringup 源码—安装空间一致性已确认

- `cs625_bringup` 定向重建成功。
- `ros2 pkg prefix cs625_bringup` 返回 `${HOME}/cs625_colcon/install/cs625_bringup`。
- 源码与安装空间的 `sim_base.launch.py` 均在第 264 行包含 `gz_ros2_control_plugin` 启动标记。
- 安装空间 launch 文件经 `readlink -f` 解析到仓库源码，排除旧安装副本或错误 overlay。
- 下一门禁：运行时启动头部必须打印 `/opt/ros/humble/lib/libgz_ros2_control-system.so`，随后验证 `/controller_manager/list_controllers` 是否可用。
### 2026-08-03：定位 Humble/Fortress 模型插件声明错配

- 最新双终端运行证明插件库路径已注入，但 Gazebo 在机器人实体创建成功后没有任何 `gz_ros2_control` 初始化输出，`controller_manager` 服务端未创建。
- `ros2 service list` 中出现 `/controller_manager/list_controllers` 不能证明服务端存在；等待中的 spawner 客户端也会让该名称进入 ROS 图。
- 对照 Humble 官方 `gz_ros2_control` 文档，模型插件应声明为 `filename="gz_ros2_control-system"`；原包装层使用了磁盘 ELF 名 `libgz_ros2_control-system.so`。
- 已仅在应用包装层改为 Humble/Fortress 的逻辑插件名，保留硬件插件 `gz_ros2_control/GazeboSimSystem` 和插件类 `gz_ros2_control::GazeboSimROS2ControlPlugin`，未修改师兄库。
- 契约检查已同步更新；下一步需定向重建 `cs625_ap_description` 后复测插件初始化和 controller manager。
### 2026-08-03：更正——插件逻辑文件名不是唯一根因

- 将模型插件名改为 Humble 文档中的 `gz_ros2_control-system` 后，运行时仍未出现 `/controller_manager` 节点和 `/joint_states`。
- 因此上一条“定位错配根因”的表述过早；该修改保留为 Humble 规范对齐，但已被运行结果证伪为完整修复。
- 当前已证实的故障边界是：机器人实体创建成功，但 Gazebo 模型插件没有完成实例化；spawner 等待、TF 断链、点云队列满均为后续症状。
- MoveIt 与 sensor adapter 在 `Ctrl+C` 后的异常退出属于清理路径问题，不作为 controller manager 启动失败的根因。
- 下一步只运行官方 `gz_ros2_control_demos` 最小示例，隔离 VM 插件安装/ABI 与本项目 URDF→SDF/spawn 链。
### 2026-08-03：官方 gz_ros2_control demo 通过

- 安装并运行 Humble 官方 `gz_ros2_control_demos/cart_example_position.launch.py` 成功。
- `/controller_manager` 节点及完整服务端存在；`joint_state_broadcaster`、`joint_trajectory_controller` 均为 `active`。
- `/joint_states` 正常发布，硬件 `GazeboSimSystem` 完成 initialize/configure/activate。
- 由此排除 VM、Gazebo Fortress、`gz_ros2_control` 二进制及 ABI 故障，问题限定在 CS625 的 URDF→SDF/实体创建组合链。
- 官方 demo 通过 `robot_description` topic 创建实体；CS625 复用的 senior launch 使用 `-string`，但在修改前先验证转换后 SDF 是否保留模型插件。
### 2026-08-03：改为单次 topic spawn 控制编排

- 本机官方 demo、CS625 展开 URDF 与 Fortress 转换后 SDF 的插件声明完全一致，确认插件未在转换中丢失。
- 故障差异限定为启动链：senior launch 的 `create -string` 与应用层定时 retry 并存，存在初始请求超时、重复实体和物理场景闪烁风险。
- 新增 `cs625_bringup/launch/sim_control.launch.py`：继续复用 senior CS625 xacro、网格与参数，仅按 Humble 官方 demo 的时序从 `robot_description` topic 单次创建实体。
- `allow_renaming=false`；实体创建退出后依次启动 joint-state broadcaster 与 trajectory controller；删除应用层 TimerAction retry。
- `sim_base` 与 `sim_active_localization` 默认指向该本地编排；师兄仓库未修改，模型和 MoveIt 配置仍由 underlay 提供。
- Python launch 语法检查通过；Phase 0–1 静态契约检查通过。运行门禁仍需 VM 定向重建后验证。
### 2026-08-03：终端 1 证实 Gazebo 卡在 RGB-D 渲染初始化

- 新 `sim_control.launch.py` 已生效：使用 `robot_description` topic，且仅有一个 `cs625_spawn_robot`。
- Gazebo 停在 `SensorsPrivate::Run`、`Initializing render context`、`Loading ignition-rendering-ogre2`，直到 `Ctrl+C` 后才完成渲染线程初始化并发布固定相机话题。
- 实体创建因此在 5 秒处超时；`gz_ros2_control` 模型插件尚无执行机会，controller manager 缺失是后果。
- 纯控制门禁不再使用含固定 RGB-D 相机的 `minimal_occlusion.sdf`，改用 `sim_control.launch.py` 的无相机 `empty.sdf` 默认值。
- 更正 `ros_gz_sim create` 参数：`-allow_renaming` 是布尔开关，传入字符串 `false` 仍会启用；现已省略该开关，使用默认禁止重命名行为。
### 2026-08-03：空世界控制链门禁通过

- `cs625_spawn_robot` 从 `robot_description` topic 一次创建实体成功，无超时和重复实体。
- `GazeboSimROS2ControlPlugin` 成功读取 URDF，六个 CS625 关节全部载入。
- hardware `cs` 完成 initialize/configure/activate，`controller_manager` 正常创建。
- `joint_state_broadcaster` 与 `joint_trajectory_controller` 均完成 configured/activated。
- 这证明此前 controller manager 缺失的直接阻塞是 RGB-D/Ogre2 世界渲染初始化，而非机械臂 ros2_control 模型。
- 发现独立资源警告：末端执行器 `package://` 网格在 SDF 中被转换为无法解析的 `model://` URI；包装层已改为解析后的 `file://$(find eli_cs_robot_description)/...`，师兄资产未复制或修改。
### 2026-08-03：joint state 与眼在手 TF 门禁通过

- `/controller_manager` 节点可见，两个控制器均为 `active`。
- `/joint_states` 发布完整六关节 position/velocity/effort。
- `base_link -> camera_depth_optical_frame` 在启动瞬态后持续输出有效变换；初始一次 frame warning 不构成持续故障。
- 控制与 TF 子门禁通过；相机渲染与 MoveIt 组合仍需单独验证。
- 为隔离相机源，新增 `rgbd_fixture.sdf` 作为固定相机接口测试世界；`minimal_occlusion.sdf` 删除固定相机，只保留机械臂眼在手相机。
- 两个 VM 世界的传感器渲染后端改为 Fortress `ogre`，相机 `visualize=false`，避免已观测的 Ogre2 初始化阻塞和画面闪烁。
### 2026-08-03：更正 headless RGB-D 渲染配置

- 分离固定相机后，带 Sensors system 的 robot world 仍在渲染线程初始化处阻塞，排除双相机和 Ogre2 本身为唯一原因。
- Gazebo Fortress 官方约束：普通 `-s` 只是 server-only；无 GUI 的渲染传感器必须额外使用 `--headless-rendering` 选择 EGL，且 EGL 只支持 Ogre2。
- `sim_control.launch.py` 的 headless 参数现为 `-s -r -v 4 --headless-rendering`；两个 RGB-D 世界恢复 Ogre2。
- 此项仍需 VM 运行验证 EGL 与 VMware `/dev/dri` 能力；若不可用，应得到明确 EGL/device 错误，而不再按插件或 TF 故障处理。
- 修正 sensor adapter 退出清理：仅在 rclpy context 仍有效时调用 shutdown，避免 launch 已关闭 context 后的重复 shutdown 异常。
