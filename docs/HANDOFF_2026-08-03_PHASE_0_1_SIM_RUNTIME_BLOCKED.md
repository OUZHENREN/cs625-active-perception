# CS625 主动感知 ROS 2 项目交接说明

- 日期：2026-08-03（Asia/Shanghai）
- 仓库：`elite_ros`
- 当前阶段：Phase 0–1
- 状态：暂停；仿真控制与 TF 子门禁通过，RGB-D 渲染门禁被 VMware `/dev/dri` 不可用阻塞
- 安全范围：仅 Gazebo 仿真；未连接或驱动真实机械臂；执行开关保持 `false`

## 1. 结论摘要

当前工作不能标记为 Phase 1 完成。

已经确认 CS625 模型、`gz_ros2_control`、控制器、六关节状态和眼在手相机 TF 链在无传感器渲染的空世界中能够正常运行。官方 `gz_ros2_control_demos` 也在同一 Humble VM 中通过，因此 ROS 2 Humble、Gazebo Fortress、`gz_ros2_control` 二进制及其 ABI 不是当前阻塞点。

未完成部分集中在 RGB-D 传感器渲染。VM 当前检查结果为 `/dev/dri` NOK，无法提供 Gazebo Fortress 的 Ogre2/EGL 无头渲染所需的 DRI 设备。带 Sensors system 和 RGB-D 相机的完整世界会停在渲染上下文初始化，导致 Gazebo 服务端主流程不能继续执行模型控制插件。随后出现的 controller manager 缺失、关节 TF 不完整、MoveIt 点云 Message Filter 队列满，都是这条阻塞链的后果。

因此：

- 控制与 TF 子门禁：通过；
- RGB-D 原始话题与统一适配器门禁：未通过；
- MoveIt 点云融合门禁：未通过；
- NBV/主动视觉算法迁移：尚未开始，继续保持禁止迁移；
- 实机路径：未测试，真实驱动仍受 Elite SDK 头文件缺失阻塞。

## 2. 必须继续遵守的边界

项目结构必须保持：

```text
一个 Git 仓库
├── 一套 common 核心源码
├── 一套 sim 配置与启动入口
└── 一套 real 配置与启动入口
```

Phase 0–1 只允许以下五个应用包：

1. `cs625_ap_interfaces`
2. `cs625_ap_description`
3. `cs625_sensor_adapter`
4. `cs625_simulation`
5. `cs625_bringup`

继续工作时必须遵守：

- 优先复用师兄/厂商 underlay 中的 CS625 模型、网格、MoveIt 配置和驱动接口；
- 不在本仓库复制整套机械臂模型或重写现成控制栈；
- 不原地修改师兄/厂商仓库；本仓库只保留薄包装和配置差异；
- sim 与 real 共用接口、描述扩展和传感器适配核心，仅启动入口与 profile 配置分离；
- 在 RGB-D、TF、MoveIt 运行门禁全部通过前，不迁移主动视觉库或 NBV 算法；
- 默认 `execute:=false`，不得因为仿真调试扩大到真实机械臂运动。

## 3. 路径与环境

| 项目 | 当前值 |
|---|---|
| Windows 仓库根目录 | `B:\Recent\Robotic arm\Ubuntu_Share\cs625_active_perception\elite_ros` |
| VM 共享仓库根目录 | `/mnt/hgfs/cs625_active_perception/elite_ros` |
| ROS 发行版 | ROS 2 Humble |
| 操作系统 | Ubuntu 22.04 Jammy |
| Gazebo | Fortress / Gazebo Sim 6 |
| 师兄 Humble underlay | `${HOME}/cs625_underlay_humble/install` |
| 本项目 overlay | `${HOME}/cs625_colcon/install` |
| colcon 工件根目录 | `${HOME}/cs625_colcon` |
| ROS 运行日志 | `${HOME}/.ros/log` |
| Obsidian 同步日志目录 | `B:\Recent\Obsidian\Do be do be do\课题\项目\工作日志` |

实机驱动 `eli_cs_robot_driver` 尚不能在 Humble underlay 构建，已知原因是缺少 `/opt/elite-sdk-custom/include` 下的 Elite SDK 头文件。这个问题与当前 Gazebo RGB-D 渲染阻塞相互独立。

## 4. 当前架构与复用关系

```text
elite_ros（单仓库）
├── common
│   ├── cs625_ap_interfaces
│   ├── cs625_ap_description
│   │   └── 包装 eli_cs_robot_description 的 cs_macro.xacro
│   └── cs625_sensor_adapter
├── sim profile
│   ├── cs625_simulation
│   └── cs625_bringup/launch/sim_*.launch.py
└── real profile
    └── cs625_bringup/launch/real_base.launch.py

Humble underlay（只复用，不原地修改）
├── eli_cs_robot_description
├── eli_cs_robot_simulation_gz
├── elite_cs625_moveit_config
└── eli_cs_robot_driver（当前因 Elite SDK 缺失而不可用）
```

应用层 Xacro 只扩展眼在手相机及仿真控制配置；机械臂本体仍来自 `eli_cs_robot_description`。MoveIt 继续复用 `elite_cs625_moveit_config` 的 SRDF、运动学、关节限制、控制器和规划管线。

## 5. 已实施内容

### 5.1 描述与 TF

- `cs625_active_perception.urdf.xacro` 包装师兄的 `cs_macro.xacro`，没有复制机械臂本体源码。
- 增加眼在手相机链：`tool0` → `camera_mount_link` → `camera_link` → color/depth optical frames。
- 已验证 `base_link -> camera_depth_optical_frame` 在控制链启动后连续可用。
- 修正末端网格 URI 为可解析的 `file://$(find eli_cs_robot_description)/...` 形式，避免转换后出现不可解析的 `model://` URI。

### 5.2 Humble/Fortress 控制组合

- 保留硬件插件：`gz_ros2_control/GazeboSimSystem`。
- 模型插件使用 Humble/Fortress 逻辑名：`gz_ros2_control-system`。
- 已验证 URDF 转换为 SDF 后控制插件声明仍然存在。
- 新增 `cs625_bringup/launch/sim_control.launch.py`：
  - 从 `robot_description` topic 单次创建实体；
  - 不再使用定时重复 spawn；
  - spawn 完成后依次启动 joint-state broadcaster 和 trajectory controller；
  - 默认使用无 Sensors system 的空世界验证纯控制链。
- `sim_base.launch.py` 默认复用该本地薄编排，同时继续使用师兄模型和参数。
- `sim_controllers.yaml` 使用 Humble 标准控制器类型。

### 5.3 相机与话题

- 仿真目标仍为眼在手 RGB-D 相机，不在机械臂世界中保留第二台固定相机。
- `rgbd_fixture.sdf` 作为独立固定相机接口测试世界，不应与机械臂完整世界同时使用。
- 统一输出目标为 `/sensors/camera/*`，由 `cs625_sensor_adapter` 对 sim/real 原始话题做 profile 映射。
- color、depth、camera_info、points 使用 Sensor Data QoS；状态话题使用可靠 QoS。
- adapter 的重复 `rclpy.shutdown()` 清理异常已修正。

### 5.4 MoveIt

- `sim_moveit.launch.py` 复用师兄 MoveIt 配置并把 `robot_description` 指向应用包装 Xacro。
- 已安装并加载上游 `moveit_ros_perception` 点云 OctoMap updater。
- 点云输入保持 `/sensors/camera/points`，OctoMap frame 为 `base_link`。
- 由于 RGB-D 渲染链未通过，MoveIt 点云融合最终门禁尚未通过。

## 6. 已获得的运行证据

| 检查项 | 结果 | 说明 |
|---|---|---|
| Ubuntu/ROS doctor | PASS | Ubuntu 22.04、Humble、ros2、colcon、rosdep、vcs、xacro 均可用 |
| 五包构建 | PASS | 五个 Phase 0–1 包均可构建 |
| 静态契约 | PASS | `python3 test/contract_checks.py` |
| Python 测试 | PASS | 当前 1 个测试通过 |
| 官方 gz_ros2_control demo | PASS | 控制器、硬件生命周期和 `/joint_states` 正常 |
| CS625 Xacro/SDF 控制插件保留 | PASS | URDF 与转换后 SDF 均包含规范插件声明 |
| CS625 空世界模型创建 | PASS | 单次创建成功，无重复实体 |
| CS625 controller manager | PASS | 空世界控制运行中节点与服务端存在 |
| 两个控制器 active | PASS | joint-state broadcaster 与 trajectory controller 均 active |
| 六关节 `/joint_states` | PASS | 六个 CS625 关节均有 position/velocity/effort |
| 眼在手 TF | PASS | `base_link -> camera_depth_optical_frame` 连续有效 |
| VMware `/dev/dri` | **NOK** | 当前终止阻塞项 |
| Ogre2/EGL 无头 RGB-D | BLOCKED | 没有 DRI 设备，无法完成运行验证 |
| `/sensors/camera/status` fresh | NOT ACCEPTED | 历史上出现过数据，但最终组合未在正确渲染条件下验收 |
| `/sensors/camera/points` 稳定 | NOT ACCEPTED | 同上 |
| MoveIt 无 Message Filter 丢帧 | NOT ACCEPTED | 当前仍会因上游渲染/TF链中断出现队列满 |
| 实机驱动 | BLOCKED | 缺少 Elite SDK 头文件，未测试真实硬件 |

注意：不要把 `ros2 service list` 中出现 `/controller_manager/list_controllers` 当成服务端存在的充分证据。等待中的 spawner 客户端也可能让该名称出现在 ROS 图中。必须同时确认 `/controller_manager` 节点、服务可调用以及 controller 状态。

## 7. 当前根因链

```text
VMware 内 /dev/dri 不可用
    ↓
Gazebo Fortress 无法建立 Ogre2/EGL 无头渲染上下文
    ↓
带 Sensors system 的世界停在渲染线程初始化
    ↓
Gazebo 服务端没有继续调度机械臂模型插件
    ↓
完整相机运行中 controller_manager / joint_states 可能缺失
    ↓
转动关节 TF 无法把相机末端树连接到 base_link/world
    ↓
MoveIt 无法把 camera_depth_optical_frame 点云转换到 OctoMap frame
    ↓
Message Filter queue full / dropping message
```

关键对照是：同一模型在不加载 Sensors system 的空世界中，controller manager、两个控制器、六关节状态和相机 TF 均已通过。由此不能再把完整运行失败归因于机械臂 URDF、控制器 YAML、插件安装或 TF 定义本身。

## 8. 已排除或已经纠正的方向

- `gz_ros2_control` 包或共享库缺失：已排除；官方 demo 通过。
- Humble/Fortress 二进制或 ABI 整体不兼容：已排除；官方 demo 通过。
- `GazeboSimSystem` pluginlib 注册缺失：已排除。
- 控制插件在 URDF→SDF 转换时丢失：已排除。
- 控制器类型/YAML 无法工作：已排除；CS625 空世界运行通过。
- 眼在手 TF 定义错误：已排除；六关节状态存在时 TF 连续有效。
- 仅把模型插件名从磁盘 ELF 名改为逻辑名即可解决：已证伪。该修改应保留以符合 Humble 规范，但不是完整根因。
- 把传感器渲染器切换到 Ogre1：已纠正。Gazebo Fortress 的 EGL headless 路径要求 Ogre2。
- 反复 spawn 解决请求超时：已移除。现在使用 topic 单次创建和确定性的退出事件编排。

## 9. 关键文件索引

- `src/cs625_ap_description/urdf/cs625_active_perception.urdf.xacro`
- `src/cs625_ap_description/urdf/cs625_camera_extension.xacro`
- `src/cs625_bringup/launch/sim_control.launch.py`
- `src/cs625_bringup/launch/sim_base.launch.py`
- `src/cs625_bringup/launch/sim_active_localization.launch.py`
- `src/cs625_bringup/launch/sim_moveit.launch.py`
- `src/cs625_bringup/launch/real_base.launch.py`
- `src/cs625_bringup/config/sim_controllers.yaml`
- `src/cs625_simulation/worlds/minimal_occlusion.sdf`
- `src/cs625_simulation/worlds/rgbd_fixture.sdf`
- `src/cs625_sensor_adapter/cs625_sensor_adapter/point_cloud_relay.py`
- `test/contract_checks.py`
- `docs/architecture.md`
- `docs/dependencies.md`
- `docs/frames_and_topics.md`
- `docs/simulation.md`
- `WORKLOG_2026-08-02_COMMON_PROFILE_UNDERLAY_AUDIT.md`

## 10. 恢复工作前的第一优先级

先恢复 VMware 客体中的 DRI/3D 能力，不要继续改 URDF、控制器、TF 或 MoveIt 参数来绕过这个问题。

只读检查：

```bash
ls -la /dev/dri
id
for device in /dev/dri/*; do
  test -e "${device}" || continue
  stat -c '%A %U %G %n' "${device}"
done
glxinfo -B
```

预期至少能看到 `/dev/dri/card*` 和/或 `/dev/dri/renderD*`，并确认当前用户通过正常的设备组权限获得访问。不要用全局 `chmod 777` 作为长期解决方案。如果目录或设备完全不存在，应检查 VMware 虚拟机设置、3D acceleration、VMware Tools/open-vm-tools、虚拟显卡驱动以及宿主机图形能力；这是虚拟机/虚拟显卡层问题，不是 ROS 包装层问题。

## 11. DRI 恢复后的建议验证顺序

### 11.1 构建和静态检查

```bash
cd /mnt/hgfs/cs625_active_perception/elite_ros
source /opt/ros/humble/setup.bash
source "${HOME}/cs625_underlay_humble/install/setup.bash"

bash scripts/build.sh
source "${HOME}/cs625_colcon/install/setup.bash"
bash scripts/test.sh --event-handlers console_direct+
python3 test/contract_checks.py
```

### 11.2 先回归已知通过的纯控制链

```bash
ros2 launch cs625_bringup sim_control.launch.py headless:=true
```

在另一个已 source 相同环境的终端：

```bash
ros2 node info /controller_manager
ros2 control list_controllers
timeout 10s ros2 topic echo /joint_states --once
timeout 10s ros2 run tf2_ros tf2_echo base_link camera_depth_optical_frame
```

### 11.3 再验证 RGB-D，不先启动 MoveIt

```bash
ros2 launch cs625_bringup sim_base.launch.py \
  launch_fixture_world:=false \
  launch_official_sim:=true \
  launch_moveit:=false \
  launch_sensor_adapter:=true \
  camera_enabled:=true \
  headless:=true
```

`headless:=true` 当前会使用 `-s -r -v 4 --headless-rendering`。普通 `-s` 只表示 server-only，不等于 EGL 无头渲染。

运行期间检查：

```bash
ros2 control list_controllers
timeout 10s ros2 topic echo /joint_states --once
ros2 topic list | grep -E 'camera|sensors|joint_states'
timeout 10s ros2 topic echo /sensors/camera/status --once
timeout 10s ros2 topic echo /sensors/camera/points --once
timeout 10s ros2 run tf2_ros tf2_echo base_link camera_depth_optical_frame
```

必须在 launch 仍运行时执行检查；不能在 `timeout` 已结束或手动 Ctrl+C 后检查 ROS 图。

### 11.4 相机通过后才加入 MoveIt

启用 `launch_moveit:=true`，确认：

- MoveIt 点云 updater 成功加载；
- `/sensors/camera/points` 持续有数据；
- `base_link -> camera_depth_optical_frame` 持续可变换；
- 不再持续出现 `Message Filter dropping message ... queue is full`；
- MoveIt 进程不崩溃，退出时 adapter 不再重复 shutdown 报错。

只有这些都通过，才允许进入 NBV/主动视觉迁移。

## 12. Phase 1 最终验收条件

以下条件必须在同一次、仍存活的完整仿真运行中同时成立：

1. `/controller_manager` 节点存在且服务可调用；
2. `joint_state_broadcaster` 和 `joint_trajectory_controller` 均为 `active`；
3. `/joint_states` 持续包含六个 CS625 转动关节；
4. 眼在手 RGB-D 原始 color/depth/camera_info/points 话题持续发布；
5. `/sensors/camera/status` 显示 connected/fresh，计数持续增加；
6. `/sensors/camera/points` 持续有数据；
7. `base_link -> camera_depth_optical_frame` 连续可用；
8. MoveIt 点云 updater 正常工作，不持续丢弃队列消息；
9. Gazebo GUI（若启用）不持续闪烁，headless 模式不阻塞渲染线程；
10. 静态契约、构建和测试仍全部通过。

## 13. 操作注意事项

- Gazebo `-s` 不等于 `--headless-rendering`。
- Fortress 的 EGL headless 渲染必须使用 Ogre2。
- `ros_gz_sim create` 的 `-allow_renaming` 是 presence flag；传入字符串 `false` 仍可能启用它。要表示 false，应省略该参数。
- 不要重新引入定时重复 spawn。
- 不要把固定测试相机重新加入机械臂完整世界；固定 fixture 与眼在手相机用途不同。
- 不要编辑师兄 underlay 来修应用层问题。
- 不要在相机/MoveIt 门禁通过前迁移 NBV。
- 若测试 GUI 路径，可用 `headless:=false` 走 GLX，但此前曾观察到 Gazebo 页面闪烁；最新分离方案尚未完成 GUI 回归，因此不能作为已验证替代方案。

## 14. Git 状态与交接风险

当前仓库显示 `No commits yet on main`，文件仍处于未跟踪状态。也就是说，目前没有可回退的 Git 基线，这是继续工作前的重要风险。

下一位维护者应先：

1. 阅读 `AGENTS.md`、课题开发说明书和本交接文档；
2. 审核当前全部文件及静态检查结果；
3. 确认没有构建产物或 VM 私有文件被纳入；
4. 在得到仓库所有者确认后建立初始基线提交。

本次交接不自动执行 commit、push 或任何真实硬件操作。

## 15. 下一位维护者检查清单

- [ ] 确认 Ubuntu 22.04 + ROS 2 Humble 环境；
- [ ] source `/opt/ros/humble`、Humble underlay、本项目 overlay，且顺序正确；
- [ ] 检查 `/dev/dri` 设备和权限，先解决 NOK；
- [ ] 运行 `glxinfo -B` 或等效只读图形诊断；
- [ ] 回归 `sim_control.launch.py` 的已知通过控制链；
- [ ] 在 Ogre2 + `--headless-rendering` 下单独验收 RGB-D；
- [ ] 验收统一 `/sensors/camera/*` 输出和 TF；
- [ ] 最后加入 MoveIt 并消除持续 queue-full；
- [ ] 完成 Phase 1 门禁后再评估主动视觉库和 NBV 复用；
- [ ] 全程不修改师兄/厂商仓库，不扩展五包边界。

## 16. 参考资料

- Gazebo Sim 6 headless rendering：<https://gazebosim.org/api/gazebo/6/headless_rendering.html>
- Humble `gz_ros2_control`：<https://control.ros.org/humble/doc/gz_ros2_control/doc/index.html>
- 仓库内说明：`docs/architecture.md`、`docs/dependencies.md`、`docs/frames_and_topics.md`、`docs/simulation.md`

## 17. 停止点

按用户要求，工作于 2026-08-03 在 `/dev/dri` 检查为 NOK 后停止。当前不再继续修改模型、渲染、MoveIt 或 NBV 代码。恢复工作时，应从本交接文档第 10 节开始，而不是重新调查已经通过的控制插件、控制器 YAML 或眼在手 TF。
