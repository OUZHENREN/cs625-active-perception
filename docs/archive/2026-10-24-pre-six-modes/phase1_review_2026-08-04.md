# CS625 Active Perception — Phase 1 Gate Review

- 日期：2026-08-04
- 阶段：Phase 1（官方仿真与最小应用骨架）
- 状态：**PASS（含 1 项已知阻塞）**
- 环境：WSL2 + Ubuntu 22.04 + ROS 2 Humble + NVIDIA RTX 3060 GPU
- 回退环境：VMware + Ubuntu 22.04 (xvfb + ogre1 fallback)

---

## 1. 目录树

```
elite_ros/
├── .gitattributes
├── .gitignore
├── AGENTS.md
├── README.md
├── .repos/{common.repos, sim.repos}
├── docs/
│   ├── architecture.md
│   ├── dependencies.md
│   ├── frames_and_topics.md
│   ├── simulation.md
│   ├── interfaces.md
│   ├── migration_from_legacy.md
│   ├── phase1_review_2026-08-04.md
│   └── decisions/{ADR-001, ADR-002, ADR-003}.md
├── scripts/{bootstrap_humble.sh, build.sh, test.sh, doctor_humble.sh}
├── config/{common, sim, real, scenes}/
├── src/
│   ├── cs625_ap_interfaces/      (msg: ActiveLocalizationState, SensorStatus)
│   ├── cs625_ap_description/     (xacro wrapper + camera extension)
│   ├── cs625_sensor_adapter/     (RGB-D relay, sim/real profiles, 1 test)
│   ├── cs625_simulation/         (worlds: rgbd_fixture, minimal_occlusion, cs625_robot, cs625_static, cs625_fixture_robot)
│   └── cs625_bringup/            (launch: sim_control, sim_base, sim_moveit, sim_static, sim_active_localization, real_base, sensor_adapter, sim_sensor_bridge)
└── test/{contract_checks.py, README.md}
```

## 2. 上游固定版本

| 依赖 | 版本 | 用途 |
|------|------|------|
| ROS 2 Humble | `humble` (apt) | 发行版 |
| MoveIt 2 | `2.5.9-1jammy.20260616.121430` | 运动规划 |
| ros_gz / gz_ros2_control | `0.244.25` / `0.7.20` | Gazebo 桥接与控制 |
| Gazebo Fortress | `6.18.0` | 仿真引擎 |
| Ogre-Next | `2.2.5+dfsg3-0ubuntu2` | 渲染引擎 |
| xacro | `2.1.1-1jammy.20260304.195513` | 模型展开 |

### Vendor Underlay（师兄/官方，仅引用不修改）
- `eli_cs_robot_description` — CS625 机械臂本体 Xacro + 网格
- `eli_cs_robot_simulation_gz` — Gazebo 仿真配置
- `elite_cs625_moveit_config` — MoveIt SRDF + 运动学参数
- `eli_common_interface` / `eli_dashboard_interface` — 消息接口

## 3. 构建与测试结果

### 构建（WSL2）
```
colcon build --symlink-install
Summary: 5 packages finished [22.8s]    (cs625_ap_interfaces,
 cs625_ap_description, cs625_sensor_adapter, cs625_simulation, cs625_bringup)
```

### 测试
```
colcon test
Summary: 1 test, 0 errors, 0 failures, 0 skipped  (cs625_sensor_adapter)
```

### 静态契约
```
python3 test/contract_checks.py
Phase 0-1 static contract checks: PASS
```

## 4. 已验证的 TF

```
base_link → ... → flange → my_end_effector_link → tool0
  → camera_mount_link → camera_link → camera_depth_optical_frame
```

- 验证命令：`ros2 run tf2_ros tf2_echo base_link camera_depth_optical_frame`
- 单一发布源：`robot_state_publisher`
- TF 唯一性：无重复发布

## 5. 已验证的话题

| 话题 | 状态 | 说明 |
|------|------|------|
| `/joint_states` | ✅ | 6 轴 CS625 关节 position/velocity/effort |
| `/sensors/camera/points` | ✅ | fixture 世界 320x240 点云, 4.11 Hz (WSL2) |
| `/sensors/camera/status` | ✅ | connected:true, fresh:true |
| `/sensors/camera/color/image` | ✅ | 通过 bridge → adapter |
| `/sensors/camera/depth/image` | ✅ | 通过 bridge → adapter |
| `/sensors/camera/depth/camera_info` | ✅ | 通过 bridge → adapter |

## 6. 控制器验证

```text
joint_state_broadcaster    → active
joint_trajectory_controller → active
```

## 7. 成功启动命令

### WSL2（GPU 硬件加速，推荐）
```bash
source /opt/ros/humble/setup.bash
source ~/cs625_underlay_humble/install/setup.bash
source ~/cs625_colcon/install/setup.bash
ros2 launch cs625_bringup sim_base.launch.py launch_moveit:=true launch_sensor_adapter:=true camera_enabled:=true
```

### VMware（无 GPU，xvfb + ogre1 回退方案）
```bash
xvfb-run -a -s "-screen 0 1920x1080x24 +iglx" \
  ros2 launch cs625_bringup sim_base.launch.py \
  launch_fixture_world:=true launch_official_sim:=false \
  launch_moveit:=false launch_sensor_adapter:=true \
  camera_enabled:=true headless:=false
```

## 8. 已知问题

### BLOCKED: Robot World RGB-D 渲染不稳定
- **现象**：fixture 世界 + 机械臂组合场景中，ogre2 Sensors 渲染线程偶发 `Waiting for init` 卡死
- **根因**：Ogre-Next 2.2.5 GL3Plus EGL PBuffer 设备选择缺陷（选 `/dev/dri/card0` 而非 `renderD128`），WSL2 GPU 路径偶发渲染初始化超时
- **影响**：静态世界（cs625_static.sdf）控制器稳定通过，相机渲染需等待/重试
- **解决方向**：升级 Ogre-Next 至 3.x（需 Ubuntu 24.04 + Jazzy）或等待上游修复

### BLOCKED: Elite SDK 缺失
- `eli_cs_robot_driver` 需 `/opt/elite-sdk-custom/include/Elite/EliteDriver.hpp`，仿真阶段不阻塞

### NOT STARTED: NBV/算法包
- Phase 0-1 严格禁止算法包。ViewpointSampler、InformationGain 等资产待 Phase 3-4 迁移。

## 9. P2 迁移建议

### P2 进入条件
- [x] 五包 build + test 通过
- [x] 控制器 + TF 验证
- [x] RGB-D fixture 世界相机验证通过
- [ ] 静态世界相机渲染稳定（阻塞项）
- [x] Git 仓库 + 文档基线建立

### P2 第一步
1. `cs625_sensor_adapter` 验证 CameraInfo 正确性
2. 保存一帧点云 + 目标位姿 rosbag 样本
3. 验证 frame、timestamp、QoS 契约

## 10. Phase 0-1 边界确认

- 无 NBV、目标感知、视点生成、运动适配器、任务编排器、实验工具算法代码
- `cs625_sensor_adapter` 只做 remap/relay，不做目标检测
- launch 只组合包，不含业务逻辑
- `execute:=false` 默认安全
- 无真实机械臂运动代码

---

## 已执行的关键命令

```bash
# 环境安装
sudo apt install -y ros-humble-desktop ros-humble-moveit ros-humble-ros-gz \
  ros-humble-xacro ros-humble-ros2-control ros-humble-ros2-controllers \
  ros-humble-ros2controlcli ros-humble-gz-ros2-control python3-colcon-common-extensions

# 构建 underlay
cd ~/cs625_underlay_humble && colcon build --symlink-install

# 构建 overlay（elite_ros 五包）
cd /mnt/b/.../elite_ros
CS625_UNDERLAY_SETUP=$HOME/cs625_underlay_humble/install/setup.bash bash scripts/build.sh

# 测试
bash scripts/test.sh --event-handlers console_direct+
python3 test/contract_checks.py

# 仿真（WSL2 GPU）
ros2 launch cs625_bringup sim_base.launch.py

# 验证
ros2 control list_controllers
ros2 topic echo /joint_states --once
ros2 run tf2_ros tf2_echo base_link camera_depth_optical_frame
ros2 topic echo /sensors/camera/status --once
ros2 topic hz /sensors/camera/points
```
