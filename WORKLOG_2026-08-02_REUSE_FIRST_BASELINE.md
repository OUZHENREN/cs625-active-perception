# 工作日志：复用优先的单仓库双 profile 基线

日期：2026-08-02

## 本次目标

严格按照 `CS625_主动感知ROS2框架与课题开发说明书.md` 和
`CODEX_PHASE_0_1_BOOTSTRAP_PROMPT.md`，在一个 Git 仓库内继续搭建基线：

```text
一个 Git 仓库
├── 一套 common 应用源码和接口
├── 一套 sim 配置与启动入口
└── 一套 real 配置与启动入口
```

不创建后续算法包，不整包复制旧 `cs625_full_system`/`cs625_nbv`，不修改
师兄或厂商 underlay，也不启动真实机械臂运动。

## 复用依据

- 师兄 CS625 描述包：复用 `cs_macro.xacro`、CS625 关节/几何底座、末端网格
  资源、`my_end_effector_link` 命名和末端偏移；通过 `cs625_ap_description`
  wrapper 接入，不复制整套机器人模型。
- 师兄仿真包：复用其 Gazebo RGB-D 传感器形状和 `cs_sim_moveit.launch.py`
  的 underlay 组合思路；官方仿真仍需在 Humble VM 核验后才允许显式启动。
- AIRLab-POLIMI active-vision：只借鉴 `bringup/interfaces/pointcloud/planning`
  的模块边界，不引入其机器人专用代码或 MoveIt fork。
- `OUZHENREN/Robot`：NBV、IK、轨迹、监控和实验日志已经登记到迁移矩阵，
  暂不迁入 Phase 0–1；后续按接口职责适配到说明书规定的未来包。
- PS800E1/图漾：当前本地代码扫描没有找到已验证的驱动包和 topic map，
  因此没有猜测厂商话题或连接参数；real profile 只接收显式配置的话题。

## 本轮变更

- 新增 `docs/migration_from_legacy.md`，记录三类参考库的逐文件/逐职责复用矩阵。
- 更新依赖、架构、仿真和根 README，明确单仓库 common/sim/real 不变量及 underlay 边界。
- `cs625_ap_description` 新增 application xacro wrapper：调用师兄的
  `eli_cs_robot_description` 宏，保留 `my_end_effector_link`，增加 `tool0`、
  `camera_mount_link`、`camera_link` 和 `camera_depth_optical_frame`。
- 相机扩展参数化复用师兄 Gazebo RGB-D 传感器段；不写入厂商默认 topic。
  相机宏要求 wrapper 显式传入 `camera_sensor_topic`，避免 xacro 空默认值歧义。
- `cs625_bringup` 增加 `common.yaml`、`sim.yaml`、`real.yaml` 和安全的
  `real_base.launch.py`。real 入口默认不启动驱动/相机适配器，只有显式传入
  已验证 underlay launch 和源 topic 才会组合。
  删除未被任何入口引用的旧 `profile_defaults.yaml`，避免出现两套 common 配置。
- 修正 `scripts/build.sh`、`scripts/test.sh` 的 colcon 调用顺序，并增加可选
  `CS625_UNDERLAY_SETUP`，用于在不修改 vendor 包的前提下接入已验证 underlay。
- 更新静态契约检查，仍强制 Phase 0–1 只有五个包，并检查 wrapper、real profile
  入口、profile 配置和安全默认值。

## 验证结果

本机静态检查：

```text
python test\contract_checks.py
Phase 0–1 static contract checks: PASS
```

本机 Python launch 文件语法检查：

```text
python -m py_compile \
  src\cs625_bringup\launch\sim_base.launch.py \
  src\cs625_bringup\launch\sim_active_localization.launch.py \
  src\cs625_bringup\launch\real_base.launch.py
PASS
```

在本轮文件变更之前，用户已在 Ubuntu 22.04/Humble VM 中报告五包 build
通过、一个 Python 测试通过。本轮新增 wrapper、real profile 和脚本修改后，
尚未在 VM 重跑 build/test；因此不能把旧结果当作本轮完整验收结果。

## 尚未完成与限制

- 尚未在 VM 中展开 wrapper xacro、验证 TF、Gazebo Fortress RGB-D、控制器和 MoveIt。
- 师兄仓库说明书 URL 为 `20260129`，本地 legacy remote 观察到的是 `20260306`；
  精确 URL/revision 未确认前不写入 `.repos` 的 PINNED 依赖。
- PS800E1/图漾真实驱动、输入 topic、相机内参和手眼标定仍需从实际 underlay/设备核验。
- 未迁移 NBV、目标感知、视点生成、运动适配器或任务编排器；这符合 Phase 0–1 边界。
- 本轮没有 fake hardware、Gazebo 或真实硬件运行，也没有发生真实机械臂运动。

## 下一步 VM 验证命令

在 Ubuntu 22.04/Humble VM 中执行：

```bash
cd /mnt/hgfs/cs625_active_perception/elite_ros
source /opt/ros/humble/setup.bash
bash scripts/doctor_humble.sh
bash scripts/build.sh
source "${HOME}/cs625_colcon/install/setup.bash"
bash scripts/test.sh --event-handlers console_direct+
python3 test/contract_checks.py
```

若已经有单独构建且 revision 已核验的 underlay，再通过环境变量传入其 setup
文件后重跑 build/test；不要把 underlay 的 build/install/log 拷贝进本仓库。
