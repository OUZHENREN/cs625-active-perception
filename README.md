# CS625 Active Perception

面向遮挡目标定位的机械臂可达性规划方法研究。

本仓库是已确认的新 Git 根目录：

```text
B:\Recent\Robotic arm\Ubuntu_Share\cs625_active_perception\elite_ros
```

## 当前状态

当前处于 **P0 已完成、P1 基线接入进行中**。当前工作先建立可审查的包边界，并按说明书接入已有底座：

- 已创建五个 Phase 0–1 允许的 ROS 2 包骨架；
- 已在 Ubuntu 22.04 / ROS 2 Humble VM 中完成五包构建；
- 已在 VM 中通过 1 个 Python 测试；
- CS625 官方/师兄模型与参数通过 underlay 和 xacro wrapper 接入；仿真使用应用层单次 topic spawn 编排，避免修改师兄仓库中的启动文件；
- sim/real 共同复用 `sensor_adapter.launch.py` 与同一套 normalized RGB-D 配置；
- 尚未迁移 NBV、感知、规划或实验算法；
- 未启动仿真、fake hardware 或真实机械臂。

## 主线决策

- 环境：Ubuntu 22.04 VM + ROS 2 Humble + MoveIt 2 Humble + Gazebo Fortress/ros_gz；
- 架构：主动感知闭环是系统主框架，NBV 是可插拔视点策略；
- 里程碑：九月前优先完成仿真闭环，真机迁移进入 P6；
- Phase 0–1：严格只创建五个基础包；
- 第三方依赖：通过 `.repos`/underlay 管理，不在本仓库直接修改。

## 单仓库双 profile 不变量

本 Git 仓库只维护一套 common 应用接口和核心源码；sim 与 real 只提供各自
的配置和启动入口，不复制第二套主动感知核心：

```text
elite_ros/
├── src/                  # 一套 common application packages
├── src/cs625_bringup/config/common.yaml
├── src/cs625_bringup/config/sim.yaml
├── src/cs625_bringup/config/real.yaml
├── src/cs625_bringup/launch/sim_*.launch.py
└── src/cs625_bringup/launch/real_*.launch.py
```

`real_base.launch.py` 只组合 underlay 中已有的 `eli_cs_robot_driver` 和显式配置
的相机源话题。机器人地址默认为空，驱动、相机适配器和机械臂控制器均不会
自动启用；它不实现厂商驱动，也不改变 common 接口。

## 文档入口

- `docs/architecture.md`：目标分层与 Phase 0–1 边界；
- `docs/dependencies.md`：依赖 URL、版本状态和修改规则；
- `docs/frames_and_topics.md`：仿真/实机统一 TF 与话题契约；
- `docs/simulation.md`：仿真分层、启动和验收命令；
- `docs/migration_from_legacy.md`：active-vision、师兄库和现有 NBV 的逐项复用矩阵；
- `docs/decisions/`：环境、包边界和 sim/real 决策记录。

## 后续环境命令

所有 ROS 命令必须在 Ubuntu 22.04 VM 中运行，并先确认：

```bash
printenv ROS_DISTRO
```

应为 `humble`。P0/P1 脚本位于 `scripts/`，当前仍需在 VM 中执行。

## 工作日志

本项目工作日志主目录与 Obsidian 镜像目录分别为：

```text
B:\Recent\Robotic arm\Ubuntu_Share\cs625_active_perception
B:\Recent\Obsidian\Do be do be do\课题\项目\工作日志
```

每次输出工作日志时，两处直接写入同名 `WORKLOG_YYYY-MM-DD_<topic>.md` 文件。
