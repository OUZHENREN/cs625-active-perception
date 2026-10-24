# scripts/ 索引

> 这些脚本**不是** ROS 包，它们是"把包和话题组合起来完成任务"的一层。
> 每个脚本只负责一件事；不在这里加新的抽象层。

状态说明：**主力**＝日常会用到、有验收证据；**工具**＝特定场景用；**旧**＝保留可读，
但新流程不再依赖（勿删）。

## 1. 任务入口（先看这四个）

| 脚本 | 作用 | 状态 |
|---|---|---|
| `run_task.py` | **六类模式唯一入口**：`--scope {load,unload,full} --mode {safe,efficient} --world {real,sim}`，合并配置 → 过环境门禁 → 调阶段执行器 → 真机运行时自动录像并归档 | 主力 |
| `run_pick_and_place.py` | 阶段执行器：load 的 19 段（含观测前缀）、unload 的 D1–D9；逐阶段速度、实测验收、`--plan-only` 串联规划 | 主力 |
| `execute_assembly_chunks.py` | 把一段关节轨迹按 chunk 下发到控制器；首点保护、`NO_MOTION`、诚实超时、`--action` 可切真机/仿真控制器 | 主力 |
| `model_environment.py` | 离线环境建模：点云 → 桌面/夹具/工件位姿 + 碰撞体 → `runtime/environment/<ts>/model.json`，并提供四种拒绝的门禁 | 主力 |

## 2. 感知与位姿

| 脚本 | 作用 | 状态 |
|---|---|---|
| `capture_dataset.py` | 带五道守卫（工作盒/观测包络/夹爪间距/取景/规划预检）的观测位移动 + 采帧；`--dry-run` 先审 | 主力 |
| `icp_pose_once.py` | 单帧 ICP 配准（CAD 模板），输出位姿 + 覆盖率/残差 | 主力 |
| `check_pose_stability.py` | 多帧共识门禁（位置 ≤2 mm、转角 ≤0.5°）；精观测位的验收工具 | 主力 |
| `capture_process_images.py` | 阶段边界抓一帧彩色+深度+meta（训练"阶段识别"用） | 主力 |
| `auto_label_four_class.py` | 几何自动标注（工件 CAD 投影 / 插槽点云投影） | 工具 |
| `auto_label_frames.py` | 单类工件几何标注（被 four_class 取代） | 旧 |
| `build_scene_model.py` | 多帧拼基座系点云 + 平面拟合 | 主力 |
| `depth_pose_once.py` | 单帧深度 → 掩膜点云 → 6D 位姿（ICP 之外的老路径） | 旧 |
| `coarse_to_fine_pose.py` | 粗→精的早期流水线（已被 `run_task` 的观测阶段取代） | 旧 |
| `ros_rgb_observer.py` | 早期实时 RGB 观察器 | 旧 |

## 3. 仿真与场景

| 脚本 | 作用 | 状态 |
|---|---|---|
| `build_sim_world.py` | **从真机实测生成** Gazebo 世界与场景配置（桌子/夹具/工件） | 主力 |
| `analyze_fixture_geometry.py` | 判定哪份夹具几何与真机一致（bbox、槽口宽度、是否贯通插入路径） | 工具 |
| `build_fixture_collision_mesh.py` | 把真机 RVS 夹具网格转成米制碰撞网格（抽稀到 60k 面） | 工具 |
| `apply_real_cell_scene.py` | 把真机场景的碰撞体发布进 MoveIt（默认只发夹具；桌子/工件按需） | 主力 |
| `camera_mode.py` | 相机在"精细 1280×960"与"高帧率"之间切换并实测 fps | 主力 |
| `sync_task_world.py` | 旧场景配置 ↔ SDF 世界的位姿同步（旧世界专用） | 旧 |
| `build_environment_collision.py` | PCD → MoveIt 碰撞体（被 `model_environment` 覆盖） | 旧 |
| `visual_mesh_publisher.py` | 按当前位姿发布盒子/插槽的可视网格 | 工具 |
| `capture_environment.py` | 环境宽视角采集（训练负样本） | 工具 |

## 4. 标定与运动辅助

| 脚本 | 作用 | 状态 |
|---|---|---|
| `verify_pose.py` | 实测 TCP 与目标比对（位置 mm / 姿态 rad），阶段后验收 | 主力 |
| `plan_pose_once.py` | 单个 TCP 位姿目标的规划（只规划） | 主力 |
| `move_tcp_linear.py` | 一次性直线 TCP 移动（带进度） | 工具 |
| `control_health.py` | 控制链健康检查（sockets / controller / joint_states），`--fix` 自动恢复 | 主力 |
| `recover_control.sh` | goal 超时/中止后的控制器恢复（resend + 切控制器 + 滑块 100%） | 主力 |
| `camera_extrinsics.py` / `sync_real_handeye.py` | 手眼外参同步到 URDF / 真机位姿链 | 工具 |
| `known_target_calibration.py` | 相机位姿链的已知目标标定（不动臂） | 工具 |

## 5. 实验数据

| 脚本 | 作用 | 状态 |
|---|---|---|
| `experiment_recorder.py` | 20 Hz 时序采集（关节/TCP/力力矩/阶段）+ `start/stop/status/set-phase` | 主力 |
| `archive_experiment.py` | 裁剪（首尾空闲 / 连续静止 / 重复帧）→ `~/cs625_experiment_archive/<id>/` + 索引行 | 主力 |
| `record_frames.py` | 旧版 1 Hz 录像 | 旧 |

## 6. 界面

| 脚本 | 作用 | 状态 |
|---|---|---|
| `cs625_ui.py` | Qt 主操作面板：六模式按钮 + 当前流程卡 + 状态 + 监视 + **实时曲线**（关节/TCP/力力矩）+ 日志；相机图在**独立窗口** | 主力 |
| `ui_plots.py` / `ui_plots_demo.py` | 自绘滚动曲线控件（10 Hz 合并重绘、暂停/清空/导出 CSV）+ 演示 | 主力 |
| `src/cs625_rviz_panel` | RViz2 内嵌面板（同样六模式按钮），`rviz2 -d .../cs625_task.rviz` | 主力 |

## 7. 环境与构建（改动要谨慎）

| 脚本 | 作用 |
|---|---|
| `source_dev_env.sh` | **唯一环境入口**（ROS Jazzy → 厂商 underlay → 本仓库 overlay） |
| `build.sh` / `test.sh` | 构建 / colcon 测试包装 |
| `run_minimum_showcase_matrix.sh` | 仿真矩阵（历史） |
| `sync_worklog.sh` | 工作日志镜像到 Obsidian |
| `bootstrap_humble.sh` / `doctor_humble.sh` | Humble 时代的环境脚本，**已过时**，仅保留 |

## 8. 约定

- 新脚本必须能 `--help`、失败时给明确退出码、默认不动机械臂；
- 动臂脚本必须支持"先预演/只规划"；
- 不在这里复制粘贴已有算法：能 import 的（`build_scene_model`、`apply_task_scene`）
  就用 importlib 复用；
- 任何脚本的输出如果是"生成物"，默认写到 `runtime/`（已 gitignore）。
