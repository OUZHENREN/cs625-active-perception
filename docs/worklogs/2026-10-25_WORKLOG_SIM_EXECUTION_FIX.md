# 2026-10-25 WORKLOG — 仿真执行链修复、真机同源修正、任务列表精简、标题乱码

> 接 `2026-10-24_WORKLOG_SIX_MODES_ACCEPTANCE.md`。上一轮只做到"六模式在仿真里
> `--plan-only` 全过"，本轮第一次**真的在 Gazebo 里执行**，暴露并修掉了一串只在
> 执行时才会出现的问题。真机仍不可达（`No route to host`），没有真机运动。

---

## 1. 一句话现状

**仿真第一次能把任务跑起来**：观测前缀 10 段全部真实执行成功（含 baseline），
执行段（pre_grasp 起）仍有一个已定位未修复的缺陷（见 §3）；
六模式 `--plan-only` 回归通过；Qt 面板窗口标题乱码已修；VS Code 任务 34 → 12。

## 2. 修掉的缺陷（每一条都是"跑起来才看见的"）

| # | 现象 | 根因 | 修法 | 影响面 |
|---|---|---|---|---|
| 1 | 五个观测阶段全 `PLAN_FAILED`，日志 `nearest solution is 2.6~3.1 rad … refusing a branch flip` | `ik()` 拿 `/joint_states` **原始数组**当 seed，却按 `JOINTS` 命名。Gazebo 按字母序发布 8 个关节（elbow 在前、两个手指夹在中间），真机现在也带夹爪关节 | seed 改为**按名字**取再排成 `JOINTS` 顺序（`arm_seed_from_state`），缺关节显式报错；加 5 个单测 | 真机+仿真 |
| 2 | 成功规划但 `ok=False`，日志 `chunk 2 starts far from the current state` | 执行子进程期间没有人 spin 节点，`/joint_states` 缓存**停留**在规划时刻 | `fresh_joints()` 等到"调用之后到达"的样本再比较 | 真机+仿真 |
| 3 | 失败一律显示 `PLAN_FAILED`，看不出是 IK 还是规划 | `detail(goal, points) if ok else 'PLAN_FAILED'` 把原因覆盖了 | 直接输出 `detail()`；executor 失败时打印**末尾 4 行**与**终端关节误差** | 真机+仿真 |
| 4 | 估算器在仿真里一直 `waiting for one colour+depth frame pair` | `icp_pose_once.py` 依赖 `/tcp_pose_broadcaster/pose`，Gazebo 没有这个控制器 | 加 TF 回退（`base_link → my_end_effector_link`），与 runner 同源 | 真机+仿真 |
| 5 | 仿真启动即卡在控制链检查 | 仿真走了真机的 `control_health.py`（等驱动/套接字），改成 `ros2 control list_controllers` 后又因 CLI 超时失败 | 直接探测**轨迹 action server** + `/joint_states` | 仿真 |
| 6 | 观测阶段后的位姿校验必然失败 | 校验用**真机照片训练的 YOLO**；Gazebo 渲染是无贴图纯灰，模型报 `no mask` | 仿真里校验降级为**提示**（直接写明原因），真机仍然阻断 | 仿真 |
| 7 | 仿真起始位姿是 vendor 全零，第一个观测就换 IK 分支 | 没有把真机起始位姿交给 Gazebo | 新增 `cs625_bringup/config/sim_initial_positions.yaml`（= 真机实测起始位姿），作为 launch 默认 | 仿真 |
| 8 | 仿真里夹爪命令会驱动**真夹爪** | runner 的 `gripper()` 无条件走 Windows PowerShell | 按 world 分流：仿真发 `/gripper_controller/commands`（EFG 开度→手指位置换算），真机才走 HIBTOT | 安全 |
| 9 | `camera_mode.py status` 崩溃 | `rclpy.shutdown()` 被调用两次 | `if rclpy.ok()` 保护 | 工具 |
| 10 | Qt 面板/相机窗**标题栏**中文变方框 | WSLg 的 Weston 用发行版字体画标题栏，`/mnt/wslg/distro/usr/share/fonts` 里只有 DejaVu，无中文字形（`xprop` 证明 Qt 已发出正确 UTF-8） | 顶层标题改 ASCII，中文名留在窗口内；`QMessageBox` 标题同改；子进程加 `PYTHONIOENCODING=utf-8` | UI |
| 11 | VS Code 任务 34 条太啰嗦、且丢了 DSH 快速启动 | 上一轮重写任务列表时遗漏 | 34 → **12 条**，picker 折叠组合，恢复 `99 · DSH Web 快速启动`（端口探测→后台 setsid 启动→从日志取带 token 的 URL→打开浏览器） | 工具 |

## 3. 仍未修、已定位的缺陷（诚实记录）

`pre_grasp` 起的所有执行段：`status=6 error=-5 (GOAL_TOLERANCE_VIOLATED)`，
终端误差集中在**单个腕关节**（第一次 `wrist_1` 1.249 rad，第二次 `wrist_2` 1.381 rad）。

已做的排除与证据：

```text
· 单关节测试：只动 wrist_2 的轨迹 → status=4 SUCCESSFUL，关节确实动了
  → 腕关节本身能执行，不是"卡死"
· 时间放慢 1.6×（仿真专用）→ 仍然 GOAL_TOLERANCE_VIOLATED
· 目标位姿的 wrist_2 = 1.571 rad ≈ π/2 → 腕部奇异（wrist_1 与 wrist_3 共线）
· 控制器：6 个关节全被 joint_trajectory_controller 认领，goal_time 15 s
```

结论：**问题在"穿过腕部奇异的规划轨迹"这一侧，不在控制器/关节侧**。下一步要查的是
规划出的轨迹在奇异点附近的关节速度/加速度是否超出仿真被控对象能跟踪的范围
（MoveIt 的时间参数化在奇异点附近会给出很大的关节速度）。

## 4. 演示视频（所有者要求的一段）

```text
~/cs625_evidence/sim_load_efficient_20261025.mp4   6.8 MB  322 帧  2 fps
内容：Gazebo 里 load/efficient 的观测前缀（阶段名/序号/状态烧在画面上）
限制：① 只覆盖到 baseline（执行段卡在 §3 的缺陷）；
      ② Gazebo 世界无贴图，画面是纯灰几何；
      ③ 录制时相机与仿真在同一 ROS 域会混流，已用独立 ROS_DOMAIN_ID 隔离。
```

## 5. 六模式只规划回归（真机配置，夹具碰撞体已发布）

```text
load/safe        19/19   exit=0
load/efficient   19/19   exit=0
unload/safe      38/38   exit=0
unload/efficient 38/38   exit=0
full/safe        57/57   exit=0   （必须 --handover auto，否则按设计在装入与取出之间停一次）
full/efficient   57/57   exit=0
```

全量单测：见 §8。

## 6. Capability layer

```text
- environment and dependencies      PASS
- robot model and simulation        PARTIAL  仿真能执行观测前缀，执行段卡在腕部奇异轨迹
- kinematics, control and planning  PASS     六模式 plan-only 回归通过
- vision, hand-eye calibration/TF   PASS     相机内参/噪声已对齐；仿真渲染无贴图 → 视觉校验仅提示
- active perception / NBV           NOT STARTED
- real-hardware integration/safety  BLOCKED  机器人网络不可达
```

## 7. 下一步唯一推荐任务

把 §3 的腕部奇异轨迹拿到桌面上查：对 `pre_grasp/grasp/pre_insert` 三个目标位姿，
打印规划轨迹的**关节速度/加速度峰值**，确认是否在奇异点附近超出仿真被控对象的跟踪能力；
若是，改用"远离奇异的目标姿态"或分段减速，再重跑 load/efficient 直到 19/19。

## 8. 本轮测试

```bash
python3 -m pytest test/ -q            # 见提交信息里的最终数字
python3 test/contract_checks.py       # PASS
# 新增 test/test_ik_seed.py：字母序/真机序/缺关节/多余关节 5 个用例
```
