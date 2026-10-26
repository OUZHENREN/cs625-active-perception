# 2026-10-26 WORKLOG — "控制器启动不了"其实是 CLI 假故障

> 所有者运行 A2 得到 `已终止，退出代码: 1`，以为控制器起不来。真机运动：否。
> 本轮只改健康检查/激活方式与 A2 任务；没有改运动逻辑。

## 1. 结论先说

**控制器一直是 active 的**。直接调 `controller_manager` 的 `list_controllers` 返回
`arm_controller state='active'`；失败的是 `ros2 control list_controllers` 这个 **CLI**：

```text
同一条命令，两次实测：
  第 1 次：1.61 s 返回正常
  第 2 次：Failed getting a result ... in 10.0 s（连续 3 次）→ 命令失败
```

A2 和 `control_health.py` 都靠解析这个 CLI 的 stdout 判断状态，CLI 一超时，
`state` 就是空的 → 打印"激活失败：状态 = 未知" → 退出 1。**根因是测量方式，不是控制器。**

诱因：相机现在按 1280x960 发布 925k 点的点云（约 1 fps、单帧 ~15 MB），DDS 负载很高；
`ros2 control` CLI 在这种流量下会间歇性卡住。`move_group` 目前也吃掉 ~54% CPU 在消费这份点云。

## 2. 改了什么

| 位置 | 改动 | 为什么 |
|---|---|---|
| `scripts/control_health.py` | ①控制器状态改成**直接调服务** `{manager}/list_controllers`（短命 rclpy 节点，12 s 上限），CLI 只作兜底；②新增 `--activate`：先查状态，已 active 就跳过，否则调 `switch_controller`（STRICT）；③`/joint_states` 探测改成 4 s 窗口 + 失败重试一次；④失败提示改成 `python3 scripts/control_health.py --activate` | 服务调用不依赖 CLI 的固定 10 s 超时逻辑；切换后 CM 忙，第一次探测会误报 joint_states 不通（实测 125 Hz 却在报 False） |
| `.vscode/tasks.json`（A2） | 打印两个 mode → `control_health.py --activate`（一次调用完成"激活 + 三项健康 + 状态打印"），不再执行 `ros2 control switch_controllers` | 旧 A2 在 CLI 超时时会白等 30 s 再误报失败（实测 37 s / 退出 1） |

## 3. 实测

```text
# 先人为 deactivate 控制器，验证真正需要激活的路径
ros2 control switch_controllers ... --deactivate arm_controller --strict   → Successfully switched controllers!
python3 scripts/control_health.py            → controller: False, arm_controller: inactive, exit 1

# A2（从 inactive 开始）
[A2] 四项检查（先看驱动状态）
  safety_mode : mode: 1
  robot_mode  : mode: 7
[A2] 激活控制器（已 active 则跳过；直接走 controller_manager 服务）
activated: arm_controller: active
health: {'sockets': True, 'controller': True, 'joint_states': True}
  arm_controller: active
→ exit 0，用时 14.5 s

# A2 再跑一次（已 active）
activated: arm_controller is already active
→ exit 0，用时 11.4 s

# 回归
pytest 245 passed；contract_checks Phase 0–6 PASS
```

## 4. 需要你知道的性能事实

- 相机 1280x960 彩色+深度+点云 = **约 1 fps、单帧 925k 点**。这会让 DDS CLI 工具间歇性超时，
  也让 `move_group` 吃掉 ~54% CPU（它在把点云转成 octomap 做碰撞）。
- 目前的规避：**所有关键判断都改走服务/话题，不再用 `ros2 control` CLI**。
- 如果后续发现 plan-only 变慢或规划被 octomap 挡住，下一轮再决定：给适配器加点云降采样，
  或让 MoveIt 不订阅这份全分辨率点云。**本轮没有擅自改**，因为那会改变规划语义。

## 5. Capability layer

```
- environment and dependencies      PASS    00 可复现
- robot model and simulation        PARTIAL 真机驱动/MoveIt 可起；仿真 insert 仍未通过
- kinematics, control and planning  PASS(真机) 控制器可由脚本可靠激活；plan-only 3 段已验证
- vision, hand-eye calibration/TF   PASS(链路) 相机→/sensors/* 已通；标定精度未复验
- active perception / NBV           NOT STARTED
- real-hardware integration/safety  PARTIAL 四项齐；环境模型仍过期，未做 execute
```

## 6. 下一轮唯一推荐任务

**继续所有者要的验证**：P3（相机 modeling）→ P4 capture（臂会动，需授权）→ P4 rebuild → P4 verify PASS
→ B dry-run（应 exit 0）→ B plan-only（19 段）→ B execute。
若在 plan-only 阶段发现规划变慢，先看 `move_group` CPU 与 `/sensors/camera/points`，再决定是否给点云降采样。
