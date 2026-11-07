# 2026-10-26 WORKLOG — 任务收敛为「4 个主按钮 + 排查项」

> 承接上一条工作日志（VS Code 任务修补）。所有者反馈"步骤太多、看不懂为什么分这么多"，本轮按提出的
> 方案把任务列表收敛：**A1 一键开工 / A2 开动 / B 跑任务 / C 离线测试**，其余降级为 P 系列排查项。
> 只改了 `.vscode/tasks.json`，没有改任何 ROS 脚本，没有让机械臂运动。

## 1. 现在的任务列表（16 条，前 4 条是日常唯一要用的）

| 标签 | 做什么 | 是否需要人守着 |
|---|---|---|
| **A1 · 一键开工（真机）** | 起驱动 + MoveIt（已在跑就跳过）→ 等就绪 → 把夹具加进规划场景 → 打印四项状态；**不激活控制器** | 约 20–40 s，之后后台常驻 |
| **A2 · 开动** | 激活 arm_controller → 再查一次状态 → 四项检查 | 几秒；动臂前最后一次人工确认 |
| **B · 跑一次任务** | dry-run / plan-only / execute（picker） | execute 时人必须在现场 |
| **C · 离线测试** | pytest + 契约检查，不碰机器人 | 改代码后跑 |
| P1 环境校验 | 换终端/改代码后 | 否 |
| P2 构建 overlay | 改了包之后 | 否 |
| P3 相机 | modeling（有深度，0.8 fps）/ fast（彩色 25 fps，无深度）/ status | 否 |
| P4 采集 + 建模 + 门禁 | capture（臂会动）/ rebuild / verify（合并了原 03 与 04） | capture 需要 |
| P5 规划场景 | apply / verify / remove（重启 MoveIt 后要重做） | 否 |
| P6 感知链路自检 | 相机 → `/sensors/*` 的 publisher 数 | 否 |
| P7 分步起真机 | A1 失败时拆成 base / moveit 排查 | 否 |
| P8 伪硬件冒烟 | 不接真机验 URDF/ros2_control | 否 |
| P9 Qt 面板 | B 的替代界面 | 否 |
| P10 实验归档 | 归档最新一次 run | 否 |
| P11 文件查看器 | 浏览器看视频/日志 | 否 |
| P12 DSH Web | 打开本项目对话界面 | 否 |

## 2. 关键设计（为什么 A1 不替你激活控制器）

- A1 只做"把机器人变成可被规划的状态"：驱动（状态来源）、MoveIt（规划服务）、场景（房间地图）。
- A2 才做"交出执行权"：`arm_controller` inactive → active。这一步有物理后果，所以保留为独立的人工点击。
- A1 是幂等的：驱动/MoveIt 已在跑就跳过启动，不会起第二份；后台进程写 `runtime/logs/real_startup_*.log`，
  关掉任务终端也不会停。

## 3. 修掉的一个真实坑（本轮实测）

`ros2 control switch_controllers --activate` 在控制器**已经 active** 时返回 **退出码 1**（消息是
`Controller with name 'arm_controller' is already active.`）。A2 最初的 `|| exit 1` 会让"已经好了"被判成失败。
现在改成：命令允许失败 → 再读一次 `list_controllers` 的 state 列 → 不是 `active` 才算失败。

## 4. 验证

```bash
# 16 条命令全部通过 bash -n（含替换 picker 默认值）
# A1 实跑（栈已在运行）：跳过启动 → 发布 1 个夹具 → 四项状态打印 → exit 0
# A2 实跑：already active 不再误判 → safety_mode 1 / robot_mode 7 / controller active /
#          control_health 三 True → exit 0
# C 实跑：pytest 238 passed + 契约 PASS → exit 0
# B dry-run 实跑：模型 7.1 h → REJECT，exit 2（符合"过期即拒绝"）
```

## 5. 仍挡在"真机自动化一次跑通"前面的两件事（未变）

1. **环境模型过期**：需要 P3（相机 modeling）→ P4 capture（臂会动，需授权）→ P4 rebuild → P4 verify PASS。
2. **相机归一化链路**：`camera_mode.py fast` 是彩色-only；`modeling` 才有深度；`real.yaml` 四个输入话题仍为空，
   `/sensors/camera/points` publisher=0。P6 会把这件事一眼打印出来。

## 6. 下一轮唯一推荐任务

先按 **A1 → A2 → P3(modeling) → P6** 确认相机链路现状，再**把 `real.yaml` 的四个输入话题补成实测值并起适配器**，
让 P6 里 `/sensors/camera/points` > 0。之后才谈 P4 采集 → rebuild → B 的 plan-only/execute。
