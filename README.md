# CS625 主动感知与精密装配

> 课题：**面向遮挡光学元件精密装配的可达观测规划与视触柔顺控制研究**（2026-11-26 定稿）
>
> 状态表、开工清单、文档导航、VS Code 任务表的**唯一权威入口是 [`docs/README.md`](docs/README.md)**。
> 本文件只保留课题定位、三分钟上手与安全红线摘要，数字如有出入以 docs/README 为准。

## 三分钟上手

```bash
source scripts/source_dev_env.sh --full     # 唯一环境入口（ROS 2 Jazzy + 本仓库 overlay）
python3 -m pytest test/ -q                  # 离线测试（411 passed；必须 source，否则缺 rclpy 少收集）
python3 scripts/check_docs.py               # 文档一致性
python3 scripts/data_report.py              # 我有哪些数据、放在哪里
python3 scripts/todo_week.py --show         # 当前周待办（唯一入口 docs/TODO.md）
```

真机动作一律走 `scripts/` 下的工具链（四项门 → `plan_pose_ik` 安全闸 → 执行器治理），
**默认 dry-run / require_confirmation，绝不裸跑**。

## 安全红线摘要（完整版见 `docs/agent_memory.md` 与 AGENTS.md）

```text
1. 速度、关节行程、工作空间限值由代码强制：关节 ≤0.10 rad/s，单次规划单关节 ≤25°，
   腕部绝对 ±150° / 单次相对默认 45° / J4 建议 ±120°；不满足就拒绝，绝不放松。
2. 动臂前四项门必须全过：safety_mode=1、robot_mode=7、arm_controller active、control_health 全 True。
3. 真机动作一律后台执行；超过 1 分钟无反馈必须停下来报告；一次调用干完一件事。
4. 不删历史：旧代码/文档/实验数据只读留档；归档与替代必须所有者书面同意。
5. 提交走 .git/hooks/pre-commit（三连检查；新机器 bash scripts/install_hooks.sh）。
```

## 仓库结构（速览）

```text
docs/            唯一导航 docs/README.md；待办 docs/TODO.md；文献精读 docs/literature/
scripts/         任务链与工具（六模式入口 run_task.py、环境建模、观测、实验记录）
src/             12 个 ROS 2 包（感知/规划/执行分层，不设 mega-node）
config/tasks/    任务常量与站位表（唯一权威数值源）
```
