# 2026-11-03 WORKLOG 夹具碰撞事故与运动硬门禁

## 1. 事故（时间线）

```text
16:24  按旧 R1（10-30，已 74 h）算的 pre_insert 下降到 z≈0.427 → 保护停止
       （y 偏 -36 mm 撞插槽左侧；所有者的观察："左偏 3–5 cm"）
16:35  用标定值目标重新下降 → 再次保护停止（z=0.427，进入槽口 16 mm 处卡住）
16:50  抬升也触发保护停止（工件被卡住，抬不动）
       → 所有者手动抬升 + 安放工件
17:0x  检查发现 effort 接口已死：Σ|effort|≈0（承载 19 kg 工件时绝不可能）
       根因：recover_control.sh 的 deactivate/activate 周期释放伺服 + 反控通道
       当日 6 次掉线后的控制器切换，把 effort 状态接口打死了
17:0x  更糟：某次恢复时臂下沉 16 mm（0.5605 → 0.5441）
```

**结论：当日 3 次夹具碰撞 + 1 次臂下沉，都是"保护停止 → 恢复 → 盲动重试"这个循环
导致的。力矩接口死亡期间，一切接触检测都是瞎的。**

## 2. 本次整改（已提交，全部有实测证据）

### 2.1 运动硬门禁（`scripts/execute_assembly_chunks.py`）

| 门禁 | 行为 |
|---|---|
| EFFORT_DEAD | 每个 chunk 执行前采集 0.8 s 力矩窗口；峰值 < 1.0 → **拒绝运动**，exit 11 |
| 保护停止标记 | 检测到保护停止时写 `/tmp/teaching_estop_marker.json`（时间+原因+轨迹） |
| 停止后禁自恢复 | 60 s 内出现过保护停止 → NO_MOTION 时**不做**自动 resend，要求人工检查 |
| 力矩窗口采集 | 修了一个时序 bug：启动探测在"位置到达"后就退出，effort 窗口可能为空 |

实测：

```text
力矩死亡（重启前）：EFFORT_DEAD peak=0.000 → 拒绝，exit 11   ✓
力矩复活（重启后）：Σ|effort|=15.86 → 微动自检 ALL_ASSEMBLY_CHUNKS_OK  ✓
```

### 2.2 恢复脚本拆分

```text
scripts/recover_control.sh       轻量：只 resend_external_script + 速度滑条
                                 （不再 deactivate/activate，不再释放伺服）
scripts/recover_control_full.sh  完整：deactivate+activate（含警告：携带负载时
                                 会释放伺服，2026-11-03 实测臂下沉 16 mm）
```

### 2.3 序列层规则（`scripts/teaching/teaching_sequence.py`）

```text
execute_plan 对 exit 9（保护停止）/ exit 11（力矩死亡）打印"序列终止"的明确规则，
禁止"恢复-重试"循环。
```

### 2.4 诊断工具（`scripts/preflight_motion.py`）

```text
一次性报告：反控三端口 ESTAB + arm_controller active + safety=1/robot=7 +
力矩流存活（max|effort| ≥ 1.0 且 Σ ≥ 2.0）。供人工在动作前自查。
```

## 3. 驱动栈重启

```text
停：SIGINT 到 launch 进程组（未用 pkill）
起：real_base.launch.py 原参数 + activate_joint_controller:=false
激活：python3 scripts/control_health.py --activate
结果：
  · 四项门全 True，safety=1 / robot=7
  · 力矩接口复活：静止 Σ|effort| = 15.862（空载+工具，与 16.0 历史基线一致）
  · 新描述生效：无 gripper_*_finger_joint，move_group 不再刷
    "Missing gripper_right_finger_joint"（0 条）
  · tcp_server 位姿桥仍在 9999
```

## 4. 状态

```text
臂：TCP (1.0924, -0.0141, 0.5441)（无负载，工件已由所有者安放）
夹爪：50 mm 张开；safety=1 / robot=7
测试：291 passed
未闭环：插槽精定位的权威来源问题（R1 离散 ±25 mm）尚未解决 —— 等所有者决定
        下一步（触测标定 B 方案 / 其他）
```

## 5. 复验命令

```bash
cd /home/oy/projects/cs625-active-perception && source scripts/source_dev_env.sh --full
python3 -m pytest test/ -q
python3 scripts/preflight_motion.py                 # 期望 PREFLIGHT_OK
# 力矩死亡时必须拒绝（需要人为制造条件才能看到）
python3 scripts/execute_assembly_chunks.py <轨迹.json> --time-scale 1.5 \
  --goal-tolerance 0.004 --verify-final 0.004 --fault-poll 0.2
# 输出以 "effort_peak=" 开头 = 力矩活着；EFFORT_DEAD = 拒绝运动
```
