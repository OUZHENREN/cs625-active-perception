# 2026-11-03 WORKLOG 夹具碰撞事故与运动硬门禁

## 1. 事故（时间线）

```text
16:24  按旧 R1（10-30，已 74 h）算的 pre_insert 下降到 z≈0.427 → 保护停止
       （y 偏 -36 mm 撞插槽左侧；所有者的观察："左偏 3–5 cm"）
16:35  用标定值目标重新下降 → 再次保护停止（z=0.427，进入槽口 16 mm 处卡住）
16:50  抬升也触发保护停止（工件被卡住，抬不动）
       → 所有者手动抬升 + 安放工件
17:0x  检查发现 effort 接口已死：Σ|effort|≈0（承载 6.8 kg 工件时绝不可能）
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

## 6. 追加（18:50–19:00）：重量更正、力矩基线、反控根因、RVS 匹配取证

### 6.1 工件重量更正（所有者）

工件质量 **6.8 kg**，不是 19 kg。已全库替换（scripts/ 注释、config、
工作日志）。模拟侧 GRIPPER_EFFORT=300 N/侧 的推导注释同步更新
（6.8 kg → 66.7 N 需求，300 N 余量更大，行为不变）。

### 6.2 effort 基线随负载变化 —— 已修复

```text
compliant_insert 的 --contact/--safety 从【绝对值】改为【基线之上的余量】：
  默认 0.68 / 1.63（= 所有者确认的 16.68/17.63 − 实测空载基线 16.001）
  seed() 时测 1 s 静止基线，contact = 基线 + 0.68，safety = 基线 + 1.63
  验证：空载 16.001 → 16.68/17.63（复现确认值）；携件 19.7 → 20.38/21.33（不再误触发）
测试：test_effort_thresholds_are_margins_over_the_live_baseline（293 passed）
```

### 6.3 反控掉线根因（驱动日志取证，5 次闭合事件与所有者观察一致）

```text
驱动日志 TcpServer 记录：50001 close 原因为 "Connection reset by peer"，
50003/50004 为 "End of file" —— 是【机器人侧】关闭连接，不是 WSL 侧。
时间线：每次闭合前要么是一次保护停止（外部脚本被机器人终止），
要么是 PrimaryPort "End of file"（机器人主通信口自己断开）。
结论：掉线是机器人端在保护停止/通信抖动时终止 external_control 脚本的结果，
WSL 侧无法根治，只能"检测→重发→带门禁"。已落地的防御：
  ① EFFORT_DEAD 门（力矩死→拒绝运动）
  ② 保护停止后 60 s 内禁止自动重发重试
  ③ 轻量恢复（只 resend+滑条，不释放伺服）
```

### 6.4 RVS 匹配取证与视位姿漂移门

```text
取证（三天数据交叉）：
  同一固定插槽的粗匹配输出 T0 跨会话漂移 39.4 mm：
    10-30 两次：(1.0841, 0.049, 0.6145)  与教学 S0 差 0.5 mm
    今天 16:03 ：(1.0785, 0.0101, 0.6172)  差 39.4 mm
  工件侧 T2 漂移 21 mm（但 S3 仍准到 3 mm —— 工件特征明显、插槽对称难锁）
  相机证据：两天 T1 触发时保存的点云质心只差 9 mm（相机几乎同位置），
  而喂给 RVS 的法兰位姿差 41 mm，返回 R1 差 50 mm
  → 结论：精匹配结果随"喂入的机器人位姿"漂移（手眼链的姿态相关误差），
    根子是粗匹配 T0/T2 先漂了；站在教学位就没事（10-30 两次都准）

修复（已提交）：
  config/tasks/common.yaml: rvs_taught_views（S0/S2 教学位姿，来自所有者工程记录）
  teaching_sequence.view_drift_mm()：每次 T0/T2 输出与教学位比对
  S0 漂移 > 20 mm → 序列拒绝（今天 39.4 mm 会被当场拦下，不会带偏 T1）
  S2 漂移 → 仅告警（工件侧 21 mm 实测可容忍）
  回放验证：10-30 S0 → 0.5 mm 通过；今天 S0 → 39.4 mm 拒绝 ✓
测试：test_view_drift_matches_session_bias（293 passed）
```

### 6.5 所有者战略调整（后续规划依据，待新 PLAN 文档）

```text
依据：G:\...\2026-10-30_CS625视触融合精密装配课题总体规划与实施方案_完整版.md
主要实验 = ①主动视觉观测提高抓取精度 ②插入过程的六维力检测
精简：后续的"取出"取消；插入若无合适进入方式则先搁置；
主攻：自己的视觉算法开发 + 仿真六维力平台搭建。
示教链路保留到"能稳定抓到工件"为止（服务于主动视觉的抓取实验）。
```
