# 2026-10-27 WORKLOG — 多视角配准定案：工件没动，是单视角 ICP 测错了

> 回应所有者 7 个问题中的第 3、4、5、6、7 点；并用今天采集的 24 帧做**多视角整模配准**
> （RVS 的口径），对"工件到底在哪"给出定论。本轮无运动指令。

## 1. 定案证据（决定性）

用今天 10:32 的 24 帧合并点云（`runtime/environment/latest/scene.ply`，88k 点）在旧工件位置
附近裁剪，与 `弹仓_装配导出.STL` 做 FPFH+RANSAC 全局配准 + 多尺度 ICP：

```text
RANSAC fitness 0.848, rmse 11.2 mm
精配准后 fitness 0.443, rmse 5.4 mm
多视角工件中心        = (0.723, 0.621)
10-24 旧模型中心     = (0.7206, 0.6182)   → 差 3.4 mm
单视角 ICP 给的中心  = (0.8029, 0.6908)   → 差 106.3 mm
```

⇒ **工件在 10-24 的位置上（3.4 mm 内），没有移动 117 mm；是单视角 ICP 锁定了一个自洽但错误的解。**
`check_pose_stability` 的"三帧一致"只证明可重复，不证明正确；模板的近方形正面还有四重对称，
单视角尤其容易锁错分支（10-23 日志已写明单视角 fitness 上限只有 0.2–0.3，RVS 的 0.5 口径是多视角）。

## 2. 由此更正的三件事

1. **上午改配置是错的**：`config/tasks/common.yaml` 已回滚到 10-24 基线（`pre_grasp [0.7677,0.4903,-0.0595]`、
   `grasp [0.8316,0.5918,-0.0583]`），并把 `workpiece_reference` 设为 10-24 实测位姿。
2. **"向右偏移"不是工件位置问题**：工件位置与 10-24 一致，而偏移在仿真里同样出现
   ⇒ 偏移在**抓取位姿/工具几何的定义**里（TCP 的 −46 mm x 偏置、夹爪支撑臂相对法兰的
   146 mm 前伸等），不在工件测姿。这是下一轮要单独量清楚的量。
3. **拒绝对陌生位姿不继续**的做法要改：正确做法是**每次建模都用多视角算出位姿**（见第 3 节），
   而不是拿单视角结果和常数比较后停住。

## 3. 设计变更（回应第 4/5/6 点）

- **24 帧的用途**（第 4 点）：不是 YOLO 训练，而是 ①环境模型：两张桌面平面、槽位、
  碰撞体素；②**多视角工件定位**：本次实验证明它能给出 3.4 mm 的稳定中心。YOLO 训练数据
  是另一条线（`capture_environment.py` / `capture_dataset.py --tag`）。
- **工件位姿不再定义成常数**（第 5 点）：`model_environment.py` 目前从命令行/配置取位姿，
  门禁拿它和上一版比，等于自证。改为：**每次建模后用多视角配准测出工件位姿**，写入
  `model.json`；任务启动时从 `model.json` 推导 `pre_grasp/grasp`（用已验证的零件系抓取点）。
- **不再"拒绝陌生位姿"**（第 6 点）：定位成功的判据是多视角配准的质量（fitness/rmse/覆盖率）
  与投影叠加；质量达标就用它推导位姿并继续；质量不达标才重测（换视角/加帧），而不是停在原地。
- **正面拍摄对 ICP 不利**（第 2 点）：已写进精观测位注释；精观测固定用斜视（az 25°/el 30°）。

## 4. 本轮代码改动

- `scripts/execute_assembly_chunks.py`（回应第 1 点"验收慢"）：
  · goal_time_tolerance 从 `duration+25 s` 收紧到 `duration+5 s`；
  · 等待预算从 `2*duration+40` 收紧到 `duration+20`；
  · 等待期间持续读 `safety_mode/robot_mode`（改用 TRANSIENT_LOCAL QoS，之前根本收不到），
    非 1/7 立即取消并返回专用码 9。
  效果：控制器失败/急停会在 ~5–20 s 内上报，而不是等近两分钟。

## 5. 仍待解决（下一轮）

1. **系统性右偏**：在仿真里用已知真值量化工具相对 TCP 的横向/纵向偏置，再决定是改
   `grasp_q`、`pre_grasp` 的横向分量，还是在工具模型里补偏置。**在这之前不做真机抓取。**
2. **多视角定位落地**：把本轮的配准步骤做成脚本（带朝向先验：工件总是"立正"摆放，
   只在名义姿态 ±10° 内搜索，避免四分之一转的简并），写入 `model.json`，并由任务读用。
3. 第 1 点的验收时延还要在真实失败上实测一次（本轮只验证了代码路径）。

## 6. Capability layer

```
- environment and dependencies      PASS
- robot model and simulation        PARTIAL
- kinematics, control and planning  PASS（plan-only 19/19、pre_grasp 验收到位）
- vision, hand-eye calibration/TF   PARTIAL（多视角定位已验证可行，尚未落地成脚本/流程）
- active perception / NBV           NOT STARTED
- real-hardware integration/safety  BLOCKED（系统性右偏未解释前不抓取）
```
