# 2026-10-27 WORKLOG — 更正：被采信的工件位姿测量是错的（配置已回滚）

> 承接 2026-10-26 抓取撞击事故。上午按"实测工件位姿"重新推导并写入了抓取位姿；
> 所有者看现场照片后指出预抓取仍然偏右。本轮**证明问题在测量与采信，不在位姿公式**，
> 并把配置回滚到唯一已验证的基线。**本轮没有任何运动指令**（机器人处于急停）。

## 1. 本轮的三个事实

### 1.1 位姿公式是对的（可复现验证）
`grasp_from_measurement.py` 用参考工件位姿 + 抓取点常量，精确复现 10-24 的成功值：
`grasp = T · (-0.3235,0.0829,0.1793) → [0.8316,0.5918,-0.0583]`（误差 <0.1 mm，测试锁定）。

### 1.2 ICP 测量本身不可信（决定性证据来自图像）
上午按"实测中心 (0.803,0.691)"作为轨道中心采集 24 帧。相机是**瞄准这个中心**的：
如果中心正确，工件应落在画面中央。实际今天 10:32 的采集图里，工件明显偏**左上**，
视觉中心约 (-0.13,-0.14) 归一化位置；按 0.62 m 距离估算，对应约 **80–100 mm** 的位姿误差。
⇒ ICP 给出的 117 mm 漂移不是工件移动，而是测量锁定到了自洽但错误的解
（模板四重对称/特征锁定都可能；`check_pose_stability` 只证明"可重复"，不证明"正确"）。

### 1.3 我把未核验的测量直接当成真值改了配置
这是本轮真正的错误：`grasp_from_measurement --check` 把"测量 vs 参考"的漂移当成
"配置该改"的信号，而**没有要求先确认测量本身正确**。结果预抓取被改到 +82 mm，
现场看就是"更偏右"。

## 2. 已做的更正

- `config/tasks/common.yaml`：抓取位姿回滚到 10-24 真机跑通值
  `pre_grasp [0.7677,0.4903,-0.0595]`、`grasp [0.8316,0.5918,-0.0583]`、
  `grasp_q [-0.4835,0.8753,0.0075,0.0018]`；`workpiece_reference` 设为 10-24 实测位姿，
  并注明"新测量必须先经视觉核验才能替换"。
- 保留的改动：`model_environment` 从 `workpiece_reference` 取默认位姿、`--accept-workpiece-move`
  只在"传入位姿与配置一致"时生效（这层逻辑本身没问题，问题在数据）。

## 3. 下一步的判定方法（不再靠猜）

恢复软件栈后，把臂停在**精观测位**（不动到抓取区），运行：

```bash
python3 scripts/icp_pose_once.py \
  --weights ~/cs625_yolo_seg/runs/segment/runs/segment/box_slot_1280/weights/best.pt \
  --template "$HOME/cs625_rvs_archive/models_20261023/弹仓_装配导出.STL" \
  --voxel 0.002 --dump-projection /tmp/pose_overlay.png
```

然后**看叠加图**：
- 模板点云贴合工件轮廓 → 测量可信，才允许按它推导位姿；
- 模板偏在工件旁边/只贴合一部分 → 测量不可信，**继续使用 10-24 基线**，并改进测姿方法。

在叠加图被确认之前：不接近、不抓取。

## 4. 关于"是否复刻了师兄的抓取流程"（诚实回答）

没有。仓库里的抓取位姿是 2026-10-22/24 由本仓库自己的 ICP 流程推导并跑通的；师兄的栈
（`~/cs625_legacy_underlay_jazzy`）是只读的历史参照，没有被复刻或替换。
这两天做的是**保持这套已验证流程不变**，修它启动/安全/判据上的漏洞；
抓取几何本身一直是同一套常量，直到今天上午被我错误地改掉，现已回滚。

## 5. Capability layer

```
- environment and dependencies      PASS
- robot model and simulation        PARTIAL（仿真 insert 未通过）
- kinematics, control and planning  PASS（plan-only 19/19、pre_grasp 验收到位）
- vision, hand-eye calibration/TF   BLOCKED（工件位姿测量未通过视觉核验）
- active perception / NBV           NOT STARTED
- real-hardware integration/safety  BLOCKED（等待清急停/重启；抓取未验证）
```

## 6. 下一轮唯一推荐任务

清急停 + 重启驱动栈后：**精观测位 → ICP 叠加图 → 判断测量是否可信**；
只有叠加图确认后，才允许做"pre_grasp → gripper_init → grip → grasp"的小步测试。
