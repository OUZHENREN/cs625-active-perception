# 2026-10-22 WORKLOG — 观测到抓取的可复现流程（体素碰撞版）

## 一、这次成功的完整链路

```text
观测位
→ T2 得 S2
→ T3 得 S3
→ 由 box target × T_b_tcp × (+120 mm x) 算 pre-grasp
→ pre-grasp plan-only（IK + 关节目标）
→ 慢速分段到 pre-grasp
→ Cartesian 小段校正到精确 pre-grasp
→ 在 pre-grasp 初始化夹爪（init=5，0 mm 收回）
→ 慢速分段进给到 grasp
→ grasp 位停下核对
```

## 二、各步骤关键点

### 1. 环境碰撞不能用草稿大框

错误做法：手画两张桌子的大包围盒。

后果：

```text
cartesian fraction 0.708（被误挡）
接触对：workpiece_table <-> camera_link depth 0.0265
点云核对：抓取区域 x 0.78–0.88, y 0.54–0.64 根本没有 z≈-0.10 的桌面点
```

正确做法：

```bash
python3 scripts/build_environment_collision.py \
    --pcd <merged.pcd> --z-min -0.13 --z-max -0.07 --voxel 0.05 --apply
```

- 用合并点云的桌面高度带生成体素框，命名 `env_voxel_*`；
- 移除 `workpiece_table` / `slot_table` 草稿框；
- 保留 `box_collision` / `slot_collision`。

效果：

```text
fraction 0.708 → 0.958
grasp 处只剩 box_collision ↔ my_end_effector_link depth 0.001（预期接触）
```

### 2. 用接触对判断阻挡，不靠猜

```bash
/check_state_validity
```

返回 `contact_body_1 <-> contact_body_2`，直接指出谁撞谁。

判断规则：

- `box_collision` 与末端在 grasp 处 < 1 mm：预期抓取接触，可接受；
- 其它 link 与桌面/插槽相交：场景或姿态有问题，不可强行执行。

### 3. pre-grasp 与 grasp 的位姿算法

```text
grasp     = T_box_target × T_b_tcp
             T_b_tcp = diag(-1, 1, -1)，平移 (-4, -215, -411) mm
pre-grasp = grasp × (+120 mm, x)
```

不能用 box target 的朝向直接当 pre-grasp 朝向。

### 4. 夹爪必须在 pre-grasp 初始化

```text
close → 0 只是驱动零位，会留机械间隙（略长于两扶手间距）
init  → 真正零位（init 1 → 5，位置 -2.4 → 0）
```

顺序：到 pre-grasp → init → 确认 init=5、position=0 → 再进给。

### 5. 分段执行要有停顿

```bash
python3 scripts/execute_assembly_chunks.py '<chunks>' --time-scale 2.5 --dwell 1.0
```

每段检查 `status=4`，失败立即停。

## 三、本次实际数据

```text
S2: 0.617876, 0.353517, 0.573330
S3: 0.653422, 0.703592, 0.355183

pre-grasp 目标: 0.7677, 0.4903, -0.0595
pre-grasp 实际: 0.768, 0.490, -0.059

grasp 目标: 0.8316, 0.5918, -0.0583
grasp 实际: 0.829, 0.588, -0.058

夹爪: init=5, 0 mm 收回
```

## 四、所有者反馈：进给速度仍需降低

本次 grasp 进给使用 `time_scale=2.5`，所有者判定“还是偏快，至少要再慢一半”。

新规则：

```text
抓取/插入/取出：time_scale ≥ 5.0（相比 2.5 再慢一倍）
规划速度：max_velocity_scaling_factor ≤ 0.025
段间停顿：≥ 1.5 s
每段后读 feedback/TCP，超差即停
```

## 五、本次未做的事

- 没有张开夹爪夹紧工件；
- 没有抬升、没有中转、没有插入；
- 抓取成功与否尚未验证。

## 六、复现清单

```text
1. 每日完整 E1 环境扫描，合并点云
2. 体素生成环境碰撞体，移除草稿框
3. T2 → S2，T3 → S3
4. 算 pre-grasp / grasp（T_b_tcp + 120 mm）
5. pre-grasp plan-only 检查
6. time_scale ≥ 5.0 慢速分段到 pre-grasp
7. Cartesian 小段校正
8. pre-grasp 初始化夹爪
9. time_scale ≥ 5.0 慢速分段进给到 grasp
10. grasp 位停下核对
```
