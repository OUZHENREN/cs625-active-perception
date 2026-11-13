# 任务位姿推导（抓取 / 预插入 / 插入 / 转正）

> 建立：2026-11-10；**2026-11-10 修订**：补上示教模式实际使用的三处修改
> （所有者指出）。实现：`scripts/teaching/pose_math.py`
> （`derive_task_poses` = 师兄原始链条；`teaching_task_poses` = 现在真正跑的链条）。

## 1. 两条链：师兄原始 vs 示教模式实际

```text
【师兄原始链】（配置里那批常量的来源，保留作参考）
  grasp      = part * T_b_tcp          T_b_tcp = diag(-1,1,-1) + (-4, -215, -411) mm
  pre_grasp  = grasp * T(+120 mm, 工具 x)
  insert     = slot * T_s_tcp          T_s_tcp = diag(-1,1,-1) + (-82, -193, -412.5) mm
  rotated    = insert * Ry(-10°)
  pre_insert = rotated * T(-350 mm, 工具 z)
  lock       = rotated * Ry(+10°)

【示教模式实际链】（2026-11-03/04/05 所有者改的，现在真机跑的是这条）
  grasp      = snap_vertical(part * T_b_tcp) 后，沿【工具 x】再回退 10 mm
  pre_grasp  = grasp + (0, 0, +150 mm)          ← 正上方，不是横向 120 mm
  insert     = slot * T_s_tcp
  rotated    = insert * Ry(-10°)
  pre_insert = rotated * T(-350 mm, 工具 z) + (0, 0, +100 mm)
  lock       = rotated * Ry(+10°)
```

三处修改的原因（都在 `teaching_sequence.py` 的注释里）：

```text
① snap_vertical：RVS 给的工件姿态约 0.8° 不垂直，夹爪的电机保护座会蹭到工件凹槽；
   压成"工具 z 指向世界 −z、保留实测 yaw"（Rz(yaw)·Rx(180°)）。
② 正上方 150 mm：工具本体宽 294.6 mm，师兄的横向 pre_grasp 在工件 yaw 变化时会撞夹具框；
   垂直下降实测 0.199 mm / 0.031°，无保护停止。150 mm 是 2026-11-04 从 120 mm 加上去的。
③ 回退 10 mm：电机保护座与工件外表面凹槽干涉（真实碰撞后 1.5→3→6→10 mm 逐步加到 10）。
④ 预插入再抬高 100 mm：350 mm 后退后工具太低、看不到槽口；**插入终点不变，只是下降起点更高**。
```

## 2. 数值（用校验过的插槽位姿 + 历史工件位姿算出）

输入：`part = [0.6534, 0.7035, 0.3551] / [0.0075,-0.0018,0.4835,0.8753]`；
`slot = teaching_slot_reference = [1.219397, 0.138884, 0.444411]`。

| 位姿 | TCP (m) | 姿态 (x,y,z,w) |
|---|---|---|
| grasp（示教） | [0.8369, 0.6002, −0.0583] | [0.4835, −0.8753, 0, 0]（工具 z 严格朝下） |
| pre_grasp（示教） | [0.8369, 0.6002, **0.0917**] | 同上（正上方 +150 mm） |
| insert（公式终点） | [1.1352, −0.0560, 0.0332] | [−0.0034, 1.0, −0.0016, −0.0043] |
| rotated（倾斜进槽） | 同上 | [−0.0035, 0.9965, −0.0013, 0.0829]（Ry −10°） |
| pre_insert（示教） | [1.0774, −0.0553, **0.4784**] | 同上 |
| lock（转正） | 插到为止的位置 | flat：Ry +10°，位置不变 |

**插入段长度变了**：

```text
pre_insert → insert(公式)       = 448.9 mm
pre_insert → insert_end(实测座) = 412.3 mm      ← 实际下降行程
（师兄原链是 350 mm 后退、没有额外 100 mm，所以旧的 350 mm 不能再用）
```

插入终点必须用**实测座** `teaching_insert_end.tcp = [1.1471, −0.0551, 0.0721]`：
公式终点比它深 39.7 mm（371–372 mm 处三面接触、保护停止，三次重复性 1.1 mm）。

## 3. 插槽位姿：RVS 的原始 R1 不能直接用

```text
RVS 的插槽位姿跨会话漂移 30–50 mm（2026-11-03 实测：用 74 小时前的 R1 推出 insert y = −0.0094，
用当天 R1 推出 −0.0447，差 35 mm；现场观察"往左偏 3–5 cm"与之一致）。
→ 插入链的输入必须是【校验过的插槽位姿】：
   runtime/slot_reference_verified.json（优先）或 config/tasks/common.yaml 的
   teaching_slot_reference；RVS 的原始 R1 只作参考，永不作权威。
   （common.yaml 里那条 45.5 mm 横向修正就是施加在"插槽参考位姿"上的。）
```

## 4. 怎么用

```python
from scripts.teaching.pose_math import teaching_task_poses, derive_task_poses

# 现在真机跑的那条链（含垂直修正/150 mm/10 mm/100 mm）
poses = teaching_task_poses(part_xyz, part_quat, slot_xyz_verified, slot_quat)
# 师兄原始链（复算历史常量用）
legacy = derive_task_poses(part_xyz, part_quat, slot_xyz, slot_quat)
```

## 5. 验证与限制

```text
· 师兄原始链复现配置常量：grasp 0.06 mm / pre_grasp 0.13 mm / insert 0.04 mm / pre_insert 0.04 mm
· 示教链的四个值见上表；lock 的姿态与 teaching_insert_end.flat_quat 差 0.02°（对实测座施加）
· 需要 base 系下的 part/slot 位姿：手眼链已于 **2026-11-10 验明**（`T_base_cam = T_base_tcp @ T_tcp_cam`，
  ≤3 mm，见 `pose_conventions.md` §4.1）；`scripts/measure_object.py` 自 2026-11-12 起直接输出
  base 系位姿（`pose_base` / `pose_base_centre_mm`），`scripts/grasp_from_measurement.py` 已接；
  插入段仍按 ADR-005 用已验证插槽位姿，不用实时测量做插入权威。
```
