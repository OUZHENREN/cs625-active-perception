# 2026-10-23 WORKLOG — 精观测位算错与修正（真机运动，含一次错误移动）

> 起因：所有者质疑「你确定这是前往精观测位？如何计算得出的，和 RVS 端的位姿比对过吗？」
> 结论：**质疑成立**。第一次"精观测位"把相机从 0.52 m 推到 1.25 m、画面从 52% 掉到 8%，
> 是反方向；已修正公式、重新规划、重新执行，现在相机到目标 0.444 m、占画面 69%。
> 本轮有真实机械臂运动（两次）。

---

## 1. 本轮解决了什么

```text
① 精观测位的算法错误（三个错误叠加）
② 规划接口依赖未入库的 /tmp/plan_pose.py（现已入库 scripts/plan_pose_once.py）
③ 观测位误用装配级公差导致规划 99999
④ 精阶段 ICP 仍走 TF 双树，结果落在 (-0.37,-0.92,0.38)；改走常量链后回到工件附近
```

---

## 2. 第一次移动（错误，已发生）

```text
粗阶段：掩膜中心 u=330 v=277 + 最近深度峰 z=0.52 m → base 目标点 (0.873, 0.639, 0.186)
精阶段：fine_TCP = target − unit(target − cam) × 1.0   → (0.324, 0.098, 0.823)
        姿态：脚本里写死的 S2 四元数
规划：  code 1，451 点，44.9 s（速度缩放 0.02）
执行：  4 段全部 ALL_ASSEMBLY_CHUNKS_OK，TCP 到位 (0.3236, 0.0980, 0.8228)
实测：  相机到目标 1.251 m，工件占画面 7.9%
```

### 错在哪（三个错误叠加）

```text
1. standoff 语义反了
   本意"相机到目标 1.0 m"，实现成"TCP 到目标 1.0 m"；
   相机在 TCP 后方 296 mm → 实际相机距离 1.0 + 0.255 ≈ 1.25 m（实测 1.251 m 吻合）
2. 方向反了
   移动前相机已在 0.52 m，脚本把它推远到 1.25 m；面积 ∝ 1/d²，画面 52% → 7.9%
3. 姿态没算
   直接用了写死的 S2 四元数，不是从目标推出来的
```

### 与 RVS 的比对（第一次没有做，这是错的）

```text
RVS S2（粗观测位）  相机到工件 0.652 m
RVS S3（抓取位）    相机到工件 0.296 m
→ 精观测位应落在 0.40–0.45 m
→ 我们跑到的 1.25 m 比 RVS 的【粗】观测位还远一倍，不可能是精观测位
```

**教训**：`check_state_validity` + 规划成功只证明"可达、不碰撞"，**不证明位姿正确**。
位姿正确性必须和参考量级（RVS 的 S2/S3、画面占比）比对。

---

## 3. 修正后的算法

```text
T_base_cam = T_base_tcp × T_tcp_cam          ← 相机的真实位姿
axis       = unit(target − cam_pos)
standoff   = 【相机】到目标的距离（不是 TCP 到目标）

desired_cam.rotation = 当前相机姿态（光轴不变）
desired_cam.position = target − axis × standoff
desired_TCP          = desired_cam @ inv(T_tcp_cam)

护栏：standoff ≥ 当前相机距离 → 拒绝执行（除非 --allow-back-off）
     打印预计画面占比 area_now × (d_now / standoff)²
```

---

## 4. 规划接口的两个问题

```text
问题 A：依赖 /tmp/plan_pose.py（未入库），重启即失效
  → 新增 scripts/plan_pose_once.py（入库、可参数化、plan only）
问题 B：观测位用了装配级公差 0.5 mm 球 + 0.001 rad
  → 同样的位姿：0.5mm/0.001rad 规划失败（code 99999）；5mm/0.02rad 成功
  → 观测位只需把相机放到位，下一级会重新解算目标，5 mm / 1.1° 足够
  → OMPL 随机性：5% 速度 + 20 次尝试 + 25 s 预算才稳定成功；脚本加了 3 次重试
```

---

## 5. 第二次移动（正确）与验收

```bash
python3 scripts/coarse_to_fine_pose.py --weights ... --template ... --plan-only   # 先只规划
python3 scripts/coarse_to_fine_pose.py --weights ... --template ... --confirm     # 再执行
```

```text
粗阶段：目标点 (0.864, 0.674, 0.204)，当前相机距离 1.26 m（8%）
精阶段：standoff 0.45 m → fine TCP (0.808, 0.599, 0.414)，预计 66%
规划：  attempt 1/3 code 1，205 点，20.3 s
执行：  4 段全部 ALL_ASSEMBLY_CHUNKS_OK
到位：  TCP (0.8094, 0.6031, 0.4133)
```

| 指标 | 第一次（错） | 第二次（修正后） | 目标 |
|---|---|---|---|
| 相机到目标 | 1.251 m | **0.444 m** | 0.40–0.45 m |
| 工件占画面 | 7.9% | **69%** | 60–80% |
| 到位位置误差 | — | 4.4 mm | ≤ 5 mm |
| 到位姿态误差 | — | 1.5° | ≤ 1.15°（0.02 rad）→ **FAIL** |

姿态超差 1.5°：这是观测位不是对准位，影响是光轴偏 1.5°（0.45 m 处约 12 mm 视场偏移），
对"拍清楚"无影响；但按 §5 铁律 8，仍如实记为 FAIL，要收紧就得把规划姿态公差降到 0.005 rad
并重新规划执行。

### 跨位姿一致性（额外收获）

同一外参链在两个差别很大的臂位姿下各做一次已知靶点标定：

```text
位姿 A（相机 0.586,0.356,0.518）: 桌面倾角 0.8°，桌面高度 -0.103 m，质心 (0.935,0.736,0.218)
位姿 B（相机 0.594,0.402,0.436）: 桌面倾角 0.7°，桌面高度 -0.101 m，质心 (0.938,0.725,0.212)
→ 桌面法向/高度一致，质心差约 12 mm：外参（旋转+平移）跨位姿自洽
```

---

## 6. 精阶段 ICP 的现状（未通过）

```text
修前（走 TF）       xyz=[-0.3722, -0.9172, 0.3757]  fitness 0.358
修后（走常量链）     xyz=[ 0.9568,  0.6581, 0.1814]  fitness 0.190  rmse 12.5 mm
粗阶段目标点         (0.864,  0.674, 0.204)
```

位置已经回到工件附近（差值约 0.10 m），但 **fitness 0.190 < 0.5 的门槛，不算通过**。
最可能的原因仍是 §11.8 的掩膜粘连：现在工件占画面 69%，掩膜把桌面也吃进去，
点云里混了平面点，ICP 被拉偏。下一步应先用"桌面平面剔除"给 ICP 喂干净点云，
再谈是否要补样本重训。

---

## 7. 速度口径（所有者当场纠正）

```text
所有者原话："正常来说，刚刚的速度是正常速度"
含义：move_group 速度缩放 0.05 就是这台 CS625 真机上的【正常工作速度】
      · 20–50 s 的轨迹时长是常态，不是异常，也不需要"提速"
      · 装配段才降到 0.005–0.01
已写进 scripts/plan_pose_once.py 与 scripts/coarse_to_fine_pose.py 的帮助文本
```

---

## 8. 改动文件

```text
改写  scripts/coarse_to_fine_pose.py   standoff=相机距离、反算 TCP、方向护栏、
                                      入库规划脚本、观测级公差、重试、--plan-only、速度口径
新增  scripts/plan_pose_once.py        入库的 plan-only 规划器（可配公差/速度/预算）
改写  scripts/icp_pose_once.py         精阶段 ICP 从 TF 改为 TCP × T_tcp_cam 常量链
文档  docs/agent_memory.md             §113 精观测位算法与速度口径
```

---

## 9. Capability layer

```text
环境与依赖        PASS    相机 640×480 彩色+深度正常，深度单位为毫米
机器人模型与仿真  不涉及
运动学/控制/规划  PASS    真机移动两次；第二次按修正公式到位 4.4 mm / 1.5°
视觉/手眼/TF      PASS    已知靶点标定在两个臂位姿下均 PASS；TF 仍绕开
主动感知 / NBV     不涉及   精观测位的"到达"闭环已通，但位姿质量未达标
真机集成与安全    PASS    safety_mode=1、robot_mode=7、控制器 active；速度 5%
                         两次移动均分段执行、全部返回 ALL_ASSEMBLY_CHUNKS_OK
```

### Critical-chain status

```text
URDF → Gazebo entity              NOT RUN
ros2_control → joint_states       PASS
base_link → camera optical TF      BLOCKED（TF 双树仍在；常量链绕开）
RGB-D → normalized sensor topics   PASS
point cloud → MoveIt planning scene NOT RUN（本轮未更新场景）
NBV decision → robot execution     PARTIAL（观测位到达已跑通；ICP 质量未达 fitness>0.5）
```

---

## 10. Intentionally not done

```text
· 没有收紧姿态公差重跑（1.5° 对观测位可接受，但对准类动作必须收紧）
· 没有做掩膜去粘连（下一优先级）；精阶段 ICP fitness 0.190 未达 0.5 门槛
· 没有更新 MoveIt 场景里的桌面/工件（attached box_collision 为旧状态，规划偏保守）
· 没有做六类模式与三轮调试
```

---

## 11. 下一步唯一推荐任务

**给 ICP 喂干净点云**：在 `scripts/icp_pose_once.py` 里先拟合桌面平面、剔除桌面点
（`known_target_calibration.py` 里已有同一套 RANSAC 平面拟合与剔除逻辑），再跑 ICP，
目标是 fitness > 0.5 且 inlier RMSE < 5 mm。这一步是"精采样闭环"真正成立的判据。
