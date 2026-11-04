# 2026-11-04 WORKLOG 示教链路：竖直接近、断点续跑、触发判据修正

> 前序：`docs/worklogs/2026-11-03_WORKLOG_OBSERVATION_BASELINES_AND_ACCURACY.md`
> 真机：✅ 真实机械臂参与；✅ 真实相机 + RVS；✅ 发生真实运动

## 1. 今天修掉的三处问题

### 1.1 预抓取位一直是"水平进入"，从未改成竖直（所有者指出）

```text
现象：所有者要求"从上方进入"，但链路第 5 步仍走行为水平方向
真因：pose_math.grasp_from_box() 里 pre_grasp = grasp × translate(+120 mm 沿工具 x)
      —— 这是师兄的原始定义，我 2026-11-03 只用【手工】做过竖直下降，
      从未把它写进 teaching_sequence
后果：工件 yaw 与 10-27 不同时（今天 94.7° vs 63.2°），水平进入会撞夹具框架
修复：teaching_sequence 第 5 步改为 grasp 位 + APPROACH_HEIGHT_M(0.120 m) 的
      世界 z 方向抬升，第 6 步纯竖直下降到抓取位
      依据：2026-11-03 实测 竖直下降 0.199 mm / 0.031°、无保护停止
```

### 1.2 粗观测触发判据自相矛盾（所有者指出"明明识别到了却还在挪")

```text
现象：基准 A 已返回有效 S2，链路却判失败并换到基准 B，最后暂停
真因：trigger_once 同时要求 --min-frames 3 与 --max-attempts 1
      一次触发只产生 1 帧，永远凑不满 3 帧 → 每次都被判失败
修复：trigger_once 改为 min_frames=1
      理由：T2 返回的是 S2【取景位姿】，只用于决定下一步站哪，1 帧足够；
            多帧一致性仍只用于 T3（真正的工件位姿）
```

### 1.3 从新基准 A 规划到 slot_coarse 被绕圈守卫拒绝

```text
现象：第 1 步 plan refused / WRAPPED_PATH elbow travels 230 deg
真因：以新 A 的关节角为种子时，slot_coarse 位姿的 IK 收敛到翻转肘分支
      （肘 +111° 而不是 −111°）
修复：把 2026-11-03 验证过的 slot_coarse 构型
      [-21.1, -72.7, -111.0, -37.5, 128.5, 62.7] 加入 PROVEN_POSTURES 种子库
```

## 2. 新增：断点续跑（--start-from）

```text
原因：每次暂停都从第 1 步重跑整条链，浪费时间且反复移动机械臂
实现：python3 scripts/teaching/teaching_sequence.py --confirm --start-from N
  N=1  完整流程（默认）
  N=3  从【工件粗观测】开始（跳过插槽观测）
  N=5  从【预抓取接近】开始（复用磁盘上最新的 T3 捕获）
状态来源：跳过步骤所需数据全部从 runtime/rvs_teaching 的最新捕获读取，
          不依赖内存状态，所以进程重启后仍可用
未提供：10 及以后（转运/插入）需要工件已在夹爪中，避免"以为抓着其实没抓"的错误续跑
验证：--start-from 1/3/5 三种 dry-run 均通过；293 测试通过
```

## 3. 今日链路执行轨迹（用于下次对照）

```text
11:37 启动 → [1] slot_coarse OK → [2] T1 OK (R1 = 1.2466, 0.1489, 0.4430)
      → [3] 新基准 A 首次 T2 即识别成功（旧 A 昨天连续 3 次失败）
         期间校验器正确剔除两个 z 异常的错特征帧（z=0.103 / z=-0.003）
      → [4] 工件精观测 OK
      → [5] 移动预抓取位（当时仍是水平进入）→ [5b] 位姿门 4.92 mm > 1.5 mm → 中止
```

**5b 门失败的解读**：不是门限太严，而是当时用的预抓取位（水平进入）与
新算出的抓取位姿态不一致，导致 TCP 差 4.92 mm。改成竖直接近后，
预抓取位与抓取位只差 z 方向 120 mm，5b 门预期能通过。

## 4. 下一步

```text
用 --start-from 5 接着跑：预抓取(竖直) → 5b 门 → 进入抓取 → 抓取 → 抬升
                        → 转运 → 预插入 → 13b 门 → 插入
```


## 5. 重要更正：−10° 插入倾斜是必需策略（2026-11-04 13:40）

**我的错误**：为了让插入姿态"完全来自今天的 R1"，我把师兄的 −10° 入口倾斜去掉了
（`SLOT_TILT_DEG = 0`），并按平姿态下降 —— 结果工件与槽口发生轻微碰撞（所有者当场制止）。

**师兄代码复验**（`~/cs625_legacy_underlay_jazzy/src/tcp_bridge/tcp_bridge/tcp_server.py`）：

```python
compute_slot_insert_tcp:            T_world_tcp = T_world_slot @ T_s_tcp
compute_slot_pre_insert_rotated_tcp: T_rot = R.from_euler('y', -10.0, degrees=True)
                                    T_world_rotated = T_world_insert @ T_rot    # 只改姿态
compute_slot_pre_insert_tcp:        T_trans[2,3] = -350/1000
                                    T_world_pre = T_world_rotated @ T_trans
```

逐位核对我们与他的常量：

| 项 | 师兄 | 我们 | 一致 |
|---|---|---|---|
| T_s_tcp | diag(−1,1,−1)，t=(−82, −193, −412.5) mm | 同 | ✅ |
| 入口倾斜 | Ry(−10°) | TILT_DEG = −10.0 | ✅ |
| 预插入后撤 | 沿旋转后 z −350 mm | PRE_INSERT_BACKOFF_M = 0.35 | ✅ |

**正确动作顺序（师兄策略）**：

```text
① pre_insert（倾斜 −10°）        ← 已经带倾斜地站在槽口上方
② insert（倾斜 −10°）            ← 带倾斜插到底
③ straighten（转平）             ← 插入完成后再转平，才到正式放置姿态
④ release
```

**结论**：倾斜属于"进入策略"，不是"姿态误差"；R1 只提供插槽位姿，
倾斜必须叠加在 R1 之上。已恢复 `SLOT_TILT_DEG = -10.0` 并在代码注释里引用师兄原文。

**当前链条（今天的 R1）**：

```text
insert(终点,平的) (1.1601, −0.0451, 0.0319)  quat (0.0048, 1.0, −0.0022, −0.0032)
rotated(插入姿态) (1.1601, −0.0451, 0.0319)  quat (0.0046, 0.9965, −0.0026, 0.0839)
pre_insert        (1.1016, −0.0430, 0.3770)  quat 同上
```
