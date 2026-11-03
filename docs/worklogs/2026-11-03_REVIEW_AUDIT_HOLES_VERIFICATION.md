# 2026-11-03 REVIEW 示教模式 7 条漏洞验收报告

> 对应审计：`docs/worklogs/2026-11-02_REVIEW_TEACHING_MODE_AUDIT.md`（§4 漏洞清单）。
> 本次验收：逐条找出代码位置、跑可复现证据、把"已修且已验证"与"已修待实跑"分开。
> 验收时点：2026-11-03 14:30，硬件开机，机械臂未动（仅 MoveIt 规划与只读采样）。
> 全量测试：`python3 -m pytest test/ -q` → **287 passed**。

## 0. 总览

| # | 漏洞 | 状态 | 证据强度 |
|---|---|---|---|
| 1 | S0/S2 无可达门、位姿目标会翻腕 | ✅ 已修 + 已实测 | 强（5 用例实机 plan-only） |
| 2 | 捕获无有效性/新鲜度校验 | ✅ 已修 + 已单测 | 强（14 单测 + 历史样本回放） |
| 3 | 抓取进入前无 TCP 门 | ✅ 已修，⏳ 待实跑 | 中（代码+CLI 已验，未在真机上跑过步骤 5/6） |
| 4 | 抬升/携带段无包络 | ✅ 已修 + 已实测 | 强（携带盒门实测拦截 + 工具盒通过） |
| 5 | R1 采了不用 | ✅ 已修，⏳ 待实跑 | 中（dry-run 打印实时链，实插未跑） |
| 6 | 帧间 spread 只打印不判据 | ✅ 已修 + 已单测 | 强（4 单测；真实运行未出现过超限，属预期） |
| 7 | 低危三项 | ✅ 已修，⏳ 待实跑 | 中（代码在位，需真机跑一次抓取验证） |

**结论：7 条全部修补完成；4 条已有实测/单测证据，3 条要等步骤 5–7 实跑才算闭环。**
本轮验收还**发现并修复了洞 1 的一个残留缺陷**（见 §1.2），这是审计时没料到的。

## 1. 洞 1：可达 + 无自碰撞 + 不翻腕

### 1.1 已落地

```text
scripts/teaching/plan_joint_locked.py（move() 的唯一规划入口）
  · IK 用 avoid_collisions=True（自碰撞位姿直接拒绝，exit 2）
  · 多种子搜索：当前构型 → 已验证姿态库（observation/pre_grasp/grasp/baseline）
  · 关节空间目标（不对位姿目标做求解，避免规划器自选分支）
  · 双规则构型检查：绝对角 >±180° 或单次移动 >120° → 拒绝
  · 计划时长 >40 s 或点数异常 → 拒绝
  · 新 CLI：--max-abs-joint-deg / --max-joint-delta-deg / --max-plan-seconds
```

### 1.2 验收中新发现并修复的残留缺陷

审计时的假设是"锁住 IK 解就等于锁住分支"。实机复验推翻了它：

```text
2026-11-03 复验：RVS 指定的 S2 位姿（0.7844,0.2928,0.5723）
  IK(avoid_collisions=True) 返回 J = [-204°, 7°, -92°, 191°, -326°, 152°]
  MoveIt 规划成功：1302 点 / 130.03 s —— 一条绕整个工位的路径
```

旧实现会**接受并执行**这条路径。现在被 `WRAPPED_BRANCH` 拒绝。

### 1.3 实测证据（硬件开机，plan-only，臂未动）

| 用例 | 期望 | 结果 |
|---|---|---|
| A. RVS 指定的 S2 位姿 | 拒绝 | **exit 2**，`REFUSED: wrist_3 = -208 deg past ±180 (wrapped branch)` |
| B. 今日预抓取位 | 通过 | exit 0，78 点 |
| C. baseline 停车位 | 通过 | exit 0，98 点 |
| D. 工件粗观测位 | 通过 | exit 0，154 点 |
| E. 今日抓取位 | 通过 | exit 0，84 点 |

单元测试：`test_audit_holes.py::test_sanity_rules_reject_wrapped_branch`（绕圈解必拒、合法解必过）。

## 2. 洞 2：捕获有效性 + 新鲜度

### 2.1 已落地

```text
teaching_sequence.validate_capture(record, command)
  · 位置在工作空间盒内 x∈[-0.6,1.8] y∈[-1.2,1.6] z∈[-0.9,1.6]
  · 四元数模长 ∈ [0.99, 1.01]
  · 按命令的 z 合理域（本轮按历史捕获重新标定，见下）
  · 时间戳可解析且 age ≤ 6 h（工件会移动：实测 10-30→11-02 位移 97 mm）
latest_capture() 从最新往回找第一个合法捕获，全部非法则返回 None + 原因
```

### 2.2 本轮修正：z 合理域原来太宽，等于没拦

审计后初版用的是 `T3: (-0.35, 1.0)`，而 10-30 那个坏 S3 是 `z = -0.067` —— **能通过**。
按磁盘上全部历史捕获重新标定：

```text
T1 插槽位姿    真实值 0.4391 / 0.4427；六模式参考 0.4446        → 窗口 (0.30, 0.70)
T3 工件位姿    9 帧被采纳的答案全在 0.3539–0.3558               → 窗口 (0.20, 0.60)
T0/T2 拍摄位   被采纳的 0.3353–0.6432；坏样本 -0.0666/-0.0771   → 窗口 (0.20, 1.40)
```

回放验证（当日实跑）：

```text
历史采纳样本  T0 0.6149 / T1 0.4427 / T2 0.5723 / T3 0.3542  → 全部 PASS
已知坏样本    T2 -0.0666 / T2 -0.0771 / T3 -0.067           → 全部 REJECT（z 不合理）
              T0 x=4.41                                      → REJECT（工作空间外）
```

单元测试 6 条：合法样本通过、越界拒绝、z 不合理拒绝、过期拒绝、坏四元数拒绝、缺时间戳拒绝。

## 3. 洞 3：抓取进入前的 TCP 门

```text
teaching_sequence 步骤 5b：verify_pose(TCP vs pre_grasp, 0.5 mm / 0.05°)
teaching_sequence 步骤 6b：verify_pose(TCP vs grasp,     0.5 mm / 0.05°)
verify_pose.py 新增 --pos-tol-mm / --rot-tol-deg（插入段仍用 0.1 mm / 0.1°）
```

**状态：代码在位，未在真机跑过步骤 5/6。** 依据：11-02 实测关节到位 0.0003 rad 时
TCP 仍差 10.8 mm，这个门就是为了在那 1 cm 变成撞件之前停下。

## 4. 洞 4：抬升/携带段包络

```text
SEGMENTED = {pre_grasp:4, grasp:6, lift:2, transit:2, staging:2}
CARRIED   = {transit, staging}        ← 横向搬运才加携带盒，竖直抬离不加
check_tool_envelope.py --carried：把携带工件盒并入包络再对两张实测桌子求间隙
```

设计说明（10-30 事故的直接教训）：抬离段起始时工件本来就压在桌上，几何门会永远拒绝它；
真正的风险在横向搬运（工件扫过整个工位），所以携带盒门只挂在 transit/staging。

实测证据：

```text
工具包络（不含携带）对合成 lift 轨迹：最小间隙 187 mm → ENVELOPE_CHECK_OK
携带盒门对低空横向路径：78 处违规、最小间隙 -9 mm → ENVELOPE_CHECK_FAILED（正确拦截）
```

单元测试：`test_descent_and_carried_sets` 锁住分段集合与携带集合的语义。

## 5. 洞 5：R1 采了不用

```text
teaching_sequence 步骤 12–17 改为：实时 R1 → pose_math.insert_from_slot()
  → insert_flat / insert_tilted / pre_insert 三个位姿都由 R1 现算
  · 无新鲜 T1 捕获时回退六模式常量，并打印 WARN
  · 常量与实时链的差距实测 13 mm（10-30 R1 反算 1.1701,-0.0094,0.0314
    vs 常量 1.1574,-0.0132,0.0321）
```

**状态：dry-run 已打印实时链；未实插。** 实插时才能确认"13 mm 修正"是否消除了过载。

## 6. 洞 6：帧间 spread 判据

```text
rvs_pose_client.spread_violation(command, max_mm, max_deg) → 原因或 None
SPREAD_LIMITS = {T1: (30 mm, 3°), T3: (30 mm, 3°)}      目标位姿是固定物，帧间必须一致
S0/S2 不设限（RVS 每次重算拍摄位，变化是合理的）
超限 → SPREAD_TOO_LARGE，exit 4
```

单元测试 4 条：限内通过、位移超限拒绝、姿态超限拒绝、拍摄位姿不设限。
真实运行中尚未触发（历史帧间 0.2–1.4 mm），属预期。

## 7. 洞 7：低危三项

| 项 | 状态 |
|---|---|
| 步骤 0 环境建模失败要告警 | ✅ 已加 WARN（明确"本链路不消费它，但六模式门禁会拒绝"） |
| grip 后要校验夹爪真的张开 | ✅ `grip_still_holding()`：position_mm ≥ 40 才允许 lift |
| run_teaching_chain 的 10-27 旧默认值要显式告警 | ✅ 默认改为 None，未传参即打印"旧值可能过期" |

**状态：代码在位，需真机跑一次抓取验证。**

## 8. 本轮顺带补齐的测试

```text
test/test_audit_holes.py（新增 12 条）
  · validate_capture 6 条（合法/越界/z/过期/四元数/时间戳）
  · spread_violation 4 条（限内/位移/姿态/拍摄位姿豁免）
  · 接线 2 条（move() 走锁定规划器、分段与携带集合语义）
  · sanity_violation 1 条（绕圈解必拒、合法解必过）
全量：287 passed
```

## 9. 仍然未闭环的（诚实清单）

```text
· 洞 3/5/7 只在代码层验证，缺真机步骤 5–7 实跑证据
· 洞 1 的多种子搜索依赖 PROVEN_POSTURES 四个姿态；新位姿若都失败会误拒（可接受，报错明确）
· z 合理域是"本机事后验尸"得来的窗口，换工位/换工件必须重新标定（注释已写）
· 洞 4 的携带盒是静态近似（工件相对 TCP 的标称变换），未验证"夹持松脱"
```

## 10. 复现步骤

```bash
cd /home/oy/projects/cs625-active-perception
source scripts/source_dev_env.sh --full
python3 -m pytest test/ -q                                   # 287 passed

# 洞1：不可站位姿必须被拒
python3 scripts/teaching/plan_joint_locked.py 0.7844 0.2928 0.5723 \
  -0.5935 0.6472 0.2667 0.3971 --velocity 0.05 --output /tmp/x.json   # exit 2

# 洞4：携带盒门必须拦截低空横向路径
python3 scripts/check_tool_envelope.py /tmp/<plan>.json --sample 2 --carried
```
