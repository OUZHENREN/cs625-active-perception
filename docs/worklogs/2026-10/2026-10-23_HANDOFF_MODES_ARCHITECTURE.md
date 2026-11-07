# 2026-10-23 HANDOFF — 六类模式架构与下一轮启动清单

## 一、当日结果

### 完成

- 装入流程真机跑通（工件从工件桌搬入插槽，无碰撞损伤）；
- 取出流程跑通到「工件已从插槽完整抽出、移动到工件桌正上方」；
- 澄清了困扰两天的碰撞机制误判（见 §三）；
- 明确了六类模式架构与三轮调试计划（本文件 §四、§五）。

### 当日结束时的现场状态

```text
TCP: 1.0309, 0.6861, 0.4398
夹爪: position=0 mm（全闭合、空夹），init=5
安全模式: 1（正常）
工件: 已由所有者手动放置回工件桌面
```

### 未完成（下次继续）

- 放置段的最后 1 cm 接近与释放（当时因控制器反复进入“接受但不执行”状态而中止）；
- 六类模式的代码落地；
- 强制环境建模脚本。

---

## 二、当日修掉的关键问题

| 问题 | 根因 | 修复 |
|---|---|---|
| 工件撞插槽桌 | 工件沿工具轴横向伸出约 0.41 m，按“竖直悬垂”直觉规划 | 先同高度横移让开插槽桌，再垂直下降 |
| 反复“原地打转” | 读取位姿用了缓存帧 | 必须等到新帧（计数增加）再返回 |
| 中止后来回抽搐 | 失败后重放旧轨迹 | 只允许重新规划，禁止重放；执行器加首点保护 0.03 rad |
| 命令接受但不动 | 多次中止后控制器失效 | `scripts/recover_control.sh`（resend + 控制器切换 + 滑块 100%）；严重时重启驱动栈 |
| 规划器“假成功” | 位姿目标容差 0.05 rad ≈ 2.9° | 收紧到 0.008 / 0.001 rad，并用 `verify_pose.py` 实测验收 |
| 夹持有异响 | 目标行程超过实际间隙 → 持续保持电流 | 夹持宽度 50 mm + 电流 0.1 A；README 与面板同步 |
| UI 图片不更新 | 只扫描 box_near / box_far 两个目录 | 改扫 `ImageSave_*/*/color.png` 取最新 |
| 夹爪面板报编码错 | PowerShell 用 GBK 输出 | UI 端多编码兜底 + 助手强制 UTF-8 输出 |

---

## 三、碰撞机制的最终结论（推翻此前误判）

```text
1. GetPlanningScene 读回的 primitive_poses / mesh_poses 一律为 0
   —— 这是“读取接口不可信”，不代表场景或 attach 失效
2. 功能测试证明内部位姿正确：
   在工件实际位置放探针 → 报 box_collision（师兄 attach）接触 0.02 m
3. MoveIt 只检查「机器人 ↔ 世界」与「机器人自碰撞」，
   不检查「世界 ↔ 世界」；因此用“世界盒子表示工件”做校验永远无碰撞，是错误方法
```

**因此：工件碰撞必须走师兄的 attach（`collision_object_publisher` + touch_links），
校验只能通过 `check_state_validity` 做，不能读场景位姿做几何判断。**

---

## 四、六类模式架构（下次落地）

### 单一入口

```bash
python3 scripts/run_task.py --scope {load|unload|full} --mode {safe|efficient} [--confirm]
```

| scope | 含义 |
|---|---|
| `load` | 抓取 + 装入 |
| `unload` | 取出 + 放置 |
| `full` | 抓取 → 装入 → 取出 → 放置 |

| mode | 含义 |
|---|---|
| `safe` | 匀速低速、分段多、每段实测验收 |
| `efficient` | 巡航快 + 末端慢；装配段仍匀速；以安全为前提提效 |

### 配置分层（六份互相独立）

```text
config/tasks/common.yaml              世界常量 + 可复用位姿（工件桌、插槽桌、中转、预插入、插入、10° 转正）
config/tasks/load_safe.yaml
config/tasks/load_efficient.yaml
config/tasks/unload_safe.yaml
config/tasks/unload_efficient.yaml
config/tasks/full_safe.yaml
config/tasks/full_efficient.yaml
```

每份包含：阶段列表、每阶段速度档案、释放授权开关、验收阈值。

### 代码级硬约束（保留并复用）

```text
装配段默认匀速低速；--allow-uneven-insert 才允许不均匀
释放夹爪必须 --allow-release（所有者当次授权）
中止后只重新规划，绝不重放
位姿读取必须取新帧
单次工具调用 ≤60 s，60 s 无位移立即排查
```

---

## 五、强制环境建模（下次第一步实现）

`scripts/model_environment.py` → `runtime/environment/<timestamp>/model.json`

```text
内容：slot_origin + quat（T1 实测）
      box_pose + quat（T3/S3 实测）
      工件桌 / 插槽桌 体素（PCD 体素化）
      实测时间戳 + 与上次基准的偏差
阈值（不通过即拒绝运动）：
      插槽偏差 > 1 mm 或 > 0.5°
      工件偏差 > 3 mm 或 > 1°
      有效点云 < 阈值
```

所有模式启动时必须先加载一份 model.json 并打印其时间戳与偏差。

---

## 六、三轮调试安排

### 第一次：可用性

- 6 类模式各 dry-run + 至少 1 次真机；
- 把已确认经验全部写进代码；
- 每次流程前强制环境建模，每阶段后实测验收。

### 第二次：效率

- 固化可复用位姿到 `common.yaml`；
- 非装配段提速，装配段保守；
- 效率模式总时间目标 ≤ 安全模式 60%。

### 第三次：验收

- 6 类模式各重复 ≥3 次，统计成功率与耗时；
- 采集数据供论文实验使用。

---

## 七、下次启动清单

```text
1. 启动驱动：real_base.launch.py（launch_driver:=true，headless）
2. 启动 MoveIt：real_moveit.launch.py（launch_rviz:=true 便于观察场景）
3. 确认 safety_mode=1、robot_mode=7、arm_controller active
4. 启动 tcp_server / collision_object_publisher / compliant_placement
5. 运行 scripts/control_health.py（应全 True）
6. 运行 scripts/model_environment.py 建立当天环境模型
7. 加载对应模式配置，dry-run 确认阶段与速度档案
8. 真机执行前确认：夹爪 init=5、工件承载状态明确、急停可达
```

---

## 八、源码保留约定

```text
师兄代码：~/cs625_legacy_underlay_jazzy（只读引用，不修改、不删除）
我方仿真代码：src/cs625_simulation（保留；旧环境配置标记不可信但保留）
诊断/历史：docs/diagnostics/**、docs/worklogs/**（保留）
归档/移动/删除任何文档需所有者书面批准
```
