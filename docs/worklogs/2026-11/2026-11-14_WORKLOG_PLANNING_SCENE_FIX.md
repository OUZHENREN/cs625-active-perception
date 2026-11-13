# 2026-11-14 工作日志：修复"规划场景里看不到夹具"（发布机制）

> 背景：2026-11-13 交接文档 §6 记录"`apply_real_cell_scene.py` 报告发布成功，
> 但 `/get_planning_scene` 查回来仍是 0 个对象"——这是自动换位与方案 A 真机验收的
> 共同前置阻塞。本轮**没有真机**，也没有启动 Gazebo；只做离线可验证的部分。

## 1. 问题定位（静态分析）

```text
现场症状：脚本打印 "published 1 collision objects"，MoveIt 场景里 0 个对象。

对比两个场景写入脚本，差别就在这里：
  · 仿真侧 apply_task_scene.py（跑得通）：调用 /apply_planning_scene **服务**，
    并检查返回的 success；
  · 真机侧 apply_real_cell_scene.py（不生效）：往 /planning_scene **话题**裸发 20 次，
    然后打印 "published N"——**从不确认 move_group 是否收到**。

⇒ 只要 MoveIt 没在跑、或订阅端没匹配上，脚本照样打印成功。这就是"发布成功"与
  "场景 0 个对象"能同时成立的唯一解释：它报的是"我发出去了"，不是"对方收下了"。
```

## 2. 修复（最小改动，沿用仓库已跑通的机制）

`scripts/apply_real_cell_scene.py`：

```text
① 写场景改走 /apply_planning_scene 服务（与 apply_task_scene.py 同一机制），
   检查返回 success；服务不在 → 打印明确错误并 exit 1（不再假装成功）。
② 写入后回查 /get_planning_scene，components 取
   WORLD_OBJECT_NAMES | WORLD_OBJECT_GEOMETRY（8|16），核对每个预期 id 真的在场景里；
   缺任何一个 → 打印"apply 报成功但场景里没有这些 id"并 exit 1。
③ --remove 同样走服务，并核对 id 已消失。
④ 抽出纯函数 build_scene()（便于离线测试），保留 --dry-run。
```

数据流（改后）：`config/cs625_real_cell_scene.yaml → body_specs() → PlanningScene(diff)
→ /apply_planning_scene（返回 success）→ /get_planning_scene 回查 → 打印实际对象列表`。

## 3. 验证（离线）

```bash
source scripts/source_dev_env.sh --full

python3 scripts/apply_real_cell_scene.py --dry-run     # exit 0，打印夹具 + 桌高交叉校验
python3 scripts/apply_real_cell_scene.py               # 无 move_group → exit 1 + 明确报错
python3 -m pytest test/test_real_cell_scene_applier.py -q   # 4 passed
python3 -m pytest test/ -q                             # 279 passed（275 + 4 新增）
python3 scripts/check_docs.py                          # ALL DOC CHECKS PASSED
python3 test/contract_checks.py                        # PASS
```

无 move_group 时的实际输出（证明不再假成功）：

```text
[ERROR] [apply_real_cell_scene]: /apply_planning_scene is not available --
is move_group running? Nothing was applied (the old topic publish would have
printed 'published' anyway, which is what made this look successful).
exit=1
```

新增测试 `test/test_real_cell_scene_applier.py`（4 条，纯离线）：
盒子对象 id/帧/尺寸、REMOVE 操作不带几何、真实配置里夹具存在且帧为 world、
默认 CLI 只发夹具（桌子/工件按需）。

## 4. 未验证（如实）

- **没有在 move_group 运行时端到端跑过**：真机不可用，仿真需要 Gazebo（本轮未启动）。
  服务调用与回查路径的"真跑到"属于仿真/真机验收范围，测试里没有伪装。
- 默认仍**只发夹具**（桌子需要 `--include-tables`）：桌子盒与实测桌高有偏差（见 §5），
  在修正之前默认发表面盒会让规划出现假的碰撞。

## 5. 附带发现（本轮不修，需所有者决定）

`--dry-run` 的桌高交叉校验暴露：**生成的世界与场景配置落后于最新实测模型**。

```text
config/cs625_real_cell_scene.yaml（= worlds/cs625_real_cell.sdf 里的值）
  workpiece_table top_z −0.10306，footprint 0.69 × 0.60
  slot_table      top_z −0.10622，footprint 0.69 × 0.85
runtime/environment/20261105_134756/model.json（最新实测）
  workpiece_table z −0.09097，footprint 0.92 × 0.80
  slot_table      z −0.04070

⇒ 顶面差 12.1 mm（工件桌）与 65.5 mm（插槽桌），桌面尺寸也不同。
```

两者是**生成物与生成源不一致**：世界/配置应当由 `scripts/build_sim_world.py` 从
最新 `model.json` 重新生成，而不是手改。这属于"仿真必须与真机 1:1"的范畴，
改它会同时改变仿真场景，需要所有者确认后再做（本轮按"一轮一个问题"未动）。

## 6. 能力层与状态

```text
Capability layer: vision/planning 输入链 —— "点云/环境 → MoveIt 规划场景" 这一环
  环境与依赖      PASS（离线：279 测试 + 合同 + 文档检查）
  规划场景写入    PASS（机制修复 + 离线验证）；运行时端到端 **NOT ACCEPTED**（未启动 MoveIt）
  真机            NOT RUN（无真机运动、未启动 Gazebo/相机）
Intentionally not done:
  · 未在 move_group 运行时验证写入（需要仿真或真机）；
  · 未重新生成世界/场景配置（§5，需所有者确认）；
  · 未把桌子纳入默认发布（等 §5 修正）。
```

## 7. 复现命令

```bash
source scripts/source_dev_env.sh --full
python3 scripts/apply_real_cell_scene.py --dry-run
python3 -m pytest test/test_real_cell_scene_applier.py -q
# 有 MoveIt 时（仿真或真机）：
python3 scripts/apply_real_cell_scene.py        # 应打印 "verified in the scene (world objects now: [...])"
```
