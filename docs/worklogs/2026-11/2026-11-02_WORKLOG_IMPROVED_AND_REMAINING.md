# 2026-11-02 WORKLOG 已改进 / 未改进清单（示教模式与三大专项）

> 盘点对象：示教链路（scripts/teaching/ 及共用脚本）、真机描述、环境碰撞（专项1）、
> 携带工件建模（专项2）、力控装配（专项3，方案期）。
> 盘点时点：2026-11-02 深夜，全部硬件已关停，WSL 无残留进程。

## 一、已改进（按模块）

### 示教链路

| 模块 | 改进 | 验证状态 |
|---|---|---|
| rvs_pose_client.py | 仓库根路径修正；多帧触发（≥3 有效帧 ≤10 次）；工作空间+四元数有效性过滤；帧间 spread 打印；T1/T3 spread 上限 30 mm/3°；单帧超时 300 s | 多日实机验证 |
| teaching_sequence.py | 固定/动态规则落地（S0/S2/S3 动态、粗采样固定）；捕获校验+6 h 新鲜度；5b/6b TCP 门（0.5 mm/0.05°）；插入链改实时 R1；SEGMENTED 分段下降（t=0 重计时）；包络门接入；携带盒门（transit/staging）；4b 环境场景；7b attach / 15b detach+place；grip 后校验；步骤 0 告警；插槽历史对比 | dry-run + 部分实机 |
| plan_joint_locked.py（新） | IK(无碰撞)+当前构型种子+关节空间规划：锁分支、拒绝不可达位姿 | 实机：不可达 S2 被拒、grasp/lift/pre_grasp 规划通过 |
| verify_pose.py | 容差参数化（--pos-tol-mm/--rot-tol-deg） | 单元级验证 |
| run_teaching_chain.py | 默认 grasp/s3-quat 改为 None + 10-27 旧值告警 | — |
| execute_assembly_chunks.py | --time-scale < 1.0 拒绝守护（事故修复） | 回归测试 8/8 |
| check_tool_envelope.py（新） | 工具包络（SolidWorks 刚体含保护座，两网格并集）；--carried 携带盒；跳过出发点；carried-margin | 实机：lift 通过、横向低空路径被拦 |

### 真机描述 / 模型

| 改进 | 验证 |
|---|---|
| cs625_parallel_gripper.xacro：77 mm 手指占位碰撞移除（保留视觉）；Gazebo ros2_control 块加 sim 门控（修复真机驱动段错误——10-26 弃用 wrapper 的真正原因） | xacro 真机展开 GazeboSimSystem=0、仿真展开=1 |
| cs625_active_perception.urdf.xacro：调用处传 sim 标志 | 同上 |
| 真机描述切回 wrapper（实测相机 + SolidWorks 工具） | 驱动存活、四项门全绿 |
| config/tasks/common.yaml：工具几何基准（法兰→TCP 353、实体 207、半宽 147.3、保护座归属、安全注记） | 与三份来源交叉一致 |

### 专项1 环境碰撞

| 改进 | 验证 |
|---|---|
| apply_environment_collision.py（新）：model.json 1476 个 2.5-D 桌面体素盒 → 规划场景；夹具/工件减除区（15 mm XY padding）；--dry-run/--remove/--keep-workpiece | 实机发布成功、往返清空 |
| test/test_apply_environment_collision.py：5 个纯逻辑测试 | 5/5 |
| 验收：wrapper 模型 + 全场景下 pre_grasp/grasp/lift 规划全绿，阴性对照被拦（元凶=厂商 camera_link 占位盒） | 实机 plan-only |

### 专项2 携带工件建模

| 改进 | 验证 |
|---|---|
| attach_workpiece.py（新）：attach（inv(T_b_tcp)）/ detach+place / reset / dry-run | 实机往返：attach→attached=1；detach+place→world=[placed]；reset→双空 |
| 修复两个 MoveIt 陷阱：robot_state.is_diff（detach 假成功）、不存在 id 的 REMOVE（diff 被拒）、中断残留双清 | 实机 |
| test/test_attach_workpiece.py：3 个纯逻辑测试 | 3/3 |
| 示教序列接入：7b attach（失败 WARN 有离线门兜底）、15b detach+place | dry-run |

### 专项3 力控装配

| 改进 | 验证 |
|---|---|
| 通读师兄源码并出两期方案（A：关节力矩接触检测，无需传感器；B：F/T 升级） | 方案文档已归档（未实施） |

### 运维与诊断知识（写进 worklog 的教训）

半死驱动诊断（56 万行报错→重启）；反控通道静默丢失 → resend_external_script 规程；
time-scale 语义；TCP 迭代微调；"同帧采样才可对比"；桌面/工具几何基准。

## 二、未改进 / 遗留（诚实清单）

### A. 执行验证层面

1. 步骤 6/7/8–11/12–18 **仍未实跑**（代码+防护完备但零实测；插入段带 10-27 过载史）。
2. 卸载段（unload）完全未开发（师兄有 /prepare_unload_flow）。
3. run_teaching_chain.py 仍是旧位姿目标逻辑（plan_pose_once），未换 plan_joint_locked。
4. 六类模式（run_task.py）未同步获得示教链路的任何新防护（两线独立）。
5. 反控通道丢失只有手动 resend 规程，执行器 NO_MOTION 时不自动恢复。
6. RVS 固定拍摄位与真机工具不匹配（S2 自碰撞）只能人工绕行，程序不自动换站位。

### B. 几何/模型层面

7. 夹爪**张开态**前伸连杆仍无模型（146–203 mm）。
8. 保护座横向包络是"并集盒"近似，非实测精确几何。
9. 关节到位 0.0003 rad 而 TCP 差 10.8 mm 的根源未查透（被 0.5 mm 门掩盖）。
10. 桌子立面/桌腿未建模（2.5-D 桌面带近似）。
11. 工件偏转时的插入姿态对齐 open question 未解（接触检测只是兜底）。

### C. 专项3

12. 力控装配 A 期未实施——插入段目前仍是"0.005 m/s + fault-poll"被动防护。
13. 卡死判定师兄未完成，我方也未实现。

### D. 工程性

14. 全部示教/专项代码仍未 git 提交（scripts/teaching/ 等整目录 untracked）。
15. /tmp 辅助脚本（plan_joint_goal / adjust_tcp / fk_height_check）未入库，重启即失。
16. 示教与六模式的插入常量两处并存，可能漂移（13 mm 差异已由 R1 链解决，但六模式侧未动）。
17. 示教 README 部分内容陈旧（"未实现插入段"等 10-27 表述）。
18. 无示教链路集成测试（仅有纯逻辑单测）。
19. 环境模型是 10-27 数据（开工前应重采）。

## 三、开工后的建议顺序

```text
1. 启动链 → 四项门 → 重采环境模型（model_environment.py）
2. 力控 A 期空载基线标定（定接触/安全阈值）→ 实施 compliant_insert.py
3. 步骤 6/7 实跑（闭合进入抓取 + 张紧式抓取）→ 8–11 → 12–18 带力控重跑
4. 六模式同步新防护（至少 time-scale 守护与 plan_joint_locked 分支锁定）
5. git 提交 + README 更新 + /tmp 脚本入库
```

## 四、分支归档（2026-11-03）

```text
分支  teaching-rvs-mode（基于 main cf54936 创建，未并入 main）
提交  cafeda2  27 files changed, +4175/-46
内容  示教链路 + 三大专项 + 真机描述修复 + 安全守护 + 测试 + 全部 worklog/方案
      （含 10-27 环境模型再生成的 scene yaml/sdf）
不包含 runtime/、build/、install/、log/（按仓库规则不提交）
```

后续顺序（待硬件开机）：重采环境模型 → 力控 A 期基线 → 步骤 6/7 实跑 → 六模式同步防护。
