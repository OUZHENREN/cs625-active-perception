# 2026-11-02 REVIEW 示教模式审计（开发程度 / 与师兄对比 / 漏洞清单）

> 审计对象：`scripts/teaching/`（示教链路，独立于六类模式）。
> 方法：通读 teaching_sequence.py（501 行）与 rvs_pose_client / run_teaching_chain /
> pose_math / gripper / verify_pose / check_tool_envelope，并对照师兄只读栈
> `~/cs625_legacy_underlay_jazzy/src/` 的功能组成。
> 结论：**观测前缀至预抓取已实测闭环；第 6 步（闭合进入抓取位）起代码完整但零实测，
> 插入段背着 10-27 的过载旧账；审计发现 7 条逻辑漏洞（3 高危 / 3 中危 / 1 低危）。**

## 1. 开发程度（按 19 步）

| 步骤 | 状态 | 依据 |
|---|---|---|
| 0 环境建模 | 代码在，输出未接入本链路（装饰性） | 生成的 model.json 无人使用，失败不阻止 |
| 1–4 观测前缀 | 完整，多次实测通过 | 10-30/11-02 T0/T1/T2/T3 全通；固定/动态规则落地；多帧策略 |
| 5 预抓取 | 完整，实测通过 | 分段+包络门+TCP 微调，11-02 到 0.35 mm / 0.04° |
| 6 进入抓取位 | 代码完整，未实跑 | 10-27 事故史步骤 |
| 7 张开夹爪 | 代码完整，未实跑 | 张紧式 50 mm |
| 8–11 抬升/转运 | 代码完整，未实跑，无包络检查 | 见漏洞 4 |
| 12–18 插入段 | 代码完整，10-27 过载史，未再验证 | 固定常量与 R1 反算差 13 mm（OPEN） |
| 卸载段 | 未开发 | 师兄有 /prepare_unload_flow |

## 2. 与师兄原模式对比

| 功能 | 师兄原模式 | 本示教模式 | 差异 |
|---|---|---|---|
| RVS 触发 | pose_sender_node | rvs_pose_client（TCP→tool0 + 多帧） | 等价+增强 |
| 位姿回传 | tcp_server（9999，S/R 报文） | 直接复用师兄 tcp_server | 复用不修改 |
| 位姿换算 | tcp_server 内 T_b_tcp/T_s_tcp | pose_math.py | 已比对 ≤1 mm |
| 运动执行 | robot_commander（60000 口，/target_pose → 机器人本体控制） | MoveIt + arm_controller（ros2_control） | **执行通道本质不同** |
| 夹爪 | 机器人端口命令 | Windows 桥 hitbot_cmd.ps1 | 等价 |
| 环境碰撞 | environment_point_cloud_publisher + environment_collision_generator | 空场景 + 离线 check_tool_envelope.py | 缺口：场景空 |
| 携带工件建模 | collision_object_publisher / attached_box_visual_publisher | 无 | 缺口 |
| 状态监控 | cs625_state_monitor | control_health.py + fault-poll | 基本等价 |
| 取出流程 | /prepare_unload_flow | 无 | 缺口 |
| 力探/柔顺 | cs625_tcp_force_probe / cs625_compliant_placement | 无 | 缺口 |

## 3. 近期改进清单（10-30 至今）

- **10-30**：捕获路径 parents[2] 修正；第 2/4 步改实时 S0/S2；多帧触发（≥3 帧 ≤10 次、
  工作空间过滤、spread 打印）；插槽历史对比；--time-scale<1 拒绝守护；README 文档化。
- **10-31**：新增 check_tool_envelope.py；分段下降（SEGMENTED、逐段校验、t=0 重计时）；
  工具几何权威化（SolidWorks gripper_0920.stl 含保护座，写入 common.yaml）；
  复验证实"10-30 事故=速度过快"假设。
- **11-02**：TCP 迭代微调；查明 RVS 指定 S2 不可站根因（唯一解自碰撞 5.3 mm）并确立
  绕行站位；定位并修复反控通道静默丢失（resend_external_script）；当天 T2/T3 三帧
  → 新抓取位（与 10-30 差 97 mm，工件确实会移动）。

## 4. 漏洞清单（7 条）

1. 【高危】S0/S2 无"可达+无自碰撞"前置门；move() 用位姿目标，无分支锁定
   （11-02 S2 唯一解自碰撞；位姿目标曾给出 918 点翻腕 / 734 点 J2=255° 路径）。
2. 【高危】latest_capture() 无有效性/新鲜度校验（坏 S2 z<0、隔天旧 T3 会被静默使用）。
3. 【高危】抓取进入前无 TCP 级 verify（实测名义与实际差 10.8–18 mm；13b 门只在插入前）。
4. 【中危】抬升/携带段（8–11）无包络检查、不分段；张开臂前伸与携带工件均未建模。
5. 【中危】R1 采了不用：插入链用固定常量，与 R1 反算差 13 mm（10-27 过载根因的另一半）。
6. 【中危】多帧 spread 只打印不判据（R1/S3 帧间差 200 mm 也 PASS）。
7. 【低危】步骤 0 装饰性；--skip-observe 无年龄提示；run_teaching_chain 默认 grasp
   是 10-27 旧值；grip 后不校验 position_mm≥40 就 lift。

## 5. 修补计划（本轮执行，按严重度）

1+2 → 新增 plan_joint_locked.py（IK avoid_collisions=True + 当前关节种子 + 关节空间规划）
      替换 move() 的 plan_pose_once；latest_capture 加工作空间/四元数/新鲜度(6h)校验。
3   → 步骤 5/6 后加 verify_pose 门（0.5 mm / 0.05°），verify_pose.py 增加 CLI 容差参数。
4   → SEGMENTED 扩到 lift/transit/staging；check_tool_envelope.py 加 --carried
      （并入 check_carried_clearance 的携带盒），CARRIED 集合自动附加。
5   → 插入链改为由实时 R1 经 insert_from_slot 计算，无 T1 时回退六模式常量并告警。
6   → rvs_pose_client 对 T1/T3 加帧间 spread 上限（30 mm / 3°），超限退出码 4。
7   → 步骤 0 失败告警；grip 后 grip_still_holding 校验；run_teaching_chain 默认值告警。

## 6. 下一步（参照师兄部分，另行专项进行）

- 环境碰撞：参照 environment_collision_generator / environment_point_cloud_publisher，
  把实测桌面+夹具灌进 MoveIt 场景（先解决"厘米级精度误报"问题）。
- 携带工件：参照 collision_object_publisher / attached_box_visual_publisher，
  抓取成功后在规划场景把工件 attach 到 TCP。
- 力控装配：参照 cs625_tcp_force_probe / cs625_compliant_placement，
  为插入段补力/力矩反馈与柔顺控制。
