# 实验数据实时记录协议

本文件规定每一次真机/仿真实验的最小记录格式。目标：任何一次有价值的实验都能在
事后回答“用什么配置、输入是什么、规划/执行是否成功、最终误差多少、有没有异常”。

## 1. 目录与命名

每次实验一个目录，不覆盖、不合并：

```text
~/cs625_legacy_snapshot_20261020/runs/YYYYMMDD_HHMMSS_<sim|real>_<task>/
  meta.json          # 元数据、输入、规划、执行、结果摘要
  joint_states.csv   # 整段运动，time,q1..q6
  tcp.csv            # 整段运动，time,x,y,z,rx,ry,rz
  plan.json          # 规划轨迹/planned TCP path 摘要或引用
  logs/              # 驱动、MoveIt、RVS 原始日志
```

## 2. meta.json 必填字段

### 运行与配置

- `run_id`, `timestamp`, `operator`, `profile`（sim/real）；
- `repo_commit`, `underlay_ref`, 启动命令；
- `robot_ip`, `rvs_project_version`, 手眼文件哈希；
- `speed_scaling`, `collision_scene_loaded`。

### 输入

- `target_source`（RVS T1/T3/人工/仿真真值）；
- 原始 RVS 输出行（S3/T1/T3）或仿真目标；
- `target_pose`、`frame_id`、`pre_grasp_pose`、`grasp_pose`；
- `T_b_tcp` / `T_s_tcp` 版本。

### 规划

- `planner_id`, `plan_success`, `planning_time`；
- 轨迹点数、轨迹时长、planned TCP path；
- 路径长度、目标超调量、是否经过 approach waypoint；
- 失败码（如 `PLAN_FAILED`）。

### 执行

- `execute_service_result`, controller goal 结果（`SUCCEEDED/ABORTED/PREEMPTED`）；
- 实际执行时长、速度缩放、是否中途停止；
- 完整 joint/TCP 采样文件引用。

### 结果

- 最终 TCP、最终关节、位置误差、姿态误差；
- 成功/失败、失败码、是否急停、现场备注、照片路径。

## 3. 实时原则

1. **每次实验前先建 run 目录**，不做事后补记；
2. 规划完成立即写 `plan.json`，执行完成立即写执行结果；
3. 运动全程采样，不能只采前几秒（今天的超调就发生在采样结束后）；
4. 有价值的失败同样记录，不删除；
5. 原始日志保留在 `logs/`，worklog 只写摘要和路径；
6. 大文件不进 git，保留在本地证据目录。

## 4. 当前已记录

- `runs/20261020_real_pregrasp/`：首次真机 pre-grasp 成功（含超调观察）。
