# 坐标系与话题约定（2026-11-07）

> 取代归档的 `frames_and_topics.md`（那份只覆盖早期 P3/P4 话题）。

## 1. TF 链

```text
world → base_link                                  世界与机械臂基座
base_link → shoulder_link → upperarm_link → forearm_link
          → wrist_1_link → wrist_2_link → wrist_3_link
          → flange → tool0 → my_end_effector_link  工具链（末端 = TCP）
base_link → camera_link → camera_depth_optical_frame
```

```text
工具常量：T_flange_tcp = (-0.046, 0, 0.353) m（控制器 TCP 与 MoveIt FK 实测差 0.0 mm）
角度约定：base_link 系，X 前 Z 上（REP-103），绕 +Z 俯视逆时针为正
```

## 2. 位姿接口（应用层）

| 话题 | 类型 | 用途 |
|---|---|---|
| `/tcp_pose_broadcaster/pose` | `PoseStamped` | 当前 TCP（面板/门检/场景都用它） |
| `/slot_target_pose` | `PoseStamped` | R1：插槽位姿（**只作对比**，不作插入权威） |
| `/slot_precision_view_tcp_pose` | `PoseStamped` | S0：插槽精观测视角 |
| `/box_target_pose` | `PoseStamped` | S3：工件位姿（抓取依据） |
| `/box_precision_view_tcp_pose` | `PoseStamped` | S2：工件精观测视角 |
| `/box_grasp_tcp_pose`、`/box_pre_grasp_tcp_pose` | `PoseStamped` | 师兄桥按 S3 推出的抓取/预抓取 |

> RVS 桥（9999）发布；RVS 命令端口 7000、RVS_AI_Center 2024。三者都不在运行期参与控制。

## 3. 状态与安全

| 话题 | 说明 |
|---|---|
| `/io_and_status_controller/safety_mode` | 1=NORMAL 才允许运动 |
| `/io_and_status_controller/robot_mode` | 7=RUNNING |
| `/io_and_status_controller/robot_task_running` | 外部控制程序是否在跑 |
| `/speed_scaling_state_broadcaster/speed_scaling` | 速度滑条（0 = 程序暂停） |
| `/energetic_controller` 相关动作 | `arm_controller/follow_joint_trajectory` |

## 4. 力矩与传感

| 话题 | 现状 |
|---|---|
| `/joint_states`（effort 字段） | 125 Hz，可读 Σ|τ| 作为接触代理 |
| 六维力 | **传感器未安装**，`force_torque_sensor_broadcaster` 无数据；面板显示"未采集" |

## 5. 相机与点云

| 话题 | 说明 |
|---|---|
| `/sensors/camera/points` | 归一化点云（面板 3D 场景订阅，抽稀绘制） |
| 品牌原始话题 | 只允许出现在 `cs625_sensor_adapter` 内 |

## 6. 不变量（违反即报错）

```text
· RVS 与 ROS 相机互斥：不能同时占用相机
· 反控端口 50001/50003/50004 由机器人主动拨入，本侧只监听
· 位姿门只验证"与计算目标一致"，不验证"目标本身正确"
```

> 来源：由归档原件 `frames_and_topics.md` 更新，补入 tcp_pose_broadcaster、RVS 采集话题、
> 状态/力矩话题与当前不变量。
