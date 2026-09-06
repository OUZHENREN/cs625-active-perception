# P7.2 位姿估计器至 P7.3 的交接合同

P7.2 不得把不可观测的偏航角伪装成低不确定性 `SE(3)`。本合同将
“完整单视角 6D 成功”与“安全拒绝并请求补观测”严格区分。

## 可观测状态

当 RGB-D/CAD 模板支持、相似度和 yaw 分离度均达到门槛时，估计器输出
`pose_valid_for_grasp=true`、有限低 covariance，并由独立 GT scorer 报告
translation、rotation、ADD、ADD-S 和 pose success。无遮挡、轻遮挡和中遮挡
各连续五个冻结窗口必须通过完整 SE(3) 外评。

## 不可观测状态

当顶面模板支持不足、相似度不足或 yaw 歧义时，估计器必须输出：

- `POSE_YAW_UNOBSERVABLE`；
- `pose_valid_for_grasp=false`；
- `texture_yaw_observable=false`；
- 偏航方差至少为均匀角先验 `pi^2/3`，同时扩大平移协方差；
- 保留候选位姿仅作 NBV 搜索中心，禁止送入抓取、预抓取或执行。

这个状态不是 P7.2 的完整 SE(3) 成功，也不能主张抓取成功。它只允许
P7.3 依据 covariance 和 failure code 选择新视角并重新观测；P7.3 后必须
重新运行 P7.2，而不能重用该候选 yaw。

## 交接门

`test/p7_2_validate_handoff_gate.py` 验证三项：无遮挡/轻/中三层各 5/5
完整 SE(3) 通过；重遮挡拒绝不读取 GT；拒绝输出具有高偏航方差且不可用于
抓取。通过后，交接决定只能是 `P7.3_REQUIRED_REOBSERVATION`。
