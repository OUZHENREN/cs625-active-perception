# P7.3 几何覆盖与位姿信息 NBV 合同

P7.3 仅在 P7.2 的受控交接状态下运行。输入为：P7.2 输出的 `SE(3)` 候选
位姿及 `6×6` covariance、同一冻结窗口的 RGB-D/PointCloud 几何、YCB CAD 网格
与纹理，以及版本化候选视点参数。估计器的重遮挡候选位姿只能作为搜索中心，
不得授权抓取。

`test/p7_3_geometry_nbv.py` 明确不读取 Gazebo 真值，也不读取或导入
`coverage_proxy`。其两个正式量分别为：

1. **真实几何 coverage**：将 CAD 三角面投影到当前深度图，只有与实际深度
   在 `12 mm` 内匹配的正面可见面才记为当前已观测；候选相机下前向、在视野内且
   不属于已观测面的 CAD 面积，除以总 CAD 面积得到新增几何覆盖率。
2. **位姿信息增益**：对候选下可见的带纹理 CAD 面，用针孔投影对 yaw 的解析
   Jacobian 和 `2 px` 测量噪声构造 Fisher 信息；以 `15000` 个独立纹理残差/
   平方米的版本化采样密度将 CAD 面积转为残差数量，并与 P7.2 的先验 yaw
   方差做信息滤波更新。输出先验/后验方差和 nats，不把它重命名为 coverage。

停止准则只读取**当前实测**的 P7.2 yaw 标准差：小于等于 `5°` 时记录
`P7.3_STOP_CRITERION_MET`。候选 Fisher 后验只用于排序，绝不在尚未实际重观测
时伪造“已收敛”。重遮挡状态若未满足停止准则，输出
`P7.4_REQUIRED_FOR_SELECTED_VIEW_EXECUTION`；这只是一项待验证视点建议。

P7.4 才能依次验证 FOV、workspace、IK、joint limit、collision 与 planning；
本合同不得据此宣称任何候选可达、无碰撞、已执行或已经抓取。P7.4 实际移动后，
必须以新 RGB-D 窗口重新运行 P7.2，再用新 covariance 复核本合同的停止准则。

## 运行时可观测性约束

候选相机的目标投影必须落入版本化的目标保护区域，避免夹爪进入有效纹理 ROI；
这是一项 P7.3 的成像可观测性约束，而不是碰撞结论。对于冻结 PointCloud，系统将
相机坐标系中的非目标点变换到世界系，去除目标近邻后作 `15 mm` 体素化；任一 CAD
表面到候选相机的射线若在其前方被该点云以 `18 mm` 横向阈值命中，即记为外遮挡，
不计入预期 coverage 或 Fisher 信息。输出必须保留
`external_occluded_surface_fraction`、`self_occlusion_safe` 与
`target_projection_v_px`。

`2026-09-02 r6` 的点云遮挡感知候选在独立 Gazebo 分区中采集了 5/5 合格的
RGB-D 窗口（640×480、同步差 0.1 s、每窗 2324 个目标支持点、
`trajectory_commands_sent=0`）。重跑 P7.2 后 5/5 均因
`CAD_TEMPLATE_SUPPORT_TOO_SPARSE` 与 `POSE_YAW_UNOBSERVABLE` 被拒绝：模板支撑
195 小于 200 像素，平移误差 49.27 mm、旋转误差 148°、ADD-S 23.58 mm。
这是一条有效的 P7.3 失败证据，说明“几何/信息预测可排序”不等价于“实际重观测
已达到抓取位姿精度”；它不得解锁 P7.4 或 P7.5。
