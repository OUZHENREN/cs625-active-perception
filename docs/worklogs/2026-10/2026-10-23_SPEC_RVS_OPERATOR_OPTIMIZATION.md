# 2026-10-23 SPEC — RVS 算子图结构与可优化清单

## 1. 分析对象与方法

```text
对象：RVS 工程 CS625_SlotBox_VisualGuidance_RVS（*.rvs，纯 JSON，约 1.5 MB）
方法：只读解析 node_list / edge_list / parameters，未修改任何原始文件
归档：~/cs625_rvs_archive/20261023/（附 sha256）
```

## 2. 工程结构

```text
顶层 8 个 Group：
  Resource                        2 子节点（TCP 连接资源、图漾相机资源）
  T0-S0 消息移至插槽拍摄位        18 子节点 / 76 节点
  T1-R1 拍摄插槽                  18 子节点
  T2-S2 消息移至弹仓拍摄位        20 子节点
  T2-S2 消息移至弹仓拍摄位_1      11 子节点（与上者高度重复）
  T3-S3 消息移至抓取位            18 子节点
  rvsGroup                        22 子节点
  rvsGroup_0                       7 子节点
```

节点类型分布（全工程）：

```text
rvsDataNode 84 / rvsMirrorNode 84（结构节点）
EmitNode 56 / GroupNode 43 / TransformNode 29 / OrNode 17
SaveNode 15 / CommonTCPClientNode 14 / TriggerNode 13 / LoadNode 11
DirectoryOperationNode 10 / AtListNode 7
```

## 3. 一次 T2-S2 的链路（代表其余组）

```text
Trigger → TyCameraAccess（图漾取流）
        → AI推理分割点云(GPU版本)：AIInferenceNode → ProjectMask2Cloud
        → 点云坐标转换（手眼标定 LoadCalibFile + Transform）
        → 计算拍摄点（Percipio3DMatchingNode + 排序 + AtList）
        → 拍摄位置转换 → Emit / Transform → 发送S2消息（CommonTCPClient）
```

## 4. 可直接调的关键参数

```text
AIInferenceNode     server_IP=127.0.0.1  server_port=2024  model_ID=0
                    show_result_visibility=1        ← 生产环境可关
Percipio3DMatching  modelRelSamplingDistance=0.03     ← 采样步长，越大越快
                    estimate_normal=1                  ← 法向估计，开销大
LoadCalibFile       res_x=2560 res_y=1920 fx=1862.9    ← 全分辨率
SaveNode            cloud.pcd / color.png 每次采集写盘
DirectoryOperation  MakeDirectory / LimitDirectory(retention=10) 每周期执行
TyCameraAccess      cloud_point_size=1
```

## 5. 可优化清单（按预期收益排序）

| # | 优化项 | 做法 | 预期收益 | 风险 | 可自动化 |
|---|---|---|---|---|---|
| 1 | 去掉每周期磁盘写入 | SaveCloud/SaveColor/目录操作用于调试；生产时旁路或改为按需开关 | 大（NTFS 写入常见 100–300 ms/次） | 低（只是丢调试文件） | 是 |
| 2 | 关闭 AI 结果可视化 | `show_result_visibility 1 → 0` | 中（省渲染与回传） | 极低 | 是 |
| 3 | 3D 匹配降采样 | `modelRelSamplingDistance 0.03 → 0.05`，`estimate_normal` 按需关 | 中–大（点云配准是主耗时） | 中（精度下降，需对比分数） | 是 |
| 4 | 合并重复 Group | `T2-S2` 与 `T2-S2_1` 合并，去除重复 Transform/ProjectMask | 中（少跑一遍链路） | 中（需回归 T2/T3） | 半自动 |
| 5 | 标定文件只加载一次 | 11 个 LoadNode 若在触发链路内，改为启动时加载一次 | 中（省文件 IO） | 低 | 半自动 |
| 6 | 两段式定位 | 低分辨率粗定位 → 高分辨率精定位，只在必要时跑精定位 | 大（平均耗时下降） | 中（需两套参数） | 否（需设计） |
| 7 | 相机降采样 | 粗定位用 1280×960，精定位再全分辨率 | 中–大 | 中 | 是 |
| 8 | 通讯异步化 | TCPServerReceive/CommonTCPClient 改为异步 + 缓存最近结果 | 中（去掉阻塞等待） | 低 | 是（我方侧） |

## 6. 与 ROS 侧的分工建议

```text
RVS 负责：定位（T2/T3/T1 的模板匹配与位姿解算）
ROS 侧负责：命令下发、结果缓存与新鲜度校验、超时重发、状态与日志
若相机可被我方直连：RVS 走定位链路，我方走高频观测与自研检测，按时间戳融合
```

## 7. 验证方法与硬约束

```text
验证：同一工件位姿下，对比改造前后 T2/T3/T1 的
      · 端到端耗时
      · RVS 返回分数
      · 目标完整度 / 位姿偏差（与基准比较）
硬约束：
  1. RVS 安装目录只读，所有改动另存新文件名（如 ..._opt_v1.rvs）
  2. 改动前归档并记录 sha256
  3. 每次只改一类，改完立即验证，便于定位回退
```

## 8. 结论

```text
工程文件是 JSON，结构清晰、参数可分；
最大的确定性收益来自「去磁盘写入 + 关可视化 + 3D 匹配降采样」，
这三项都属于低风险、可自动化，且不改变定位算法本身。
两段式定位与相机降采样收益更大，但需要设计两套参数与回归验证。
```

## 9. 分工模式：并行 vs 分阶段（所有者决策方向）

### 9.1 首选：并行分工

```text
RVS 侧：详细位姿处理（模板匹配、位姿解算，精度优先）
ROS 侧：实时观测 + 推理（高频、低延迟，用于搜索与过程监视）
融合规则：RVS 结果为主，ROS 结果为辅；两者都带时间戳与来源标记
```

前提：相机允许多进程同时访问。

### 9.2 回退：分阶段交替（确定可落地）

```text
阶段 A（RVS 主导）：环境建模 + 详细位姿
  · 工件桌 / 插槽桌碰撞体
  · 插槽绝对位姿校验
  · 工件初位姿
阶段 B（ROS 主导）：后续全部流程
  · 实时观测、目标确认
  · 抓取 / 插入 / 取出过程在线监视
  · 需要时（每轮任务开始）再切回 RVS 复测
```

切换要求：

```text
1. 交接文件：model.json + 任务记录（时间戳、来源、测量者）
2. 结果带来源标记（rvs / ros），上层按新鲜度与置信度选择
3. 切换时相机句柄完全释放/重新获取，避免独占冲突
```

### 9.3 需要先验证的三件事

```text
① 相机是 GigE 还是 USB3
② 是否允许多进程同时打开
③ ROS 2 侧驱动可用性（图漾 / Percipio）
```
