# P7.2 现成 6D 位姿估计器复用审计（2026-09-01）

## 结论

正式 P7.2 优先接入 NVIDIA FoundationPose 的 model-based 路线，CPU 圆柱基线只保留为输入/评测合同与平移下限。当前不得在 Jazzy overlay 内直接安装 FoundationPose，也不得用 identity quaternion 或 Gazebo GT 补齐纹理 yaw。

## 候选比较

| 候选 | 与本项目输入的匹配 | 环境代价 | 结论 |
| --- | --- | --- | --- |
| [NVlabs/FoundationPose](https://github.com/NVlabs/FoundationPose) | RGB、Depth、mask、K、CAD OBJ；输出完整 6D pose，支持 YCB/新物体且无需本对象重训练 | 官方建议 Docker 或 Conda；需 PyTorch、PyTorch3D、NVDiffRast、CUDA toolkit、两组权重 | **首选，独立环境接入** |
| [MegaPose](https://github.com/megapose6d/megapose6d) | RGB、K、mesh、2D box，可选 depth | 官方建议 Conda/Docker及独立模型数据 | 备选；当前不优于 FoundationPose 的 RGB-D 接口 |
| [DenseFusion](https://github.com/j96w/DenseFusion) | 原生 RGB-D 与 YCB-Video | 旧版训练/依赖栈，依赖对象级训练与分割 | 仅作论文基线参考，不作为主接入路径 |

## 已锁定上游身份

- 外部只读工作副本：`B:\Recent\Robotic arm\Ubuntu_Share\third_party\FoundationPose`
- 官方仓库：`https://github.com/NVlabs/FoundationPose.git`
- 浅克隆提交：`a1b694b83e633c2cb6115b9063d940a687759392`
- 提交时间/标题：`2026-04-29T09:53:07-07:00 Local Conda install: fix mycpp import, streamline dependencies, refresh readme (#407)`
- 许可证：上游 `LICENSE` / NVIDIA Source Code License；没有把上游源码复制进本项目仓库。

## 当前主机能力与阻断项

- WSL GPU：NVIDIA GeForce RTX 3060 Laptop GPU，6144 MiB，驱动 `566.07`；只能说明 GPU 可见，不能保证注册阶段显存足够。
- 当前 Jazzy Python 无 PyTorch；WSL 中无 Docker、Conda、Micromamba 和 `nvcc`。
- 官方本地路径要求 CUDA toolkit 编译 PyTorch3D 与 NVDiffRast，并另行下载 refiner `2023-10-28-18-33-37`、scorer `2024-01-11-20-02-45` 权重。
- 因此当前阻断不是 P7.1 数据，而是隔离推理运行时。不得把这些依赖安装到 `$CS625_APP_INSTALL` 或 ROS underlay。

## 冻结接入边界

1. ROS/Jazzy 侧只负责导出冻结 RGB、Depth、K、mask、YCB OBJ 和帧时间戳 TF。
2. FoundationPose 在独立环境中读取文件并输出 camera-frame 4×4 pose、score、耗时和上游提交/权重哈希。
3. 项目侧独立 adapter 将 pose 转到 `world`，形成 pose + covariance + quality 记录；GT 只由 scorer 在估计文件落盘后读取。
4. 先用一个 r7 窗口做显存/依赖 smoke；成功后才执行 5 窗口与轻/中/重遮挡。任何 OOM、编译或权重缺失均写失败码，不回退 GT relay。
5. FoundationPose smoke 通过和三遮挡层完整 SE(3) 指标通过前，P7.2 保持 `IN PROGRESS`，P7.3 保持 `NOT STARTED`。

## 需要的下一项环境动作

推荐新建独立 FoundationPose Conda/Micromamba 环境并安装 CUDA toolkit，或安装 Docker 后使用上游镜像；两者都会新增较大的系统依赖与模型权重，需作为独立环境变更执行，不属于当前 Jazzy 增量构建。
