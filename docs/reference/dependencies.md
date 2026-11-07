# 依赖与外部组件（2026-11-07）

> 取代归档的 `dependencies.md`。原有供应商条目沿用，新增运行期踩到的三条事实。

## 1. 版本与工作空间

```text
ROS 2 Jazzy / Ubuntu 24.04（WSL2）
底层链：/opt/ros/jazzy → ~/cs625_underlay_jazzy → 本仓库 install
入口脚本：source scripts/source_dev_env.sh --full
相机工作空间：~/cs625_camport_ws（percipio_camera，需要时叠加）
```

## 2. 供应商组件

| 组件 | 位置 | 状态 |
|---|---|---|
| Elite CS625 官方驱动 | underlay（`eli_cs_robot_driver`） | 只读，不修改 |
| Elite CS SDK | underlay | 只读 |
| MoveIt 2 / ros2_control | 系统安装 | 只读 |
| 图漾相机驱动 | `~/cs625_camport_ws` | 只读 |
| RVS（Windows） | 7000 命令 / 2024 RVS_AI_Center | 外部程序，黑盒 |

**规则**：不原地改供应商代码；需要改就走 fork + 固定 commit，并在本文件记录。

## 3. 运行期必须知道的三条事实（2026-11-05/07 实测）

```text
① 相机与 RVS 互斥
   RVS 占用相机时 ROS 相机无法启动；两者不能同时用。切换前停掉另一个。

② 网络必须 0% 丢包
   机器人 192.168.1.200 / 主机 192.168.1.100（WSL 镜像 Windows 以太网）。
   实测 76% 丢包时表现为"反控端口为 0、示教器显示未连接到服务端"；
   网络恢复后端口仍为 0 说明主端口连接已死 → 必须重启驱动栈。

③ Windows"公用网络"会拦反控入站
   50001/50003/50004 被防火墙丢弃时，本侧 ss 连 SYN_RECV 都看不到。
   处置：将网络改为"专用"，或显式放行这三个端口。
```

## 4. 已知不可用/受限

```text
· 六维力传感器未安装（十二月），force_torque 相关接口无数据
· rviz_common 无 Python 绑定 → 面板 3D 视图为自绘，非 RViz
· RVS 的 pose fitness 需从 Windows 日志读取，属间接通道
```

> 来源：由归档原件 `dependencies.md` 更新，追加相机互斥、镜像网络与防火墙三条运行期事实。
