# 下一次对话交接：开始视觉算法研究（2026-11-09 起）

## 0. 一句话现状

真机链条已跑到"预插入"，插入未完成；面板与文档整理完毕；**下一步做视觉算法**（十一月主线）。

## 1. 已就绪的东西（可直接用）

```bash
source scripts/source_dev_env.sh --full

# 两模式
bash scripts/resume_teaching.sh              # 安全模式：自动重启栈 + --resume 续跑
python3 scripts/teaching/teaching_debug.py --stage 5 --dry-run   # 效率模式
python3 scripts/cs625_panel.py               # 面板（两标签页 + 相机 + 夹爪 + RViz 状态）
scripts/open_rviz.sh                         # 带 MoveIt 参数的 RViz
python3 scripts/show_targets_in_scene.py     # 工件/插槽模型显示
python3 scripts/camera_mode.py {status,fast,modeling,high,stop}
```

```text
文档入口：docs/README.md
规则必读：docs/agent_memory.md（含断联、插入权威、工件附近先抬升等硬规则）
真机经验：docs/manuals/2026-11-03_MANUAL_REAL_MACHINE_EXPERIENCE.md（29 条）
架构与链路：docs/reference/architecture_current.md
指标体系：docs/reference/experiment_metrics.md（六类 18 项）
```

## 2. 视觉研究的起点与关键决定

```text
RVS 的定位已经改为【一次性位姿采集工具】，不是运行时组件：
  · 采集耗时 2.2–18.6 s，可用于"采一次定死位姿"
  · 系统偏置实测 33 mm、帧间离散 920 mm → 必须多帧 + 一致性 + 位姿库交叉校验
  · 与 ROS 相机互斥，不能同时用
  
因此视觉工作的目标不是"替代运行期视觉"，而是：
  ① 用自研管线替代 RVS 做位姿采集（可解释、可量化、能写进论文）
  ② 运行期用固化位姿 + ROS 高帧率彩色流（监视/录制/力控实验同步）
```

## 3. 建议的第一步（不需要机械臂）

```text
目标：跑通自研采集链路的"数据入口"，并量化三件事——帧率、点距、镀膜面反射情况。

步骤
  1) 停 RVS（Windows 端），确认相机可被 ROS 占用
  2) python3 scripts/camera_mode.py modeling     # 1280×960 含深度/点云
  3) 采 20 组静态点云：5 种工件朝向 × 4 次
     落盘到 runtime/vision_dataset/<stamp>/{cloud.pcd|npy, color.png, depth.npy, meta.json}
  4) 统计：深度帧率、有效点比例、平面性（5 指标里的 planarity/CV）、镀膜面空洞率

验收
  · 能复现同一姿态下两次采集的位姿差 < 2 mm
  · 记录每帧耗时，给出"帧数 vs 精度 vs 耗时"曲线（论文第一张图）
```

## 4. 已有代码与接口（复用，不要重写）

```text
scripts/panel_camera.py        相机订阅与 1:1 显示（best-effort QoS 已处理）
scripts/show_targets_in_scene.py  已知位姿 → RViz marker（验证位姿对不对）
scripts/teaching/rvs_pose_client.py  RVS 采集契约（fitness/离散度/工作空间校验）可作为
                                    自研管线的验收标准：同样的门限、同样的落盘字段
scripts/teaching/pose_library.py    位姿库（fitness<0.9 不入库）
runtime/environment/latest/model.json  工位几何（桌子 z + xy 范围、体素、slot/工件位姿）
config/tasks/common.yaml       slot_origin / workpiece_reference / 抓取位等权威值
```

## 5. 陷阱清单（踩过的）

```text
· 相机发布端是 BEST_EFFORT：订阅用默认 reliable 会一帧收不到且不报错
· 同一进程里 rclpy 只能 init 一次；多线程 spin 必须串行
· RVS 与 ROS 相机互斥：切换前停掉另一个
· 保护停止后不自动重试；连续两次 NO_MOTION 才允许一次驱动栈重启
· 工件附近禁止横向移动，必须先竖直抬升 20 mm
· 插入权威 = runtime/slot_reference_verified.json（当天 R1 差 33 mm，不能用）
· 文档/代码里的绝对路径：仓库在 /home/oy/projects/cs625-active-perception
```

## 6. 交接时需要知道的未完成项

```text
· 插入段未在真机完成（372 mm 不是物理底部；三面接触）
· 槽内转 10°：源码依据已确认（师兄 rotated = insert @ Ry(-10°)），未实机跑通
· 六维力传感器十二月到位 → B 类指标全部"未采集"
· 效率模式、straighten_in_slot、装配搜索：未上机
```

## 7. 下周一建议的第一条命令

```bash
cd /home/oy/projects/cs625-active-perception
source scripts/source_dev_env.sh --full
python3 scripts/camera_mode.py status     # 先看相机在不在、什么分辨率、多少 fps
```

然后再决定：直接开始采集脚本，还是先补一个 `scripts/vision_capture.py` 把落盘格式定下来。
