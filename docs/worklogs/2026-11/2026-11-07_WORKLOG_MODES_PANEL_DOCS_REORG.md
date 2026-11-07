# 2026-11-07 工作日志：两模式收敛、面板重做、文档重整、RViz 接入

> 本日无机械臂运动（未接机器人、未上电运动）。全部改动为离线开发、离线测试与文档。

## 1. 所有者当天要求（按提出顺序）

1. 删除 VS Code 里六模式相关任务，任务栏不超过 5 个快捷任务；
2. 只保留两种模式：**安全模式**（完全等同已开发完成的示教链路）与**效率模式**（示教链路的调试版，
   十一月视觉算法、十二月力控算法都在这里开发），两者都要接入新面板；
3. 重整工作区（各年代开发混在一起、层级混乱）；可合并文档，但旧文档保留一份；
4. 实验指标偏少、尤其缺机械属性指标；面板按 4K 屏重新设计；
5. 评估完整方案可行性（视觉端已定算法、力控端路线、仿真→实机→残差 RL）；
6. 逐文件核验文档内容，不是泛泛而谈；
7. 3D 场景不要自造，直接参照 RViz / 师兄源码的设置；
8. 把教学分支与 main 推到 GitHub。

## 2. 结果一览（对应 8 项要求）

| # | 要求 | 结果 | 提交 |
|---|---|---|---|
| 1 | 任务收敛 | 17 → **5** 个任务；旧文件归档 | `c048322` |
| 2 | 两模式 | 安全模式=`resume_teaching.sh`；效率模式=`teaching_debug.py` | `76c762e` |
| 3 | 文档重整 | `reference/` 9 篇、`worklogs/` 分月、archive+MANIFEST | `8776ccc` `b602b2f` `fcd0b5c` |
| 4 | 指标 + 4K 面板 | 六类 18 项指标骨架；三列紧凑面板 | `76c762e` `8d862be` |
| 5 | 方案评估 | 已给出书面评估（视觉/力控/整体），未单独提交文档 | — |
| 6 | 逐文件核验 | 11 份文档 29 项机械校验全过；修 18 处旧路径 | `b97db76` |
| 7 | 3D 改 RViz | 派生 MoveIt 的 RViz 配置 + `open_rviz.sh` + 面板按钮 | `b97db76` 起 |
| 8 | 推送云端 | `teaching-rvs-mode` 与 `main` 均为 `b97db76` | — |

## 3. 改动文件

```text
新增
  scripts/resume_teaching.sh                 一键续跑（重启栈→激活→--resume）
  scripts/recover_driver_stack.sh            驱动栈重启并等就绪
  scripts/teaching/teaching_debug.py         效率模式入口（285 行）
  scripts/teaching/straighten_in_slot.py     槽内分步转平 + 自由转动空间测量
  scripts/check_docs.py                      文档一致性检查器
  scripts/cs625_panel.py                     两模式面板（~1010 行）
  scripts/panel_scene.py                     自绘 3D（离线缩略视图）
  scripts/open_rviz.sh                       打开带 MoveIt 参数的 RViz
  config/rviz/cs625_teaching.rviz            RViz 显示配置（MoveIt 设置 + 点云）
  src/cs625_bringup/launch/teaching_rviz.launch.py   仅 RViz 的启动入口
  docs/reference/*.md (9)                    当前技术参考
  docs/decisions/ADR-005-*.md                两模式与插入权威
  docs/archive/2026-11-07-pre-reorg/         重整前原件 + MANIFEST

修改
  .vscode/tasks.json                         17 → 5
  scripts/teaching/teaching_sequence.py       4 个调试开关 + 阶段落盘 + 自动重启驱动栈
  scripts/teaching/pose_library.py            fitness<0.9 不入库
  src/cs625_bringup/launch/real_moveit.launch.py  新增 rviz_only 开关
  src/cs625_bringup/launch/real_base.launch.py    local_ip 透传
  docs/README.md                             重写为导航
  docs/agent_memory.md                       补规则 + 修 18 处旧路径
  docs/manuals/...（29 条，本次补 15–29）
```

## 4. 执行过的命令与结果

```bash
python3 -m pytest test/ -q          → 293 passed（每次提交前都跑）
python3 scripts/check_docs.py       → ALL DOC CHECKS PASSED
python3 scripts/cs625_panel.py --smoke /tmp/p.png   → 2200x1250 离屏渲染成功
python3 scripts/teaching/teaching_debug.py --stage 5 --dry-run  → 参数与归档路径正确
bash -n scripts/resume_teaching.sh && bash -n scripts/open_rviz.sh  → 语法 OK
colcon build --packages-select cs625_bringup --symlink-install     → 1.83 s 完成
ros2 launch cs625_bringup teaching_rviz.launch.py --show-args      → 参数可见
git push origin teaching-rvs-mode / main                           → 成功，两端同为 b97db76
```

## 5. 能力层与关键链状态

```text
能力层
  · 环境与依赖            PASS（source_dev_env.sh --full）
  · 机器人模型与仿真      PASS（离线）
  · 运动学/控制/规划      PASS（离线：plan_joint_locked + 速度契约）
  · 视觉/手眼/TF          NOT ACCEPTED（手眼未独立验证；RVS 为黑盒）
  · 主动视觉/NBV          NOT STARTED
  · 真机集成与安全        PASS（历史：整链到预插入跑通；今日未上机）

关键链
  URDF → Gazebo 实体                PASS（仿真，离线）
  ros2_control → joint_states       PASS（真机历史验证）
  base_link → 相机光轴 TF           NOT ACCEPTED（手眼标定未独立验证）
  RGB-D → 归一化传感器话题          PASS（历史）
  点云 → MoveIt 规划场景            PASS（历史）
  NBV 决策 → 机器人执行             NOT STARTED

仿真/实机对齐
  复用公共核心 ✅ / 仿真入口 ✅ / 真机入口 ✅ / 真机执行今日未启用 / 门禁状态正常
```

## 6. 今日发现并修掉的问题

```text
① 面板文件被并行的子智能体覆盖一次 → 冻结该子智能体后重写
② 自绘 3D 读错字段（tables 没有 center/size，只有 z + xy_min/xy_max）→ 桌子根本没画出来
③ slot/工件在模型里只有位姿没有尺寸 → 改为坐标轴标记，不编造盒子
④ 单独 rviz2 -d 启动拿不到 robot_description → 改为走 teaching_rviz.launch.py（带 MoveIt 参数）
⑤ agent_memory 与手册里 18 处路径指向重整前位置 → 全部改写，活文档引用 100% 可解析
⑥ --resume 参数解析了但没接线 → 补接并加断言
```

## 7. 诚实声明（未完成 / 未验证）

```text
· 效率模式、straighten_in_slot、装配搜索：均未在真机验证
· 插入段仍未在真机完成（372 mm 不是物理底部，三面接触）
· 槽内 10° 转平：源码依据已确认（师兄 rotated = insert @ Ry(-10°)），但未实机跑通
· 六维力传感器未安装 → B 类指标全部"未采集"
· 自研视觉管线未开始（十一月主线）
· RViz 修复后尚未在有机器人的情况下实际观察（本次只验证了参数可见与环境构建）
```

## 8. 下一步（唯一推荐）

明天上机：起驱动栈 → 点面板「打开 RViz」→ 确认机械臂模型、TF、点云、规划场景都能显示；
确认后再做视觉采集（停 RVS、启 percipio 相机、采 20 组静态点云）。

---

## 9. 续（2026-11-08 凌晨）：面板、RViz、相机

> 所有者要求：这一段也并入 11-07 的工作日志，不另算一天。

### 9.1 做的事

```text
① 3D 由 RViz 负责，面板只显示状态卡
   · 找到根因：`rviz2 -d config` 单独启动拿不到 robot_description
     （报 "Could not find parameter robot_description ... within 10 s"，场景只剩空坐标系）
   · 修复：real_moveit.launch.py 增加 rviz_only:=true（跳过 move_group、保留全部参数），
     新增 teaching_rviz.launch.py 薄封装，open_rviz.sh 改为走它；
     显示配置 config/rviz/cs625_teaching.rviz（MoveIt 显示 + /sensors/camera/points）
   · 面板场景区改为「3D 状态卡」：四项数据源（/joint_states、/compute_fk、
     /sensors/camera/points、/robot_description）+ 诊断 + 打开 RViz
② 相机嵌入面板：1:1 原始像素、不缩放、不滚动
   · scripts/panel_camera.py：best-effort QoS 订阅 color/depth，深度 mm→灰度线性映射
   · 图像尺寸反过来抬高窗口最小尺寸（宽 = 图宽 + 1000，高 = 图高 + 220）
   · 两个标签页统一由主窗口连接尺寸信号，构造时立即应用一次
③ 六维力：六格数字 → 整行曲线（六轴彩色轨迹 + 0 轴 + 图例），与关节力矩同高 178 px
④ 阶段表新增「说明」列（每阶段真实约束），列宽固定、数值列右对齐
⑤ 夹爪控制面板：状态 / 初始化 / 张开 / 闭合（调 scripts/teaching/gripper.py）
⑥ 相机控制：面板「相机 ▾」菜单（fast / modeling / high-2560 / 停止 / 状态），
   camera_mode.py 新增 stop 与 high 子命令
⑦ 运行记录表格列宽固定、表头与数据同侧；「最近一次运行」摘要延伸到底部
⑧ 效率模式：相机占右列上方整块；3D 状态与运行记录并排
```

### 9.2 修掉的 bug

```text
· RuntimeError: Context.init() must only be called once
  → 相机与状态两个 rclpy 线程共享进程，改为只在未初始化时 init、只由初始化者 shutdown
· RuntimeError: Executor is already spinning
  → 两个线程并发 spin_once，加模块级锁串行化
· 自绘 3D 读错字段（model.json 的 tables 是 z + xy_min/xy_max，没有 center/size）→ 桌子未画出
  → 结论：不自己造 3D，删掉自绘模块，交给 RViz
· 效率模式相机在构造期发尺寸信号，主窗口尚未建立 → 回调落空、图像被裁
  → 统一由窗口连接并立即应用一次；离屏实测 1400×900 启动会自动撑到 2280×1180
```

### 9.3 验证

```bash
python3 -m pytest test/ -q                → 293 passed（每次提交前都跑）
python3 scripts/check_docs.py             → ALL DOC CHECKS PASSED
离屏渲染 3000×1700 / 1400×900             → 无裁切；窗口自动撑到能放下整幅相机图
相机 1:1（占位图 1280×960）               → 画布 1282×962、滚动条不可见
```

### 9.4 仍未验证

```text
· RViz 修复后未在有机器人的情况下实际观察
· 相机未接真机（面板显示占位图）
· 效率模式分阶段执行、straighten_in_slot、装配搜索仍未上机
· 六维力曲线只有"未采集"状态（传感器十二月到位）
```

### 9.5 当日新增提交（11-07 夜 → 11-08 凌晨，同一条工作线）

```text
2ec4ce6 自动重启驱动栈 + 槽内分步转平
f10b576/3084cc6 --resume 接线 + 文档检查器
5f43186 一键续跑 + 5 秒通道看门狗 + 位姿库 fitness
c048322 任务收敛 17→5
76c762e 效率模式（分阶段 + 归档 + 18 项指标）
8776ccc/b602b2f/fcd0b5c 文档重整与合并
8d862be/69ceb3d/4ec195c/af726ef 面板（状态卡、相机、夹爪、三栏）
6a8c.../93a...b97db76 RViz 接入 + 文档路径修正
cbf2a8a 工作日志
b97db76→da7e23b 面板迭代与相机 1:1 修复
477b349 交接文档与 README 索引
```
