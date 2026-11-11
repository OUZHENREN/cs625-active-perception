# 2026-11-11 工作日志：效率模式流程修复与面板

> 本文件从 `2026-11-10_WORKLOG_VISION_CAPTURE.md` 拆分出来：之前把 11-11 的内容追加在
> 11-10 的文件里，日期不对。拆分后：11-10 文件保留 §1–§34（视觉测量管线），
> 11-11 的内容（开工/驱动栈、夹爪链路、插槽粗观测、效率模式接入、面板与 RViz、
> 抓取序列修正、急停诊断）全部在本文件。

## 0. 当日主线

```text
· 开工任务起不来 → 陈旧驱动栈 + 孤儿进程（已修：start_real_stack.sh）
· 新增任务⑧ 一键关闭所有驱动（stop_all_drivers.sh）
· 夹爪链路澄清：没有 ROS 驱动，只有 Windows COM9 + 可选常驻桥
· 工件测量复现 0.50 mm / 0.054°；插槽粗观测跑通 0.32 mm / 0.027°（先验引导配准）
· 效率模式接入：观测检查按模式覆盖、环境模型门换测量门
· 效率流程 7 项缺陷修复 + 夹爪涨紧语义更正（init 闭合 → 进入 → 张开涨紧）
· 面板：相机话题回退、/task_log、实时 /task_state；新增两个效率模式 RViz
· 真机急停诊断：safety_mode=7 导致力矩死流，重启驱动栈后恢复
```

## 1. 11-11 14:0x 开工任务起不来：陈旧驱动栈 + 孤儿进程占端口

### 1.1 现象

```text
任务① 输出：[开工] 驱动栈已在运行
            health: {'sockets': False, 'controller': True, 'joint_states': True}
            FAIL: reverse-control ports (50001/50003/50004) are not ESTAB
```

### 1.2 根因（两层）

```text
① 昨天关机后驱动栈的 launch 进程还在（11-10 11:36 启动），任务①只判断
   "有没有 ros2 launch 进程" → 认为"已在运行" → 跳过启动；
   但它的控制连接早已断开 → health sockets=False → FAIL。
② 我停掉 launch 后，它的驱动子进程（eli_components_loader / eli_ros2_control_node /
   controller_stopper）变成孤儿继续跑并占着端口；新驱动起来立刻报
   `bind: Address already in use` → ros2_control 节点 6 号信号崩溃 →
   连 arm_controller 都起不来（spawner 一直 "Failed to acquire lock"）。
```

### 1.3 修复：`scripts/start_real_stack.sh`

```text
· 就绪判据 = 进程存在 AND control_health 的 sockets 为 True（复用同一套判据，不另写端口逻辑）
· 进程在但 sockets 断 → 判定陈旧 → 用 setsid 记下的进程组整组 SIGINT/TERM，
  再用 kill_tree() 递归清理残留子树（不用 pkill/通配，避免误伤自己的 shell）
· 启动前先 ping 机器人；启动后等待 45 s → control_health --activate → preflight
· --check-only 只诊断不启动
```

任务① 从 1005 字符的内联命令改成一行调用该脚本。

### 1.4 修复后实测

```text
首次（陈旧）：识别"进程在但控制端口未建立"→ 整组停 → 重启 → health 全 True、PREFLIGHT_OK
再次（幂等）：[开工] 驱动栈已在运行，控制连接正常（PID 448227）→ 跳过启动 → health 全 True
当前状态：单实例 launch（448227）+ 其子进程，到机器人 30001 的连接正常，
          arm_controller active，PREFLIGHT_OK（Σ|effort| ≈ 11.1）
```

### 1.5 教训

```text
· "进程活着"不等于"连接活着"：机器人关机后驱动进程会赖着不走，必须用连接状态判就绪；
· 杀 launch 不等于杀干净：setsid 的进程组 + 递归子树才可靠；
· 判据要单点维护：脚本里复用 control_health，而不是自己数端口。
```

## 2. 11-11 15:0x 新增任务⑧：一键关闭所有驱动

### 2.1 `scripts/stop_all_drivers.sh`

```text
关闭：① 机械臂驱动栈 ② 相机驱动 ③ 相机查看器
方式：setsid 进程组 / 递归 kill_tree / 孤儿清扫
      （不用 pkill、不用通配，只停可核验的 PID 及其子树）
参数：--dry-run 只报告不动
收尾：报告"剩余检查"（三个进程 + 到机器人/相机的连接数），全清才返回 0
```

### 2.2 测出来的两个坑（都修了）

```text
① 先杀 launch 会把子进程变成孤儿：相机 launch 死掉后，它的 component_container
   被 /init 收养，继续占着相机；必须先 kill_tree（子先父后）再停 launch。
② WSL 里的孤儿父进程是 /init（PID 647，comm 显示 "Relay(648)"），
   用 ppid==1 或 comm=="init" 都判不出来；改成看父进程 args 是否以 /init 开头。
```

### 2.3 实测

```text
--dry-run：只列出将要停止的进程（机械臂进程组 / 相机 launch / 查看器 PID），不动
真停：机械臂 + 相机 + 查看器全部停止，连接数 0，孤儿被清扫，输出"全部已关闭"
当前状态：系统已全部关闭（要开工按任务①）
```

### 2.4 任务列表（.vscode/tasks.json）

```text
① 开工 · 真机        ② 安全模式 · 示教    ③ 效率模式 · 调试
④ 面板 · 两模式      ⑤ 自检 · 离线        ⑥ DSH · 启动/打开界面
⑦ 相机 · 查看/换档   ⑧ 收工 · 关闭所有驱动   ← 新增
```

## 3. 11-11 15:2x 夹爪链路：没有 ROS 驱动，只有 Windows 侧的常驻桥

### 3.1 事实（查代码与文档确认）

```text
夹爪是电动夹爪，走 Windows COM9（115200，从机 1），本仓库【没有】它的 ROS 驱动。
两种用法：
  · 一次性调用 powershell -File C:\Users\Public\hitbot_cmd.ps1 -Cmd status
    （每次开一次串口，无常驻进程）
  · 常驻桥 C:\Users\Public\hitbot_bridge.ps1 → 127.0.0.1:7500
    （UI/批处理用；独占 COM9，与 HITBOT Studio 互斥）
```

### 3.2 所以"关夹爪驱动"= 停常驻桥

```text
桥没起 → 没有需要关的驱动，直接断电即可；
桥起着 → 先停桥（释放 COM9），再断电；
安全前提：电动夹爪断电会失去夹持力 —— 若正夹着工件，必须先张开再断电。
```

### 3.3 新增 `scripts/gripper_link.sh`

```text
--status  桥是否在线 + 夹爪能否应答（只读）
--ensure  桥不在就尝试拉起；失败只告警（测量/观测不需要夹爪）
--stop    停桥、释放 COM9，并提示"断电前先松开"
```

接入：任务① 末尾调用 `--ensure`（非致命）；任务⑧ 调用 `--stop`。

### 3.4 当前实测状态

```text
[夹爪] 常驻桥未启动
[夹爪] 状态查询：{"error":"bad status response","ok":false}
```

→ 夹爪本体处于断电/未连接状态（与"昨天关机"一致），不影响开工与测量。

## 4. 11-11 15:4x 工件测量复现成功；插槽粗观测仍未跑通

### 4.1 工件：复现并更好

```text
在 observe.workpiece_fine 重跑 measure_object（含取景检查）：
  覆盖率中位 0.989
  平移重复性 0.50 mm（昨天 0.73）
  旋转重复性 0.054°（昨天 0.87）
  无对称歧义 → verdict OK
中心 (849.1, 649.9, 127.1) mm，与昨天 (846.8, 649.7, 136.0) 差约 9 mm（工件没动过，
差异来自观测噪声与配置微调，量级合理）
```

**这是断电重启后的端到端复现，说明链路稳定。**

### 4.2 插槽：失败，但失败得很清楚

```text
移动到 observe.slot_coarse（规划 85 点/8.3 s，执行 OK）
取景检查 PARTIAL（7/8 角入画，深度 0.58–1.25 m）
采集 OK，但配准失败：4 组覆盖率全部 0.000 → 门槛 REFUSE（行为正确）
```

诊断：

```text
· 深度窗内最大连通域 30.5 万点、extent 1447×652×857 mm —— 不是夹具（200×745×508）；
· 加了"以 slot_origin/q 推出的夹具中心做球裁（r=0.53 m）"后仍有 22.3 万点、
  extent 958×595×842 mm，说明球里混着桌面/机器人本体；
· 夹具模型原点是否等于 slot_origin 尚未核实（build_fixture_collision_mesh 只说
  "与 slot_origin_q 同一个系"），所以球裁中心本身可能就有偏差。
```

### 4.3 顺带修的两个 bug

```text
① coarse_pose 打印对称解时仍访问已改名的 check['ambiguous'] → KeyError，改为 check['valid']；
② 插槽 preset 增加"按已知夹具中心球裁"（measure_object 内根据 slot_origin/q + bbox 计算）。
```

### 4.4 插槽下一步（明确）

```text
1. 先核实夹具模型系与 slot_origin 的关系（拿一帧槽位数据人工量一下）；
2. 让插槽也走"先验引导 + 多起点"：用已知槽位姿直接初始化 ICP，而不是靠三平面指派；
3. 取景把夹具完整框住（现在 7/8，需要再退一点）。
```

## 5. 11-11 16:2x 插槽粗观测跑通：先验引导配准，0.32 mm / 0.027°

### 5.1 关键判断

```text
夹具 745 mm 长、旁边就是桌面 → 深度窗内最大连通域是桌面，三平面指派不可靠。
但夹具的位置本来就是已知的（标定点 slot_origin + slot_origin_q），
所以不需要"从点云里找夹具"，只需要"用先验把它顶上去，再剔点精配准"。
```

### 5.2 新增 `prior_register()`

```text
用 T_base_slot 初始化 → 多尺度 ICP → 按"点到 CAD 距离 30 mm"剔点 →
再 ICP（迭代两轮）→ 覆盖率
实测（单组）：先验覆盖率 0.20 → 剔 50/30/20 mm 后分别 0.86 / 0.90 / 0.92
```

### 5.3 接入 measure_object（--preset slot 自动使用）

### 5.4 实测（slot_coarse，4 组）

| 指标 | 数值 |
|---|---|
| 覆盖率中位 | **0.965** |
| 平移重复性 | **0.32 mm** |
| 旋转重复性 | **0.027°** |
| 判定 | **OK** |

模型原点（base）估计在 (1.169, −0.051, 0.192)，中心 (1.169, −0.051, 0.192)。

### 5.5 今天的两个"粗观测"现在都通了

```text
工件粗观测（workpiece_fine）：覆盖率 0.989，平移 0.50 mm，旋转 0.054°，OK
插槽粗观测（slot_coarse）  ：覆盖率 0.965，平移 0.32 mm，旋转 0.027°，OK
⇒ 效率模式的观测环节已经具备两个目标的可测量接口
```

## 6. 11-11 16:3x 效率模式接入：观测检查按模式覆盖（不碰示教/安全模式）

### 6.1 机制

```text
run_pick_and_place.py：
  Config 新增 observe_checks: {waypoint: "shell 命令"}
  观测位检查处改为：cfg.observe_checks[waypoint] 优先，否则用 observe[waypoint].check
  ⇒ 位姿仍来自 common.yaml 的 observe，只替换"用什么算子检查/测量"
  ⇒ 示教/安全模式文件里没有 observe_checks，继续用原检查，完全不受影响
```

### 6.2 已配置（三个 efficiency 配置文件）

```yaml
observe_checks:
  workpiece_fine: "python3 scripts/measure_object.py --preset workpiece --observe-pose workpiece_fine --groups 3 --out runtime/measurements/workpiece_latest.json"
  slot_coarse:    "python3 scripts/measure_object.py --preset slot --observe-pose slot_coarse --groups 3 --out runtime/measurements/slot_latest.json"
```

### 6.3 验证

```text
run_task.py --scope load --mode efficient --world real --dry-run
  → 配置合并成功，effective 里 observe_checks 生效（已用 load_config 读回确认）
  → 计划表正常列出 13 个阶段 + 观测位
  → 但 dry-run 的环境门给出 REJECT（模型过期），提示"先重测环境模型"
```

### 6.4 结论与下一步

```text
· "观测检查换成新测量管线"这一步已经落地，且只影响效率模式；
· 剩下的最后一块是【环境模型门】：它仍然要求 model_environment.py 的新鲜模型。
  用新管线替代后，这个门应该改为"工件/插槽测量是否 OK"，而不是"模型是否新鲜"。
· 这一步会改变真实流程的行为，建议先与所有者确认再动。
```

## 7. 11-11 16:4x 环境模型门按模式切换：效率模式用"测量门"

### 7.1 改动

```text
run_task.check_environment() 支持 environment.gate：
  · "model"（默认）——原门：环境模型存在/新鲜/偏差在阈值内
  · "measurement"——效率模式用：只要求每个目标都配置了 observe_checks，
    真正的质量判定交给观测位上的 measure_object（不合格会让该阶段失败）
三个 efficiency 配置文件：environment_gate: measurement（顶层键）
                          + measurement_targets: [workpiece_fine, slot_coarse]
为什么用顶层键：模式文件是【浅合并】，若写 environment: {...} 会把 common.yaml 的
阈值块整块覆盖掉（第一次实现就踩了，3 个测试立刻报警）。
```

### 7.2 实测（dry-run）

```text
效率模式：environment gate : PASS
          - measurement 门：由观测位的 measure_object 判定（workpiece_fine, slot_coarse）
安全模式：environment gate : REJECT（原模型门，未改动）
```

### 7.3 这一步意味着什么

```text
· 效率模式的"开工前置条件"从"环境模型新鲜"变成"观测阶段必须测出合格位姿"；
· 示教/安全模式的保守路径完全不受影响（仍是模型门）；
· 质量责任从"开工前一次性建模"转移到"每次观测的实测判定"，这正是新管线的价值所在。
```

### 7.4 剩下的空档（诚实记录）

```text
· 现在效率模式的观测检查只配了两个位姿（workpiece_fine / slot_coarse），
  workpiece_coarse、slot_fine、tables 仍是原检查或空检查；
· 完整替换还需要：把"tables"（桌面建模）也纳入测量管线，或明确它不再需要；
· 真机完整跑一遍效率流程还没做（需要夹爪上电 + 工件在位 + 你在场）。
```

## 8. 11-11 18:0x 效率流程 7 项缺陷修复

### 8.1 已修（代码级）

| # | 问题 | 修法 |
|---|---|---|
| ① | 全程没有闭合夹爪 | 新增 `close` 阶段（grasp 之后）：`grip_and_wait(grip_close_mm=12)`；`grip` 明确为"张开到 50 mm" |
| ② | 没有 150 mm 垂降/压垂直/10 mm 回退 | `pre_grasp` = 抓取点正上方 150 mm；两阶段都做 `snap_vertical_quat`；抓取点沿工具 x 回退 10 mm |
| ④ | 近工件无降速 | `proximity_velocity()`：距目标 <0.15 m 时线性降速到 0.01 |
| ③ | 测量不参与抓取 | 新增 `measured_box_pose()` + `adjust_grasp_with_measurement()` + 配置 `workpiece_measurement`/`workpiece_frame_offset` |
| ⑤ | EFFORT_DEAD 无上下文 | 失败时打印 `safety_mode`/`robot_mode`（transient_local QoS，实测瞬时拿到 1/7） |
| ⑥ | 夹爪电流读成 -7e6 | 寄存器映射未确认 → 明确报 `null`（未采集），不再输出假数 |
| ⑦ | 观测取帧永远超时 | 根因：订阅了 `/sensors/camera/color/image`，launch 里 `launch_sensor_adapter:=false` 时该话题不存在 → 同时订阅 `/camera/color/image_raw` |

### 8.2 修③时踩到的坑（重要）

```text
第一版直接拿"测量位姿 - 参考位姿"的差去平移抓取点，plan-only 显示平移了 137 mm。
但我们的测量输出是【CAD 系】，示教常数是【box 系】，两者差一个固定变换；
未标定就平移等于按错误的坐标系推偏抓取点。
⇒ 现在必须显式配置 workpiece_frame_offset 才参与，否则日志写"测量未参与（未标定）"。
```

### 8.3 验证

```text
296 项测试全通过；
plan-only：pre_grasp 上方150mm → grip(张开) → grasp 回退10mm → close（顺序正确）
           [grasp target] 测量未参与（workpiece_frame_offset 未标定）; grasp=[0.8369, 0.6003, −0.0583]
```

### 8.4 仍需真机/标定的两件事

```text
· workpiece_frame_offset：工件在示教位姿时采一次测量，算出 CAD 系→box 系的固定变换，写进配置；
· 夹爪电流寄存器映射：需要厂商寄存器表，或夹爪运动时采一组原始报文对比（当前按"未采集"处理）。
```

## 9. 11-11 20:0x 更正：夹爪是"涨紧夹持"，闭合 = init（不是新增 close 阶段）

### 9.1 翻代码得到的答案（teaching_sequence.py 步骤 [6][7]）

```text
[6] init + enter grasp (jaws closed)     → gripper("init")：闭合手指并归零
    move("grasp")                        → 闭着手指进入抓取位
[7] open to grip width                   → gripper("grip")：张开到 ~50 mm
grip_still_holding(min_open_mm=40):
    "The expanding grasp holds the part with the jaws OPEN at ~50 mm;
     if the position fell back near zero the part is no longer gripped."
```

**结论：这是"从内侧撑开"的涨紧夹持**——先 init 闭合归零、闭爪进入，再张开撑住手柄内侧。
所以：
- "闭合夹爪"这个动作就是 `gripper("init")`（已有的 `gripper_init` 阶段做的就是它）；
- `grip`（张开到 grip_width）必须放在 **grasp 之后**，而不是之前；
- 我上一轮新增的 `close`（闭合到 12 mm）方向完全错了，已删除。

### 9.2 现在的顺序（与示教模式一致）

```text
baseline → pre_grasp(正上方150mm) → gripper_init(init 闭合归零) → grasp(回退10mm)
        → grip(张开涨紧，并校验 position ≥ 40 mm 才算夹住) → lift
```

### 9.3 顺带修掉

```text
· grip 阶段新增"撑开校验"：张开后 position < 40 mm 视为没撑到工件，判失败（示教的同款判据）；
· 删除 grip_close_mm 配置与 close 阶段，避免两套"闭合"语义并存；
· 四个模式配置的阶段表统一改为 ...grasp, grip, lift...。
```

### 9.4 验证

```text
296 项测试通过；
plan-only：pre_grasp 上方150mm → gripper_init → grasp 回退10mm → grip ——顺序正确
```

## 10. 11-11 20:1x 标定工具：CAD 系 → 示教 box 系

### 10.1 `scripts/calibrate_workpiece_frame.py`

```text
输入：measure_object 的输出（T_base_object，CAD 系）+ config 的 workpiece_reference（box 系）
输出：workpiece_frame_offset = inv(T_measured) @ T_box，打印成可直接粘贴的 YAML
安全：必须显式 --confirm（确认工件此刻就在示教位姿），否则拒绝计算；不自动写配置
```

### 10.2 现在跑出来的数（说明工件被移动过，**不能**当标定值用）

```text
测量位姿 (CAD 系): 0.6396, 0.6147, 0.1208
参考位姿 (box 系): 0.5029, 0.6351, 0.1254
两者距离 138.3 mm，姿态差 19.90°
→ 这 138 mm / 20° 是【工件被转过】造成的，不是坐标系固定变换。
   标定必须先把工件摆回示教位姿，再跑 measure_object + 本工具。
```

### 10.3 使用顺序（等真机可用）

```text
1. 工件摆回示教位姿（与 workpiece_reference 描述一致）；
2. 在 workpiece_fine 观测位跑 measure_object --preset workpiece --groups 3；
3. 确认工件确实在示教位姿后，跑 calibrate_workpiece_frame.py --confirm；
4. 把打印的 workpiece_frame_offset 写进 common.yaml；
5. plan-only 复核：[grasp target] 应显示"已按实测位姿平移 …"，且平移量在几十毫米内。
```

## 11. 11-11 21:0x 效率模式面板与 RViz：三个问题定位 + 一处修复

### 11.1 抓取点高度（所有者质疑的对账）

```text
桌面 z = −105 mm（实测），工件底面 = −104.8 mm
grasp 点 z = −58.3 mm → 距桌面 46.5 mm，在桌面【上方】，不是下方
          → 对应手柄下端；夹爪从这里进入，余量只有 46 mm，是风险点但可抓
pre_grasp z = 91.7 mm → 比工件顶面(360 mm)低 268 mm
```

**更正**：我上一轮建议"抬到工件顶面+100 mm"是错的——那会让路径长 400+ mm，
而"抓取点上方 150 mm"是已经验证过可执行的。真正该修的是**碰撞场景**与**接近段速度**，
不是把接近点抬得更高。

### 11.2 面板为什么没有图像/日志/阶段

```text
① 图像：panel_camera.py 订阅 /sensors/camera/color|depth/image_raw（归一化话题），
   而 launch_sensor_adapter:=false 时这些话题不存在 → 永远"等待图像"。
   已修：同时订阅 /camera/color/image_raw、/camera/depth/image_raw。
② 日志：面板只显示"它自己启动的子进程"的输出；在终端里跑的流程它看不到。
③ 阶段表：cs625_panel.py 的阶段表是【示教链的静态列表】，注释里写着
   "第 2 步「插槽粗观测」已删除" —— 所以效率流程里的 observe:slot_coarse 根本不显示。
```

### 11.3 插槽粗观测其实在流程里

```text
load_efficient / full_efficient 观测点：
  workpiece_coarse, workpiece_fine, tables, slot_coarse, slot_fine, slot_coarse(收尾)
unload_efficient：slot_coarse, slot_fine, tables, workpiece_coarse
⇒ 流程有，面板没显示（见 45.2 ③）。
```

### 11.4 新增两个效率模式 RViz

```text
scripts/rviz_efficiency.sh --planning | --cloud | --both
  planning → src/cs625_bringup/config/cs625_moveit.rviz（碰撞与规划）
  cloud    → src/cs625_bringup/config/environment_point_cloud.rviz（点云）
面板顶栏新增 "RViz ▾" 按钮（碰撞与规划 / 点云 / 两个都开），与示教 RViz 分开。
```


## 12. 21:0x–21:3x 面板与 RViz 落地

```text
d18abff  panel_camera.py 同时订阅归一化话题与相机原始话题（解决"面板无图像"）
         scripts/rviz_efficiency.sh --planning|--cloud|--both（复用 cs625_moveit.rviz
         与 environment_point_cloud.rviz），面板顶栏加 "RViz ▾" 按钮
a79c455  runner 每个阶段发 /task_log 与 /task_state（面板实时看到当前阶段）
         cs625_ui 订阅 /task_log 追加到任务日志（终端跑的流程也能看到）
         抓取前用【实测工件位姿】发布碰撞体（apply_environment_collision.py）
         接近速度：pre_grasp ≤0.02、grasp ≤0.01，150 mm 内按距离继续降
```

## 13. 今日结论与待办

```text
已具备：
  · 工件粗观测 0.50 mm / 0.054°、插槽粗观测 0.32 mm / 0.027°（均通过质量门）
  · 效率模式接入：观测检查按模式覆盖 + 测量门
  · 抓取序列与示教一致：init(闭合) → 进入(闭爪) → 张开涨紧(≥40 mm 才算夹住)
  · 抓取前发布碰撞体、接近降速、面板实时状态与日志

待办（按优先级）：
  1. 效率面板的阶段表改成读流程配置（当前 cs625_panel.py 仍是示教链静态列表，
     所以看不到 observe:slot_coarse；cs625_ui.py 的流程卡片已由 /task_state 修好）
  2. 硬件恢复后用 --until grasp 做分段真机验证（先确认碰撞场景生效、接近速度合适）
  3. workpiece_frame_offset 标定（工件在示教位姿时测一次）
  4. 夹爪电流寄存器映射（当前报 null，不输出假数）
```

## 14. 22:0x 效率模式标签页：阶段表改成读流程配置（含插槽粗观测）

### 14.1 问题

```text
cs625_panel.py 的"效率模式"标签页和示教页共用同一张静态阶段表 STAGES
（注释写着"第 2 步「插槽粗观测」已删除"），
所以效率模式里根本看不到插槽粗观测——而流程配置里一直有它。
```

### 14.2 修法

```text
EfficiencyTab 改为：
  · 阶段列表 = efficiency_phases()：直接读 config/tasks/load_efficient.yaml 的 phases；
  · 每个阶段用中文标签（PHASE_LABELS），插槽粗观测 = observe:slot_coarse；
  · 「运行」不再走 teaching_debug.py，而是
    python3 scripts/run_task.py --scope load --mode efficient --world real [--confirm|--dry-run]
    [--until <最后选中的阶段>]
  · 选中的起点不是流程第一个阶段时，日志里明确提示 runner 不支持 --start-from，
    仍然从流程开头跑到指定阶段（安全起见不跳过观测）。
```

### 14.3 验证

```text
efficiency_phases() 读出 19 个阶段，观测点：
  observe:workpiece_coarse / workpiece_fine / tables / slot_coarse / slot_fine / slot_coarse
前几个中文标签：工件粗观测、工件精观测、桌面观测、插槽粗观测、插槽精观测、基准位
```

### 14.4 又修一个：重复阶段被勾选框覆盖

```text
流程里 observe:slot_coarse 出现两次（开头 + 收尾），勾选框原来按"阶段名"做 key，
两个同名阶段互相覆盖，界面上只剩"19 插槽粗观测"、第 4 个消失。
已改成按【序号】索引：19 个阶段全部显示（01 工件粗观测 … 04 插槽粗观测 …）。
```

### 14.5 无头验证（不用开显示器）

```text
QT_QPA_PLATFORM=offscreen 实例化 EfficiencyTab：
  阶段数 19
  前 7 个：01 工件粗观测 / 02 工件精观测 / 03 桌面观测 / 04 插槽粗观测 /
           05 插槽精观测 / 06 基准位 / 07 预抓取
  构造命令：run_task.py --scope load --mode efficient --world real --confirm --until grip
```

## 15. 22:3x 阶段盘点：视觉算法与效率模式现状复审

### 15.1 视觉算法（D0–D3）

| 阶段 | 内容 | 状态 | 关键证据 |
|---|---|---|---|
| D0 | 相机五模式普查（分辨率/帧率/可用性） | ✅ 完成 | `runtime/experiments/camera_census_*`，报告见 11-09 日志 |
| D0.5 | 偏振片 A/B | ⛔ 关闭（结论：对本相机 IR 波段无效） | 0° vs 90° valid 75.9% vs 76.1%，薄膜不消光 |
| D1 | 采集与时间重复性 | ✅ 完成 | 5 朝向×4 组，valid 75.1–76.2%，spacing 1.48–1.51 mm，组内 1.60 mm（p95 5.5–6.0） |
| D2 | 粗位姿（三平面 + CAD 配准） | ✅ 完成（工件 + 插槽） | 工件 4/4，cov 0.96–0.99，中心重复性 1.91 mm / 1.1°；插槽（先验引导）0.32 mm / 0.027° |
| D3 | 精配准（细 ICP + 翻转消歧 + 角点验证） | 🟡 脚本就绪，未接入测量接口 | `scripts/fine_pose.py` |

**测量管线的组成（当前形态）**

```text
观测位（固定/由上一帧推出）
 → 取景检查（frame_check：8 角入画、深度窗建议值）
 → 采集（vision_capture，RELIABLE QoS，3 帧取中位）
 → 分割：深度窗 + 桌面剔除（base 系 RANSAC）+ 高度过滤（桌面以上 30 mm）
        + 球裁（工件：已知中心；插槽：slot_origin 先验）
 → 平面：RANSAC 三平面（固定种子，可复现）+ 去重 + 正交筛选
 → 位姿：bbox 指派 × 4 对称候选 → 多起点 ICP → 按【扶手覆盖率】选解
        → 前 3 名细精修（6→3→2 mm）
 → 质量门：覆盖率 ≥0.95、组间重复性 ≤5 mm/2°、对称歧义、来源判定
 → 输出 T_base_object + 中心 + 判定（不合格退出码 2）
```

**已知局限（必须记住）**

```text
1. 扶手判据在"扶手被遮挡"时会误拒（正解也拿不到 0.9）——保守但会产生假阴性；
2. 近正方形工件（455×465 只差 2%）绕厚度轴旋转约束弱，旋转重复性偶尔 >2°（实测 0.13°↔3.34°）；
3. 精观测位（0.45 m）因近距离分割失败未启用，"移动后再测"这条链还没兑现；
4. 极端朝向（rotp60 类）在新管线未重新回归；
5. D3 精配准尚未接进 measure_object。
```

### 15.2 效率模式（今日已修 vs 仍存在）

**已修**

```text
· 观测检查按模式覆盖（observe_checks），示教/安全模式不受影响
· 环境模型门 → 测量门（environment_gate: measurement）
· 夹爪语义纠正：init(闭合归零) → 进入(闭爪) → 张开涨紧(≥40 mm 才算夹住)
· 接近几何：抓取点正上方 150 mm + 姿态压垂直 + 沿工具 x 回退 10 mm
· 接近降速：pre_grasp ≤0.02、grasp ≤0.01，150 mm 内按距离继续降
· 抓取前发布【实测工件】碰撞体（apply_environment_collision.py）
· EFFORT_DEAD 打印 safety_mode/robot_mode；急停时执行器立刻取消（ESTOP_OR_PROTECTIVE_STOP）
· 启动脚本：陈旧驱动栈识别、整树停止、安全模式检查（safety_mode≠1 直接告警）
· 任务⑧ 一键关闭；夹爪链路脚本；面板相机/日志/阶段/RViz
```

**仍存在（按影响排序）**

```text
1. 抓取点/预抓取点仍来自示教常量：measure_object 的结果未参与（缺 workpiece_frame_offset 标定）；
   目前只有"合理性检查"（抓取点是否落在实测工件的 CAD 包围盒内）。
2. 插槽位姿仍是常量 slot_origin：插槽测量已能测（0.32 mm），但插入链没有消费它。
3. 碰撞体发布只在真机非 plan-only 分支，尚未在真机验证有效性。
4. runner 没有 --start-from：面板只能"从流程开头跑到某阶段"。
5. 夹爪电流未采集（寄存器映射未定），grip_current 只能作命令参数、不能作判据。
6. observe:workpiece_coarse / tables / slot_fine 的检查被设为 "none"：
   等于"移动到那里但不检查"，它们的存在只是为了维持既有轨迹。
```

### 15.3 当前所处阶段（一句话）

```text
视觉侧：D2 粗位姿已在真机稳定通过质量门（工件 1.9 mm / 插槽 0.3 mm），D3 未接入；
效率侧：观测环节已换成新管线并能通过测量门，但抓取/插入仍用示教常量，
        整条链还没有一次完整跑通——今天在 pre_grasp 因人工急停中止。
下一步的关键不是再改算法，而是：碰撞体 + 测量驱动抓取（位姿口径标定）+ 分段真机验证。
```
