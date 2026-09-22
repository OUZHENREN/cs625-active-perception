---
title: "屏蔽片模块插槽任务：从 SolidWorks 导出到 Gazebo 模型"
date: 2026-09-19
type: WORKLOG
project: cs625-active-perception
tags: [CS625, 精密光学模块, 屏蔽片模块, 插槽, 楔形槽, Gazebo, 模型, 质量属性]
---

# 屏蔽片模块插槽任务：从 SolidWorks 导出到 Gazebo 模型

## 一、目标与结论

把用户提供的 SolidWorks 导出（插槽架子 + 弹仓/屏蔽片模块）变成可用的
Gazebo / MoveIt 模型，并解出两者在装配体坐标系下的精确相对位姿。

**结论：几何、质量、惯量、位姿全部拿到，两个模型已入库并通过一致性校验。**
唯一遗留是把夹具镜像进 MoveIt planning scene 和契约测试（下一步）。

## 二、关键结果

### 2.1 弹仓在装配坐标系下的位姿（精确解，非拟合）

```
R = [[ 0,  -0.994521895, -0.104528465],     t = [0.1480452, 0.2085, -0.0143114] m
     [ 1,   0,             0           ],
     [ 0,  -0.104528465,   0.994521895]]     det(R) = 1.000000000
```

解法：装配体 STL 的三角形数**精确等于**两份单独导出的和
（150 114 + 276 642 = 426 756），说明 SolidWorks 对每个零件只三角化一次并
在各次导出中复用。于是用三角形边长签名做 1:1 匹配 → Kabsch 求刚体变换 →
迭代剔除离群点收敛。

**独立验证**：把整个弹仓用该位姿放入装配体，全部顶点到装配网格的最近距离

```
中位数 0.008 µm   90 分位 0.014 µm   最大 0.027 µm   100% 顶点 < 1 µm
```

这是 float32 精度下的恒等关系，不是最小二乘拟合。

### 2.2 槽是楔形，不是平行槽

`0.104528465 = sin 6°`。拟合槽壁平面后，槽壁与弹仓面夹角 **12.62°**。
沿插入方向射线投射得到的间隙：

| 项目 | 测量值 |
|---|---|
| +Y 面命中槽壁 | 300 / 900 |
| −Y 面命中槽壁 | **0 / 816**（另一侧无壁） |
| 间隙范围 | **1.008 → 14.28 mm** |
| 沿插入轴分布 | Z∈[80,160) 时中位 4.36 mm |

因此初始"平行槽 + 单边 2 mm"的工作假设不成立——**是假设错了，不是装配体错了**。

### 2.3 质量属性（实测，非估算）

| | 体积 | 质量 | 等效密度 |
|---|---:|---:|---:|
| 弹仓 | 3084.430 cm³ (SolidWorks) | **19.0 kg** | **6295 kg/m³** |
| 插槽架子 | 4804.13 cm³ | 12.965 kg | 2700 (铝) |

> **修正**：弹仓不是铝。19.0 kg / 3084.43 cm³ = 6.16 g/cm³，之前按铝 6061 估的
> 8.14 kg **低了 2.3 倍**。符合"屏蔽片"含高密度材料的预期。

弹仓惯量（绕质心，kg·m²，等密度等效）：

```
 3.983969590e-01  1.171961458e-06  2.749539088e-06
 1.171961458e-06  7.426666580e-01  3.706897912e-03
 2.749539088e-06  3.706897912e-03  3.925590894e-01
```

### 2.4 插入行程（自己推的，原 240 mm 不采用）

```
弹仓轴向   [-287.0, 178.0] mm   长 465.0
架子轴向   [-259.3, 239.2] mm   长 498.5
坐到底时轴向重叠 437.3 mm；弹仓顶面低于架子顶面 61.2 mm
=> 完整插入行程 = 465 + 61 = 526 mm；加 20 mm 接近余量 = 546 mm
```

### 2.5 负载问题（新增，未解决）

CS625 额定 25 kg。弹仓 19.0 kg + 夹爪 0.33 kg（**URDF 占位值**）+ 相机 0.06 kg
= 19.39 kg = 额定的 78%。而 `cs625_parallel_gripper.xacro` 里 0.33 kg 的夹爪
**不可能夹住 19 kg 工件**，真实值预计 2~5 kg，合计将到 ~22.5 kg（90%）。

## 三、变更文件

```text
新增  test/inspect_stl_mass_properties.py
新增  src/cs625_simulation/assets/cs625_task/shielding_module/{model.config,model.sdf,meshes/shielding_module.stl}
新增  src/cs625_simulation/assets/cs625_task/slot_fixture/{model.config,model.sdf,meshes/slot_fixture.stl}
新增  src/cs625_simulation/worlds/cs625_insertion_scene.sdf
新增  src/cs625_bringup/config/cs625_task_scene.yaml
```

## 四、执行命令

```bash
# 1) 新建并验证质量属性工具（解析解 + 参考件双重验证）
python3 test/inspect_stl_mass_properties.py --unit m --density 1092.8 \
    src/cs625_simulation/assets/ycb/005_tomato_soup_can/google_16k/nontextured.stl
python3 test/audit_binary_stl_extents.py "$D/插槽架子.STL" "$D/弹仓.STL"

# 2) 连通分量 + 体积指纹 + 惯量特征值匹配
# 3) 三角形边长签名 1:1 匹配 → Kabsch → 迭代剔离群点
# 4) 射线投射量槽壁间隙（Möller-Trumbore，向量化）
# 5) 减面（VTK vtkQuadricDecimation）并验证体积/包围盒
# 6) 校验：XML/YAML 可解析、世界文件 <-> 配置一致、mesh URI 可解析
python3 test/contract_checks.py
git diff --cached --check
```

## 五、通过 / 失败

```text
质量属性工具（解析立方体 I=m/6 精确命中）           PASS
质量属性工具（番茄罐 vs VTK 体积 319.361 cm³）       PASS
弹仓位姿解算（顶点最大误差 0.027 µm）                PASS
槽壁间隙射线投射                                     PASS
减面（弹仓 +0.11%，架子 +0.05% 体积，包围盒不变）    PASS
XML/YAML 解析、世界<->配置一致、mesh URI 解析        PASS
python3 test/contract_checks.py                      PASS
Gazebo 实际加载与楔形槽碰撞行为                      NOT VERIFIED（见限制）
弹仓真实惯量（SolidWorks 惯性张量）                  NOT OBTAINED（用等密度等效）
夹爪真实质量                                        UNKNOWN（URDF 为占位值）
```

## 六、限制

- **本轮未启动 Gazebo**。沙箱禁止写 `/dev/shm` 与 `~/.ros`，`gz_ros2_control`
  收不到 `robot_description`、控制器无法激活，因此**模型能否被 Gazebo 加载、
  夹具的凹网格碰撞是否被 dartsim 退化为凸包，都未运行验证**。这是下一轮用户
  本机执行的第一件事。
- 夹具碰撞用网格（凸包仅占包围盒 57%，且楔形槽壁就是接触面）。若 dartsim
  退化为凸包，槽会变成实心块，必须改成显式楔形板。
- 弹仓惯量假设**密度均匀**。真实件含多种材料，需 SolidWorks 的惯性张量替换。
  → **已解决，见第九节。**
- 弹仓碰撞用包围盒（凸包占 85%），是保守包络，无法解析真实壁厚。
- 插入行程 546 mm 是几何推导值，未经任务验证。
- 场景布局（夹具在 +X 0.62 m，弹仓初始在 −Y 0.55 m）是我定的**默认值**，
  9/28 现场需按实际工位修正。

## 七、能力层与关键链

```text
Capability layer
- environment and dependencies   : Jazzy + underlay + overlay；VTK 9.1 可用；
                                   open3d / sklearn 仍缺
- robot model and simulation       : 任务模型已入库并静态校验；未运行时验证
- kinematics, control, planning    : 未涉及
- vision, hand-eye and TF          : 未涉及
- active perception / NBV          : 未涉及
- real-hardware integration        : 未涉及
- force / contact                  : 两个模型都带 Gazebo contact sensor 话题

Critical-chain status
- URDF -> Gazebo entity           : NOT VERIFIED（模型已就位，未加载过）
- ros2_control -> joint_states    : NOT STARTED（本轮范围外）
- base_link -> camera optical TF  : NOT STARTED（本轮范围外）
- RGB-D -> normalized topics      : NOT STARTED（本轮范围外）
- point cloud -> MoveIt scene     : NOT STARTED（夹具尚未镜像进 planning scene）
- NBV decision -> robot execution : NOT STARTED（本轮范围外）

Sim/real alignment
- reused common core      : 沿用 cs625_simulation assets/worlds 与 bringup config 范式
- sim entry point         : 世界 cs625_insertion_scene.sdf（尚未接 launch）
- real entry point        : 未涉及
- real execution allowed  : 否
- real motion occurred    : 无
```

## 八、下一步

```text
1. 用户本机：加载世界，确认两个模型出现、夹具碰撞不是凸包
2. 夹具镜像进 MoveIt planning scene（沿用 FIXTURE_PROFILES 范式）+ 一致性测试
3. 把世界接进 launch，暴露 scene config
4. 索取 SolidWorks 惯性张量与真实夹爪质量
5. 定义抓取模板（弹仓从哪个面夹、夹哪里）
```

---

## 九、补充（同日）：真实惯性张量与真实夹爪

用户提供了 SolidWorks 的惯量数据和夹爪质量属性，本节的结论**取代**第三节
和第六节中的相应估算。

### 9.1 弹仓真实惯性张量（已写入 model.sdf）

原始数据单位 g·mm²，换算系数 1e-9 → kg·m²：

```
     3.993602428e-01   2.092340000e-05   1.099317000e-05
     2.092340000e-05   7.891516666e-01   4.086532080e-03
     1.099317000e-05   4.086532080e-03   4.349741182e-01
```

与等密度估算对比：

| | 真实 | 等密度估算 | 偏差 |
|---|---:|---:|---:|
| Ixx | 0.399360 | 0.398397 | −0.24% |
| Iyy | 0.789152 | 0.742667 | **−5.89%** |
| Izz | 0.434974 | 0.392559 | **−9.75%** |

所以等密度近似对 Ixx 够用，对 Iyy/Izz 不够——真实张量是必要的。

### 9.2 符号约定（用夹爪数据严格验证，不是假设）

SolidWorks 的「正张量记数法」输出 `Lxy = ∫xy dm`，而**惯量张量的非对角项是
−Lxy**。这一条用夹爪自己的两组张量做了独立判据：

```
对角项   Lo − Lc = [9058560.413, 5640351.391, 3418936.184]
         m(cy²+cz², cx²+cz², cx²+cy²) = [9058395.311, 5640250.411, 3418872.094]   吻合
非对角   Lo − Lc = +35255.173     vs     +m·cx·cy = +35255.654                     吻合
```

非对角是 **+** 号才吻合，证明 `L` 是正张量、非对角需取负。若照抄不改符号，
所有惯性积都会反号。

### 9.3 真实夹爪：负载到额定的 94.6%

```
夹爪「夹持机构0228」: 4.580 kg, 1710866.064 mm³  (密度 2.677 g/cm³，铝)
末端链条 = 19.0 + 4.580 + 0.06(相机) = 23.64 kg = 额定 25 kg 的 94.6%
```

这从"78%（用占位值）"变成**"94.6%"**。额定负载是**指定质心偏置下的静态值**，
接近满载时可用加速度和末端力急剧下降，而本任务需要 546 mm 的插入行程和接触力。

### 9.4 夹爪机理与 URDF 不符

夹爪是**张开式撑紧机构**，不是平行夹爪——张开后撑住工件的扶手。
因此 `cs625_ap_description/urdf/cs625_parallel_gripper.xacro`（0.33 kg、平行指）
**在质量和机理上都不对**，在替换之前任何基于它的规划结论都不成立。

已写入 `cs625_task_scene.yaml` 的 `grasp` 段（`style: expanding_brace`）。

### 9.5 扶手候选（待用户确认）

把弹仓按连通分量拆开后，唯一像扶手的件是**分量 9**：

```
体积 188.721 cm³    尺寸 424.0 × 38.0 × 76.0 mm
弹仓自身坐标系 min [-420.5, -5.0, 89.0]  max [3.5, 33.0, 165.0]
```

它位于弹仓顶部（Z ∈ [89,165]），Y ∈ [−5,33]，而主体外壳的 Y ∈ [12,109]——
**它从主体 −Y 面凸出**。但 Z=127 mm 处的剖面显示杆顶（Y≈33）与外壳底（Y≈45）
之间只有约 12 mm 间隙，通道很窄，需要用户确认是否就是它、夹爪具体撑哪里。

### 9.6 能力层更新

```text
Capability layer
- force / contact : 两个模型都带 Gazebo contact sensor
                    惯性张量已用 SolidWorks 真值；夹爪机理待重新建模

Critical-chain status
- point cloud -> MoveIt scene : NOT STARTED（夹具尚未镜像进 planning scene）
- 新增阻塞：夹爪 URDF 与实物机理不符，抓取模板无法定义
```

---

## 十、补充：质心与体积核对

用户随后提供了 `box` 装配体的完整质量属性，据此更新了 `<inertial><pose>`。

### 10.1 质心（已写入 model.sdf）

| | X | Y | Z |
|---|---:|---:|---:|
| SolidWorks 实测 | −208.49 | 63.55 | −19.44 |
| 我的几何形心 | −208.502 | 63.361 | −18.444 |
| 偏差 (mm) | 0.012 | 0.189 | 0.996 |

偏差 **≤1 mm**，说明等密度形心本来就够用；现已改用实测值。

### 10.2 体积出现分歧，采用新值

| 来源 | 体积 | 与细网格的差 |
|---|---:|---:|
| 用户第一次给的 | 3084.43 cm³ | −10.1% |
| `box` 质量属性（本次） | **3412.09 cm³** | **−0.51%** |
| 我的细网格（276 642 面） | 3429.72 cm³ | 基准 |
| 我的粗网格（63 228 面） | 3014.91 cm³ | −12.1% |

本次的 3412.09 与独立算出的细网格体积只差 0.51%，**故采用 3412.09 cm³**；
第一次给的 3084.43 cm³ 应该是另一个选择集/配置。等效密度随之从 6295 改为
**5568 kg/m³**（且质量是"用户覆盖"值，密度不是材料属性）。

### 10.3 正张量约定第二次验证

用弹仓自己的两组张量再验一遍：

```
对角   Lo − Lc = [8.390415e+07, 8.331005e+08, 9.026445e+08]
       m(cy²+cz²,…) = [8.391381e+07, 8.330739e+08, 9.026270e+08]     吻合
非对角 Lo − Lc = −251729951.04   vs   +m·cx·cy = −251741250.50        吻合
```

两次独立验证（夹爪、弹仓）都指向同一约定，**−Lxy 已确认**。

### 10.4 仍未解决

```text
- 夹爪「夹持机构0228」的几何未拿到：需要 STEP/STL 才能替换 parallel_gripper.xacro
- 扶手位置待用户确认（见 9.5）
- Gazebo 加载与楔形槽碰撞行为未运行验证
```

---

## 十一、补充：Gazebo 卡住的根因、夹爪更正、扶手确认

### 11.1 Gazebo 卡住 = 模型没被安装（不是渲染问题）

```bash
$ ls install/cs625_simulation/share/cs625_simulation/assets/
ycb                                     <- 只有 ycb，没有 cs625_task
$ ls install/.../assets/cs625_task
No such file or directory
```

`assets` 是**真实目录而非软链接**，所以 `src/` 下新增的 `cs625_task/` 从未进入
install，`model://cs625_task/*` 全部解析失败，Gazebo 在 spawn 两个模型处卡死。

修法：`colcon build --packages-select cs625_simulation`。

同时避开两个已知坑：

```text
- sim_base 的 launch_fixture_world 默认 true，会额外起 rgbd_fixture.sdf
  -> 用任务场景时应传 launch_fixture_world:=false
- LIBGL_ALWAYS_SOFTWARE=1 由 sim_base 设置；直接 gz sim 会丢掉它，
  撞上项目已诊断过的 Ogre-Next + WSLg EGL 设备选择问题
```

### 11.2 更正：夹爪机理并没有不符

读 `cs625_parallel_gripper.xacro`：手指关节轴 `0 -1 0` 与 `0 1 0`，**两指往两侧
张开**——正是"张开撑紧"。§9.4 中"机理不符"的结论**撤回**，那是只凭
"parallel gripper" 这个名字就下的判断。

真正不符的是**尺度**：URDF 合计 0.33 kg，实物 4.580 kg，差 14 倍；本体
80×90×50 mm、手指 90×14×70 mm、每指行程 0~40 mm 均为占位值。

### 11.3 扶手确认：分量 10 与 11，且夹爪跨距对得上

用户框出的两处对应弹仓分量 10 和 11，体积逐位相同、关于主体中心严格对称：

```text
左 (分量11) X [-356.5, -330.5]   中心 -343.5   (距中心 -135.0)
右 (分量10) X [ -86.5,  -60.5]   中心  -73.5   (距中心 +135.0)
```

跨距候选与夹爪支撑臂实测 246.57 mm 对比：

| 候选 | 跨距 | 与 246.57 之差 |
|---|---:|---:|
| **内侧面** | **244.00** | **+2.57** |
| 中心 | 270.00 | −23.43 |
| 外侧面 | 296.00 | −49.43 |

只有内侧面在同一量级上，**故确认夹爪伸入两扶手之间、向外撑开撑住内侧面**。

接触几何（弹仓自身坐标系，mm）：接触面 X = −330.5 / −86.5；扶手
Y ∈ [−22, 33]，主体外壳自 Y=+12 起，扶手前凸 34.0；Z ∈ [−172, −26]，
接触中心 Z = −99。已写入 `cs625_task_scene.yaml` 的 `grasp` 段。

**遗留 2.57 mm**：夹爪张开到 246.57 而配合面只有 244.00，单边过盈 1.29 mm。
三种解释待定——扶手有弹性可作预压、两套 CAD 略有出入、或 246.57 是中间状态。
**建模接触力之前必须定这个。**

---

## 十二、补充：夹爪网格与预紧量确认

### 12.1 夹爪网格核对通过

`夹持机构0228.STL`（米制，185 896 面）：

| | 我算的 | SolidWorks | 差 |
|---|---:|---:|---:|
| 体积 | 1710.978 cm³ | 1710.866 cm³ | **+0.007%** |
| 质量 @2677 kg/m³ | 4.580288 kg | 4.580 kg | 吻合 |
| 质心 | (8.892, 858.822, −1109.545) mm | (8.910, 863.944, −1109.692) | Δy 5.1 mm |

Δy 那 5.1 mm 是几何形心与真实质心之差（夹爪内部有电机，密度不均匀），
**采用 SolidWorks 值**。

外形 294.57 × 214.00 × 85.00 mm；部件坐标系有偏置（Y ∈ [766.5, 980.5]，
Z ∈ [−1158.8, −1073.8] mm）。

**正张量约定第三次验证通过**：我按 2677 kg/m³ 算的 Ixy/Ixz/Iyz 与用户给的
L 取负后逐项同号同量级。

### 12.2 支撑臂在网格里定位到

```text
左臂  160.571 cm³  X [-138.3, -18.3]  Y [823.5, 969.5]  Z [-1132.8, -1074.8]
右臂   83.951 cm³  X [ 122.3, 156.3]  Y [823.5, 969.5]  Z [-1132.8, -1074.8]
```

两臂 Y、Z 跨度完全一致，确认是成对的支撑臂；夹爪中心 X = 9.0。

### 12.3 预紧量确认，附带一个仿真后果

用户确认 2.57 mm 是**预紧量**（单边 1.29 mm）。由此：

> **预紧过盈配合无法用刚体接触诚实仿真。**

三个选项，必须选一个：

```text
(a) 把臂放到恰好 244.00 mm，接受法向力只来自求解器（≈0）
(b) 用 attachment 关节表示夹紧，仓库已有 cs625_task_orchestrator/
    p7_attachment_adapter，但须标注为"机理诊断"而非物理抓取
(c) 把臂建成柔性体
```

**在任何 episode 依赖夹持力之前必须先定这个。** 19 kg 的件，靠摩擦夹持时
夹持力直接决定会不会掉。

### 12.4 已落地

```text
新增  src/cs625_simulation/assets/cs625_task/gripper_0228/meshes/gripper_0228.stl
      185896 -> 22306 面，体积偏差 -0.006%，908 KB
修改  src/cs625_bringup/config/cs625_task_scene.yaml  （grasp 段补预紧与夹爪几何）
```

### 12.5 新的阻塞项

```text
法兰安装面 -> 夹爪部件坐标系 的变换未知。
现有 xacro 的 tool0_to_gripper_base xyz="0.085 0 0" 是占位值，
不足以把真实夹爪挂到机械臂上。
```

---

## 十三、补充：MoveIt 镜像、launch 接入、契约断言

### 13.1 变更文件

```text
新增  src/cs625_bringup/scripts/apply_task_scene.py         (安装，launch 可调)
新增  src/cs625_bringup/launch/sim_task_scene.launch.py
新增  test/test_task_scene_applier.py                       (5 项)
修改  src/cs625_bringup/config/cs625_task_scene.yaml        (补原始正张量数据)
修改  test/contract_checks.py                               (新增 check_cs625_task_scene)
修改  docs/simulation.md                                    (新增第 6 节)
```

### 13.2 MoveIt 镜像

夹具**必须用网格**做碰撞体：凸包只占包围盒 57%，且楔形槽壁就是插入的接触面，
用长方体拼会又松又错；静态体用网格在运行时零成本。

规划坐标系确认为 `world`（SRDF 虚拟关节 `world → base_link` 固定，
`sim_moveit.launch.py` 设 `octomap_frame: world`）。

脚本装在 `cs625_bringup/scripts/` 而**不是 test/**，因为 launch 需要通过
`FindPackageShare` 调用它。路径解析同时支持源码树与安装树：

```text
源码树:  向上找 src/cs625_simulation/assets/cs625_task 存在则用之
安装树:  找不到则退回 ament share
```

脚本刻意不 import `test/` 下任何东西。

实测输出（dry-run）：

```text
cs625_task_slot_fixture:          frame=world position=(0.6200, 0.0000, 0.2851)
                                  quaternion=(0,0,0,1)  vertices=4445 triangles=9006
cs625_task_shielding_module_seated: frame=world position=(0.7680, 0.2085, 0.2708)
                                  quaternion=(-0.037007,-0.037007,0.706138,0.706138)
                                  vertices=5947 triangles=12644
```

### 13.3 契约断言抓到一个真错

`check_cs625_task_scene()` 第一次运行就失败：

```text
AssertionError: grasp preload_per_side_m does not sum to preload_total_m
```

原因是我把单边预紧 **1.285 mm** 四舍五入写成了 **1.29**，于是 `1.29×2=2.58 ≠ 2.57`。
已改为 0.001285。**这正是断言存在的意义**——写在注释里的一句话不会被检查，
写成断言才会。

四项断言：

```text
1. 世界 <include> 位姿 == 配置 pose_world / home_pose_world
2. 惯量非对角 == -(记录的 SolidWorks 正张量 Lxy/Lxz/Lyz) × scale
   （由原始数据推导期望值，而不是把结论写死；有人"修"符号就会失败）
3. 负载合计 <= 额定，且与记录的 total_kg 一致
4. 预紧自洽：arm_span_open - handle_inner_span == preload_total
              preload_per_side × 2 == preload_total
              arm_span_at_contact == handle_inner_span
              接触面跨距 == handle_inner_span
              扶手中跨距/外跨距 与 handle_x_ranges 自洽
```

### 13.4 RPY 约定的独立验证

`test_task_scene_applier.py` 里最要紧的一条：把 `roll=-6°, yaw=90°` 转成四元数
再转回矩阵，与**解析构造**的 `Rz(90°)·Rx(-6°)` 比对（不是照抄实现）。

```
解析值 sin6° = 0.10452846326765347
解位姿得到的 float32 值 = 0.104528465
两者吻合到 1e-8  ->  两条独立路径互相印证
```

### 13.5 通过 / 失败

```text
test/test_task_scene_applier.py                 5 passed
test/test_p7_fixture_grasp_scene.py             3 passed（回归未被破坏）
python3 test/contract_checks.py                 PASS（且抓到上述 1.29/1.285 错误）
sim_task_scene.launch.py 构造                    PASS（6 个参数全部暴露）
launch 内实际启动                               NOT VERIFIED（沙箱禁写 ~/.ros）
Gazebo 加载与楔形槽碰撞行为                     NOT VERIFIED
```

### 13.6 限制

- 沙箱禁写 `~/.ros/log`，`ros2 launch` 无法在沙箱内实跑；已改用
  `importlib` 直接构造 `LaunchDescription` 验证参数与实体，并另设
  `ROS_LOG_DIR` 到工作区内验证过一次。
- 夹具凹网格是否被 dartsim 退化为凸包，**仍未验证**，这是用户本机第一次运行
  必须看的事情。

---

## 十四、补充：首次 Gazebo 目视后的三处修正

用户本机加载成功（说明 `colcon build` 后 `model://cs625_task/*` 已能解析，
第 11.1 节的根因判断成立），并反馈三个问题，逐一修正。

### 14.1 摆放方向反了（两个错误，一个旋转同时修正）

用户指出：架子和工件"正反都搞错了，应该以现在的上方为底边"，
且"应该对着夹爪一侧，目前正好是相反的"。

重新推导后确认**两个错误都真实存在**：

```text
(1) 弹仓的扶手在它自身 -Y 面，而 -Y 映射到装配系 +X。
    我把装配原点放在世界 +X，所以扶手侧背对机器人 —— 方向错。
(2) 装配系 +Z 实际朝下。翻正前弹仓从夹具底部穿出 19.3 mm，
    等于插进地板；翻正后从顶部露出 19.3 mm —— 才是插入件的正常状态。
```

一次 **绕世界 Y 轴 180°** 同时修正两者：

```text
R_aw = Ry(180°) = [[-1,0,0],[0,1,0],[0,0,-1]]
  R_aw @ (+X) = -X   -> 扶手侧转向机器人
  R_aw @ (+Z) = -Z   -> 竖直翻转
```

新摆放（装配原点世界 [0.720, 0.000, 0.260709]）：

```text
夹具  世界 X[0.5175,0.7495] Y[-0.3725,0.3725] Z[0.0000,0.5058]   （直接落地）
弹仓坐到底  世界 X[0.5250,0.7039] Y[-0.2273,0.2273] Z[0.0490,0.5251]
  顶端露出夹具顶面 +0.0193 m  ✓
弹仓初始    世界 (0.550,-0.450,0.225) RPY (180°,0,90°)
  与夹具 Y 向间隙 0.0587 m；到基座中心水平距离 0.886 m
```

**推导值与"翻正后应露出 +19.3 mm"的预测逐位吻合**，这是独立自洽的旁证。
坐到底位姿 RPY 由 (-6°,0,90°) 变为 **(174°,0,90°)**。

### 14.2 全白看不出楔形槽

给两个模型和地面加了显式 diffuse 颜色（夹具 0.30/0.32/0.36 深灰，
弹仓 0.72/0.45/0.18 橙），否则白色渲染下无法判断槽是否变成实心块。

### 14.3 夹爪：保留现有的，只改质量

用户指出 RViz 里已有的夹爪就是实物。核对后确认**保留它是正确选择**，
而且这样正好消掉"法兰→夹爪变换未知"这个阻塞项：现有 xacro 的运动学已经
是对的（两指轴 `0 -1 0` / `0 1 0` 向外张开），且已经挂在机械臂上。

改动：质量 0.25+0.04+0.04 = 0.33 kg → **4.20+0.19+0.19 = 4.580 kg**，
与 `夹持机构0228` 实测一致（已用 `xacro` 展开后求和验证）。

**仍不对的是尺度**：实物支撑臂跨距 246.57 mm，占位手指只到 ±58 mm。
连杆几何保持占位，注释里已写明，等法兰变换确定后再换真实网格。

### 14.4 通过 / 失败

```text
Gazebo 加载 model://cs625_task/*            PASS（用户本机，第 11.1 节根因成立）
夹爪质量 4.580 kg                           PASS（xacro 展开求和验证）
python3 test/contract_checks.py             PASS
test/test_task_scene_applier.py             5 passed
摆放方向修正后的渲染                       PENDING（待用户再看一次）
楔形槽是否被 dartsim 退化为凸包             NOT VERIFIED（加色后应可目视判断）
```

---

## 十五、补充：第二次 Gazebo 目视后的几何修正

用户确认**楔形槽没有被 dartsim 退化**（第 12 节遗留的关键未知已关闭），
并提出三个几何问题。全部重新量过，并推翻了第 14 节的一次错误修正。

### 15.1 撤回：第 14 节的 Ry(180°) 是错的

| | 第 14 节做法 | 实际应为 |
|---|---|---|
| 夹具位置 | 世界 +X | **世界 −X** |
| 装配→世界 | Ry(180°) | **Ry(−5.004°)** |

两个错误：

```text
(1) 机械臂零位朝 −X。用 FK 验证：腕部原点在 (−1374, 0, 401) mm。
    所以工件应在 −X 侧。我之前放 +X，再转 180° 把扶手甩到了背面。
(2) 夹具的安装pad在装配系 −Z 端。Ry(180) 把 −Z 端翻到了世界顶部
    —— 夹具是倒立的。这是"架子和地面不垂直"的直接原因之一。
```

正确解只需近似单位阵：装配系本来就接近世界系，扶手侧（装配 +X）
在夹具位于 −X 时自然朝向机器人。

### 15.2 夹具整体倾斜 5.004°，不是建模误差

对缩小后的夹具网格做平面族聚类（法向取族内平均，不取整）：

| 面积 | 法向 | 主轴 | 偏离 Z |
|---:|---|---|---:|
| 1878.1 cm² | (−0.000235, **+1.000000**, −0.000301) | Y | 89.983° |
| 1877.0 cm² | (+0.000085, **−1.000000**, −0.000518) | Y | 89.970° |
| 1126.8 cm² | (−0.996057, +0.001099, +0.088710) | X | 84.911° |
| **947.5 cm²** | (+0.087226, −0.000353, **+0.996189**) | Z | **5.004°** |
| 838.6 cm² | (−0.089671, −0.000200, −0.995971) | Z | 5.145° |
| 808.9 cm² | (+0.995990, −0.001180, −0.089455) | X | 84.868° |

判据：两个 **1878 cm²** 的 ±Y 面精确对齐 Y 轴，而所有 Z 向面都偏 5.0~5.1°。
若是个别面的建模误差，不会所有面一致偏移同一角度且 ±Y 面保持精确。
**结论：整个夹具绕自身 Y 轴刚性旋转了 5.004°。**

修正 `Ry(−5.004°)`，以最大的水平面（947.5 cm²）为基准。
修正后仍有小面偏离轴线——焊接件本该如此，报告为残余而非缺陷。

### 15.3 弹仓坐到底时比夹具安装面低 12.1 mm

```text
校平后：夹具最低点 Z = −245.640 mm（安装面）
        弹仓最低点 Z = −257.737 mm
        -> 弹仓比安装面低 12.10 mm
```

所以夹具平放地面时弹仓会穿过地面。当前用 **32.1 mm 垫高**，让弹仓离地 20 mm。
这不是设计偏置，是仿真摆放选择，已写进 world 注释：**若实际台面是开孔避让的，
把装配原点 Z 改为 0.245640。**

顺带修正：**第 14 节把弹仓当自身坐标系直接旋转，漏了坐到底变换**，
那次算出的 +19.3 mm 露出量是错的。正确值是 −12.1 mm（向下）。

### 15.4 弹仓初始位姿移动

原位置 (0.550, −0.450) 在 X 向与夹具重叠。改到 **(0.450, 0.450)**——
基座的另一侧，机械臂需要转底座才能取件再送到夹具，这是正常作业姿态。
到基座水平距离 0.636 m。

### 15.5 当前摆放

```text
装配原点世界 (−0.720, 0.000, 0.277737)   RPY (0, −5.004°, 0)
夹具       世界 X[−0.7382,−0.5382] Y[−0.3725,0.3725] Z[0.0321,0.5398]
弹仓坐到底  世界 X[−0.7212,−0.5209] Y[−0.2273,0.2273] Z[0.0200,0.4910]
弹仓初始    世界 (0.450,0.450,0.248593) RPY (−5.004°,0,90°)
夹具中心到基座水平距离 0.633 m（机械臂零位朝 −X，最大伸展约 1.37 m）
```

### 15.6 通过 / 失败

```text
楔形槽未被 dartsim 退化                  PASS（用户目视确认）
夹具倾斜测量（平面族聚类）               PASS
摆放修正后 contract_checks               PASS
摆放修正后 test_task_scene_applier       5 passed
摆放方向修正后的渲染                     PENDING（待用户再看一次）
```

---

## 十六、补充：夹具倒置修正

用户指出架子又倒了。这次不再靠形态猜测，改用**沿 Z 的截面跨度**做判据。

### 16.1 判据

对夹具网格逐 Z 段统计 X 与 Y 跨度：

```text
Z∈[-260,-230)  X 跨度  20.8   Y 跨度 386.0
Z∈[-230,-200)  X 跨度 104.9   Y 跨度 474.0
Z∈[ 100, 160)  X 跨度 132~141 Y 跨度 543~681
Z∈[ 190, 220)  X 跨度 151.4   Y 跨度 745.0   <- 最宽
Z∈[ 220, 250)  X 跨度 200.5   Y 跨度 745.0   <- 最宽
Z∈[ 250, 280)  X 跨度   6.4   Y 跨度 731.6
```

**最宽端（X 200.5 mm、Y 满 745 mm）在装配系 +Z。** 底座必然是最宽端，
所以 **装配 +Z 必须朝下**。

### 16.2 撤回 15.1 的结论

15.1 把装配 −Z 端的小件（comp 0/1，X 跨度仅 20 mm）当成了底脚，
实际那是**顶部配件**。教训：判断"哪端是底"要用**跨度/承载面**，
不能用"哪个小件像脚"。

### 16.3 正确变换

```text
R = Rx(180°) · Ry(+5.004°)
  R @ (+Z装配) = (-0.087, 0, -0.996)  -> 宽底座朝下      ✓
  R @ (+X装配) = (+0.996, 0, -0.087)  -> 扶手朝世界 +X    ✓
装配原点世界 (-0.720, 0.000, 0.262063)   RPY (180°, 5.004°, 0)
```

三个约束同时满足，而且**不需要垫高**：

```text
夹具        世界 Z[0.0000, 0.5077]   直接落地
弹仓坐到底   世界 Z[0.0488, 0.5198]   离地 48.8 mm
弹仓顶端超出夹具顶 +12.1 mm           插入状态合理
```

15.3 引入的 32.1 mm 垫高**取消**。

### 16.4 通过 / 失败

```text
夹具倒置修正后 contract_checks        PASS
test_task_scene_applier               5 passed
夹具/弹仓最终朝向                     PENDING（待用户目视）
摆放位置（用户箭头所指）              PENDING（需用户给出世界坐标）
```

---

## 十七、补充：删除幽灵夹爪与相机方块

### 17.1 定位过程（两次猜错，第三次才问对）

上一轮我把 Entity Tree 里的 `*_fixed_joint_jump__*` 名字当成层级错误，
推断"353 mm 遗留偏移是师兄的工具、应当删除"。用户指出图里那块
（`tool0_vision.STL`）**就是他自己的夹爪和相机**，所以那个推断是反的。

**判据**：`tool0_vision.STL` / `tool0_collision.STL` 是 `my_end_effector_link`
的网格，其中同时包含夹爪和相机；而 `cs625_parallel_gripper.xacro` 里的
80×90×50 本体 + 90×14×70 手指，和 `cs625_camera_extension.xacro` 里的
30×50×20 相机方块，**都是本仓库自己建的占位体**，于是在真实工具旁边画出了
第二个幽灵工具。

**教训**：`*_fixed_joint_jump__*` 是 Gazebo 固定关节合并的正常命名，不是错误。
判断"哪个是真实工具"要问用户，不能从名字或偏移量推断。

### 17.2 改法：只删几何，保留 link

直接删 link 会打断四处引用：

```text
cs625_active_perception.srdf      disable_collisions 里点名这些 link
sim_controllers.yaml              驱动 gripper_right_finger_joint
test/apply_task_fixture_scene.py  FINGER_LINKS 按名字取
config/cs625_task_scene.yaml      payload 预算计入这些质量
```

所以保留 link、关节、inertial、ros2_control 条目和 `rgbd_camera` 传感器
（整条 RGB-D 契约都挂在 `camera_link` 及其光学坐标系上），**只删除
`<visual>` 与 `<collision>` 的 box 几何**。

### 17.3 验证

```text
xacro 展开                    OK，仍是 22 links / 21 joints
带几何的 link                 仅 7 个臂网格 + my_end_effector_link
夹爪/相机/p7 link             全部保留
指关节 + ros2_control 接口     全部保留
python3 test/contract_checks.py   PASS
test_task_scene_applier + p7_fixture_grasp_scene   8 passed
```

### 17.4 仍是占位、且对本任务不正确（记录而非隐藏）

```text
flange_to_eef_joint 的 0.353 m 偏移    师兄那套工具的安装长度，未核实
camera_mount_xyz / rpy                 相机外参占位值
指关节行程 0..0.040 m                  实物是 0.00257 m 预紧撑紧
```

---

## 十八、补充：真实夹爪挂到法兰上（B 方案完成）

用户导出了 `夹持机构0920.STL`——与 `夹持机构0228.STL` **同一个刚体**，
但坐标系直接定义在**法兰系**。这解掉了自"夹爪第一次测量"以来一直悬着的阻塞项。

### 18.1 坐标系确认（不是假设，是量出来的）

```text
X ∈ [-147.285,  147.285] mm   对称到 0.000 mm
Y ∈ [  -35.000,  50.000] mm
Z ∈ [   -7.000, 207.000] mm   0 是安装面；-7 是定位凸台，+207 是工具前端
```

**ICP 与 0228 对齐的结果**：中位 0.035 µm、最大 0.068 µm、**全部样本 < 1 µm**
——两者是精确刚体变换下的同一物体，所以新坐标系可信。

### 18.2 撤掉的 353 mm 偏移：属于师兄的工具

`tool0_vision.STL` 与真实夹爪**不是同一个物体**：

| | tool0_vision.STL（师兄） | 夹持机构0920（你的） |
|---|---|---|
| 尺寸 | 177 × 240 × 214 mm | **294.6 × 85 × 214 mm** |
| 格式 | **ASCII，50 MB** | 二进制，1.4 MB |
| 三角形 | 183 373 | 185 896 → 减面后 27 883 |
| 自身 Z 范围 | −360 ~ −146 mm（远离原点） | −7 ~ +207 mm（原点即安装面） |

师兄网格自身偏离原点 146~360 mm，**这正是 `-0.046 0 0.353` 那个偏移的来源**。

### 18.3 改动

```text
my_end_effector_link  visual/collision  -> cs625_ap_description/meshes/tool/gripper_0920.stl
my_end_effector_link  inertial          -> mass 4.580 kg, CoM (-0.0001, 0.0141, 0.1096) m
                                          惯量由零件系旋转到法兰系
flange_to_eef_joint   origin            -> 0 0 0   （改前 -0.046 0 0.353）
占位夹爪 link 质量                       -> 4.20/0.19/0.19 kg 改为各 1 g（避免重复计入）
CMakeLists                              -> install(DIRECTORY urdf meshes ...)
```

**FK 验证**：`flange -> my_end_effector_link` 距离由 **356.0 mm 变为 0.000 mm**。

**未受影响**：SRDF 的 tip（仍是 `my_end_effector_link`）、P7 契约、`ros2_control`
条目、两个指关节——只改了网格和一处固定关节原点。

减面：185 896 → 27 883 面，**体积偏差 0.0000%**。

### 18.4 通过 / 失败

```text
ICP 对齐（0.035 µm 中位，100% < 1 µm）      PASS
xacro 展开                                 PASS（22 links / 21 joints）
flange -> eef = 0.000 mm                   PASS
python3 test/contract_checks.py            PASS
test_task_scene_applier                    5 passed
Gazebo 里真实夹爪的目视确认                PENDING
相机传感器位置                             PENDING（仍是占位外参）
```

---

## 十九、补充：手指可张开 + 零过盈

用户指出两点，其实是同一个问题的两面：

```text
1. URDF 应该是左右两指可以张出的，目前不行
2. 指关节用零过盈
```

### 19.1 为什么原来张不开

上一轮"删幽灵几何"把手指的 box 也删了，**没有东西可动**；而且**残留的关节轴方向是错的**：

```text
原: gripper_right_finger_joint  origin (0.005, -0.018, 0)  axis (0,-1,0)
    gripper_left_finger_joint   origin (0.005,  0.018, 0)  axis (0, 1,0)
```

这是**旧占位坐标系的 Y 轴**，既不是对称轴也不是正确位置。

### 19.2 改法（尺度是量出来的）

真实支撑臂在法兰系里：

```text
左臂  X [-147.31, -27.31]  Y [-9.03, 48.97]  Z [  4.0, 150.0] mm
右臂  X [ 113.29, 147.29]  Y [-9.03, 48.97]  Z [  4.0, 150.0] mm
```

对称轴是**法兰系的 X**。所以：

```text
手指截面      55 x 58 x 146 mm（真实臂的截面，146 mm 高度以法兰系 Z=77 mm 居中）
关节轴        ±X
收拢位置      ±102.00 mm
张开到位      ±122.00 mm   （= 244.00 / 2，扶手内侧面跨距，零过盈）
行程          20 mm
```

**FK 验证**：`joint=0` 跨距 **204.00 mm**，`joint=0.020` 跨距 **244.00 mm** ✓

手指不是真实连杆机构的复现，而是**真实接触面 + 真实运动方向 + 真实截面**配一个简化本体。
link / 关节名保持不变，SRDF、`sim_controllers.yaml`、`P7` 契约都不受影响。

### 19.3 零过盈（用户决定）

配置改为：

```text
interference_fit      = none_zero_preload
preload_per_side_m    = 0.0
preload_total_m       = 0.0
arm_span_open_m       = 0.24400
arm_span_at_contact_m = 0.24400
```

**必须在论文里说明的后果**：零过盈意味着仿真里的夹持**只靠摩擦**，
19 kg 的弹仓可能在仿真里掉落而真机（有 2.57 mm 预紧）不会。
**因此本模型不验证抓取可靠性。**

### 19.4 p7_grasp_center_link

从 `(0.080, 0, 0)` 移到 **`(0, 0, 0.077)`**——两臂之间、工具轴上。
原来的 0.080 m 是占位夹爪的指垫中心，方向也是错的。

### 19.5 通过 / 失败

```text
xacro 展开                        PASS（22 links / 21 joints）
FK 跨距 204.00 -> 244.00 mm       PASS
python3 test/contract_checks.py   PASS
test_task_scene_applier           5 passed
相机真机对应件                    未知（用户需查找），传感器仍是占位外参
```

---

## 二十、补充：场景摆放的读取与保存工具

用户问"如何保存当前位置信息"。Gazebo **不会把拖动写回 SDF 文件**，所以"保存"
必然是两步：**从界面读出位姿 → 写进配置**。`scripts/sync_task_world.py` 现在
两半都管。

### 20.1 用法

```bash
# 看当前记录的位姿（RPY 与四元数都给）
python3 scripts/sync_task_world.py --print

# 把从 Gazebo 右侧 Pose 面板读到的数写进配置，并同步世界
python3 scripts/sync_task_world.py --set fixture \
    --position -0.720 0.000 0.262063 --rpy-deg 180 5.004 0
python3 scripts/sync_task_world.py --set module \
    --position 0.450 0.450 0.221264 --quat 0.70385 -0.70385 -0.06780 0.06780

# 两者必须一致，契约检查会强制
python3 scripts/sync_task_world.py            # 世界跟上配置
python3 scripts/sync_task_world.py --check    # 只报告不一致
python3 test/contract_checks.py               # 应通过
```

`--set` 同时接受 `--rpy-deg`（度）和 `--quat`（四元数），因为 Gazebo 的 Pose
面板两种记法都可能显示；`--print` 两种都打印，方便对照。

`insertion.seated_pose_world` **刻意不可设置**——它是从 SolidWorks 装配体量出来的，
是夹具变换的推论而非选择。

### 20.2 Gazebo 侧怎么读数

Entity Tree 里点模型 → 右侧面板展开 `Pose`。位置显示在父坐标系下，
这两个模型的父坐标系就是 `world`，所以数字可以直接抄。

### 20.3 验证

```text
test_sync_task_world.py                     5 passed
  - RPY 约定对着解析构造的 Rz(yaw)Ry(pitch)Rx(roll) 验证（不是照抄实现）
  - 四元数往返（三个单轴 + 一个混合）
  - 记录的夹具/弹仓位姿仍是校平后的值
端到端：--set 改值 -> --check 报一致 -> 还原 -> 全链通过
python3 test/contract_checks.py             PASS
test_sync_task_world + test_task_scene_applier  10 passed
```

---

## 二十一、补充：应用用户的场景摆放

用户从 Gazebo 的 Pose 面板读出数值（面板标的是 **(rad)**）：

```text
slot_fixture      X  1.03   Y  0.57   Z 0.29   Roll  3.11   Pitch 0.08   Yaw -0.43
shielding_module  X -0.74   Y -0.97   Z 0.23   Roll -3.14   Pitch 0.00   Yaw -0.94
```

`sync_task_world.py` 增加 **`--rpy-rad`**（Gazebo 就是显示弧度，不该逼用户换算）。

写入后的世界包围盒：

```text
夹具   X[ 0.858, 1.348]  Y[ 0.148, 0.909]  Z[0.017, 0.542]  离地 16.63 mm
弹仓   X[-1.081,-0.734]  Y[-1.044,-0.627]  Z[0.005, 0.470]  离地  4.83 mm
```

### 21.1 两个需要知道的后果

```text
1. 两个物体都距基座约 1.22 m —— 接近 1.37 m 水平伸展极限，
   而且对 19 kg 的弹仓是很大的力臂。负载余量需要复核。
2. 夹具悬空 16.63 mm（弹仓 4.83 mm）。夹具 Z 可由 0.29 减到约 0.2734 落地。
```

### 21.2 测试改为断言不变量

`test_recorded_poses_are_the_levelled_placement` 在场景合法移动后**立刻失败**——
它硬编码了旧坐标，管错了东西。改为从网格 + 当前记录的位姿计算：

```text
两个物体都落在地面（或几毫米内）
弹仓自身 +Z 偏离世界 +Z < 2°
夹具与弹仓包围盒不重叠
```

这样无论摆放怎么改都能通过，而真正错误（悬空穿地、倾倒、互相插进去）会失败。

### 21.3 通过 / 失败

```text
python3 test/contract_checks.py                  PASS
test_sync_task_world + test_task_scene_applier   10 passed
实际 Gazebo 渲染                                PENDING（待用户确认）
```

---

## 二十二、补充：坐到底位姿修正为派生量（1.7 m 的错误）

用户应用新夹具位姿后，我核对时发现 **`insertion.seated_pose_world` 偏差 1733 mm**。

### 22.1 根因：我把不变量和派生量搞混了

```text
装配体坐标系下的坐到底位姿   量出来的，是【不变量】
世界坐标系下的坐到底位姿     = 夹具世界位姿 × 上面的不变量   【派生量】
```

上一轮我把派生量当成数据写死，还给它加了一句"不可设置，因为是量出来的"——
**那句话只对了一半**：量出来的确实不可选，但世界位姿必须跟着夹具走。
夹具一移动，坐到底位姿就指向了夹具**原来**的地方。

### 22.2 修法

```text
配置新增（不变量，量出来的）:
  insertion.assembly_seated_position_m: [0.1480452, 0.2085, -0.0143114]
  insertion.assembly_seated_rpy_rad:    [-0.10472, 0.0, 1.570796327]

配置中的 seated_pose_world 标注为 DERIVED，每次 sync_task_world.py 运行时重算
--check 在位姿陈旧时退出码改为 1（原来只打警告却返回 0，所以漏过了）
```

### 22.3 修正结果（用用户的夹具位姿）

```text
旧（陈旧）: (-0.571271, -0.208500,  0.263407)
新（派生）: ( 1.078969,  0.318772,  0.298992)
```

几何自洽验证：

```text
夹具       世界 X[0.858, 1.348]  Y[0.148, 0.909]  Z[0.017, 0.542]
弹仓坐到底  世界 X[0.943, 1.270]  Y[0.310, 0.763]  Z[0.071, 0.555]
  弹仓完全落在夹具包围盒内            ✓
  弹仓顶端超出夹具顶 13.2 mm          ✓（插入件应有的露出量）
  弹仓最低点离地 70.9 mm，夹具 16.6 mm ✓
  到基座水平距离 夹具 1.223 m / 坐到底 1.229 m
```

### 22.4 测试

`test_sync_task_world.py` 增加一条，三重验证派生：

```text
1. 派生值 == 配置里记录的值
2. 夹具平移 0.25 m -> 派生位移恰好 0.25 m
3. 夹具位姿取单位阵 -> 派生结果就是装配体位姿本身
```

### 22.5 通过 / 失败

```text
python3 test/contract_checks.py                  PASS
test_sync_task_world                            6 passed
test_sync_task_world + test_task_scene_applier   11 passed
--check 在位姿陈旧时退出 1                        PASS
```

---

## 二十三、补充：把同一类错误堵在 MoveIt 镜像那一侧

第 22 节修的是**配置里的**陈旧坐到底位姿。但同一类错误在**运行时**仍然可达：
`apply_task_scene.py` 直接信任 `insertion.seated_pose_world`，所以"改了夹具但忘了跑
同步"就会把弹仓的坐到底碰撞体发到**夹具原来所在的位置**，MoveIt 于是在一个
不存在的场景上做规划。

### 23.1 修法

```text
--seated-module 时先重算：assembly 不变量 × 夹具位姿
与配置记录值不一致 -> 打印原因并以 exit 2 拒绝发布
--allow-stale-seated 可显式覆盖
```

实测：

```text
正常:              发布派生位姿 (1.079, 0.3188, 0.299) m，exit 0
人为制造陈旧位姿:  exit 2，不发布，打印修复命令
恢复后:            exit 0
```

### 23.2 关于重复实现

`apply_task_scene.py` 装进 `cs625_bringup` 后**无法 import 仓库的 `scripts/`**，
所以位姿复合逻辑在两处存在。**用测试管住重复**：

```text
test_task_scene_applier.py:
  两份实现的结果一致到 1e-12
  配置里的坐到底位姿不是陈旧值
```

### 23.3 通过 / 失败

```text
python3 test/contract_checks.py                        PASS
test_task_scene_applier + test_sync_task_world         12 passed
陈旧位姿防护                                           实测拦得住（exit 2）
```

---

## 二十四、补充：夹具移到用户指定位置 + GUI 精度陷阱

用户把夹具移到 `(-0.85, 0.49, 0.27)`（面板显示值），朝向不变。

### 24.1 Gazebo 的 Pose 面板只显示 2 位小数

面板显示 `X -0.85  Y 0.49  Z 0.27`，但按 `0.27` 写入会让夹具
**陷入地面 3.4 mm**。用户的真实值几乎肯定是 **0.27337**（面板把 0.27337
显示成 0.27，把 0.29 显示成 0.29）。

采用 **Z = 0.27337**，夹具最低点 **-0.002 mm**，正好落地。

> **精度警告已写进 `sync_task_world.py` 的文档**：面板把位置和角度都四舍五入到
> 2 位小数，直接从面板抄数据可能差厘米级。凡是重要的数，优先用配置里已有的值，
> 只改你真正想改的那一项；或者用 `--print` 读回精确值。

### 24.2 距离改善

```text
夹具       距基座 1.223 m  ->  0.897 m
弹仓坐到底  距基座 1.229 m  ->  0.898 m
```

这是重要改善：19 kg 在 1.22 m 处对 25 kg 额定值（在更小质心偏置下给出）是很长的力臂。

### 24.3 派生自动跟随

坐到底位姿自动更新为 `(-0.801031, 0.238772, 0.278992)`——**这正是第 22 节
"不变量 / 派生量分开"的价值**：夹具一动，坐到底位姿自己跟上，不需要手改。

### 24.4 当前摆放

```text
夹具       世界 X[-1.022,-0.532] Y[ 0.068, 0.829] Z[-0.002, 0.522]  距基座 0.897 m
弹仓坐到底  世界 X[-0.937,-0.610] Y[ 0.230, 0.683] Z[ 0.051, 0.535]  距基座 0.898 m
弹仓初始    世界 X[-1.081,-0.734] Y[-1.044,-0.627] Z[ 0.005, 0.470]  距基座 1.234 m
  弹仓坐到底顶端超出夹具顶 13.2 mm  ✓
  夹具与弹仓初始无重叠               ✓
```

### 24.5 仍待处理

```text
弹仓初始位姿仍在 1.234 m 处 —— 比夹具还远。建议一并挪到 0.9 m 附近。
相机真机对应件与安装外参 —— 待用户查找。
```

### 24.6 通过 / 失败

```text
python3 test/contract_checks.py                        PASS
test_sync_task_world + test_task_scene_applier         12 passed
夹具落地（最低点 -0.002 mm）                           PASS
```

---

## 二十五、补充：弹仓移到用户指定位置

面板显示 `(-0.51, -0.81, 0.22)`，朝向不变。同样遇到 2 位小数陷阱：
按 `0.22` 写入会**陷入地面 5.17 mm**。落地所需精确 Z 由网格反算为 **0.225173**，
采用后最低点 **-0.00 mm**。

### 25.1 当前摆放（两者都进到合理工作范围）

```text
夹具        X[-1.022,-0.532] Y[ 0.068, 0.829] Z[-0.000, 0.525]  距基座 0.897 m
弹仓初始     X[-0.851,-0.504] Y[-0.884,-0.467] Z[-0.000, 0.465]  距基座 0.957 m
弹仓坐到底   X[-0.937,-0.610] Y[ 0.230, 0.683] Z[ 0.054, 0.538]  距基座 0.898 m
```

```text
夹具与弹仓初始无重叠                  ✓
弹仓竖直度 0.09° 偏离竖直             ✓
两者都落地                            ✓
```

**距基座从 1.22/1.23 m 收到 0.90/0.96 m**（最大伸展 1.37 m）——
对 19 kg 的弹仓，力臂显著改善。

### 25.2 通过 / 失败

```text
python3 test/contract_checks.py                        PASS
test_sync_task_world + test_task_scene_applier         12 passed
```

### 25.3 场景摆放阶段性完成

```text
夹具、弹仓初始、弹仓坐到底 三者位置已定，配置 <-> 世界一致性由契约强制
坐到底位姿由夹具位姿派生，改夹具会自动跟随
MoveIt 镜像脚本就绪，可随时运行
```

### 25.4 剩余

```text
相机真机对应件与安装外参   —— 待用户查找后补齐（RGB-D 整条契约挂在它上面）
抓取模板                  —— 依赖上一条
```

---

## 二十六、补充：快捷启动更新 + 修正一条不可能工作的命令

### 26.1 VS Code 任务从 9 个扩到 13 个

`.vscode/tasks.json`（**被 .gitignore 忽略，不随仓库分发**）原有的 9 个任务里
**没有任务场景的入口**。新增三个并重排：

```text
 0  环境校验
 1  构建 overlay
 2  契约检查 + 单元测试（补入任务场景的两项测试）
 3  仿真底座（无头 + RViz）
 4  仿真底座（Gazebo GUI + 网格 + RViz）
 5  任务场景（插槽夹具 + 弹仓 + RViz）★主入口        ← 新增
 6  任务场景 - 摆放查看 / 一致性校验                  ← 新增
 7  任务场景 - MoveIt 场景镜像                        ← 新增
 8  任务场景 - 镜像干跑（只打印）                     ← 新增
 9  仿真主动感知流水线
10  真机底座 - fake hardware
11  真机底座 - 真驱动
12  启动 DSH Harness
```

任务 5 在 launch 前先跑 `sync_task_world.py --check`：配置与世界不一致时**直接
不启动**，避免起来一个错的场景。

### 26.2 修正：`ros2 run` 找不到这个脚本

```text
错的: ros2 run cs625_bringup apply_task_scene.py --seated-module
      -> "No executable found"

对的: python3 src/cs625_bringup/scripts/apply_task_scene.py --seated-module
```

`apply_task_scene.py` 装在 **`share/cs625_bringup/scripts/`**，而 `ros2 run` 只认
**`lib/<pkg>/`** 里的 ROS 可执行文件。launch 文件一直是对的
（`FindExecutable("python3")` + share 路径），错的是**脚本文档和两个 VS Code 任务**。
三处都已改正，并在脚本文档里写明了原因。

### 26.3 README 补上任务场景（仓库内入口）

`.vscode/` 不随仓库分发，所以 README 才是新克隆能看到的入口。新增一节：

```text
任务场景（插槽夹具 + 弹仓）
  · ros2 launch cs625_bringup sim_task_scene.launch.py 及其行为
  · 为什么强制 launch_fixture_world:=false
  · 场景几何的唯一权威是 config/cs625_task_scene.yaml
  · --print / --check / --set 的用法
  · Pose 面板 2 位小数的精度警告
  · seated_pose_world 是派生量、不可直接设置
  · 为什么用 python3 而不是 ros2 run
```

README 内部链接已逐个校验，**全部解析通过**。

### 26.4 通过 / 失败

```text
tasks.json JSON 合法（13 个任务）                     PASS
任务里引用的 10 个文件全部存在                        PASS
非 launch 命令逐条实跑                                PASS
README 内部链接逐个解析                               PASS
python3 test/contract_checks.py                       PASS
test_task_scene_applier                              6 passed
```

---

## 二十七、补充：首次 RViz 目视后的三个修正

用户报告：RViz 里只有架子没有光学元件、末端夹爪是红色的、底部有
`DetachableJoint.cc:348 Child Link target_link could not be found` 警告。

### 27.1 红色 = 假自碰撞（真问题）

```
my_end_effector_link  碰撞体 = 真实工具网格（294 x 85 x 214 mm，含支撑臂）
gripper_left/right_finger_link  碰撞体 = 55 x 58 x 146 mm 的平板
  -> 平板完全落在工具网格包围盒内（重叠 55 x 58 x 146 mm）
  -> SRDF 的 14 条豁免里没有这一对
  -> MoveIt 判定自碰撞 -> 所有视图把工具画成红色
```

先排除了 `Collision Enabled`（RViz 配置里是 `false`），也确认 SRDF **已有**
`my_end_effector_link ↔ wrist_3_link` 豁免，所以不是法兰那处。

补三条豁免（手指在任何关节值下都在工具体内，故 `reason="Never"`）：

```xml
my_end_effector_link <-> gripper_base_link         Adjacent
my_end_effector_link <-> gripper_left_finger_link  Never
my_end_effector_link <-> gripper_right_finger_link Never
```

SRDF 豁免从 14 条增到 **17 条**。

### 27.2 RViz 没有光学元件 = 默认没镜像坐到底弹仓

`task_scene_seated_module` 原来默认 `false`（我当时的理由是"那是规划目标不是固定
世界几何"）。但对插入任务来说，坐到底位置**就是**要达到的目标，应该在 RViz 里
看得见、也应该挡住穿模规划。**默认改为 `true`**。

### 27.3 修正一次无效且会泄漏的 launch 参数

关闭 P7 attachment 插件的第一次尝试是**错的**：

```text
我做的：  sim_task_scene 里传 launch_arguments={"p7_attachment_enabled": "false"}
问题 1：  sim_base 并不声明这个参数 -> 传进去不生效（no-op）
问题 2：  IncludeLaunchDescription 会把未声明的参数写成【全局】launch 配置
          —— 正是 sim_base.launch.py 里已经记录过的那类泄漏
实际机制：sim_control.launch.py 从【环境变量】CS625_P7_ATTACHMENT_ENABLED 读取
```

改为在 shell 层设置环境变量，并在 README 与 VS Code 任务里写明原因。
**并且加了契约检查**：`sim_task_scene.launch.py` 里出现 `p7_attachment_enabled`
就直接失败（已实测：加回去会报错，移除后通过）。

### 27.4 通过 / 失败

```text
SRDF 合法，17 条 disable_collisions                     PASS
契约检查（含新增的无效参数禁令）                        PASS
  实测：把参数加回去 -> AssertionError；移除 -> PASS
test_sync_task_world + test_task_scene_applier          12 passed
README 内部链接                                         无断链
实际 RViz 渲染（红色是否消失、弹仓是否出现）            PENDING（待用户确认）
```

### 27.5 一个仍未解决的建模问题

手指平板**整体落在真实工具网格内部**，这意味着：

```text
· 真实接触几何来自【工具网格】，不是手指平板
· 手指平板作为碰撞体是冗余的，作为可见几何也被网格挡住
· 所以"手指张出"目前是【运动学占位】，不是物理接触模型
```

这与"零过盈"的决定一致——**抓取可靠性本来就不由本模型验证**。但若要让张开
动作在视觉上可信，需要把支撑臂从工具网格里**分离出来**单独做 link。
待用户决定是否值得做。

---

## 二十八、补充：找到并验证真实手眼标定（来自 RVS）

用户问"能不能从 `E:\Desktop\Record\20260608_Ubuntu 24.04` 那个 ROS 环境读 URDF"。

### 28.1 VM 本身读不了，但不需要读

那是 **VMware 虚拟机**（3 个快照增量 + 父盘、2 GB 分段稀疏盘、62 GB）。本机
**没有** `qemu-img`/`7z`/`libguestfs`/`vmware-mount`，无法直接读。三条路：启动 VM
拷出 / VMware 映射虚拟磁盘 / 自写读取器（父盘在、935 GB 空闲，SparseExtentHeader
已解析通，可行约 1~2 小时）。

**但那个 VM 的共享目录就在本机**：`D:\Program Files (x86)\RobotVisionSuite\runtime\`。

### 28.2 相机手眼标定（关键发现）

```ini
# HandEyeTool.ini
Eye In Hand = true          ← 正是本课题需要的
[colorToRobot]  x=96.95  y=42.212  z=97.54    rx=179.96  ry=179.215 rz=89.262
[depthToRobot]  x=96.754 y=17.803  z=97.328   rx=-179.861 ry=179.987 rz=89.451
```

单位：`.ini` 是 **mm + 度**，`ColorToRobotTCP.txt`/`DepthToRobotTCP.txt` 是
**m + 弧度**；两者换算最大差 **4.5e-06**，交叉验证通过。

### 28.3 控制器 TCP 已独立验证

`tool_data_actual.csv` 同时含 TCP 位姿与 `tcp_offset`：

```text
tcp_offset = (-46.0, 0.0, 353.0, 0, 0, 0) mm
```

用关节角 + 我们的 URDF 做 FK 加该偏移：

```text
推算 TCP (mm): [963.3, 649.0, -57.4]     CSV: [963.3, 649.0, -57.4]
推算姿态(deg): [179.44, 0.48, -128.30]   CSV: [179.44, 0.48, -128.30]
距离 0.0 mm，姿态一致
```

**更正**：那个 `flange_to_eef_joint xyz="-0.046 0 0.353"` 我当初当成"师兄工具的
遗留长度"删掉了——**它其实是控制器上的 TCP 偏移**，同一个数。工具网格的改动仍然
正确（真实夹爪网格坐标系就是法兰系、该挂 0 偏移），但我们一直缺 TCP 这个坐标系。

### 28.4 两个坐标系问题必须先定，数值才有意义

```text
参考坐标系：相机实测距法兰约 98 mm，与原始数值吻合
            -> 标定在【法兰系】下，不能叠加 TCP 偏移
            -> 叠加会把相机放到 450 mm（越过 207 mm 的夹爪前端 243 mm，悬空）

标定帧：    +Z 沿法兰 +Z（沿工具轴看出去）——眼在手上的相机必须如此
            -> 记录的是【光学坐标系】
            -> camera_link 需去掉标准本体->光学旋转
            -> 复合结果与彩色眼一致到 1e-12（已断言）
```

两眼光心间距 **24.411 mm**，夹角 **0.79°**（真实双目失配），与"双目相机"吻合。

### 28.5 产出

```text
scripts/camera_extrinsics.py                  从 RVS 输出推导，含单位交叉验证
test/test_camera_extrinsics.py                7 passed
src/cs625_bringup/config/camera_extrinsics_sim.yaml   推导结果 + 来源与日期
```

RVS 安装路径是机器本地的，**不写进仓库**，由 `--rvs-dir` 或
`CS625_RVS_RUNTIME_DIR` 提供（符合 AGENTS.md 的"不硬编码绝对用户路径"）。

推导值（相机在法兰系）：

```text
camera_mount_xyz  = [0.096950, 0.042212, 0.097540]
camera_mount_rpy  = [1.519565756, -1.557075671, -1.532437269]
camera_depth_xyz  = [0.000123, -0.024410, -0.000118]
stereo_baseline_m = 0.024411
```

### 28.6 通过 / 失败

```text
RVS 输出解析（.ini 与 .txt 双路交叉验证）        PASS
TCP 偏移 FK 验证（0.0 mm）                       PASS
光学轴沿工具轴（<2°，实测 0.79°）                PASS
camera_mount 往返一致性                          PASS
python3 test/contract_checks.py                  PASS
test_camera_extrinsics                           7 passed
URDF 实际接入                                    PENDING（待用户决定，见下）
```

### 28.7 未做，以及为什么

**没有把标定值接进 URDF 默认值。** 现有默认 `camera_mount_xyz = (0.03, 0, 0.15)`
配 45° 倾斜是**为旧占位夹爪调的**，而 **P7.1 的传感器可见性证据就是用它验收的**
（`src/cs625_task_orchestrator/tests/test_p7_capture_tools.py` 还断言了这个字符串）。

真实的标定是**沿工具轴正看**，姿态差别很大。换默认值会**使 P7.1 已验收的证据失效**，
按 AGENTS.md 的门禁规则必须显式决定，不能顺手改。

---

## 二十九、真实相机接入 URDF（A 方案）

用户选择 **A：换成真实标定，并重跑 P7.1 传感器门**。

### 29.1 一个更好的结构：锚在深度眼

Gazebo 的 `rgbd_camera` 挂在 `camera_link` 上、沿该 link 的 **+X** 看，而点云契约
用的是 `camera_depth_optical_frame`。所以让 **`camera_link` 落在深度眼**：

```text
传感器(camera_link) 与深度光学帧位置差 = 0.0000 mm
传感器视轴 vs 深度光轴偏差            = 0.0000 deg
```

**完全重合**。彩色眼作为 24.411 mm 偏移的那个——而且这也更诚实：RGB-D 的彩色图
本来就配准到深度帧。

### 29.2 展开后的 FK 与标定逐位对照

```text
深度光学帧在法兰系 = ( 96.754, 17.803, 97.328) mm   标定 96.754, 17.803, 97.328  ✓
彩色光学帧在法兰系 = ( 96.950, 42.212, 97.540) mm   标定 96.950, 42.212, 97.540  ✓
两眼光心间距       = 24.411 mm
```

### 29.3 配置是运行时权威，不只是文档

```text
config/camera_extrinsics_sim.yaml   权威来源（含出处与日期）
sim_control.launch.py               启动时读取并传 5 个参数给 xacro；
                                    文件缺失则拒绝启动，而不是退回未标定的位姿
xacro 默认值                          同样的数，供裸调用
test/contract_checks.py              数值比较两者，漂移即失败
```

**实测防漂移**：把默认值改回旧占位 `(0.03, 0, 0.15)`，契约检查立刻报
`defaults camera_mount_xyz to [0.03, 0.0, 0.15], but the calibration ... says
[0.096754, 0.017803, 0.097328]` ✓

### 29.4 测试确实起了作用

改锚点后 `test_camera_extrinsics.py` **立刻失败**（彩色眼断言不成立）——说明测试
在管真东西。两处已更新为深度眼参考：

```text
test_derived_body_frame_reproduces_the_depth_optical_frame
  （并断言本体 +X == 深度光学 +Z，即 Gazebo 视轴与深度光轴一致）
test_camera_sits_near_the_tool_not_past_it
```

`test_p7_capture_tools.py` 断言了旧占位字符串，也必须改；现在断言标定值，
**仍能拦住"悄悄改回调参位姿"**。

### 29.5 通过 / 失败

```text
FK 与标定逐位一致（深度眼、彩色眼）              PASS
传感器与深度光学帧重合（0.0000 mm / 0.0000 deg）  PASS
契约检查（含新增的相机防漂移）                    PASS
  实测改回旧值 -> 报错；改回标定 -> PASS
test_camera_extrinsics + sync_task_world + applier   19 passed
test_p7_capture_tools                                11 passed
camera_extrinsics --check                            与标定一致
docs/simulation.md 内部链接                          无断链
P7.1 传感器门重跑                                    PENDING（需用户在 Gazebo 跑）
```

### 29.6 关键链路状态更新

```text
base_link -> camera optical TF   NOT ACCEPTED
  几何与 TF 已按真实标定接好、FK 逐位验证通过，
  但 P7.1 传感器可见性证据属于旧占位相机，必须重跑才算验收。
  重跑命令（见 docs/simulation.md §7.4；test/*.sh 无可执行位，须用 bash 调）：
    bash test/run_p7_1_sensor_sim.sh
    export P7_1_EVIDENCE_DIR="$HOME/p7_1_sensor_gate/$(date +%Y%m%d_%H%M%S)"
    CS625_P7_1_SENSOR_GATE=1 CS625_P7_SIMULATION_EXECUTION=1 \
      bash test/run_p7_1_sensor_gate_capture.sh "$P7_1_EVIDENCE_DIR"
  在此之前不得在其上构建主动感知结果。
```

### 29.7 已知限制

```text
· 标定相对【法兰】，日期 2026-04-01。若相机支架移动、或控制器 TCP 重定义后
  按新 TCP 重新标定，这些数就失效，必须重跑 scripts/camera_extrinsics.py。
· Gazebo 只仿真【一个】RGB-D，不是双目。彩色眼只作为 TF 帧与标定偏移存在。
```

---

## 三十、修 P7 门禁脚本的 install 布局假设（阻塞 P7.1 重跑）

用户跑 P7.1 时先撞到**文档错误**（`Permission denied`、`<占位符>` 被当重定向），
修完后撞到**真正的阻塞**：

```text
P7.1 initial-position profile is not installed:
  .../install/share/cs625_bringup/config/p7_1_observation_initial_positions.yaml
```

### 30.1 根因

```text
7 处（6 个脚本）都按 $CS625_APP_INSTALL/share/cs625_bringup/... 拼路径
  -> 那是 --merge-install 工作空间的布局
本仓库用 --symlink-install（默认隔离布局）
  -> 真实路径 $CS625_APP_INSTALL/cs625_bringup/share/cs625_bringup/...
  -> 所以这些脚本【一个都跑不起来】，在加载配置时就退出
```

### 30.2 修法

不手工拼布局，改为问环境：

```bash
cs625_bringup_prefix="$(ros2 pkg prefix cs625_bringup)"
cs625_bringup_share="$cs625_bringup_prefix/share/cs625_bringup"
```

**两种布局都对**，并且加了明确的错误信息（包不在环境里 / config 目录缺失）。
落到 6 个脚本：

```text
run_p7_1_sensor_sim.sh              run_p7_1_sensor_gate_capture.sh
run_p7_adapter.sh                   run_p7_planning_probe.sh
run_p7_static_episode_capture.sh    run_p7_tipfix_sim.sh
```

### 30.3 验证

```text
6 个脚本 bash -n 全通过
5 个所需配置全部解析到（initial/settled/static_grasp/safe_initial/view_planning）
P7.1 启动脚本越过了原来卡死的那道门
  （沙箱起不了 Gazebo，停在 "launch exited before readiness"，属预期）
```

### 30.4 加检查防复发

`contract_checks.py` 拒绝任何 shell 脚本**可执行行**里出现
`$CS625_APP_INSTALL/share/` 或 `cs625_app_install/share/`（注释里允许，因为
修复说明本身要提到这个坏写法）。

**实测**：写一个带旧路径的探针脚本 -> 检查以文件名报错；删掉 -> PASS ✓

### 30.5 通过 / 失败

```text
契约检查（含文档命令检查 + install 布局检查）     PASS
两处检查都实测能触发                               PASS
test_camera_extrinsics + sync_task_world + applier 19 passed
P7.1 传感器门重跑                                  PENDING（沙箱无法跑 Gazebo）
```

---

## 三十一、P7.1 起来了：新相机位姿下 RGB-D 契约是活的

修完 install 布局后，`bash test/run_p7_1_sensor_sim.sh` 打印：

```text
P7_1_SIM_READY partition=p7_1_sensor_20260921_150358_1112997 ros_domain_id=default
```

用户以为"又卡住了"。**其实不是**——脚本末尾是 `wait "$launch_pid"`，故意在前台持有
仿真，不退出。

### 31.1 这行 READY 本身就是接口证据

它只在以下全部成立时打印：

```text
joint_state_broadcaster        active
joint_trajectory_controller    active
/sensors/camera/color/image         sensor_msgs/msg/Image
/sensors/camera/depth/image         sensor_msgs/msg/Image
/sensors/camera/depth/camera_info   sensor_msgs/msg/CameraInfo
/sensors/camera/points              sensor_msgs/msg/PointCloud2
```

**所以换成真实标定后的相机位姿，整条四路归一化 RGB-D 契约仍然成立** ✓
（P7.1 的**接口**部分通过；目标可见性要靠 capture 才判定。）

### 31.2 又发现一个静默失败点：Gazebo partition

抓取脚本会跑 `gz model -m target_object --pose` 读目标真值位姿——**gz-transport
命令，依赖 partition**。而 partition 是在第一终端进程内生成的，第二终端没有。

**忘了导出不会报错，只会让真值位姿为空**——典型的静默失败。修法：

```text
run_p7_1_sensor_sim.sh     READY 时把 partition 写进 /tmp/cs625_p7_1_partition
                          并在 READY 横幅里提示"本终端是故意持有的，另开终端"
run_p7_1_sensor_gate_capture.sh
                           未显式给出 IGN_PARTITION 时自动读取该文件
                           显式给定优先（已隔离验证两种情形）
```

### 31.3 文档同步

`docs/simulation.md` §7.4 与 `docs/p7_five_gate_protocol.md` 都补上：

```text
· READY 之后停住【不是卡住】，是故意的（不要 Ctrl-C）
· READY 行本身证明了什么（四路话题 + 两个控制器）
· partition 的跨终端传递机制
· 证据目录必须全新（脚本拒绝复用）
```

### 31.4 通过 / 失败

```text
P7.1 仿真启动 + 四路传感器 + 两控制器        PASS（用户实测 READY）
partition 跨终端传递（自动取用 / 显式优先）    PASS（隔离验证）
契约检查                                      PASS
文档内部链接                                  无断链
P7.1 目标可见性 capture                       PENDING（待用户跑第二终端）
```

---

## 三十二、P7.1 capture：preflight 通过，卡在 gz-transport partition

### 32.1 好消息：preflight 全绿

```json
"success": true
"controllers": joint_state_broadcaster / joint_trajectory_controller / gripper_controller  全 active
"joint_errors_rad": 1e-7 ~ 2e-6   （容差 2e-3）
"joint_spreads_rad": 全部 0.0     （稳定窗口内无漂移）
"settle_sim_completed": true, "settle_sim_sec": 5.0
"clock_regression_count": 0
```

**所以 P7.1 的控制器、关节定位、时钟同步、稳定窗口都通过了**——而且这是在
**新的真实相机位姿**下。四个归一化传感器话题也确认存在：

```text
/sensors/camera/color/image          sensor_msgs/msg/Image
/sensors/camera/depth/image          sensor_msgs/msg/Image
/sensors/camera/depth/camera_info    sensor_msgs/msg/CameraInfo
/sensors/camera/points               sensor_msgs/msg/PointCloud2
```

### 32.2 卡点：`gz model` 找不到模型

```text
ValueError: Gazebo output does not identify model 'target_object'
manifest 里 "gz_partition": ""  <- 空的
```

**两个都排查过了**：

```text
target_object 是正确模型名     ✓  p7_ycb_tomato_light.sdf 第 23 行 <model name="target_object">
capture 脚本语法正常           ✓  bash -n 通过（那条 "line 130" 报错是更早一次运行留下的）
```

**根因是 partition 不匹配**：仿真脚本自己生成了 partition
（`p7_1_sensor_<时间戳>_<pid>`），而抓取脚本一个都没有。用户当时运行的仿真
**是在"写 partition 文件"这个补丁之前启动的**，所以 `/tmp/cs625_p7_1_partition`
不存在。

### 32.3 顺带发现：`gz model` 失败时返回 0

```text
$ gz model -m target_object --pose
Service call to [/gazebo/worlds] timed out
Command failed when trying to get the world name of the running simulation.
$ echo $?
0
```

**失败也返回 0**，只有输出文本能判断。所以 capture 才会一路走到 parser 才炸，
报出一个指向"模型名错了"的假线索（而模型名是对的）。

已加前置检查：先探 `gz model --list`，命中 `timed out|Command failed` 就
**exit 4** 并打印两个 partition 值、gz 的原话、以及两条修法；再加一条检查确认
运行中的世界**确实**有 `target_object`（这样模型名真错时仍会如实报出）。

**实测（无仿真时）**：

```text
Cannot reach a running simulation through gz-transport.
  IGN_PARTITION=<unset>  GZ_PARTITION=<unset>
  gz model said:  Service call to [/gazebo/worlds] timed out ...
exit=4
```

### 32.4 通过 / 失败

```text
P7.1 preflight（控制器 / 关节 / 时钟 / 稳定窗口）   PASS
四路归一化传感器话题存在                            PASS
gz-transport 前置检查（含 target_object 存在性）     PASS（实测 exit 4 + 可操作提示）
契约检查                                            PASS
目标真值位姿捕获                                    BLOCKED（需重启仿真以写出 partition）
```

---

## 三十三、P7.1 capture：partition 修好，preflight 抓到"读的是瞬态"

### 33.1 partition 传递生效

```text
using the running P7.1 session partition: p7_1_sensor_20260921_151131_1116644
manifest: "gz_partition": "p7_1_sensor_20260921_151131_1116644"
```

前一节的修复确认有效 ✓

### 33.2 新失败：`INITIAL_JOINT_MISMATCH`（只有肩关节）

```json
"failure_codes": ["INITIAL_JOINT_MISMATCH"]
"joint_errors_rad":  shoulder_lift = 0.002627   > 容差 0.002
                     其余 5 个关节  ~1e-7
"joint_spreads_rad": shoulder_lift = 0.001646   （仍在动）
                     其余          ~1e-14
expected  shoulder_lift = -0.490042812931
observed  shoulder_lift = -0.487415764311944
```

### 33.3 排查过程（三个假设，两个被数据否掉）

```text
假设 1：真实夹爪变重导致重力下垂
        否 —— 所有机器人 link 的 <gravity>false</gravity> 均生效（含 my_end_effector_link）

假设 2：更大的真实夹爪撞到了世界里的物体
        否 —— 用记录的关节角做 FK：工具世界系 X[-1.286,-0.996]，
              最低点 Z=0.1335（未穿地），与遮挡板(X 0.55~0.65)相距 1.6 m

假设 3：读的是收敛过程中的瞬态
        成立 ✓
```

**关键证据**：同一配置**在 07:08 通过（1e-8）、在 07:12 失败（2.6e-3）**。
差别是前者抓的是一个已运行很久的仿真，后者抓的是刚重启的。

再看两个位姿配置文件的差异——**只有肩关节**：

```text
初始(命令值) -0.468793    稳定(期望值) -0.490043    差 -0.021250 rad (-1.218°)
```

而且 `p7_1_observation_settled_positions.yaml` 的注释**本来就写着**这个偏移来自
更早的 d3/d4 两次独立运行，并说明"把请求值与稳定值分开记录，是为了不削弱
0.002 rad 的判据、也不假装请求值已经达到"。

### 33.4 定量确认

```text
5 s 仿真时间已走完 87.7% 的行程
  -> 一阶拟合 tau = 2.39 s
  -> 预测 2 s 窗口内位移 0.00148 rad
  -> 实测 0.00165 rad        ✓ 吻合
  -> 20 s 后残差 < 1e-5 rad  远超 0.002 判据
```

### 33.5 修法：给时间，不动判据

```text
--settle-sim-sec        默认 5 -> 20 秒（仿真时间）
两个 capture 脚本        暴露 P7_SETTLE_SIM_SEC / P7_STABILITY_WINDOW_SEC
--joint-tolerance-rad   保持 0.002【故意不动】
```

> **这是时序缺陷，不是模型缺陷。** 放宽 0.002 会把问题藏起来，所以没动。

### 33.6 通过 / 失败

```text
partition 跨终端传递                              PASS
"没有接触"结论（FK + 包围盒）                      PASS
一阶收敛模型与实测吻合（0.00148 vs 0.00165）        PASS
契约检查                                          PASS
test_p7_capture_tools                             11 passed
两个 capture 脚本 bash -n                          PASS
P7.1 完整 capture                                 PENDING（重跑，settle 已提到 20 s）
```

### 33.7 一个值得记录的遗留问题

**位置接口控制、且重力关闭的关节，为什么会从命令值偏离 1.218°？**
这不是本次改动引入的（d3/d4 就有了，且稳定文件已记录），但它本身不平凡：
位置控制 + 无重力 + 无接触，理论上应该精确到位。这条留作待查项，
当前靠"请求值/稳定值分开记录"绕开。

---

## 三十四、我上一个提交引入的缺陷：settle 提到 20 s，但 timeout 还是 20 s

### 34.1 现象

```json
"controllers": {},                     ← 空的，ListControllers 没返回
"failure_codes": ["CONTROLLERS_NOT_ACTIVE", "SETTLE_SIM_TIME_INCOMPLETE"],
"settle_sim_completed": false,
"settle_sim_sec": 20.0,
"joint_sample_counts": 全 0,
"joint_errors_rad": shoulder_lift = 1.15e-08   ← 关节其实完美
```

### 34.2 好消息：settle 诊断被证实

`shoulder_lift` 误差 **1.15e-08** —— **关节确实收敛到 -0.490043**。第三十三节的诊断
（之前读的是收敛中的瞬态）**成立**。

### 34.3 坏消息：这是我自己引入的缺陷

```text
--settle-sim-sec      用【仿真秒】（/clock 计）
--timeout-sec         用【墙钟】，默认 20 s 没跟着改
软件渲染下墙钟比仿真时间跑得快
  -> 20 s 仿真 settle 走完时，20 s 墙钟 deadline 同时到期
  -> 控制器查询与稳定窗口被整体跳过
  -> 报出 CONTROLLERS_NOT_ACTIVE + SETTLE_SIM_TIME_INCOMPLETE
```

**这两个失败码描述的是测试脚手架，不是机器人。** 是我把 settle 从 5 提到 20 时
漏了 deadline。

### 34.4 修法

```python
def effective_timeout(settle_sim_sec, stability_window_sec, requested_timeout_sec):
    return max(requested_timeout_sec,
               settle_sim_sec + stability_window_sec + DEADLINE_MARGIN_SEC)   # 30 s
```

```text
settle=20 stability=2 requested=20  -> 52.0 s 墙钟
settle=20 stability=2 requested=300 -> 300 s（显式宽裕值不被改动）
settle=60 stability=2 requested=20  -> 92.0 s
```

**0.002 rad 判据依旧没动**——这是上一提交的我的缺陷，不是机器人的问题。

### 34.5 顺带清理一个我误建的重复文件

我把回归测试追加到了 **`test/test_p7_capture_tools.py`**，而现有测试文件在
**`src/cs625_task_orchestrator/tests/test_p7_capture_tools.py`** —— 建出了一个
重复文件（AGENTS.md 明确禁止）。已并入真文件并删除误建文件，同时修掉
`pathlib.Path`（真文件用的是 `from pathlib import Path`）与 repo root 层数。

### 34.6 通过 / 失败

```text
effective_timeout 边界（4 组）                    PASS
test_p7_capture_tools                            12 passed
契约检查                                          PASS
误建的重复文件清理                                PASS
P7.1 完整 capture                                 PENDING（重跑，deadline 已修正）
```

---

## 三十五、P7.1 跑通了全流程：gate 失败的真因不是遮挡，是相机指向

这一轮 capture **完整跑完**（前几轮都卡在工具链上）。

### 35.1 工具链全部通过

```text
preflight               "success": true（控制器 active、关节 1e-8、spread 0.0、settle 完成）
目标真值                  gz model 读到 target_object @ (0.72, 0, -7.9e-05)
五窗口                  全部采集成功，CDR + 派生文件齐全，四路校验 layout_valid=true
gate_pass               false
```

### 35.2 失败原因：目标在相机背后

```json
"camera_xyz_m": [0.0034, 1.5213, -1.2138]   ← z 为负 = 在相机背后
"in_front": false,  "inside_image": false
"finite_point_count": 0                     ← 整个点云 0 个有效点
```

光学系是 **(x 右, y 下, z 前)**。目标在 `z = −1.214 m`。

**根因**：`p7_1_observation_initial_positions.yaml` 是**为旧占位相机解的**
（45° 斜看、装在 (0.03,0,0.15)）。换成真实相机（沿工具轴正看）后，同样的关节角
把相机指向了完全不同的方向——**连地面都看不到**（所以点云全空）。

**这是一个"改了 A 没检查 B"的典型**：换相机时没有任何检查验证观测位姿还能看见目标。

### 35.3 用仓库自己的求解器重解（不是手调）

`test/p7_solve_static_observation_pose.py` 本来就是干这个的，而且它用的正是
**`camera_depth_optical_frame`** —— 我们改锚点的那个 frame，所以自动用上新外参。

```bash
--urdf <展开的URDF> --seed-joints p7_safe_initial_positions.yaml \
  --target-pose <gazebo目标位姿> --lock-proximal
```

`--lock-proximal` 让**只有腕部运动**、前三个关节留在安全种子：

```text
shoulder_pan 0.0   shoulder_lift -0.35   elbow 0.0
wrist_1 +0.056945  wrist_2 -1.689989     wrist_3 -1.594572
```

### 35.4 离线验证（旧位姿从来没有过这一步）

```text
目标在光学系 (0.018, -0.175, 1.895) m，距离 1.903 m          ✓ 在相机前方
像素 (162.6, 94.4)，距最近图像边缘 94 px                      ✓ 居中
真实夹爪：前方 16757 点，图像内 2440 点，距相机 0.053~0.228 m
目标像素 ±20px 内的夹爪点 = 0                                 ✓ 不遮挡
所有关节限位裕度 ≥ 3.14 rad                                   ✓
最低 distal link 原点离地 0.423 m                             ✓
```

### 35.5 顺带解决一个悬了很久的担忧

第三节我担心过："真实相机沿工具轴正看，视野中心会不会就是夹爪？"

**答案是：不会。** 只要观测位姿是按该相机重解的，夹爪虽然占据画面
（2440 px，u[2,284]、v[1,221]），但**绕开了目标**。眼在手上 + 沿工具轴正看
并不天然遮挡。

### 35.6 加了正是这次缺失的检查

`contract_checks.py` 现在会在以下任一情况失败：

```text
观测位姿诊断所用的 URDF 哈希与当前 xacro 不一致   ← 这次就是栽在这里
目标在相机背后 / 不在图像内
求解时没有 --lock-proximal
夹爪遮挡目标
记录的位姿与 initial_positions 文件不一致
```

**实测**：伪造 URDF 哈希 -> 立刻报错并给出重解命令 ✓

### 35.7 通过 / 失败

```text
工具链（partition / settle / deadline）           PASS
preflight                                        PASS
五窗口采集                                        PASS
相机指向重解 + 离线遮挡验证                        PASS
契约检查（含新增的观测位姿检查）                    PASS（实测能触发）
test_sync_task_world + applier + camera_extrinsics  19 passed
P7.1 gate 本身                                    PENDING —— 见 35.8
```

### 35.8 还差一步才能过 gate

```text
p7_1_observation_settled_positions.yaml 里存的还是【上一条命令】测出的稳定偏移
新位姿下肩关节的偏移会不同，必须重新测量
下次 preflight 会报 INITIAL_JOINT_MISMATCH 并在 observed_joint_positions_rad
里给出新的实测值；用它更新该文件后，gate 才能过
```

这条已写进诊断的 `tool_fixes_before_next_runtime`。

---

## 三十六、P7.1 新位姿实测：**零稳定偏移**

用户重启仿真（新的 initial positions 生效）后跑 capture，preflight 报
`INITIAL_JOINT_MISMATCH`——**这是预期的**，因为稳定位姿文件还是旧位姿测的。

但关键数据是：

```json
"observed_joint_positions_rad": {
  "shoulder_pan_joint": 0.0,        "shoulder_lift_joint": -0.35,   "elbow_joint": 0.0,
  "wrist_1_joint": 0.056945119,     "wrist_2_joint": -1.689988597,  "wrist_3_joint": -1.594572224
},
"joint_spreads_rad": { 全部 0.0 },
"settle_sim_completed": true, "settle_sim_sec": 20.0
```

**实测值 = 命令值，逐位一致；200 采样 / 2.0 s 内漂移全部为 0。**

### 36.1 新位姿没有稳定偏移

旧位姿有 `shoulder_lift` 的 **−0.021250 rad（1.218°）** 偏移，需要单独记录稳定值。
新位姿（肘关节在 0、手臂折叠）**精确停在命令位置**，所以：

```text
p7_1_observation_settled_positions.yaml := 实测值 = initial 值
```

这是**测量结果**，不是简化——文件注释里写明了这一点，并且明确说明"为什么一个构型
会偏离命令值而另一个不会，仍未解释"（对应第 33.7 节的待查项），
**要求以后重解后必须重新测量，不能假定。**

### 36.2 通过 / 失败

```text
initial 与 settled 逐位一致                       PASS
契约检查                                          PASS
test_p7_capture_tools                            12 passed
preflight（下一次运行）                           待验证 —— 预期 PASS
P7.1 sensor gate                                 待验证 —— 真正的重点
```

---

## 三十七、✅ P7.1 传感器门通过（关键链路里程碑）

### 37.1 结果

```text
gate                      P7.1_PERCEPTION_INPUT
gate_pass                 true
windows                   5 / 5 全部 window_pass
failure_codes             []
record_status             complete
trajectory_commands_sent  0        （仅传感器输入，无运动）
real_hardware_connected   false
evidence（本地归档）        ~/p7_1_sensor_gate/20260921_152724
```

按 AGENTS.md 的规定，原始证据留在**本地归档**（含 5 个窗口的 CDR、PPM、npy、
metadata、`manifest.json`、`checksums.sha256`），**未提交进仓库**。

### 37.2 离线预测被 Gazebo 逐位复现

这是本次最有价值的一点——**离线几何与仿真实现在数值上一致**：

| | 离线求解 | Gazebo 实测 | 差 |
|---|---|---|---|
| 目标在相机系 (m) | (0.0179, −0.1747, 1.8950) | (0.017896054, −0.174709352, 1.894976353) | 逐位 |
| 投影像素 | (162.6, 94.4) | (162.6178, 94.4440) | 逐位 |
| 相机世界位置 (m) | (−1.143419401970, −0.129024743842, 0.422607220321) | (−1.143419401890, −0.129024743913, 0.422607220350) | **1e-10** |

**这一次性验证了四件事**：手眼标定、坐标系约定（深度眼锚点）、URDF 外参传递、
以及离线求解器本身。

### 37.3 独立复核（不采信 gate 自己的结论）

从 `raw/window_01/points_xyz_m.npy` 原始点云重新计算：

```text
有效点 57418 / 76800                    与 gate 记录 57418 一致 ✓
落在目标碰撞代理内 150 点                gate 记 151（差 1 点来自我用的 ±5mm 容差）
  这些点世界范围 X[0.678,0.742] Y[0.049,0.117] Z[0.0045,0.106]
  -> 正是半径 0.034 / 高 0.102、中心 (0.711,0.084,0.051) 的圆柱
独立投影目标中心 -> 像素 (162.6178, 94.4440)
gate 记录          -> 像素 (162.6178, 94.4440)       差 2.8e-14 px
```

**目标确实在点云里**，不是 gate 自说自话。

### 37.4 一个悬了很久的担忧，现在有证据了

第 28 节我担心过："真实相机沿工具轴正看，视野中心会不会就是夹爪？"

**答案：不会。** 真实夹爪占据画面 **2440 px**，但**目标像素 ±20 px 内一个夹爪点都没有**。
眼在手上 + 沿工具轴正看**并不天然遮挡**——只要观测位姿是按该相机重解的。

### 37.5 关键链路状态更新

```text
base_link -> camera optical TF       NOT ACCEPTED  ->  PASS
```

### 37.6 Capability layer 现状

```text
environment and dependencies          PASS  统一入口，契约检查通过
robot model and simulation            PASS  真实夹爪 + 真实相机外参，FK 逐位验证
kinematics, control and planning      PASS  控制器 active，关节误差 1e-7~1e-8
vision, hand-eye calibration, TF      PASS  P7.1 通过，外参被仿真复现到 1e-10
active perception / NBV               NOT STARTED  前置层刚通过，尚未开始
real-hardware integration and safety  NOT STARTED  仅 fake hardware 冒烟
```

### 37.7 Critical chain

```text
URDF -> Gazebo entity                 PASS
ros2_control -> joint_states          PASS
base_link -> camera optical TF        PASS   ← 本轮
RGB-D -> normalized sensor topics     PASS   四路话题类型 + 5 窗口 layout_valid
point cloud -> MoveIt planning scene  NOT ACCEPTED  夹具/坐到底弹仓已镜像，但未与 P7.x 联合验证
NBV decision -> robot execution       NOT STARTED
```

### 37.8 通过 / 失败

```text
P7.1 gate（5/5 窗口）                    PASS
独立点云复核                              PASS
契约检查                                  PASS
docs/simulation.md §7.4 更新              PASS（无断链）
```

### 37.9 下一步（按门禁顺序）

```text
1. P7.2 位姿估计门   —— 输入必须来自本次冻结的 P7.1 原始观测
2. 插槽任务场景      —— 夹具/弹仓几何与摆放已就绪，MoveIt 镜像脚本就绪
3. 抓取模板          —— 外参已定，可开始
注意：P7.1 刚通过，未经 P7.2/P7.3/P7.4 不得跳到 NBV
```

---

## 三十八、抓取模板（模块坐标系，已验证）

模块是**撑开式夹持**（不是捏取）：夹爪收拢着进入两扶手之间，再张开到接触跨距。

所以模板不是一个位姿，而是 **模块系下的抓取变换 + 臂序列**。写在**模块坐标系**里，
这样 P7.2 观测到的任意模块位姿都能直接套用，不用重新推导。

### 38.1 模板内容

```text
rotation_gripper_to_module   工具轴沿模块 +Y（从 −Y 侧进入），臂分离轴沿模块 +X
translation_module_m         (-0.2085, -0.1000, -0.0990)
arm                          收拢 0.204 m -> 接触 0.244 m，指令 0.020 m
approach                     沿模块 −Y，后退 0.120 m
lift                         沿世界 +Z，0.080 m
```

**每个平移分量都来自一个实测不变量**（不是调出来的）：

```text
x  两接触面的中点
z  接触中心高度
y  臂沿工具轴的延展中心 对准 扶手沿模块 Y 的延展中心
```

### 38.2 三次"测量推翻了第一直觉"

**① 插入深度不是"越深越好、直到撞本体"。**
扫 160 mm 范围**全程无碰撞**——因为按一维推理的"撞本体深度"远在臂的实际有用范围之外。
深度成为一个**选择**，于是用"臂中心对准扶手中心"来定。

**② 一维区间推理给出的是错答案。**
把"模块在两扶手之间的最小 Y"与"夹爪在间隙内的最大 Z"相比，预测 t_y = −221 mm 接触；
而该深度处的**网格实测距离是 36 mm**——因为这两个特征在不同的 X 上。
**只有网格测量可信。**

**③ 接近路径有擦碰。**
网格处于单一刚性状态时，夹爪进入途中最小间隙 **0.483 mm**（后退 30 mm 处）。
网格**无法收拢手臂**，所以这个数偏悲观；真实序列是收拢进入的。
配置里**如实写明了这个limitation 与这个擦碰值**，没有抹平。**收拢后是否消除擦碰，需要仿真确认。**

### 38.3 一个我自己写错、被测试抓住的 bug

`derive()` 里我在 `(N,3,3)` 三角形数组上写 `gripper[:,0]`，取到的是"每个三角形第一个顶点的
x"，于是**工具全宽被报成 414 mm（真实 295 mm）**。测试
`test_arm_extents_come_from_every_vertex_not_one_per_triangle` 专门锁住这一点。

同轮还修了：**lift 沿世界 Z**（我一开始当成抓取系的子位姿去复合，结果模块几乎没抬起、
反而平移了 6.5 cm）。测试 `test_world_poses_lift_along_world_z` 锁住。

### 38.4 产出

```text
scripts/grasp_template.py                    推导 + 网格实测 + --write / --check
src/cs625_bringup/config/cs625_grasp_template.yaml   模板 + 世界系位姿
test/test_grasp_template.py                  9 passed
```

世界系位姿（对 `module.home_pose_world`，仅作便利；观测到别的位姿时应重新复合）：

```text
pregrasp_world  (-0.4554, -0.5120, +0.3245)
grasp_world     (-0.5523, -0.5827, +0.3243)
lift_world      (-0.5523, -0.5827, +0.4043)     抓取->抬起 纯 +Z 80 mm ✓
```

### 38.5 通过 / 失败

```text
抓取位无穿透（最小间隙 2.270 mm）              PASS
臂中心对准扶手中心（t_y = −100.003 mm）        PASS
接触跨距 = 扶手内侧面跨距 244.000 mm           PASS
世界系位姿：抬起为纯 +Z 80 mm                  PASS
契约检查（新增 5 条抓取不变量）                 PASS
  实测：改坏深度 -> 报错；改坏跨距 -> 报错；还原 -> PASS
test_grasp_template                          9 passed
接近擦碰是否因收拢手臂而消除                   PENDING（需仿真）
```

### 38.6 未做，以及为什么

```text
· 未接入 P7.5 抓取门 —— 抓取门需要真实的接触/力证据，而本任务选了【零过盈】，
  夹持力是已知未知量。模板本身（位姿 + 臂序列 + 无穿透）已可复现可用。
· 未做真机验证 —— 真机前置条件未满足（见 docs/real_hardware_readiness.md）。
```

---

## 三十九、接近擦碰：已闭环（是假警报）

第 38.2 ③ 记的 0.483 mm 接近擦碰，我当时说"需要仿真确认"。**其实不需要仿真**——
夹爪的臂**沿工具 X 平动**，所以把网格的臂区域按行程向内滑动，就近似了接近时真正用的状态。
这正是 URDF 指关节所做的同一个近似。

```text
臂伸出     最差 0.483 mm @ 后退 30 mm     <- 擦碰
臂收拢     最差 1.039 mm @ 抓取位本身      <- 无擦碰
```

**结论：那个擦碰是"网格是单一刚性状态"造成的假警报，不是真实障碍。**

配置里现在同时记录两个数、这条 finding、以及剩余风险：

```text
grasp_minimum_m            0.001039489
approach_minimum_m         0.001039489
arms_extended_worst_m      0.000482586
finding                    伸出时 0.483 mm 是刚性网格必然报出的；收拢后最近 1.039 mm
remaining_risk             收拢状态是近似；且因选了零过盈，接触本身未建模
```

### 39.1 契约检查又加两条

```text
收拢后的接近路径必须保持间隙
且必须【优于】伸出状态   <- 防止"收拢测量"悄悄变成空操作
```

实测：把 `grasp_minimum_m` 改成 0.0001 -> 立刻报 "the grasp pose penetrates the module" ✓

### 39.2 通过 / 失败

```text
收拢接近无擦碰（1.039 mm）              PASS
契约检查（新增 2 条）                    PASS，且实测能触发
test_grasp_template                    9 passed
grasp_template --check                 与配置一致
```

### 39.3 剩余

```text
· 接触本身未建模（零过盈的选择所致），抓取力是已知未知量
· 真机验证未做（前置条件未满足）
· 仿真里走一遍 pre-grasp -> grasp 仍是值得做的确认，但不再是阻塞项
```

---

## 四十、插槽任务序列（含一个**未闭环的插入直线问题**）

### 40.1 结构决定：抓取写在模块系

```text
flange_world = module_world . grasp_in_module
```

**每个路径点都是同一个变换在不同模块位姿上的求值**，没有逐步重推。这也是为什么
P7.2 观测到的位姿可以直接抓——不需要动代码。

```text
scripts/insertion_sequence.py                          测量 + 组装 + --write/--check
cs625_task_orchestrator/task_insertion_sequence.py      纯几何（可导入、无 ROS 依赖）
config/cs625_insertion_sequence.yaml                    路径点 + 臂指令 + 测量记录
test/test_task_insertion_sequence.py                   6 passed
```

### 40.2 插入轴与行程（测出来的，不是假设）

```text
模块在夹具系里【恰好一端外露】: 模块 Z -264.4 mm  vs  夹具 -245.1 mm
  -> 外露的那端就是开口，拔出方向 = 夹具系 −Z
夹具自身位姿绕 X 转了 π
  -> 夹具 −Z = 世界 +Z  ✓  模块是【从上方下降】插进去的
行程 = 模块沿该轴的尺寸不再与夹具重叠处 = 525 mm（模块 470 + 外露 19）
```

**一条走过的弯路**：我一开始按"哪个方向能动"来扫，结果 ±X/±Z 都是 900 mm"自由"、
±Y 是 0——**完全没有区分度**。因为**远离夹具永远不会碰撞**，"哪个方向自由"这个问题
本身没有答案。只有从"模块在哪端外露"这个几何事实出发才有答案。

### 40.3 ⚠️ 未闭环：插入直线会擦到夹具

```text
沿拔出方向（世界向上）移动：
  坐到底间隙   1.198 mm
  最 小 间隙   0.131 mm  @ 离坐到底 259 mm      （装配系独立复核：0.098 mm @ 240 mm）
  入口间隙     108.2 mm
```

**两个方向、两种坐标系都测到 0.1~0.3 mm 量级。**

**但夹具网格是从 150k 面抽稀到 9k 的**（体积变化 0.05%），这个量级的表面位移
正好也在 0.1 mm 量级。**所以结论是不确定的，不是"能通过"。**

处理方式（**没有掩盖**）：

```text
配置里记录 passes: false 与实测值
insertion_sequence.py --check 以 exit 2 报 "NOT CERTIFIED"
contract_checks 反而断言"配置必须仍然承认它不通过"
  —— 数字突然变成通过，说明测量方法变了，不是运动变安全了
```

### 40.4 这一轮我犯的两个坐标系错误（都是静默的）

```text
① 把【模块在装配系的姿态】当成【装配系->世界的旋转】
   -> 轴指向地板 (0.10, 0, -0.99)，间隙报成 658 mm
② 把夹具网格留在装配系、模块放在世界系去比距离
   -> 报出"距离一个不存在的夹具"的几百毫米
```

两个都由 `test_measured_insertion_axis_points_out_of_the_fixture` 与
`--check` 的荒谬数值抓住。

### 40.5 通过 / 失败

```text
插入轴指向世界向上                     PASS（有测试锁住符号）
行程 525 mm，与外露几何自洽             PASS
序列结构（顺序、自由空间段、臂指令）     PASS（契约检查 6 条）
契约检查                               PASS
test_task_insertion_sequence          6 passed
插入直线安全性                         NOT CERTIFIED（0.131 mm，在抽稀误差量级）
```

### 40.6 需要你决定的事

插入直线擦到夹具 0.1 mm，有四种可能，**我无法从现有数据区分**：

```text
a) 抽稀误差 —— 用原始未抽稀网格重测即可判定（最可能）
b) 楔形自锁 —— 模块本就该倾斜着取出，不是纯平移
c) 插入轴不是精确的夹具 Z，有微小夹角
d) 夹具确实有个唇边需要越过去
```

**最快的判定是 (a)**：原始 STL 还在
`src/cs625_simulation/assets/cs625_task/slot_fixture/meshes/` 的来源处（你导出的那份）。
如果你手上还有未抽稀的版本，给我路径，我用它重测一次就能定性。

在这一点定下来之前，**插槽任务不应进入仿真执行**——会让机械臂带着 19 kg 撞上去。

---

## 四十一、插入直线闭环：不穿透，是**精密配合**（结论与我上一轮相反）

用户提供了**未抽稀的原始导出**，这直接判定上一个未闭环项——**而且结论是反的**：

```text
                     坐到底间隙    拔出最小间隙    位置
抽稀（仓库资产）        1.198 mm      0.098 mm    240 mm
原始（你导出的）        0.700 mm      0.040 mm    240 mm
```

**原始网格更紧，不是更松。** 所以那 0.1 mm **不是抽稀误差**——真实数据里模块穿过夹具
只剩 **0.040 mm**。

### 41.1 我上一轮的结论错了

我写的是"0.131 mm 在抽稀误差量级，所以不确定、不予认证"。**实际上：**

```text
· 几何上【成立】—— 0.040 mm > 0，不穿透          ✓ 我错判为"不确定"
· 但 0.040 mm 的间隙【不可能靠位置控制完成】       这个判断我漏了
```

**"能不能通过一个 0.5 mm 阈值"这一个问题，把两件不同的事混成了一件。** 已拆开：

```text
penetrates      是否穿透                （是/否）
precision_fit   间隙是否小于 1 mm       （是 -> 带 advisory）
```

`--check` 现在 **exit 0 + ADVISORY**：

```text
ADVISORY: the insertion line clears by only 0.131 mm at 259 mm above seated,
which is a precision fit; plan it with force or compliance control
```

### 41.2 四个原始文件的身份也确认了

```text
插槽架子.STL                36 546 面   零件系
插槽架子_装配导出.STL       150 114 面  装配系   ← 仓库夹具资产的来源
弹仓.STL                    63 228 面   零件系
弹仓_装配导出.STL           276 642 面  同一坐标系、更细  ← 仓库模块资产的来源
屏蔽片模块插槽和弹仓装配.STL 426 756 面 = 150114+276642，模块已坐到底
```

原始文件**不入库**（15 MB），只把测得的对比作为 provenance 记进配置；
契约检查断言"更细的网格仍然更紧"——因为**结论依赖这个方向**。

### 41.3 这是一个真实的工程发现，不是缺陷

**屏蔽模块与插槽是精密配合（0.04 mm）。** 这对任务设计有三个直接后果：

```text
1. 位置控制的下降一定会卡死 —— 仿真与真机都一样
2. 必须用力控/柔顺 —— 仓库里已有相应依赖：
     admittance_controller（ROS 图里出现过）
     StartCompliantPlacement.srv / CompliantPlacementState.msg（RVS 那套里的）
3. 或者需要在插槽入口做倒角/导向
```

### 41.4 通过 / 失败

```text
插入轴指向世界向上（符号有测试锁住）         PASS
行程 525 mm                                  PASS
插入直线【不穿透】（0.040 mm）               PASS
精密配合判定 + advisory                      PASS（有测试覆盖三种情形）
契约检查（含"更细网格更紧"的方向断言）        PASS
test_task_insertion_sequence                7 passed
```

### 41.5 现在挡在仿真执行前面的东西变了

**不再是几何问题**（几何已经证明可行），而是：

```text
下降段必须柔顺/力控，否则 0.04 mm 间隙必然卡死
```

这是一个**任务设计决策**，需要用户定：加 admittance 控制？还是给插槽加导向倒角？
还是接受仿真里"卡住"作为真实约束记录下来？

---

## 四十二、插槽任务执行编排 + 判定契约（C 方案）

用户选 **C**：把 0.04 mm 精密配合当作**记录在案的真实约束**，让仿真如实呈现，
把"精密插入需要柔顺"变成**论文结论**而不是要绕过的 bug。

这需要两件东西：**能跑一遍**，以及**能把"预期中的卡死"和"其他失败"分开**。

### 42.1 不重复造运动

```text
每一腿        test/p7_execute_pose_capture.py   （现有，已验收）
夹爪          /p7/gripper_command              （P7 夹爪适配器）
接触          test/p7_capture_gazebo_contacts.py（现有，gz-transport）
```

腿 → phase 的映射是**语义的**，不是凑的：

```text
pregrasp_module  -> pregrasp
grasp_module     -> approach
lift_module      -> lift
insert_entry     -> pregrasp
insert_seated    -> approach     ← approach 是唯一会规划【带碰撞检查的笛卡尔直线】
                                   且【拒绝被截断路径】的 phase —— 正是插入段需要的，
                                   也正是卡死会暴露出来的地方
release_retreat  -> lift
```

### 42.2 判定分成三种，而不是通过/失败

```text
PASS                全部腿完成，模块坐到底
PRECISION_FIT_JAM   下降段【在接触中】中止，而接近段与规划本身都干净
OTHER_FAILURE       其他一切
```

**接触记录是必需项，这是关键**：没有任何东西接触的跟踪中止是**控制器问题**；
把它叫作精密配合卡死，就是**从一份不支持该结论的证据里声称结论**。

9 项测试里有 **5 项专门守这条线**：无接触、接触记录缺失、下降之前的腿失败、
腿未记录、配合记录缺失——**全部返回 OTHER_FAILURE 而不是预期中的卡死**。

### 42.3 又加一条契约检查

```text
编排脚本映射并调用的腿集合，必须与序列定义的完全一致
```

否则**改个名字就会静默跳过一条腿**，而 episode 仍能组装（缺失腿被记为
`LEG_NOT_RUN`，但没有任何东西发现那是改名造成的）。实测：把 `lift_module` 改成
`lift_modul` → 立刻报错 ✓

### 42.4 通过 / 失败

```text
契约检查（含编排一致性）                        PASS，且实测能触发
test_insertion_episode_contract              9 passed
test_task_insertion_sequence                 7 passed
test_grasp_template                          9 passed
编排一致性（腿名/位姿/调用三方一致）            PASS
```

### 42.5 ⚠️ 运行时验证：**未做**

```text
test/run_insertion_sequence.sh 从未对活体仿真执行过
它复用的每一件都经过验收，但【编排本身未经验证】
脚本头部已如实写明
```

这是本项目里我**唯一交付了未经运行验证的代码**的地方，理由：它无法在沙箱里跑，
而用户选 C 需要它。**请第一次运行时把 `episode_dir` 里的 `episode.json` 发我**，
特别是 `verdict` 与 `failure_codes`——如果结果是 `OTHER_FAILURE`，
`first_failed_leg` 会指出问题在哪一腿。

### 42.6 预期结果

```text
按 C 的选择，预期是 PRECISION_FIT_JAM：
  前四腿 + insert_entry 成功
  insert_seated 在接触中中止
  contacts.json 记录到模块与夹具的接触
```

**如果结果是 `PASS`**（位置控制居然穿过了 0.04 mm），那也是真实结果，
而且会推翻"必须柔顺"的前提——同样值得记录。

---

## 四十三、第一次运行编排：**install 树缺新配置**（又是同一类问题）

```text
FileNotFoundError: .../install/cs625_bringup/share/cs625_bringup/config/cs625_insertion_sequence.yaml
```

`install/cs625_bringup/share/cs625_bringup/config/` 是**真实目录副本**，不是符号链接，
所以**上次 build 之后新增的配置根本不在里面**。实测缺三个：

```text
camera_extrinsics_sim.yaml
cs625_grasp_template.yaml
cs625_insertion_sequence.yaml
```

### 43.1 为什么 P7.1 当时能读到相机外参

因为 **launch 文件是符号链接**：`Path(__file__).resolve()` 落在**源码树**，
`parent.parent/config/` 于是指向源码 ✓。**只有 `config/` 是真实副本。**

### 43.2 修法与仓库既有做法一致

```text
先源码树  $repo_root/src/cs625_bringup/config/...
再 install $cs625_bringup_share/config/...
都没有 -> exit 2 并打印重新生成它的命令
```

这正是 `apply_task_scene.py` 已经在用的解析顺序。

### 43.3 这一类的第三次

```text
第一次  6 个 P7 脚本用 $CS625_APP_INSTALL/share/<pkg>（merge-install 假设）-> 加了契约检查
第二次  cs625_ap_description 的 meshes 目录没进 install（新增目录）-> 已记录
第三次  新增的 config 文件没进 install（真实副本目录）-> 本次
```

**共同点**：`--symlink-install` 只对部分安装规则生效，**新增文件/目录不会自动出现**。
已加的两条契约检查（文档命令、install 布局）覆盖不到这类；**这次的修法是"不依赖 install 树"**，
比再加一条检查更根本。
