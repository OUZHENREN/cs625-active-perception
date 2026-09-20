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
