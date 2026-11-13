#!/usr/bin/env python3
"""仿真专用：生成 robot_description，并做两处"仿真专属、画面上不可见"的注入。

为什么需要这一层
================
仿真跑的是**真机那份描述**（`eli_cs_robot_description/cs625.urdf.xacro`）——
所有者 2026-10-25 明确定下的参考模型：它的 `my_end_effector_link` 挂
`tool0_vision.STL`，一个 CAD 网格里同时含夹爪、两条支撑臂、相机安装座和相机。
厂商包是只读的，项目规则不允许改，所以这里在生成 `robot_description` 时做一次
**纯文本后处理**：不改任何厂商文件、不改配置，只把两样仿真必须的东西塞进已经
展开好的 URDF。

注入 1：`position_proportional_gain`（ros2_control 位置环刚度）
----------------------------------------------------------------
厂商 `<hardware>` 块里没有这个参数，gz_ros2_control 会用默认 0.1，位置环留下
0.01~0.05 rad 稳态静差（"到位"差一截）。它只能写在 URDF 的 `<hardware>` 块里，
运行中设不了。默认值 0.5：落在 Jazzy 文档"≤1 非振荡"范围内，也是 2026-10-25 全流程里
自由运动阶段（6 个 observe 阶段）都能过 goal 容差的那个值。
实测取舍：增益是一阶积分环的收敛速度（时间常数 1/增益）。把增益提到 5.0 后，
接触工况下的末端误差反而从 0.049 rad 放大到 0.185 rad（接触力更大 → 互相顶），
所以不要为了"跟得更紧"盲目调大；自由运动若出现滞后再单独评估。
可用 CS625_POSITION_PROPORTIONAL_GAIN 覆盖。

注入 2：Gazebo 官方 `DetachableJoint`（仿真里的"夹住"）
--------------------------------------------------------
真实夹紧靠夹爪摩擦，仿真里没有力闭环可依赖。今天尝试过的"用 set_pose 服务把
工件位姿跟随到工具上"在物理上不成立：set_pose 只改位姿、不重置速度，重力会让
速度无界累积（实测 2 s 内工件被甩到 z ≈ -1300 m）。正确做法是物理引擎里的
真约束：Gazebo 官方的 `gz-sim-detachable-joint-system`（本机 gz-sim8 8.11.0 自带，
官方示例世界 `detachable_joint.sdf` 就是"插件写在机器人模型里、child 是世界里的
另一个模型"这个拓扑）。抓取时发一次 attach、释放时发一次 detach，工件与腕部刚性
同体；attach 按**触发那一刻的当前相对位姿**建固定关节，不会把工件瞬移到名义夹持位。

注意：这个插件**启动时就处于"已吸附"状态**（源码 attachRequested{true}），所以
`scripts/sim_carry_workpiece.py` 启动后第一件事必须是发 detach 并确认。

注入 3：去掉 `camera_link` 的 `<collision>`（仿真物理专用）— 2026-10-26
--------------------------------------------------------------------
厂商/仓库描述给相机加了一个 0.03×0.05×0.02 m 的占位碰撞盒，位置比
`my_end_effector_link` 还低 5 cm（真机相机实际在法兰前 0.097 m、比 TCP 高
0.256 m）。它成了整个工具的最低点，于是 pre_grasp 位姿要求它钻进工件桌桌面
35 mm 才能到位 → Gazebo 里关节被桌面**顶住**，shoulder_lift 差 0.03~0.04 rad，
末端停在目标上方 39 mm、工具轴 3°，JTC 到 goal_time 报 GOAL_TOLERANCE_VIOLATED。
仿真里相机只是传感器（camera_link 的 visual + rgbd_camera），这个碰撞盒不参与
任何真实抓取，所以去掉它的碰撞；视觉网格、传感器、TF、相机外参全部不动。
详见 `strip_camera_collision()` 里的实测数据。

用法（由 sim_control.launch.py 的 prepare 步骤与 sim_moveit.launch.py 调用，
等价于原来的 xacro 调用）
------------------------------------------------------------------------
    python3 scripts/sim_robot_description.py <xacro 可执行> <urdf.xacro> [key:=value ...]

环境变量
--------
    CS625_POSITION_PROPORTIONAL_GAIN   注入的刚度值，默认 0.5；0 或空则不注入。
    CS625_SIM_CARRY_ATTACH             1（默认）= 注入 DetachableJoint；0 = 不注入。
    CS625_SIM_TOOL_COLLISION_PROXY     1（默认）= 工具碰撞换成闭合态盒子；0 = 用原网格。
    CS625_SIM_CAMERA_COLLISION         0（默认）= 去掉相机碰撞盒；1 = 保留
                                       （保留后 pre_grasp 会被工件桌顶住，仅用于复现）。
"""

from __future__ import annotations

import os
import re
import subprocess
import sys
from pathlib import Path

DEFAULT_GAIN = 0.5

# DetachableJoint 插件片段。parent_link 由 inject_detachable_joint() 从展开后的
# URDF 里取真实 link 名（兼容 prefix:=），child 是生成世界里的工件模型/link。
# 话题名刻意避开 P7 时代的 /p7/attachment/*，两套机制互不牵连。
DETACHABLE_JOINT = """  <gazebo>
    <plugin filename="gz-sim-detachable-joint-system"
            name="gz::sim::systems::DetachableJoint">
      <!-- 只能是腕部 3 号 link：URDF→SDF 会把 flange/tool0/my_end_effector_link
           这些固定关节全部 lump 进它，那些名字在 SDF 里只剩 <frame>，不是 link。 -->
      <parent_link>{parent_link}</parent_link>
      <child_model>shielding_module</child_model>
      <child_link>shielding_module_link</child_link>
      <attach_topic>/cs625/carry/attach</attach_topic>
      <detach_topic>/cs625/carry/detach</detach_topic>
      <output_topic>/cs625/carry/state</output_topic>
      <!-- 找不到 child 时每个 PreUpdate 都会 gzwarn（1 kHz 刷屏），所以静音；
           失败信号由 /cs625/carry/state 收不到 "attached" 体现。 -->
      <suppress_child_warning>true</suppress_child_warning>
    </plugin>
  </gazebo>
"""


def inject(xml: str, gain: float) -> str:
    """把 <param name="position_proportional_gain"> 插到第一个 <hardware> 块里。

    只要整份描述里已经有 position_proportional_gain 就完全不动（仓库包装层
    cs625_active_perception.urdf.xacro 在 gz_ros2_control 的 <plugin> 块里自带
    0.5，那是上一段对话验收过 18/19 的配置）；只有厂商描述这种完全没有该参数的
    输入才会注入 DEFAULT_GAIN。
    """

    param = f'<param name="position_proportional_gain">{gain:g}</param>'
    if "position_proportional_gain" in xml:
        return xml  # 已经有了，不重复插
    m = re.search(r"<hardware>", xml)
    if not m:
        return xml  # 没有硬件块（例如纯描述模式），原样返回
    at = m.end()
    return xml[:at] + param + xml[at:]


TOOL_COLLISION_PROXY = (
    '<collision><origin xyz="{origin}"/><geometry><box size="{size}"/></geometry>'
    '</collision>'
)
# 闭合态工具碰撞代理的默认值（米）：x 60 mm × y **140 mm** × z 214 mm，
# 沿工具轴中心在 link 系 z=-0.253（= -(0.360+0.146)/2，工具网格在 eef 系
# 占 z -0.360…-0.146）。
#
# 为什么 y 取 140 而不是"闭合跨距 204"：离线按实测工件位姿算过（2026-10-26），
# 204 mm 宽的盒子在 **pre_grasp 位姿处会与工件 body 碰撞盒重叠 12 mm**（工件系 x），
# 也就是说拿着"闭合态双臂"在预抓取点就已经顶到工件了 —— 与仿真里"pre_grasp 把
# 6.8 kg 工件推走"的现象一致。140 mm 是 CAD 审计量到的**相机装配体 y 向包络
# (±71 mm)**，也就是真机上不随夹爪收拢的那部分；按 140 mm 重算，pre_grasp 与
# grasp 两个位姿都与工件不接触。支撑臂本身在真机是收拢后进入 233 mm 槽口的，
# 静态 URDF 无法表达，所以这里只建"不动的核心体"。
DEFAULT_PROXY_SIZE = "0.060 0.140 0.214"
DEFAULT_PROXY_ORIGIN = "0 0 -0.253"


def inject_plugin_gain(xml: str, gain: float) -> str:
    """把 <position_proportional_gain> 写进 gz_ros2_control 的 **<plugin>** 元素。

    为什么两处都要：仓库包装层（上一段对话验收过 18/19 的那份）把这个增益写在
    <plugin> 里，而厂商描述里没有；而 gz_ros2_control 插件自己会声明一个同名
    ROS 参数（库里有 "Parameter 'position_proportional_gain' has already been
    declared / invalid name / wrong type" 这些字符串）。2026-10-26 00:30 的实测：
    只写进 <hardware><param> 时 pre_grasp 末端跟踪误差仍有 0.042~0.062 rad（> 0.01
    容差 → 动作 ABORT）。所以两处都写，值一致，避免"到底读了哪一处"这种暗坑。
    插件块不存在时原样返回（其它包装层可能已经注入过）。
    """

    element = f"<position_proportional_gain>{gain:g}</position_proportional_gain>"
    if "position_proportional_gain" in xml.split("<plugin")[-1][:2000] and \
            re.search(r"<plugin[^>]*gz_ros2_control[^>]*>(?:(?!</plugin>).)*?"
                      r"position_proportional_gain", xml, re.S):
        return xml  # 插件块里已经有了
    m = re.search(
        r'(<plugin[^>]*name="gz_ros2_control::GazeboSimROS2ControlPlugin"[^>]*>)'
        r'(.*?)(</plugin>)', xml, re.S)
    if not m:
        return xml
    return (xml[:m.start()] + m.group(1) + m.group(2) + "      " + element + "\n    "
            + m.group(3) + xml[m.end():])


def inject_tool_collision_proxy(xml: str, size: str, origin: str) -> str:
    """把工具 link 的碰撞体换成"夹爪闭合态"的细长盒子（只影响 Gazebo 物理）。

    为什么必须换（所有者 2026-10-25 批准，方案 A）：
      真机的夹爪是**可收拢**的——进槽前先收到 204 mm 跨距，进去以后再外撑到
      246.57 mm 顶住两条导轨。厂商 `tool0_vision.STL` / `tool0_collision.STL`
      画的是**张开态**（y 向 240 mm），而槽口自由跨距只有 233 mm：静态 URDF 没有
      收拢动作，工具一进就顶住工件，实测把 6.8 kg 工件推走 0.4 m 掉下桌
      （pre_grasp/grasp 的 GOAL_TOLERANCE 误差从 0.017 rad 放大到 0.185 rad）。
      所以在仿真里把**碰撞**换成闭合态包络（视觉网格不动，Gazebo 里看到的仍是
      厂商 CAD），抓取由 DetachableJoint 完成；插槽段本来就按 0.005 m/s 且不依赖
      碰撞规划，工件自己仍有真实碰撞。

    只替换工具 link 里的**第一个** `<collision>`；找不到工具 link 或没有碰撞块就
    原样返回并在 stderr 说一句（不猜、不静默改别的 link）。
    """

    m = re.search(r'<link\s+name="([^"]*my_end_effector_link)"\s*>(.*?)</link>',
                  xml, re.S)
    if not m:
        sys.stderr.write(
            "[sim_robot_description] 找不到 my_end_effector_link，跳过工具碰撞代理"
            "（仿真里会保留张开态碰撞，进槽可能顶工件）\n")
        return xml
    body = m.group(2)
    box = TOOL_COLLISION_PROXY.format(origin=origin, size=size)
    new_body, count = re.subn(r"<collision>.*?</collision>", box, body, count=1,
                              flags=re.S)
    if count == 0:
        new_body = body + box
    return xml[:m.start()] + m.group(0).replace(body, new_body, 1) + xml[m.end():]


def strip_camera_collision(xml: str) -> str:
    """去掉相机 link 的 `<collision>`（只影响 Gazebo 物理，视觉与传感器不动）。

    为什么（2026-10-26 实测根因：pre_grasp 被桌面顶住）：
      相机碰撞盒 0.03×0.05×0.02 m 挂在 wrist_3_link 局部 (0.004, 0, 0.403)、
      rpy(0, 0.785, 0)，也就是比 `my_end_effector_link` 还低 5 cm，成了整个工具
      **最低点**。pre_grasp 位姿要求它钻进工件桌桌面 35 mm 才能到位，Gazebo 里
      物理上走不到，于是关节被桌面顶住：shoulder_lift 差 0.03~0.04 rad、末端滞差
      39 mm、工具轴 3°，JTC 到 goal_time 报 GOAL_TOLERANCE_VIOLATED。

      实测数据（正向运动学已用 Gazebo `/pose/info` 与 MoveIt `/compute_fk` 双向
      校验，误差 < 0.5 mm）：
        * 停住时实测关节角 → 相机盒世界 z = [-0.09098, -0.05506]，
          工件桌桌面顶面 z = -0.09097（相差 0.01 mm，正好压在桌面上）；
        * 计划终点关节角 → 相机盒世界 z = [-0.12595, -0.09024]，要钻进桌面 35 mm；
        * 交叉验证：把 elbow +0.2 rad 移出折叠姿态后，同一条 shoulder_lift 到
          -2.5 的指令立刻可走到 -2.099；只有 shoulder_lift 短、其余 5 关节误差 0.000。
      真机 2026-10-22 跑通过同一个 pre_grasp 位（common.yaml 的数值就来自那次），
      说明真实相机不占这个位置。相机在仿真里只是传感器（camera_link 的 visual +
      rgbd_camera），碰撞盒对物理没有用处，去掉它才是"与真机 1:1 对齐"。

    只改 camera_link 里的 `<collision>`；找不到该 link 或没有碰撞块就原样返回并
    在 stderr 说一句（不猜、不静默改别的 link）。
    """

    m = re.search(r'<link\s+name="([^"]*camera_link)"\s*>(.*?)</link>', xml, re.S)
    if not m:
        sys.stderr.write(
            "[sim_robot_description] 找不到 camera_link，跳过相机碰撞剔除"
            "（pre_grasp 可能被桌面顶住）\n")
        return xml
    body = m.group(2)
    new_body, count = re.subn(r"<collision>.*?</collision>", "", body, flags=re.S)
    if count == 0:
        return xml
    # 成功路径必须**静默**：prepare_gazebo_model.py 把预处理命令的任何 stderr 输出
    # 当成失败（"executed command showed stderr output" → launch 直接中止，
    # 2026-10-26 实测踩过）。所以这里不打印，去掉了什么写在本文档里。
    return xml[:m.start()] + m.group(0).replace(body, new_body, 1) + xml[m.end():]


# ---------------------------------------------------------------- 物理夹爪
# 真机的夹爪是"外撑式"：两条支撑臂张开（EFG opening 0–50 mm ↔ 跨距 204→246.57 mm），
# 撑住工件扶手（两条导轨）的内壁靠摩擦夹住 6.8 kg 工件。厂商描述里这两条臂只是
# tool0_vision.STL 里的一段静态几何、没有关节，所以仿真一直只能"焊住"工件。
# 这里把**真实 CAD 里拆出来的两条支撑臂**（58×34×146 mm，各 1,382 面，位于 y=±103 mm）
# 做成两个 prismatic 关节 + 位置控制，夹爪本体用"去掉两条臂"的同一份 CAD。
#
# 两个关键数值（都有依据）：
#   GRIPPER_MOUNT_Z = 0.203 m：厂商 CAD 装到 TCP 上时，两条支撑臂（CAD 里在 z -0.203..-0.349）
#     整段落在 TCP 后方 203~349 mm = 工件顶部之上，撑不到导轨；把这 203 mm 补上，
#     支撑臂正好落在 TCP 向下 0~146 mm（工件 z 0.033~0.179，完全在导轨高度范围内）。
#     （工作日志 2026-10-25 记的"未建模支架"）；把这 146 mm 补上，工具尖端才落在真机
#     TCP（法兰前 353 mm）上，两支撑臂也才进得了工件扶手。
#   GRIPPER_EFFORT = 300 N/侧：工件 6.8 kg → 186 N，导轨摩擦 μ=0.6 ⇒ 每侧法向力需 ≥155 N；
#     仓库旧模型的 80 N/侧在物理上会滑落。
GRIPPER_MOUNT_Z = 0.203
GRIPPER_STROKE = 0.021285
GRIPPER_EFFORT = 300.0

GRIPPER_BLOCK = """  <link name="{prefix}gripper_right_finger_link">
    <visual><geometry><mesh filename="file://{mesh_dir}/gripper_arm_right.stl"/></geometry>
      <material name="gripper_arm"><color rgba="0.6 0.6 0.62 1.0"/></material></visual>
    <!-- 碰撞体用不可见盒子代理（58×34×146 mm，与真实支撑臂同尺寸同位置）；
         视觉仍是真实 CAD 手臂。mesh 碰撞在 dartsim 里建不出来（日志实见
         "The geometry element of collision ... couldn't be created"）。 -->
    <collision><origin xyz="0.026 0.085 -0.276"/><geometry><box size="0.058 0.034 0.146"/></geometry>
      <surface><friction><ode><mu>0.6</mu><mu2>0.6</mu2></ode></friction></surface></collision>
    <inertial><mass value="0.35"/>
      <inertia ixx="2.0e-4" ixy="0" ixz="0" iyy="1.2e-3" iyz="0" izz="1.2e-3"/></inertial>
  </link>
  <link name="{prefix}gripper_left_finger_link">
    <visual><geometry><mesh filename="file://{mesh_dir}/gripper_arm_left.stl"/></geometry>
      <material name="gripper_arm"><color rgba="0.6 0.6 0.62 1.0"/></material></visual>
    <!-- 碰撞体用不可见盒子代理（58×34×146 mm，与真实支撑臂同尺寸同位置）；
         视觉仍是真实 CAD 手臂。mesh 碰撞在 dartsim 里建不出来（日志实见
         "The geometry element of collision ... couldn't be created"）。 -->
    <collision><origin xyz="0.026 -0.085 -0.276"/><geometry><box size="0.058 0.034 0.146"/></geometry>
      <surface><friction><ode><mu>0.6</mu><mu2>0.6</mu2></ode></friction></surface></collision>
    <inertial><mass value="0.35"/>
      <inertia ixx="2.0e-4" ixy="0" ixz="0" iyy="1.2e-3" iyz="0" izz="1.2e-3"/></inertial>
  </link>
  <joint name="{prefix}gripper_right_finger_joint" type="prismatic">
    <parent link="{prefix}my_end_effector_link"/><child link="{prefix}gripper_right_finger_link"/>
    <origin xyz="0 0 {mount_z}"/><axis xyz="0 1 0"/>
    <limit lower="0" upper="{stroke}" effort="{effort}" velocity="0.05"/>
  </joint>
  <joint name="{prefix}gripper_left_finger_joint" type="prismatic">
    <parent link="{prefix}my_end_effector_link"/><child link="{prefix}gripper_left_finger_link"/>
    <origin xyz="0 0 {mount_z}"/><axis xyz="0 -1 0"/>
    <limit lower="0" upper="{stroke}" effort="{effort}" velocity="0.05"/>
  </joint>
  <gazebo reference="{prefix}gripper_right_finger_link"><gravity>false</gravity></gazebo>
  <gazebo reference="{prefix}gripper_left_finger_link"><gravity>false</gravity></gazebo>
  <ros2_control name="{prefix}cs625_gripper" type="system">
    <hardware><plugin>gz_ros2_control/GazeboSimSystem</plugin></hardware>
    <joint name="{prefix}gripper_right_finger_joint">
      <command_interface name="position"/>
      <state_interface name="position"/><state_interface name="velocity"/><state_interface name="effort"/>
    </joint>
    <joint name="{prefix}gripper_left_finger_joint">
      <command_interface name="position"/>
      <state_interface name="position"/><state_interface name="velocity"/><state_interface name="effort"/>
    </joint>
  </ros2_control>
"""


def inject_gripper(xml: str) -> str:
    """把真实 CAD 的两条支撑臂做成可动夹爪，并把工具本体按 146 mm 支架外移。

    - 工具本体：`my_end_effector_link` 的 visual 换成"去掉两条臂"的真实 CAD，沿工具轴
      外移 146 mm（补上未建模支架），碰撞体交给两条臂（本体不加碰撞，避免凸包把
      工件槽口填满）。
    - 两条支撑臂：独立的 link + prismatic 关节（行程 0.021285 m/侧、effort 300 N），
      由已有的 `gripper_controller`（配置里就叫这两个关节名）驱动。
    幂等；找不到工具 link 就原样返回。
    """
    if "gripper_right_finger_joint" in xml:
        return xml
    m = re.search(r'<link\s+name="([^"]*my_end_effector_link)"\s*>(.*?)</link>', xml, re.S)
    if not m:
        return xml
    prefix = m.group(1)[: -len("my_end_effector_link")]
    mesh_dir = str((Path(__file__).resolve().parent.parent
                    / "src" / "cs625_ap_description" / "meshes" / "tool"))
    body = (
        f'<link name="{m.group(1)}">'
        f'<visual><origin xyz="0 0 {GRIPPER_MOUNT_Z}"/><geometry>'
        f'<mesh filename="file://{mesh_dir}/tool0_vision_body.stl"/></geometry>'
        '<material name="eef_color"><color rgba="0.75 0.75 0.75 1.0"/></material></visual>'
        '</link>'
    )
    out = xml[: m.start()] + body + xml[m.end():]
    block = GRIPPER_BLOCK.format(prefix=prefix, mesh_dir=mesh_dir,
                                 mount_z=GRIPPER_MOUNT_Z, stroke=GRIPPER_STROKE,
                                 effort=GRIPPER_EFFORT)
    at = out.rfind("</robot>")
    return out[:at] + block + out[at:] if at >= 0 else out


def inject_detachable_joint(xml: str) -> str:
    """把 Gazebo 的 DetachableJoint 插件追加到 </robot> 之前（仿真附着机制）。

    幂等：描述里已经有 gz-sim-detachable-joint-system 就不重复注入（历史
    P7 附着块已于 2026-11-13 归档并从 xacro 移除，此检查仍保留以防御其它
    注入源）。parent_link 从展开后的 URDF 里取真实名字，
    取不到就原样返回并在 stderr 说一句，绝不猜名字（猜错的表现是插件 Configure
    直接失败、连话题都不会建）。
    """

    if "gz-sim-detachable-joint-system" in xml:
        return xml
    m = re.search(r'<link\s+name="([^"]*wrist_3_link)"', xml)
    if not m:
        sys.stderr.write(
            "[sim_robot_description] 找不到 wrist_3_link，跳过 DetachableJoint 注入"
            "（仿真附着会不生效）\n")
        return xml
    block = DETACHABLE_JOINT.format(parent_link=m.group(1))
    at = xml.rfind("</robot>")
    if at < 0:
        return xml
    return xml[:at] + block + xml[at:]


def main() -> int:
    if len(sys.argv) < 3:
        print(__doc__, file=sys.stderr)
        return 2
    xacro_exe, urdf_path, *rest = sys.argv[1:]

    proc = subprocess.run([xacro_exe, urdf_path, *rest],
                          capture_output=True, text=True)
    if proc.returncode != 0:
        sys.stderr.write(proc.stderr)
        return proc.returncode

    raw_gain = os.environ.get("CS625_POSITION_PROPORTIONAL_GAIN", str(DEFAULT_GAIN)).strip()
    try:
        gain = float(raw_gain)
    except ValueError:
        gain = DEFAULT_GAIN
    out = proc.stdout if gain <= 0 else inject(proc.stdout, gain)
    if gain > 0:
        out = inject_plugin_gain(out, gain)

    # 物理外撑夹爪默认**关闭**：厂商 CAD 里工具占法兰前 0..207 mm，而真机 TCP 在 353 mm，
    # 中间 146 mm 是**没有任何 CAD 描述**的支架；把工具外移会让它和法兰之间出现 146+ mm
    # 的悬空间隙（画面实测："夹爪压根没连到法兰上"）。这段几何没测出来之前，先回到
    # 厂商原装位（视觉连续）+ DetachableJoint 附着；需要时用 CS625_SIM_GRIPPER=1 打开。
    raw_gripper = os.environ.get("CS625_SIM_GRIPPER", "0").strip().lower()
    if raw_gripper in ("1", "true", "yes", "on"):
        out = inject_gripper(out)

    raw_attach = os.environ.get("CS625_SIM_CARRY_ATTACH", "1").strip().lower()
    if raw_attach in ("1", "true", "yes", "on"):
        out = inject_detachable_joint(out)

    raw_proxy = os.environ.get("CS625_SIM_TOOL_COLLISION_PROXY", "1").strip().lower()
    if raw_proxy in ("1", "true", "yes", "on"):
        out = inject_tool_collision_proxy(
            out,
            os.environ.get("CS625_SIM_TOOL_PROXY_SIZE", DEFAULT_PROXY_SIZE),
            os.environ.get("CS625_SIM_TOOL_PROXY_ORIGIN", DEFAULT_PROXY_ORIGIN))

    # 相机碰撞盒比 TCP 还低 5 cm，pre_grasp 位姿会被工件桌顶住（见
    # strip_camera_collision 的实测数据）。默认去掉；CS625_SIM_CAMERA_COLLISION=0
    # 可恢复原样，用来复现"被桌面顶住"的失败或做 A/B 对比。
    raw_cam = os.environ.get("CS625_SIM_CAMERA_COLLISION", "0").strip().lower()
    if raw_cam not in ("1", "true", "yes", "on"):
        out = strip_camera_collision(out)

    sys.stdout.write(out)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
