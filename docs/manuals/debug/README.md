# 系统调试命令手册（经验整理）

> 建立：2026-11-09。用途：把"常用的系统调试命令"按**能看懂的方式**整理下来——
> 每条命令做什么、正常应该看到什么、关键字段是什么、失败分别意味着什么。
> 这里只记会反复用的命令；单次事故的排障过程写 `docs/worklogs/`。

## 通用三步（任何"看不到数据"的问题都先走这三步）

```bash
# 第 1 步：装环境。三层顺序固定，不要混别的发行版
source scripts/source_dev_env.sh --full

# 第 2 步：看 ROS 图上到底有没有发布者（不是"话题存不存在"）
ros2 node list
ros2 topic info /camera/color/image_raw      # 看 Publisher count

# 第 3 步：看数据是否在动（数字，不靠肉眼）
python3 scripts/camera_probe.py --duration 20
```

判读：

| 现象 | 说明 | 下一步 |
|---|---|---|
| `node list` 为空 | 没有节点在跑，或另一个终端 `ROS_DOMAIN_ID` 不同 | 去起栈的终端 `echo $ROS_DOMAIN_ID` 对照 |
| 有节点、话题 `Publisher count: 0` | 话题只是被订阅端创建出来的 | 换 `ros2 topic info` 确认，别信 `topic list` |
| 有发布者但探针帧数不涨 | 订阅端 QoS 不匹配（best-effort 收不到 reliable 大图会静默丢） | 用 reliable 订阅，见 `camera.md` |
| 帧数在涨但画面不动 | 相机帧率本来就低（深度档 0.8 fps） | 正常，看帧率别靠肉眼 |

## 分册

| 文件 | 内容 |
|---|---|
| [`camera.md`](camera.md) | 相机：起停、换档、帧率/深度质量、偏振 A/B |
| [`ros_graph.md`](ros_graph.md) | ROS 图与消息：节点、话题、QoS、时钟、bag |
