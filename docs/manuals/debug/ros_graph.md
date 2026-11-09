# ROS 图与消息调试命令

> 这些命令都在 `source scripts/source_dev_env.sh --full` 之后执行。

## 1. 图里有什么

```bash
ros2 node list
ros2 topic list
ros2 topic info /camera/color/image_raw
```

- **作用**：看节点、话题、以及话题的**发布者/订阅者数量**。
- **关键字段**：`Publisher count` —— 判断"数据源在不在"必须看这个，不能看 `topic list`。
  订阅行为本身也会让话题出现在 `topic list` 里。
- **失败含义**：`node list` 为空 → 没有节点，或另一个终端 `ROS_DOMAIN_ID` 不同。

## 2. 帧率与内容

```bash
ros2 topic hz /camera/color/image_raw --window 10
ros2 topic echo /camera/color/camera_info --once --qos-reliability best_effort
```

- **作用**：前者统计实际到达帧率；后者读一帧内容（如内参）。
- **正常输出**：modeling 档 ~0.8 Hz；`camera_info` 的 `k` 前几个数是 931.47 / 0 / 648.70 …
- **失败含义**：
  - `hz` 无输出 → 订阅端没收到（QoS 或发布者不在）；
  - `echo` 卡住 → 该话题的 QoS 是 best-effort，加 `--qos-reliability best_effort`。

## 3. 环境与域

```bash
echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}"
echo "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-default}"
ros2 doctor --report
```

- **作用**：确认两个终端是否在同一个 ROS 域、同一个中间件实现。
- **失败含义**：域号不同 → 互相看不见，表现为"我这边一帧都收不到"。

## 4. 录包（需要回放排查时）

```bash
ros2 bag record -o runtime/bags/camera_$(date +%H%M%S) /camera/color/image_raw /camera/depth/image_raw /camera/color/camera_info
ros2 bag info runtime/bags/<目录>
```

- **注意**：0.8 fps 下 1 分钟约 50 帧，体积不小；只录需要的话题，录完记得关。
