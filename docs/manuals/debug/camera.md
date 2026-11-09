# 相机调试命令

> 相机是 Percipio PS800-E1，驱动在独立工作区 `~/cs625_camport_ws`，
> 由 `scripts/camera_mode.py` 启动/换档（只操作 `runtime/camera.pid` 记录的进程）。

## 1. 起相机 / 换档

```bash
source scripts/source_dev_env.sh --full
python3 scripts/camera_mode.py status      # 只看状态，不动相机
python3 scripts/camera_mode.py modeling    # 1280×960 彩色+深度+点云（默认用这个）
python3 scripts/camera_mode.py fast        # 640×480 仅彩色（25 fps，无深度）
python3 scripts/camera_mode.py high        # 彩色 2560×1920 + 深度 1280×960
python3 scripts/camera_mode.py stop        # 只停相机 launch
```

- **作用**：重启相机 launch 并实测帧率。分辨率只能在启动时设定，`ros2 param set` 改了不生效。
- **正常输出**：`mode=modeling fps=0.79 res=1280x960 depth=0.79fps@1280x960`。
  `fps` 是深度真实速率，`driver_fps` 是驱动自己报的。
- **失败含义**：
  - `Unsupported Depth stream resolution` → 该分辨率设备不支持（深度上限 1280×960）；
  - 一直 `waiting for first frame` → RVS 占着相机（Windows 端）或网线/供电问题；
  - `multiple percipio camera launch processes found` → 有多个相机 launch，脚本拒绝猜测，先手工停掉一个。

## 2. 看实时数据（用数字，不靠肉眼）

```bash
python3 scripts/camera_probe.py --duration 20            # 每 0.5 s 一行
python3 scripts/camera_probe.py --duration 120 --out runtime/probe.csv
```

- **作用**：reliable 订阅彩色与深度，打印累计帧数、彩色均值、深度有效点比例、中位深度。
- **正常输出**：帧数持续增长约 0.8/s（modeling 档），有效点 ~76%，中位深度随场景变化。
- **失败含义**：
  - 帧数一直是 0 → 发布者没起或 QoS 不匹配（大图用 reliable）；
  - 彩色均值不变但有效点掉 → 挡住的是红外光路，不是彩色镜头；
  - 帧数在涨、画面看着不动 → 0.8 fps 本来就慢，属正常。

## 3. 窗口遮挡测试（判断"哪个窗是什么"）

```bash
python3 scripts/camera_probe.py --duration 120
# 一次挡一扇窗，挡 3 秒，看两列数字
```

| 挡的位置 | 彩色均值 | 深度有效点 |
|---|---|---|
| 彩色镜头 | 掉到接近 0 | 基本不变 |
| IR 接收窗（左右任一） | 不变 | 崩到 ~0%（双目少一路） |
| IR 投射器 | 不变 | 崩到 ~0%（没有图案可三角测量） |
| 外壳/丝印 | 不变 | 不变 |

## 4. 偏振 A/B（装膜前后）

```bash
python3 scripts/polarizer_ab.py capture --state before --frames 10
# 装片、转到最暗
python3 scripts/polarizer_ab.py capture --state after  --frames 10
python3 scripts/polarizer_ab.py compare --before runtime/polarizer_ab/<...>_before \
                                        --after  runtime/polarizer_ab/<...>_after
```

- **作用**：静止采帧并对比"飞点比例 / 点距 / 有效点 / 亮度"。
- **判读**：飞点降到一半以下 = 有效；亮度不变而飞点也不变 = 膜没进光路或在 830 nm 不工作。

## 5. 档位普查（初步实验用）

```bash
python3 scripts/camera_census.py --modes fast modeling high --frames 4
```

产物在 `runtime/experiments/camera_census_<时间戳>/`：`summary.md`、`metrics.csv`、每档的图像与深度。
