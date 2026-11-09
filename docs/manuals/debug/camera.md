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
- **注意**：`--state before/after` 只是标签，程序不知道你什么时候换了光学件。
  必须在**两条 capture 命令之间**完成贴膜或转动，否则两组数据必然一样（就是今天的误区）。

### 4.0 一只手扶着膜也能做：交互式角度扫描（推荐）

```bash
python3 scripts/polarizer_ab.py sweep-capture \
    --baseline runtime/polarizer_ab/<...>_recv_only \
    --steps 8 --step 20 --frames 4
```

- **节拍**：屏幕提示"转到 XX°"→ 你转膜 → 按回车 → 脚本自动采 4 帧（约 5 s，你什么都不用做）→ 下一步。
- **全自动**：加 `--auto-seconds 15`，每 15 秒自动采一拍，屏幕上倒计时并显示目标角度，双手都能扶着膜。
- **结束自动排序**：给了 `--baseline` 就会在最后按飞点比例列出所有角度，直接看哪一个是正交角。
- **注意**：只转投射端那一片，接收端的膜不要动；膜先别粘死，用胶带临时固定。

### 4.1 找投射端偏振片的最佳角度（手动分组方式）

```bash
python3 scripts/polarizer_ab.py capture --state recv_only --frames 6      # 基线：只有接收端膜
# 贴投射端膜，转到 0°
python3 scripts/polarizer_ab.py capture --state proj_000 --frames 6
# 每转 20° 采一组：proj_020 / proj_040 / proj_060 / proj_080 / proj_100 / proj_120
python3 scripts/polarizer_ab.py sweep --baseline runtime/polarizer_ab/<...>_recv_only \
    --candidates runtime/polarizer_ab/*_proj_*
```

- **作用**：把每组与基线比较并按"飞点比例"排序，直接给出飞点最少的那个角度。
- **正常输出**：某几组飞点明显低于基线（`相对基线` 一列为负且绝对值大）；最好的一组就是正交角。
- **失败含义**：所有角度都与基线相当 → 膜不消光（换 800–900 nm 专用膜）或投射光本身不偏振。

## 5. 档位普查（初步实验用）

```bash
python3 scripts/camera_census.py --modes fast modeling high --frames 4
```

产物在 `runtime/experiments/camera_census_<时间戳>/`：`summary.md`、`metrics.csv`、每档的图像与深度。
