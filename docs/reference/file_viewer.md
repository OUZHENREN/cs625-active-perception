# 文件查看器（在 Windows 浏览器里看 WSL 的文件）

## 为什么不用资源管理器
`\\wsl.localhost\...` 打开慢，mp4 通常不能预览、也不能拖进度条。这个工具起一个只读的
本地网页服务，浏览器里直接播视频、看图片、读文本。

## 用起来

```bash
cd ~/projects/cs625-active-perception
source scripts/source_dev_env.sh --full      # 可选，本脚本不需要 ROS

python3 scripts/file_viewer.py                       # 默认服务 ~/cs625_evidence，端口 8090
python3 scripts/file_viewer.py --root ~ --port 8090  # 服务整个 home（能进 cs625_evidence）
python3 scripts/file_viewer.py --root ~ --open       # 起完自动开浏览器
```

Windows 浏览器打开：**http://localhost:8090/**

- 视频：点进去直接在页面里播放，**支持拖动进度条**（HTTP Range）；
- 图片：页面内预览；
- `.md/.log/.yaml/.json/.csv/.py/.sh`：页面内直接显示（不会被当成下载）；
- 目录：可逐级点进去，面包屑能跳回。

VS Code 里也有对应任务：`50 · 文件查看器（浏览器看视频/图片/日志）`。

## 演示视频在哪
```text
~/cs625_evidence/sim_load_efficient_20261025.webm    7.9 MB  322 帧  4 fps
```
在查看器里：`cs625_evidence/` → 点文件名 → 播放页（带下载链接）。

**为什么是 webm 不是 mp4**：原先用 OpenCV 写的 `mp4v`（MPEG-4 Part 2）浏览器**解不了**，
页面里只有一个 0:00 的播放器；Chromium/Edge 只认 H.264 的 mp4 或 VP8/VP9 的 webm。
本机 OpenCV 没有 H.264 编码器（`avc1` 打不开），VP9 可以，所以改成 `webm`。

## 安全边界（刻意很窄）
- 只读：只实现 GET/HEAD，没有上传/删除/重命名；
- 只服务 `--root` 以下，`..` 越界一律 403（已实测 `curl --path-as-is /../../etc/passwd` → 403）；
- 默认只监听 `127.0.0.1`（`--host 0.0.0.0` 会把文件暴露给局域网，不建议）；
- 停止：Ctrl+C，或关掉 VS Code 里那个任务面板。

## 如果浏览器打不开 localhost
这台机器的 WSL 是镜像网络（WSL IP 就是局域网地址），一般 `localhost` 直通。若不通，用：

```bash
hostname -I        # 取 WSL 的 IP
# 然后访问 http://<那个 IP>:8090/
```
并按需执行 `wsl_expose`（仅当确实需要从 Windows 访问 WSL 端口时）。
