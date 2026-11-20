#!/usr/bin/env python3
"""清理 FastDDS 共享内存里的僵尸段（自动检测 + 修复，2026-11-20）。

为什么需要它
------------
FastDDS 给每个参与者（进程）在 /dev/shm 建一个 `fastrtps_<hash>` 段。进程异常退出
（被 SIGKILL、崩溃、断点停止）时段文件会残留。残留段会占住端口/句柄，导致新进程
创建参与者失败：

    [RTPS_TRANSPORT_SHM Error] Failed init_port fastrtps_port7005:
    open_and_lock_file failed

现象（今天遇到两次）：节点的**进程还活着，但 ROS 图里它的发布者全没了**——
`move_group` 是规划服务消失，`/camera/camera` 是图像发布者消失。看起来像"节点空壳"。

这个脚本自动判断"哪些段已经没有任何活进程映射"，只删这些；仍在用的一个都不动。

用法
----
    python3 scripts/clean_dds_shm.py              # 只报告，不删除
    python3 scripts/clean_dds_shm.py --clean      # 清理僵尸段
    python3 scripts/clean_dds_shm.py --threshold 64   # 仅当段数超过阈值才清理

退出码：0=正常（清理后无僵尸段）；1=仍有僵尸段（--clean 没给）；2=清理后仍有异常。

自愈集成：`start_real_stack.sh` 会在启动驱动栈**之前**调用
    python3 scripts/clean_dds_shm.py --clean --threshold 32
这样每次开工都会先把上一次运行留下的僵尸段收干净。
"""

from __future__ import annotations

import argparse
import os
import pathlib
import sys

SHM = pathlib.Path("/dev/shm")
PREFIX = "fastrtps_"


def segments() -> list[pathlib.Path]:
    try:
        names = sorted(os.listdir(SHM))
    except OSError:
        return []
    return [SHM / name for name in names if name.startswith(PREFIX)]


def mapped_segments(proc_dir: pathlib.Path | None = None,
                    shm_dir: pathlib.Path | None = None) -> dict[str, int]:
    """找出被活进程**占用**的段 → {段名: 占用它的进程数}。

    2026-11-20 教训：只看 /proc/<pid>/maps 会漏掉 `_el`（事件循环）文件——
    它们是用 open() 打开的普通文件、**不会出现在 maps 里**。第一版脚本因此把
    在用的 `_el` 当僵尸删掉，直接破坏了运行中进程的 DDS 通信。
    所以这里同时扫 maps 和 fd（文件描述符）。
    """
    used: dict[str, int] = {}
    proc = proc_dir or pathlib.Path("/proc")
    shm_prefix = str((shm_dir or SHM).resolve()) + "/"
    for entry in proc.iterdir():
        if not entry.name.isdigit():
            continue
        # 1) 内存映射（主段 fastrtps_<hash> 是 mmap 进来的）
        try:
            maps = (entry / "maps").read_text(errors="ignore")
        except (OSError, PermissionError):
            maps = ""
        for line in maps.splitlines():
            if shm_prefix not in line:
                continue
            name = line.rsplit(shm_prefix, 1)[-1].strip()
            if name.endswith(" (deleted)"):
                name = name[: -len(" (deleted)")].strip()
            if name.startswith(PREFIX):
                used[name] = used.get(name, 0) + 1
        # 2) 打开的文件描述符（_el 这类普通文件走这里）
        try:
            fds = list((entry / "fd").iterdir())
        except (OSError, PermissionError):
            continue
        for fd in fds:
            try:
                target = os.readlink(fd)
            except OSError:
                continue
            if shm_prefix in target and PREFIX in target:
                name = target.rsplit(shm_prefix, 1)[-1].strip()
                if name.startswith(PREFIX):
                    used[name] = used.get(name, 0) + 1
    return used


def main() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--clean", action="store_true", help="删除僵尸段（默认只报告）")
    parser.add_argument("--threshold", type=int, default=0,
                        help="仅当僵尸段数量达到该阈值时才清理（0=总是清理）")
    args = parser.parse_args()

    all_segments = segments()
    if not all_segments:
        print("[shm] /dev/shm 里没有 fastrtps 段")
        return 0

    used = mapped_segments()
    # 主段在用 ⇒ 它的 _el 也在用（_el 只是事件循环文件，不能单独判死）
    for segment in all_segments:
        if segment.name.endswith("_el"):
            owner = segment.name[: -len("_el")]
            if owner in used:
                used[segment.name] = used[owner]
    alive = [s for s in all_segments if s.name in used]
    stale = [s for s in all_segments if s.name not in used]
    stale_bytes = sum(s.stat().st_size for s in stale if s.exists())

    print(f"[shm] fastrtps 段共 {len(all_segments)} 个："
          f"在用 {len(alive)}，僵尸 {len(stale)}（{stale_bytes/1024/1024:.1f} MB）")

    if not stale:
        return 0
    if not args.clean:
        for item in stale[:10]:
            print(f"       僵尸段：{item.name}")
        if len(stale) > 10:
            print(f"       …另有 {len(stale) - 10} 个")
        print("[shm] 未清理（加 --clean 执行删除）")
        return 1
    if args.threshold and len(stale) < args.threshold:
        print(f"[shm] 僵尸段 {len(stale)} < 阈值 {args.threshold}，本次不清理")
        return 0

    removed = 0
    for item in stale:
        try:
            item.unlink()
            removed += 1
        except OSError as error:
            print(f"       删除失败 {item.name}: {error}")
    print(f"[shm] 已删除 {removed} 个僵尸段（释放约 {stale_bytes/1024/1024:.1f} MB）")

    remaining = [s for s in segments() if s.name not in mapped_segments()]
    if remaining:
        print(f"[shm] 仍有 {len(remaining)} 个僵尸段未删除（可能权限不足）")
        return 2
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
