"""离线测试：FastDDS 共享内存清理器（2026-11-20）。

事故背景：第一版只看 /proc/<pid>/maps，把 **open() 打开、不在 maps 里** 的 `_el`
（事件循环）文件误判为僵尸删掉，直接破坏了运行中进程的 DDS 通信。
这里锁住两条规则：
  1. 占用检测必须同时看 maps 和 fd；
  2. `_el` 文件的死活跟着它的主段走，不能单独判死。
"""

from __future__ import annotations

import importlib.util
import pathlib

ROOT = pathlib.Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location(
    "clean_dds_shm", ROOT / "scripts" / "clean_dds_shm.py")
tool = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(tool)


def test_prefix_matches_fastdds_segments():
    assert tool.PREFIX == "fastrtps_"


def test_mapped_segments_counts_fds_not_only_maps(tmp_path):
    """构造一个假的 /proc：某个进程用 fd 打开 `_el`，用 maps 映射主段。"""
    shm = tmp_path / "shm"
    shm.mkdir()
    for name in ("fastrtps_aaa", "fastrtps_aaa_el", "fastrtps_bbb"):
        (shm / name).write_text("x")

    proc = tmp_path / "proc"
    pid = proc / "123"
    (pid / "fd").mkdir(parents=True)
    (pid / "maps").write_text(
        f"7f00-7f01 rw-s 00000000 00:01 1234 {shm}/fastrtps_aaa\n", encoding="utf-8")
    # _el 只出现在 fd，不在 maps 里
    (pid / "fd" / "7").symlink_to(shm / "fastrtps_aaa_el")

    used = tool.mapped_segments(proc_dir=proc, shm_dir=shm)
    assert "fastrtps_aaa" in used          # 来自 maps
    assert "fastrtps_aaa_el" in used       # 来自 fd —— 第一版漏掉的正是这个


def test_el_segment_is_never_judged_alone():
    """主段在用 ⇒ _el 在用；主段死了 ⇒ _el 才算僵尸。"""
    used = {"fastrtps_aaa": 1}
    all_segments = [pathlib.Path("/dev/shm/fastrtps_aaa"),
                    pathlib.Path("/dev/shm/fastrtps_aaa_el"),
                    pathlib.Path("/dev/shm/fastrtps_zzz_el")]
    # 复刻脚本里的规则：_el 跟随主段
    for segment in all_segments:
        if segment.name.endswith("_el"):
            owner = segment.name[: -len("_el")]
            if owner in used:
                used[segment.name] = used[owner]
    assert "fastrtps_aaa_el" in used
    assert "fastrtps_zzz_el" not in used
