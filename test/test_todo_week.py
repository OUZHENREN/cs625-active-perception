"""周待办归档脚本（所有者 2026-11-26 定：待办独立于工作日志，每周滚动）。

只测纯逻辑：解析 / 统计 / 渲染 / 归档不覆盖。真正的文件操作在临时目录里做，
绝不碰 docs/TODO.md。
"""

from __future__ import annotations

import importlib.util
import pathlib
import sys

ROOT = pathlib.Path(__file__).resolve().parents[1]
SPEC = importlib.util.spec_from_file_location("todo", ROOT / "scripts" / "todo_week.py")
todo = importlib.util.module_from_spec(SPEC)
sys.modules["todo"] = todo
SPEC.loader.exec_module(todo)

SAMPLE = """# 待办（当前周）

> 说明行不计入逻辑。

**本周**：2026-11-24（周一）～ 2026-11-30（周日）

## 一、真机

- [ ] 插入标定
- [x] 一键回停机位
- [!] 装机（阻塞：法兰未加工）

## 二、数据

- [ ] 夹爪电流
"""


def _write(tmp_path) -> pathlib.Path:
    path = tmp_path / "TODO.md"
    path.write_text(SAMPLE, encoding="utf-8")
    return path


def test_parse_and_stats(tmp_path):
    header, week_line, sections = todo.parse(_write(tmp_path))
    assert week_line.startswith("**本周**：2026-11-24")
    done, pending, blocked = todo.stats(sections)
    assert (done, pending, blocked) == (1, 2, 1)
    titles = [t for t, _ in sections]
    assert titles == ["一、真机", "二、数据"]


def test_week_range_and_name(tmp_path):
    _, week_line, _ = todo.parse(_write(tmp_path))
    start, end = todo.week_range(week_line)
    assert str(start) == "2026-11-24" and str(end) == "2026-11-30"
    name = todo.iso_week_name(start, end)
    assert name.endswith("_2026-11-24_2026-11-30.md") and "-W" in name


def test_rollover_drops_done_and_keeps_pending(tmp_path):
    path = _write(tmp_path)
    header, week_line, sections = todo.parse(path)
    start, end = todo.week_range(week_line)
    archive = todo.render(header, week_line, sections, drop_done=False, start=start, end=end)
    assert "- [x] 一键回停机位" in archive           # 归档保留全部
    fresh = todo.render(header, week_line, sections, drop_done=True,
                        start=end.replace(day=1), end=end)
    assert "- [x] 一键回停机位" not in fresh          # 新周不带已完成
    assert "- [ ] 插入标定" in fresh
    assert "- [!] 装机（阻塞：法兰未加工）" in fresh   # 阻塞项必须保留


def test_rollover_never_overwrites_existing_archive(tmp_path, monkeypatch):
    """同名归档已存在时必须拒绝，且不破坏已有文件（归档只新增不覆盖）。"""
    path = _write(tmp_path)
    archive_dir = tmp_path / "todos"
    monkeypatch.setattr(todo, "TODO", path)
    monkeypatch.setattr(todo, "ARCHIVE", archive_dir)
    header, week_line, _ = todo.parse(path)
    start, end = todo.week_range(week_line)
    archive_dir.mkdir()
    existing = archive_dir / todo.iso_week_name(start, end)
    existing.write_text("OLD ARCHIVE\n", encoding="utf-8")
    sys.argv = ["todo_week.py", "--rollover"]
    assert todo.main() == 2
    assert existing.read_text(encoding="utf-8") == "OLD ARCHIVE\n"
