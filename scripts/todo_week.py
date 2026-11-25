#!/usr/bin/env python3
"""周待办归档与滚动（所有者 2026-11-26 定）。

待办只有一个入口：``docs/TODO.md``（当前周）。每周把上周归档到
``docs/todos/<年>-W<周>_<起>_<止>.md``，``TODO.md`` 只保留**未完成/阻塞**项。

用法
----
    python3 scripts/todo_week.py --show      # 只打印当前周与统计（不写文件）
    python3 scripts/todo_week.py --rollover  # 归档上周 + 重开本周（已完成项不带到新周）

安全
----
· 只动 ``docs/TODO.md`` 与 ``docs/todos/``；归档文件**只新增不覆盖**（同名已存在则报错退出）。
· 解析规则很窄：只认 ``## `` 标题与 ``- [ ]`` / ``- [x]`` / ``- [!]`` 行；
  其余文本（说明、表格）原样保留在文件头，不参与逻辑。
退出码：0 成功；2 归档文件已存在；3 文件结构异常。
"""

from __future__ import annotations

import argparse
import datetime as dt
import pathlib
import re
import sys

sys.stdout.reconfigure(line_buffering=True)

REPO = pathlib.Path(__file__).resolve().parents[1]
TODO = REPO / "docs" / "TODO.md"
ARCHIVE = REPO / "docs" / "todos"
HEADER_RE = re.compile(r"^\*\*本周\*\*：(\d{4}-\d{2}-\d{2})（.*?）～ (\d{4}-\d{2}-\d{2})")
ITEM_RE = re.compile(r"^- \[([ x!])\] (.*)$")


def _label(path: pathlib.Path) -> str:
    """仓库内显示相对路径；测试里路径可能在仓库外，退化成绝对路径。"""
    try:
        return str(path.relative_to(REPO))
    except ValueError:
        return str(path)


def parse(path: pathlib.Path) -> tuple[str, str, list[tuple[str, list[tuple[str, str]]]]]:
    """返回 (文件头, 周区间行, [(小节标题, [(标记, 内容), ...]), ...])。"""
    text = path.read_text(encoding="utf-8")
    lines = text.splitlines()
    header: list[str] = []
    week_line = ""
    sections: list[tuple[str, list[tuple[str, str]]]] = []
    current: list[tuple[str, str]] | None = None
    for line in lines:
        if line.startswith("## "):
            current = []
            sections.append((line[3:].strip(), current))
            continue
        match = ITEM_RE.match(line)
        if match and current is not None:
            current.append((match.group(1), match.group(2)))
            continue
        if not week_line:
            found = HEADER_RE.match(line)
            if found:
                week_line = line
                continue
        if current is None:
            header.append(line)
    if not week_line:
        raise SystemExit("TODO.md 里找不到 '**本周**：YYYY-MM-DD（…）～ YYYY-MM-DD' 行")
    return "\n".join(header), week_line, sections


def stats(sections) -> tuple[int, int, int]:
    done = sum(1 for _, items in sections for mark, _ in items if mark == "x")
    todo = sum(1 for _, items in sections for mark, _ in items if mark == " ")
    blocked = sum(1 for _, items in sections for mark, _ in items if mark == "!")
    return done, todo, blocked


def week_range(week_line: str) -> tuple[dt.date, dt.date]:
    found = HEADER_RE.match(week_line)
    assert found
    return (dt.date.fromisoformat(found.group(1)),
            dt.date.fromisoformat(found.group(2)))


def next_monday(date: dt.date) -> dt.date:
    return date + dt.timedelta(days=7 - date.weekday())


def iso_week_name(start: dt.date, end: dt.date) -> str:
    return f"{start.isocalendar().year}-W{start.isocalendar().week:02d}_{start}_{end}.md"


def render(header: str, week_line: str, sections, drop_done: bool,
           start: dt.date, end: dt.date) -> str:
    out = [header.rstrip(), "", f"**本周**：{start}（周一）～ {end}（周日）", ""]
    for title, items in sections:
        kept = [(m, t) for m, t in items if not (drop_done and m == "x")]
        if not kept and drop_done:
            continue
        out.append(f"## {title}")
        out.append("")
        for mark, text in kept:
            out.append(f"- [{mark}] {text}")
        out.append("")
    return "\n".join(out).rstrip() + "\n"


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--show", action="store_true")
    parser.add_argument("--rollover", action="store_true")
    args = parser.parse_args()
    if not TODO.exists():
        print(f"{TODO} 不存在")
        return 3
    header, week_line, sections = parse(TODO)
    done, todo, blocked = stats(sections)
    start, end = week_range(week_line)
    print(f"当前周：{start} ～ {end}｜已完成 {done}｜未完成 {todo}｜阻塞 {blocked}")
    for title, items in sections:
        pending = [t for m, t in items if m != "x"]
        if pending:
            print(f"  [{title}] 待办 {len(pending)} 条")
    if args.show or not args.rollover:
        return 0

    ARCHIVE.mkdir(parents=True, exist_ok=True)
    target = ARCHIVE / iso_week_name(start, end)
    if target.exists():
        print(f"归档文件已存在，不覆盖：{_label(target)}")
        return 2
    target.write_text(render(header, week_line, sections, drop_done=False,
                             start=start, end=end), encoding="utf-8")
    new_start = end + dt.timedelta(days=1)
    new_end = next_monday(new_start) - dt.timedelta(days=1)
    TODO.write_text(render(header, week_line, sections, drop_done=True,
                           start=new_start, end=new_end), encoding="utf-8")
    print(f"已归档：{_label(target)}")
    print(f"新一周：{new_start} ～ {new_end}（已完成项不带到新周，未完成/阻塞保留）")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
