#!/usr/bin/env python3
"""Consistency checks for the documents this project relies on.

Why (owner, 2026-11-07): a whole batch of manual entries was reported as "added" while
nothing changed on disk, because the script anchored on a heading that had since been edited
and str.replace silently matched nothing.  Documentation edits need the same verification as
code edits, so this script checks the properties that a silent failure would break:

  1. every "## N." section in the manual is present exactly once and numbered 1..N without gaps;
  2. the index table rows match the sections one-to-one;
  3. every repository path mentioned in the docs actually exists;
  4. every commit hash mentioned in the docs exists in git;
  5. agent_memory.md has no duplicated top-level bullet keys.

Usage:
    python3 scripts/check_docs.py [--verbose]

Exit codes: 0 all checks pass; 1 at least one problem found; 2 could not read a document.
"""

from __future__ import annotations

import argparse
import pathlib
import re
import subprocess
import sys

sys.stdout.reconfigure(line_buffering=True)

REPO = pathlib.Path(__file__).resolve().parents[1]
MANUAL = REPO / "docs" / "manuals" / "2026-11-03_MANUAL_REAL_MACHINE_EXPERIENCE.md"
AGENT_MEMORY = REPO / "docs" / "agent_memory.md"
DOC_GLOBS = ("docs/**/*.md", "README.md", "AGENTS.md")

PROBLEMS: list[str] = []


def problem(text: str) -> None:
    PROBLEMS.append(text)
    print(f"  [FAIL] {text}")


def section_numbers(text: str) -> list[int]:
    return [int(m) for m in re.findall(r"^## (\d+)\.", text, flags=re.MULTILINE)]


def check_manual() -> None:
    if not MANUAL.exists():
        problem(f"manual missing: {MANUAL.relative_to(REPO)}")
        return
    text = MANUAL.read_text()
    numbers = section_numbers(text)
    print(f"manual: {len(numbers)} numbered sections")
    if not numbers:
        problem("manual has no numbered sections")
        return
    expected = list(range(1, max(numbers) + 1))
    if sorted(numbers) != expected:
        missing = sorted(set(expected) - set(numbers))
        extra = sorted(set(numbers) - set(expected))
        duplicates = sorted({n for n in numbers if numbers.count(n) > 1})
        problem(f"manual section numbering: missing={missing} unexpected={extra} "
                f"duplicated={duplicates}")
    else:
        print(f"  [ok] sections 1..{max(numbers)} complete and unique")

    index_rows = [int(m) for m in re.findall(r"^\| (\d+) \|", text, flags=re.MULTILINE)]
    if sorted(index_rows) != expected:
        problem(f"manual index rows {len(index_rows)} do not match {len(expected)} sections "
                f"(missing={sorted(set(expected) - set(index_rows))}, "
                f"extra={sorted(set(index_rows) - set(expected))})")
    else:
        print(f"  [ok] index table lists all {len(index_rows)} entries")


def collect_referenced_paths() -> dict[str, set[str]]:
    """Repository-relative paths mentioned in the docs."""
    found: dict[str, set[str]] = {}
    for pattern in DOC_GLOBS:
        for path in REPO.glob(pattern):
            if not path.is_file():
                continue
            # Historical records (work logs, archived docs) cite files as they existed then;
            # re-checking them would only flag history.  Only live docs must resolve.
            rel = str(path.relative_to(REPO))
            # 2026-11-26：文献笔记里会引用"尚未产出的笔记""计划要写的脚本"，
            # 那是研究过程记录，不是失效链接；和历史日志一样不参与路径校验。
            if rel.startswith(("docs/worklogs/", "docs/archive/", "docs/literature/notes/")):
                continue
            for match in re.findall(r"`((?:docs|scripts|config|src|test)/[^`\s]+)`",
                                    path.read_text(errors="ignore")):
                # Glob patterns, placeholders and bare directory mentions are not concrete
                # files; checking them only produces noise (2026-11-07).
                if any(token in match for token in ("*", "{", "<", ">", "?", "...")):
                    continue
                if match.endswith("/"):
                    continue
                found.setdefault(match, set()).add(str(path.relative_to(REPO)))
    return found


def resolve_reference(ref: str) -> pathlib.Path | None:
    """Locate a cited path, tolerating the 2026-11-07 reorganisation.

    Historical work logs cite files by their pre-reorg location (docs/worklogs/<file>,
    docs/<file>).  Those citations must keep working without rewriting history, so a
    reference is resolved against:
      1. its literal path;
      2. docs/worklogs/<YYYY-MM>/<basename>   (work logs moved into month folders);
      3. docs/archive/**/<basename>          (outdated docs kept read-only);
      4. docs/reference/<basename>            (updated technical reference).
    """
    # 2026-11-26：文档里常用 `config/tasks/common.yaml:workpiece_views` 这种
    # "文件:键名" 的写法（指向 YAML 的某个键，不是文件）。这里把 :键名 也剥掉，
    # 否则每条这样的引用都会被误判成"路径不存在"（文献笔记里出现很多）。
    ref = re.sub(r":[A-Za-z_][\w.\-]*$", "", ref)
    ref = re.sub(r":\d+(-\d+)?$", "", ref)      # strip a trailing :line or :line-line
    ref = re.sub(r"::[A-Za-z_][\w.]*\(\)?$", "", ref)   # strip a trailing ::symbol reference
    literal = REPO / ref
    if literal.exists():
        return literal
    name = pathlib.PurePosixPath(ref).name
    if not name or "." not in name:
        return None
    parent = pathlib.PurePosixPath(ref).parent.name
    candidates: list[pathlib.Path] = []
    if "worklogs" in ref:
        candidates += sorted(REPO.glob(f"docs/worklogs/*/{name}"))
    # archives may nest by group (e.g. docs/archive/<stamp>/six-mode/<file>),
    # so resolve a basename anywhere inside the archive tree
    candidates += sorted(REPO.glob(f"docs/archive/**/{name}"))
    candidates += [REPO / "docs" / "reference" / name,
                   REPO / "docs" / "manuals" / name]
    if parent:
        candidates += sorted(REPO.glob(f"**/{parent}/{name}"))
    for candidate in candidates:
        if candidate.exists():
            return candidate
    return None


def check_paths(verbose: bool) -> None:
    referenced = collect_referenced_paths()
    missing = {ref: where for ref, where in referenced.items()
               if resolve_reference(ref) is None}
    print(f"referenced paths: {len(referenced)} checked, {len(missing)} missing")
    for ref, where in sorted(missing.items()):
        problem(f"referenced path does not exist: {ref}  (cited in {sorted(where)[:3]})")


def check_commits() -> None:
    hashes: dict[str, set[str]] = {}
    for pattern in DOC_GLOBS:
        for path in REPO.glob(pattern):
            if not path.is_file():
                continue
            rel = str(path.relative_to(REPO))
            if rel.startswith(("docs/worklogs/", "docs/archive/")):
                continue          # historical citations, checked only for live docs
            text = path.read_text(errors="ignore")
            for match in re.findall(
                    r"(?:commit|提交|Commit|COMMIT)[^\n]{0,24}?\b([0-9a-f]{7,40})\b", text):
                hashes.setdefault(match[:7], set()).add(str(path.relative_to(REPO)))
    if not hashes:
        return
    result = subprocess.run(["git", "cat-file", "--batch-check=%(objectname)"],
                            input="\n".join(hashes), cwd=REPO,
                            capture_output=True, text=True)
    lines = result.stdout.splitlines()
    unknown = [h for h, line in zip(hashes, lines) if "missing" in line]
    print(f"commit hashes: {len(hashes)} checked, {len(unknown)} unknown")
    for short in unknown:
        problem(f"commit hash not in this repository: {short} "
                f"(cited in {sorted(hashes[short])[:3]})")


def check_agent_memory() -> None:
    if not AGENT_MEMORY.exists():
        problem("docs/agent_memory.md missing")
        return
    text = AGENT_MEMORY.read_text()
    keys = re.findall(r"^- \*\*(.{2,40}?)\*\*", text, flags=re.MULTILINE)
    duplicates = sorted({k for k in keys if keys.count(k) > 1})
    print(f"agent_memory: {len(keys)} bold rules")
    if duplicates:
        problem(f"agent_memory duplicated rules: {duplicates[:5]}")


TODO_GROUPS = (
    "⚠️ 下一步就做",
    "一、", "二、", "三、", "四、", "五、", "六、", "七、", "八、", "九、",
)
# 追加式编号的典型形态：六b / 六k / 六l …（所有者 2026-11-26 明确禁止）
APPEND_STYLE_GROUP = re.compile(r"^##\s*[一二三四五六七八九]\s*[b-z]")


def check_todo_structure() -> None:
    """TODO 必须有固定分组结构，且禁止追加式编号（所有者 2026-11-26 定）。

    背景：TODO 曾膨胀到 230 行，编号顺序变成 一→六→六b→六l→六k→六j→六f…，
    读的人找不到"下一步做什么"。规则只写在 AGENTS 里不够，这里让提交时拦住。
    """
    path = REPO / "docs" / "TODO.md"
    if not path.is_file():
        problem("docs/TODO.md missing")
        return
    text = path.read_text(encoding="utf-8")
    headings = [ln.strip() for ln in text.splitlines() if ln.startswith("## ")]
    if not headings:
        problem("TODO has no section headings")
        return

    append_style = [h for h in headings if APPEND_STYLE_GROUP.match(h)]
    if append_style:
        problem(f"TODO uses append-style numbering (forbidden): {append_style[:3]}")

    # 必须包含"下一步就做"分组（否则又会出现"不知道先做什么"）
    if not any("下一步就做" in h for h in headings):
        problem("TODO is missing the '⚠️ 下一步就做' block at the top")

    # 分组顺序必须符合白名单顺序（允许缺组，但不允许乱序）
    order = {prefix: i for i, prefix in enumerate(TODO_GROUPS)}
    seen = []
    for h in headings:
        for prefix in TODO_GROUPS:
            if prefix in h:
                seen.append(order[prefix])
                break
    if seen != sorted(seen):
        problem(f"TODO groups are out of order: {[h for h in headings]}")


WORKLOG_SECTION = re.compile(r"^#{2,3}\s+\d{1,2}:\d{2}\b")
WORKLOG_BANNED = re.compile(r"^###\s*$")


def check_worklog_format() -> None:
    """工作日志：时间戳节必须按时间单调递增，且每节至少有"做了什么/结果"之一。

    背景（所有者 2026-11-26）：曾出现两个 14:10 重复节、以及一整节用完全不同的
    密排编号列表写，与其它节的统一结构冲突。这里只校验可机器判定的部分。
    """
    logs = sorted((REPO / "docs" / "worklogs").glob("*/*WORKLOG*.md"))
    for path in logs[-3:]:                     # 只查最近三份，避免历史日志误报
        text = path.read_text(encoding="utf-8")
        times: list[tuple[int, int]] = []
        for ln in text.splitlines():
            m = WORKLOG_SECTION.match(ln.strip())
            if m:
                stamp = re.search(r"(\d{1,2}):(\d{2})", ln)
                if stamp:
                    times.append((int(stamp.group(1)), int(stamp.group(2))))
        if len(times) < 2:
            continue
        if times != sorted(times):
            problem(f"{path.name}: time-stamped sections are not chronological")
        duplicates = {t for t in times if times.count(t) > 1}
        if duplicates:
            problem(f"{path.name}: duplicate section timestamps "
                    f"{['%02d:%02d' % t for t in sorted(duplicates)]}")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--verbose", action="store_true")
    args = parser.parse_args()

    print("== manual structure ==")
    check_manual()
    print("\n== referenced paths ==")
    check_paths(args.verbose)
    print("\n== commit hashes ==")
    check_commits()
    print("\n== agent memory ==")
    check_agent_memory()
    print("\n== TODO structure ==")
    check_todo_structure()
    print("\n== work-log format ==")
    check_worklog_format()

    print()
    if PROBLEMS:
        print(f"{len(PROBLEMS)} PROBLEM(S) FOUND")
        return 1
    print("ALL DOC CHECKS PASSED")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
