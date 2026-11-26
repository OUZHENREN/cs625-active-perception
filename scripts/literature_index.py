#!/usr/bin/env python3
"""把 docs/literature/notes/ 下的精读笔记汇总成索引（Markdown）。

用途：一眼看到"读了哪些论文、相关性如何、各自一句话结论"，并可直接复制到
Obsidian `04-课题与项目/04_文献与研究问题`。

用法
----
    python3 scripts/literature_index.py                 # 生成/更新 docs/literature/NOTES_INDEX.md
    python3 scripts/literature_index.py --to-obsidian   # 同时复制到 Obsidian 文献目录

笔记约定（由 docs/literature/templates/PAPER_NOTE_TEMPLATE.md 规定）：
    文件名 `<itemKey>_<短标题>.md`
    第 1 节含 "Zotero itemKey"、第 2 节含 "相关性分类"、第 2 节含 "一句话"
退出码：0 正常；2 笔记目录不存在。
"""

from __future__ import annotations

import argparse
import pathlib
import re
import shutil
import sys
import time

sys.stdout.reconfigure(line_buffering=True)

REPO = pathlib.Path(__file__).resolve().parents[1]
NOTES = REPO / "docs" / "literature" / "notes"
INDEX = REPO / "docs" / "literature" / "NOTES_INDEX.md"
VAULT = pathlib.Path("/mnt/g/我的云端硬盘/Obsidian_Vault/OY/Do be do be do/04-课题与项目/04_文献与研究问题")


def field(text: str, pattern: str) -> str:
    found = re.search(pattern, text, re.MULTILINE)
    return found.group(1).strip() if found else ""


REL_ORDER = ("直接对象证据 + 方法迁移", "直接对象证据", "方法迁移", "工程工具", "暂不相关", "未标注")


def extract_year(text: str) -> str:
    head = "\n".join(text.splitlines()[:15])
    m = re.search(r"(?:年份|发表|日期|答辩|提交)[：:\s\*]*((?:19|20)\d{2})", head)
    if m:
        return m.group(1)
    top = "\n".join(
        ln for ln in text.splitlines()[1:12]
        if not any(w in ln for w in ("阅读", "2026-11", "无任何年份证据"))
    )
    # 只剔除"1995—"这类出生年份标记，避免误删"2023-03"等带连字符的答辩/发表年月
    top_clean = re.sub(r"(?:19|20)\d{2}\s*[—–](?!\d)", "", top)
    m2 = re.search(r"\b((?:19|20)\d{2})\b", top_clean)
    return m2.group(1) if m2 else ""


def extract_relevance(text: str) -> str:
    m = re.search(r"相关性[^\n：:]*[：:]\s*\**(.+?)(?:\*\*|$|[（(—。，])", text, re.MULTILINE)
    if not m:
        return "未标注"
    raw = re.sub(r"\s+", " ", m.group(1)).strip()
    for cat in REL_ORDER[:-1]:
        if raw.startswith(cat) or cat in raw:
            return cat
    return raw or "未标注"


def summarize(path: pathlib.Path) -> dict | None:
    text = path.read_text(encoding="utf-8", errors="replace")
    key = path.name.split("_")[0]
    if not re.fullmatch(r"[A-Z0-9]{8}", key):
        return None
    return {
        "key": key,
        "file": path.name,
        "title": field(text, r"^#\s+(.+)$") or field(text, r"\*\*标题\*\*[：:]\s*(.+)"),
        "kind": field(text, r"(硕士学位论文|博士学位论文|期刊|会议|journal|conference)"),
        "year": extract_year(text),
        "relevance": extract_relevance(text),
        "conclusion": field(text, r"一句话[^\n：:]*[：:]\s*(.+)"),
        "lines": len(text.splitlines()),
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--to-obsidian", action="store_true")
    args = parser.parse_args()
    if not NOTES.is_dir():
        print(f"笔记目录不存在：{NOTES}")
        return 2
    rows = [s for p in sorted(NOTES.glob("*.md")) if (s := summarize(p))]
    if not rows:
        print("还没有笔记（docs/literature/notes/*.md）")
        return 2
    by_rel: dict[str, list[dict]] = {}
    for row in rows:
        by_rel.setdefault(row["relevance"] or "未标注", []).append(row)
    out = ["# 文献精读索引（自动生成）", "",
           f"生成时间：{time.strftime('%Y-%m-%d %H:%M')}｜共 **{len(rows)}** 篇",
           "", "> 由 `scripts/literature_index.py` 从 `docs/literature/notes/` 汇总；"
               "笔记格式见 `docs/literature/templates/PAPER_NOTE_TEMPLATE.md`。", ""]
    rel_rank = {cat: idx for idx, cat in enumerate(REL_ORDER)}
    for rel in sorted(by_rel, key=lambda r: (rel_rank.get(r, 98), r)):
        out += [f"## {rel}（{len(by_rel[rel])} 篇）", "",
                "| Zotero | 年份 | 类型 | 标题 | 一句话结论 | 笔记 |",
                "|---|---|---|---|---|---|"]
        for row in sorted(by_rel[rel], key=lambda r: r["year"] or "0"):
            title = (row["title"] or "").replace("|", "/")[:44]
            concl = (row["conclusion"] or "").replace("|", "/")[:60]
            out.append(f"| `{row['key']}` | {row['year']} | {row['kind'][:4]} | {title} "
                       f"| {concl} | [{row['file']}](notes/{row['file']}) |")
        out.append("")
    INDEX.write_text("\n".join(out) + "\n", encoding="utf-8")
    print(f"索引已生成：{INDEX.relative_to(REPO)}（{len(rows)} 篇）")
    if args.to_obsidian:
        if not VAULT.is_dir():
            print(f"Obsidian 目录不存在，跳过复制：{VAULT}")
            return 0
        # G 盘挂载不支持 chmod/utime（9p），只能做纯内容复制。
        def _content_copy(src: pathlib.Path, dst: pathlib.Path) -> None:
            dst.write_bytes(src.read_bytes())
        _content_copy(INDEX, VAULT / INDEX.name)
        for path in NOTES.glob("*.md"):
            _content_copy(path, VAULT / path.name)
        print(f"已复制到 {VAULT}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
