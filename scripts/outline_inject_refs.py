#!/usr/bin/env python3
"""把引用矩阵注入论文纲要：让每一节在纲要里直接看到 ≥3 篇文献 + 该文献章节。

所有者 2026-11-26 的要求："每个部分都能给出至少三篇能参考的文献和对应的章节，这样我心里才有底。"

做法
----
1. 读 `docs/literature/CITATION_MATRIX.md`，解析出 "## <节名>" 下的表格行（Zotero key + 章节结构）；
2. 在 `docs/plans/2026-11-26_THESIS_OUTLINE_V2.md` 末尾维护一个
   `## 附录 A：每节的文献支撑（自动生成）` 区块（**重复运行会整体替换该区块**）；
3. 每个小节输出一行：`- **<节名>**：`KEY1`、`KEY2`、`KEY3` —— 详见 CITATION_MATRIX.md。

用法：
    python3 scripts/outline_inject_refs.py
退出码：0 成功；2 输入缺失。
"""

from __future__ import annotations

import pathlib
import re
import sys

sys.stdout.reconfigure(line_buffering=True)
REPO = pathlib.Path(__file__).resolve().parents[1]
MATRIX = REPO / "docs" / "literature" / "CITATION_MATRIX.md"
OUTLINE = REPO / "docs" / "plans" / "2026-11-26_THESIS_OUTLINE_V2.md"
MARK = "## 附录 A：每节的文献支撑（自动生成）"


def parse_matrix() -> dict[str, list[str]]:
    text = MATRIX.read_text(encoding="utf-8")
    sections: dict[str, list[str]] = {}
    current = None
    for line in text.splitlines():
        head = re.match(r"^## (.+)$", line)
        if head:
            current = head.group(1).strip()
            sections[current] = []
            continue
        if current and line.startswith("| ") and not line.startswith("| #"):
            cells = [c.strip() for c in line.strip("|").split("|")]
            if len(cells) >= 2 and cells[1].startswith("`"):
                key = cells[1].strip("`")
                if re.fullmatch(r"[A-Z0-9]{8}", key) and len(sections[current]) < 3:
                    sections[current].append(key)
    return sections


def main() -> int:
    for path in (MATRIX, OUTLINE):
        if not path.exists():
            print(f"缺少输入：{path}")
            return 2
    sections = parse_matrix()
    block = [MARK, "",
             "> 由 `scripts/outline_inject_refs.py` 从 `docs/literature/CITATION_MATRIX.md` 生成；",
             "> 每条给出该节最相关的 3 篇精读文献（Zotero itemKey），对应章节与页码证据见矩阵。", ""]
    empty = 0
    for name, keys in sections.items():
        if not keys:
            block.append(f"- **{name}**：⚠ 暂无候选（需补检索）")
            empty += 1
        else:
            block.append(f"- **{name}**：" + "、".join(f"`{k}`" for k in keys))
    text = OUTLINE.read_text(encoding="utf-8")
    if MARK in text:
        text = text[: text.index(MARK)].rstrip() + "\n\n"
    text = text.rstrip() + "\n\n" + "\n".join(block) + "\n"
    OUTLINE.write_text(text, encoding="utf-8")
    print(f"已注入 {len(sections)} 个小节的文献支撑（空缺 {empty} 个）→ {OUTLINE.relative_to(REPO)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
