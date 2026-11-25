#!/usr/bin/env python3
"""生成「引用矩阵」：论文设计的每一节 → ≥3 篇可引文献（带该文献的章节/页码证据）。

所有者 2026-11-26 要求：每个部分至少 3 篇能参考的文献 + 对应章节，"这样心里才有底"。

做法
----
1. 章节口径来自 `docs/plans/2026-11-26_THESIS_OUTLINE_V2.md` 的 6 章结构（本脚本内维护）：
   每节给一组关键词（中英）与"需要哪类支撑"（方法来源/参数依据/对比基线/反例与边界）。
2. 对 `docs/literature/notes/*.md`（T1 精读笔记）做关键词打分：
   命中标题权重 3、命中 §4/§7 权重 2、命中其它权重 1；
   并从笔记里摘出**页码证据行**（含 `p.` 或"第 X 章"的行）作为可核查引用点。
3. 输出 `docs/literature/CITATION_MATRIX.md`：
   每节列前 3 名候选；不足 3 篇的写 `⚠ 库内不足`，并给出建议补检索式。
4. 同时建议一批"值得升 T1 的 T2 文献"（当某节候选不足时，从 runtime/lit_triage/all.json 里按关键词找）。

用法：
    python3 scripts/citation_matrix.py            # 生成/更新矩阵
    python3 scripts/citation_matrix.py --min 3    # 每节最少条数（默认 3）
退出码：0 成功；2 笔记目录为空。
"""

from __future__ import annotations

import argparse
import json
import pathlib
import re
import sys

sys.stdout.reconfigure(level=__import__("logging").INFO) if False else None
sys.stdout.reconfigure(line_buffering=True)
REPO = pathlib.Path(__file__).resolve().parents[1]
NOTES = REPO / "docs" / "literature" / "notes"
TRIAGE = REPO / "runtime" / "lit_triage" / "all.json"
OUT = REPO / "docs" / "literature" / "CITATION_MATRIX.md"

# 章节 → (关键词, 需要哪类支撑)
SECTIONS: dict[str, tuple[list[str], str]] = {
 "3.1 手眼标定与残差评价": (["手眼", "AX=XB", "标定", "重投影", "残差"], "方法来源"),
 "3.2 粗-精分层位姿解算": (["三平面", "RANSAC", "ICP", "配准", "粗精", "位姿估计"], "方法来源"),
 "3.3 可达性判定": (["可达", "工作空间", "IK", "逆解", "碰撞", "奇异"], "方法来源"),
 "3.4 受约束局部视角优化": (["主动视觉", "NBV", "视角", "观测位", "视场", "相机位姿"], "方法来源"),
 "3.5 测量精度实验与对照": (["重复性", "精度", "误差", "真值", "样本"], "对比基线"),
 "4.1 误差源辨识": (["误差源", "误差模型", "MDH", "D-H", "几何参数", "辨识"], "方法来源"),
 "4.2 角度误差的杠杆放大": (["角度", "姿态误差", "间隙", "插入深度", "偏差"], "参数依据"),
 "4.3 蒙特卡洛与公差分配": (["蒙特卡洛", "公差", "精度分配", "灵敏度", "可靠度", "分布"], "参数依据"),
 "4.4 测量系统 R&R": (["重复性", "再现性", "R&R", "GRR", "方差"], "参数依据"),
 "5.1 六维力接入与重力补偿": (["六维力", "重力补偿", "零漂", "质心", "标定", "滤波"], "方法来源"),
 "5.2 接触状态识别": (["接触状态", "状态机", "阈值", "力特征", "卡阻"], "方法来源"),
 "5.3 位置型导纳控制": (["导纳", "阻抗", "柔顺", "M/B/K", "阻尼", "刚度"], "方法来源"),
 "5.4 视觉-力觉切换": (["视觉失效", "力觉接管", "切换", "盲区", "遮挡"], "反例与边界"),
 "6.1 全链实验与失败模式": (["成功率", "失败", "试次", "帕累托", "重复"], "对比基线"),
 "6.2 与文献方法对比": (["对比", "基线", "优于", "传统"], "对比基线"),
 "6.3 可靠度评估": (["可靠度", "置信", "分布", "K-S", "3σ"], "参数依据"),
}


def load_notes() -> list[dict]:
    out = []
    for path in sorted(NOTES.glob("*.md")):
        key = path.name.split("_")[0]
        if not re.fullmatch(r"[A-Z0-9]{8}", key):
            continue
        text = path.read_text(encoding="utf-8", errors="replace")
        title = ""
        head = re.search(r"^#\s+(.+)$", text, re.MULTILINE)
        if head:
            title = head.group(1).strip()
        rel = ""
        rel_match = re.search(r"相关性[^\n：:]*[：:]\s*\**([^\n*]+)", text)
        if rel_match:
            rel = rel_match.group(1).strip()
        # ① 该文献自己的章节结构（笔记 §1 里的"章节："行）
        chapters = ""
        chapter_match = re.search(r"章节[：:]\s*(.+)", text)
        if chapter_match:
            chapters = chapter_match.group(1).strip()[:110]
        # ② 可核查论断：同一行里既有页码又有数字，且不是元数据/阅读状态行
        skip = ("阅读状态", "PDF：", "类型：", "页码", "章节：", "元数据", "Zotero itemKey",
                "未读", "只读到", "仅目录", "未定位")
        claims = []
        for ln in text.splitlines():
            s = ln.strip().lstrip("-").strip()
            if not s or any(s.startswith(k) or k in s[:12] for k in skip):
                continue
            if re.search(r"p\.\s?\d+", s) and re.search(r"\d", s) and len(s) > 24:
                claims.append(re.sub(r"\s+", " ", s)[:120])
        evidence = claims[:3]
        lines = text.splitlines()
        # §4 与 §7 的正文（用于加权）
        def section_body(name: str) -> str:
            start = next((i for i, ln in enumerate(lines) if ln.strip().startswith(f"## {name}")), None)
            if start is None:
                return ""
            end = next((i for i in range(start + 1, len(lines)) if lines[i].strip().startswith("## ")), len(lines))
            return "\n".join(lines[start:end])
        out.append({"key": key, "title": title, "rel": rel, "evidence": evidence,
                    "chapters": chapters,
                    "file": path.name, "body4": section_body("4."), "body7": section_body("7."),
                    "text": text})
    return out


def score(note: dict, keywords: list[str]) -> int:
    total = 0
    for kw in keywords:
        if kw in note["title"]:
            total += 3
        if kw in note["body4"] or kw in note["body7"]:
            total += 2
        total += note["text"].count(kw) > 0
    return total


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--min", type=int, default=3)
    args = parser.parse_args()
    notes = load_notes()
    if not notes:
        print(f"{NOTES} 下没有笔记")
        return 2
    triage = []
    if TRIAGE.exists():
        triage = json.loads(TRIAGE.read_text(encoding="utf-8"))

    out = ["# 引用矩阵：每一节 → 可引文献（含章节/页码证据）", "",
           f"由 `scripts/citation_matrix.py` 从 `docs/literature/notes/` 的 **{len(notes)} 篇精读笔记**自动生成。",
           "每节列前 3 名候选；`⚠` 表示候选不足 3 篇（需补检索或升 T1）。", ""]
    gaps = 0
    for section, (keywords, kind) in SECTIONS.items():
        ranked = sorted(((score(n, keywords), n) for n in notes), key=lambda x: -x[0])
        ranked = [(s, n) for s, n in ranked if s > 0][: max(args.min, 3)]
        out.append(f"## {section}")
        out.append(f"**需要的支撑类型**：{kind}")
        out.append("")
        if len(ranked) < args.min:
            gaps += 1
            out.append(f"⚠ **候选只有 {len(ranked)} 篇（要求 ≥{args.min}）** —— 需补检索："
                       f"`{' OR '.join(keywords[:3])}`")
            # 从 T2 里找候选
            sugg = [t for t in triage
                    if any(k in t["title"] for k in keywords)][:5]
            if sugg:
                out.append("")
                out.append("T2 中可能的候选（需升 T1 精读后才能引用）：")
                for s in sugg:
                    out.append(f"- `{s['itemKey']}` {s['year']} {s['title'][:52]}")
        out.append("")
        out.append("| # | Zotero | 年份 | 标题 | 该文献的章节结构 | 可核查的论断（带页码） | 笔记 |")
        out.append("|---|---|---|---|---|---|---|")
        for i, (sc, n) in enumerate(ranked, 1):
            ev = "<br>".join(e.replace("|", "/") for e in n["evidence"][:2]) or "（见笔记 §4/§5）"
            year = (re.search(r"(20\d{2})", n["text"]) or [""])[0]
            chap = (n.get("chapters") or "（见笔记 §1）").replace("|", "/")
            out.append(f"| {i} | `{n['key']}` | {year} | {n['title'][:34]} | {chap} | {ev} | "
                       f"[{n['file']}](notes/{n['file']}) |")
        out.append("")
    OUT.write_text("\n".join(out) + "\n", encoding="utf-8")
    print(f"引用矩阵已生成：{OUT.relative_to(REPO)}")
    print(f"  章节 {len(SECTIONS)} 个｜笔记 {len(notes)} 篇｜候选不足的章节 {gaps} 个")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
