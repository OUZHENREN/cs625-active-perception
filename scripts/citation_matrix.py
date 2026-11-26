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
SECTIONS: dict[str, tuple[list[str], str, list[str]]] = {
 "3.1 手眼标定与残差评价": (["手眼", "AX=XB", "标定", "残差", "重投影"], "方法来源",
     [r"AX\s*=\s*XB", r"手眼标定", r"eye[\s-]?(in|to)[\s-]?hand", r"Tsai", r"对偶四元数"]),
 "3.2 粗-精分层位姿解算": (["三平面", "RANSAC", "ICP", "配准", "粗精"], "方法来源",
     [r"ICP", r"RANSAC", r"三平面", r"点云配准", r"配准"]),
 "3.3 可达性判定": (["可达", "工作空间", "IK", "逆解", "碰撞", "奇异"], "方法来源",
     [r"可达", r"reachab", r"工作空间", r"逆解", r"inverse kinematics", r"碰撞", r"collision", r"奇异"]),
 "3.4 受约束局部视角优化": (["主动视觉", "NBV", "视角", "观测位", "视场"], "方法来源",
     [r"视角", r"视点", r"viewpoint", r"view planning", r"NBV", r"next[- ]?best[- ]?view", r"主动视觉", r"观测位"]),
 "3.5 实验：局部优化对照": (["覆盖率", "重复性", "对照", "成功率"], "对比基线",
     [r"实验", r"实测", r"成功率", r"重复性", r"对照", r"对比"]),
 "4.1 误差源辨识": (["误差源", "误差模型", "MDH", "几何参数", "辨识"], "方法来源",
     [r"误差源", r"error source", r"误差模型", r"MDH", r"D-H", r"几何参数", r"辨识"]),
 "4.2 传播建模与角度误差杠杆放大": (["姿态误差", "角度", "间隙", "深度", "偏差"], "参数依据",
     [r"姿态误差", r"角度误差", r"间隙", r"clearance", r"偏差", r"杠杆"]),
 "4.3 蒙特卡洛与公差分配": (["蒙特卡洛", "公差", "精度分配", "灵敏度"], "参数依据",
     [r"蒙特卡洛", r"Monte Carlo", r"公差", r"tolerance", r"精度分配", r"灵敏度"]),
 "4.4 验证实验：杠杆臂（4 深度）": (["杠杆", "深度", "偏差", "实验"], "对比基线",
     [r"杠杆", r"插入深度", r"深度.*偏差", r"偏置.*实验"]),
 "4.5 测量系统 R&R": (["R&R", "GRR", "再现性", "NDC", "重复性"], "参数依据",
     [r"R\s*&\s*R", r"GRR", r"Gage", r"再现性", r"reproducib", r"NDC"]),
 "5.1 KWR75 通信与驱动": (["六维力", "RS485", "采样", "滤波"], "方法来源",
     [r"六维力", r"six[- ]?axis", r"采样", r"滤波", r"filter"]),
 "5.2 重力补偿与零漂标定": (["重力补偿", "零漂", "质心", "偏心"], "方法来源",
     [r"重力补偿", r"gravity compensation", r"零漂", r"零点", r"质心", r"偏心"]),
 "5.3 接触状态识别": (["接触状态", "状态机", "阈值", "卡阻"], "方法来源",
     [r"接触状态", r"contact state", r"状态机", r"卡阻", r"jamming", r"wedging", r"力阈值"]),
 "5.4 位置型导纳与视觉-力觉切换": (["导纳", "阻抗", "柔顺", "切换"], "方法来源",
     [r"导纳", r"admittance", r"阻抗", r"impedance", r"柔顺", r"compliance", r"切换"]),
 "5.5 导纳参数整定与残差强化学习": (["强化学习", "残差", "奖励", "策略", "阻尼"], "方法来源",
     [r"强化学习", r"reinforcement learning", r"残差", r"residual", r"变阻抗", r"variable impedance", r"变阻尼", r"奖励"]),
 "5.6 实验：三档对照": (["对照", "消融", "成功率", "试次"], "对比基线",
     [r"对照", r"对比", r"消融", r"成功率", r"success rate", r"试次", r"trials"]),
 "6.1–6.3 结论与展望（可靠度/对比证据）": (["可靠度", "置信", "成功率", "对比"], "对比基线",
     [r"可靠度", r"reliability", r"置信", r"confidence", r"成功率", r"对比"]),
}


# 人工复核覆盖（评审 2026-11-26 逐节核对）：自动词频在这几节被笔记自身模板污染，
# 这里的 key 是人工核对过"论文自己确实做了这件事"的。新文献进来后需再人工复核一次。
MANUAL_OVERRIDE: dict[str, list[str]] = {
    "3.1 手眼标定与残差评价": ["RGBUCYKU", "8G2H64GR", "T8IJCCH3"],   # AX=XB（MLE/Tsai-Lenz）真做的
    "3.4 受约束局部视角优化": ["4USD9WCL", "J85Y8BDM", "GX9BMX7X"],   # 主动探索/两级视场/换视点（弱）；待 lit-en 的 94THW73P 补强
    "4.2 传播建模与角度误差杠杆放大": ["8G2H64GR", "T9FQSZ7I", "RE3RSLML"],
    "4.4 验证实验：杠杆臂（4 深度）": ["8G2H64GR", "T9FQSZ7I", "P7CZ6JWL"],
    "6.1–6.3 结论与展望（可靠度/对比证据）": ["RLHBFRBA", "U8EW6HDC", "5UFZI35T"],
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
        # 论文自身内容 = 标题 + §1..§7；§8 是我们路线的冲突点（会提到手眼标定/站位表等我们的词，
        # 造成词面碰撞），§9 是待确认清单 —— 两者都不参与匹配（评审指出的根因）。
        own_parts = [title]
        for sec in ("1.", "2.", "3.", "4.", "5.", "6.", "7."):
            own_parts.append(section_body(sec))
        own_text = "\n".join(own_parts)
        out.append({"key": key, "title": title, "rel": rel, "evidence": evidence,
                    "chapters": chapters,
                    "file": path.name, "body4": section_body("4."), "body7": section_body("7."),
                    "text": text, "own_text": own_text})
    return out


NEGATION = re.compile(r"(没有|未|无|不采用|不是|缺失|不存在|回避|跳过|空白)")


def _positive_hits(text: str, pat: str) -> int:
    """命中但所在句是否定句（"本文没有手眼标定"）的不算——评审指出的词面碰撞。"""
    count = 0
    for sentence in re.split(r"[。；\n]|(?<=[a-z])\. ", text):
        for m in re.finditer(pat, sentence, re.IGNORECASE):
            head = sentence[: m.start()]
            if not NEGATION.search(head[-25:]):
                count += 1
    return count


def qualifies(note: dict, required: list[str]) -> bool:
    """硬证据过滤：论文自身内容（§1–§7）必须正向命中至少一个证据正则。"""
    text = note["own_text"]
    return any(_positive_hits(text, pat) > 0 for pat in required)


def score(note: dict, keywords: list[str]) -> int:
    total = 0
    own = note["own_text"]
    for kw in keywords:
        if kw in note["title"]:
            total += 3
        if kw in note["body4"] or kw in note["body7"]:
            total += 2
        total += kw.lower() in own.lower()
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
    for section, (keywords, kind, required) in SECTIONS.items():
        manual = False
        if section in MANUAL_OVERRIDE:
            by_key = {n["key"]: n for n in notes}
            ranked = [(99 - i, by_key[k]) for i, k in enumerate(MANUAL_OVERRIDE[section]) if k in by_key]
            manual = True
        else:
            eligible = [n for n in notes if qualifies(n, required)]
            ranked = sorted(((score(n, keywords), n) for n in eligible), key=lambda x: -x[0])
            ranked = [(s, n) for s, n in ranked if s > 0][: max(args.min, 3)]
        out.append(f"## {section}")
        out.append(f"**需要的支撑类型**：{kind}" + ("（候选为人工复核，见 `scripts/citation_matrix.py` 的 MANUAL_OVERRIDE）" if manual else ""))
        out.append("")
        if len(ranked) < args.min:
            gaps += 1
            out.append(f"⚠ **候选只有 {len(ranked)} 篇（要求 ≥{args.min}）** —— 证据正则：`{required}`；需补检索或补精读")
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
