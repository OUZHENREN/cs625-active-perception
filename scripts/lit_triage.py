#!/usr/bin/env python3
"""T2 通读登记：把库里剩下的文献做**机器化**登记（不占子智能体、不读全文）。

目的（所有者 2026-11-26 批准的三层口径）：
  T1 精读 72 篇（子智能体逐篇读正文，出 9 节笔记）
  T2 通读登记 **其余全部**（本脚本）——每篇 1 条记录：元数据 + 摘要 + 四条线归类
     + 可引用点候选 + 是否值得升 T1
  T3 按需升精读（被正文引用到的 T2 文献）

数据来源：Zotero 库（只读副本），不碰原库。
输出：
  runtime/lit_triage/all.json     全量登记（含 itemKey/标题/年/类型/摘要摘要/归类/理由）
  runtime/lit_triage/summary.md   统计：各线数量、有/无摘要、T1 候选列表

用法：
    python3 scripts/lit_triage.py --db "/mnt/g/我的云端硬盘/Zotero/zotero.sqlite"
    python3 scripts/lit_triage.py --db ... --top 60      # 只列最值得升 T1 的 60 篇
退出码：0 成功；2 参数/文件问题。
"""

from __future__ import annotations

import argparse
import json
import pathlib
import re
import shutil
import sqlite3
import sys

sys.stdout.reconfigure(line_buffering=True)
REPO = pathlib.Path(__file__).resolve().parents[1]
OUT = REPO / "runtime" / "lit_triage"

LINES = {
    "A_装校装备与结构": r"装校|抓取工装|机构设计|转运|洁净|对接|拆装|机械手|装校机构|平台设计",
    "B_视觉测量与标定": r"视觉|标定|位姿测量|点云|配准|6D|相机|手眼|测量装置|视觉伺服|图像",
    "C_力觉与柔顺": r"六维力|力觉|柔顺|阻抗|导纳|力控|卡阻|轴孔|力反馈|主动柔顺|接触力",
    "D_误差与可靠规划": r"误差|可靠|精度|公差|可达|运动规划|轨迹|蒙特卡洛|灵敏度|补偿",
}
# 与课题强相关的加分词（用于挑"值得升 T1"的）
BOOST = r"光学模块|光机|装校|六维力|柔顺装配|轴孔|间隙|手眼标定|位姿测量|误差分配|可靠度|插入"


def field(conn, iid: int, name: str) -> str:
    row = conn.execute("""SELECT idv.value FROM itemData id
                          JOIN itemDataValues idv ON id.valueID=idv.valueID
                          JOIN fields f ON id.fieldID=f.fieldID
                          WHERE id.itemID=? AND f.fieldName=?""", (iid, name)).fetchone()
    return row[0] if row else ""


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--db", required=True)
    parser.add_argument("--top", type=int, default=80, help="列出最值得升 T1 的条数")
    args = parser.parse_args()
    db = pathlib.Path(args.db)
    if not db.is_file():
        print(f"找不到 Zotero 库：{db}")
        return 2
    copy = pathlib.Path("/tmp/zotero_triage.sqlite")
    shutil.copy2(db, copy)
    conn = sqlite3.connect(copy)

    # 已精读（T1）的 itemKey，避免重复登记
    t1 = {p.name.split("_")[0] for p in (REPO / "docs" / "literature" / "notes").glob("*.md")}

    rows = []
    for iid, typ in conn.execute("""SELECT i.itemID, it.typeName FROM items i
                                    JOIN itemTypes it ON i.itemTypeID=it.itemTypeID
                                    WHERE it.typeName IN ('thesis','journalArticle','conferencePaper')"""):
        key = (conn.execute("SELECT key FROM items WHERE itemID=?", (iid,)).fetchone() or [""])[0]
        if not key or key in t1:
            continue
        title = field(conn, iid, "title").strip()
        if not title:
            continue
        abstract = field(conn, iid, "abstractNote")
        year = field(conn, iid, "date")[:4]
        kind = field(conn, iid, "thesisType") or typ
        hits = {name: len(re.findall(pat, title + " " + abstract)) for name, pat in LINES.items()}
        best = max(hits, key=hits.get)
        line = best if hits[best] else "未归类"
        boost = len(re.findall(BOOST, title + " " + abstract))
        # 可引用点候选：从摘要里抽第一句含数字的论断
        quote = ""
        for sentence in re.split(r"[。；;]", abstract):
            if re.search(r"\d", sentence) and len(sentence) > 12:
                quote = sentence.strip()[:90]
                break
        rows.append({"itemKey": key, "title": title, "year": year, "kind": kind,
                     "line": line, "hits": hits, "boost": boost,
                     "has_abstract": len(abstract) > 80,
                     "abstract_quote": quote})
    rows.sort(key=lambda r: (-r["boost"], -(1 if r["has_abstract"] else 0), r["year"]))
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / "all.json").write_text(json.dumps(rows, ensure_ascii=False, indent=1), encoding="utf-8")

    by_line: dict[str, int] = {}
    for r in rows:
        by_line[r["line"]] = by_line.get(r["line"], 0) + 1
    no_abs = sum(1 for r in rows if not r["has_abstract"])
    summary = ["# T2 通读登记（机器化）", "",
               f"登记条目：**{len(rows)}**（已排除 T1 精读的 {len(t1)} 篇）",
               f"有摘要：{len(rows) - no_abs}｜无摘要（需读 PDF）：{no_abs}", "",
               "## 按四条线", ""]
    for name, count in sorted(by_line.items(), key=lambda kv: -kv[1]):
        summary.append(f"- {name}: **{count}**")
    summary += ["", f"## 最值得升 T1 的 {min(args.top, len(rows))} 篇", "",
                "| itemKey | 年 | 类型 | 线 | 标题 |", "|---|---|---|---|---|"]
    for r in rows[: args.top]:
        summary.append(f"| `{r['itemKey']}` | {r['year']} | {r['kind'][:4]} | {r['line'][:6]} "
                       f"| {r['title'][:46]} |")
    (OUT / "summary.md").write_text("\n".join(summary) + "\n", encoding="utf-8")
    print(f"登记 {len(rows)} 条 → runtime/lit_triage/all.json + summary.md")
    print("各线：", {k: v for k, v in sorted(by_line.items(), key=lambda kv: -kv[1])})
    print(f"无摘要 {no_abs} 条（这些只能读 PDF 才能判断，已列入待读候选）")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
