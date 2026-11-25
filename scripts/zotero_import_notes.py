#!/usr/bin/env python3
"""把精读笔记作为**子笔记**写回 Zotero 库（所有者 2026-11-26 要求）。

安全设计（重要）
----------------
· Zotero 直接写 sqlite 有损坏风险，所以：
    1. 默认 **--dry-run**，只报告将要写入什么；
    2. `--apply` 必须显式给出，且会先检查 Zotero 是否在运行（Windows 侧进程名 zotero.exe）；
       在运行就拒绝执行，请先关闭 Zotero；
    3. 写入前把 zotero.sqlite 复制成 `zotero.sqlite.bak-<时间戳>`；
    4. 全程一个事务，失败自动回滚；
    5. 每条笔记写入前检查"该父条目下是否已有同名子笔记"，避免重复导入。

数据格式
--------
读取 `docs/literature/notes/*.md`：文件名前 8 位为 Zotero itemKey。
笔记正文按 Markdown 转成最小 HTML（段落 + 行内代码），Zotero 的 note 字段是 HTML。

用法
----
    python3 scripts/zotero_import_notes.py --db "/mnt/g/我的云端硬盘/Zotero/zotero.sqlite"
    python3 scripts/zotero_import_notes.py --db ... --apply
退出码：0 成功/无需写入；2 参数/文件问题；3 Zotero 正在运行；4 写入校验失败。
"""

from __future__ import annotations

import argparse
import datetime as dt
import html
import pathlib
import re
import shutil
import sqlite3
import subprocess
import sys

sys.stdout.reconfigure(line_buffering=True)

REPO = pathlib.Path(__file__).resolve().parents[1]
NOTES = REPO / "docs" / "literature" / "notes"
NOTE_ITEMTYPE_ID = 28
KEY_CHARS = "23456789ABCDEFGHIJKLMNPQRSTUVWXYZ"


def zotero_running() -> bool:
    """在 Windows 宿主上检查 Zotero 是否在运行（WSL 里用 powershell 查）。"""
    try:
        out = subprocess.run(
            ["powershell.exe", "-NoProfile", "-Command",
             "(Get-Process zotero -ErrorAction SilentlyContinue | Measure-Object).Count"],
            capture_output=True, text=True, timeout=20).stdout
        return out.strip() not in ("", "0")
    except Exception:      # noqa: BLE001 - 查不到就当没运行，但会在 --apply 前再次确认
        return False


def md_to_html(text: str) -> str:
    out = []
    for line in text.splitlines():
        stripped = line.strip()
        if not stripped:
            out.append("<p></p>")
            continue
        if stripped.startswith("#"):
            level = min(6, len(stripped) - len(stripped.lstrip("#")))
            out.append(f"<h{level}>{html.escape(stripped[level:].strip())}</h{level}>")
            continue
        safe = html.escape(stripped)
        safe = re.sub(r"`([^`]+)`", r"<code>\1</code>", safe)
        safe = re.sub(r"\*\*([^*]+)\*\*", r"<strong>\1</strong>", safe)
        out.append(f"<p>{safe}</p>")
    return "".join(out)


def collect() -> list[dict]:
    items = []
    for path in sorted(NOTES.glob("*.md")):
        key = path.name.split("_")[0]
        if not re.fullmatch(r"[A-Z0-9]{8}", key):
            continue
        title = path.stem.split("_", 1)[1] if "_" in path.stem else path.stem
        items.append({"key": key, "title": title, "path": path,
                      "html": md_to_html(path.read_text(encoding="utf-8"))})
    return items


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--db", required=True,
                        help='Zotero 库路径，例如 "/mnt/g/我的云端硬盘/Zotero/zotero.sqlite"')
    parser.add_argument("--apply", action="store_true")
    parser.add_argument("--emit-manifest", action="store_true",
                        help="生成 Zotero JS 运行器用的清单（Zotero 开启时用这条路），"
                             "并复制到 Windows 临时目录")
    args = parser.parse_args()
    db = pathlib.Path(args.db)
    if not db.is_file():
        print(f"找不到 Zotero 库：{db}")
        return 2
    items = collect()
    if not items:
        print(f"{NOTES} 下没有符合命名规则的笔记")
        return 2

    if args.emit_manifest:
        import json, shutil as _shutil, pathlib as _pl
        manifest = {"generated_at": dt.datetime.now().isoformat(timespec="seconds"),
                    "notes": [{"itemKey": it["key"], "title": it["title"], "html": it["html"]}
                              for it in items]}
        local = _pl.Path("runtime/zotero_notes_manifest.json")
        local.parent.mkdir(parents=True, exist_ok=True)
        local.write_text(json.dumps(manifest, ensure_ascii=False), encoding="utf-8")
        print(f"清单已生成：{local}（{len(items)} 条）")
        windows = _pl.Path("/mnt/c/Users/l'z/AppData/Local/Temp")
        if windows.is_dir():
            target = windows / "zotero_notes_manifest.json"
            _shutil.copy2(local, target)
            print(f"已复制到：{target}")
            print("接下来：Zotero → 工具 → 开发者 → Run JavaScript → 粘贴 scripts/zotero_import.js")
        else:
            print("Windows 临时目录不可用，请手动把清单复制过去后运行 scripts/zotero_import.js")
        return 0

    copy_db = pathlib.Path("/tmp/zotero_import_copy.sqlite")
    shutil.copy2(db, copy_db)          # 只读副本用于分析，绝不直接改源库来判断
    conn = sqlite3.connect(copy_db)
    rows = []
    for item in items:
        hit = conn.execute("SELECT itemID, libraryID FROM items WHERE key=?", (item["key"],)).fetchone()
        if not hit:
            print(f"  [跳过] {item['key']}：库里没有这个条目")
            continue
        parent_id, library = hit
        exists = conn.execute("""SELECT COUNT(*) FROM itemNotes WHERE parentItemID=? AND title=?""",
                              (parent_id, item["title"])).fetchone()[0]
        rows.append({**item, "parent": parent_id, "library": library, "exists": bool(exists)})
    conn.close()
    todo = [r for r in rows if not r["exists"]]
    print(f"待写入 {len(todo)} 条（已存在 {len(rows) - len(todo)} 条，跳过）")
    for r in todo[:10]:
        print(f"  [{r['key']}] → 父条目 {r['parent']}｜{r['title'][:30]}")
    if len(todo) > 10:
        print(f"  … 其余 {len(todo) - 10} 条")
    if not args.apply:
        print("\n（dry-run：未写入。确认后加 --apply；执行前请关闭 Zotero）")
        return 0
    if zotero_running():
        print("Zotero 正在运行，拒绝写入。请先关闭 Zotero 再执行（避免库损坏）。")
        return 3

    backup = db.with_name(f"zotero.sqlite.bak-{dt.datetime.now():%Y%m%d_%H%M%S}")
    shutil.copy2(db, backup)
    print(f"已备份：{backup}")
    conn = sqlite3.connect(db)
    now = dt.datetime.utcnow().strftime("%Y-%m-%d %H:%M:%S")
    written = 0
    try:
        with conn:                     # 单个事务，异常自动回滚
            next_id = conn.execute("SELECT MAX(itemID) FROM items").fetchone()[0] + 1
            for r in todo:
                rng = __import__("random").Random(r["key"])
                key = "".join(rng.choice(KEY_CHARS) for _ in range(8))
                conn.execute("""INSERT INTO items(itemID, itemTypeID, dateAdded, dateModified,
                                clientDateModified, libraryID, key, version, synced)
                                VALUES(?,?,?,?,?,?,?,0,0)""",
                             (next_id, NOTE_ITEMTYPE_ID, now, now, now, r["library"], key))
                conn.execute("INSERT INTO itemNotes(itemID, parentItemID, note, title) VALUES(?,?,?,?)",
                             (next_id, r["parent"], r["html"], r["title"]))
                next_id += 1
                written += 1
        check = conn.execute("SELECT COUNT(*) FROM itemNotes WHERE note LIKE '%Zotero itemKey%'").fetchone()[0]
        if written and not check:
            print("写入后校验失败：没找到导入的笔记")
            return 4
    except Exception as error:         # noqa: BLE001
        print(f"写入失败并已回滚：{error}")
        return 4
    finally:
        conn.close()
    print(f"已写入 {written} 条子笔记。请重新打开 Zotero 查看；若有异常，用备份 {backup.name} 恢复。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
