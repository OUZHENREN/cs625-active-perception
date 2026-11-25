// Zotero 子笔记导入器（所有者 2026-11-26：Zotero 开启状态下安全写入）
//
// 为什么用这个而不是改数据库
// ------------------------------------------------------------------
// 直接写 zotero.sqlite 在 Zotero 运行时可能损坏库；Zotero 本地 API 也未开启。
// Zotero 自带 JS 运行器可以在**运行中**安全地创建条目，所以走这条路。
//
// 用法
// ----
// 1. 先在仓库里生成清单（会同时复制到 Windows 临时目录）：
//      python3 scripts/zotero_import_notes.py --emit-manifest
// 2. Zotero → 工具(Tools) → 开发者(Developer) → Run JavaScript
// 3. 打开本文件，粘贴全部内容，点 Run
// 4. 返回值形如：created 12, skipped 0, missing 0
//
// 去重：每条笔记 HTML 里带 `<!-- litnote:<itemKey> -->` 标记，已存在则跳过。

const MANIFEST_PATHS = [
  "C:\\Users\\l'z\\AppData\\Local\\Temp\\zotero_notes_manifest.json",
  "C:\\Users\\Public\\zotero_notes_manifest.json",
];

(async () => {
  let manifest = null, used = null;
  for (const p of MANIFEST_PATHS) {
    try {
      const raw = await Zotero.File.getContentsAsync(p);
      manifest = JSON.parse(raw);
      used = p;
      break;
    } catch (e) { /* try next */ }
  }
  if (!manifest) {
    return "找不到清单文件，请先运行 python3 scripts/zotero_import_notes.py --emit-manifest";
  }
  const libraryID = Zotero.Libraries.userLibraryID;
  let created = 0, skipped = 0, missing = 0;
  const missingKeys = [];
  for (const entry of manifest.notes) {
    const item = Zotero.Items.getByLibraryAndKey(libraryID, entry.itemKey);
    if (!item) { missing++; missingKeys.push(entry.itemKey); continue; }
    const marker = `<!-- litnote:${entry.itemKey} -->`;
    const existing = item.getNotes()
      .map(id => Zotero.Items.get(id))
      .find(n => (n.getNote() || "").includes(marker));
    if (existing) { skipped++; continue; }
    const note = new Zotero.Item("note");
    note.libraryID = item.libraryID;
    note.parentItemID = item.id;
    note.setNote(marker + entry.html);
    await note.saveTx();
    created++;
  }
  return `清单 ${used}\ncreated ${created}, skipped ${skipped}, missing ${missing}`
    + (missingKeys.length ? `\n缺失: ${missingKeys.join(", ")}` : "");
})();
