#!/usr/bin/env python3
"""CS625 文件查看器 —— 在 WSL 里起一个只读的小型网页服务，Windows 浏览器直接看。

为什么需要它
------------
实验视频、过程图、配置、日志都在 WSL 的文件系统里，Windows 资源管理器直接打开
`\\\\wsl.localhost\\...` 既慢又经常无法预览 mp4（拖不动进度条）。这个脚本只做三件事：

1. 列出目录（文件名、大小、修改时间），目录可点进去；
2. 图片/文本直接在页面里显示，视频用 `<video>` 播放并**支持拖动进度条**（HTTP Range）；
3. 一切都是只读的：只服务一个根目录，拒绝越界路径，只监听 127.0.0.1。

用法
----
    python3 scripts/file_viewer.py                     # 默认服务 ~/cs625_evidence
    python3 scripts/file_viewer.py --root ~ --port 8090
    python3 scripts/file_viewer.py --root . --open     # 起完自动打开浏览器

然后在 Windows 浏览器访问 `http://localhost:<port>/`（WSL2 默认把 localhost 转发到 WSL）。

安全边界（刻意保持很窄）
------------------------
- 只读：只实现 GET/HEAD，不实现上传、删除、重命名；
- 只服务 `--root` 以下的路径，`..` 一律拒绝；
- 只监听 127.0.0.1，不对外网暴露；
- 不跟随符号链接到 root 之外（`resolve()` 后重新校验）。
"""

from __future__ import annotations

import argparse
import html
import mimetypes
import os
import pathlib
import shutil
import subprocess
import sys
import time
import urllib.parse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

MIME_OVERRIDES = {
    ".mp4": "video/mp4", ".webm": "video/webm", ".mkv": "video/x-matroska",
    ".jpg": "image/jpeg", ".jpeg": "image/jpeg", ".png": "image/png",
    ".md": "text/markdown; charset=utf-8", ".log": "text/plain; charset=utf-8",
    ".yaml": "text/plain; charset=utf-8", ".yml": "text/plain; charset=utf-8",
    ".json": "application/json; charset=utf-8", ".csv": "text/csv; charset=utf-8",
}
TEXT_SUFFIXES = {".md", ".txt", ".log", ".yaml", ".yml", ".json", ".csv", ".py", ".sh"}
VIDEO_SUFFIXES = {".mp4", ".webm", ".mkv"}
IMAGE_SUFFIXES = {".jpg", ".jpeg", ".png", ".gif", ".webp"}


def human_size(num: int) -> str:
    for unit in ("B", "KB", "MB", "GB"):
        if num < 1024 or unit == "GB":
            return f"{num:.0f} {unit}" if unit == "B" else f"{num:.1f} {unit}"
        num /= 1024.0
    return f"{num:.1f} GB"


class ViewerHandler(BaseHTTPRequestHandler):
    """只读目录浏览 + Range 请求（视频拖动进度条必需）。"""

    root: pathlib.Path = pathlib.Path.home() / "cs625_evidence"
    server_version = "CS625FileViewer/1.0"

    # ---------------------------------------------------------------- helpers
    def _resolve(self, raw_path: str) -> pathlib.Path | None:
        """把 URL 路径解析成 root 以内的真实路径；越界返回 None。"""
        relative = urllib.parse.unquote(raw_path).lstrip("/")
        candidate = (self.root / relative).resolve()
        root = self.root.resolve()
        if candidate != root and root not in candidate.parents:
            return None
        return candidate

    def _send_bytes(self, data: bytes, content_type: str, status: int = 200) -> None:
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        if self.command != "HEAD":
            self.wfile.write(data)

    # ------------------------------------------------------------- GET/HEAD
    def do_HEAD(self) -> None:  # noqa: N802 - BaseHTTPRequestHandler API
        self.do_GET(head_only=True)

    def do_GET(self, head_only: bool = False) -> None:  # noqa: N802
        path = self._resolve(self.path.split("?")[0])
        if path is None:
            self._send_bytes("403 路径越界".encode(), "text/plain; charset=utf-8", 403)
            return
        if not path.exists():
            self._send_bytes("404 不存在".encode(), "text/plain; charset=utf-8", 404)
            return
        if path.is_dir():
            self._send_bytes(self._render_index(path), "text/html; charset=utf-8")
            return
        self._serve_file(path, head_only)

    # ------------------------------------------------------------ directory
    def _render_index(self, directory: pathlib.Path) -> bytes:
        root = self.root.resolve()
        relative = directory.resolve().relative_to(root)
        parts = [html.escape(relative.as_posix())] if str(relative) != "." else []
        breadcrumb = '<a href="/">根目录</a>'
        acc = ""
        for part in parts:
            if part in ("", "."):
                continue
            acc += "/" + part
            breadcrumb += f' / <a href="{urllib.parse.quote(acc)}/">{html.escape(part)}</a>'

        entries = []
        try:
            children = sorted(directory.iterdir(),
                              key=lambda p: (not p.is_dir(), p.name.lower()))
        except OSError as exc:
            children = []
            entries.append(f'<p class="err">无法列出目录：{html.escape(str(exc))}</p>')
        for item in children:
            if item.name.startswith("."):
                continue
            try:
                stat = item.stat()
            except OSError:
                continue
            if item.is_dir():
                href = urllib.parse.quote("/" + str(item.relative_to(root)) + "/")
                kind, size = "目录", "—"
                name = html.escape(item.name) + "/"
            else:
                href = urllib.parse.quote("/" + str(item.relative_to(root)))
                suffix = item.suffix.lower()
                if suffix in VIDEO_SUFFIXES:
                    kind = "视频"
                elif suffix in IMAGE_SUFFIXES:
                    kind = "图片"
                elif suffix in TEXT_SUFFIXES:
                    kind = "文本"
                else:
                    kind = "文件"
                size = human_size(stat.st_size)
                name = html.escape(item.name)
            stamp = time.strftime("%m-%d %H:%M", time.localtime(stat.st_mtime))
            entries.append(
                f'<tr><td><a href="{href}">{name}</a></td><td>{kind}</td>'
                f'<td class="num">{size}</td><td class="num">{stamp}</td></tr>')

        page = f"""<!doctype html>
<html lang="zh-CN"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>CS625 文件查看器 · /{html.escape(str(relative))}</title>
<style>
 :root {{ color-scheme: dark; }}
 body {{ margin:0; padding:24px; background:#0f141a; color:#dbe7f3;
        font:14px/1.6 system-ui, "Segoe UI", "Microsoft YaHei", sans-serif; }}
 h1 {{ font-size:15px; font-weight:600; margin:0 0 16px; color:#9fb4c8; }}
 h1 a {{ color:#35d0c0; text-decoration:none; }}
 table {{ width:100%; border-collapse:collapse; background:#161d26; border-radius:10px;
         overflow:hidden; }}
 th, td {{ padding:9px 12px; text-align:left; border-bottom:1px solid #24303d; }}
 th {{ font-weight:600; color:#8aa0b4; font-size:12px; letter-spacing:.04em; }}
 tr:last-child td {{ border-bottom:none; }}
 tr:hover td {{ background:#1b2430; }}
 a {{ color:#dbe7f3; text-decoration:none; }}
 a:hover {{ color:#35d0c0; }}
 .num {{ text-align:right; color:#8aa0b4; font-variant-numeric:tabular-nums; }}
 .err {{ color:#ef6b6b; }}
 video, img {{ max-width:100%; border-radius:10px; display:block; margin:16px 0; }}
 pre {{ background:#161d26; padding:14px; border-radius:10px; overflow:auto; }}
 .meta {{ color:#8aa0b4; font-size:12px; margin-bottom:16px; }}
</style></head><body>
<h1>{breadcrumb}</h1>
<div class="meta">只读 · 根目录 {html.escape(str(root))} · {len(entries)} 项</div>
<table><thead><tr><th>名称</th><th>类型</th><th class="num">大小</th>
<th class="num">修改时间</th></tr></thead><tbody>
{''.join(entries) if entries else '<tr><td colspan="4">（空目录）</td></tr>'}
</tbody></table></body></html>"""
        return page.encode("utf-8")

    # ------------------------------------------------------------------ file
    def _serve_file(self, path: pathlib.Path, head_only: bool) -> None:
        suffix = path.suffix.lower()
        content_type = MIME_OVERRIDES.get(suffix) or mimetypes.guess_type(
            str(path))[0] or "application/octet-stream"
        try:
            stat = path.stat()
        except OSError:
            self._send_bytes("404 不存在".encode(), "text/plain; charset=utf-8", 404)
            return

        # 文本：包一层 <pre>，省得浏览器把 .md 当下载
        if suffix in TEXT_SUFFIXES and stat.st_size < 2_000_000:
            body = path.read_text(encoding="utf-8", errors="replace")
            page = ("<html><head><meta charset='utf-8'><style>"
                    "body{margin:0;padding:20px;background:#0f141a;color:#dbe7f3;"
                    "font:13px/1.55 ui-monospace,Consolas,monospace;}"
                    "pre{white-space:pre-wrap;word-break:break-word;}"
                    "a{color:#35d0c0;}</style></head><body><pre>"
                    f"{html.escape(body)}</pre></body></html>")
            self._send_bytes(page.encode("utf-8"), "text/html; charset=utf-8")
            return

        # 视频/图片：交给浏览器，支持 Range 才能拖进度条
        range_header = self.headers.get("Range")
        if range_header and range_header.startswith("bytes="):
            self._serve_range(path, stat.st_size, content_type, range_header, head_only)
            return
        self.send_response(200)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(stat.st_size))
        self.send_header("Accept-Ranges", "bytes")
        self.end_headers()
        if head_only:
            return
        with path.open("rb") as handle:
            shutil.copyfileobj(handle, self.wfile, length=256 * 1024)

    def _serve_range(self, path: pathlib.Path, size: int, content_type: str,
                     range_header: str, head_only: bool) -> None:
        try:
            spec = range_header.split("=", 1)[1].split(",")[0]
            start_text, _, end_text = spec.partition("-")
            start = int(start_text) if start_text else 0
            end = int(end_text) if end_text else size - 1
        except (ValueError, IndexError):
            self.send_error(416, "Range 无法解析")
            return
        start = max(0, min(start, size - 1))
        end = max(start, min(end, size - 1))
        length = end - start + 1
        self.send_response(206)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Range", f"bytes {start}-{end}/{size}")
        self.send_header("Content-Length", str(length))
        self.send_header("Accept-Ranges", "bytes")
        self.end_headers()
        if head_only:
            return
        with path.open("rb") as handle:
            handle.seek(start)
            remaining = length
            while remaining > 0:
                chunk = handle.read(min(256 * 1024, remaining))
                if not chunk:
                    break
                self.wfile.write(chunk)
                remaining -= len(chunk)

    def log_message(self, fmt: str, *args) -> None:  # keep the terminal readable
        sys.stderr.write("%s  %s\n" % (time.strftime("%H:%M:%S"), fmt % args))


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--root", default="~/cs625_evidence",
                        help="要浏览的目录（默认 ~/cs625_evidence）")
    parser.add_argument("--port", type=int, default=8090)
    parser.add_argument("--host", default="127.0.0.1",
                        help="只建议 127.0.0.1；改成 0.0.0.0 会把文件暴露给局域网")
    parser.add_argument("--open", action="store_true", help="启动后自动打开浏览器")
    args = parser.parse_args(argv)

    root = pathlib.Path(args.root).expanduser()
    if not root.is_dir():
        print(f"目录不存在：{root}", file=sys.stderr)
        return 2
    ViewerHandler.root = root
    server = ThreadingHTTPServer((args.host, args.port), ViewerHandler)
    url = f"http://{args.host}:{args.port}/"
    print(f"CS625 文件查看器: {url}")
    print(f"  根目录 : {root}")
    print(f"  只读   : 只实现 GET/HEAD，越界路径一律 403")
    print(f"  停止   : Ctrl+C")
    if args.open:
        for opener in ("xdg-open", "wslview"):
            if shutil.which(opener):
                subprocess.Popen([opener, url], stdout=subprocess.DEVNULL,
                                 stderr=subprocess.DEVNULL)
                break
        else:
            print("  提示   : 手动在 Windows 浏览器打开上面的地址")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\n已停止")
    finally:
        server.server_close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
