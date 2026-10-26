#!/usr/bin/env python3
"""CS625 任务实时状态页（只读观测，不参与控制）。

为什么需要它
------------
`run_pick_and_place.py` 每跑一个阶段才往 `runtime/phase_log_<scope>.json` 写一条记录，
而终端里的输出是滚动的。想在仿真跑的时候"边看边判断"，需要一个能一直开着、
自动刷新的页面：阶段跑到哪、成没成、六关节现在是什么值、日志最后几行说了什么。

它做什么
--------
一个极小的 HTTP 服务，把三样东西渲染成一页：

    runtime/phase_log_<scope>.json   → 阶段进度表（ok / 耗时 / 细节）
    runtime/effective_*.yaml         → 本次应该跑哪些阶段（用来显示"还没开始"的行）
    /joint_states                    → 六关节实时值（只订阅，不发任何指令）

数据流：文件 + ROS 话题  →  本脚本  →  http://127.0.0.1:8091/

它不做什么
----------
- 不发任何运动指令、不调用任何服务、不写仓库文件
- 不改 `run_pick_and_place.py` / `run_task.py` 的任何行为

用法
----
    source scripts/source_dev_env.sh --full
    python3 scripts/live_status_page.py                       # 默认 8091
    python3 scripts/live_status_page.py --port 8091 --domain 80
    python3 scripts/live_status_page.py --log /tmp/run.log    # 附带日志尾部

然后浏览器打开 http://127.0.0.1:8091/ （页面每秒自动刷新）。
"""

from __future__ import annotations

import argparse
import glob
import html
import json
import os
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
JOINTS = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
          "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]

# ---------------------------------------------------------------- shared state
LOCK = threading.Lock()
STATE = {
    "joints": {},          # name -> rad
    "joints_t": 0.0,       # 最近一次收到的时间
    "ros_ok": False,
    "ros_note": "未连接",
    "domain": "",
}


def _newest(pattern: str) -> Path | None:
    hits = [Path(p) for p in glob.glob(str(REPO / pattern))]
    if not hits:
        return None
    return max(hits, key=lambda p: p.stat().st_mtime)


def read_phase_log() -> tuple[list[dict], Path | None]:
    p = _newest("runtime/phase_log_*.json")
    if p is None:
        return [], None
    try:
        data = json.loads(p.read_text(encoding="utf-8"))
        return (data if isinstance(data, list) else []), p
    except Exception:
        return [], p


def read_expected_phases() -> tuple[list[str], str]:
    """从最近一次生效配置里取阶段表，用来渲染"还没跑到"的行。"""
    p = _newest("runtime/effective_*.yaml")
    if p is None:
        return [], ""
    try:
        import yaml
        cfg = yaml.safe_load(p.read_text(encoding="utf-8")) or {}
        phases = [str(x) for x in (cfg.get("phases") or [])]
        tag = f"{cfg.get('scope', '?')}/{cfg.get('mode', '?')}"
        return phases, tag
    except Exception:
        return [], ""


def tail_lines(path: str, n: int = 18) -> list[str]:
    if not path or not os.path.exists(path):
        return []
    try:
        with open(path, "r", errors="replace") as f:
            return [ln.rstrip("\n") for ln in f.readlines()[-n:]]
    except Exception:
        return []


# ------------------------------------------------------------------- ROS 只读
def ros_thread(domain: str) -> None:
    """只订阅 /joint_states。失败就降级，页面其余部分照常可用。"""
    if domain:
        os.environ["ROS_DOMAIN_ID"] = str(domain)
    try:
        import rclpy
        from sensor_msgs.msg import JointState
    except Exception as exc:                     # ROS 环境没加载
        with LOCK:
            STATE["ros_note"] = f"未加载 ROS 环境（{type(exc).__name__}）"
        return

    try:
        rclpy.init()
        node = rclpy.create_node("live_status_page")

        def cb(msg):
            with LOCK:
                for i, name in enumerate(msg.name):
                    if name in JOINTS and i < len(msg.position):
                        STATE["joints"][name] = msg.position[i]
                STATE["joints_t"] = time.time()
                STATE["ros_ok"] = True
                STATE["ros_note"] = "已连接"

        node.create_subscription(JointState, "/joint_states", cb, 10)
        with LOCK:
            STATE["domain"] = os.environ.get("ROS_DOMAIN_ID", "")
        while True:
            rclpy.spin_once(node, timeout_sec=0.2)
    except Exception as exc:
        with LOCK:
            STATE["ros_ok"] = False
            STATE["ros_note"] = f"ROS 线程异常：{type(exc).__name__}: {exc}"


# ---------------------------------------------------------------------- 渲染
CSS = """
 body{font-family:system-ui,-apple-system,"Segoe UI",Roboto,"Helvetica Neue",sans-serif;
      margin:0;padding:18px 22px;background:#0f1216;color:#e6e9ef;}
 h1{font-size:19px;margin:0 0 4px;font-weight:600}
 .sub{color:#8b95a5;font-size:12px;margin-bottom:16px}
 .card{background:#161b22;border:1px solid #232a34;border-radius:8px;padding:14px 16px;margin-bottom:14px}
 .card h2{font-size:13px;margin:0 0 10px;color:#9aa7b8;font-weight:600;letter-spacing:.04em}
 table{border-collapse:collapse;width:100%;font-size:13px}
 th,td{text-align:left;padding:5px 9px;border-bottom:1px solid #1e242c}
 th{color:#7d8899;font-weight:500;font-size:11.5px;letter-spacing:.03em}
 td.mono{font-family:ui-monospace,SFMono-Regular,Menlo,monospace;font-size:12px}
 .ok{color:#3fb950;font-weight:600}.bad{color:#f85149;font-weight:600}
 .wait{color:#6e7681}.now{color:#d29922;font-weight:600}
 .bar{height:7px;background:#1e242c;border-radius:4px;overflow:hidden;margin:8px 0 12px}
 .bar>i{display:block;height:100%;background:#3fb950}
 pre{background:#0b0e12;border:1px solid #1e242c;border-radius:6px;padding:10px;
     font-size:11.5px;line-height:1.45;overflow-x:auto;color:#b7c0cd;margin:0;
     font-family:ui-monospace,SFMono-Regular,Menlo,monospace}
 .kv{display:flex;flex-wrap:wrap;gap:22px;font-size:13px}
 .kv div span{color:#7d8899;margin-right:6px}
 .pill{display:inline-block;padding:1px 8px;border-radius:10px;font-size:11px;
       background:#1f2630;color:#9aa7b8}
"""


def render(log_path: str) -> str:
    entries, plog = read_phase_log()
    expected, tag = read_expected_phases()
    with LOCK:
        joints = dict(STATE["joints"])
        joints_t = STATE["joints_t"]
        ros_note = STATE["ros_note"]
        dom = STATE["domain"]
        ros_ok = STATE["ros_ok"]

    done = {e.get("phase"): e for e in entries}
    total = max(len(expected), len(entries))
    ok_n = sum(1 for e in entries if e.get("ok"))
    pct = int(100 * len(entries) / total) if total else 0

    rows = []
    for i in range(total):
        if i < len(expected):
            name = expected[i]
        elif i < len(entries):
            name = entries[i].get("phase", "?")
        else:
            continue
        e = done.get(name)
        if e is None:
            cls, mark, dur, detail = "wait", "待跑", "", ""
        else:
            ok = e.get("ok")
            cls = "ok" if ok else "bad"
            mark = "通过" if ok else "失败"
            dur = f"{e.get('dur', '')}" if e.get("dur") else ""
            detail = str(e.get("detail", ""))[:150]
            if not dur:
                import re
                m = re.search(r"([\d.]+)s", detail)
                dur = m.group(1) + "s" if m else ""
        tcp = (e or {}).get("tcp")
        tcp_s = "(" + ", ".join(f"{v:+.3f}" for v in tcp) + ")" if tcp else ""
        rows.append(
            f"<tr><td class='mono'>{i+1:02d}</td><td>{html.escape(name)}</td>"
            f"<td class='{cls}'>{mark}</td><td class='mono'>{dur}</td>"
            f"<td class='mono'>{tcp_s}</td><td class='mono'>{html.escape(detail)}</td></tr>")

    jr = []
    for n in JOINTS:
        v = joints.get(n)
        if v is None:
            jr.append(f"<tr><td>{n}</td><td class='wait mono'>—</td></tr>")
        else:
            import math
            jr.append(f"<tr><td>{n}</td><td class='mono'>{v:+.4f} rad"
                      f"<span class='wait'> / {math.degrees(v):+.2f}°</span></td></tr>")

    age = f"{time.time() - joints_t:.1f}s 前" if joints_t else "无数据"
    log_html = html.escape("\n".join(tail_lines(log_path))) or "（未指定 --log，或文件为空）"
    now = time.strftime("%H:%M:%S")

    return f"""<!doctype html><html lang="zh"><head><meta charset="utf-8">
<meta http-equiv="refresh" content="1">
<title>CS625 任务实时状态</title><style>{CSS}</style></head><body>
<h1>CS625 任务实时状态</h1>
<div class="sub">数据源 {html.escape(str(plog.name if plog else "（无阶段日志）"))}
 · 刷新 {now} · 每秒自动刷新 · 只读页面，不发送任何指令</div>

<div class="card"><h2>总览</h2>
<div class="kv">
 <div><span>任务</span>{html.escape(tag or "—")}</div>
 <div><span>阶段</span>{len(entries)} / {total}</div>
 <div><span>通过</span><b class="ok">{ok_n}</b></div>
 <div><span>失败</span><b class="bad">{len(entries)-ok_n}</b></div>
 <div><span>ROS</span><span class="pill">{html.escape(ros_note)}</span>
      <span class="pill">域 {html.escape(dom or "默认")}</span></div>
 <div><span>关节数据</span>{age}</div>
</div>
<div class="bar"><i style="width:{pct}%"></i></div></div>

<div class="card"><h2>阶段进度</h2>
<table><tr><th>#</th><th>阶段</th><th>状态</th><th>耗时</th><th>实测 TCP</th><th>细节</th></tr>
{''.join(rows)}</table></div>

<div class="card"><h2>六关节（实时）</h2>
<table><tr><th>关节</th><th>位置</th></tr>{''.join(jr)}</table></div>

<div class="card"><h2>日志尾部</h2><pre>{log_html}</pre></div>
</body></html>"""


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", type=int, default=8091)
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--domain", default=os.environ.get("ROS_DOMAIN_ID", ""))
    ap.add_argument("--log", default="", help="要显示尾部的日志文件")
    args = ap.parse_args()

    threading.Thread(target=ros_thread, args=(args.domain,), daemon=True).start()

    class Handler(BaseHTTPRequestHandler):
        def do_GET(self):                       # noqa: N802
            body = render(args.log).encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def log_message(self, *a):              # 安静一点
            pass

    srv = ThreadingHTTPServer((args.host, args.port), Handler)
    print(f"CS625 实时状态页: http://{args.host}:{args.port}/   (Ctrl-C 退出)", flush=True)
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
