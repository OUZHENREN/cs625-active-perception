#!/usr/bin/env bash
# Record the whole WSLg desktop (the Gazebo / RViz window) to an H.264 video.
#
# Why not `ffmpeg -f x11grab`: WSLg's X server refuses root-window reads, so that
# capture comes out black (verified: std = 0.0). The Windows side *can* capture its own
# desktop, so this script drives a small PowerShell loop that dumps raw BGRA frames and
# then encodes them here with ffmpeg (GDI+ Save() hangs on this machine, hence raw).
#
# Usage:
#   scripts/record_sim_screen.sh start             # start capturing in the background
#   scripts/record_sim_screen.sh stop  [name]      # stop, encode to ~/cs625_evidence/<name>.mp4
#   scripts/record_sim_screen.sh shot  [file.png]  # one screenshot
#   scripts/record_sim_screen.sh clean             # drop the raw frames
#
# Disk: 2560x1440x4 B = 14.7 MB per frame. At 4 fps a 15 min run is ~53 GB, so keep
# recordings short (capture only the run you care about) and call `clean` afterwards.
set -euo pipefail

PS1=/mnt/c/Temp/win_grab_loop.ps1
OUTDIR=/mnt/c/Temp/wingrab
PIDFILE=/tmp/cs625_screen_rec.pid
LOG=/tmp/cs625_screen_rec.log
FPS=${FPS:-4}

start() {
    mkdir -p /mnt/c/Temp "$OUTDIR"
    rm -f "$OUTDIR"/*.bgra
    : > "$LOG"
    ( cd /mnt/c && setsid nohup powershell.exe -NoProfile -NonInteractive \
        -ExecutionPolicy Bypass -File 'C:\Temp\win_grab_loop.ps1' \
        -Frames 1000000 -IntervalMs $((1000 / FPS)) -OutDir 'C:\Temp\wingrab' \
        >>"$LOG" 2>&1 & echo $! > "$PIDFILE" )
    sleep 3
    echo "recording ${FPS} fps -> $OUTDIR"
    grep -oE '[0-9]+ x [0-9]+' "$LOG" | tail -1 || true
}

stop() {
    local name="${1:-sim_screen_$(date +%Y%m%d_%H%M%S)}"
    if [ -f "$PIDFILE" ]; then
        kill -TERM "$(cat "$PIDFILE")" 2>/dev/null || true
        rm -f "$PIDFILE"
    fi
    sleep 2
    local count
    count=$(ls "$OUTDIR"/*.bgra 2>/dev/null | wc -l)
    echo "frames captured: $count"
    [ "$count" -gt 0 ] || { echo "no frames" >&2; exit 1; }

    local w h
    w=$(grep -oE '[0-9]+ x [0-9]+' "$LOG" | tail -1 | awk '{print $1}')
    h=$(grep -oE '[0-9]+ x [0-9]+' "$LOG" | tail -1 | awk '{print $3}')
    w=${w:-2560}; h=${h:-1440}

    local outdir="$HOME/cs625_evidence" all=/tmp/cs625_screen_frames.bgra
    mkdir -p "$outdir"
    cat "$OUTDIR"/frame_*.bgra > "$all"
    ffmpeg -hide_banner -loglevel error -y \
        -f rawvideo -pix_fmt bgra -s "${w}x${h}" -r "$FPS" -i "$all" \
        -vf "scale=1280:-2" -c:v libx264 -preset veryfast -crf 26 -pix_fmt yuv420p \
        "$outdir/$name.mp4"
    rm -f "$all"
    echo "video: $outdir/$name.mp4 ($(du -h "$outdir/$name.mp4" | cut -f1), ${w}x${h} -> 1280 wide)"
}

case "${1:-}" in
    start) start ;;
    stop)  stop "${2:-}" ;;
    shot)  /tmp/grab_wslg.sh "${2:-/tmp/shot.png}"; echo "screenshot: ${2:-/tmp/shot.png}" ;;
    clean) rm -f "$OUTDIR"/*.bgra; echo "raw frames removed" ;;
    *)     sed -n '2,20p' "$0"; exit 2 ;;
esac
