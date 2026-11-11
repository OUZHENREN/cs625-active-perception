#!/usr/bin/env bash
# 夹爪串口链路（Windows 侧 HITBOT 夹爪，COM9）的检查/启动/停止。
#
# 夹爪不是 ROS 驱动
# -----------------
# 它是电动夹爪，走 Windows 的 COM9（115200，从机 1），有两种用法：
#   · 一次性调用：powershell -File C:\Users\Public\hitbot_cmd.ps1 -Cmd status
#     （每次开一次串口，不需要常驻进程）
#   · 常驻桥：C:\Users\Public\hitbot_bridge.ps1，监听 127.0.0.1:7500
#     （UI 面板/批处理用；它独占 COM9，与 HITBOT Studio 互斥）
#
# 所以"关夹爪驱动"= 停掉那个常驻桥（如果起着）；一次性调用没有常驻进程。
#
# 用法
# ----
#   bash scripts/gripper_link.sh --status   # 只看：桥是否在、夹爪能否应答
#   bash scripts/gripper_link.sh --ensure   # 桥不在就尝试拉起（失败只告警）
#   bash scripts/gripper_link.sh --stop     # 停掉桥，释放 COM9
set -u

PS_CMD='C:\Users\Public\hitbot_cmd.ps1'
PS_BRIDGE='C:\Users\Public\hitbot_bridge.ps1'

have_powershell() { command -v powershell.exe >/dev/null 2>&1; }

bridge_port_open() {
  # 127.0.0.1:7500 在 WSL 里访问不到 Windows 的 localhost，用 PowerShell 测
  have_powershell || return 1
  timeout 20 powershell.exe -NoProfile -Command \
    "(Test-NetConnection -ComputerName 127.0.0.1 -Port 7500 -InformationLevel Quiet) -eq \$true" \
    2>/dev/null | tr -d '\r' | grep -qi true
}

gripper_status() {
  have_powershell || { echo '{"ok":false,"error":"no powershell"}'; return; }
  timeout 30 powershell.exe -NoProfile -ExecutionPolicy Bypass -File "$PS_CMD" -Cmd status 2>&1 \
    | tr -d '\r' | tail -1
}

start_bridge() {
  have_powershell || { echo "[夹爪] 没有 powershell，跳过"; return 1; }
  timeout 30 powershell.exe -NoProfile -Command \
    "Start-Process powershell -ArgumentList '-NoProfile','-ExecutionPolicy','Bypass','-File','$PS_BRIDGE' -WindowStyle Hidden" \
    >/dev/null 2>&1 || return 1
  sleep 3
  bridge_port_open
}

stop_bridge() {
  have_powershell || return 1
  timeout 30 powershell.exe -NoProfile -Command \
    "Get-CimInstance Win32_Process -Filter \"Name='powershell.exe'\" | Where-Object { \$_.CommandLine -like '*hitbot_bridge.ps1*' } | ForEach-Object { Stop-Process -Id \$_.ProcessId -Force }" \
    >/dev/null 2>&1
  sleep 2
  ! bridge_port_open
}

MODE="${1:---status}"
case "$MODE" in
  --status)
    if bridge_port_open; then echo "[夹爪] 常驻桥在线（127.0.0.1:7500）"; else echo "[夹爪] 常驻桥未启动"; fi
    echo "[夹爪] 状态查询：$(gripper_status)"
    ;;
  --ensure)
    if bridge_port_open; then
      echo "[夹爪] 常驻桥已在线"
    elif start_bridge; then
      echo "[夹爪] 常驻桥已启动（127.0.0.1:7500）"
    else
      echo "[夹爪] 常驻桥启动失败或夹爪未上电——本次开工不带夹爪（不影响测量/观测）"
    fi
    echo "[夹爪] 状态：$(gripper_status)"
    ;;
  --stop)
    if bridge_port_open; then
      if stop_bridge; then echo "[夹爪] 常驻桥已停止，COM9 已释放"; else echo "[夹爪] 停止失败，请到 Windows 任务管理器结束 hitbot_bridge.ps1" >&2; exit 1; fi
    else
      echo "[夹爪] 常驻桥本来就没起，无需处理"
    fi
    echo "[提醒] 电动夹爪断电会失去夹持力：若正夹着工件，先张开再断电。"
    ;;
  *)
    echo "用法：bash scripts/gripper_link.sh [--status|--ensure|--stop]" >&2
    exit 2
    ;;
esac
