#!/usr/bin/env bash
# One command for "the efficiency run stopped, get me going again where it left off".
#
# 为什么需要它（所有者 2026-11-19）：示教链早就有 resume_teaching.sh（重启驱动栈 + 断点续跑），
# 但面板效率页用的 run_task.py 从来没有——失败就退出，只能从流程开头重来。
# 这里补上对称的一键恢复，但**故意不自动执行**：
#   · 自动只做不动机器人的动作（重发外部控制脚本 ensure_robot_ready.py --force）；
#   · 重启驱动栈 + 继续运动必须由人执行这一条命令（急停后先人工确认安全的铁律）。
#
# 用法：
#   bash scripts/resume_task.sh                 # 重挂 + 重启驱动栈 + 从失败阶段重试
#   bash scripts/resume_task.sh --no-restart    # 只重挂 + 续跑（驱动栈已健康）
#   bash scripts/resume_task.sh --allow-no-grasp # 恢复点在 grasp 之后但夹爪没夹着件时放行
#   bash scripts/resume_task.sh -- --until insert   # 额外参数透传给 run_task.py（在 -- 之后）
#
# 退出码：run_task.py 的退出码；读不到续跑状态或安全检查不过时为 1。
set -u

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
STATE="${REPO}/runtime/run_task_resume.json"
RESTART=1
ALLOW_NO_GRASP=0
EXTRA=()
while [ $# -gt 0 ]; do
  case "$1" in
    --no-restart) RESTART=0; shift ;;
    --allow-no-grasp) ALLOW_NO_GRASP=1; shift ;;
    --) shift; EXTRA=("$@"); break ;;
    *) EXTRA+=("$1"); shift ;;
  esac
done

if [ ! -f "${STATE}" ]; then
  echo "[resume] 没有续跑状态（${STATE} 不存在）——说明上次没有失败，或已成功跑完。"
  echo "         需要从头跑：python3 scripts/run_task.py --scope load --mode efficient --world real --confirm"
  exit 1
fi

echo "[resume] 续跑状态："
sed 's/^/         /' "${STATE}"

SCOPE=$(python3 -c "import json,sys;print(json.load(open(sys.argv[1])).get('scope',''))" "${STATE}")
MODE=$(python3 -c "import json,sys;print(json.load(open(sys.argv[1])).get('mode',''))" "${STATE}")
WORLD=$(python3 -c "import json,sys;print(json.load(open(sys.argv[1])).get('world',''))" "${STATE}")
START_INDEX=$(python3 -c "import json,sys;print(json.load(open(sys.argv[1])).get('start_index',0))" "${STATE}")
FAILED_PHASE=$(python3 -c "import json,sys;print(json.load(open(sys.argv[1])).get('failed_phase','?'))" "${STATE}")

if [ -z "${SCOPE}" ] || [ -z "${MODE}" ] || [ -z "${WORLD}" ]; then
  echo "[resume] FAILED：续跑状态缺 scope/mode/world，拒绝执行"
  exit 1
fi
echo "[resume] 计划：scope=${SCOPE} mode=${MODE} world=${WORLD} 从第 $((START_INDEX + 1)) 个阶段（${FAILED_PHASE}）重试"

cd "${REPO}" || exit 1
# shellcheck disable=SC1091
source "${REPO}/scripts/source_dev_env.sh" --full >/dev/null 2>&1

# --- 安全检查 1：恢复点在 grasp 之后时，夹爪必须真的夹着件 -------------------
GRASP_INDEX=$(python3 - "${SCOPE}" "${MODE}" <<'PY'
import sys, yaml, pathlib
scope, mode = sys.argv[1], sys.argv[2]
path = pathlib.Path(f"runtime/effective_{scope}_{mode}_{scope}.yaml")
if not path.exists():
    path = pathlib.Path(f"runtime/effective_{scope}_{mode}.yaml")
try:
    phases = yaml.safe_load(path.read_text(encoding="utf-8")).get("phases") or []
    print(phases.index("grasp") if "grasp" in phases else -1)
except Exception:
    print(-1)
PY
)
if [ "${GRASP_INDEX}" -ge 0 ] && [ "${START_INDEX}" -gt "${GRASP_INDEX}" ]; then
  echo "[resume] 恢复点在 grasp（第 $((GRASP_INDEX + 1)) 阶段）之后，检查夹爪是否夹着工件…"
  GRIP_STATUS="$(bash scripts/gripper_link.sh --status 2>&1 | tail -2)"
  echo "         ${GRIP_STATUS}"
  if echo "${GRIP_STATUS}" | grep -qiE "position_mm *=? *(0|[0-9])($|[^0-9])" \
     && ! echo "${GRIP_STATUS}" | grep -qiE "position_mm *=? *[4-9][0-9]"; then
    if [ "${ALLOW_NO_GRASP}" -eq 0 ]; then
      echo "[resume] REFUSED：夹爪看起来没夹着件（position_mm 偏小）。"
      echo "         工件可能已经掉落或没夹住；请先人工确认，再决定是否用 --allow-no-grasp。"
      exit 1
    fi
    echo "[resume] 警告：夹爪未确认夹持，但 --allow-no-grasp 已给出，继续。"
  fi
fi

# --- 安全检查 2：重挂（不动机器人） -----------------------------------------
echo "[resume] 重发外部控制脚本（ensure_robot_ready.py --force）"
python3 scripts/ensure_robot_ready.py --force || {
  echo "[resume] FAILED：重挂失败（code=$?）；先看示教器状态，不要继续动臂"
  exit 1
}

if [ "${RESTART}" -eq 1 ]; then
  echo "[resume] 重启驱动栈（recover_driver_stack.sh --timeout 120）"
  if ! "${REPO}/scripts/recover_driver_stack.sh" --timeout 120; then
    echo "[resume] FAILED：驱动栈没回来；不继续"
    exit 1
  fi
  python3 scripts/control_health.py --activate >/dev/null 2>&1 || true
else
  echo "[resume] 按要求跳过驱动栈重启"
fi

# --- 续跑 -------------------------------------------------------------------
echo "[resume] 继续效率流程：run_task.py --resume"
python3 -u scripts/run_task.py --scope "${SCOPE}" --mode "${MODE}" --world "${WORLD}" \
        --confirm --resume "${EXTRA[@]}"
status=$?
echo "[resume] run_task 退出码 ${status}"
if [ "${status}" -eq 0 ] && [ -f "${STATE}" ]; then
  echo "[resume] 注意：run_task 成功时应已清除续跑状态；若仍在，说明是本次又失败留下的"
fi
exit "${status}"
