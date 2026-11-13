#!/usr/bin/env bash
# scripts/pre_commit_check.sh -- 提交前的离线检查一键入口（"三连"）。
#
# 为什么需要它
# ------------
# 提交前要跑的三件事原本分散在三个命令里，顺序固定，而且都必须先加载开发环境
# （不加载就会缺 rclpy 等依赖，pytest 会少收集测试模块，结果不可信）：
#
#   1) python3 -m pytest test/ -q        离线单元测试
#   2) python3 scripts/check_docs.py     文档一致性
#   3) python3 test/contract_checks.py   仓库治理合同
#
# 任何一步失败就立刻停在原地并说明是哪一步，避免"后两步通过"掩盖第一步的失败。
# 脚本自己切到仓库根、自己加载环境，因此可以在任意工作目录调用。
#
# 用法：
#   bash scripts/pre_commit_check.sh          # 完整三连（改代码后）
#   bash scripts/pre_commit_check.sh --fast   # 只改文档时：跳过 pytest，只跑 2) + 3)
set -euo pipefail

# 本脚本位于 scripts/ 下：先按自身路径解析仓库根，再切换过去，
# 后续所有相对路径（test/、install/、config/ 等）才有确定含义。
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd -- "${script_dir}/.." && pwd)"

FAST=0
for arg in "$@"; do
  case "${arg}" in
    --fast) FAST=1 ;;
    -h|--help)
      echo "用法: bash scripts/pre_commit_check.sh [--fast]"
      echo "  无参数  完整三连：pytest + 文档一致性 + 治理合同"
      echo "  --fast  跳过 pytest，只跑后两项（只改文档时用）"
      exit 0
      ;;
    *)
      echo "[pre-commit] 未知参数: ${arg}" >&2
      echo "用法: bash scripts/pre_commit_check.sh [--fast]" >&2
      exit 2
      ;;
  esac
done

cd "${repo_root}"
echo "[pre-commit] 仓库根: ${repo_root}"

# 环境必须由脚本自己加载（调用方 shell 里可能什么都没有）。
echo "[pre-commit] 加载开发环境: source scripts/source_dev_env.sh --full"
if ! source "${repo_root}/scripts/source_dev_env.sh" --full; then
  echo "[pre-commit] 失败：开发环境加载失败（scripts/source_dev_env.sh --full）" >&2
  exit 1
fi

step_total=3
step_index=0

# run_step "<描述>" <命令...>：打印进度，失败则立刻退出并指明是哪一步。
run_step() {
  local desc="$1"
  shift
  step_index=$((step_index + 1))
  echo "[pre-commit] 步骤 ${step_index}/${step_total}: ${desc}"
  if ! "$@"; then
    echo "[pre-commit] 失败：步骤 ${step_index}/${step_total}（${desc}）未通过，后续步骤未执行" >&2
    exit 1
  fi
}

if [[ "${FAST}" == "1" ]]; then
  step_total=2
  echo "[pre-commit] --fast：跳过 pytest，只检查文档一致性与治理合同"
  run_step "文档一致性 (python3 scripts/check_docs.py)" python3 scripts/check_docs.py
  run_step "仓库治理合同 (python3 test/contract_checks.py)" python3 test/contract_checks.py
else
  run_step "pytest 离线单元测试 (python3 -m pytest test/ -q)" python3 -m pytest test/ -q
  run_step "文档一致性 (python3 scripts/check_docs.py)" python3 scripts/check_docs.py
  run_step "仓库治理合同 (python3 test/contract_checks.py)" python3 test/contract_checks.py
fi

echo "[pre-commit] 全部通过，可以提交"
exit 0
