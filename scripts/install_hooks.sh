#!/usr/bin/env bash
# 把提交前检查 hook 装到本机 .git/hooks（hook 不进版本库，换机器要重装）。
set -euo pipefail
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd -- "${script_dir}/.." && pwd)"
cp "${script_dir}/pre-commit.hook" "${repo_root}/.git/hooks/pre-commit"
chmod +x "${repo_root}/.git/hooks/pre-commit"
echo "已安装 .git/hooks/pre-commit（改动代码跑完整三连，只改文档跑 --fast）"
