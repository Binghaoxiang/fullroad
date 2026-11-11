#!/bin/sh
# ---------------------------------------------------------------------------
# 云效流水线 / 合并请求门禁用的检查入口（Linux 构建机上跑）
#
#   用法（流水线"执行命令"步骤里粘这一行）：
#       sh codestyle/ci_check.sh
#   或指定基线分支：
#       sh codestyle/ci_check.sh origin/main
#
#   检查不通过 → 退出码非 0 → 流水线失败 → 合并请求的"合并"按钮点不动。
#   这就是"上传必须经过检查"的服务端强制点，本地 --no-verify 绕不过去。
# ---------------------------------------------------------------------------
set -u

BASE="${1:-origin/main}"
ROOT=$(git rev-parse --show-toplevel 2>/dev/null) || {
    echo "不在 git 仓库里，无法检查"
    exit 1
}
cd "$ROOT" || exit 1

# 浅克隆的构建机可能没有基线分支，先补一下
git fetch origin "$BASE" --depth=100 >/dev/null 2>&1 || true

if [ ! -f codestyle/check_code.py ]; then
    echo "找不到 codestyle/check_code.py"
    exit 1
fi

if command -v python3 >/dev/null 2>&1; then
    exec python3 codestyle/check_code.py --ci "$BASE"
elif command -v python >/dev/null 2>&1; then
    exec python codestyle/check_code.py --ci "$BASE"
else
    # 门禁必须"失败关闭"：没有 python 就当作不通过，否则这道关就是空的
    echo "=============================================================="
    echo " 构建机上没有 python3，检查跑不了 —— 本次判定为不通过。"
    echo " 处理办法：流水线里把构建镜像换成带 Python 的（如 python:3.11），"
    echo "           或在步骤里先装：apt-get update && apt-get install -y python3"
    echo "=============================================================="
    exit 1
fi
