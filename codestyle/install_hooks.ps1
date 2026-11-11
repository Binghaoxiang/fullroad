# 一键启用代码检查（每台电脑只需要跑一次）
#
#   powershell -ExecutionPolicy Bypass -File codestyle\install_hooks.ps1
#
# 之后每次 git commit 都会自动检查，不合规直接拦下来。

$ErrorActionPreference = "Stop"

$root = git rev-parse --show-toplevel 2>$null
if (-not $root) {
    Write-Host "当前目录不是 git 仓库，先 cd 到项目根目录再运行。" -ForegroundColor Red
    exit 1
}
Set-Location $root

if (-not (Test-Path "codestyle\check_code.py")) {
    Write-Host "找不到 codestyle\check_code.py，检查一下文件在不在。" -ForegroundColor Red
    exit 1
}
if (-not (Test-Path "codestyle\hooks\pre-commit")) {
    Write-Host "找不到 codestyle\hooks\pre-commit，检查一下文件在不在。" -ForegroundColor Red
    exit 1
}

git config core.hooksPath codestyle/hooks
if ($LASTEXITCODE -ne 0) {
    Write-Host "设置 core.hooksPath 失败。" -ForegroundColor Red
    exit 1
}

Write-Host ""
Write-Host "  [OK] 代码检查已启用（core.hooksPath = codestyle/hooks）" -ForegroundColor Green
Write-Host ""
Write-Host "  以后每次 git commit 会自动检查这 5 件事："
Write-Host "    1. 有没有把编译产物 / 安装包 / dist / ~`$xxx.pptx 锁文件传上去"
Write-Host "    2. 有没有超过 20MB 的文件"
Write-Host "    3. 有没有明文密码 / 私钥"
Write-Host "    4. 新增的 .c 有没有忘记加进 Keil 工程"
Write-Host "    5. 中断服务程序里有没有调用延时函数"
Write-Host "  另外还会检查提交信息格式：feat(模块): 说明"
Write-Host ""
Write-Host "  手动体检整个仓库：  py -3 codestyle\check_code.py"
Write-Host "  临时跳过一次检查：  git commit --no-verify"
Write-Host "  彻底关掉：          git config --unset core.hooksPath"
Write-Host ""
