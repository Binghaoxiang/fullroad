# STM32CubeProgrammer 一键解压 + 静默安装 + 检测
#   用法：
#     powershell -ExecutionPolicy Bypass -File .\tools\install_cubeprog.ps1
#     powershell -ExecutionPolicy Bypass -File .\tools\install_cubeprog.ps1 -Zip "D:\Downloads\stm32cubeprg-win64-v2-19-0.zip"
#   不指定 -Zip 时会自动在"下载"目录里找 stm32cubeprg*.zip

param(
    [string]$Zip = ''
)

$ErrorActionPreference = 'Stop'

function Find-Zip {
    $dirs = @("$env:USERPROFILE\Downloads", "$env:USERPROFILE\Desktop", "$env:USERPROFILE\OneDrive\Downloads")
    foreach ($d in $dirs) {
        if (Test-Path $d) {
            $f = Get-ChildItem $d -Filter 'stm32cubeprg*.zip' -ErrorAction SilentlyContinue |
                 Sort-Object LastWriteTime -Descending | Select-Object -First 1
            if ($f) { return $f.FullName }
        }
    }
    return $null
}

if (-not $Zip) { $Zip = Find-Zip }
if (-not $Zip -or -not (Test-Path $Zip)) {
    Write-Host '没找到 stm32cubeprg*.zip。' -ForegroundColor Red
    Write-Host '请把下载好的 zip 放到"下载"目录，或用 -Zip 指定路径：' -ForegroundColor Yellow
    Write-Host '  .\tools\install_cubeprog.ps1 -Zip "D:\Downloads\stm32cubeprg-win64-v2-19-0.zip"' -ForegroundColor Yellow
    exit 1
}

$sizeMB = [math]::Round((Get-Item $Zip).Length / 1MB, 1)
Write-Host "使用安装包: $Zip  ($sizeMB MB)" -ForegroundColor Cyan
if ($sizeMB -lt 200) {
    Write-Host '这个 zip 太小了，可能是下载中断的残包（正常约 1 GB）' -ForegroundColor Red
    exit 1
}

# 1. 解压
$work = Join-Path $env:TEMP 'cubeprog_setup'
if (Test-Path $work) { Remove-Item $work -Recurse -Force }
New-Item -ItemType Directory -Force -Path $work | Out-Null
Write-Host '解压中（1 GB 左右，要一会儿）...' -ForegroundColor Cyan
Expand-Archive -Path $Zip -DestinationPath $work -Force

# 2. 找 setup 程序并校验签名
$setup = Get-ChildItem $work -Recurse -Filter 'SetupSTM32CubeProgrammer*.exe' |
         Select-Object -First 1 -ExpandProperty FullName
if (-not $setup) {
    Write-Host 'zip 里没找到 SetupSTM32CubeProgrammer*.exe，看看目录结构：' -ForegroundColor Red
    Get-ChildItem $work -Recurse -Depth 2 | Select-Object -First 20 FullName
    exit 1
}
Write-Host "安装程序: $setup" -ForegroundColor Cyan
$sig = Get-AuthenticodeSignature $setup
Write-Host ("数字签名: {0} / {1}" -f $sig.Status, $sig.SignerCertificate.Subject) -ForegroundColor Gray
if ($sig.Status -ne 'Valid') {
    Write-Host '⚠ 签名不是 Valid，为安全起见停止安装' -ForegroundColor Red
    exit 1
}

# 3. 静默安装（依次尝试不同的静默参数）
$attempts = @(
    @('/s', '/v/qn'),
    @('/S'),
    @('/silent'),
    @('-i', 'silent')
)
$installed = $false
foreach ($a in $attempts) {
    Write-Host ("尝试静默安装: {0} {1}" -f $setup, ($a -join ' ')) -ForegroundColor Cyan
    try {
        $p = Start-Process -FilePath $setup -ArgumentList $a -Wait -PassThru
        Write-Host "  退出码: $($p.ExitCode)"
    } catch {
        Write-Host "  启动失败: $($_.Exception.Message)"
        continue
    }
    $cli = 'C:\Program Files\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe'
    if (Test-Path $cli) { $installed = $true; break }
}

if (-not $installed) {
    Write-Host '`n静默安装没成功，请手动运行：' -ForegroundColor Yellow
    Write-Host "  $setup" -ForegroundColor Yellow
    Write-Host '装完再执行:  .\tools\check_rdp.ps1' -ForegroundColor Yellow
    exit 2
}

Write-Host '`n安装成功，开始检测芯片读保护等级...' -ForegroundColor Green
& powershell.exe -NoProfile -ExecutionPolicy Bypass -File (Join-Path $PSScriptRoot 'check_rdp.ps1')
