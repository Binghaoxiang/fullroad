# SWD 连接重试探针
#   用法：在 PowerShell 里运行，然后一直按板子的复位键 / 重新插拔 USB-C
#       .\tools\swd_probe.ps1                  # 默认 30 秒，1 MHz
#       .\tools\swd_probe.ps1 -Seconds 60 -Speed 500
#   目的：判断"复位瞬间能不能连上"，从而区分是芯片/固件状态问题还是纯硬件链路问题。

param(
    [int]$Seconds = 30,
    [int]$Speed   = 1000,
    [string]$Device = 'STM32F429IGTx'
)

$exe = 'C:\Program Files\SEGGER\JLink_V986\JLink.exe'
if (-not (Test-Path $exe)) {
    $exe = (Get-ChildItem 'C:\Program Files\SEGGER\JLink_*\JLink.exe' -ErrorAction SilentlyContinue |
            Sort-Object FullName -Descending | Select-Object -First 1).FullName
}
if (-not $exe) { Write-Host '找不到 JLink.exe，请先安装 SEGGER J-Link 软件' -ForegroundColor Red; exit 1 }

$script = Join-Path $env:TEMP 'swd_probe.jlink'
$deadline = (Get-Date).AddSeconds($Seconds)
$n = 0; $pass = 0

Write-Host ("探针开始：{0} 秒内反复尝试，速度 {1} kHz，器件 {2}" -f $Seconds, $Speed, $Device) -ForegroundColor Cyan
Write-Host '现在开始连按板子的复位键（或反复插拔 USB-C），看什么时候能连上' -ForegroundColor Yellow

while ((Get-Date) -lt $deadline) {
    $n++
    @("si SWD", "speed $Speed", "device $Device", "connect", "q") |
        Out-File $script -Encoding ascii
    $out = & $exe -NoGui 1 -CommanderScript $script 2>&1
    $t = (Get-Date).ToString('HH:mm:ss.fff')

    if ($out -match 'Cortex-M4 identified|Cortex-M\d identified') {
        $pass++
        Write-Host "[$t] 第 $n 次：连接成功 ✅" -ForegroundColor Green
    } elseif ($out -match 'DAP initialized successfully') {
        Write-Host "[$t] 第 $n 次：DAP 通，但 CPU 挂不上（内核/供电问题）" -ForegroundColor Yellow
    } elseif ($out -match 'Failed to initialize DAP') {
        Write-Host "[$t] 第 $n 次：DAP 无响应（物理层）" -ForegroundColor DarkYellow
    } else {
        Write-Host "[$t] 第 $n 次：无响应" -ForegroundColor DarkGray
    }
}

Write-Host ("`n共 {0} 次尝试，成功 {1} 次" -f $n, $pass) -ForegroundColor Cyan
Write-Host '判定：'
Write-Host '  · 按复位键时能连上 → 是板内固件/低功耗状态，先全片擦除'
Write-Host '  · 一次都连不上      → 查 VCAP1/VCAP2(2.2uF)、NRST、VDD 虚焊，或换板对比'
Write-Host '  · 只有 DAP 通       → 内核域没起来（VCAP 电容或芯片损坏）'
