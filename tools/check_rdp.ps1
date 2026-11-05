# 读保护(RDP)等级检测
#   前提：装好 STM32CubeProgrammer（用浏览器去 st.com 下），J-Link 已接好
#   用法：powershell -ExecutionPolicy Bypass -File .\tools\check_rdp.ps1

$ErrorActionPreference = 'Stop'

# 1. 找 STM32_Programmer_CLI.exe
$candidates = @(
    'C:\Program Files\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe',
    'C:\Program Files (x86)\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe',
    "$env:LOCALAPPDATA\Programs\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe"
)
$cli = $candidates | Where-Object { Test-Path $_ } | Select-Object -First 1
if (-not $cli) {
    $cli = (Get-ChildItem 'C:\Program Files','C:\Program Files (x86)' -Recurse -Depth 4 `
                -Filter 'STM32_Programmer_CLI.exe' -ErrorAction SilentlyContinue |
            Select-Object -First 1 -ExpandProperty FullName)
}
if (-not $cli) {
    Write-Host '没找到 STM32_Programmer_CLI.exe。' -ForegroundColor Red
    Write-Host '请先安装 STM32CubeProgrammer：https://www.st.com/en/development-tools/stm32cubeprog.html' -ForegroundColor Yellow
    exit 1
}
Write-Host "使用: $cli`n" -ForegroundColor Cyan

# 2. 连接（1 MHz，SWD）
Write-Host '=== 尝试连接 ===' -ForegroundColor Cyan
$out = & $cli -c port=SWD freq=1000 mode=UR 2>&1
$out | ForEach-Object { Write-Host $_ }

# 3. 读选项字节（能连上才有用）
Write-Host "`n=== 读选项字节（RDP 等级）===" -ForegroundColor Cyan
$ob = & $cli -c port=SWD freq=1000 mode=UR -ob displ 2>&1
$ob | ForEach-Object { Write-Host $_ }

# 4. 判定
Write-Host "`n=== 判定 ===" -ForegroundColor Cyan
$all = ($out + $ob) -join "`n"
if ($all -match 'RDP:\s*Level\s*0(\.5)?' -or $all -match 'RDP.*Level 0[^0-9]') {
    Write-Host 'RDP Level 0/0.5 —— 芯片没被锁，问题在别处（看 VCAP / 内核 / 焊接）' -ForegroundColor Green
} elseif ($all -match 'RDP:\s*Level\s*2' -or $all -match 'Level 2') {
    Write-Host 'RDP Level 2 —— 调试口被永久关闭，只能换芯片！' -ForegroundColor Red
} elseif ($all -match 'RDP:\s*Level\s*1' -or $all -match 'read.*protect|protected') {
    Write-Host 'RDP Level 1 —— 可救：执行下面的全片擦除即可解锁' -ForegroundColor Yellow
    Write-Host '    & "' + $cli + '" -c port=SWD freq=1000 mode=UR -e all' -ForegroundColor Yellow
} elseif ($all -match 'No STM32 device|Cannot connect|Error') {
    Write-Host '连不上 —— 若这里也报"无法连接"，说明不是锁，是芯片/硬件没起来' -ForegroundColor Red
} else {
    Write-Host '结果不明确，请把上面的原始输出发我' -ForegroundColor DarkYellow
}
