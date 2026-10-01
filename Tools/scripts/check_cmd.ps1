param(
    [string]$Cmd = "N,0,100",
    [string]$Elf = "D:\worka\dart_final_work\make_one\build_dart_launcher_final\control-2026.elf",
    [string]$JLink = "C:\Program Files\SEGGER\JLink_V966\JLink.exe"
)

# check_cmd.ps1 - 发送 RTT 命令并读回 cmd.c 诊断变量, 用于定位"命令是否真的进了大脑"
# 字段: [74C]m0_after [750]t0_after(f) [754]branch [755]set_motor [756]angle_deg [758]set_angle [75C]rx

function Get-JMem([string]$addr, [int]$len) {
    $scr = "si SWD`nspeed 4000`ndevice STM32F407IG`nconnect`nh`nmem $addr, $len`nqc"
    $f = Join-Path $env:TEMP "chk_cmd.jlink"
    Set-Content -LiteralPath $f -Value $scr -Encoding ASCII
    $out = & $JLink -NoGui 1 -CommanderScript $f 2>&1
    return (($out | Select-String -Pattern "^2000" | ForEach-Object { $_.Line.Trim() }) -join " | ")
}

Write-Host "=== 发送前 ===" -ForegroundColor Cyan
Write-Host ("  cmd diag  @0x2000074C: " + (Get-JMem "0x2000074C" 0x14))
Write-Host ("  hold_tgt  @0x20000784: " + (Get-JMem "0x20000784" 0x10))

Write-Host ""
Write-Host ("=== 发送 " + $Cmd + " ===") -ForegroundColor Yellow
& powershell -ExecutionPolicy Bypass -File "D:\worka\dart_final_work\Tools\scripts\rtt_send.ps1" -Cmd $Cmd -Elf $Elf 2>&1 | Select-Object -First 1

Start-Sleep -Milliseconds 900

Write-Host ""
Write-Host "=== 发送后 ===" -ForegroundColor Cyan
Write-Host ("  cmd diag  @0x2000074C: " + (Get-JMem "0x2000074C" 0x14))
Write-Host ("  hold_tgt  @0x20000784: " + (Get-JMem "0x20000784" 0x10))
