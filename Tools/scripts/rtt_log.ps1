<#
.SYNOPSIS
  用 J-Link RTT Logger 把 RTT 上行(通道0)日志保存到文件(供 MATLAB 处理)。
.DESCRIPTION
  dart_final_test_app 每秒经 RTT 输出状态表; 本脚本启动 SEGGER JLinkRTTLogger 把通道0
  写入指定文件。运行期间请保持 J-Link 连接; 结束用 Ctrl+C。
.EXAMPLE
  powershell -File Tools\scripts\rtt_log.ps1 -Out Debug\dart_test_log.txt
#>
param(
    [string]$Out = "Debug\dart_test_log.txt",
    [string]$Device = "STM32F407IG",
    [int]$Channel = 0,
    [int]$Speed = 4000,
    [string]$RttLogger = "C:\Program Files\SEGGER\JLink_V966\JLinkRTTLogger.exe"
)

$outAbs = Join-Path (Resolve-Path "$PSScriptRoot\..\..").Path $Out
$outDir = Split-Path -Parent $outAbs
if (-not (Test-Path -LiteralPath $outDir)) { New-Item -ItemType Directory -Path $outDir | Out-Null }

Write-Host ("RTT log -> {0}  (device={1}, ch={2}); Ctrl+C 结束" -f $outAbs, $Device, $Channel) -ForegroundColor Cyan
& $RttLogger "-Device" $Device "-If" "SWD" "-Speed" $Speed "-RTTChannel" $Channel $outAbs
