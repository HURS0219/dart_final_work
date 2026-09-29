<#
.SYNOPSIS
  复位并运行目标(STM32), 用 J-Link。
.EXAMPLE
  powershell -File Tools\scripts\reset.ps1
#>
param(
    [string]$Device = "STM32F407IG",
    [string]$JLink = "C:\Program Files\SEGGER\JLink_V966\JLink.exe"
)

$lines = @("si SWD", "speed 4000", "device $Device", "connect", "r", "g", "qc")
$tmp = Join-Path $env:TEMP "reset.jlink"
Set-Content -LiteralPath $tmp -Value $lines -Encoding ASCII
Write-Host ("reset {0}" -f $Device) -ForegroundColor Cyan
& $JLink -NoGui 1 -CommanderScript $tmp
