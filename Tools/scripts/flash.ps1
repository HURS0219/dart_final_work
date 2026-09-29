<#
.SYNOPSIS
  用 J-Link 烧录一个 hex 到目标(STM32)。
.EXAMPLE
  powershell -File Tools\scripts\flash.ps1 -Hex make_one\build_dart_final\control-2026.hex
#>
param(
    [Parameter(Mandatory = $true)][string]$Hex,
    [string]$Device = "STM32F407IG",
    [string]$JLink = "C:\Program Files\SEGGER\JLink_V966\JLink.exe"
)

$hexAbs = (Resolve-Path $Hex).Path -replace '\\', '/'
$lines = @(
    "si SWD",
    "speed 4000",
    "device $Device",
    "connect",
    "r",
    "h",
    "loadfile $hexAbs",
    "r",
    "g",
    "qc"
)
$tmp = Join-Path $env:TEMP "flash.jlink"
Set-Content -LiteralPath $tmp -Value $lines -Encoding ASCII
Write-Host ("flash {0} -> {1}" -f $hexAbs, $Device) -ForegroundColor Cyan
& $JLink -NoGui 1 -CommanderScript $tmp
