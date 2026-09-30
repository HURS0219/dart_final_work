# =====================================================================
# Tools/scripts/oneclick_flash.ps1  (ASCII only)
#
# One-shot: build a robot (optional) then J-Link flash its hex.
# Used by VS Code tasks (.vscode/tasks.json) and from the terminal.
#
# Usage:
#   powershell -File Tools\scripts\oneclick_flash.ps1 -Robot dart_final
#   powershell -File Tools\scripts\oneclick_flash.ps1 -Robot dart_final_test_app
#   powershell -File Tools\scripts\oneclick_flash.ps1 -Robot servo_test -Clean
#   powershell -File Tools\scripts\oneclick_flash.ps1 -Robot dart_final -NoBuild
#   powershell -File Tools\scripts\oneclick_flash.ps1 -Robot dart_final -Device STM32H723ZG
# =====================================================================
param(
    [string]$Robot = "dart_final",
    [ValidateSet("GIMBAL_BOARD", "CHASSIS_BOARD", "ONE_BOARD", "H743_BOARD", "DART_F405_BOARD")][string]$Board = "GIMBAL_BOARD",
    [switch]$NoBuild,
    [switch]$NoFlash,
    [switch]$Clean,
    [string]$Device = ""
)

$ErrorActionPreference = "Stop"

# Tools\scripts -> repo root
$Here = $PSScriptRoot
$Root = Split-Path -Parent (Split-Path -Parent $Here)

$BuildScript = Join-Path $Root "make_one\build.ps1"
$FlashScript = Join-Path $Root "Tools\scripts\flash.ps1"
$Hex         = Join-Path $Root ("make_one\build_" + $Robot + "\control-2026.hex")

# device default by board (override with -Device)
if ([string]::IsNullOrEmpty($Device)) {
    switch ($Board) {
        "CHASSIS_BOARD" { $Device = "STM32H723ZG" }
        "H743_BOARD"    { $Device = "STM32H743ZI" }
        "DART_F405_BOARD" { $Device = "STM32F405RG" }
        default         { $Device = "STM32F407IG" }
    }
}

Write-Host ""
Write-Host ("################  ROBOT = " + $Robot + "   BOARD = " + $Board + "  ################") -ForegroundColor Yellow

# 1) build (unless -NoBuild)
if (-not $NoBuild) {
    if (-not (Test-Path -LiteralPath $BuildScript)) { Write-Error ("build.ps1 not found: " + $BuildScript); exit 1 }
    $bargs = @("-NoProfile", "-ExecutionPolicy", "Bypass", "-File", $BuildScript, "-Robot", $Robot, "-Board", $Board)
    if ($Clean) { $bargs += "-Clean" }
    Write-Host ("==> Build  robot=" + $Robot + "  board=" + $Board) -ForegroundColor Cyan
    & powershell @bargs
    if ($LASTEXITCODE -ne 0) { Write-Error "build failed"; exit 1 }
}

# 2) flash
if ($NoFlash) {
    Write-Host ("==> Build only (-NoFlash): " + $Hex) -ForegroundColor Green
    exit 0
}
if (-not (Test-Path -LiteralPath $Hex)) {
    Write-Error ("hex not found: " + $Hex + "  (build it first, or drop -NoBuild)")
    exit 1
}
Write-Host ("==> Flash  robot=" + $Robot + "  device=" + $Device) -ForegroundColor Cyan
$hexItem = Get-Item -LiteralPath $Hex
Write-Host ("    hex: " + $Hex + "   (" + $hexItem.LastWriteTime + ")") -ForegroundColor DarkGray
& powershell -NoProfile -ExecutionPolicy Bypass -File $FlashScript -Hex $Hex -Device $Device
if ($LASTEXITCODE -ne 0) { Write-Error "flash failed"; exit 1 }

Write-Host ("==> OK: " + $Robot + " flashed.") -ForegroundColor Green
