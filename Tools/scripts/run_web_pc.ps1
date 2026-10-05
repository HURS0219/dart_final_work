# =====================================================================
# Tools/scripts/run_web_pc.ps1  (ASCII only)
#
# Launch the PC web tool (J-Link RTT, NO ESP) and auto-open the browser.
# Note: J-Link is single-session -> do NOT flash while this is running.
#
# Usage:
#   powershell -File Tools\scripts\run_web_pc.ps1 -Board GIMBAL_BOARD
# =====================================================================
param(
    [ValidateSet("GIMBAL_BOARD", "CHASSIS_BOARD", "ONE_BOARD", "H743_BOARD", "DART_F405_BOARD")]
    [string]$Board = "GIMBAL_BOARD",
    [int]$Port = 8000
)

$ErrorActionPreference = "Stop"

switch ($Board) {
    "CHASSIS_BOARD"   { $dev = "STM32H723ZG" }
    "ONE_BOARD"       { $dev = "STM32H723ZG" }
    "H743_BOARD"      { $dev = "STM32H743ZI" }
    "DART_F405_BOARD" { $dev = "STM32F405RG" }
    default           { $dev = "STM32F407IG" }
}

$Here = $PSScriptRoot
$Root = Split-Path -Parent (Split-Path -Parent $Here)   # Tools\scripts -> repo root
$WebDir = Join-Path $Root "Tools\dart_launcher_web_pc"
if (-not (Test-Path -LiteralPath $WebDir)) {
    Write-Error ("PC web tool not found: " + $WebDir)
    exit 1
}

$env:DART_RTT_DEVICE = $dev
$env:DART_WEB_PORT = "$Port"
$url = "http://127.0.0.1:$Port"

Write-Host ("PC web : device=" + $dev + "  url=" + $url) -ForegroundColor Cyan
Write-Host ("dir    : " + $WebDir) -ForegroundColor DarkGray
Write-Host "J-Link is single-session: close this (Ctrl+C) before flashing." -ForegroundColor Yellow

# Background job: wait until the server answers, then open the browser.
$opener = Start-Job -ScriptBlock {
    param($u)
    for ($i = 0; $i -lt 80; $i++) {
        Start-Sleep -Milliseconds 300
        try {
            $null = Invoke-WebRequest ($u + "/state") -UseBasicParsing -TimeoutSec 1
            Start-Process $u
            return
        } catch { }
    }
} -ArgumentList $url

Set-Location $WebDir
& python -u main.py

Stop-Job $opener -ErrorAction SilentlyContinue
Remove-Job $opener -Force -ErrorAction SilentlyContinue
