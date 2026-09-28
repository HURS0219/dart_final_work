<#
.SYNOPSIS
  一键编译仓库里所有 robot(app)。
.DESCRIPTION
  遍历 UserApp\robot\*\robot.cmake, 对每个 robot 调 make_one\build.ps1 编译;
  最后汇总成功/失败列表。便于 CI 或改动 common 代码后做全量回归。
.EXAMPLE
  powershell -File Tools\scripts\build_all.ps1 -Board GIMBAL_BOARD
#>
param(
    [string]$Board = "GIMBAL_BOARD",
    [string]$Build = "Debug"
)

$root = (Resolve-Path (Join-Path $PSScriptRoot "..\..")).Path
$buildScript = Join-Path $root "make_one\build.ps1"
$robotDir = Join-Path $root "UserApp\robot"

$robots = Get-ChildItem $robotDir -Directory |
    Where-Object { Test-Path (Join-Path $_.FullName "robot.cmake") } |
    ForEach-Object { $_.Name } | Sort-Object

Write-Host ("=== build_all: {0} robots on {1} ({2}) ===" -f $robots.Count, $Board, $Build) -ForegroundColor Cyan

$ok = @(); $fail = @()
foreach ($rb in $robots) {
    Write-Host "--- $rb ---" -ForegroundColor Yellow
    & powershell -NoProfile -ExecutionPolicy Bypass -File $buildScript -Robot $rb -Board $Board -Build $Build
    if ($LASTEXITCODE -eq 0) { $ok += $rb } else { $fail += $rb }
}

Write-Host "================= SUMMARY =================" -ForegroundColor Cyan
Write-Host ("OK   ({0}): {1}" -f $ok.Count, ($ok -join ", ")) -ForegroundColor Green
Write-Host ("FAIL ({0}): {1}" -f $fail.Count, ($fail -join ", ")) -ForegroundColor Red
if ($fail.Count -gt 0) { exit 1 }
