# =====================================================================
# make_one/build.ps1 - configure + build one robot (ASCII only!)
#
# Usage:
#   powershell -ExecutionPolicy Bypass -File make_one\build.ps1 -Robot dart_fc -Board GIMBAL_BOARD
#   powershell -ExecutionPolicy Bypass -File make_one\build.ps1 -Robot infantry_six_wheel_example2 -Board GIMBAL_BOARD
#   powershell -ExecutionPolicy Bypass -File make_one\build.ps1 -Robot dart_fc -Board GIMBAL_BOARD -Clean
#
# Notes:
#   - Build dir is always make_one\build_<Robot>  (keeps repo root clean).
#   - BOARD_TYPE -> MCU_TYPE is derived here as a fallback so that older
#     examples (whose robot.cmake lacks the mapping) still configure.
#     If robot.cmake sets MCU_TYPE itself, that value wins.
# =====================================================================
param(
    [Parameter(Mandatory = $true)][string]$Robot,
    [ValidateSet("GIMBAL_BOARD", "CHASSIS_BOARD", "ONE_BOARD", "H743_BOARD")][string]$Board = "GIMBAL_BOARD",
    [ValidateSet("Debug", "Release", "RelWithDebInfo", "MinSizeRel")][string]$Build = "Debug",
    [switch]$Clean
)

$ErrorActionPreference = "Stop"

# directory of this script (make_one); its parent is the repo root
if ($PSScriptRoot) {
    $Here = $PSScriptRoot
} else {
    $Here = Split-Path -Parent $MyInvocation.MyCommand.Path
}
$Root = Split-Path -Parent $Here
$BuildDir = Join-Path $Here ("build_" + $Robot)

# BOARD_TYPE -> MCU_TYPE fallback
switch ($Board) {
    "GIMBAL_BOARD"  { $Mcu = "stm32-f4" }
    "CHASSIS_BOARD" { $Mcu = "stm32-h7" }
    "ONE_BOARD"     { $Mcu = "stm32-f4" }
    "H743_BOARD"    { $Mcu = "stm32-h743" }
    default         { $Mcu = "stm32-f4" }
}

$RobotCmake = Join-Path $Root ("UserApp\robot\" + $Robot + "\robot.cmake")
if (-not (Test-Path -LiteralPath $RobotCmake)) {
    Write-Error ("robot not found: " + $RobotCmake)
    exit 1
}

if ($Clean -and (Test-Path -LiteralPath $BuildDir)) {
    Write-Host ("clean " + $BuildDir) -ForegroundColor Yellow
    Remove-Item -LiteralPath $BuildDir -Recurse -Force
}

Write-Host ("==> Configure  robot=" + $Robot + " board=" + $Board + " mcu=" + $Mcu + " build=" + $Build) -ForegroundColor Cyan
cmake -G Ninja -S $Root -B $BuildDir "-DROBOT_TYPE=$Robot" "-DBOARD_TYPE=$Board" "-DMCU_TYPE=$Mcu" "-DCMAKE_BUILD_TYPE=$Build"
if ($LASTEXITCODE -ne 0) { Write-Error "CMake configure failed"; exit 1 }

Write-Host "==> Build" -ForegroundColor Cyan
cmake --build $BuildDir
if ($LASTEXITCODE -ne 0) { Write-Error "Build failed"; exit 1 }

$Hex = Join-Path $BuildDir "control-2026.hex"
Write-Host ("==> Done: " + $Hex) -ForegroundColor Green
