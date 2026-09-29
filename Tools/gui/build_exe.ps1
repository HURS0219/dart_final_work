# =====================================================================
# Tools/gui/build_exe.ps1 - package the GUI into a single exe (PyInstaller)
# Output: Tools\gui\dist\dart_test_gui.exe
# =====================================================================
param([switch]$Clean)

$ErrorActionPreference = "Stop"
$Here = $PSScriptRoot
Set-Location $Here

if ($Clean) {
    Remove-Item -Recurse -Force (Join-Path $Here "build"), (Join-Path $Here "dist"), (Join-Path $Here "dart_test_gui.spec") -ErrorAction SilentlyContinue
}

python -m PyInstaller --noconfirm --onefile --windowed `
    --name dart_test_gui `
    --paths $Here `
    main.py

Write-Host ("==> exe: " + (Join-Path $Here "dist\dart_test_gui.exe")) -ForegroundColor Green
