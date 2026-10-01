param(
    [int]$Slot = 0,
    [int[]]$Degs = @(300, 800, 1500, 1800),
    [int]$SettleMs = 2500,
    [string]$Elf = "D:\worka\dart_final_work\make_one\build_dart_launcher_final\control-2026.elf",
    [string]$JLink = "C:\Program Files\SEGGER\JLink_V966\JLink.exe",
    [string]$Log = "$env:TEMP\spring_step.log"
)

# spring_step.ps1 - 拉簧分步加大角度并记录每步的到位情况
# 用法: powershell -File Tools\scripts\spring_step.ps1 -Slot 0 -Degs 300,800,1500,1800

foreach ($d in $Degs) {
    Write-Host ""
    Write-Host ("=== N,{0},{1}  (目标 {1} 度) ===" -f $Slot, $d) -ForegroundColor Yellow
    & powershell -ExecutionPolicy Bypass -File "D:\worka\dart_final_work\Tools\scripts\rtt_send.ps1" `
        -Cmd ("N,{0},{1}" -f $Slot, $d) -Elf $Elf 2>&1 | Select-Object -First 1

    Remove-Item $Log -ErrorAction SilentlyContinue
    $p = Start-Process -FilePath (Join-Path (Split-Path $JLink) "JLinkRTTLogger.exe") `
        -ArgumentList "-Device", "STM32F407IG", "-If", "SWD", "-Speed", "4000", "-RTTChannel", "0", $Log `
        -PassThru -WindowStyle Hidden
    Start-Sleep -Milliseconds $SettleMs
    if (!$p.HasExited) { $p.Kill() }
    Start-Sleep -Milliseconds 300

    $lines = Get-Content $Log -ErrorAction SilentlyContinue | Select-String -Pattern "lch\] A deg="
    if ($lines) {
        $last = $lines | Select-Object -Last 1
        Write-Host ("  -> " + ($last.Line -replace '\x1b\[[0-9;]*m', '').Trim())
    } else {
        Write-Host "  -> (无遥测)" -ForegroundColor Red
    }
}
