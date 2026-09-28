<#
.SYNOPSIS
  通过 J-Link 直接向目标 RTT 下行缓冲(通道 0)注入一行命令。

.DESCRIPTION
  适用场景: 无法在 J-Link RTT Viewer 里手动打字(或不便)时, 用本脚本把命令"喂"给固件。
  原理: 解析 ELF 的 _acDownBuffer / _SEGGER_RTT 符号 -> 计算 aDown[0] 的 WrOff/RdOff ->
        用 J-Link 写内存(先写缓冲字节, 再清 RdOff, 最后写 WrOff 触发) -> 目标固件从 RTT 读到命令。
  说明: 每次重新编译后符号地址会变; 本脚本每次都**动态解析**, 故无需改地址。

.PARAMETER Cmd
  要发送的命令(不含换行; 脚本自动补 "\r\n")。最长 16 字节(RTT down buffer 默认 16B)。

.PARAMETER Elf
  目标固件 ELF 路径(如 make_one\build_servo_test\control-2026.elf)。

.PARAMETER MaxNumUpBuffers
  SEGGER_RTT_MAX_NUM_UP_BUFFERS(见 SEGGER_RTT_Conf.h), 决定 WrOff 在控制块内的偏移。
  默认 3(本仓库配置); 若改了 SEGGER_RTT_Conf.h 需同步。

.EXAMPLE
  powershell -File Tools\scripts\rtt_send.ps1 -Cmd "Z" -Elf make_one\build_servo_test\control-2026.elf
#>
param(
    [Parameter(Mandatory = $true)][string]$Cmd,
    [Parameter(Mandatory = $true)][string]$Elf,
    [string]$Nm = "D:\workp\tea\pack\Toolchain\arm_gnu_toolchain\bin\arm-none-eabi-nm.exe",
    [string]$JLink = "C:\Program Files\SEGGER\JLink_V966\JLink.exe",
    [string]$Device = "STM32F407IG",
    [int]$MaxNumUpBuffers = 3
)

function Get-ElfSymbol([string]$name) {
    $out = & $Nm $Elf 2>$null
    foreach ($line in $out) {
        $tok = ($line.Trim() -split '\s+')
        if ($tok.Length -ge 3 -and $tok[-1] -eq $name) {
            return [Convert]::ToUInt32($tok[0], 16)
        }
    }
    return $null
}

$buf = Get-ElfSymbol "_acDownBuffer"   # 下行缓冲数组(通道0)
$cb = Get-ElfSymbol "_SEGGER_RTT"      # RTT 控制块
if ($null -eq $buf -or $null -eq $cb) {
    Write-Error "符号解析失败: _acDownBuffer/_SEGGER_RTT 未找到 (检查 ELF: $Elf)"
    exit 1
}

# 控制块偏移: acID[16] + MaxNumUp(4) + MaxNumDown(4) = 24; 每个 up buffer 24B;
# down buffer 内 WrOff 偏移 12, RdOff = WrOff + 4
$wrOfs = 24 + $MaxNumUpBuffers * 24 + 12
$WROFF = $cb + $wrOfs
$RDOFF = $WROFF + 4
$BUF = $buf

$bytes = [System.Text.Encoding]::ASCII.GetBytes($Cmd + "`r`n")
if ($bytes.Length -gt 16) { Write-Error "命令过长(>16B, RTT down buffer 仅 16B): $Cmd"; exit 1 }
$words = [int][math]::Ceiling($bytes.Length / 4)
$padded = New-Object byte[] ($words * 4)
[Array]::Copy($bytes, $padded, $bytes.Length)

$lines = @("si SWD", "speed 4000", "device $Device", "connect", "h")
for ($i = 0; $i -lt $padded.Length; $i += 4) {
    $w = [BitConverter]::ToUInt32($padded, $i)
    $lines += ("w4 0x{0:X8} 0x{1:X8}" -f ($BUF + $i), $w)
}
$lines += ("w4 0x{0:X8} 0x00000000" -f $RDOFF)              # 清 RdOff
$lines += ("w4 0x{0:X8} 0x{1:X8}" -f $WROFF, $bytes.Length) # 置 WrOff 触发固件读取
$lines += "g"
$lines += "qc"

$tmp = Join-Path $env:TEMP "rtt_send.jlink"
Set-Content -LiteralPath $tmp -Value $lines -Encoding ASCII
& $JLink -NoGui 1 -CommanderScript $tmp | Out-Null
Write-Output ("RTT sent: {0}  (BUF=0x{1:X} WrOff=0x{2:X} RdOff=0x{3:X})" -f $Cmd, $BUF, $WROFF, $RDOFF)
