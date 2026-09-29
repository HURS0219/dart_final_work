<#
.SYNOPSIS
  推送本仓库到 GitHub; 自动走 443 端口(22 端口常被网络拒绝)。
.DESCRIPTION
  通过 git 的 core.sshCommand 指定 ssh 经 ssh.github.com:443 连接, 规避 SSH 22 被拒。
.EXAMPLE
  powershell -File Tools\scripts\git_push_443.ps1                 # 默认推 main + --tags
  powershell -File Tools\scripts\git_push_443.ps1 -Refs main,smc  # 指定 refs
#>
param(
    [string]$Remote = "origin",
    [string[]]$Refs = @("main", "--tags"),
    [string]$Key = "C:/Users/zhong/.ssh/id_rsa_github"
)

$root = (Resolve-Path (Join-Path $PSScriptRoot "..\..")).Path
$ssh = "ssh -i $Key -o IdentitiesOnly=yes -o StrictHostKeyChecking=accept-new -o HostName=ssh.github.com -p 443"

$gitArgs = @("-C", $root, "-c", "core.sshCommand=$ssh", "push", $Remote) + $Refs
Write-Host ("git push {0} {1}" -f $Remote, ($Refs -join " ")) -ForegroundColor Cyan
& git @gitArgs
