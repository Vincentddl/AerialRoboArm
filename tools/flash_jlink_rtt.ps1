[CmdletBinding()]
param(
    [string]$BuildDirectory = "cmake-build-debug",
    [ValidateRange(50, 4000)]
    [int]$AdapterSpeedKHz = 500,
    [switch]$SkipBuild
)

Set-StrictMode -Version Latest
$ErrorActionPreference = "Stop"

$projectRoot = Split-Path -Parent $PSScriptRoot
$flashScript = Join-Path $PSScriptRoot "flash_jlink.ps1"
$rttConfig = Join-Path $projectRoot "jlink_rtt_attach.cfg"

function Find-OpenOcd {
    $command = Get-Command openocd -ErrorAction SilentlyContinue
    if ($null -ne $command) {
        return $command.Source
    }

    $wingetRoot = Join-Path $env:LOCALAPPDATA "Microsoft\WinGet\Packages"
    if (Test-Path -LiteralPath $wingetRoot) {
        $candidate = Get-ChildItem -LiteralPath $wingetRoot `
            -Filter "openocd.exe" -File -Recurse -ErrorAction SilentlyContinue |
            Where-Object { $_.FullName -match "openocd-xpack" } |
            Select-Object -First 1 -ExpandProperty FullName
        if ($null -ne $candidate) {
            return $candidate
        }
    }

    throw "OpenOCD was not found. Install xpack-dev-tools.openocd-xpack with winget."
}

$portOwner = Get-NetTCPConnection -State Listen -LocalPort 9090 -ErrorAction SilentlyContinue
if ($null -ne $portOwner) {
    throw "TCP port 9090 is already in use. Stop the previous OpenOCD/RTT session first."
}

if (-not (Test-Path -LiteralPath $flashScript)) {
    throw "Missing flash script: $flashScript"
}
if (-not (Test-Path -LiteralPath $rttConfig)) {
    throw "Missing RTT configuration: $rttConfig"
}

$flashArguments = @{
    BuildDirectory = $BuildDirectory
    AdapterSpeedKHz = $AdapterSpeedKHz
}
if ($SkipBuild) {
    $flashArguments.SkipBuild = $true
}

# Build, program, verify, and reset the target first.
& $flashScript @flashArguments

$openocd = Find-OpenOcd
Write-Host "RTT server starting on 127.0.0.1:9090 ..." -ForegroundColor Cyan
Write-Host "Connect PuTTY/MobaXterm/VOFA+ to TCP 127.0.0.1:9090." -ForegroundColor Cyan
Write-Host "Stop this session with Ctrl+C or CLion's red Stop button." -ForegroundColor Yellow

# This process intentionally stays in the foreground to keep RTT alive.
& $openocd -f $rttConfig

if ($LASTEXITCODE -ne 0) {
    throw "The J-Link RTT server stopped with exit code $LASTEXITCODE."
}
