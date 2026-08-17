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
$buildPath = Join-Path $projectRoot $BuildDirectory
$firmwarePath = Join-Path $buildPath "FOC_DEMO2.hex"
$jlinkConfig = Join-Path $projectRoot "jlink.cfg"

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

if (-not (Test-Path -LiteralPath $jlinkConfig)) {
    throw "Missing J-Link configuration: $jlinkConfig"
}

if (-not $SkipBuild) {
    Write-Host "[1/2] Building $BuildDirectory ..." -ForegroundColor Cyan
    & cmake --build $buildPath --parallel
    if ($LASTEXITCODE -ne 0) {
        throw "Firmware build failed with exit code $LASTEXITCODE."
    }
}

if (-not (Test-Path -LiteralPath $firmwarePath)) {
    throw "Firmware image was not found: $firmwarePath"
}

$openocd = Find-OpenOcd
$openocdFirmwarePath = $firmwarePath.Replace("\", "/")

Write-Host "[2/2] Programming and verifying FOC_DEMO2.hex ..." -ForegroundColor Cyan
Write-Host "Probe: J-Link / SWD / $AdapterSpeedKHz kHz"

& $openocd `
    -f $jlinkConfig `
    -c "adapter speed $AdapterSpeedKHz" `
    -c "program {$openocdFirmwarePath} verify reset exit"

if ($LASTEXITCODE -ne 0) {
    throw "OpenOCD programming failed with exit code $LASTEXITCODE. Check SWDIO, SWCLK, GND, VTref and NRST."
}

Write-Host "J-Link programming and verification completed successfully." -ForegroundColor Green
