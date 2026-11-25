[CmdletBinding()]
param(
    [switch]$ListOnly,
    [switch]$IncludeResetTests,
    [switch]$StopOnUsbAccessDenied,
    [switch]$XpackOnly
)

# Read-only target diagnostics. No flash erase, programming or option-byte writes.
# Reset tests require DAPLink nRESET connected to the target NRST pin.
$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path -Parent $PSScriptRoot
$boardConfig = Join-Path $projectRoot 'daplink.cfg'
$legacyOpenOcd = 'D:\STM32_Env\OpenOCD-20231002-0.12.0\bin\openocd.exe'
$candidates = @($legacyOpenOcd)
$pathOpenOcd = Get-Command openocd.exe -ErrorAction SilentlyContinue
if ($pathOpenOcd) {
    $candidates += $pathOpenOcd.Source
}
$wingetPattern = Join-Path $env:LOCALAPPDATA 'Microsoft\WinGet\Packages\xpack-dev-tools.openocd-xpack_*\xpack-openocd-*\bin\openocd.exe'
$candidates += @(Get-ChildItem -Path $wingetPattern -ErrorAction SilentlyContinue | ForEach-Object FullName)
$executables = @($candidates | Where-Object { $_ -and (Test-Path -LiteralPath $_) } | Select-Object -Unique)
if ($XpackOnly) {
    $executables = @($executables | Where-Object { $_ -match 'xpack-openocd' })
}
if (-not $executables.Count) {
    throw 'No OpenOCD executable found.'
}
if (-not (Test-Path -LiteralPath $boardConfig)) {
    throw "Board configuration missing: $boardConfig"
}

$cases = @(
    [pscustomobject]@{ Name = 'auto-100khz'; Speed = 100; Backend = 'auto'; Reset = $false },
    [pscustomobject]@{ Name = 'hid-100khz'; Speed = 100; Backend = 'hid'; Reset = $false },
    [pscustomobject]@{ Name = 'auto-20khz'; Speed = 20; Backend = 'auto'; Reset = $false }
)
if ($IncludeResetTests) {
    $cases += [pscustomobject]@{ Name = 'under-reset-100khz'; Speed = 100; Backend = 'auto'; Reset = $true }
}

function Invoke-BoundedOpenOcd {
    param([string]$Executable, [string[]]$CommandArguments)

    # Pass data directly to the executable, without cmd.exe or PowerShell evaluation.
    $quotedArguments = foreach ($argument in $CommandArguments) {
        '"' + $argument.Replace('"', '\"') + '"'
    }
    $startInfo = New-Object System.Diagnostics.ProcessStartInfo
    $startInfo.FileName = $Executable
    $startInfo.Arguments = $quotedArguments -join ' '
    $startInfo.UseShellExecute = $false
    $startInfo.CreateNoWindow = $true
    $startInfo.RedirectStandardOutput = $true
    $startInfo.RedirectStandardError = $true
    $process = New-Object System.Diagnostics.Process
    $process.StartInfo = $startInfo
    try {
        [void]$process.Start()
        $stdout = $process.StandardOutput.ReadToEndAsync()
        $stderr = $process.StandardError.ReadToEndAsync()
        $timedOut = -not $process.WaitForExit(15000)
        if ($timedOut) {
            # Terminate only the process started for this diagnostic case.
            $process.Kill()
            $process.WaitForExit()
        }
        [pscustomobject]@{
            ExitCode = $process.ExitCode
            TimedOut = $timedOut
            Output = $stdout.Result + [Environment]::NewLine + $stderr.Result
        }
    }
    finally {
        $process.Dispose()
    }
}

function Get-DiagnosticCategory {
    param($Result)
    if ($Result.TimedOut) { return 'PROCESS_TIMEOUT' }
    if ($Result.Output -match 'Access denied|insufficient permissions') { return 'USB_ACCESS_DENIED' }
    if ($Result.Output -match '0x[eE]0042000:\s+[0-9a-fA-F]{8}' -and $Result.ExitCode -eq 0) {
        return 'TARGET_REGISTERS_READ'
    }
    if ($Result.Output -match 'cannot read IDR') { return 'SWD_DPIDR_UNREADABLE' }
    if ($Result.Output -match 'unable to find.*CMSIS-DAP|No CMSIS-DAP|no device found') {
        return 'PROBE_NOT_OPENED'
    }
    if ($Result.Output -match 'SWD DPIDR|Read DPIDR|Cortex-M3') { return 'TARGET_FOUND_READ_FAILED' }
    return 'OTHER_FAILURE'
}

function Get-OpenOcdScripts {
    param([string]$Executable)
    $packageRoot = Split-Path -Parent (Split-Path -Parent $Executable)
    $scriptCandidates = @(
        (Join-Path $packageRoot 'share\openocd\scripts'),
        (Join-Path $packageRoot 'openocd\scripts'),
        (Join-Path $packageRoot 'scripts')
    )
    foreach ($directory in $scriptCandidates) {
        if (Test-Path -LiteralPath (Join-Path $directory 'target\stm32f1x.cfg')) {
            return $directory
        }
    }
    return $null
}

if ($ListOnly) {
    foreach ($executable in $executables) {
        Write-Output "OpenOCD: $executable"
        $scripts = Get-OpenOcdScripts -Executable $executable
        if (-not $scripts) { throw "Script directory missing for $executable" }
        Write-Output "Scripts: $scripts"
        foreach ($case in $cases) { Write-Output "  $($case.Name)" }
    }
    return
}

$activeOpenOcd = @(Get-Process openocd -ErrorAction SilentlyContinue)
if ($activeOpenOcd.Count) {
    throw 'Another OpenOCD process is running. Stop its debug/download session, then retry.'
}

Write-Host 'Keep the controller powered and servo motor power disconnected.'
Write-Host 'Only target identification and memory reads will be attempted.'
$logFolder = Join-Path $projectRoot ('outputs\swd_diagnostics_' + (Get-Date -Format 'yyyyMMdd_HHmmss_fff'))
[void](New-Item -ItemType Directory -Path $logFolder)
$summaryPath = Join-Path $logFolder 'summary.txt'
$summary = New-Object 'System.Collections.Generic.List[string]'
$identified = $false
$stopTesting = $false
$toolIndex = 0

foreach ($executable in $executables) {
    $toolIndex++
    $scripts = Get-OpenOcdScripts -Executable $executable
    if (-not $scripts) {
        $summary.Add("SKIPPED: missing script directory for $executable")
        continue
    }
    foreach ($case in $cases) {
        $caseName = "tool$toolIndex-$($case.Name)"
        Write-Host "Testing $caseName"
        $commandArguments = @(
            '-s', $scripts, '-f', $boardConfig,
            '-c', 'tcl_port disabled', '-c', 'gdb_port disabled', '-c', 'telnet_port disabled',
            '-c', "adapter speed $($case.Speed)",
            '-c', "cmsis_dap_backend $($case.Backend)"
        )
        if ($case.Reset) {
            $commandArguments += @('-c', 'reset_config srst_only srst_nogate connect_assert_srst')
        }
        $commandArguments += @(
            '-c', 'init', '-c', 'targets',
            '-c', 'mdw 0xE0042000 1',
            '-c', 'mdw 0x40022010 1',
            '-c', 'mdw 0x08000000 2',
            '-c', 'shutdown'
        )
        $result = Invoke-BoundedOpenOcd -Executable $executable -CommandArguments $commandArguments
        $logPath = Join-Path $logFolder ($caseName + '.log')
        @("Executable: $executable", "Case: $caseName", $result.Output) |
            Set-Content -LiteralPath $logPath -Encoding UTF8
        $category = Get-DiagnosticCategory -Result $result
        $summary.Add("$caseName : $category ; exit=$($result.ExitCode) ; executable=$executable")
        Write-Host "  $category"
        if ($category -eq 'TARGET_REGISTERS_READ') {
            Write-Host $result.Output
            $identified = $true
            $stopTesting = $true
            break
        }
        if ($category -eq 'USB_ACCESS_DENIED' -and $StopOnUsbAccessDenied) {
            Write-Host 'USB access is denied in this execution context. Run this script in your own PowerShell.'
            $stopTesting = $true
            break
        }
    }
    if ($stopTesting) { break }
}

$summary.Add("Target register reads completed: $identified")
$summary | Set-Content -LiteralPath $summaryPath -Encoding UTF8
Write-Host ''
$summary | ForEach-Object { Write-Host $_ }
Write-Host "Logs: $logFolder"
if (-not $identified) {
    Write-Host 'The root cause is not yet confirmed. The log categories show the failing stage.'
}
