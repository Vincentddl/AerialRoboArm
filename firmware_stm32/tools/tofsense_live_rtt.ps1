param(
    [string]$Server = "127.0.0.1",
    [int]$Port = 9090,
    [int]$PeriodMs = 200
)

$ErrorActionPreference = "Stop"
$repo = (Resolve-Path (Join-Path $PSScriptRoot "..")).Path
$ownedOpenOcd = $null
$client = $null

function Test-RttPort {
    try {
        $probe = [System.Net.Sockets.TcpClient]::new()
        $async = $probe.BeginConnect($Server, $Port, $null, $null)
        $ready = $async.AsyncWaitHandle.WaitOne(150)
        if ($ready) {
            $probe.EndConnect($async)
        }
        $probe.Close()
        return $ready
    }
    catch {
        return $false
    }
}

try {
    if (-not (Test-RttPort)) {
        $openocd = Get-ChildItem -Path "$env:LOCALAPPDATA\Microsoft\WinGet\Packages\xpack-dev-tools.openocd-xpack_*\xpack-openocd-*\bin\openocd.exe" -ErrorAction Stop |
            Select-Object -First 1 -ExpandProperty FullName
        $cfg = Join-Path $repo "daplink_rtt_attach.cfg"
        $ownedOpenOcd = Start-Process -FilePath $openocd `
            -ArgumentList @("-f", ('"' + $cfg + '"')) `
            -WorkingDirectory $repo `
            -WindowStyle Hidden `
            -PassThru

        $deadline = [DateTime]::UtcNow.AddSeconds(10)
        while ((-not (Test-RttPort)) -and ([DateTime]::UtcNow -lt $deadline)) {
            Start-Sleep -Milliseconds 200
        }
        if (-not (Test-RttPort)) {
            throw "RTT server did not open $Server`:$Port. Check DAPLink/OpenOCD."
        }
    }

    $client = [System.Net.Sockets.TcpClient]::new($Server, $Port)
    $stream = $client.GetStream()
    $request = [System.Text.Encoding]::ASCII.GetBytes("i")
    $buffer = New-Object byte[] 8192
    $textBuffer = ""

    Clear-Host
    Write-Host "TOFSense-F2 P live distance (I2C 0x08)" -ForegroundColor Cyan
    Write-Host "Power: current PCB 3.3 V undervoltage test" -ForegroundColor Yellow
    Write-Host "Refresh: $PeriodMs ms   Press Ctrl+C to stop" -ForegroundColor DarkGray
    Write-Host ""

    while ($true) {
        $stream.Write($request, 0, $request.Length)
        $deadline = [DateTime]::UtcNow.AddMilliseconds([Math]::Max($PeriodMs, 100))

        while ([DateTime]::UtcNow -lt $deadline) {
            while ($stream.DataAvailable) {
                $count = $stream.Read($buffer, 0, $buffer.Length)
                if ($count -le 0) {
                    throw "RTT connection closed."
                }
                $textBuffer += [System.Text.Encoding]::UTF8.GetString($buffer, 0, $count)
            }

            while ($textBuffer.Contains("`n")) {
                $split = $textBuffer.IndexOf("`n")
                $line = $textBuffer.Substring(0, $split).Trim("`r")
                $textBuffer = $textBuffer.Substring($split + 1)

                if ($line -match '\[F2P\] OK.*distance=(-?\d+) mm valid=(\d+) signal=(\d+)') {
                    $distance = [int]$Matches[1]
                    $valid = [int]$Matches[2]
                    $signal = [int]$Matches[3]
                    $stamp = Get-Date -Format "HH:mm:ss.fff"
                    $color = if ($valid -eq 1) { "Green" } else { "Red" }
                    Write-Host ("{0}  distance={1,6} mm  valid={2}  signal={3}" -f $stamp, $distance, $valid, $signal) -ForegroundColor $color
                }
                elseif ($line -match '\[F2P\] (NACK|ACK.*failed)') {
                    Write-Host ((Get-Date -Format "HH:mm:ss.fff") + "  " + $line) -ForegroundColor Red
                }
            }
            Start-Sleep -Milliseconds 5
        }
    }
}
finally {
    if ($null -ne $client) {
        $client.Close()
    }
    if (($null -ne $ownedOpenOcd) -and (-not $ownedOpenOcd.HasExited)) {
        Stop-Process -Id $ownedOpenOcd.Id -Force -ErrorAction SilentlyContinue
    }
}
