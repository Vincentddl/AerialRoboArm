<#
.SYNOPSIS
  Configure an HC-13 module from a Windows PC serial port.

.DESCRIPTION
  Put HC-13 KEY low before running this script so the module is in AT mode.
  The script probes common baud rates, sends AT commands, and configures the
  module for the project default: S7 air rate, C043 channel, 230400 baud.

.EXAMPLE
  powershell -ExecutionPolicy Bypass -File tools\hc13_config.ps1 -Port COM8

.EXAMPLE
  powershell -ExecutionPolicy Bypass -File tools\hc13_config.ps1 -Port COM8 -QueryOnly
#>

param(
    [string]$Port,
    [int[]]$ProbeBaud = @(9600, 115200, 230400),
    [ValidatePattern('^S[1-7]$')]
    [string]$AirRate = 'S7',
    [ValidatePattern('^\d{3}$')]
    [string]$Channel = '043',
    [ValidateSet(1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200, 230400)]
    [int]$TargetBaud = 230400,
    [switch]$SkipChannel,
    [switch]$QueryOnly
)

$ErrorActionPreference = 'Stop'

function Show-Ports {
    Write-Host 'Available serial ports:'
    [System.IO.Ports.SerialPort]::GetPortNames() |
        Sort-Object |
        ForEach-Object { Write-Host "  $_" }
}

function Open-Hc13Port {
    param(
        [string]$Name,
        [int]$Baud
    )

    $sp = [System.IO.Ports.SerialPort]::new(
        $Name,
        $Baud,
        [System.IO.Ports.Parity]::None,
        8,
        [System.IO.Ports.StopBits]::One
    )
    $sp.Handshake = [System.IO.Ports.Handshake]::None
    $sp.ReadTimeout = 120
    $sp.WriteTimeout = 500
    $sp.NewLine = "`r`n"
    $sp.DtrEnable = $false
    $sp.RtsEnable = $false
    $sp.Open()
    Start-Sleep -Milliseconds 80
    $sp.DiscardInBuffer()
    $sp.DiscardOutBuffer()
    return $sp
}

function Read-Hc13Response {
    param(
        [System.IO.Ports.SerialPort]$Serial,
        [int]$TimeoutMs = 900
    )

    $deadline = [Environment]::TickCount + $TimeoutMs
    $text = ''
    while ([Environment]::TickCount -lt $deadline) {
        try {
            $chunk = $Serial.ReadExisting()
            if ($chunk.Length -gt 0) {
                $text += $chunk
                if ($text -match "(\r|\n)") {
                    Start-Sleep -Milliseconds 40
                    $text += $Serial.ReadExisting()
                    break
                }
            }
        } catch [System.TimeoutException] {
        }
        Start-Sleep -Milliseconds 20
    }
    return $text.Trim()
}

function Send-Hc13Command {
    param(
        [System.IO.Ports.SerialPort]$Serial,
        [string]$Command,
        [string]$Expect = ''
    )

    $endings = @('', "`r`n", "`r")
    $resp = ''

    foreach ($ending in $endings) {
        $Serial.DiscardInBuffer()
        $Serial.Write($Command + $ending)
        $resp = Read-Hc13Response -Serial $Serial
        if ($resp.Length -gt 0) {
            break
        }
        Start-Sleep -Milliseconds 80
    }

    $shown = if ($resp.Length -gt 0) { $resp } else { '<no response>' }
    Write-Host ("{0,-12} -> {1}" -f $Command, $shown)

    if (($Expect.Length -gt 0) -and ($resp -notlike "*$Expect*")) {
        throw "Command '$Command' expected '$Expect', got '$shown'."
    }
    return $resp
}

function Find-Hc13Baud {
    foreach ($baud in $ProbeBaud) {
        $sp = $null
        try {
            Write-Host "Probing $Port at $baud baud..."
            $sp = Open-Hc13Port -Name $Port -Baud $baud
            $resp = Send-Hc13Command -Serial $sp -Command 'AT'
            if ($resp -like '*OK*') {
                Write-Host "AT mode found at $baud baud."
                return @{
                    Serial = $sp
                    Baud = $baud
                }
            }
        } catch {
            Write-Host "  no valid AT response at $baud"
        } finally {
            if ($sp -and $sp.IsOpen) {
                if ($resp -notlike '*OK*') {
                    $sp.Close()
                }
            }
        }
    }

    throw "Cannot find HC-13 AT mode. Check COM port, wiring, and KEY low."
}

if ([string]::IsNullOrWhiteSpace($Port)) {
    Show-Ports
    Write-Host ''
    Write-Host 'Usage: powershell -ExecutionPolicy Bypass -File tools\hc13_config.ps1 -Port COMx'
    exit 1
}

Write-Host 'HC-13 configuration helper'
Write-Host 'Before running: connect USB-TTL GND/TX/RX, pull HC-13 KEY low, then power the module.'
Write-Host ''

$found = Find-Hc13Baud
$serial = $found.Serial
$currentBaud = $found.Baud

try {
    Write-Host ''
    Write-Host "Current AT baud: $currentBaud"
    Write-Host ''

    if ($QueryOnly) {
        Send-Hc13Command -Serial $serial -Command 'AT+RB'
        Send-Hc13Command -Serial $serial -Command 'AT+RS'
        Send-Hc13Command -Serial $serial -Command 'AT+RC'
        Send-Hc13Command -Serial $serial -Command 'AT+RP'
        Send-Hc13Command -Serial $serial -Command 'AT+RU'
        Send-Hc13Command -Serial $serial -Command 'AT+V'
        return
    }

    Send-Hc13Command -Serial $serial -Command "AT+$AirRate" -Expect "OK+$AirRate"
    if (-not $SkipChannel) {
        Send-Hc13Command -Serial $serial -Command "AT+C$Channel" -Expect "OK+C$Channel"
    }
    Send-Hc13Command -Serial $serial -Command "AT+B$TargetBaud" -Expect "OK+B$TargetBaud"

    Write-Host ''
    Write-Host 'Configuration commands accepted.'
    Write-Host 'Now release KEY high or disconnect it from GND, wait at least 30 ms, then use transparent mode.'
    Write-Host "Project target: HC13 $AirRate, channel C$Channel, UART $TargetBaud 8N1."
} finally {
    if ($serial -and $serial.IsOpen) {
        $serial.Close()
    }
}
