$ErrorActionPreference = 'Stop'
$repo = Split-Path -Parent $PSScriptRoot
Set-Location -LiteralPath $repo
$Host.UI.RawUI.WindowTitle = 'HX8 + JY61P capture - started ' + (Get-Date -Format 'HH:mm:ss') + ' - LIVE / RECORDING / SAVED'
Write-Host 'Keep this window open. Send angles in VOFA, not here.' -ForegroundColor Cyan
Write-Host 'VOFA: r 1 = recording ON; r 0 = recording OFF. Recording is OFF after MCU restart.' -ForegroundColor Cyan
Write-Host 'A successful move must show: SAVED ... COMPLETE: True' -ForegroundColor Green
Write-Host 'NEW_DROPS > 0 or STREAM STALE means stop the test and check capture.' -ForegroundColor Yellow
& 'D:\app\miniconda\envs\python312\python.exe' -u .\tools\capture_hx8_jy60.py
Write-Host "Capture stopped (exit $LASTEXITCODE). Previous files are preserved." -ForegroundColor Yellow
