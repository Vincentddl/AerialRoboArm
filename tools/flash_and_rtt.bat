@echo off
setlocal EnableExtensions

rem AerialRoboArm one-shot bring-up helper:
rem 1) kill stale OpenOCD processes
rem 2) flash and verify the current debug ELF
rem 3) start the persistent RTT server on 127.0.0.1:9090

for %%I in ("%~dp0..") do set "REPO=%%~fI"

set "OPENOCD=D:\STM32_Env\OpenOCD-20231002-0.12.0\bin\openocd.exe"
set "OPENOCD_SCRIPTS=D:\STM32_Env\OpenOCD-20231002-0.12.0\share\openocd\scripts"
set "ELF=%REPO%\cmake-build-debug\FOC_DEMO2.elf"
set "DAPLINK_CFG=%REPO%\daplink.cfg"
set "RTT_CFG=%REPO%\daplink_rtt_attach.cfg"
set "ELF_TCL=%ELF:\=/%"

echo [1/3] Stop stale OpenOCD processes...
powershell -NoProfile -ExecutionPolicy Bypass -Command "Get-Process openocd -ErrorAction SilentlyContinue | Stop-Process -Force"

if not exist "%OPENOCD%" (
    echo [ERR] OpenOCD not found: "%OPENOCD%"
    exit /b 1
)

if not exist "%ELF%" (
    echo [ERR] ELF not found: "%ELF%"
    echo       Build the project first, then run this script again.
    exit /b 1
)

echo [2/3] Flash and verify firmware...
"%OPENOCD%" -s "%OPENOCD_SCRIPTS%" ^
    -c "tcl_port disabled" ^
    -c "gdb_port disabled" ^
    -c "telnet_port disabled" ^
    -f "%DAPLINK_CFG%" ^
    -c "program {%ELF_TCL%} verify reset exit"

if errorlevel 1 (
    echo [ERR] Flash or verify failed.
    exit /b 1
)

echo [3/3] Start attach-only RTT server...
echo       VOFA+: TCP Client 127.0.0.1:9090, RawData/text.
echo       Keep this window open. Press Ctrl+C to stop RTT.

"%OPENOCD%" -s "%OPENOCD_SCRIPTS%" -f "%RTT_CFG%"
