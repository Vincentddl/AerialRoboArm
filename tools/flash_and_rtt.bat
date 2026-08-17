@echo off
setlocal EnableExtensions

rem AerialRoboArm one-shot bring-up helper:
rem 1) find the current OpenOCD installation
rem 2) build the current debug firmware
rem 3) kill stale OpenOCD processes
rem 4) flash and verify with DAPLink
rem 5) start the persistent RTT server on 127.0.0.1:9090

for %%I in ("%~dp0..") do set "REPO=%%~fI"

set "OPENOCD="

rem Prefer the real xPack executable. The winget command alias located at
rem ...\Packages\openocd.exe can be present but invalid inside CLion.
for /d %%D in ("%LOCALAPPDATA%\Microsoft\WinGet\Packages\xpack-dev-tools.openocd-xpack_*") do (
    for /d %%V in ("%%~fD\xpack-openocd-*") do (
        if exist "%%~fV\bin\openocd.exe" if not defined OPENOCD set "OPENOCD=%%~fV\bin\openocd.exe"
    )
)

rem Fall back to a normal PATH installation only when xPack is absent.
if not defined OPENOCD (
    for /f "delims=" %%I in ('where openocd.exe 2^>nul') do if exist "%%~fI" if not defined OPENOCD set "OPENOCD=%%~fI"
)

set "BUILD_DIR=%REPO%\cmake-build-debug"
set "ELF=%REPO%\cmake-build-debug\FOC_DEMO2.elf"
set "DAPLINK_CFG=%REPO%\daplink.cfg"
set "RTT_CFG=%REPO%\daplink_rtt_attach.cfg"
set "ELF_TCL=%ELF:\=/%"

if not defined OPENOCD (
    echo [ERR] OpenOCD was not found in PATH or the winget xPack installation.
    echo       Install it with: winget install xpack-dev-tools.openocd-xpack
    exit /b 1
)

echo [1/4] Build firmware...
cmake --build "%BUILD_DIR%" --parallel
if errorlevel 1 (
    echo [ERR] Firmware build failed.
    exit /b 1
)

if not exist "%ELF%" (
    echo [ERR] ELF not found: "%ELF%"
    exit /b 1
)

echo [2/4] Stop stale OpenOCD processes...
powershell -NoProfile -ExecutionPolicy Bypass -Command "Get-Process openocd -ErrorAction SilentlyContinue | Stop-Process -Force"

echo [3/4] Flash and verify firmware with DAPLink...
"%OPENOCD%" ^
    -c "tcl_port disabled" ^
    -c "gdb_port disabled" ^
    -c "telnet_port disabled" ^
    -f "%DAPLINK_CFG%" ^
    -c "program {%ELF_TCL%} verify reset exit"

if errorlevel 1 (
    echo [ERR] Flash or verify failed.
    exit /b 1
)

echo [4/4] Start attach-only RTT server...
echo       VOFA+: TCP Client 127.0.0.1:9090, RawData/text.
echo       Keep this window open. Press Ctrl+C to stop RTT.

"%OPENOCD%" -f "%RTT_CFG%"
