@echo off
setlocal
for %%I in ("%~dp0..") do set "REPO=%%~fI"
set "OPENOCD="
for /d %%D in ("%LOCALAPPDATA%\Microsoft\WinGet\Packages\xpack-dev-tools.openocd-xpack_*") do (
    for /d %%V in ("%%~fD\xpack-openocd-*") do (
        if exist "%%~fV\bin\openocd.exe" if not defined OPENOCD set "OPENOCD=%%~fV\bin\openocd.exe"
    )
)
if not defined OPENOCD for /f "delims=" %%I in ('where openocd.exe 2^>nul') do if exist "%%~fI" if not defined OPENOCD set "OPENOCD=%%~fI"
if not defined OPENOCD (
    echo [ERROR] openocd.exe not found.
    pause
    exit /b 1
)
powershell -NoProfile -ExecutionPolicy Bypass -File "%REPO%\tools\build_hc13_host.ps1"
if errorlevel 1 (
    echo [ERROR] Firmware build failed.
    pause
    exit /b 1
)
copy /y "%REPO%\cmake-build-hc13-host\FOC_DEMO2.elf" "%REPO%\firmware_latest\" >nul
copy /y "%REPO%\cmake-build-hc13-host\FOC_DEMO2.hex" "%REPO%\firmware_latest\" >nul
copy /y "%REPO%\cmake-build-hc13-host\FOC_DEMO2.bin" "%REPO%\firmware_latest\" >nul
set "ELF=%REPO%\cmake-build-hc13-host\FOC_DEMO2.elf"
set "ELF_TCL=%ELF:\=/%"
"%OPENOCD%" -c "tcl port disabled" -c "gdb port disabled" -c "telnet port disabled" -f "%REPO%\daplink.cfg" -c "program {%ELF_TCL%} verify reset exit"
if errorlevel 1 (
    echo [ERROR] DAPLink flash failed. Check SWD/USB connection and retry.
    pause
    exit /b 1
)
echo [OK] HC13 V2 + VISION_FOLLOW firmware flashed and verified successfully!
pause
