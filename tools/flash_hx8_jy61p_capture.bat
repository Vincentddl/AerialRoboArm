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
if not defined OPENOCD exit /b 1
cmake -S "%REPO%" -B "%REPO%\cmake-build-hx8-jy61p-capture" -G Ninja -DCMAKE_BUILD_TYPE=Debug -DHX8_JY60_CAPTURE=ON -DJY61P_CAPTURE=ON -DJY60_UART_DEMO=OFF -DSERVO1_ONLY_TEST=OFF
if errorlevel 1 exit /b 1
cmake --build "%REPO%\cmake-build-hx8-jy61p-capture" --parallel
if errorlevel 1 exit /b 1
powershell -NoProfile -Command "Get-CimInstance Win32_Process -Filter \"Name='openocd.exe'\" | Where-Object { $_.CommandLine -match 'daplink_rtt_capture.cfg' } | ForEach-Object { Stop-Process -Id $_.ProcessId -Force }"
set "ELF=%REPO%\cmake-build-hx8-jy61p-capture\FOC_DEMO2.elf"
set "ELF_TCL=%ELF:\=/%"
"%OPENOCD%" -c "tcl port disabled" -c "gdb port disabled" -c "telnet port disabled" -f "%REPO%\daplink.cfg" -c "program {%ELF_TCL%} verify reset exit"
if errorlevel 1 exit /b 1
"%OPENOCD%" -f "%REPO%\daplink_rtt_capture.cfg"
