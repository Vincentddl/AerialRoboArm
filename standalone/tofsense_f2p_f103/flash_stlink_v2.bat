@echo off
setlocal EnableExtensions

rem ST-Link V2 + OpenOCD ELF programming script for STM32F103C8T6.
rem Double-click: programs the default Debug ELF.
rem Drag an ELF onto this file: programs the dropped ELF.

set "PROJECT_DIR=%~dp0"
set "OPENOCD_EXE=D:\DevEnv\openocd-v0.12.0-i686-w64-mingw32\bin\openocd.exe"
set "OPENOCD_CFG=%PROJECT_DIR%TOFSense_F2P_F103_STLinkV2.cfg"
set "ELF_FILE=%PROJECT_DIR%cmake-build-debug\TOFSense_F2P_F103.elf"
set "NO_PAUSE=0"
set "RESULT=0"

rem The installation path can be overridden without editing this file.
if defined TOFSENSE_OPENOCD set "OPENOCD_EXE=%TOFSENSE_OPENOCD%"

if /I "%~1"=="--no-pause" (
    set "NO_PAUSE=1"
) else if not "%~1"=="" (
    set "ELF_FILE=%~f1"
)

if /I "%~2"=="--no-pause" set "NO_PAUSE=1"

echo [ST-Link V2] ELF: "%ELF_FILE%"

if not exist "%OPENOCD_EXE%" (
    echo [ERROR] OpenOCD not found: "%OPENOCD_EXE%"
    echo Set TOFSENSE_OPENOCD to the full path of openocd.exe.
    set "RESULT=2"
    goto finish
)

if not exist "%OPENOCD_CFG%" (
    echo [ERROR] OpenOCD config not found: "%OPENOCD_CFG%"
    set "RESULT=3"
    goto finish
)

if not exist "%ELF_FILE%" (
    echo [ERROR] ELF not found: "%ELF_FILE%"
    echo Build the Debug target first, or drag an ELF file onto this script.
    set "RESULT=4"
    goto finish
)

echo [1/3] Connecting to ST-Link V2 over SWD...
echo [2/3] Programming and verifying ELF...
"%OPENOCD_EXE%" -f "%OPENOCD_CFG%" -c "program {%ELF_FILE%} verify reset exit"
set "RESULT=%ERRORLEVEL%"

if "%RESULT%"=="0" (
    echo [3/3] SUCCESS: firmware verified and MCU reset.
) else (
    echo [ERROR] Programming failed. OpenOCD exit code: %RESULT%
    echo Check ST-Link wiring, target power, driver, and SWD connection.
)

:finish
if "%NO_PAUSE%"=="0" pause
exit /b %RESULT%
