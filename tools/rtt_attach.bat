@echo off
setlocal EnableExtensions

rem Start OpenOCD RTT server without resetting the target.
rem Use this after the firmware is already running from Flash.

for %%I in ("%~dp0..") do set "REPO=%%~fI"

set "OPENOCD=D:\STM32_Env\OpenOCD-20231002-0.12.0\bin\openocd.exe"
set "OPENOCD_SCRIPTS=D:\STM32_Env\OpenOCD-20231002-0.12.0\share\openocd\scripts"
set "RTT_ATTACH_CFG=%REPO%\daplink_rtt_attach.cfg"

echo Start attach-only RTT server.
echo VOFA+: TCP Client 127.0.0.1:9090, RawData/text.
echo Keep this window open. Press Ctrl+C to stop RTT.

"%OPENOCD%" -s "%OPENOCD_SCRIPTS%" -f "%RTT_ATTACH_CFG%"
