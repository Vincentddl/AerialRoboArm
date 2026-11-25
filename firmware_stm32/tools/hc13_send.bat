@echo off
setlocal

set "HC13_PYTHON=D:\app\miniconda\envs\python312\python.exe"
if not exist "%HC13_PYTHON%" set "HC13_PYTHON=python"

"%HC13_PYTHON%" "%~dp0hc13_send.py" %*
exit /b %ERRORLEVEL%
