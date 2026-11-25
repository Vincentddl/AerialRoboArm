@echo off
setlocal
cd /d "%~dp0"
set "PYTHONHOME="
set "PYTHONPATH="
if exist "D:\app\miniconda\envs\python312\python.exe" (
    "D:\app\miniconda\envs\python312\python.exe" main.py
) else if exist "%~dp0.venv\Scripts\python.exe" (
    "%~dp0.venv\Scripts\python.exe" main.py
) else (
    "%~dp0runtime\python.exe" -E -s main.py
)
if errorlevel 1 pause
