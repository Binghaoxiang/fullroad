@echo off
cd /d "%~dp0"
set "PY=C:\Users\a\.dsh\dsh-runtimes\dsh-primary-runtime\dependencies\python\python.exe"
if not exist "%PY%" set "PY=python"
if "%~1"=="" (
    "%PY%" tools\pos_ui.py COM5
) else (
    "%PY%" tools\pos_ui.py %~1
)
pause