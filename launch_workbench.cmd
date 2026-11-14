@echo off
setlocal
cd /d "%~dp0"
if exist ".venv\Scripts\python.exe" (
    ".venv\Scripts\python.exe" -m robotic_car.workbench
) else (
    python -m robotic_car.workbench
)
if errorlevel 1 pause
