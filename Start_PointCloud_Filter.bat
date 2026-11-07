@echo off
setlocal
cd /d "%~dp0"

echo PointCloud Filter 3.1 - preparing Python environment...
if exist ".venv\Scripts\python.exe" goto run

rem Prefer Python 3.12, which is compatible with the user's installed Python.
where py >nul 2>&1
if not errorlevel 1 (
    py -3.12 -m venv .venv 2>nul
    if not errorlevel 1 goto run
    py -3 -m venv .venv 2>nul
    if not errorlevel 1 goto run
)
where python >nul 2>&1
if not errorlevel 1 (
    python -m venv .venv
    if not errorlevel 1 goto run
)
echo Could not create a Python virtual environment.
echo Install Python 3.12 (64-bit) from python.org and try again.
pause
exit /b 1

:run
".venv\Scripts\python.exe" pointcloud_filter.py
if errorlevel 1 (
    echo.
    echo PointCloud Filter exited with an error.
    pause
)
endlocal
