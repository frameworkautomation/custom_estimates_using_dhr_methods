@echo off
REM Build the DHR knitwear-cell RoboDK station from robodk.yaml
REM
REM Prerequisites:
REM   1. Python 3.12+ installed and on PATH
REM   2. RoboDK running on port 20502:
REM      "C:\RoboDK\bin\RoboDK.exe" -NEWINSTANCE -PORT=20502
REM
REM Usage: double-click this file or run from CMD/PowerShell

cd /d "%~dp0\..\clones\knitwear-cell"
if errorlevel 1 (
    echo [ERROR] knitwear-cell clone not found. Run cloning_stuff\make_clones.sh first.
    pause
    exit /b 1
)

REM Check for Python
where python >nul 2>nul
if errorlevel 1 (
    echo [ERROR] Python not found on PATH.
    echo Install Python 3.12+ from python.org and ensure it's on PATH.
    pause
    exit /b 1
)

REM Install deps if needed
echo [INFO] Installing dependencies...
pip install robodk==5.9.4 "pydantic==2.9" injector loguru numpy PyYAML redis grpcio protobuf typing-extensions asyncua opcua

REM Set local config mode
set ENV_MODE=local

echo.
echo [INFO] Building station (this takes about a minute)...
echo [INFO] Make sure RoboDK is running on port 20502!
echo.

python ..\..\robert_checker_stuff\build_dhr_station.py %*

echo.
pause
