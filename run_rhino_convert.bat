@echo off
setlocal

:: Adjust this path if your Rhino is installed elsewhere
set RHINO="C:\Program Files\Rhino 8\System\Rhino.exe"
if not exist %RHINO% set RHINO="C:\Program Files\Rhino 7\System\Rhino.exe"

set SCRIPT=%~dp0rhino_convert_to_step.py

echo Launching Rhino to batch convert SolidWorks assemblies to STEP...
echo Script: %SCRIPT%
echo.

%RHINO% /nosplash /runscript="-_RunPythonScript (%SCRIPT%)"

echo.
echo Rhino exited.
pause
