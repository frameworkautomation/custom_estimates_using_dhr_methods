@echo off
setlocal

:: Adjust this path if your Rhino is installed elsewhere
set RHINO="C:\Program Files\Rhino 8\System\Rhino.exe"
if not exist %RHINO% set RHINO="C:\Program Files\Rhino 7\System\Rhino.exe"

set SCRIPT=%~dp0rhino_convert_to_step.py
set LOG=%~dp0..\rhino_convert.log

echo Launching Rhino to batch convert SolidWorks assemblies to STEP...
echo Progress will appear below. This window will close when done.
echo.

:: Launch Rhino in the background
start "" %RHINO% /nosplash /runscript="-_RunPythonScript (%SCRIPT%)"

:: Wait for the log file to appear (Rhino takes a moment to start)
:waitforlog
if not exist "%LOG%" (
    timeout /t 2 /nobreak >nul
    goto waitforlog
)

:: Stream the log file, show a progress bar, exit when done
powershell -Command "Get-Content -Path '%LOG%' -Wait | ForEach-Object { Write-Host $_; if ($_ -match '\[(\d+)/(\d+)\]') { $c = [int]$Matches[1]; $t = [int]$Matches[2]; Write-Progress -Activity 'Converting SolidWorks to STEP' -Status \"$c of $t files\" -PercentComplete ([int]($c/$t*100)) }; if ($_ -match 'Done\.') { Write-Progress -Activity 'Converting SolidWorks to STEP' -Completed; Start-Sleep 1; exit } }"

echo.
echo Conversion complete. See rhino_convert.log for full output.
pause
