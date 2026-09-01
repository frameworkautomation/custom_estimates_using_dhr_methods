# Download and install Python 3.12 on Windows
#
# Usage: right-click -> Run with PowerShell (as Administrator recommended)
#   OR from PowerShell: .\robert_checker_stuff\install_python_windows.ps1

$ErrorActionPreference = "Stop"

$PythonUrl = "https://www.python.org/ftp/python/3.12.9/python-3.12.9-amd64.exe"
$Installer = "$env:TEMP\python-3.12.9-amd64.exe"

# Check if already installed
try {
    $ver = & python --version 2>&1
    if ($ver -match "3\.1[2-9]") {
        Write-Host "[INFO] Python already installed: $ver"
        Write-Host "[INFO] Installing DHR dependencies..."
        & python -m pip install --quiet `
            "robodk==5.9.4" "pydantic==2.9" injector loguru numpy PyYAML `
            redis grpcio protobuf typing-extensions asyncua opcua
        Write-Host "[DONE] Dependencies installed."
        pause
        exit 0
    }
} catch {}

Write-Host "[INFO] Downloading Python 3.12.9..."
Invoke-WebRequest -Uri $PythonUrl -OutFile $Installer

Write-Host "[INFO] Installing Python 3.12.9..."
Write-Host "       (This adds Python to PATH and installs pip)"
Start-Process -Wait -FilePath $Installer -ArgumentList `
    "/quiet", "InstallAllUsers=0", "PrependPath=1", "Include_pip=1"

# Clean up
Remove-Item $Installer -ErrorAction SilentlyContinue

Write-Host "[INFO] Refreshing PATH..."
$env:Path = [System.Environment]::GetEnvironmentVariable("Path", "User") + ";" + [System.Environment]::GetEnvironmentVariable("Path", "Machine")

# Verify
try {
    $ver = & python --version 2>&1
    Write-Host "[INFO] Python installed: $ver"
} catch {
    Write-Host "[WARN] Python installed but not on PATH yet."
    Write-Host "       Close and reopen PowerShell, then try: python --version"
    pause
    exit 0
}

# Install deps
Write-Host "[INFO] Installing DHR dependencies..."
& python -m pip install `
    "robodk==5.9.4" "pydantic==2.9" injector loguru numpy PyYAML `
    redis grpcio protobuf typing-extensions asyncua opcua

Write-Host ""
Write-Host "[DONE] Python 3.12 + dependencies installed."
Write-Host "       Close and reopen PowerShell, then run:"
Write-Host "       .\robert_checker_stuff\build_dhr_station.ps1"
pause
