# Build the DHR knitwear-cell RoboDK station from robodk.yaml
#
# Prerequisites:
#   1. Start RoboDK on port 20502:
#      & "C:\RoboDK\bin\RoboDK.exe" -NEWINSTANCE -PORT=20502
#
# Usage: right-click -> Run with PowerShell
#   OR from PowerShell: .\robert_checker_stuff\build_dhr_station.ps1

$ErrorActionPreference = "Stop"

$ScriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$ProjectRoot = Split-Path -Parent $ScriptDir
$KnitwearCell = Join-Path $ProjectRoot "clones\knitwear-cell"
$SavePath = Join-Path $ProjectRoot "robo_dk_saves\generated_from_dhr_clone.rdk"

if (-not (Test-Path $KnitwearCell)) {
    Write-Host "[ERROR] knitwear-cell not found at: $KnitwearCell"
    Write-Host "        Run cloning_stuff\make_clones.sh first."
    pause
    exit 1
}

# Check for Python
$python = $null
foreach ($candidate in @("python", "python3", "py -3.12", "py")) {
    try {
        $ver = & $candidate.Split()[0] $candidate.Split()[1..99] --version 2>&1
        if ($ver -match "3\.1[2-9]") {
            $python = $candidate
            Write-Host "[INFO] Found Python: $ver"
            break
        }
    } catch {}
}

if (-not $python) {
    Write-Host "[ERROR] Python 3.12+ not found on PATH."
    Write-Host ""
    Write-Host "Install from https://www.python.org/downloads/"
    Write-Host "Make sure to check 'Add Python to PATH' during install."
    pause
    exit 1
}

# Install deps
Write-Host "[INFO] Installing dependencies..."
& $python.Split()[0] $python.Split()[1..99] -m pip install --quiet `
    "robodk==5.9.4" "pydantic==2.9" injector loguru numpy PyYAML `
    redis grpcio protobuf typing-extensions asyncua opcua

if ($LASTEXITCODE -ne 0) {
    Write-Host "[ERROR] Failed to install dependencies"
    pause
    exit 1
}

# Set environment and run
Write-Host "[INFO] Building station from $KnitwearCell"
Write-Host "[INFO] Make sure RoboDK is running on port 20502!"
Write-Host ""

$env:ENV_MODE = "local"
Push-Location $KnitwearCell

try {
    & $python.Split()[0] $python.Split()[1..99] `
        "..\..\robert_checker_stuff\build_dhr_station.py"
} finally {
    Pop-Location
}

if (Test-Path $SavePath) {
    $size = (Get-Item $SavePath).Length
    Write-Host ""
    Write-Host "[DONE] Saved: $SavePath ($size bytes)"
} else {
    Write-Host ""
    Write-Host "[INFO] If the station is visible in RoboDK, save manually:"
    Write-Host "       File -> Save As -> $SavePath"
}

pause
