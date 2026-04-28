param(
    # Skip the build step -- assume `quick.ps1` (or `build.ps1`) was just run and
    # bin/artifacts/Release is fresh. Useful when iterating on the deploy step
    # itself.
    [switch]$NoBuild,

    # Don't auto-launch SpaceCalibrator after copying. Useful when you want
    # to inspect the install or wait for SteamVR to start the overlay itself.
    [switch]$NoLaunch
)

# Hot-swap the freshly built overlay EXE into the installed location and
# (re)launch it. Designed for the inner iteration loop:
#   1. Edit code.
#   2. quick.ps1 (or this script's auto-build step).
#   3. deploy-test.ps1  → kills running overlay, copies new EXE, relaunches.
#
# Driver DLL swaps require restarting SteamVR (the driver is loaded by
# vrserver.exe and Windows holds a file lock on it). This script deliberately
# only touches the overlay EXE -- driver changes need a manual SteamVR restart.

$ErrorActionPreference = "Stop"
Set-Location $PSScriptRoot

# --- Step 1: build (unless -NoBuild) ----------------------------------------
if (-not $NoBuild) {
    Write-Host "Building..." -ForegroundColor Cyan
    & powershell.exe -ExecutionPolicy Bypass -File (Join-Path $PSScriptRoot "quick.ps1")
    if ($LASTEXITCODE -ne 0) {
        throw "quick.ps1 failed with exit $LASTEXITCODE"
    }
}

$BuiltExe = Join-Path $PSScriptRoot "bin\artifacts\Release\SpaceCalibrator.exe"
if (-not (Test-Path $BuiltExe)) {
    throw "Built overlay not found at $BuiltExe -- run quick.ps1 first or omit -NoBuild."
}

# --- Step 2: locate the installed copy --------------------------------------
# Registry tells us where the NSIS installer dropped the EXE. Three fallback
# locations cover x64-on-x64, 32-bit-as-WOW6432, and per-user installs. We
# prefer the registry over a hardcoded "C:\Program Files\..." path because the
# user might have installed elsewhere.

function Get-InstallDirFromRegistry() {
    $keys = @(
        "HKLM:\Software\Microsoft\Windows\CurrentVersion\Uninstall\OpenVRSpaceCalibrator",
        "HKLM:\SOFTWARE\WOW6432Node\Microsoft\Windows\CurrentVersion\Uninstall\OpenVRSpaceCalibrator",
        "HKCU:\Software\Microsoft\Windows\CurrentVersion\Uninstall\OpenVRSpaceCalibrator"
    )
    foreach ($k in $keys) {
        $v = (Get-ItemProperty -Path $k -ErrorAction SilentlyContinue).InstallLocation
        if ($v -and (Test-Path $v)) { return $v }
        # InstallLocation may be missing; derive from UninstallString if present.
        $u = (Get-ItemProperty -Path $k -ErrorAction SilentlyContinue).UninstallString
        if ($u) {
            $u = $u.Trim('"')
            $dir = Split-Path -Parent $u
            if ($dir -and (Test-Path $dir)) { return $dir }
        }
    }
    return $null
}

$InstallDir = Get-InstallDirFromRegistry
if (-not $InstallDir) {
    # Fallback to the NSIS installer's default location. If neither registry
    # nor this path exists, the user hasn't installed yet -- bail with a clear
    # message rather than silently copying nowhere.
    $defaultPath = "C:\Program Files\OpenVR-SpaceCalibrator"
    if (Test-Path $defaultPath) {
        $InstallDir = $defaultPath
    } else {
        throw "Could not find an installed copy of OpenVR-SpaceCalibrator. Install the latest release first (one-time setup), then re-run this script."
    }
}

$InstalledExe = Join-Path $InstallDir "SpaceCalibrator.exe"
Write-Host "Install dir:  $InstallDir" -ForegroundColor DarkGray
Write-Host "Installed exe: $InstalledExe" -ForegroundColor DarkGray

# --- Step 3: stop running overlay -------------------------------------------
# SpaceCalibrator may be open as a SteamVR overlay or as a standalone window.
# Either way the EXE is locked while it's running; kill it so we can replace.
$proc = Get-Process -Name "SpaceCalibrator" -ErrorAction SilentlyContinue
if ($proc) {
    Write-Host "Stopping SpaceCalibrator (PID $($proc.Id))..." -ForegroundColor DarkGray
    $proc | Stop-Process -Force
    # Give Windows a moment to release the file handle. Stop-Process is
    # synchronous w.r.t. process termination but the file lock release lags
    # by a few ms in practice; a short sleep avoids a CopyItem race.
    Start-Sleep -Milliseconds 750
} else {
    Write-Host "SpaceCalibrator wasn't running." -ForegroundColor DarkGray
}

# --- Step 4: copy the new EXE ------------------------------------------------
# Program Files needs admin. If we can't write, relaunch this script elevated
# and pass the same flags through. Detect by trying the copy and catching the
# UnauthorizedAccessException -- cleaner than testing IsInRole/IsAdmin which
# can give a false negative under split-token UAC.
function Try-Copy() {
    Copy-Item -Force -Path $BuiltExe -Destination $InstalledExe -ErrorAction Stop
}

try {
    Try-Copy
    Write-Host "Copied $BuiltExe -> $InstalledExe" -ForegroundColor Green
} catch [System.UnauthorizedAccessException], [System.IO.IOException] {
    Write-Host "Copy needs elevation. Relaunching as admin..." -ForegroundColor Yellow
    $argList = @(
        '-ExecutionPolicy', 'Bypass',
        '-File', $PSCommandPath,
        '-NoBuild'  # we already built; don't rebuild under the elevated child
    )
    if ($NoLaunch) { $argList += '-NoLaunch' }
    Start-Process -FilePath "powershell.exe" -ArgumentList $argList -Verb RunAs -Wait
    # The elevated child handled launch; bail out without doing it twice.
    exit 0
}

# --- Step 5: launch the overlay ---------------------------------------------
if ($NoLaunch) {
    Write-Host "Skipping launch (-NoLaunch)." -ForegroundColor DarkGray
    exit 0
}

# Prefer SteamVR's overlay registry entry if SteamVR is running -- that gives
# us the same launch path the user gets from the dashboard, including any
# auto-launch settings. Otherwise just spawn the EXE directly. Either way we
# don't wait on the process.
$vrServerRunning = $null -ne (Get-Process -Name "vrserver" -ErrorAction SilentlyContinue)
if ($vrServerRunning) {
    Write-Host "SteamVR is running -- launching overlay via vrmonitor would be ideal," -ForegroundColor DarkGray
    Write-Host "but the simplest reliable path is to start the EXE directly." -ForegroundColor DarkGray
}

Write-Host "Launching $InstalledExe ..." -ForegroundColor Green
Start-Process -FilePath $InstalledExe -WorkingDirectory $InstallDir
