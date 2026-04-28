param(
    # Skip the build step -- assume `quick.ps1` (or `build.ps1`) was just run and
    # bin/artifacts/Release is fresh.
    [switch]$NoBuild,

    # Don't auto-launch SpaceCalibrator after copying.
    [switch]$NoLaunch,

    # Force-kill SteamVR (vrserver.exe + vrmonitor.exe) if a driver swap is
    # needed. Default: don't kill -- print clear instructions and bail. Pass
    # this flag if you want the script to do it for you.
    [switch]$KillSteamVR
)

# Hot-swap the freshly built overlay EXE -- and the driver DLL when its bits
# have changed -- into the installed location, then relaunch the overlay.
#
# Two locks to think about:
#   - The overlay EXE: locked while SpaceCalibrator is open. Stop-Process is
#     fine -- it's our own process, no privileges needed beyond user.
#   - The driver DLL: locked by SteamVR (vrserver.exe). Replacing it requires
#     stopping SteamVR completely, which we will do iff -KillSteamVR was passed.
#
# Elevation is only required when the install lives under Program Files (the
# default NSIS path). We detect that up-front by probing the install dir for
# writability, and either run inline (already admin / user-writable install)
# or relaunch the script as admin and exit. Critically the destructive steps
# (Stop-Process, file copy) only run AFTER we know we have write access, so a
# script run never both "kills SteamVR" then "fails to copy" -- which is what
# made the previous version produce duplicated output and confuse you.

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
$BuiltDll = Join-Path $PSScriptRoot "bin\driver_01spacecalibrator\bin\win64\driver_01spacecalibrator.dll"
if (-not (Test-Path $BuiltExe)) {
    throw "Built overlay not found at $BuiltExe -- run quick.ps1 first or omit -NoBuild."
}
if (-not (Test-Path $BuiltDll)) {
    throw "Built driver DLL not found at $BuiltDll -- run quick.ps1 first or omit -NoBuild."
}

# --- Step 2: locate the installed overlay -----------------------------------
function Get-InstallDirFromRegistry() {
    $keys = @(
        "HKLM:\Software\Microsoft\Windows\CurrentVersion\Uninstall\OpenVRSpaceCalibrator",
        "HKLM:\SOFTWARE\WOW6432Node\Microsoft\Windows\CurrentVersion\Uninstall\OpenVRSpaceCalibrator",
        "HKCU:\Software\Microsoft\Windows\CurrentVersion\Uninstall\OpenVRSpaceCalibrator"
    )
    foreach ($k in $keys) {
        $v = (Get-ItemProperty -Path $k -ErrorAction SilentlyContinue).InstallLocation
        if ($v -and (Test-Path $v)) { return $v }
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
    $candidate1 = "C:\Program Files\OpenVR-SpaceCalibrator"
    $candidate2 = "C:\Program Files\SpaceCalibrator"
    if (Test-Path $candidate1) { $InstallDir = $candidate1 }
    elseif (Test-Path $candidate2) { $InstallDir = $candidate2 }
    else {
        throw "Could not find an installed copy of OpenVR-SpaceCalibrator. Install the latest release first (one-time setup), then re-run this script."
    }
}

$InstalledExe = Join-Path $InstallDir "SpaceCalibrator.exe"
Write-Host "Install dir:   $InstallDir" -ForegroundColor DarkGray
Write-Host "Installed exe: $InstalledExe" -ForegroundColor DarkGray

# --- Step 3: locate the installed driver DLL --------------------------------
function Get-SteamRoot() {
    $candidates = @(
        @{ Hive = "HKLM:"; Key = "SOFTWARE\WOW6432Node\Valve\Steam"; Value = "InstallPath" },
        @{ Hive = "HKLM:"; Key = "SOFTWARE\Valve\Steam";              Value = "InstallPath" },
        @{ Hive = "HKCU:"; Key = "SOFTWARE\Valve\Steam";              Value = "SteamPath"   }
    )
    foreach ($c in $candidates) {
        $path = "$($c.Hive)\$($c.Key)"
        $v = (Get-ItemProperty -Path $path -Name $c.Value -ErrorAction SilentlyContinue).$($c.Value)
        if ($v -and (Test-Path $v)) { return $v }
    }
    return $null
}

function Get-SteamLibraries($steamRoot) {
    $out = @()
    if (-not $steamRoot) { return $out }
    $defaultLib = Join-Path $steamRoot "steamapps"
    if (Test-Path $defaultLib) { $out += $defaultLib }

    # Build the array with explicit elements; calling Join-Path inside an @(...)
    # initializer trips the PowerShell parser (it tries to glue the comma onto
    # Join-Path's argument list instead of treating it as the array separator).
    $vdf1 = Join-Path $steamRoot "config\libraryfolders.vdf"
    $vdf2 = Join-Path $steamRoot "steamapps\libraryfolders.vdf"
    $vdfCandidates = @($vdf1, $vdf2)

    foreach ($vdf in $vdfCandidates) {
        if (-not (Test-Path $vdf)) { continue }
        $lines = Get-Content -Path $vdf -ErrorAction SilentlyContinue
        foreach ($line in $lines) {
            if ($line -match '"path"\s+"([^"]+)"') {
                $libRoot = $matches[1].Replace('\\', '\')
                $libSteamApps = Join-Path $libRoot "steamapps"
                if (Test-Path $libSteamApps) { $out += $libSteamApps }
            }
        }
        break
    }
    return ($out | Sort-Object -Unique)
}

function Find-InstalledDriver() {
    $steamRoot = Get-SteamRoot
    $libs = Get-SteamLibraries $steamRoot
    foreach ($lib in $libs) {
        $candidate = Join-Path $lib "common\SteamVR\drivers\01spacecalibrator\bin\win64\driver_01spacecalibrator.dll"
        if (Test-Path $candidate) { return $candidate }
    }
    $fallback = "C:\Program Files (x86)\Steam\steamapps\common\SteamVR\drivers\01spacecalibrator\bin\win64\driver_01spacecalibrator.dll"
    if (Test-Path $fallback) { return $fallback }
    return $null
}

$InstalledDll = Find-InstalledDriver
if ($InstalledDll) {
    Write-Host "Installed dll: $InstalledDll" -ForegroundColor DarkGray
} else {
    Write-Host "Installed dll: NOT FOUND -- the SteamVR driver is missing." -ForegroundColor Yellow
}

# --- Step 4: hash compare; what needs swapping? -----------------------------
function Get-FileHashSafe($path) {
    if (-not $path -or -not (Test-Path $path)) { return $null }
    return (Get-FileHash -Algorithm SHA256 -Path $path).Hash
}

$builtExeHash = Get-FileHashSafe $BuiltExe
$instExeHash  = Get-FileHashSafe $InstalledExe
$builtDllHash = Get-FileHashSafe $BuiltDll
$instDllHash  = Get-FileHashSafe $InstalledDll

$needExeSwap = ($builtExeHash -ne $instExeHash) -or (-not $instExeHash)
$needDllSwap = ($InstalledDll -and ($builtDllHash -ne $instDllHash))

Write-Host ""
Write-Host "Overlay swap needed: $needExeSwap" -ForegroundColor DarkGray
Write-Host "Driver  swap needed: $needDllSwap" -ForegroundColor DarkGray

if (-not $needExeSwap -and -not $needDllSwap) {
    Write-Host "Nothing to do -- installed copies already match the build." -ForegroundColor Green
    if (-not $NoLaunch) {
        Write-Host "Launching $InstalledExe ..." -ForegroundColor Green
        Start-Process -FilePath $InstalledExe -WorkingDirectory $InstallDir
    }
    exit 0
}

# --- Step 5: do we need elevation? -----------------------------------------
# Probe the install dir for write access by creating a tiny temp file. If it
# works, we can do the copies inline. If it throws (UnauthorizedAccess), we
# elevate and bail BEFORE running any destructive step. This is the key fix
# vs. the previous version: that one stopped SpaceCalibrator + killed
# SteamVR in the parent, then on copy failure relaunched as admin which did
# the same destructive steps again, producing the doubled output you saw.

function Test-CanWriteToDir($dir) {
    $probePath = Join-Path $dir "spacecal_writeprobe.tmp"
    try {
        [System.IO.File]::WriteAllText($probePath, "x")
        Remove-Item -Path $probePath -Force -ErrorAction SilentlyContinue
        return $true
    } catch {
        return $false
    }
}

# Two install dirs to consider: the overlay's dir and the driver's parent dir
# (when present). If EITHER is non-writable, we need elevation.
$canWriteOverlay = $true
if ($needExeSwap) {
    $canWriteOverlay = Test-CanWriteToDir $InstallDir
}

$canWriteDriver = $true
if ($needDllSwap -and $InstalledDll) {
    $driverDir = Split-Path -Parent $InstalledDll
    $canWriteDriver = Test-CanWriteToDir $driverDir
}

if (-not $canWriteOverlay -or -not $canWriteDriver) {
    Write-Host ""
    Write-Host "Needs elevation. Relaunching as admin..." -ForegroundColor Yellow
    $argList = @(
        '-NoProfile',
        '-ExecutionPolicy', 'Bypass',
        '-File', $PSCommandPath,
        '-NoBuild'
    )
    if ($NoLaunch)    { $argList += '-NoLaunch' }
    if ($KillSteamVR) { $argList += '-KillSteamVR' }
    Start-Process -FilePath "powershell.exe" -ArgumentList $argList -Verb RunAs -Wait
    # The elevated child handled everything; we are done.
    exit 0
}

# --- Step 6: stop running overlay -------------------------------------------
$proc = Get-Process -Name "SpaceCalibrator" -ErrorAction SilentlyContinue
if ($proc) {
    Write-Host "Stopping SpaceCalibrator (PID $($proc.Id))..." -ForegroundColor DarkGray
    $proc | Stop-Process -Force
    Start-Sleep -Milliseconds 750
}

# --- Step 7: handle SteamVR if a driver swap is required --------------------
if ($needDllSwap) {
    $vrserver = Get-Process -Name "vrserver" -ErrorAction SilentlyContinue
    $vrmonitor = Get-Process -Name "vrmonitor" -ErrorAction SilentlyContinue
    if ($vrserver -or $vrmonitor) {
        if ($KillSteamVR) {
            Write-Host "Stopping SteamVR (you passed -KillSteamVR)..." -ForegroundColor Yellow
            if ($vrserver)  { $vrserver  | Stop-Process -Force }
            if ($vrmonitor) { $vrmonitor | Stop-Process -Force }
            # Children of vrserver (vrcompositor, vrwebhelper, vrdashboard) exit when
            # vrserver does, but a sleep gives them a moment so the DLL handle drops.
            Start-Sleep -Seconds 2
        } else {
            Write-Host ""
            Write-Host "================================================================" -ForegroundColor Red
            Write-Host " SteamVR is running with the OLD driver loaded." -ForegroundColor Red
            Write-Host " The driver DLL has changed (likely a protocol-version bump)," -ForegroundColor Red
            Write-Host " so the new overlay won't be able to talk to the old driver." -ForegroundColor Red
            Write-Host "" -ForegroundColor Red
            Write-Host " Close SteamVR completely (right-click the SteamVR taskbar" -ForegroundColor Red
            Write-Host " icon -> Exit, or close it from the Steam client), then" -ForegroundColor Red
            Write-Host " re-run this script." -ForegroundColor Red
            Write-Host "" -ForegroundColor Red
            Write-Host " Or pass -KillSteamVR to have this script do it for you:" -ForegroundColor Red
            Write-Host "   .\deploy-test.ps1 -KillSteamVR" -ForegroundColor Red
            Write-Host "================================================================" -ForegroundColor Red
            exit 2
        }
    }
}

# --- Step 8: copy ------------------------------------------------------------
# The driver DLL is sometimes loaded by steam.exe itself (for app
# enumeration), not just vrserver. Killing SteamVR doesn't release that
# handle. Direct Copy-Item fails with sharing-violation. The workaround:
# rename the locked file (Windows allows rename on a DELETE-shared handle,
# which DLLs typically are) to free the canonical filename, then copy the
# new DLL into place. The old file's content stays in memory until steam.exe
# unloads it; the next SteamVR load picks up the new file.
function Replace-File($src, $dst) {
    try {
        Copy-Item -Force -Path $src -Destination $dst -ErrorAction Stop
        return
    } catch [System.IO.IOException] {
        $oldPath = "$dst.old"
        if (Test-Path $oldPath) {
            try { Remove-Item -Force -Path $oldPath -ErrorAction Stop } catch { }
        }
        Move-Item -Force -Path $dst -Destination $oldPath -ErrorAction Stop
        Copy-Item -Force -Path $src -Destination $dst -ErrorAction Stop
        Write-Host "  (renamed prior file to .old; it's loaded in another process and will release on next restart)" -ForegroundColor DarkGray
    }
}

if ($needExeSwap) {
    Replace-File $BuiltExe $InstalledExe
    Write-Host "Copied overlay -> $InstalledExe" -ForegroundColor Green
}
if ($needDllSwap) {
    Replace-File $BuiltDll $InstalledDll
    Write-Host "Copied driver  -> $InstalledDll" -ForegroundColor Green
}

# --- Step 9: launch ----------------------------------------------------------
if ($NoLaunch) {
    Write-Host "Skipping launch (-NoLaunch)." -ForegroundColor DarkGray
    if ($needDllSwap) {
        Write-Host ""
        Write-Host "REMEMBER: SteamVR was stopped to swap the driver. Start it again" -ForegroundColor Yellow
        Write-Host "from your Steam client, then launch the overlay." -ForegroundColor Yellow
    }
    exit 0
}

if ($needDllSwap) {
    Write-Host ""
    Write-Host "Driver was swapped. Start SteamVR now (Steam client -> Library ->" -ForegroundColor Yellow
    Write-Host "SteamVR -> Play). The overlay will be available from the SteamVR" -ForegroundColor Yellow
    Write-Host "dashboard once SteamVR finishes loading." -ForegroundColor Yellow
} else {
    Write-Host "Launching $InstalledExe ..." -ForegroundColor Green
    Start-Process -FilePath $InstalledExe -WorkingDirectory $InstallDir
}
