param(
    # Skip the cmake configure step. Default ON for quick.ps1 — if you've already
    # configured once, MSBuild can rebuild from the existing solution. Pass
    # -Reconfigure to force a fresh cmake configure (needed after CMakeLists or
    # toolchain changes).
    [switch]$Reconfigure,

    # By default we skip both configure AND release-zip packaging so the
    # iteration loop is just "save -> run quick.ps1 -> launch". Pass -Zip to
    # also produce the drop-in distribution zip.
    [switch]$Zip,

    # After building, hot-swap the runtime files into the existing install at
    # $InstallPath. Self-elevates so you don't need to spawn an admin shell
    # first. Without -Install, quick.ps1 just builds.
    [switch]$Install,

    # Full deploy: closes Steam (which holds the driver DLL locked while
    # running), copies the driver DLL into <SteamVR>/drivers, hot-swaps the
    # overlay (implies -Install), disables the legacy 01fingersmoothing
    # driver folder if present (its hooks would collide with SC's now that
    # SC owns the IVRDriverInputInternal hook), then relaunches Steam.
    # Only needed when iterating on driver-side changes; pure overlay edits
    # use -Install instead. Self-elevates for the admin operations.
    [switch]$DeployDriver,

    # Where the install lives. Override only if you've installed somewhere
    # non-default (custom NSIS install path, etc.).
    [string]$InstallPath = "C:\Program Files\SpaceCalibrator",

    # Where Steam lives. Required for -DeployDriver so we can locate the
    # SteamVR drivers folder + the steam.exe to graceful-shutdown / relaunch.
    [string]$SteamPath = "C:\Program Files (x86)\Steam"
)

# -DeployDriver implies -Install: keeping overlay and driver out of sync would
# fail the protocol::Version handshake on the next overlay launch, which is a
# worse experience than the small extra cost of swapping the overlay too.
if ($DeployDriver -and -not $Install) {
    $Install = $true
}

$ErrorActionPreference = "Stop"
Set-Location $PSScriptRoot

# Forward to build.ps1 with the right flags. quick.ps1 exists purely so the
# user has a one-word command to run while iterating. Default behaviour:
#   - skip cmake configure (it's slow and rarely needed when iterating)
#   - skip the release zip (we don't need a distribution artifact mid-iteration)
#   - leave version stamping ON so SPACECAL_BUILD_STAMP increments and the
#     in-app updater + git hooks can tell builds apart
$buildArgs = @()
if (-not $Reconfigure) { $buildArgs += "-SkipConfigure" }
if (-not $Zip)         { $buildArgs += "-SkipZip" }

Write-Host "quick.ps1: forwarding to build.ps1 $($buildArgs -join ' ')" -ForegroundColor DarkGray
& powershell.exe -ExecutionPolicy Bypass -File (Join-Path $PSScriptRoot "build.ps1") @buildArgs
$exitCode = $LASTEXITCODE
if ($exitCode -ne 0) {
    throw "build.ps1 failed (exit $exitCode)"
}

$ArtifactsDir = Join-Path $PSScriptRoot "bin/artifacts/Release"
$Overlay      = Join-Path $ArtifactsDir "SpaceCalibrator.exe"
if (-not (Test-Path $Overlay)) {
    throw "Build said it succeeded but $Overlay isn't there. Something is off."
}

if (-not $Install) {
    Write-Host ""
    Write-Host "Built overlay:  $Overlay" -ForegroundColor Green
    Write-Host ""
    Write-Host "Run directly from the build dir:" -ForegroundColor DarkGray
    Write-Host "  & '$Overlay'" -ForegroundColor DarkGray
    Write-Host ""
    Write-Host "Or build + hot-swap into your installed copy in one step:" -ForegroundColor DarkGray
    Write-Host "  ./quick.ps1 -Install            (overlay only; SteamVR can stay running)" -ForegroundColor DarkGray
    Write-Host "  ./quick.ps1 -DeployDriver       (overlay + driver; closes Steam, restarts it)" -ForegroundColor DarkGray
    return
}

# --- DeployDriver pre-flight: close Steam so the driver DLL isn't locked --
# Steam's driver scanner enumerates SteamVR/drivers/* on startup and holds
# the manifests + DLLs until Steam itself exits. Closing only SteamVR isn't
# enough (real-world bite from 2026-05-01: copy succeeded but the OS
# silently kept the old DLL mapped because steam.exe still had a handle).
# Documented in reference_install_procedure.md.
$DriverDll        = Join-Path $PSScriptRoot "bin/driver_01spacecalibrator/bin/win64/driver_01spacecalibrator.dll"
$SteamExe         = Join-Path $SteamPath "steam.exe"
$SteamDriversDir  = Join-Path $SteamPath "steamapps/common/SteamVR/drivers"
$DestDriverDll    = Join-Path $SteamDriversDir "01spacecalibrator/bin/win64/driver_01spacecalibrator.dll"
$LegacyFsManifest = Join-Path $SteamDriversDir "01fingersmoothing/driver.vrdrivermanifest"

if ($DeployDriver) {
    if (-not (Test-Path $DriverDll)) {
        throw "Driver DLL not found at '$DriverDll'. Did the build produce a driver target?"
    }
    if (-not (Test-Path $SteamExe)) {
        throw "Steam not found at '$SteamExe'. Pass -SteamPath '<your Steam dir>' if installed elsewhere."
    }
    if (-not (Test-Path $SteamDriversDir)) {
        throw "SteamVR drivers folder not found at '$SteamDriversDir'. Is SteamVR installed?"
    }

    $steamRunning = @(Get-Process -Name steam -ErrorAction SilentlyContinue).Count -gt 0
    if ($steamRunning) {
        Write-Host ""
        Write-Host "Sending Steam graceful shutdown..." -ForegroundColor Green
        & $SteamExe -shutdown | Out-Null
        # Steam's IPC shutdown is asynchronous: we get an immediate return,
        # then steam.exe walks its child-process graveyard (VRChat, vrserver,
        # vrmonitor, etc.) and exits. Real-world wait observed: 2-10s with
        # SteamVR + a game open. 30s ceiling is generous; force-kill below.
        $deadline = (Get-Date).AddSeconds(30)
        while ((Get-Process -Name steam -ErrorAction SilentlyContinue) -and (Get-Date) -lt $deadline) {
            Start-Sleep -Milliseconds 500
        }
        if (Get-Process -Name steam -ErrorAction SilentlyContinue) {
            Write-Host "Steam still up after 30s; force-killing Steam + SteamVR processes..." -ForegroundColor Yellow
            Stop-Process -Name steam,steamwebhelper,vrserver,vrmonitor,vrwebhelper,vrcompositor,vrstartup -Force -ErrorAction SilentlyContinue
            Start-Sleep -Seconds 2
        }
        Write-Host "Steam closed." -ForegroundColor Green
    } else {
        Write-Host "Steam not running; skipping shutdown." -ForegroundColor DarkGray
        # Defensive: if Steam was killed but a SteamVR helper lingered, kill
        # it now. Their handles on the driver DLL would still block the copy.
        Stop-Process -Name vrserver,vrmonitor,vrwebhelper,vrcompositor,vrstartup -Force -ErrorAction SilentlyContinue
    }
}

# --- Hot-swap install -----------------------------------------------------
# Stops any running overlay (so the EXE isn't locked), copies the runtime
# files into $InstallPath, refreshes the Windows icon cache so taskbar /
# pinned-shortcut icons pick up the new embedded .ico.
#
# Self-elevates: if we're not already in an admin shell, the copy runs in
# a UAC-elevated PowerShell. Iterating in a non-admin terminal is the
# common case; making the user spawn an admin shell every time would
# defeat the "quick" in quick.ps1.
#
# When -DeployDriver is on, the elevated block also copies the driver DLL
# and disables the legacy 01fingersmoothing driver (predecessor to the SC
# integration; its IVRDriverInputInternal hook would collide with ours).

if (-not (Test-Path $InstallPath)) {
    throw "Install path '$InstallPath' doesn't exist. Either install via the NSIS installer first, or pass -InstallPath '<your path>'."
}

# Files that go into the install dir. License / README / Uninstall.exe are
# NSIS install metadata -- we don't replace them on hot-swap, only the
# runtime artifacts that change build-to-build.
$RuntimeFiles = @("SpaceCalibrator.exe", "icon.png", "taskbar_icon.png", "manifest.vrmanifest", "openvr_api.dll")

$copyCmds = $RuntimeFiles | ForEach-Object {
    "Copy-Item -Force -Path '$ArtifactsDir\$_' -Destination '$InstallPath\$_'"
}

# If an instance is already running, remember that so we can re-launch it
# after the swap. The user's typical workflow is "leave the program open
# while iterating," so silently killing it without a relaunch is a worse
# experience than a brief flicker. We check from the unelevated side to
# avoid having the elevation path lose this state.
$wasRunning = @(Get-Process -Name SpaceCalibrator -ErrorAction SilentlyContinue).Count -gt 0
$installedExe = Join-Path $InstallPath "SpaceCalibrator.exe"

# Extra steps for -DeployDriver: also baked into the elevated block so
# everything that needs admin happens in a single UAC prompt. Driver DLL
# copy goes into <SteamVR>/drivers/01spacecalibrator/bin/win64/. The
# legacy FingerSmoothing manifest gets renamed (not deleted) so a future
# rollback is just rename-it-back. Skips both steps cleanly when their
# prerequisites are missing -- a fresh box without the legacy driver
# folder is fine.
$extraCmds = @()
if ($DeployDriver) {
    $extraCmds += "Copy-Item -Force -Path '$DriverDll' -Destination '$DestDriverDll'"
    $extraCmds += "Write-Host 'Copied driver DLL to $DestDriverDll'"
    $disabledManifest = "$LegacyFsManifest.disabled-by-quick-deploydriver"
    $extraCmds += "if (Test-Path '$LegacyFsManifest') { Move-Item -Force -Path '$LegacyFsManifest' -Destination '$disabledManifest'; Write-Host 'Disabled legacy 01fingersmoothing driver (renamed manifest).' }"
}

# Icon-cache refresh runs after the copy. ie4uinit's -show command tells
# Windows to re-read icon resources for known shortcuts; combined with the
# fresh EXE's last-write-time, the taskbar's pinned-shortcut icon updates
# without needing an explorer.exe restart in most cases.
$elevatedScript = @"
Stop-Process -Name SpaceCalibrator -Force -ErrorAction SilentlyContinue
Start-Sleep -Milliseconds 500
$($copyCmds -join "`n")
$($extraCmds -join "`n")
Write-Host 'Files copied. Refreshing icon cache...'
& ie4uinit.exe -show 2>&1 | Out-Null
Write-Host 'Done. Window will close in 2 seconds.'
Start-Sleep -Seconds 2
"@

Write-Host ""
Write-Host "Hot-swapping into: $InstallPath" -ForegroundColor Green
if ($wasRunning) {
    Write-Host "Detected running instance -- will re-launch after install." -ForegroundColor DarkGray
}
Write-Host "Approve the UAC prompt when it appears." -ForegroundColor DarkGray
Start-Process powershell -Verb RunAs -ArgumentList "-NoProfile","-Command",$elevatedScript -Wait

# Auto-relaunch DISABLED 2026-05-03 after a user report that tracking
# consistently broke when the script re-launched the overlay, but worked
# fine when the same EXE was launched from Start menu / pinned shortcut.
#
# Same exe, same driver, same loaded profile — only difference is the
# launcher. Hypothesis: parent process tree / session context affects how
# SteamVR's overlay registration handshakes the new SC instance. A
# Start-menu launch comes from explorer.exe; the script's Start-Process
# inherits the PowerShell -> CI/agent -> ... ancestry, which may produce
# subtly different IPC behaviour with vrserver. Without a confirmed root
# cause, the safest action is to not auto-launch and tell the user to do
# it themselves the way they know works.
#
# If you ever want this back, the place to look first is whether
# `Start-Process explorer.exe -ArgumentList $installedExe` (which makes
# explorer.exe the spawning parent, mimicking a double-click) reproduces
# the broken-tracking symptom or not.
if (-not $DeployDriver -and $wasRunning -and (Test-Path $installedExe)) {
    Write-Host ""
    Write-Host "WAS running before the swap, but auto-relaunch is disabled." -ForegroundColor Yellow
    Write-Host "Launch manually from Start menu / pinned shortcut to avoid the" -ForegroundColor Yellow
    Write-Host "tracking-breakage that script-launched instances exhibit." -ForegroundColor Yellow
}

# Sanity-check from the unelevated side: do the install-dir timestamps now
# match the build timestamps? If not, the elevated copy probably failed
# (cancelled UAC, locked file, etc.).
Write-Host ""
$buildTime = (Get-Item $Overlay).LastWriteTime
$installedExe = Join-Path $InstallPath "SpaceCalibrator.exe"
if (Test-Path $installedExe) {
    $installedTime = (Get-Item $installedExe).LastWriteTime
    if ($installedTime -ge $buildTime.AddSeconds(-2)) {
        Write-Host "Install timestamps match build. Launch from Start menu / pinned shortcut." -ForegroundColor Green
    } else {
        Write-Host "WARNING: installed EXE older than the build. Did UAC get cancelled?" -ForegroundColor Yellow
        Write-Host "  build:     $buildTime"
        Write-Host "  installed: $installedTime"
    }
}

# Driver-deploy: same timestamp sanity check on the destination DLL, then
# relaunch Steam (which kicks off SteamVR + auto-launches our overlay if it
# is registered).
if ($DeployDriver) {
    if (Test-Path $DestDriverDll) {
        $driverBuildTime     = (Get-Item $DriverDll).LastWriteTime
        $driverInstalledTime = (Get-Item $DestDriverDll).LastWriteTime
        if ($driverInstalledTime -ge $driverBuildTime.AddSeconds(-2)) {
            Write-Host "Driver DLL timestamps match build." -ForegroundColor Green
        } else {
            Write-Host "WARNING: installed driver DLL older than the build. UAC cancelled, or DLL still locked?" -ForegroundColor Yellow
            Write-Host "  build:     $driverBuildTime"
            Write-Host "  installed: $driverInstalledTime"
        }
    } else {
        Write-Host "WARNING: driver DLL not present at $DestDriverDll after install. Check the elevated output." -ForegroundColor Yellow
    }

    Write-Host ""
    Write-Host "Relaunching Steam..." -ForegroundColor Green
    Start-Process -FilePath $SteamExe
}
