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

    # Where the install lives. Override only if you've installed somewhere
    # non-default (custom NSIS install path, etc.).
    [string]$InstallPath = "C:\Program Files\SpaceCalibrator"
)

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
    Write-Host "  ./quick.ps1 -Install" -ForegroundColor DarkGray
    return
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

# Icon-cache refresh runs after the copy. ie4uinit's -show command tells
# Windows to re-read icon resources for known shortcuts; combined with the
# fresh EXE's last-write-time, the taskbar's pinned-shortcut icon updates
# without needing an explorer.exe restart in most cases.
$elevatedScript = @"
Stop-Process -Name SpaceCalibrator -Force -ErrorAction SilentlyContinue
Start-Sleep -Milliseconds 500
$($copyCmds -join "`n")
Write-Host 'Files copied. Refreshing icon cache...'
& ie4uinit.exe -show 2>&1 | Out-Null
Write-Host 'Done. Window will close in 2 seconds.'
Start-Sleep -Seconds 2
"@

Write-Host ""
Write-Host "Hot-swapping into: $InstallPath" -ForegroundColor Green
Write-Host "Approve the UAC prompt when it appears." -ForegroundColor DarkGray
Start-Process powershell -Verb RunAs -ArgumentList "-NoProfile","-Command",$elevatedScript -Wait

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
