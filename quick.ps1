param(
    # Skip the cmake configure step. Default ON for quick.ps1 — if you've already
    # configured once, MSBuild can rebuild from the existing solution. Pass
    # -Reconfigure to force a fresh cmake configure (needed after CMakeLists or
    # toolchain changes).
    [switch]$Reconfigure,

    # By default we skip both configure AND release-zip packaging so the
    # iteration loop is just "save -> run quick.ps1 -> launch". Pass -Zip to
    # also produce the drop-in distribution zip.
    [switch]$Zip
)

$ErrorActionPreference = "Stop"
Set-Location $PSScriptRoot

# Forward to build.ps1 with the right flags. quick.ps1 exists purely so the
# user has a one-word command to run while iterating. Default behaviour:
#   - skip cmake configure (it's slow and rarely needed when iterating)
#   - skip the release zip (we don't need a distribution artifact mid-iteration)
#   - leave version stamping ON so SPACECAL_BUILD_STAMP increments and the
#     in-app updater + git hooks can tell builds apart
$args = @()
if (-not $Reconfigure) { $args += "-SkipConfigure" }
if (-not $Zip)         { $args += "-SkipZip" }

Write-Host "quick.ps1: forwarding to build.ps1 $($args -join ' ')" -ForegroundColor DarkGray
& powershell.exe -ExecutionPolicy Bypass -File (Join-Path $PSScriptRoot "build.ps1") @args
$exitCode = $LASTEXITCODE
if ($exitCode -ne 0) {
    throw "build.ps1 failed (exit $exitCode)"
}

# Locate the built overlay so the user can copy-paste a launch command.
$Overlay = Join-Path $PSScriptRoot "bin/artifacts/Release/SpaceCalibrator.exe"
if (Test-Path $Overlay) {
    Write-Host ""
    Write-Host "Built overlay:  $Overlay" -ForegroundColor Green
    Write-Host ""
    Write-Host "Hot-swap into installed copy (run in admin shell):" -ForegroundColor DarkGray
    Write-Host "  Copy-Item -Force '$Overlay' 'C:\Program Files\OpenVR-SpaceCalibrator\SpaceCalibrator.exe'" -ForegroundColor DarkGray
    Write-Host ""
    Write-Host "Or run directly from bin/artifacts/Release/:" -ForegroundColor DarkGray
    Write-Host "  & '$Overlay'" -ForegroundColor DarkGray
}
