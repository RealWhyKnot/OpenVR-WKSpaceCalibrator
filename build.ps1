param(
    # When set, this overrides the auto-derived YYYY.M.D.N-XXXX stamp. The Release workflow
    # passes the git tag here (with the leading "v" stripped) so the published release's tag,
    # zip filename, and embedded version are all the same string. Local builds leave this
    # empty and get the auto-derived stamp from $Today + $BuildCount + a fresh GUID prefix.
    [string]$Version = "",

    # Skip the release zip packaging step. The zip is what the GitHub release workflow uploads
    # -- CI builds always want it. Local rebuilds usually don't, and accumulate one zip per
    # build in release/ otherwise. Pass -SkipZip to short-circuit the Compress-Archive at the
    # end. bin/ artifacts are still produced normally.
    [switch]$SkipZip,

    # Skip the CMake configure step (rerun MSBuild only). Useful when iterating on a single
    # source file -- saves the ~10s reconfigure. The CMake configure runs unconditionally
    # without this flag because that's the safe default; missed reconfigures cause stale
    # build files to mask source changes.
    [switch]$SkipConfigure
)

$ErrorActionPreference = "Stop"

# Pin the working directory to the script's own root so relative paths (src/..., bin/...)
# resolve consistently regardless of how the script is invoked.
Set-Location $PSScriptRoot

# Activate the repo's tracked git hooks (.githooks/) the first time the build runs in a clone.
# We do this from the build script -- rather than asking users to run `git config core.hooksPath`
# manually -- because forgetting the setup silently disables the hooks, which defeats the point of
# checks like the commit-msg version-stamp guard. Idempotent: only writes when it would actually
# change. If we're not inside a git checkout, the call no-ops harmlessly.
try {
    $current = (& git config --local --get core.hooksPath 2>$null)
    if ($LASTEXITCODE -eq 0 -and $current -ne ".githooks") {
        & git config --local core.hooksPath .githooks
        if ($LASTEXITCODE -eq 0) {
            Write-Host "Configured local git core.hooksPath = .githooks (commit-msg guard now active)." -ForegroundColor DarkGray
        }
    } elseif ($LASTEXITCODE -ne 0) {
        # `git config --get` returns 1 when the key is unset. Treat that as "needs setup" too.
        & git config --local core.hooksPath .githooks 2>$null
        if ($LASTEXITCODE -eq 0) {
            Write-Host "Configured local git core.hooksPath = .githooks (commit-msg guard now active)." -ForegroundColor DarkGray
        }
    }
} catch { }

# --- Submodule check ---
# Required submodules: glfw, imgui, implot, minhook, openvr. The build will fail with cryptic
# CMake errors if any are missing; check up-front and fix before configuring.
$openvrHeader = Join-Path $PSScriptRoot "lib/openvr/headers/openvr_driver.h"
if (!(Test-Path $openvrHeader)) {
    Write-Host "Submodules missing -- running 'git submodule update --init --recursive'..." -ForegroundColor Yellow
    $PrevEap = $ErrorActionPreference
    $ErrorActionPreference = "Continue"
    try {
        & git submodule update --init --recursive
        if ($LASTEXITCODE -ne 0) {
            throw "Submodule init failed (exit $LASTEXITCODE). Inspect the output above; you may need to fetch lib/openvr's recorded commit manually."
        }
    } finally {
        $ErrorActionPreference = $PrevEap
    }
}

# --- Daily Versioning Logic ---
# Two shapes are accepted:
#   - Release: YYYY.M.D.N           -- N = release iteration for the day (0, 1, 2, ...). No suffix.
#   - Dev:     YYYY.M.D.N-XXXX      -- N = local build count for the day; XXXX = 4-hex UID. The
#                                     suffix exists because a dev session produces many builds and
#                                     the UID disambiguates rebuilds at the same N.
# Local builds always emit the dev shape. Release-pipeline calls supply -Version explicitly with
# whichever shape is appropriate (release.yml passes the bare tag minus the leading "v").
# The semver "marketing" version stays in src/common/Version.h as SPACECAL_VERSION_STRING.
$LocalVersionState = Join-Path $PSScriptRoot "build/local_build_state.json"
$BuildStateDir = Split-Path $LocalVersionState -Parent
if (!(Test-Path $BuildStateDir)) { New-Item -ItemType Directory -Path $BuildStateDir -Force | Out-Null }

if ($Version) {
    if ($Version -notmatch '^\d{4}\.\d+\.\d+\.\d+(-[A-Fa-f0-9]{4})?$') {
        throw "Invalid -Version '$Version'. Expected YYYY.M.D.N (release) or YYYY.M.D.N-XXXX (dev, XXXX = 4 hex chars)."
    }
    $FullVersion = $Version
    Write-Host "Using caller-supplied version (skipping local-state increment)." -ForegroundColor DarkGray
} else {
    $Today = Get-Date -Format "yyyy.M.d"
    $BuildCount = 0
    $UID = [Guid]::NewGuid().ToString().Substring(0, 4).ToUpper()

    if (Test-Path $LocalVersionState) {
        $State = Get-Content $LocalVersionState | ConvertFrom-Json
        if ($State.Date -eq $Today) {
            $BuildCount = $State.Count + 1
        }
    }

    $FullVersion = "$Today.$BuildCount-$UID"
    @{ "Date" = $Today; "Count" = $BuildCount } | ConvertTo-Json | Out-File $LocalVersionState -Encoding UTF8
}

Write-Host "Building Version: $FullVersion" -ForegroundColor Magenta

# Emit version.txt at the repo root. The git prepare-commit-msg hook reads this to stamp
# commits. Use UTF-8 (no-BOM via -NoNewline + the hook's BOM stripper) -- PowerShell's default
# UTF-8 here is fine since the hook tolerates a BOM.
$FullVersion | Set-Content -Path (Join-Path $PSScriptRoot "version.txt") -Encoding UTF8 -NoNewline

# --- CMake configure ---
# The minhook submodule's CMakeLists declares cmake_minimum_required(2.8.12) which CMake 4.x
# rejects outright. -DCMAKE_POLICY_VERSION_MINIMUM=3.5 lets us configure without forking the
# submodule. Mirror this in .github/workflows/ci.yml.
if (-not $SkipConfigure) {
    Write-Host "`n--- CMake configure ---" -ForegroundColor Cyan
    $PrevEap = $ErrorActionPreference
    $ErrorActionPreference = "Continue"
    try {
        # Quoting matters: PowerShell's call operator splits `-DCMAKE_POLICY_VERSION_MINIMUM=3.5`
        # on the dot in some non-interactive environments — cmake then sees `3` and `.5` as
        # separate args, and the policy override is silently dropped. Pass it as a single
        # quoted token to keep PS from helpful tokenisation.
        & cmake -G "Visual Studio 17 2022" -A x64 -B bin -S . "-DCMAKE_POLICY_VERSION_MINIMUM=3.5"
        if ($LASTEXITCODE -ne 0) { throw "cmake configure failed (exit $LASTEXITCODE)" }
    } finally {
        $ErrorActionPreference = $PrevEap
    }
}

# --- MSBuild ---
Write-Host "`n--- Building (Release|x64) ---" -ForegroundColor Cyan
$PrevEap = $ErrorActionPreference
$ErrorActionPreference = "Continue"
try {
    & cmake --build bin --config Release --parallel
    if ($LASTEXITCODE -ne 0) { throw "cmake build failed (exit $LASTEXITCODE)" }
} finally {
    $ErrorActionPreference = $PrevEap
}

# --- Locate built artifacts ---
$OverlayExe = Join-Path $PSScriptRoot "bin/artifacts/Release/SpaceCalibrator.exe"
$DriverDir  = Join-Path $PSScriptRoot "bin/driver_01spacecalibrator"
$DriverDll  = Join-Path $DriverDir "bin/win64/driver_01spacecalibrator.dll"

if (!(Test-Path $OverlayExe)) { throw "Expected overlay exe not found at $OverlayExe" }
if (!(Test-Path $DriverDll))  { throw "Expected driver DLL not found at $DriverDll" }

# --- Release zip ---
# Drop-in distribution layout:
#   <zip-root>/SpaceCalibrator.exe
#   <zip-root>/driver_01spacecalibrator/...
#   <zip-root>/version.txt
# User extracts the zip anywhere, then copies driver_01spacecalibrator/ into Steam's
# steamapps/common/SteamVR/drivers/.
if ($SkipZip) {
    Write-Host "`nSkipping release zip (-SkipZip)." -ForegroundColor DarkGray
} else {
    Write-Host "`n--- Packaging release zip ---" -ForegroundColor Cyan

    $ReleaseDir = Join-Path $PSScriptRoot "release"
    if (!(Test-Path $ReleaseDir)) { New-Item -ItemType Directory -Path $ReleaseDir -Force | Out-Null }

    $StageDir = Join-Path $ReleaseDir "_stage_$FullVersion"
    if (Test-Path $StageDir) { Remove-Item -Recurse -Force $StageDir }
    New-Item -ItemType Directory -Path $StageDir -Force | Out-Null

    Copy-Item -Path $OverlayExe -Destination $StageDir
    Copy-Item -Path $DriverDir -Destination $StageDir -Recurse
    $FullVersion | Set-Content -Path (Join-Path $StageDir "version.txt") -Encoding UTF8 -NoNewline

    $ZipPath = Join-Path $ReleaseDir "OpenVR-SpaceCalibrator-$FullVersion.zip"
    if (Test-Path $ZipPath) { Remove-Item -Force $ZipPath }
    Compress-Archive -Path (Join-Path $StageDir "*") -DestinationPath $ZipPath -Force
    Remove-Item -Recurse -Force $StageDir

    $ZipHash = (Get-FileHash $ZipPath -Algorithm SHA256).Hash
    Write-Host "Release zip ready: $ZipPath" -ForegroundColor Green
    Write-Host "SHA256: $ZipHash" -ForegroundColor Green
}

Write-Host "`nBuild $FullVersion complete." -ForegroundColor Green
Write-Host "  Overlay: $OverlayExe" -ForegroundColor DarkGray
Write-Host "  Driver:  $DriverDll"  -ForegroundColor DarkGray
