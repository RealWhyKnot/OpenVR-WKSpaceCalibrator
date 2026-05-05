# Building

OpenVR-SpaceCalibrator builds from source on Windows with Visual Studio 2022 and CMake. The repo's `build.ps1` orchestrates everything: submodule init, version stamping, CMake configure, MSBuild, and zip packaging. Local rebuilds typically take 30-60 seconds (incremental).

## Prerequisites

- **Windows 10/11** (the build produces win64 binaries; the SteamVR driver is Windows-only).
- **Visual Studio 2022** with the *Desktop development with C++* workload. The "C++ build tools" alone is enough -- the IDE isn't required for headless builds.
- **CMake ≥ 3.8** (4.x works with `-DCMAKE_POLICY_VERSION_MINIMUM=3.5` for the minhook submodule). Either install the standalone CMake or rely on the one VS bundles.
- **Git**. Required to fetch submodules. The build also activates the repo's git hooks the first time it runs.
- **PowerShell 5.1+** (default on Windows 10/11). For running `build.ps1`.

## Quick start

```powershell
git submodule update --init --recursive
powershell -ExecutionPolicy Bypass -File build.ps1
```

This produces:

- `bin/artifacts/Release/SpaceCalibrator.exe` -- the overlay app.
- `bin/driver_01spacecalibrator/bin/win64/driver_01spacecalibrator.dll` -- the SteamVR driver.
- `release/OpenVR-SpaceCalibrator-<version>.zip` -- drop-in distribution: extract the `driver_01spacecalibrator/` folder into `<Steam>/steamapps/common/SteamVR/drivers/` and put `SpaceCalibrator.exe` wherever you want.

Pass `-SkipZip` to skip the release-zip step on rapid local rebuilds.

## Phases

`build.ps1` runs these in order:

### 1. Git hooks activation (idempotent)

Sets `git config --local core.hooksPath = .githooks` if not already configured. Activates two hooks:

- **`prepare-commit-msg`** -- appends the current build version (read from `version.txt`) to the commit subject in parens, e.g. `fix: handle ID reuse (2026.4.27.3-9F2A)`. Skipped for merge/squash messages.
- **`commit-msg`** -- rejects subjects with more than one build-version stamp (catches the editor-template autocomplete footgun where the previous commit's stamp gets autocompleted alongside the fresh one).

Bypass either with `--no-verify` if you really need to.

### 2. Submodule check

`build.ps1` verifies `lib/openvr/headers/openvr_driver.h` exists. If not, it runs `git submodule update --init --recursive` for you.

### 3. Version stamping

Two shapes are accepted:

- **Dev**: `YYYY.M.D.N-XXXX` -- `N` is the daily build counter, `XXXX` is a fresh 4-hex GUID prefix per build to disambiguate rebuilds at the same `N`. Local builds always emit this.
- **Release**: `YYYY.M.D.N` -- no suffix. Only emitted when CI passes a tag-derived `-Version` argument.

The version is written to `version.txt` in the repo root. The git `prepare-commit-msg` hook reads from there.

The semver "marketing" version stays in `src/common/Version.h` as `SPACECAL_VERSION_STRING`. That's the version displayed in the overlay's About panel and used by the NSIS installer. Bump it manually when cutting a fork release.

### 4. CMake configure + MSBuild

```
cmake -G "Visual Studio 17 2022" -A x64 -B bin -S . -DCMAKE_POLICY_VERSION_MINIMUM=3.5
cmake --build bin --config Release --parallel
```

Output goes to `bin/artifacts/Release/` (overlay + libs) and `bin/driver_01spacecalibrator/bin/win64/` (driver DLL with the manifest layout SteamVR expects).

### 5. Release zip (skippable with `-SkipZip`)

Packages the overlay + driver folder structure into `release/OpenVR-SpaceCalibrator-<version>.zip` and computes a SHA256 of the zip. The CI release workflow uses this hash in the GitHub release notes for verification.

## Submodules

- **`lib/glfw`** -- windowing.
- **`lib/imgui`** -- immediate-mode UI.
- **`lib/implot`** -- plotting in the Debug tab.
- **`lib/minhook`** -- function detour library used by the driver.
- **`lib/openvr`** -- Valve's OpenVR SDK headers and import lib.

`build.ps1` will refresh these on the first run if they're missing.

## Manual configure-only (no PowerShell)

The legacy path is the one-line `GenWin64.bat`:

```bat
cmake -G "Visual Studio 17 2022" -A x64 -B bin -S .
```

Then open `bin/SpaceCalibrator.sln` in Visual Studio and build the `Release|x64` configuration. This skips version stamping and zip packaging -- fine for IDE-driven development, not appropriate for cutting a release.

## CI

`.github/workflows/ci.yml` runs the build on `windows-latest` for every push to `main` and every PR. Build artifacts are uploaded so PR reviewers can grab the `.exe` + `.dll` without building locally.

`.github/workflows/release.yml` triggers on `v*` tags. Pushing `v2026.4.27.0` (or any valid stamp) builds the zip via `build.ps1 -Version 2026.4.27.0`, uploads it as a GitHub release, and includes the SHA256 in the release notes.

## Troubleshooting

- **"Compatibility with CMake < 3.5 has been removed."** The `lib/minhook` submodule's `CMakeLists.txt` declares an old minimum. `build.ps1` passes `-DCMAKE_POLICY_VERSION_MINIMUM=3.5` to work around this without modifying the submodule. If you bypass `build.ps1`, add the same flag.
- **"Unable to find current revision in submodule path 'lib/openvr'."** The pinned commit isn't in your local clone of the openvr submodule. Run `git -C lib/openvr fetch origin` (or `git fetch --all`) and rerun the submodule update.
- **Build succeeds but SteamVR doesn't see the driver.** Driver folder layout matters: SteamVR expects `<drivers>/driver_01spacecalibrator/bin/win64/driver_01spacecalibrator.dll` plus `driver.vrdrivermanifest` and `resources/`. The release zip ships this layout; manual builds need you to set up SteamVR's `drivers.cfg` to point at `bin/driver_01spacecalibrator/` directly, or copy the folder into SteamVR's `drivers/` directory.
