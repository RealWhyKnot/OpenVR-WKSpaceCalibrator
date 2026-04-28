# Contributing

Welcome, and thanks for taking an interest in this project. This is the WhyKnot fork of OpenVR-SpaceCalibrator. Bug reports, feature requests, and pull requests are all welcome — open an issue or PR against this repo and we'll take a look.

## Before you start

A bit of orientation goes a long way:

- Skim the [Architecture](https://github.com/RealWhyKnot/OpenVR-SpaceCalibrator/wiki/Architecture) and [Continuous Calibration](https://github.com/RealWhyKnot/OpenVR-SpaceCalibrator/wiki/Continuous-Calibration) wiki pages to get the lay of the land.
- For a bug report, the [Troubleshooting](https://github.com/RealWhyKnot/OpenVR-SpaceCalibrator/wiki/Troubleshooting) wiki page covers the common scenarios first — please check it before filing. If you do file an issue and it's a stuck-offset / drift problem, the issue template asks for specific log columns; capturing those up-front saves a round-trip.

## Setting up the build

**Prerequisites** (full details on the [Building](https://github.com/RealWhyKnot/OpenVR-SpaceCalibrator/wiki/Building) wiki page):

- Windows 10/11
- Visual Studio 2022 with the *Desktop development with C++* workload
- CMake (the one VS bundles is fine)
- git
- PowerShell 5.1+ (default on Windows 10/11)

**One-liner clone-to-build:**

```powershell
git submodule update --init --recursive
powershell -ExecutionPolicy Bypass -File build.ps1
```

The first run of `build.ps1` activates `.githooks/` for this clone automatically (it sets `git config --local core.hooksPath .githooks`). Two hooks ride along:

- `prepare-commit-msg` stamps the commit subject with the current build version, read from `version.txt` (e.g. `fix: handle ID reuse (2026.4.27.3-9F2A)`).
- `commit-msg` rejects subjects that contain more than one stamp — editor templates sometimes autocomplete a stale stamp from the previous commit alongside the fresh one, and that's almost never intentional.

If you ever genuinely need to bypass either, `git commit --no-verify` does it. Use sparingly.

**Iteration loop:** once you've configured once, `build.ps1 -SkipConfigure -SkipZip` is the fast path — it skips the CMake reconfigure (~10s) and the release-zip packaging step. Use it for tight edit-compile cycles; drop the flags when you want a clean build or a release artifact.

## Editing the wiki

The wiki is **source-controlled at `docs/wiki/`** in this repo. That means:

- Edits go through normal PR review, same as code.
- **Do not edit on the github.com Wiki UI.** Web edits get overwritten the next time the sync workflow runs.
- On every push to `develop` that touches `docs/wiki/**`, the [wiki-sync workflow](.github/workflows/wiki-sync.yml) mirrors the changes to the GitHub Wiki repo.

## Submitting a PR

- Branch from `develop`. Open the PR against `develop` (the fork's default branch).
- The [PR template](.github/PULL_REQUEST_TEMPLATE.md) auto-populates the description. Fill the checkboxes honestly — particularly the build-verification one (`build.ps1` succeeds end-to-end on the branch).
- **Protocol-changing PRs:** if you touch the driver/overlay IPC, bump `protocol::Version` in `src/common/Protocol.h` *and* update `docs/wiki/Driver-Protocol.md` to describe what changed in the new version. The two are part of the same change.
- Keep PRs focused. Mixing unrelated changes makes review harder and slower for everyone.

## Code review expectations

- The CI workflow (CMake configure + MSBuild on `windows-latest`) runs on every PR. Don't merge until it's green; if it goes red and the failure isn't obvious, leave a comment rather than force-pushing over it.
- Be ready to iterate. Expect at least one review pass on anything non-trivial.

## Commit message style

- Conventional-ish prefixes are appreciated but not enforced: `feat:`, `fix:`, `docs:`, `chore:`, `refactor:`, `ci:`.
- Keep the subject ≤72 characters. Remember the version stamp the hook appends counts toward that — leave a few characters of headroom.
- The body is for the *why*. The diff already shows the *what*; explain the reasoning, the alternatives you rejected, the gotcha you're working around.

## Reporting security issues

Please don't file a public issue for a security vulnerability. Use the repo's **Security tab → Report a vulnerability** for a private disclosure. See [SECURITY.md](.github/SECURITY.md) for details.
