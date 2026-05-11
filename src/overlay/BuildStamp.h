// Tracked fallback so umbrella and direct cmake builds always resolve
// the include. build.ps1 overwrites this file with the real per-build
// YYYY.M.D.N-XXXX stamp the in-app updater compares against the latest
// GitHub release before producing a release artifact.
#pragma once

#define SPACECAL_BUILD_STAMP "0.0.0.0-DEV"
#define SPACECAL_BUILD_CHANNEL "dev"
