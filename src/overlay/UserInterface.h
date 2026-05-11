#pragma once

#include <string>

void BuildMainWindow(bool runningInOverlay);
void CCal_DrawTab();
void RequestImmediateRedraw();
void RequestExit();

// True once the VR stack (OpenVR runtime + driver IPC + shmem) has connected
// and the overlay is ready for calibration. False at startup if SteamVR
// isn't running yet -- the main loop retries the init sequence and flips
// this to true when the connection lands. UI gates calibration-action
// buttons and device dropdowns on this so the user can still browse the
// settings tabs while waiting.
bool IsVRReady();

// Last human-readable error from a failed connection attempt, for display
// in the "Waiting for SteamVR" banner. Empty string before the first
// attempt; never reset to empty after that (we just overwrite on each
// retry).
const std::string& LastVRConnectError();
