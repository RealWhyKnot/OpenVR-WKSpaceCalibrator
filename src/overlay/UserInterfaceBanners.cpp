#include "stdafx.h"
#include "UserInterfaceBanners.h"
#include "UserInterface.h"
#include "VRState.h"
#include "UpdateChecker.h"
#include "Updater.h"
#include "BuildStamp.h"

#include <string>
#include <shellapi.h>
#include <imgui/imgui.h>

extern bool s_inUmbrella;
extern spacecal::updates::UpdateChecker s_updateChecker;
extern spacecal::updates::Updater s_updater;
extern bool s_updateInitialKicked;
extern bool s_updateBannerDismissed;
extern double s_updateBannerHiddenUntil;

namespace spacecal::ui {

// One-line banner that announces the VR stack isn't connected yet. Clears
// automatically the moment SteamVR starts (the main loop retries the
// connection once per second). Kept compact because the user already
// knows what state they're in -- they launched the calibrator before
// starting SteamVR. No need to dump verbose error strings.
void DrawVRWaitingBanner() {
	if (IsVRReady()) return;
	// In umbrella mode the same status is already in the footer
	// (Driver: waiting for SteamVR). Don't duplicate it at the top of
	// the calibration tab where it pushes content down.
	if (s_inUmbrella) return;

	// Yellow-orange shade so it's distinct from the blue update banner --
	// "attention needed" rather than "FYI." Single-line height to stay
	// out of the way.
	ImGui::PushStyleColor(ImGuiCol_ChildBg, ImVec4(0.45f, 0.34f, 0.10f, 1.0f));
	ImGui::PushStyleColor(ImGuiCol_Border, ImVec4(0.95f, 0.78f, 0.30f, 1.0f));
	ImGui::PushStyleVar(ImGuiStyleVar_ChildBorderSize, 1.0f);
	ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(10.0f, 6.0f));

	const float bannerHeight = ImGui::GetFrameHeightWithSpacing() * 1.2f;
	if (ImGui::BeginChild("VRWaitingBanner",
			ImVec2(ImGui::GetContentRegionAvail().x, bannerHeight),
			ImGuiChildFlags_Border)) {
		ImGui::TextColored(ImVec4(1.0f, 0.95f, 0.80f, 1.0f),
			"Waiting for SteamVR -- calibration controls enable when tracking is live.");
	}
	ImGui::EndChild();

	ImGui::PopStyleVar(2);
	ImGui::PopStyleColor(2);

	ImGui::Spacing();
}

// Format a byte count as "1.23 MB" / "456 KB" / "789 B". Used by the
// update-banner progress display only.
static std::string FormatBytes(uint64_t n) {
	char buf[64];
	if (n >= (1ull << 30)) snprintf(buf, sizeof(buf), "%.2f GB", (double)n / (double)(1ull << 30));
	else if (n >= (1ull << 20)) snprintf(buf, sizeof(buf), "%.1f MB", (double)n / (double)(1ull << 20));
	else if (n >= (1ull << 10)) snprintf(buf, sizeof(buf), "%.0f KB", (double)n / (double)(1ull << 10));
	else snprintf(buf, sizeof(buf), "%llu B", (unsigned long long)n);
	return buf;
}

// Top-of-window banner that surfaces "update available", lets the user start
// the in-app upgrade, and reports progress. Only visible when:
//   - this is a release build (SPACECAL_BUILD_CHANNEL == "release"),
//   - the GitHub check completed and an update is available,
//   - the user hasn't dismissed the banner this session.
//
// On first call we kick the GitHub check; the rest of the banner is
// driven by polling UpdateChecker / Updater state every frame.
void DrawUpdateBanner() {
	using namespace spacecal::updates;

	// Only release builds nag for updates. Dev builds (`build.ps1` without
	// `-Version`) would just match against an arbitrary local stamp and
	// constantly suggest the user "downgrade" to the published release.
	const bool isReleaseBuild =
		std::string(SPACECAL_BUILD_CHANNEL) == "release";
	if (!isReleaseBuild) return;

	if (!s_updateInitialKicked) {
		s_updateChecker.CheckAsync();
		s_updateInitialKicked = true;
	}

	const auto checkerState = s_updateChecker.GetState();
	if (checkerState != State::HasResult) return;

	UpdateInfo info = s_updateChecker.GetResult();
	if (!info.available) return;

	const auto progress = s_updater.GetProgress();
	const bool busy =
		progress.state == DownloadState::Downloading ||
		progress.state == DownloadState::Verifying ||
		progress.state == DownloadState::Launching ||
		progress.state == DownloadState::Done;
	const bool failed = progress.state == DownloadState::Failed;

	if (s_updateBannerDismissed && !busy && !failed) return;

	// Background panel. Slightly different shade than the rest of the window so
	// it draws the eye without being alarming.
	ImGui::PushStyleColor(ImGuiCol_ChildBg, ImVec4(0.18f, 0.30f, 0.45f, 1.0f));
	ImGui::PushStyleColor(ImGuiCol_Border, ImVec4(0.40f, 0.65f, 0.95f, 1.0f));
	ImGui::PushStyleVar(ImGuiStyleVar_ChildBorderSize, 1.0f);
	ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(10.0f, 8.0f));

	const float bannerHeight = ImGui::GetFrameHeightWithSpacing() * 2.4f;
	if (ImGui::BeginChild("UpdateBanner",
			ImVec2(ImGui::GetContentRegionAvail().x, bannerHeight),
			ImGuiChildFlags_Border)) {

		if (progress.state == DownloadState::Done) {
			ImGui::TextColored(ImVec4(0.80f, 0.95f, 0.80f, 1.0f),
				"Installer launched. Closing Space Calibrator...");
			// Closing the program lets the installer replace the EXE without
			// fighting Windows' file locks. Fire-and-forget; one tick is plenty
			// to display the message before the main loop notices.
			RequestExit();
		} else if (busy) {
			const char* label = "Working";
			if (progress.state == DownloadState::Downloading) label = "Downloading installer";
			else if (progress.state == DownloadState::Verifying) label = "Verifying SHA-256";
			else if (progress.state == DownloadState::Launching) label = "Launching installer";

			float frac = 0.0f;
			if (progress.bytesTotal > 0)
				frac = (float)((double)progress.bytesReceived / (double)progress.bytesTotal);

			ImGui::Text("%s -- %s / %s", label,
				FormatBytes(progress.bytesReceived).c_str(),
				FormatBytes(progress.bytesTotal).c_str());

			if (progress.state == DownloadState::Downloading) {
				ImGui::ProgressBar(frac, ImVec2(-1.0f, 0.0f), "");
			} else {
				// Indeterminate-looking bar while we hash / launch -- full bar
				// is wrong (we're not done) but a thin pulsing one needs more
				// state than we want to track here. Just show 100% during the
				// final brief steps; they finish in well under a second.
				ImGui::ProgressBar(1.0f, ImVec2(-1.0f, 0.0f), "...");
			}
		} else if (failed) {
			ImGui::TextColored(ImVec4(1.0f, 0.55f, 0.55f, 1.0f),
				"Update failed: %s", progress.errorMessage.c_str());

			if (ImGui::Button("Retry")) {
				s_updater.DownloadAndLaunch(info.installerUrl, info.installerSha256, info.installerName);
			}
			ImGui::SameLine();
			if (ImGui::Button("Open release page")) {
				if (!info.releaseNotesUrl.empty())
					ShellExecuteA(nullptr, "open", info.releaseNotesUrl.c_str(), nullptr, nullptr, SW_SHOWNORMAL);
			}
			ImGui::SameLine();
			if (ImGui::Button("Dismiss")) {
				s_updateBannerDismissed = true;
			}
		} else {
			// Idle: an update is available, show the call-to-action.
			ImGui::Text("Update available: v%s (current: %s)",
				info.latestVersion.c_str(), SPACECAL_BUILD_STAMP);

			const bool canInstall =
				!info.installerUrl.empty() && !info.installerName.empty();

			ImGui::BeginDisabled(!canInstall);
			if (ImGui::Button("Update now")) {
				s_updater.DownloadAndLaunch(
					info.installerUrl, info.installerSha256, info.installerName);
			}
			ImGui::EndDisabled();
			if (!canInstall && ImGui::IsItemHovered()) {
				ImGui::SetTooltip("This release does not publish a Setup.exe asset.\n"
					"Use 'Release notes' to download a build manually.");
			}

			ImGui::SameLine();
			if (ImGui::Button("Release notes")) {
				if (!info.releaseNotesUrl.empty())
					ShellExecuteA(nullptr, "open", info.releaseNotesUrl.c_str(), nullptr, nullptr, SW_SHOWNORMAL);
			}
			ImGui::SameLine();
			if (ImGui::Button("Dismiss")) {
				s_updateBannerDismissed = true;
			}

			if (info.installerSha256.empty()) {
				ImGui::SameLine();
				ImGui::TextColored(ImVec4(0.95f, 0.85f, 0.40f, 1.0f),
					"  (no SHA published -- verify manually)");
			}
		}
	}
	ImGui::EndChild();

	ImGui::PopStyleVar(2);
	ImGui::PopStyleColor(2);

	ImGui::Spacing();
}

} // namespace spacecal::ui
