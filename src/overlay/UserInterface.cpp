#include "stdafx.h"
#include "UserInterface.h"
#include "Calibration.h"
#include "Configuration.h"
#include "VRState.h"
#include "CalibrationMetrics.h"
#include "IPCClient.h"
#include "Protocol.h"
#include "Version.h"
#include "BuildStamp.h"
#include "UpdateChecker.h"
#include "Updater.h"
#include "MotionRecording.h"
#include "Wizard.h"

#include <thread>
#include <string>
#include <vector>
#include <algorithm>
#include <shellapi.h>
#include <shlobj_core.h>
#include <imgui/imgui.h>
#include "imgui_extensions.h"

extern IPCClient Driver;

void TextWithWidth(const char *label, const char *text, float width);
void DrawVectorElement(const std::string id, const char* text, double* value, int defaultValue = 0, const char* defaultValueStr = " 0 ");

VRState LoadVRState();
void BuildSystemSelection(const VRState &state);
void BuildDeviceSelections(const VRState &state);
void BuildProfileEditor();
void BuildMenu(bool runningInOverlay);

static const ImGuiWindowFlags bareWindowFlags =
	ImGuiWindowFlags_NoTitleBar |
	ImGuiWindowFlags_NoResize |
	ImGuiWindowFlags_NoMove |
	ImGuiWindowFlags_NoScrollbar |
	ImGuiWindowFlags_NoScrollWithMouse |
	ImGuiWindowFlags_NoCollapse;

void BuildContinuousCalDisplay();
void ShowVersionLine();
static void DrawModePill();
static void DrawUpdateBanner();
static void DrawVRWaitingBanner();
static void CCal_DrawLogsPanel();
static void DrawDiagnosticsPanel(ImVec2 panelSize);
static void DrawTipPanel(ImVec2 panelSize);

// Forward decls for the tab content called from both modes. CCal_BasicInfo /
// CCal_DrawSettings / CCal_DrawPredictionSuppression were declared near the
// continuous-mode tab bar; the non-continuous flow needs them in scope at
// BuildMainWindow time too, so the decls live at file scope.
void CCal_BasicInfo();
void CCal_DrawSettings();
void CCal_DrawPredictionSuppression();
static void OneShot_DrawSettings();

static bool runningInOverlay;

// Update-check state. Lives for the lifetime of the process. We kick off the
// initial check on the first BuildMainWindow tick so we don't slow startup.
// The Updater is reused across retries (it transitions back to Idle on
// failure when DownloadAndLaunch is called again — see Updater::DownloadAndLaunch).
static spacecal::updates::UpdateChecker s_updateChecker;
static spacecal::updates::Updater s_updater;
static bool s_updateInitialKicked = false;
static bool s_updateBannerDismissed = false;
static double s_updateBannerHiddenUntil = 0.0;

void BuildMainWindow(bool runningInOverlay_)
{
	runningInOverlay = runningInOverlay_;
	bool continuousCalibration = CalCtx.state == CalibrationState::Continuous || CalCtx.state == CalibrationState::ContinuousStandby;

	auto& io = ImGui::GetIO();

	ImGui::SetNextWindowPos(ImVec2(0.0f, 0.0f), ImGuiCond_Always);
	ImGui::SetNextWindowSize(io.DisplaySize, ImGuiCond_Always);

	if (!ImGui::Begin("SpaceCalibrator", nullptr, bareWindowFlags))
	{
		ImGui::End();
		return;
	}

	ImGui::PushStyleColor(ImGuiCol_PlotHistogram, ImGui::GetStyleColorVec4(ImGuiCol_Button));

	// Drawn before everything else so it's always visible; the banner is a
	// no-op until the first GitHub check completes (a few seconds after
	// startup) and an update is actually available.
	DrawUpdateBanner();

	// "Waiting for SteamVR" banner -- visible whenever the program is up
	// without a connected VR stack (e.g. user launched us before starting
	// SteamVR). Disappears the moment the connection lands. Renders after
	// the update banner because update checks work without VR; the VR
	// banner is the more transient state.
	DrawVRWaitingBanner();

	// First-run auto-open of the setup wizard. Defer until VR is ready --
	// the wizard's first step depends on enumerating tracking systems via
	// VRState::Load, which is empty without a live OpenVR connection. If
	// it auto-opened on a no-VR launch, the user would see an empty wizard
	// and have to dismiss it. With this gate, the wizard pops up the moment
	// VR comes online, which is what a first-time user would expect.
	{
		static bool s_firstRunChecked = false;
		if (!s_firstRunChecked && IsVRReady()) {
			s_firstRunChecked = true;
			if (!CalCtx.wizardCompleted && !spacecal::wizard::IsActive()) {
				spacecal::wizard::Open();
			}
		}
	}
	spacecal::wizard::Draw();

	if (continuousCalibration) {
		BuildContinuousCalDisplay();
	}
	else {
		// Persistent status pill at the top of the non-continuous main window
		// too, so a user with a fixed-offset profile sees the live state
		// without first opening continuous calibration.
		DrawModePill();
		ImGui::Spacing();

		auto state = LoadVRState();

		ImGui::BeginDisabled(CalCtx.state == CalibrationState::Continuous);
		BuildSystemSelection(state);
		BuildDeviceSelections(state);
		ImGui::EndDisabled();
		BuildMenu(runningInOverlay);

		// Non-continuous tabbed surface. Mirrors the depth of access
		// continuous-calibration users get -- the previous version showed
		// only the action buttons here, leaving Settings / Advanced /
		// Prediction / Logs reachable only after the user committed to
		// continuous mode. With this tab bar, a one-shot user can open
		// debug logs and tweak settings without ever clicking
		// "Continuous Calibration".
		//
		// Hidden during the in-progress calibration popup (state != None)
		// because the user is captured by the modal anyway and the tabs
		// would just clutter the background.
		if (CalCtx.state == CalibrationState::None) {
			ImGui::Spacing();
			if (ImGui::BeginTabBar("OneShotTabs", 0)) {
				if (ImGui::BeginTabItem("Settings")) {
					OneShot_DrawSettings();
					ImGui::EndTabItem();
				}
				if (ImGui::BeginTabItem("Advanced")) {
					CCal_DrawSettings();
					ImGui::EndTabItem();
				}
				if (ImGui::BeginTabItem("Prediction")) {
					CCal_DrawPredictionSuppression();
					ImGui::EndTabItem();
				}
				if (ImGui::BeginTabItem("Logs")) {
					CCal_DrawLogsPanel();
					ImGui::EndTabItem();
				}
				ImGui::EndTabBar();
			}
		}
	}

	ShowVersionLine();

	ImGui::PopStyleColor();
	ImGui::End();
}

// Render a small filled circle aligned with the current text baseline. Used as
// the connection-status dot in the version line. Drawn directly to the window
// draw list so we don't have to fiddle with widget sizing — the caller manages
// SameLine/spacing.
static void DrawStatusDot(ImU32 color, float radiusScale = 0.32f) {
	ImDrawList* dl = ImGui::GetWindowDrawList();
	const float h = ImGui::GetTextLineHeight();
	const float r = h * radiusScale;
	ImVec2 cursor = ImGui::GetCursorScreenPos();
	// Center vertically on the text line; nudge left/right so the dot sits in
	// its own little gutter rather than overlapping the next character.
	ImVec2 center(cursor.x + r + 2.0f, cursor.y + h * 0.5f);
	dl->AddCircleFilled(center, r, color);
	// Reserve the space the dot occupies so SameLine() spacing works.
	ImGui::Dummy(ImVec2(r * 2.0f + 4.0f, h));
	ImGui::SameLine();
}

void ShowVersionLine() {
	ImGui::SetNextWindowPos(ImVec2(10.0f, ImGui::GetWindowHeight() - ImGui::GetFrameHeightWithSpacing()));
	if (!ImGui::BeginChild("bottom line", ImVec2(ImGui::GetWindowWidth() - 20.0f, ImGui::GetFrameHeightWithSpacing() * 2), ImGuiChildFlags_None)) {
		ImGui::EndChild();
		return;
	}

	// Driver-connection dot. Three states:
	//   green  - connected, everything's working.
	//   amber  - VR stack hasn't connected yet (SteamVR not running, or the
	//            program was launched standalone). Not an error; the main
	//            loop is retrying and will flip to green automatically.
	//   red    - established connection has dropped (IPC pipe broke after
	//            initial handshake). This is the case where reinstalling the
	//            driver is genuinely the right advice.
	// The version we show is the build-time client protocol version --
	// IPCClient::Connect() throws unless the driver reports the same number,
	// so when IsConnected() is true that number is also the live driver
	// version.
	const bool driverConnected = Driver.IsConnected();
	if (driverConnected) {
		DrawStatusDot(IM_COL32(80, 200, 120, 255));
		ImGui::TextColored(ImVec4(0.5f, 0.85f, 0.55f, 1.0f),
			"Driver: connected (v%u)", (unsigned)protocol::Version);
	} else if (!IsVRReady()) {
		// Pre-connection. Not an error; SteamVR just isn't up yet.
		DrawStatusDot(IM_COL32(220, 170, 60, 255));
		ImGui::TextColored(ImVec4(0.95f, 0.80f, 0.40f, 1.0f),
			"Driver: waiting for SteamVR");
	} else {
		// We did successfully connect to OpenVR (so IsVRReady is true), but the
		// IPC pipe is down. That means SteamVR is running without our driver
		// loaded -- typically a missing or broken driver install.
		DrawStatusDot(IM_COL32(220, 80, 80, 255));
		ImGui::TextColored(ImVec4(0.95f, 0.45f, 0.45f, 1.0f),
			"Driver: disconnected — reinstall the SteamVR driver");
	}

	ImGui::SameLine();
	ImGui::Text("  |  Space Calibrator v" SPACECAL_VERSION_STRING);
	if (runningInOverlay)
	{
		ImGui::SameLine();
		ImGui::Text("- close VR overlay to use mouse");
	}
	ImGui::EndChild();
}

// Render the persistent rounded "mode pill" that summarises the current
// calibration state. Called from BuildContinuousCalDisplay (and BuildMainWindow
// for the non-continuous flows) so the user always sees at a glance whether a
// fixed offset is live, continuous mode is updating, etc.
static void DrawModePill() {
	const auto state = CalCtx.state;
	const bool validProfile = CalCtx.validProfile;
	const bool enabled = CalCtx.enabled;

	const char* label = nullptr;
	const char* tooltip = nullptr;
	ImVec4 textColor;
	ImVec4 bgColor;

	if (!validProfile) {
		label = "[NO PROFILE]";
		tooltip = "No saved calibration profile is loaded.\n"
		          "Hit \"Start Calibration\" or \"Continuous Calibration\" below to create one.";
		textColor = ImVec4(0.85f, 0.85f, 0.85f, 1.0f);
		bgColor   = ImVec4(0.30f, 0.30f, 0.30f, 1.0f);
	} else if (state == CalibrationState::ContinuousStandby) {
		label = "[STANDBY — waiting for tracking]";
		tooltip = "Continuous calibration is on, but the reference or target tracker isn't currently\n"
		          "reporting valid poses. Calibration resumes automatically when both come back online.";
		textColor = ImVec4(0.95f, 0.95f, 0.95f, 1.0f);
		bgColor   = ImVec4(0.40f, 0.40f, 0.42f, 1.0f);
	} else if (state == CalibrationState::Continuous) {
		// Amber when we've been rejecting samples for a while — calibration is
		// nominally running but hasn't accepted anything recently.
		const double now = ImGui::GetTime();
		const double sinceAccept = now - Metrics::error_currentCal.lastTs();
		const bool searching = Metrics::consecutiveRejections.last() > 10.0;
		const bool recentlyUpdated = sinceAccept >= 0.0 && sinceAccept < 5.0;
		if (searching) {
			label = "[LIVE — searching]";
			tooltip = "Continuous calibration is running but hasn't accepted a new estimate in a while.\n"
			          "Usually means the user isn't moving enough to give the solver useful samples.\n"
			          "Try slowly rotating + translating the target tracker through varied directions.";
			textColor = ImVec4(0.10f, 0.10f, 0.10f, 1.0f);
			bgColor   = ImVec4(0.95f, 0.70f, 0.20f, 1.0f);
		} else if (recentlyUpdated) {
			label = "[LIVE — updating]";
			tooltip = "Continuous calibration is running and just accepted a fresh estimate.\n"
			          "The driver is blending toward the new offset; if Recalibrate-on-movement is on,\n"
			          "the blend only progresses while the device is actively moving.";
			textColor = ImVec4(0.95f, 0.95f, 1.00f, 1.0f);
			bgColor   = ImVec4(0.20f, 0.50f, 0.85f, 1.0f);
		} else {
			label = "[LIVE]";
			tooltip = "Continuous calibration is running and the current estimate is being applied.\n"
			          "No recent updates needed — the calibration is stable.";
			textColor = ImVec4(0.95f, 0.95f, 1.00f, 1.0f);
			bgColor   = ImVec4(0.25f, 0.45f, 0.75f, 1.0f);
		}
	} else if (enabled && state == CalibrationState::None) {
		label = "[FIXED OFFSET ACTIVE]";
		tooltip = "A one-shot calibration is applied as a fixed offset. The driver applies the\n"
		          "stored transform; no continuous re-solving. Switch to Continuous mode if the\n"
		          "offset drifts over time.";
		textColor = ImVec4(0.10f, 0.20f, 0.10f, 1.0f);
		bgColor   = ImVec4(0.45f, 0.80f, 0.45f, 1.0f);
	} else {
		// validProfile but not enabled and not in a continuous state — treat as
		// idle/no-op rather than hide the pill entirely so the user always has
		// a status they can point to in a bug report.
		label = "[IDLE]";
		tooltip = "A profile is loaded but no calibration is being applied. This usually means the\n"
		          "current HMD tracking system doesn't match the profile's reference system.";
		textColor = ImVec4(0.85f, 0.85f, 0.85f, 1.0f);
		bgColor   = ImVec4(0.30f, 0.30f, 0.30f, 1.0f);
	}

	// Draw a rounded filled rectangle behind the label. Kept compact so the pill
	// doesn't push the rest of the UI down too much.
	const ImVec2 textSize = ImGui::CalcTextSize(label);
	const ImVec2 padding(10.0f, 4.0f);
	ImVec2 cursor = ImGui::GetCursorScreenPos();
	ImVec2 rectMin = cursor;
	ImVec2 rectMax(cursor.x + textSize.x + padding.x * 2.0f,
	               cursor.y + textSize.y + padding.y * 2.0f);
	ImDrawList* dl = ImGui::GetWindowDrawList();
	dl->AddRectFilled(rectMin, rectMax, ImGui::GetColorU32(bgColor), 8.0f);
	dl->AddText(ImVec2(rectMin.x + padding.x, rectMin.y + padding.y),
	            ImGui::GetColorU32(textColor), label);
	// Use a Dummy widget for layout reservation AND hover-detection. ImGui's
	// IsItemHovered() checks the last submitted widget, so the Dummy must come
	// before the SetTooltip — which is the call below.
	ImGui::Dummy(ImVec2(textSize.x + padding.x * 2.0f, textSize.y + padding.y * 2.0f));
	if (tooltip && ImGui::IsItemHovered()) {
		ImGui::SetTooltip("%s", tooltip);
	}
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
// One-line banner that announces the VR stack isn't connected yet. Clears
// automatically the moment SteamVR starts (the main loop retries the
// connection once per second). Kept compact because the user already
// knows what state they're in -- they launched the calibrator before
// starting SteamVR. No need to dump verbose error strings.
static void DrawVRWaitingBanner() {
	if (IsVRReady()) return;

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
			"Waiting for SteamVR — calibration controls enable when tracking is live.");
	}
	ImGui::EndChild();

	ImGui::PopStyleVar(2);
	ImGui::PopStyleColor(2);

	ImGui::Spacing();
}

static void DrawUpdateBanner() {
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
				"Installer launched. Closing Space Calibrator…");
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

			ImGui::Text("%s — %s / %s", label,
				FormatBytes(progress.bytesReceived).c_str(),
				FormatBytes(progress.bytesTotal).c_str());

			if (progress.state == DownloadState::Downloading) {
				ImGui::ProgressBar(frac, ImVec2(-1.0f, 0.0f), "");
			} else {
				// Indeterminate-looking bar while we hash / launch — full bar
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
					"  (no SHA published — verify manually)");
			}
		}
	}
	ImGui::EndChild();

	ImGui::PopStyleVar(2);
	ImGui::PopStyleColor(2);

	ImGui::Spacing();
}

void BuildContinuousCalDisplay() {
	ImGui::SetNextWindowPos(ImVec2(0, 0));
	ImGui::SetNextWindowSize(ImGui::GetWindowSize());
	ImGui::SetNextWindowBgAlpha(1);
	if (!ImGui::Begin("Continuous Calibration", nullptr,
		bareWindowFlags & ~ImGuiWindowFlags_NoTitleBar
	)) {
		ImGui::End();
		return;
	}

	ImVec2 contentRegion;
	contentRegion.x = ImGui::GetWindowContentRegionWidth();
	contentRegion.y = ImGui::GetWindowHeight() - ImGui::GetFrameHeightWithSpacing() * 2.1f;

	if (!ImGui::BeginChild("CCalDisplayFrame", contentRegion, ImGuiChildFlags_None)) {
		ImGui::EndChild();
		ImGui::End();
		return;
	}

	// Persistent mode pill above the tab bar -- surfaces the high-level state
	// (LIVE updating, searching, standby, fixed-offset) at a glance regardless
	// of which tab the user is looking at.
	DrawModePill();
	ImGui::Spacing();

	// Tab bar layout.  The user-facing categories are:
	//   - Basic:    everything a casual user touches.  No graphs, no jargon.
	//               Plain buttons + the handful of settings most people change.
	//   - Graphs:   the live plots.  For users who want to watch what the math
	//               is doing in real time.
	//   - Advanced: every technical knob -- speed radios, alignment thresholds,
	//               latency tuning, the obscure checkboxes.  This is also the
	//               only place where a user can override the AUTO defaults.
	//   - Prediction: stays separate since it's its own feature surface
	//               (external-tool detection + per-device suppression).
	//   - Logs:     list debug-log CSV files for sending to bug reports;
	//               always visible (user may have old logs to attach even
	//               with logging currently off).
	if (ImGui::BeginTabBar("CCalTabs", 0)) {
		if (ImGui::BeginTabItem("Basic")) {
			CCal_BasicInfo();
			ImGui::EndTabItem();
		}

		if (ImGui::BeginTabItem("Graphs")) {
			ShowCalibrationDebug(2, 3);
			ImGui::EndTabItem();
		}

		if (ImGui::BeginTabItem("Advanced")) {
			CCal_DrawSettings();
			ImGui::EndTabItem();
		}

		if (ImGui::BeginTabItem("Prediction")) {
			CCal_DrawPredictionSuppression();
			ImGui::EndTabItem();
		}

		// Logs tab: always visible. Even when debug logging is OFF right now,
		// the user might have older log files to attach to a bug report. The
		// panel shows the toggle's state alongside the file list, so it's
		// obvious whether a fresh log is being captured.
		if (ImGui::BeginTabItem("Logs")) {
			CCal_DrawLogsPanel();
			ImGui::EndTabItem();
		}

		ImGui::EndTabBar();
	}

	ImGui::EndChild();

	ShowVersionLine();

	ImGui::End();
}

static void ScaledDragFloat(const char* label, double& f, double scale, double min, double max, int flags = ImGuiSliderFlags_AlwaysClamp) {
	float v = (float) (f * scale);
	std::string labelStr = std::string(label);

	// If starts with ##, just do a normal SliderFloat
	if (labelStr.size() > 2 && labelStr[0] == '#' && labelStr[1] == '#') {
		ImGui::SliderFloat(label, &v, (float)min, (float)max, "%1.2f", flags);
	} else {
		// Otherwise do funny
		ImGui::Text(label);
		ImGui::SameLine();
		ImGui::PushID((std::string(label) + "_id").c_str());
		// Line up to a column, multiples of 100
		constexpr uint32_t LABEL_CURSOR = 100;
		uint32_t cursorPosX = (int) ImGui::GetCursorPosX();
		uint32_t roundedPosition = ((cursorPosX + LABEL_CURSOR / 2) / LABEL_CURSOR) * LABEL_CURSOR;
		ImGui::SetCursorPosX((float) roundedPosition);
		ImGui::SliderFloat((std::string("##") + label).c_str(), &v, (float)min, (float)max, "%1.2f", flags);
		ImGui::PopID();
	}
	
	f = v / scale;
}

// Helper for the per-slider "Reset to default" right-click context menu added in
// the settings split. Always pops up on right-click of the slider that called
// AddDefaultsContextMenu() last (i.e. the previous widget in submission order).
// resetFn is run when the user picks "Reset to default".
template<typename Fn>
static void AddResetContextMenu(const char* popupId, Fn resetFn) {
	if (ImGui::BeginPopupContextItem(popupId)) {
		if (ImGui::MenuItem("Reset to default")) {
			resetFn();
		}
		ImGui::EndPopup();
	}
}

void CCal_DrawSettings() {

	// panel size for boxes
	ImVec2 panel_size { ImGui::GetWindowContentRegionMax().x - ImGui::GetWindowContentRegionMin().x, 0 };

	// Tip first (consistent with Basic). Tells the user about hover tooltips
	// and right-click reset, which is even more relevant in Advanced where
	// almost every row has a slider.
	DrawTipPanel(panel_size);

	// Diagnostics panel: stuck-loop watchdog + HMD-stall purge counters.
	// Lives in Advanced because it's bug-report breadcrumbs, not something
	// a casual user is going to action on. Keeping it visible (rather than
	// behind another collapsing header) so a user with a problem can copy
	// the numbers into a bug report without spelunking.
	DrawDiagnosticsPanel(panel_size);

	// === Advanced toggles =================================================
	// Power-user checkboxes that aren't worth Basic real estate.  Kept at the
	// top of the Advanced tab because they're settings, and Advanced users
	// expect to find them quickly without scrolling past the speed matrix.
	if (ImGui::Checkbox("Hide tracker", &CalCtx.quashTargetInContinuous)) {
		SaveProfile(CalCtx);
	}
	if (ImGui::IsItemHovered()) {
		ImGui::SetTooltip("Suppress the target tracker's pose in OpenVR while continuous calibration runs.\n"
		                  "Use when the target tracker would otherwise show up as a duplicate of the reference\n"
		                  "(e.g. taping a Vive tracker to a Quest controller for calibration).");
	}
	ImGui::SameLine();
	if (ImGui::Checkbox("Static recalibration", &CalCtx.enableStaticRecalibration)) {
		SaveProfile(CalCtx);
	}
	if (ImGui::IsItemHovered()) {
		ImGui::SetTooltip("Use the locked reference->target relative pose for fast \"snap-back\" corrections.\n"
		                  "When the live solver's estimate diverges noticeably from the locked relative pose,\n"
		                  "we snap to the locked solution instead of waiting for incremental convergence.");
	}
	ImGui::SameLine();
	ImGui::Checkbox("Ignore outliers", &CalCtx.ignoreOutliers);
	if (ImGui::IsItemHovered()) {
		ImGui::SetTooltip("Drop sample pairs whose rotation axis disagrees with the consensus before the LS solve.\n"
		                  "Default on.  Turn off only if you suspect the outlier rejector is throwing out good samples\n"
		                  "(e.g. genuinely jittery motion the cosine-similarity test mistakes for outliers).");
	}
	ImGui::Spacing();

	// === ADVANCED SETTINGS ==================================================
	// All technical knobs.  No longer behind a CollapsingHeader: the tab
	// itself is now the gate.  A user clicking "Advanced" is signalling
	// they want to see everything at once, so flatten the hierarchy.
	{

		// Calibration speed radio + speed-threshold matrix
		{
			ImGui::BeginGroupPanel("Calibration speeds", panel_size);

			ImGui::PushStyleColor(ImGuiCol_Text, ImGui::GetStyleColorVec4(ImGuiCol_TextDisabled));
			ImGui::TextWrapped(
				"SpaceCalibrator uses up to three different speeds at which it drags the calibration back into "
				"position when drift occurs. These settings control how far off the calibration should be before going back to low speed (for "
				"Decel) or going to higher speeds (for Slow and Fast)."
			);
			ImGui::PopStyleColor();

			// Calibration Speed radio
			{
				ImGui::BeginGroupPanel("Calibration speed", panel_size);

				auto speed = CalCtx.calibrationSpeed;

				ImGui::Columns(4, nullptr, false);
				if (ImGui::RadioButton(" Auto          ", speed == CalibrationContext::AUTO)) {
					CalCtx.calibrationSpeed = CalibrationContext::AUTO;
					SaveProfile(CalCtx);
				}
				if (ImGui::IsItemHovered()) {
					ImGui::SetTooltip("Pick the speed automatically based on observed tracker jitter.\n"
					                  "Sub-mm jitter -> Fast.  1-5mm -> Slow.  Above 5mm -> Very Slow.\n"
					                  "Re-evaluates while continuous calibration runs; sticky so it doesn't oscillate.\n"
					                  "Recommended for everyone except people who want a specific speed for a reason.");
				}
				ImGui::NextColumn();
				if (ImGui::RadioButton(" Fast          ", speed == CalibrationContext::FAST)) {
					CalCtx.calibrationSpeed = CalibrationContext::FAST;
					SaveProfile(CalCtx);
				}
				if (ImGui::IsItemHovered()) {
					ImGui::SetTooltip("100-sample buffer. Fastest convergence, most sensitive to noise.\n"
					                  "Good for clean lighthouse setups.");
				}
				ImGui::NextColumn();
				if (ImGui::RadioButton(" Slow          ", speed == CalibrationContext::SLOW)) {
					CalCtx.calibrationSpeed = CalibrationContext::SLOW;
					SaveProfile(CalCtx);
				}
				if (ImGui::IsItemHovered()) {
					ImGui::SetTooltip("250-sample buffer. Smoother result at the cost of slower response.\n"
					                  "Good for typical mixed setups.");
				}
				ImGui::NextColumn();
				if (ImGui::RadioButton(" Very Slow     ", speed == CalibrationContext::VERY_SLOW)) {
					CalCtx.calibrationSpeed = CalibrationContext::VERY_SLOW;
					SaveProfile(CalCtx);
				}
				if (ImGui::IsItemHovered()) {
					ImGui::SetTooltip("500-sample buffer. Maximum smoothing, slowest convergence.\n"
					                  "For noisy / reflective rooms or drift-prone IMU trackers.");
				}
				ImGui::Columns(1);

				// Show the resolved speed when AUTO is on so the user understands
				// what the program decided. Faded text so it doesn't draw the eye.
				if (speed == CalibrationContext::AUTO) {
					const auto resolved = CalCtx.ResolvedCalibrationSpeed();
					const char* resolvedName =
						resolved == CalibrationContext::FAST ? "Fast" :
						resolved == CalibrationContext::SLOW ? "Slow" :
						resolved == CalibrationContext::VERY_SLOW ? "Very Slow" : "?";
					ImGui::PushStyleColor(ImGuiCol_Text, ImGui::GetStyleColorVec4(ImGuiCol_TextDisabled));
					ImGui::Text("    Currently resolved to: %s  (jitter ref %.2f mm, target %.2f mm)",
					            resolvedName,
					            Metrics::jitterRef.last() * 1000.0,
					            Metrics::jitterTarget.last() * 1000.0);
					ImGui::PopStyleColor();
				}

				ImGui::EndGroupPanel();
			}

			if (ImGui::BeginTable("SpeedThresholds", 3, 0)) {
				ImGui::TableNextRow();
				ImGui::TableSetColumnIndex(1);
				ImGui::Text("Translation (mm)");
				ImGui::TableSetColumnIndex(2);
				ImGui::Text("Rotation (degrees)");


				ImGui::TableNextRow();
				ImGui::TableSetColumnIndex(0);
				ImGui::Text("Decel");
				ImGui::TableSetColumnIndex(1);
				ScaledDragFloat("##TransDecel", CalCtx.alignmentSpeedParams.thr_trans_tiny, 1000.0, 0, 20.0);
				AddResetContextMenu("trans_decel_ctx", [] { CalCtx.alignmentSpeedParams.thr_trans_tiny = 0.98f / 1000.0f; });
				ImGui::TableSetColumnIndex(2);
				ScaledDragFloat("##RotDecel", CalCtx.alignmentSpeedParams.thr_rot_tiny, 180.0 / EIGEN_PI, 0, 5.0);
				AddResetContextMenu("rot_decel_ctx", [] { CalCtx.alignmentSpeedParams.thr_rot_tiny = 0.49f * (float)(EIGEN_PI / 180.0f); });

				ImGui::TableNextRow();
				ImGui::TableSetColumnIndex(0);
				ImGui::Text("Slow");
				ImGui::TableSetColumnIndex(1);
				ScaledDragFloat("##TransSlow", CalCtx.alignmentSpeedParams.thr_trans_small, 1000.0,
					CalCtx.alignmentSpeedParams.thr_trans_tiny * 1000.0, 20.0);
				AddResetContextMenu("trans_slow_ctx", [] { CalCtx.alignmentSpeedParams.thr_trans_small = 1.0f / 1000.0f; });
				ImGui::TableSetColumnIndex(2);
				ScaledDragFloat("##RotSlow", CalCtx.alignmentSpeedParams.thr_rot_small, 180.0 / EIGEN_PI,
					CalCtx.alignmentSpeedParams.thr_rot_tiny * (180.0 / EIGEN_PI), 10.0);
				AddResetContextMenu("rot_slow_ctx", [] { CalCtx.alignmentSpeedParams.thr_rot_small = 0.5f * (float)(EIGEN_PI / 180.0f); });

				ImGui::TableNextRow();
				ImGui::TableSetColumnIndex(0);
				ImGui::Text("Fast");
				ImGui::TableSetColumnIndex(1);
				ScaledDragFloat("##TransFast", CalCtx.alignmentSpeedParams.thr_trans_large, 1000.0,
					CalCtx.alignmentSpeedParams.thr_trans_small * 1000.0, 50.0);
				AddResetContextMenu("trans_fast_ctx", [] { CalCtx.alignmentSpeedParams.thr_trans_large = 20.0f / 1000.0f; });
				ImGui::TableSetColumnIndex(2);
				ScaledDragFloat("##RotFast", CalCtx.alignmentSpeedParams.thr_rot_large, 180.0 / EIGEN_PI,
					CalCtx.alignmentSpeedParams.thr_rot_small * (180.0 / EIGEN_PI), 20.0);
				AddResetContextMenu("rot_fast_ctx", [] { CalCtx.alignmentSpeedParams.thr_rot_large = 5.0f * (float)(EIGEN_PI / 180.0f); });

				ImGui::EndTable();
			}

			ImGui::EndGroupPanel();
		}

		// Alignment speeds (Decel/Slow/Fast)
		{
			ImGui::BeginGroupPanel("Alignment speeds", panel_size);

			ScaledDragFloat("Decel", CalCtx.alignmentSpeedParams.align_speed_tiny, 1.0, 0, 2.0, 0);
			AddResetContextMenu("align_decel_ctx", [] { CalCtx.alignmentSpeedParams.align_speed_tiny = 1.0f; });
			ScaledDragFloat("Slow", CalCtx.alignmentSpeedParams.align_speed_small, 1.0, 0, 2.0, 0);
			AddResetContextMenu("align_slow_ctx", [] { CalCtx.alignmentSpeedParams.align_speed_small = 1.0f; });
			ScaledDragFloat("Fast", CalCtx.alignmentSpeedParams.align_speed_large, 1.0, 0, 2.0, 0);
			AddResetContextMenu("align_fast_ctx", [] { CalCtx.alignmentSpeedParams.align_speed_large = 2.0f; });

			ImGui::EndGroupPanel();
		}

		// Other advanced sliders
		{
			ImGui::BeginGroupPanel("Continuous calibration (advanced)", panel_size);

			// Max relative error threshold
			ImGui::Text("Max relative error threshold");
			ImGui::SameLine();
			ImGui::PushID("max_relative_error_threshold");
			ImGui::SliderFloat("##max_relative_error_threshold_slider", &CalCtx.maxRelativeErrorThreshold, 0.01f, 1.0f, "%1.1f", 0);
			if (ImGui::IsItemHovered(0)) {
				ImGui::SetTooltip("Controls the maximum acceptable relative error. If the error from the relative calibration is too poor, the calibration will be discarded.");
			}
			AddResetContextMenu("max_rel_err_ctx", [] { CalCtx.maxRelativeErrorThreshold = 0.005f; });
			ImGui::PopID();

			// Target latency offset (manual)
			ImGui::Text("Target latency offset (ms)");
			ImGui::SameLine();
			ImGui::PushID("target_latency_offset");
			{
				float latencyMs = (float)CalCtx.targetLatencyOffsetMs;
				if (ImGui::SliderFloat("##target_latency_offset_slider", &latencyMs, -100.0f, 100.0f, "%.1f", 0)) {
					CalCtx.targetLatencyOffsetMs = (double)latencyMs;
				}
			}
			if (ImGui::IsItemHovered(0)) {
				ImGui::SetTooltip("Manual end-to-end-latency offset for the target tracking system, in milliseconds.\n"
					"Use this when the target system (e.g. Slime IMU, Quest) lags the reference (e.g. Lighthouse).\n"
					"At sample-collection time the reference pose is extrapolated by this amount using its\n"
					"reported velocity, so quick motions don't bias the calibration.\n"
					"Default 0 disables the feature. Auto-detection is on the roadmap.");
			}
			AddResetContextMenu("target_latency_ctx", [] { CalCtx.targetLatencyOffsetMs = 0.0; });
			ImGui::PopID();

			ImGui::EndGroupPanel();
		}

		// Tracker offset / Playspace scale stay in advanced — these are rarely
		// touched by hand and live next to the per-axis math anyway.
		{
			ImVec2 panel_size_inner { panel_size.x - 11 * 2, 0};
			ImGui::BeginGroupPanel("Tracker offset", panel_size_inner);
			DrawVectorElement("cc_tracker_offset", "X", &CalCtx.continuousCalibrationOffset.x());
			DrawVectorElement("cc_tracker_offset", "Y", &CalCtx.continuousCalibrationOffset.y());
			DrawVectorElement("cc_tracker_offset", "Z", &CalCtx.continuousCalibrationOffset.z());
			ImGui::EndGroupPanel();
		}

		{
			ImVec2 panel_size_inner{ panel_size.x - 11 * 2, 0 };
			ImGui::BeginGroupPanel("Playspace scale", panel_size_inner);
			DrawVectorElement("cc_playspace_scale", "Playspace Scale", &CalCtx.calibratedScale, 1, " 1 ");
			ImGui::EndGroupPanel();
		}
	}

	ImGui::NewLine();
	ImGui::Indent();
	if (ImGui::Button("Reset settings")) {
		CalCtx.ResetConfig();
	}
	ImGui::SameLine();
	if (ImGui::Button("Run setup wizard")) {
		spacecal::wizard::Open();
	}
	if (ImGui::IsItemHovered()) {
		ImGui::SetTooltip(
			"Re-run the first-run setup wizard. Useful after changing your hardware\n"
			"(adding/removing a tracking system) or if you want to start fresh.");
	}
	ImGui::Unindent();
	ImGui::NewLine();

	// Section: Contributors credits
	{
		ImGui::BeginGroupPanel("Credits", panel_size);

		ImGui::TextDisabled("pushrax");
		ImGui::TextDisabled("hyblocker");
		ImGui::TextDisabled("tach");
		ImGui::TextDisabled("bd_");
		ImGui::TextDisabled("ArcticFox");
		ImGui::TextDisabled("hekky");
		ImGui::TextDisabled("pimaker");
		ImGui::TextDisabled("WhyKnot");

		ImGui::EndGroupPanel();
	}
}

void DrawVectorElement(const std::string id, const char* text, double* value, int defaultValue, const char* defaultValueStr) {
	constexpr float CONTINUOUS_CALIBRATION_TRACKER_OFFSET_DELTA = 0.01f;

	ImGui::Text(text);

	ImGui::SameLine();

	ImGui::PushID((id + text + "_btn_reset").c_str());
	if (ImGui::Button(defaultValueStr)) {
		*value *= defaultValue;
	}
	ImGui::PopID();
	ImGui::SameLine();
	if (ImGui::ArrowButton((id + text + "_decrease").c_str(), ImGuiDir_Down)) {
		*value -= CONTINUOUS_CALIBRATION_TRACKER_OFFSET_DELTA;
	}
	ImGui::SameLine();
	ImGui::PushItemWidth(100);
	ImGui::PushID((id + text + "_text_field").c_str());
	ImGui::InputDouble("##label", value, 0, 0, "%.2f");
	ImGui::PopID();
	ImGui::PopItemWidth();
	ImGui::SameLine();
	if (ImGui::ArrowButton((id + text + "_increase").c_str(), ImGuiDir_Up)) {
		*value += CONTINUOUS_CALIBRATION_TRACKER_OFFSET_DELTA;
	}
}

inline const char* GetPrettyTrackingSystemName(const std::string& value) {
	// To comply with SteamVR branding guidelines (page 29), we rename devices under lighthouse tracking to SteamVR Tracking.
	if (value == "lighthouse" || value == "aapvr") {
		return "SteamVR Tracking";
	}
	return value.c_str();
}

void CCal_DrawPredictionSuppression() {
	auto& ctx = CalCtx;

	// === Detection status box =============================================
	// Always-visible header so the user sees at a glance whether the program
	// has noticed an external smoothing tool. The previous design buried this
	// information; users couldn't tell if the program even *knew* OVR-SmoothTracking
	// was running.
	{
		ImVec4 bgColor;
		ImVec4 textColor = ImVec4(0.95f, 0.95f, 0.95f, 1.0f);
		const char* statusText = nullptr;
		const char* tool = ctx.externalSmoothingToolName.empty()
			? "an external smoothing tool"
			: ctx.externalSmoothingToolName.c_str();
		char statusBuf[256];

		if (ctx.externalSmoothingDetected) {
			snprintf(statusBuf, sizeof statusBuf,
				"DETECTED: %s is running.", tool);
			statusText = statusBuf;
			bgColor = ImVec4(0.55f, 0.40f, 0.10f, 1.0f); // amber
		} else {
			statusText = "No external smoothing tool detected.";
			bgColor = ImVec4(0.20f, 0.40f, 0.25f, 1.0f); // green
		}

		const ImVec2 textSize = ImGui::CalcTextSize(statusText);
		const ImVec2 padding(10.0f, 6.0f);
		ImVec2 cursor = ImGui::GetCursorScreenPos();
		ImVec2 rectMin = cursor;
		ImVec2 rectMax(cursor.x + ImGui::GetContentRegionAvail().x,
		               cursor.y + textSize.y + padding.y * 2.0f);
		ImDrawList* dl = ImGui::GetWindowDrawList();
		dl->AddRectFilled(rectMin, rectMax, ImGui::GetColorU32(bgColor), 6.0f);
		dl->AddText(ImVec2(rectMin.x + padding.x, rectMin.y + padding.y),
		            ImGui::GetColorU32(textColor), statusText);
		ImGui::Dummy(ImVec2(0, textSize.y + padding.y * 2.0f));
	}

	// External-tool warning. We don't try to interop -- when an external
	// smoothing tool is detected, our driver's velocity/acceleration scaling
	// fights it in unpredictable ways. Tell the user clearly.
	if (ctx.externalSmoothingDetected) {
		ImGui::Spacing();
		ImGui::PushStyleColor(ImGuiCol_ChildBg,  ImVec4(0.55f, 0.20f, 0.20f, 1.0f));
		ImGui::PushStyleColor(ImGuiCol_Border,   ImVec4(0.95f, 0.45f, 0.45f, 1.0f));
		ImGui::PushStyleVar(ImGuiStyleVar_ChildBorderSize, 1.0f);
		ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(10.0f, 8.0f));
		const float h = ImGui::GetFrameHeightWithSpacing() * 3.0f;
		if (ImGui::BeginChild("##ext_warn", ImVec2(ImGui::GetContentRegionAvail().x, h),
		                      ImGuiChildFlags_Border)) {
			const char* tool = ctx.externalSmoothingToolName.empty()
				? "An external smoothing tool"
				: ctx.externalSmoothingToolName.c_str();
			ImGui::TextColored(ImVec4(1.0f, 0.95f, 0.95f, 1.0f),
				"%s is running.  We don't support working alongside it -- our smoothing\n"
				"and its smoothing will fight, and the result is unpredictable.\n"
				"Please close it and use the per-tracker smoothness sliders below instead.",
				tool);
		}
		ImGui::EndChild();
		ImGui::PopStyleVar(2);
		ImGui::PopStyleColor(2);
	}

	ImGui::Spacing();
	ImGui::TextWrapped(
		"Prediction smoothness scales each tracker's reported velocity / acceleration "
		"down toward zero. 0 leaves the pose untouched (raw motion, sharp response). 100 "
		"fully zeros velocity, which defeats SteamVR's pose extrapolation entirely (smoothest "
		"motion at the cost of a tiny lag). Pick a value between to trade response for jitter.\n\n"
		"Three devices can never be smoothed: the HMD, the calibration reference tracker, "
		"and the calibration target tracker. Suppressing them would either cause judder in "
		"your view or corrupt the calibration math, so they're locked at 0 regardless of "
		"what you pick here.");
	ImGui::Spacing();
	ImGui::Separator();
	ImGui::TextDisabled("Per-tracker smoothness");
	ImGui::TextWrapped(
		"Settings stick to a tracker by serial number, so a device that disconnects and reconnects "
		"keeps its slider value.");
	ImGui::Spacing();

	auto vrSystem = vr::VRSystem();
	if (!vrSystem) {
		ImGui::TextDisabled("(VR system not available)");
		return;
	}

	// Resolve hard-block serials once. We compare by serial (stable across ID
	// reassignment) rather than by index. The calibration profile stores the
	// "calibrated" serials in referenceStandby.serial / targetStandby.serial;
	// match against those so the lock holds even when a device just reconnected
	// and its OpenVR ID is fresh.
	const std::string refSerial = ctx.referenceStandby.serial;
	const std::string tgtSerial = ctx.targetStandby.serial;

	bool anyShown = false;
	char buffer[vr::k_unMaxPropertyStringSize];
	for (uint32_t id = 0; id < vr::k_unMaxTrackedDeviceCount; ++id) {
		auto deviceClass = vrSystem->GetTrackedDeviceClass(id);
		if (deviceClass == vr::TrackedDeviceClass_Invalid) continue;

		vr::ETrackedPropertyError err = vr::TrackedProp_Success;
		vrSystem->GetStringTrackedDeviceProperty(id, vr::Prop_SerialNumber_String, buffer, sizeof buffer, &err);
		if (err != vr::TrackedProp_Success || buffer[0] == 0) continue;
		std::string serial = buffer;

		vrSystem->GetStringTrackedDeviceProperty(id, vr::Prop_RenderModelName_String, buffer, sizeof buffer, &err);
		std::string model = (err == vr::TrackedProp_Success) ? buffer : "";

		vrSystem->GetStringTrackedDeviceProperty(id, vr::Prop_TrackingSystemName_String, buffer, sizeof buffer, &err);
		std::string sys = (err == vr::TrackedProp_Success) ? GetPrettyTrackingSystemName(buffer) : "";

		const bool isHmd = (deviceClass == vr::TrackedDeviceClass_HMD);
		const bool isRef = !refSerial.empty() && refSerial == serial;
		const bool isTgt = !tgtSerial.empty() && tgtSerial == serial;
		const bool blocked = isHmd || isRef || isTgt;

		// Read current value (0 if absent from the map).
		int smoothness = 0;
		auto it = ctx.trackerSmoothness.find(serial);
		if (it != ctx.trackerSmoothness.end()) smoothness = it->second;
		// If blocked, force-display 0 even if a stale value is stored.
		if (blocked) smoothness = 0;

		const char* roleTag = isHmd ? "HMD"
			: isRef ? "calibration reference"
			: isTgt ? "calibration target"
			: nullptr;

		// Label row: device info text.
		ImGui::PushID(("trk_" + serial).c_str());
		ImGui::Text("%s  [%s]  %s",
			model.empty() ? "(unknown model)" : model.c_str(),
			sys.empty() ? "?" : sys.c_str(),
			serial.c_str());
		if (roleTag) {
			ImGui::SameLine();
			ImGui::TextColored(ImVec4(0.85f, 0.85f, 0.55f, 1.0f), "[%s, locked]", roleTag);
		}

		// Slider row.
		ImGui::BeginDisabled(blocked);
		if (ImGui::SliderInt("smoothness##slider", &smoothness, 0, 100, "%d%%")) {
			if (smoothness <= 0) ctx.trackerSmoothness.erase(serial);
			else ctx.trackerSmoothness[serial] = smoothness;
			SaveProfile(ctx);
		}
		ImGui::EndDisabled();
		if (ImGui::IsItemHovered()) {
			if (blocked) {
				ImGui::SetTooltip(
					"Locked to 0 because this device is the %s.\n"
					"Suppressing it would %s.",
					roleTag,
					isHmd
						? "cause judder in your view"
						: "corrupt the calibration math (it reads this device's velocity)");
			} else {
				ImGui::SetTooltip(
					"0 = raw motion (no suppression).\n"
					"100 = fully suppressed (matches the old binary 'freeze' behaviour).\n"
					"Try around 50-75 for IMU-based trackers that feel jittery.");
			}
		}
		ImGui::Spacing();
		ImGui::PopID();
		anyShown = true;
	}
	if (!anyShown) {
		ImGui::TextDisabled("(No tracked devices found.)");
	}
}

// === Logs panel ============================================================
// User-friendly view of the debug-log files written when "Enable debug logs"
// is on. The previous Recordings tab was dev-tooling -- it loaded a CSV and
// replayed it through the calibration math, which only made sense for the
// developer iterating on a fix. Most users just want a way to find and copy
// log files to attach to a bug report. So we surface the file list, a button
// to open the logs folder in Explorer, and a per-row copy-path action.
//
// The replay machinery (spacecal::replay::LoadRecording / RunReplay) is still
// in MotionRecording.cpp for the standalone replay CLI tool and the test
// suite; it just isn't surfaced in the overlay UI anymore.
namespace {

struct LogsPanelState {
	std::vector<spacecal::replay::LogFileEntry> files;
	bool listBuilt = false;
	int selectedIdx = -1;
	std::string copyHint;        // "Path copied" / errors -- transient feedback
	double copyHintExpireTime = 0.0;
};

LogsPanelState& LogsState() {
	static LogsPanelState s;
	return s;
}

void RebuildLogsList() {
	auto& s = LogsState();
	s.files = spacecal::replay::ListRecordings();
	s.listBuilt = true;
	if (s.selectedIdx >= (int)s.files.size()) s.selectedIdx = -1;
}

// Format file age relative to "now" using FILETIME math. We don't bother with
// localized strings — "5 min ago" / "2 hours ago" / "3 days ago" is plenty
// detail for picking the right log out of a list.
std::string FormatFileAge(uint64_t mtimeFt) {
	FILETIME nowFt{};
	GetSystemTimeAsFileTime(&nowFt);
	const uint64_t now = ((uint64_t)nowFt.dwHighDateTime << 32) | nowFt.dwLowDateTime;
	if (mtimeFt > now) return "in the future"; // clock skew sentinel
	const uint64_t deltaTicks = now - mtimeFt; // 100-ns ticks
	const uint64_t deltaSec = deltaTicks / 10'000'000ull;
	char buf[64];
	if (deltaSec < 60)             snprintf(buf, sizeof buf, "%llus ago", (unsigned long long)deltaSec);
	else if (deltaSec < 3600)      snprintf(buf, sizeof buf, "%llum ago", (unsigned long long)(deltaSec / 60));
	else if (deltaSec < 86400)     snprintf(buf, sizeof buf, "%lluh ago", (unsigned long long)(deltaSec / 3600));
	else                           snprintf(buf, sizeof buf, "%llud ago", (unsigned long long)(deltaSec / 86400));
	return buf;
}

std::string FormatBytesShort(uint64_t n) {
	char buf[64];
	if (n >= (1ull << 20)) snprintf(buf, sizeof buf, "%.1f MB", (double)n / (double)(1ull << 20));
	else if (n >= (1ull << 10)) snprintf(buf, sizeof buf, "%.0f KB", (double)n / (double)(1ull << 10));
	else snprintf(buf, sizeof buf, "%llu B", (unsigned long long)n);
	return buf;
}

// Resolve the logs directory. ListRecordings discovers it internally for
// scanning; we want the parent path for the Explorer button. If the list is
// empty, derive from the first entry's full path; if THAT's empty too, fall
// back to the standard %LOCALAPPDATA%\Low\SpaceCalibrator\Logs path.
std::wstring GetLogsDirectory() {
	auto& s = LogsState();
	if (!s.files.empty()) {
		const auto& full = s.files.front().fullPath;
		const size_t lastSlash = full.find_last_of(L"\\/");
		if (lastSlash != std::wstring::npos) {
			return full.substr(0, lastSlash);
		}
	}
	// Fallback: standard Windows AppDataLow path. This matches what
	// Metrics::enableLogs writes into. We don't take a hard dependency on
	// shlobj here -- the path is stable and well-documented for the lifetime
	// of this app.
	wchar_t* appDataLow = nullptr;
	if (SUCCEEDED(SHGetKnownFolderPath(FOLDERID_LocalAppDataLow, 0, nullptr, &appDataLow)) && appDataLow) {
		std::wstring path(appDataLow);
		CoTaskMemFree(appDataLow);
		path += L"\\SpaceCalibrator\\Logs";
		return path;
	}
	return L"";
}

// Copy a UTF-16 string to the Windows clipboard. Returns true on success.
bool CopyToClipboardW(const std::wstring& text) {
	if (!OpenClipboard(nullptr)) return false;
	EmptyClipboard();
	const size_t bytes = (text.size() + 1) * sizeof(wchar_t);
	HGLOBAL h = GlobalAlloc(GMEM_MOVEABLE, bytes);
	if (!h) { CloseClipboard(); return false; }
	if (auto* buf = (wchar_t*)GlobalLock(h)) {
		memcpy(buf, text.c_str(), bytes);
		GlobalUnlock(h);
		SetClipboardData(CF_UNICODETEXT, h);
	} else {
		GlobalFree(h);
		CloseClipboard();
		return false;
	}
	CloseClipboard();
	return true;
}

} // namespace

static void CCal_DrawLogsPanel() {
	auto& state = LogsState();

	ImGui::TextWrapped(
		"Debug logs are CSV files written one row per calibration tick. They're the "
		"first thing to attach to a bug report -- the team can replay the captured "
		"poses against the live math to reproduce what you saw.");
	ImGui::Spacing();

	// Recording status: reflects the live debug-log toggle. The log file IS
	// the recording -- there's no separate "start/stop recording" notion.
	if (Metrics::enableLogs) {
		ImGui::TextColored(ImVec4(0.45f, 0.85f, 0.45f, 1.0f),
			"● Logging: ON  (a fresh CSV is being written this session)");
	} else {
		ImGui::TextColored(ImVec4(0.85f, 0.55f, 0.45f, 1.0f),
			"○ Logging: OFF  (enable \"Enable debug logs\" in Settings to capture the next session)");
	}

	ImGui::Spacing();
	ImGui::Separator();

	ImGui::TextDisabled("Log files");
	ImGui::SameLine();
	if (ImGui::SmallButton("Refresh##logs")) {
		RebuildLogsList();
	}
	ImGui::SameLine();
	if (ImGui::SmallButton("Open folder##logs")) {
		const std::wstring dir = GetLogsDirectory();
		if (!dir.empty()) {
			ShellExecuteW(nullptr, L"open", dir.c_str(), nullptr, nullptr, SW_SHOWNORMAL);
		}
	}

	if (!state.listBuilt) RebuildLogsList();

	if (state.files.empty()) {
		ImGui::TextDisabled("(No log files found in the Logs directory.)");
	} else {
		// Selectable list of log files, newest first.
		const float listHeight = ImGui::GetTextLineHeightWithSpacing() * 8.0f;
		if (ImGui::BeginChild("##logs_list",
				ImVec2(0, listHeight), ImGuiChildFlags_Border)) {
			for (int i = 0; i < (int)state.files.size(); ++i) {
				const auto& f = state.files[i];
				char label[512];
				snprintf(label, sizeof label, "%s   (%s, %s)",
					f.name.c_str(),
					FormatBytesShort(f.sizeBytes).c_str(),
					FormatFileAge(f.mtimeFileTime).c_str());
				if (ImGui::Selectable(label, state.selectedIdx == i)) {
					state.selectedIdx = i;
				}
			}
		}
		ImGui::EndChild();
	}

	ImGui::Spacing();

	const bool haveSelection = state.selectedIdx >= 0
		&& state.selectedIdx < (int)state.files.size();
	ImGui::BeginDisabled(!haveSelection);
	if (ImGui::Button("Open selected##logs")) {
		ShellExecuteW(nullptr, L"open",
			state.files[state.selectedIdx].fullPath.c_str(),
			nullptr, nullptr, SW_SHOWNORMAL);
	}
	ImGui::SameLine();
	if (ImGui::Button("Copy path##logs")) {
		if (CopyToClipboardW(state.files[state.selectedIdx].fullPath)) {
			state.copyHint = "Path copied to clipboard";
		} else {
			state.copyHint = "Failed to copy path (clipboard busy?)";
		}
		state.copyHintExpireTime = ImGui::GetTime() + 2.5;
	}
	ImGui::EndDisabled();

	// Transient feedback row -- shown for ~2.5s after a clipboard action.
	if (!state.copyHint.empty() && ImGui::GetTime() < state.copyHintExpireTime) {
		ImGui::SameLine();
		ImGui::PushStyleColor(ImGuiCol_Text, ImGui::GetStyleColorVec4(ImGuiCol_TextDisabled));
		ImGui::TextUnformatted(state.copyHint.c_str());
		ImGui::PopStyleColor();
	} else if (!state.copyHint.empty()) {
		state.copyHint.clear();
	}
}

// Mirror of the "Reference HMD not detected" banner from BuildMenu, rendered
// inside the continuous-cal Status tab so the user sees it even when they jump
// straight into continuous mode. Returns true if the banner was drawn.
static bool DrawProfileMismatchBanner() {
	if (!CalCtx.validProfile || CalCtx.enabled) return false;
	const char* refSystem = GetPrettyTrackingSystemName(CalCtx.referenceTrackingSystem);
	// The "actual" tracking system is whatever the current HMD reports — fall
	// back to a plain "current HMD" wording when we can't read it cleanly.
	std::string actualSystem;
	if (auto vrSystem = vr::VRSystem()) {
		char buffer[vr::k_unMaxPropertyStringSize] = { 0 };
		vr::ETrackedPropertyError err = vr::TrackedProp_Success;
		vrSystem->GetStringTrackedDeviceProperty(
			vr::k_unTrackedDeviceIndex_Hmd,
			vr::Prop_TrackingSystemName_String,
			buffer, sizeof buffer, &err);
		if (err == vr::TrackedProp_Success && buffer[0] != 0) {
			actualSystem = GetPrettyTrackingSystemName(buffer);
		}
	}
	ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.95f, 0.55f, 0.45f, 1.0f));
	if (!actualSystem.empty()) {
		ImGui::TextWrapped(
			"Profile expects %s HMD but current HMD is on %s. Calibration not applied.",
			refSystem, actualSystem.c_str());
	} else {
		ImGui::TextWrapped(
			"Profile expects %s HMD but current HMD is unavailable or on a different tracking system. Calibration not applied.",
			refSystem);
	}
	ImGui::PopStyleColor();
	if (ImGui::Button("Clear profile")) {
		CalCtx.Clear();
		SaveProfile(CalCtx);
	}
	ImGui::SameLine();
	if (ImGui::Button("Recalibrate")) {
		// Same trigger BuildMenu uses for "Start Calibration" — kicks the state
		// machine into Begin via StartCalibration. The popup isn't relevant here
		// since the user is already inside the continuous-cal window.
		StartCalibration();
	}
	ImGui::Separator();
	return true;
}

// Render the watchdog / HMD-stall diagnostic counters wrapped in a group panel.
// Lives in Advanced (not Basic) since these are bug-report breadcrumbs, not
// something a casual user needs to see while running.
static void DrawDiagnosticsPanel(ImVec2 panelSize) {
	ImGui::BeginGroupPanel("Diagnostics", panelSize);

	// Watchdog reset tracking. We reflect whether the count has changed
	// recently (within ~15 s) by colouring the line amber, matching the
	// continuous-recalibration banner pattern.
	static int s_lastSeenWatchdog = -1;
	static double s_lastWatchdogResetTime = 0.0;
	static int s_lastSeenStallCount = 0;
	static int s_stallPurgeCount = 0;
	static double s_lastStallPurgeTime = 0.0;

	const int wdResets = GetWatchdogResetCount();
	const double now = ImGui::GetTime();
	if (s_lastSeenWatchdog < 0) {
		s_lastSeenWatchdog = wdResets;
	} else if (wdResets != s_lastSeenWatchdog) {
		s_lastSeenWatchdog = wdResets;
		s_lastWatchdogResetTime = now;
	}

	// HMD-stall purge detection: watch CalCtx.consecutiveHmdStalls cross the
	// threshold the calibration tick uses internally (~30 samples = ~1.5 s).
	const int kHmdStallPurgeThreshold = 30;
	const int curStalls = CalCtx.consecutiveHmdStalls;
	if (curStalls >= kHmdStallPurgeThreshold && s_lastSeenStallCount < kHmdStallPurgeThreshold) {
		s_stallPurgeCount++;
		s_lastStallPurgeTime = now;
	}
	s_lastSeenStallCount = curStalls;

	const bool wdRecent = wdResets > 0 && (now - s_lastWatchdogResetTime) < 15.0 && s_lastWatchdogResetTime > 0.0;
	if (wdRecent) {
		ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 0.78f, 0.30f, 1.0f));
		ImGui::Text("Watchdog reset %.0fs ago - recollecting samples (count: %d)",
			now - s_lastWatchdogResetTime, wdResets);
		ImGui::PopStyleColor();
	} else if (wdResets == 0) {
		ImGui::TextDisabled("Watchdog resets: 0 (last: never)");
	} else {
		ImGui::TextDisabled("Watchdog resets: %d (last: %.0fs ago)", wdResets,
			s_lastWatchdogResetTime > 0.0 ? (now - s_lastWatchdogResetTime) : 0.0);
	}
	if (ImGui::IsItemHovered()) {
		ImGui::SetTooltip("Stuck-loop watchdog: fires when continuous calibration has been rejecting every new\n"
		                  "sample for ~25 seconds. When it fires, the current estimate is discarded and\n"
		                  "we recollect from scratch. A high count here usually means motion conditioning is\n"
		                  "poor (move slower, rotate around more axes) or trackers are drifting against each\n"
		                  "other faster than the solver can keep up.");
	}

	const bool stallRecent = s_stallPurgeCount > 0 && (now - s_lastStallPurgeTime) < 15.0;
	if (stallRecent) {
		ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 0.78f, 0.30f, 1.0f));
		ImGui::Text("HMD-stall purge %.0fs ago - recollecting samples (count: %d)",
			now - s_lastStallPurgeTime, s_stallPurgeCount);
		ImGui::PopStyleColor();
	} else if (s_stallPurgeCount == 0) {
		ImGui::TextDisabled("HMD-stall purges: 0 (last: never)");
	} else {
		ImGui::TextDisabled("HMD-stall purges: %d (last: %.0fs ago)", s_stallPurgeCount,
			now - s_lastStallPurgeTime);
	}
	if (ImGui::IsItemHovered()) {
		ImGui::SetTooltip("HMD-stall purge: fires when the headset stops reporting fresh poses for ~1.5 seconds\n"
		                  "(SteamVR hiccup, tracking loss, sleep-wake). The sample buffer is purged because the\n"
		                  "stale samples around the stall are unreliable. Normal during a brief tracking glitch;\n"
		                  "frequent stalls suggest a tracking-environment problem (lighting, USB bandwidth, etc).");
	}

	ImGui::EndGroupPanel();
}

// Tip strip for both tabs. Reminds the user that hover = tooltip and
// right-click on sliders = reset to default. Light-weight, identical between
// Basic and Advanced so the hint is always reachable.
static void DrawTipPanel(ImVec2 panelSize) {
	ImGui::BeginGroupPanel("Tip", panelSize);
	ImGui::TextWrapped("Hover over any setting to learn more about it. Right-click any slider to reset it to its default value.");
	ImGui::EndGroupPanel();
}

// One-shot mode's Settings tab. Mirrors the Common Settings panel from
// CCal_BasicInfo (continuous mode) but trimmed to what a one-shot user
// actually touches: jitter / lock / recal-on-movement / static-recal /
// debug logs. Continuous-only knobs (recalibration threshold, alignment
// thresholds, latency tuning) live in the Advanced tab, which is shared
// verbatim with continuous mode.
//
// The reasoning for having both this AND CCal_BasicInfo's Common settings
// rather than one shared function: the surrounding contexts differ -- the
// continuous Basic tab has device-status table + Cancel/Restart/Pause action
// buttons above, this one is just the settings. Splitting keeps each call
// site readable; the per-row code is short enough that the duplication isn't
// worth a parameter-explosion in a shared helper.
static void OneShot_DrawSettings() {
	ImVec2 panelSize{ ImGui::GetWindowContentRegionMax().x - ImGui::GetWindowContentRegionMin().x, 0 };
	ImGui::BeginGroupPanel("Settings", panelSize);

	if (ImGui::BeginTable("##oneshot_settings_grid", 2,
			ImGuiTableFlags_SizingStretchProp | ImGuiTableFlags_NoBordersInBody)) {
		ImGui::TableSetupColumn("##label", ImGuiTableColumnFlags_WidthFixed, 230.0f);
		ImGui::TableSetupColumn("##control", ImGuiTableColumnFlags_WidthStretch);

		// --- Jitter threshold ---
		ImGui::TableNextRow();
		ImGui::TableSetColumnIndex(0);
		ImGui::AlignTextToFramePadding();
		ImGui::TextUnformatted("Jitter threshold");
		ImGui::TableSetColumnIndex(1);
		ImGui::PushID("oneshot_jitter_threshold");
		ImGui::SetNextItemWidth(-FLT_MIN);
		ImGui::SliderFloat("##oneshot_jitter_threshold_slider", &CalCtx.jitterThreshold, 0.1f, 10.0f, "%1.1f", 0);
		if (ImGui::IsItemHovered()) {
			ImGui::SetTooltip("Maximum sample-to-sample noise (mm) the math will tolerate before refusing\n"
				"to start a one-shot calibration. Higher values let noisier trackers calibrate\n"
				"at the cost of a less stable result.");
		}
		AddResetContextMenu("oneshot_jitter_threshold_ctx", [] { CalCtx.jitterThreshold = 3.0f; });
		ImGui::PopID();

		// --- Lock relative position (tristate) ---
		ImGui::TableNextRow();
		ImGui::TableSetColumnIndex(0);
		ImGui::AlignTextToFramePadding();
		ImGui::TextUnformatted("Lock relative position");
		ImGui::TableSetColumnIndex(1);
		ImGui::PushID("oneshot_lock_mode");
		const char* lockLabels[] = { "Off", "On", "Auto" };
		const CalibrationContext::LockMode lockModes[] = {
			CalibrationContext::LockMode::OFF,
			CalibrationContext::LockMode::ON,
			CalibrationContext::LockMode::AUTO
		};
		for (int i = 0; i < 3; ++i) {
			if (i > 0) ImGui::SameLine();
			if (ImGui::RadioButton(lockLabels[i], CalCtx.lockRelativePositionMode == lockModes[i])) {
				CalCtx.lockRelativePositionMode = lockModes[i];
				SaveProfile(CalCtx);
			}
		}
		if (ImGui::IsItemHovered()) {
			ImGui::SetTooltip(
				"Off:  the math is free to re-solve the relative pose every cycle. Right for\n"
				"      independent devices (HMD on head + body tracker on hip).\n"
				"On:   freeze the relative pose once calibrated. Right for rigid setups\n"
				"      (tracker glued to HMD, taped to a controller).\n"
				"Auto: detect rigid attachment from observed motion. Recommended.");
		}
		ImGui::PopID();

		// --- Recalibrate on movement ---
		ImGui::TableNextRow();
		ImGui::TableSetColumnIndex(0);
		ImGui::AlignTextToFramePadding();
		ImGui::TextUnformatted("Recalibrate on movement");
		ImGui::TableSetColumnIndex(1);
		if (ImGui::Checkbox("##oneshot_recal_on_move", &CalCtx.recalibrateOnMovement)) {
			SaveProfile(CalCtx);
		}
		if (ImGui::IsItemHovered()) {
			ImGui::SetTooltip("When the calibration math updates, only blend the new offset in while you're moving.\n"
				"Stationary users (lying down, sitting still) won't see phantom body shifts.\n"
				"Default ON.");
		}

		// --- Static recalibration ---
		ImGui::TableNextRow();
		ImGui::TableSetColumnIndex(0);
		ImGui::AlignTextToFramePadding();
		ImGui::TextUnformatted("Static recalibration");
		ImGui::TableSetColumnIndex(1);
		if (ImGui::Checkbox("##oneshot_static_recal", &CalCtx.enableStaticRecalibration)) {
			SaveProfile(CalCtx);
		}
		if (ImGui::IsItemHovered()) {
			ImGui::SetTooltip("Use the locked reference->target relative pose for fast snap-back when the\n"
				"live solver diverges. No-op for independent devices; accelerates rigid recovery.\n"
				"Default ON -- it's a no-op when there's nothing locked, so safe to leave on.");
		}

		// --- Hide tracker ---
		ImGui::TableNextRow();
		ImGui::TableSetColumnIndex(0);
		ImGui::AlignTextToFramePadding();
		ImGui::TextUnformatted("Hide tracker (during cal)");
		ImGui::TableSetColumnIndex(1);
		if (ImGui::Checkbox("##oneshot_hide_tracker", &CalCtx.quashTargetInContinuous)) {
			SaveProfile(CalCtx);
		}
		if (ImGui::IsItemHovered()) {
			ImGui::SetTooltip("Suppress the target tracker's pose in OpenVR while calibration runs.\n"
				"Use when the target would otherwise show as a duplicate of the reference\n"
				"(e.g. a Vive tracker taped to a Quest controller for one-shot calibration).");
		}

		// --- Ignore outliers ---
		ImGui::TableNextRow();
		ImGui::TableSetColumnIndex(0);
		ImGui::AlignTextToFramePadding();
		ImGui::TextUnformatted("Ignore outliers");
		ImGui::TableSetColumnIndex(1);
		if (ImGui::Checkbox("##oneshot_ignore_outliers", &CalCtx.ignoreOutliers)) {
			SaveProfile(CalCtx);
		}
		if (ImGui::IsItemHovered()) {
			ImGui::SetTooltip("Drop sample pairs whose rotation axis disagrees with the consensus before\n"
				"the LS solve. Helps with intermittent USB glitches or brief tracking loss.");
		}

		// --- Base station drift correction (AUTO/OFF) ---
		ImGui::TableNextRow();
		ImGui::TableSetColumnIndex(0);
		ImGui::AlignTextToFramePadding();
		ImGui::TextUnformatted("Auto-correct universe shifts");
		ImGui::TableSetColumnIndex(1);
		if (ImGui::Checkbox("##oneshot_base_station_drift", &CalCtx.baseStationDriftCorrectionEnabled)) {
			SaveProfile(CalCtx);
		}
		if (ImGui::IsItemHovered()) {
			ImGui::SetTooltip("AUTO (on): when Lighthouse base stations are detected, watch for\n"
			                  "uniform pose shifts across all of them between ticks -- a SteamVR\n"
			                  "universe re-origin (chaperone reset, seated zero pose reset, etc.) --\n"
			                  "and apply the inverse to the stored calibration so body trackers stay\n"
			                  "aligned with your physical position. No-op if no base stations are\n"
			                  "present (Quest-only setups, etc.). Math is honest: requires actual\n"
			                  "evidence of a universe shift, not a heuristic guess.\n\n"
			                  "OFF: never adjust the calibration based on base station poses.");
		}

		// --- Debug logs ---
		ImGui::TableNextRow();
		ImGui::TableSetColumnIndex(0);
		ImGui::AlignTextToFramePadding();
		ImGui::TextUnformatted("Enable debug logs");
		ImGui::TableSetColumnIndex(1);
		ImGui::Checkbox("##oneshot_debug_logs", &Metrics::enableLogs);
		if (ImGui::IsItemHovered()) {
			ImGui::SetTooltip("Write a per-tick CSV log of calibration state. Useful for bug reports.\n"
			                  "The Logs tab lists captured sessions and lets you copy paths.");
		}

		ImGui::EndTable();
	}

	ImGui::EndGroupPanel(); // Settings

	// Wizard / reset row at the bottom of the Settings panel.
	ImGui::Spacing();
	if (ImGui::Button("Run setup wizard")) {
		spacecal::wizard::Open();
	}
	if (ImGui::IsItemHovered()) {
		ImGui::SetTooltip("Re-run the first-run setup wizard. Useful after changing your hardware\n"
		                  "(adding/removing a tracking system) or if you want to start fresh.");
	}
	ImGui::SameLine();
	if (ImGui::Button("Reset settings")) {
		CalCtx.ResetConfig();
	}
	if (ImGui::IsItemHovered()) {
		ImGui::SetTooltip("Reset all settings (jitter / speed / lock / etc.) to defaults.\n"
		                  "Does NOT clear your calibrated profile -- only the tunables.");
	}
}

void CCal_BasicInfo() {
	// Mirror the profile-mismatch banner from BuildMenu so it's visible while
	// the user is in continuous mode.
	DrawProfileMismatchBanner();

	ImVec2 panelSize{ ImGui::GetWindowContentRegionMax().x - ImGui::GetWindowContentRegionMin().x, 0 };

	DrawTipPanel(panelSize);

	// --- Devices panel -----------------------------------------------------
	// The same reference + target table as before, but wrapped in a group
	// panel so it visually matches the panels in Advanced. The panel widget
	// owns its own width so we don't pass a non-zero panelSize here -- letting
	// the panel auto-fit its content prevents a stray right-edge gap when the
	// table is narrower than the window.
	ImGui::BeginGroupPanel("Devices", panelSize);
	if (ImGui::BeginTable("DeviceInfo", 2, 0)) {
		ImGui::TableSetupColumn("Reference device");
		ImGui::TableSetupColumn("Target device");
		ImGui::TableHeadersRow();

		const char* refTrackingSystem = GetPrettyTrackingSystemName(CalCtx.referenceStandby.trackingSystem);
		const char* targetTrackingSystem = GetPrettyTrackingSystemName(CalCtx.targetStandby.trackingSystem);

		ImGui::TableNextRow();
		ImGui::TableSetColumnIndex(0);
		ImGui::BeginGroup();
		ImGui::Text("%s / %s / %s",
			refTrackingSystem,
			CalCtx.referenceStandby.model.c_str(),
			CalCtx.referenceStandby.serial.c_str()
		);
		const char* status;
		if (CalCtx.referenceID < 0) {
			ImGui::TableSetBgColor(ImGuiTableBgTarget_CellBg, 0xFF000080);
			status = "NOT FOUND";
		} else if (!CalCtx.ReferencePoseIsValidSimple()) {
			ImGui::TableSetBgColor(ImGuiTableBgTarget_CellBg, 0xFFFF0080);
			status = "NOT TRACKING";
		} else {
			status = "OK";
		}
		ImGui::Text("Status: %s", status);
		ImGui::EndGroup();

		ImGui::TableSetColumnIndex(1);
		ImGui::BeginGroup();
		ImGui::Text("%s / %s / %s",
			targetTrackingSystem,
			CalCtx.targetStandby.model.c_str(),
			CalCtx.targetStandby.serial.c_str()
		);
		if (CalCtx.targetID < 0) {
			ImGui::TableSetBgColor(ImGuiTableBgTarget_CellBg, 0xFF000080);
			status = "NOT FOUND";
		}
		else if (!CalCtx.TargetPoseIsValidSimple()) {
			ImGui::TableSetBgColor(ImGuiTableBgTarget_CellBg, 0xFFFF0080);
			status = "NOT TRACKING";
		}
		else {
			status = "OK";
		}
		ImGui::Text("Status: %s", status);
		ImGui::EndGroup();

		ImGui::EndTable();
	}
	ImGui::EndGroupPanel(); // Devices

	// --- Actions panel -----------------------------------------------------
	// Three-way grid (Cancel | Restart sampling | Pause) inside a group panel
	// so it visually matches the rest of Basic.
	ImGui::BeginGroupPanel("Actions", panelSize);
	float width = ImGui::GetWindowContentRegionWidth(), scale = 1.0f;
	if (ImGui::BeginTable("##CCal_Cancel", 3, 0, ImVec2(width * scale, ImGui::GetTextLineHeight() * 2))) {
		ImGui::TableNextRow();
		ImGui::TableSetColumnIndex(0);
		if (ImGui::Button("Cancel Continuous Calibration", ImVec2(-FLT_MIN, 0.0f))) {
			EndContinuousCalibration();
		}
		if (ImGui::IsItemHovered()) {
			ImGui::SetTooltip("Stop continuous calibration. The last applied offset stays in place\n"
			                  "as a fixed offset until you start calibration again.");
		}

		ImGui::TableSetColumnIndex(1);
		// User-facing rename of the old "Debug: Force break calibration"
		// button. Same underlying call — pushing a random offset forces the
		// solver to re-search from samples instead of trusting the current
		// estimate, which is what "restart sampling" means in practice.
		if (ImGui::Button("Restart sampling", ImVec2(-FLT_MIN, 0.0f))) {
			DebugApplyRandomOffset();
		}
		if (ImGui::IsItemHovered()) {
			ImGui::SetTooltip("Discards the current incremental estimate and forces continuous calibration to recollect samples from scratch.\n"
			                  "Use this if the calibration looks off and you want a fresh search instead of nudging from the current estimate.");
		}

		ImGui::TableSetColumnIndex(2);
		// Toggle button: while paused, the calibration tick is expected to
		// skip ComputeIncremental so the live offset stays put. The flag
		// itself lives on CalibrationContext (see Calibration.h); the gate
		// is on the math/tick side.
		const bool paused = CalCtx.calibrationPaused;
		if (paused) {
			ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.65f, 0.45f, 0.15f, 1.0f));
			ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.75f, 0.55f, 0.20f, 1.0f));
			ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImVec4(0.55f, 0.40f, 0.10f, 1.0f));
		}
		if (ImGui::Button(paused ? "Resume updates" : "Pause updates", ImVec2(-FLT_MIN, 0.0f))) {
			CalCtx.calibrationPaused = !CalCtx.calibrationPaused;
		}
		if (paused) {
			ImGui::PopStyleColor(3);
		}
		if (ImGui::IsItemHovered()) {
			ImGui::SetTooltip("Freeze the live calibration offset without ending continuous mode.\n"
			                  "Useful when something looks momentarily wrong and you want to investigate before the solver self-corrects.");
		}

		ImGui::EndTable();
	}
	ImGui::EndGroupPanel(); // Actions

	// === Common settings ===================================================
	// The handful of settings most users actually touch.  Two-column table
	// inside the panel so labels and sliders/checkboxes line up cleanly --
	// the previous Text + SameLine + Slider layout was readable but the
	// columns wandered with label width.
	ImGui::BeginGroupPanel("Common settings", panelSize);

	// Two-column grid: label on the left, control on the right. Lets each row
	// have a consistent baseline regardless of label length, instead of the
	// previous Text + SameLine + Slider layout where columns wandered.
	if (ImGui::BeginTable("##common_settings_grid", 2,
			ImGuiTableFlags_SizingStretchProp | ImGuiTableFlags_NoBordersInBody)) {
		ImGui::TableSetupColumn("##label", ImGuiTableColumnFlags_WidthFixed, 230.0f);
		ImGui::TableSetupColumn("##control", ImGuiTableColumnFlags_WidthStretch);

		// --- Jitter threshold ---
		ImGui::TableNextRow();
		ImGui::TableSetColumnIndex(0);
		ImGui::AlignTextToFramePadding();
		ImGui::TextUnformatted("Jitter threshold");
		ImGui::TableSetColumnIndex(1);
		ImGui::PushID("basic_jitter_threshold");
		ImGui::SetNextItemWidth(-FLT_MIN);
		ImGui::SliderFloat("##basic_jitter_threshold_slider", &CalCtx.jitterThreshold, 0.1f, 10.0f, "%1.1f", 0);
		if (ImGui::IsItemHovered()) {
			ImGui::SetTooltip("Controls how much jitter will be allowed for calibration.\n"
				"Higher values allow worse tracking to calibrate, but may result in poorer tracking.");
		}
		AddResetContextMenu("basic_jitter_threshold_ctx", [] { CalCtx.jitterThreshold = 3.0f; });
		ImGui::PopID();

		// --- Recalibration threshold ---
		ImGui::TableNextRow();
		ImGui::TableSetColumnIndex(0);
		ImGui::AlignTextToFramePadding();
		ImGui::TextUnformatted("Recalibration threshold");
		ImGui::TableSetColumnIndex(1);
		ImGui::PushID("basic_recalibration_threshold");
		ImGui::SetNextItemWidth(-FLT_MIN);
		ImGui::SliderFloat("##basic_recalibration_threshold_slider", &CalCtx.continuousCalibrationThreshold, 1.01f, 10.0f, "%1.1f", 0);
		if (ImGui::IsItemHovered()) {
			ImGui::SetTooltip("Controls how good the calibration must be before realigning the trackers.\n"
				"Higher values cause calibration to happen less often, and may be useful for systems with lots of tracking drift.");
		}
		AddResetContextMenu("basic_recalibration_threshold_ctx", [] { CalCtx.continuousCalibrationThreshold = 1.5f; });
		ImGui::PopID();

		// --- Lock relative position (tristate) ---
		ImGui::TableNextRow();
		ImGui::TableSetColumnIndex(0);
		ImGui::AlignTextToFramePadding();
		ImGui::TextUnformatted("Lock relative position");
		ImGui::TableSetColumnIndex(1);
		ImGui::PushID("basic_lock_mode");
		const char* lockLabels[] = { "Off", "On", "Auto" };
		const CalibrationContext::LockMode lockModes[] = {
			CalibrationContext::LockMode::OFF,
			CalibrationContext::LockMode::ON,
			CalibrationContext::LockMode::AUTO
		};
		for (int i = 0; i < 3; ++i) {
			if (i > 0) ImGui::SameLine();
			if (ImGui::RadioButton(lockLabels[i], CalCtx.lockRelativePositionMode == lockModes[i])) {
				CalCtx.lockRelativePositionMode = lockModes[i];
				SaveProfile(CalCtx);
			}
		}
		if (ImGui::IsItemHovered()) {
			ImGui::SetTooltip(
				"Off:  the math is free to re-solve the relative pose every cycle. Right for\n"
				"      independent devices (HMD on head + body tracker on hip).\n"
				"On:   freeze the relative pose once calibrated. Right for rigid setups\n"
				"      (tracker glued to HMD, taped to a controller).\n"
				"Auto: detect rigid attachment from observed motion. Recommended -- starts\n"
				"      unlocked, then locks once the relative pose has been stable for ~15s.");
		}
		// Resolved-state caption directly below the radios so the user sees
		// what AUTO decided. Disabled-text colour keeps it visually subordinate.
		if (CalCtx.lockRelativePositionMode == CalibrationContext::LockMode::AUTO) {
			ImGui::PushStyleColor(ImGuiCol_Text, ImGui::GetStyleColorVec4(ImGuiCol_TextDisabled));
			if (CalCtx.autoLockEffectivelyLocked) {
				ImGui::TextWrapped("Auto: locked (detected as rigidly attached, %d samples)",
					(int)CalCtx.autoLockHistory.size());
			} else if (CalCtx.autoLockHistory.size() < 30) {
				ImGui::TextWrapped("Auto: collecting motion data (%d/30 samples)",
					(int)CalCtx.autoLockHistory.size());
			} else {
				ImGui::TextWrapped("Auto: unlocked (devices move independently)");
			}
			ImGui::PopStyleColor();
		}
		ImGui::PopID();

		// --- Require trigger press ---
		ImGui::TableNextRow();
		ImGui::TableSetColumnIndex(0);
		ImGui::AlignTextToFramePadding();
		ImGui::TextUnformatted("Require trigger press");
		ImGui::TableSetColumnIndex(1);
		if (ImGui::Checkbox("##basic_require_trigger", &CalCtx.requireTriggerPressToApply)) {
			SaveProfile(CalCtx);
		}
		if (ImGui::IsItemHovered()) {
			ImGui::SetTooltip("If on, only apply the calibrated offset while a controller trigger is held.\n"
				"Useful for verifying the result before committing.");
		}

		// --- Recalibrate on movement ---
		ImGui::TableNextRow();
		ImGui::TableSetColumnIndex(0);
		ImGui::AlignTextToFramePadding();
		ImGui::TextUnformatted("Recalibrate on movement");
		ImGui::TableSetColumnIndex(1);
		if (ImGui::Checkbox("##basic_recal_on_move", &CalCtx.recalibrateOnMovement)) {
			SaveProfile(CalCtx);
		}
		if (ImGui::IsItemHovered()) {
			ImGui::SetTooltip("When the calibration math updates, only blend the new offset in while you're actually moving.\n"
				"Stationary users (e.g. lying down) won't see phantom body shifts; the catch-up happens during natural motion.\n"
				"Default ON. Turn off to get instantaneous time-based blending regardless of motion state.");
		}

		// --- Debug logs ---
		// Lives inside the same panel so the user doesn't have a checkbox
		// floating in white space below the panel like before. Functionally
		// the same; visually consistent with everything else in Basic.
		ImGui::TableNextRow();
		ImGui::TableSetColumnIndex(0);
		ImGui::AlignTextToFramePadding();
		ImGui::TextUnformatted("Enable debug logs");
		ImGui::TableSetColumnIndex(1);
		ImGui::Checkbox("##basic_debug_logs", &Metrics::enableLogs);
		if (ImGui::IsItemHovered()) {
			ImGui::SetTooltip("Write a per-tick CSV log of calibration state to\n"
			                  "%%LocalAppDataLow%%\\SpaceCalibrator\\Logs\\spacecal_log.<date>.txt\n"
			                  "Useful for bug reports. The Logs tab lists all captured sessions\n"
			                  "and lets you copy paths or open the folder for attaching files.");
		}

		ImGui::EndTable();
	}

	ImGui::EndGroupPanel(); // Common settings

	// === Status messages ===================================================
	// Whatever the calibration state machine has logged this session, plus
	// a tiny scrollback so a user can see the most recent "Applying updated
	// transformation..." line without enabling the debug log.
	ImGui::Spacing();
	ImGui::PushStyleColor(ImGuiCol_FrameBg, ImVec4(0, 0, 0, 1));
	for (const auto& msg : CalCtx.messages) {
		if (msg.type == CalibrationContext::Message::String) {
			ImGui::TextWrapped("> %s", msg.str.c_str());
		}
	}
	ImGui::PopStyleColor();
}

void BuildMenu(bool runningInOverlay)
{
	auto &io = ImGui::GetIO();
	ImGuiStyle &style = ImGui::GetStyle();
	ImGui::Text("");

	if (CalCtx.state == CalibrationState::None)
	{
		if (CalCtx.validProfile && !CalCtx.enabled)
		{
			// New wording: tell the user which system the profile expects vs
			// which one their current HMD is on. Pull the actual tracking
			// system name from the current HMD when we can; fall back to a
			// generic phrasing otherwise so the line still makes sense if the
			// HMD isn't reachable.
			const char* refSystem = GetPrettyTrackingSystemName(CalCtx.referenceTrackingSystem);
			std::string actualSystem;
			if (auto vrSystem = vr::VRSystem()) {
				char buffer[vr::k_unMaxPropertyStringSize] = { 0 };
				vr::ETrackedPropertyError err = vr::TrackedProp_Success;
				vrSystem->GetStringTrackedDeviceProperty(
					vr::k_unTrackedDeviceIndex_Hmd,
					vr::Prop_TrackingSystemName_String,
					buffer, sizeof buffer, &err);
				if (err == vr::TrackedProp_Success && buffer[0] != 0) {
					actualSystem = GetPrettyTrackingSystemName(buffer);
				}
			}
			ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.95f, 0.55f, 0.45f, 1.0f));
			if (!actualSystem.empty()) {
				ImGui::TextWrapped(
					"Profile expects %s HMD but current HMD is on %s. Calibration not applied.",
					refSystem, actualSystem.c_str());
			} else {
				ImGui::TextWrapped(
					"Profile expects %s HMD but current HMD is unavailable or on a different tracking system. Calibration not applied.",
					refSystem);
			}
			ImGui::PopStyleColor();
			if (ImGui::Button("Clear profile")) {
				CalCtx.Clear();
				SaveProfile(CalCtx);
			}
			ImGui::SameLine();
			ImGui::BeginDisabled(!IsVRReady());
			if (ImGui::Button("Recalibrate")) {
				ImGui::OpenPopup("Calibration Progress");
				StartCalibration();
			}
			ImGui::EndDisabled();
			if (!IsVRReady() && ImGui::IsItemHovered()) {
				ImGui::SetTooltip("Waiting for SteamVR.");
			}
			ImGui::Text("");
		}

		float width = ImGui::GetWindowContentRegionWidth(), scale = 1.0f;
		if (CalCtx.validProfile)
		{
			width -= style.FramePadding.x * 4.0f;
			scale = 1.0f / 4.0f;
		}

		// Start / Continuous Calibration both need a live VR stack to enumerate
		// devices and collect samples. Edit / Clear are pure-memory operations
		// on the saved profile and stay enabled even without SteamVR running.
		ImGui::BeginDisabled(!IsVRReady());
		if (ImGui::Button("Start Calibration", ImVec2(width * scale, ImGui::GetTextLineHeight() * 2)))
		{
			ImGui::OpenPopup("Calibration Progress");
			StartCalibration();
		}
		ImGui::EndDisabled();
		if (!IsVRReady() && ImGui::IsItemHovered()) {
			ImGui::SetTooltip("Waiting for SteamVR. Start it and the button will enable automatically.");
		}

		ImGui::SameLine();
		ImGui::BeginDisabled(!IsVRReady());
		if (ImGui::Button("Continuous Calibration", ImVec2(width * scale, ImGui::GetTextLineHeight() * 2))) {
			StartContinuousCalibration();
		}
		ImGui::EndDisabled();
		if (!IsVRReady() && ImGui::IsItemHovered()) {
			ImGui::SetTooltip("Waiting for SteamVR. Start it and the button will enable automatically.");
		}

		if (CalCtx.validProfile)
		{
			ImGui::SameLine();
			if (ImGui::Button("Edit Calibration", ImVec2(width * scale, ImGui::GetTextLineHeight() * 2)))
			{
				CalCtx.state = CalibrationState::Editing;
			}

			ImGui::SameLine();
			if (ImGui::Button("Clear Calibration", ImVec2(width * scale, ImGui::GetTextLineHeight() * 2)))
			{
				CalCtx.Clear();
				SaveProfile(CalCtx);
			}
		}

		width = ImGui::GetWindowContentRegionWidth();
		scale = 1.0f;
		if (CalCtx.chaperone.valid)
		{
			width -= style.FramePadding.x * 2.0f;
			scale = 0.5;
		}

		ImGui::Text("");
		ImGui::BeginDisabled(!IsVRReady());
		if (ImGui::Button("Copy Chaperone Bounds to profile", ImVec2(width * scale, ImGui::GetTextLineHeight() * 2)))
		{
			LoadChaperoneBounds();
			SaveProfile(CalCtx);
		}
		ImGui::EndDisabled();
		if (!IsVRReady() && ImGui::IsItemHovered()) {
			ImGui::SetTooltip("Waiting for SteamVR.");
		}

		if (CalCtx.chaperone.valid)
		{
			ImGui::SameLine();
			ImGui::BeginDisabled(!IsVRReady());
			if (ImGui::Button("Paste Chaperone Bounds", ImVec2(width * scale, ImGui::GetTextLineHeight() * 2)))
			{
				ApplyChaperoneBounds();
			}
			ImGui::EndDisabled();
			if (!IsVRReady() && ImGui::IsItemHovered()) {
				ImGui::SetTooltip("Waiting for SteamVR.");
			}

			if (ImGui::Checkbox(" Paste Chaperone Bounds automatically when geometry resets", &CalCtx.chaperone.autoApply))
			{
				SaveProfile(CalCtx);
			}
		}

		ImGui::Text("");
		auto speed = CalCtx.calibrationSpeed;

		ImGui::Columns(5, nullptr, false);
		ImGui::Text("Calibration Speed");

		ImGui::NextColumn();
		if (ImGui::RadioButton(" Auto          ", speed == CalibrationContext::AUTO)) {
			CalCtx.calibrationSpeed = CalibrationContext::AUTO;
			SaveProfile(CalCtx);
		}
		if (ImGui::IsItemHovered()) {
			ImGui::SetTooltip("Pick automatically from observed jitter. Recommended.");
		}

		ImGui::NextColumn();
		if (ImGui::RadioButton(" Fast          ", speed == CalibrationContext::FAST)) {
			CalCtx.calibrationSpeed = CalibrationContext::FAST;
			SaveProfile(CalCtx);
		}

		ImGui::NextColumn();
		if (ImGui::RadioButton(" Slow          ", speed == CalibrationContext::SLOW)) {
			CalCtx.calibrationSpeed = CalibrationContext::SLOW;
			SaveProfile(CalCtx);
		}

		ImGui::NextColumn();
		if (ImGui::RadioButton(" Very Slow     ", speed == CalibrationContext::VERY_SLOW)) {
			CalCtx.calibrationSpeed = CalibrationContext::VERY_SLOW;
			SaveProfile(CalCtx);
		}

		ImGui::Columns(1);
	}
	else if (CalCtx.state == CalibrationState::Editing)
	{
		BuildProfileEditor();

		if (ImGui::Button("Save Profile", ImVec2(ImGui::GetWindowContentRegionWidth(), ImGui::GetTextLineHeight() * 2)))
		{
			SaveProfile(CalCtx);
			CalCtx.state = CalibrationState::None;
		}
	}
	else
	{
		ImGui::Button("Calibration in progress...", ImVec2(ImGui::GetWindowContentRegionWidth(), ImGui::GetTextLineHeight() * 2));
	}

	ImGui::SetNextWindowPos(ImVec2(20.0f, 20.0f), ImGuiCond_Always);
	ImGui::SetNextWindowSize(ImVec2(io.DisplaySize.x - 40.0f, io.DisplaySize.y - 40.0f), ImGuiCond_Always);
	if (ImGui::BeginPopupModal("Calibration Progress", nullptr, bareWindowFlags))
	{
		ImGui::PushStyleColor(ImGuiCol_FrameBg, (ImVec4)ImVec4(0, 0, 0, 1));
		for (auto &message : CalCtx.messages)
		{
			switch (message.type)
			{
			case CalibrationContext::Message::String:
				ImGui::TextWrapped(message.str.c_str());
				break;
			case CalibrationContext::Message::Progress:
				float fraction = (float)message.progress / (float)message.target;
				ImGui::Text("");
				ImGui::ProgressBar(fraction, ImVec2(-1.0f, 0.0f), "");
				ImGui::SetCursorPosY(ImGui::GetCursorPosY() - ImGui::GetFontSize() - style.FramePadding.y * 2);
				ImGui::Text(" %d%%", (int)(fraction * 100));
				break;
			}
		}
		ImGui::PopStyleColor();

		// Live motion-coverage feedback while the user is actively running a
		// one-shot calibration (Begin / Rotation / Translation). The math runs
		// at sample-buffer fill regardless; these bars just tell the user
		// "you've moved enough on every axis" so they don't stop early or
		// keep waving long after they're done.
		const bool isCollecting =
			CalCtx.state == CalibrationState::Begin ||
			CalCtx.state == CalibrationState::Rotation ||
			CalCtx.state == CalibrationState::Translation;
		if (isCollecting) {
			ImGui::Spacing();
			ImGui::Separator();
			ImGui::TextDisabled("Motion coverage");
			ImGui::Spacing();

			const float trDiv = (float)Metrics::translationDiversity.last();
			const float rotDiv = (float)Metrics::rotationDiversity.last();

			char trLabel[64], rotLabel[64];
			snprintf(trLabel, sizeof trLabel, "Translation %d%%", (int)(trDiv * 100.0f));
			snprintf(rotLabel, sizeof rotLabel, "Rotation %d%%", (int)(rotDiv * 100.0f));

			constexpr float kGoodThreshold = 0.70f;
			const ImVec4 trColor = trDiv >= kGoodThreshold
				? ImVec4(0.40f, 0.85f, 0.40f, 1.0f)
				: ImVec4(0.95f, 0.70f, 0.20f, 1.0f);
			const ImVec4 rotColor = rotDiv >= kGoodThreshold
				? ImVec4(0.40f, 0.85f, 0.40f, 1.0f)
				: ImVec4(0.95f, 0.70f, 0.20f, 1.0f);

			ImGui::PushStyleColor(ImGuiCol_PlotHistogram, trColor);
			ImGui::ProgressBar(trDiv, ImVec2(-1.0f, 0.0f), trLabel);
			ImGui::PopStyleColor();
			if (ImGui::IsItemHovered()) {
				ImGui::SetTooltip("Translation coverage: how much you've moved the tracker along all three axes.\n"
				                  "Wave it ~30 cm in each of left/right, up/down, and forward/back to fill this bar.\n"
				                  "Green = enough variety for a clean calibration.");
			}
			ImGui::PushStyleColor(ImGuiCol_PlotHistogram, rotColor);
			ImGui::ProgressBar(rotDiv, ImVec2(-1.0f, 0.0f), rotLabel);
			ImGui::PopStyleColor();
			if (ImGui::IsItemHovered()) {
				ImGui::SetTooltip("Rotation coverage: the widest angle between any two sampled tracker rotations.\n"
				                  "Twist the tracker through ~90 degrees at some point to fill this bar.\n"
				                  "Green = enough variety for a clean calibration.");
			}

			if (trDiv < kGoodThreshold || rotDiv < kGoodThreshold) {
				ImGui::PushStyleColor(ImGuiCol_Text, ImGui::GetStyleColorVec4(ImGuiCol_TextDisabled));
				if (trDiv < rotDiv) {
					ImGui::TextWrapped("Tip: try moving the tracker through wider distances on every axis.");
				} else {
					ImGui::TextWrapped("Tip: try rotating the tracker more (point it in different directions).");
				}
				ImGui::PopStyleColor();
			}
		}

		if (CalCtx.state == CalibrationState::None)
		{
			ImGui::Text("");
			if (ImGui::Button("Close", ImVec2(ImGui::GetWindowContentRegionWidth(), ImGui::GetTextLineHeight() * 2)))
				ImGui::CloseCurrentPopup();
		}

		ImGui::EndPopup();
	}
}

void BuildSystemSelection(const VRState &state)
{
	if (state.trackingSystems.empty())
	{
		ImGui::Text("No tracked devices are present");
		return;
	}

	ImGuiStyle &style = ImGui::GetStyle();
	float paneWidth = ImGui::GetWindowContentRegionWidth() / 2 - style.FramePadding.x;

	TextWithWidth("ReferenceSystemLabel", "Reference Space", paneWidth);
	ImGui::SameLine();
	TextWithWidth("TargetSystemLabel", "Target Space", paneWidth);

	int currentReferenceSystem = -1;
	int currentTargetSystem = -1;
	int firstReferenceSystemNotTargetSystem = -1;

	std::vector<const char *> referenceSystems;
	std::vector<const char *> referenceSystemsUi;
	for (const std::string& str : state.trackingSystems)
	{
		if (str == CalCtx.referenceTrackingSystem)
		{
			currentReferenceSystem = (int) referenceSystems.size();
		}
		else if (firstReferenceSystemNotTargetSystem == -1 && str != CalCtx.targetTrackingSystem)
		{
			firstReferenceSystemNotTargetSystem = (int) referenceSystems.size();
		}
		referenceSystems.push_back(str.c_str());
		referenceSystemsUi.push_back(GetPrettyTrackingSystemName(str));
	}

	if (currentReferenceSystem == -1 && CalCtx.referenceTrackingSystem == "")
	{
		if (CalCtx.state == CalibrationState::ContinuousStandby) {
			auto iter = std::find(state.trackingSystems.begin(), state.trackingSystems.end(), CalCtx.referenceStandby.trackingSystem);
			if (iter != state.trackingSystems.end()) {
				currentReferenceSystem = (int) (iter - state.trackingSystems.begin());
			}
		}
		else {
			currentReferenceSystem = firstReferenceSystemNotTargetSystem;
		}
	}

	ImGui::PushItemWidth(paneWidth);
	ImGui::Combo("##ReferenceTrackingSystem", &currentReferenceSystem, &referenceSystemsUi[0], (int)referenceSystemsUi.size());

	if (currentReferenceSystem != -1 && currentReferenceSystem < (int) referenceSystems.size())
	{
		CalCtx.referenceTrackingSystem = std::string(referenceSystems[currentReferenceSystem]);
		if (CalCtx.referenceTrackingSystem == CalCtx.targetTrackingSystem)
			CalCtx.targetTrackingSystem = "";
	}

	if (CalCtx.targetTrackingSystem == "") {
		if (CalCtx.state == CalibrationState::ContinuousStandby) {
			auto iter = std::find(state.trackingSystems.begin(), state.trackingSystems.end(), CalCtx.targetStandby.trackingSystem);
			if (iter != state.trackingSystems.end()) {
				currentTargetSystem = (int) (iter - state.trackingSystems.begin());
			}
		}
		else {
			currentTargetSystem = 0;
		}
	}

	std::vector<const char *> targetSystems;
	std::vector<const char *> targetSystemsUi;
	for (const std::string& str : state.trackingSystems)
	{
		if (str != CalCtx.referenceTrackingSystem)
		{
			if (str != "" && str == CalCtx.targetTrackingSystem)
				currentTargetSystem = (int) targetSystems.size();
			targetSystems.push_back(str.c_str());
			targetSystemsUi.push_back(GetPrettyTrackingSystemName(str));
		}
	}

	ImGui::SameLine();
	ImGui::Combo("##TargetTrackingSystem", &currentTargetSystem, &targetSystemsUi[0], (int)targetSystemsUi.size());

	if (currentTargetSystem != -1 && currentTargetSystem < targetSystems.size())
	{
		CalCtx.targetTrackingSystem = std::string(targetSystems[currentTargetSystem]);
	}

	ImGui::PopItemWidth();
}

void AppendSeparated(std::string &buffer, const std::string &suffix)
{
	if (!buffer.empty())
		buffer += " | ";
	buffer += suffix;
}

std::string LabelString(const VRDevice &device)
{
	std::string label;

	/*if (device.controllerRole == vr::TrackedControllerRole_LeftHand)
		label = "Left Controller";
	else if (device.controllerRole == vr::TrackedControllerRole_RightHand)
		label = "Right Controller";
	else if (device.deviceClass == vr::TrackedDeviceClass_Controller)
		label = "Controller";
	else if (device.deviceClass == vr::TrackedDeviceClass_HMD)
		label = "HMD";
	else if (device.deviceClass == vr::TrackedDeviceClass_GenericTracker)
		label = "Tracker";*/

	AppendSeparated(label, device.model);
	AppendSeparated(label, device.serial);
	return label;
}

std::string LabelString(const StandbyDevice& device) {
	std::string label("< ");

	label += device.model;
	AppendSeparated(label, device.serial);

	label += " >";
	return label;
}

void BuildDeviceSelection(const VRState &state, int &initialSelected, const std::string &system, StandbyDevice &standbyDevice)
{
	int selected = initialSelected;
	ImGui::TextColored(ImVec4(0.5f, 0.5f, 0.5f, 1), "Devices from: %s", GetPrettyTrackingSystemName(system));

	if (selected != -1)
	{
		bool matched = false;
		for (auto &device : state.devices)
		{
			if (device.trackingSystem != system)
				continue;

			if (selected == device.id)
			{
				matched = true;
				break;
			}
		}

		if (!matched)
		{
			// Device is no longer present.
			selected = -1;
		}
	}

	bool standby = CalCtx.state == CalibrationState::ContinuousStandby;

	if (selected == -1 && !standby)
	{
		for (auto &device : state.devices)
		{
			if (device.trackingSystem != system)
				continue;

			if (device.controllerRole == vr::TrackedControllerRole_LeftHand)
			{
				selected = device.id;
				break;
			}
		}

		if (selected == -1) {
			for (auto& device : state.devices)
			{
				if (device.trackingSystem != system)
					continue;
				
				selected = device.id;
				break;
			}
		}
	}

	uint64_t iterator = 0;
	if (selected == -1 && standby) {
		bool present = false;
		for (auto& device : state.devices)
		{
			if (device.trackingSystem != system)
				continue;

			if (standbyDevice.model != device.model) continue;
			if (standbyDevice.serial != device.serial) continue;

			present = true;
			break;
		}

		if (!present) {
			auto label = LabelString(standbyDevice);
			std::string uniqueId = label + "_pass0_" + std::to_string(iterator);
			iterator++;
			ImGui::PushID(uniqueId.c_str());
			ImGui::Selectable(label.c_str(), true);
			ImGui::PopID();
		}
	}

	iterator = 0;

	for (auto &device : state.devices)
	{
		if (device.trackingSystem != system)
			continue;

		auto label = LabelString(device);
		std::string uniqueId = label + "_pass1_" + std::to_string(iterator);
		iterator++;
		ImGui::PushID(uniqueId.c_str());
		if (ImGui::Selectable(label.c_str(), selected == device.id)) {
			selected = device.id;
		}
		ImGui::PopID();
	}
	if (selected != initialSelected) {
		const auto& device = std::find_if(state.devices.begin(), state.devices.end(), [&](const auto& d) { return d.id == selected; });
		if (device == state.devices.end()) return;

		initialSelected = selected;
		standbyDevice.trackingSystem = system;
		standbyDevice.model = device->model;
		standbyDevice.serial = device->serial;
	}
}

void BuildDeviceSelections(const VRState &state)
{
	ImGuiStyle &style = ImGui::GetStyle();
	ImVec2 paneSize(ImGui::GetWindowContentRegionWidth() / 2 - style.FramePadding.x, ImGui::GetTextLineHeightWithSpacing() * 5 + style.ItemSpacing.y * 4);

	ImGui::BeginChild("left device pane", paneSize, ImGuiChildFlags_Borders);
	BuildDeviceSelection(state, CalCtx.referenceID, CalCtx.referenceTrackingSystem, CalCtx.referenceStandby);
	ImGui::EndChild();

	ImGui::SameLine();

	ImGui::BeginChild("right device pane", paneSize, ImGuiChildFlags_Borders);
	BuildDeviceSelection(state, CalCtx.targetID, CalCtx.targetTrackingSystem, CalCtx.targetStandby);
	ImGui::EndChild();

	if (ImGui::Button("Identify selected devices (blinks LED or vibrates)", ImVec2(ImGui::GetWindowContentRegionWidth(), ImGui::GetTextLineHeightWithSpacing() + 4.0f)))
	{
		for (unsigned i = 0; i < 100; ++i)
		{
			vr::VRSystem()->TriggerHapticPulse(CalCtx.targetID, 0, 2000);
			vr::VRSystem()->TriggerHapticPulse(CalCtx.referenceID, 0, 2000);
			std::this_thread::sleep_for(std::chrono::milliseconds(5));
		}
	}
}

VRState LoadVRState() {
	VRState state = VRState::Load();
	auto& trackingSystems = state.trackingSystems;

	// Inject entries for continuous calibration targets which have yet to load

	if (CalCtx.state == CalibrationState::ContinuousStandby) {
		auto existing = std::find(trackingSystems.begin(), trackingSystems.end(), CalCtx.referenceTrackingSystem);
		if (existing == trackingSystems.end()) {
			trackingSystems.push_back(CalCtx.referenceTrackingSystem);
		}

		existing = std::find(trackingSystems.begin(), trackingSystems.end(), CalCtx.targetTrackingSystem);
		if (existing == trackingSystems.end()) {
			trackingSystems.push_back(CalCtx.targetTrackingSystem);
		}
	}

	return state;
}

void BuildProfileEditor()
{
	ImGuiStyle &style = ImGui::GetStyle();
	float width = ImGui::GetWindowContentRegionWidth() / 3.0f - style.FramePadding.x;
	float widthF = width - style.FramePadding.x;

	TextWithWidth("YawLabel", "Yaw", width);
	ImGui::SameLine();
	TextWithWidth("PitchLabel", "Pitch", width);
	ImGui::SameLine();
	TextWithWidth("RollLabel", "Roll", width);

	ImGui::PushItemWidth(widthF);
	ImGui::InputDouble("##Yaw", &CalCtx.calibratedRotation(1), 0.1, 1.0, "%.8f");
	ImGui::SameLine();
	ImGui::InputDouble("##Pitch", &CalCtx.calibratedRotation(2), 0.1, 1.0, "%.8f");
	ImGui::SameLine();
	ImGui::InputDouble("##Roll", &CalCtx.calibratedRotation(0), 0.1, 1.0, "%.8f");

	TextWithWidth("XLabel", "X", width);
	ImGui::SameLine();
	TextWithWidth("YLabel", "Y", width);
	ImGui::SameLine();
	TextWithWidth("ZLabel", "Z", width);

	ImGui::InputDouble("##X", &CalCtx.calibratedTranslation(0), 1.0, 10.0, "%.8f");
	ImGui::SameLine();
	ImGui::InputDouble("##Y", &CalCtx.calibratedTranslation(1), 1.0, 10.0, "%.8f");
	ImGui::SameLine();
	ImGui::InputDouble("##Z", &CalCtx.calibratedTranslation(2), 1.0, 10.0, "%.8f");

	TextWithWidth("ScaleLabel", "Scale", width);

	ImGui::InputDouble("##Scale", &CalCtx.calibratedScale, 0.0001, 0.01, "%.8f");
	ImGui::PopItemWidth();
}

void TextWithWidth(const char *label, const char *text, float width)
{
	ImGui::BeginChild(label, ImVec2(width, ImGui::GetTextLineHeightWithSpacing()));
	ImGui::Text(text);
	ImGui::EndChild();
}

