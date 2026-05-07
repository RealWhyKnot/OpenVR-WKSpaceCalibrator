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
static void GetModeStatus(const char*& label, const char*& tooltip, ImVec4& accent);
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
void CCal_DrawFingerSmoothing();
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

	// Scroll allowed on the main window: bareWindowFlags disables it (right
	// for the calibration-progress popup and the continuous-mode root,
	// where content fits by construction), but the main page mixes
	// device dropdowns + action buttons + a tab bar and can overflow on
	// short windows. Without this, content past the bottom edge gets
	// silently clipped (no scrollbar, no scroll-with-mouse), which is
	// what produced the "I can't scroll down" report.
	const ImGuiWindowFlags mainFlags =
		bareWindowFlags & ~ImGuiWindowFlags_NoScrollbar
		                & ~ImGuiWindowFlags_NoScrollWithMouse;
	if (!ImGui::Begin("SpaceCalibrator", nullptr, mainFlags))
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

	// Auto-recovery banner (audit UX #3). Sticky for 60 s after the auto-
	// recover fires so the user actually notices that their calibration was
	// just clobbered, with Undo + Dismiss buttons. Without this the only
	// signal was a single line in CalCtx.messages, swept on the next
	// messages.clear(), invisible on tabs other than Basic. The 2026-05-02
	// false-positive recoveries that destroyed working cals would have been
	// caught here -- the user could have hit Undo within seconds.
	{
		double recoveryAge = 0.0, recoveryDelta = 0.0;
		if (LastAutoRecoveryActive(recoveryAge, recoveryDelta)) {
			ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 0.85f, 0.4f, 1.0f));
			ImGui::TextWrapped(
				"Auto-recovery cleared calibration %.0fs ago (~%.0f cm HMD jump). Recalibrating from scratch.",
				recoveryAge, recoveryDelta * 100.0);
			ImGui::PopStyleColor();
			if (ImGui::SmallButton("Undo (restore prior calibration)")) {
				UndoLastAutoRecovery();
			}
			ImGui::SameLine();
			if (ImGui::SmallButton("Dismiss")) {
				DismissAutoRecoveryBanner();
			}
			ImGui::Separator();
		}
	}

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
		// (Mode pill moved to the global footer; no longer takes a row at
		// the top of the main page.)

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
				if (ImGui::BeginTabItem("Fingers")) {
					CCal_DrawFingerSmoothing();
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
	// Footer is two short rows:
	//   row 1: Hover for tooltips. Right-click sliders to reset.
	//   row 2: ● Driver: ...   |   Space Calibrator <build>   |   mode
	// The hover tip sits above so the more-prominent status row is
	// visually anchored to the bottom edge of the window.
	const float lineH  = ImGui::GetTextLineHeight();
	const float footerH = lineH * 2.0f + 12.0f;
	ImGui::SetNextWindowPos(ImVec2(10.0f, ImGui::GetWindowHeight() - footerH));
	if (!ImGui::BeginChild("bottom line", ImVec2(ImGui::GetWindowWidth() - 20.0f, footerH), ImGuiChildFlags_None)) {
		ImGui::EndChild();
		return;
	}

	// --- Row 1: hover-for-tooltip hint ---
	ImGui::TextDisabled("Hover any setting for help. Right-click a slider to reset it.");

	// --- Row 2: driver status + version + mode ---
	// Driver-connection dot. Three states:
	//   green  - connected, everything's working.
	//   amber  - VR stack hasn't connected yet (SteamVR not running, or the
	//            program was launched standalone). Not an error; the main
	//            loop is retrying and will flip to green automatically.
	//   red    - established connection has dropped (IPC pipe broke after
	//            initial handshake). This is the case where reinstalling the
	//            driver is genuinely the right advice.
	const bool driverConnected = Driver.IsConnected();
	if (driverConnected) {
		DrawStatusDot(IM_COL32(80, 200, 120, 255));
		ImGui::TextColored(ImVec4(0.5f, 0.85f, 0.55f, 1.0f),
			"Driver: connected (v%u)", (unsigned)protocol::Version);
	} else if (!IsVRReady()) {
		DrawStatusDot(IM_COL32(220, 170, 60, 255));
		ImGui::TextColored(ImVec4(0.95f, 0.80f, 0.40f, 1.0f),
			"Driver: waiting for SteamVR");
	} else {
		DrawStatusDot(IM_COL32(220, 80, 80, 255));
		ImGui::TextColored(ImVec4(0.95f, 0.45f, 0.45f, 1.0f),
			"Driver: disconnected — reinstall the SteamVR driver");
	}

	ImGui::SameLine();
	ImGui::Text("  |  Space Calibrator " SPACECAL_BUILD_STAMP);
	if (runningInOverlay)
	{
		ImGui::SameLine();
		ImGui::Text("- close VR overlay to use mouse");
	}

	// Mode label after the version. Plain text styled the same way as the
	// driver-status text -- we apply the per-mode accent colour but no
	// rounded-pill background. Tooltip on hover for the longer
	// explanation.
	{
		const char* modeLabel = nullptr;
		const char* modeTooltip = nullptr;
		ImVec4 modeAccent;
		GetModeStatus(modeLabel, modeTooltip, modeAccent);
		ImGui::SameLine();
		ImGui::TextColored(modeAccent, "  |  %s", modeLabel);
		if (modeTooltip && ImGui::IsItemHovered()) {
			ImGui::SetTooltip("%s", modeTooltip);
		}
	}

	ImGui::EndChild();
}

// Compute the current calibration-state label + tooltip + accent colour.
// Used by the footer where it renders as plain text styled like the
// surrounding driver-status / version row, rather than a coloured pill.
static void GetModeStatus(const char*& label, const char*& tooltip, ImVec4& accent) {
	const auto state = CalCtx.state;
	const bool validProfile = CalCtx.validProfile;
	const bool enabled = CalCtx.enabled;

	if (!validProfile) {
		label = "no profile";
		tooltip = "No saved calibration profile is loaded.\n"
		          "Hit \"Start Calibration\" or \"Continuous Calibration\" below to create one.";
		accent = ImVec4(0.70f, 0.70f, 0.70f, 1.0f);
	} else if (state == CalibrationState::ContinuousStandby) {
		label = "standby — waiting for tracking";
		tooltip = "Continuous calibration is on, but the reference or target tracker isn't currently\n"
		          "reporting valid poses. Calibration resumes automatically when both come back online.";
		accent = ImVec4(0.85f, 0.85f, 0.55f, 1.0f);
	} else if (state == CalibrationState::Continuous) {
		const double now = ImGui::GetTime();
		const double sinceAccept = now - Metrics::error_currentCal.lastTs();
		const bool searching = Metrics::consecutiveRejections.last() > 10.0;
		const bool recentlyUpdated = sinceAccept >= 0.0 && sinceAccept < 5.0;
		if (searching) {
			label = "live — searching";
			tooltip = "Continuous calibration is running but hasn't accepted a new estimate in a while.\n"
			          "Usually means the user isn't moving enough to give the solver useful samples.\n"
			          "Try slowly rotating + translating the target tracker through varied directions.";
			accent = ImVec4(0.95f, 0.70f, 0.20f, 1.0f);
		} else if (recentlyUpdated) {
			label = "live — updating";
			tooltip = "Continuous calibration is running and just accepted a fresh estimate.\n"
			          "The driver is blending toward the new offset; if Recalibrate-on-movement is on,\n"
			          "the blend only progresses while the device is actively moving.";
			accent = ImVec4(0.55f, 0.75f, 0.95f, 1.0f);
		} else {
			label = "live";
			tooltip = "Continuous calibration is running and the current estimate is being applied.\n"
			          "No recent updates needed — the calibration is stable.";
			accent = ImVec4(0.55f, 0.75f, 0.95f, 1.0f);
		}
	} else if (enabled && state == CalibrationState::None) {
		label = "fixed offset active";
		tooltip = "A one-shot calibration is applied as a fixed offset. The driver applies the\n"
		          "stored transform; no continuous re-solving. Switch to Continuous mode if the\n"
		          "offset drifts over time.";
		accent = ImVec4(0.55f, 0.85f, 0.55f, 1.0f);
	} else {
		label = "idle";
		tooltip = "A profile is loaded but no calibration is being applied. This usually means the\n"
		          "current HMD tracking system doesn't match the profile's reference system.";
		accent = ImVec4(0.70f, 0.70f, 0.70f, 1.0f);
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

	// (Mode pill moved to the global footer alongside the driver-status dot;
	// no longer takes a row above the tab bar.)

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

		if (ImGui::BeginTabItem("Fingers")) {
			CCal_DrawFingerSmoothing();
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

	// (The "hover for tooltips" hint moved to the global footer so it
	// doesn't take a row at the top of every tab.)

	// Diagnostics panel: stuck-loop watchdog + HMD-stall purge counters.
	// Lives in Advanced because it's bug-report breadcrumbs, not something
	// a casual user is going to action on. Keeping it visible (rather than
	// behind another collapsing header) so a user with a problem can copy
	// the numbers into a bug report without spelunking.
	DrawDiagnosticsPanel(panel_size);

	// === Toggles panel ====================================================
	// Power-user checkboxes that aren't worth Basic real estate. Static
	// recalibration removed -- it's now always-on, gated implicitly by
	// whether Lock relative position has identified a rigid attachment, so
	// the separate toggle was redundant.
	ImGui::BeginGroupPanel("Toggles", panel_size);
	if (ImGui::Checkbox("Hide tracker", &CalCtx.quashTargetInContinuous)) {
		SaveProfile(CalCtx);
	}
	if (ImGui::IsItemHovered()) {
		ImGui::SetTooltip("Suppress the target tracker's pose in OpenVR while continuous calibration runs.\n"
		                  "Use when the target tracker would otherwise show up as a duplicate of the reference\n"
		                  "(e.g. taping a Vive tracker to a Quest controller for calibration).");
	}
	ImGui::SameLine();
	ImGui::Checkbox("Ignore outliers", &CalCtx.ignoreOutliers);
	if (ImGui::IsItemHovered()) {
		ImGui::SetTooltip("Drop sample pairs whose rotation axis disagrees with the consensus before the LS solve.\n"
		                  "Default on.  Turn off only if you suspect the outlier rejector is throwing out good samples\n"
		                  "(e.g. genuinely jittery motion the cosine-similarity test mistakes for outliers).");
	}
	ImGui::EndGroupPanel();
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
			ImGui::BeginGroupPanel("Thresholds", panel_size);

			// Jitter threshold (moved from Basic / one-shot Settings -- these
			// are rarely touched and were padding the Basic surfaces). Sized
			// label/control via a 2-col table so they line up cleanly with
			// the other thresholds.
			if (ImGui::BeginTable("##advanced_thresholds_grid", 2,
					ImGuiTableFlags_SizingStretchProp | ImGuiTableFlags_NoBordersInBody)) {
				ImGui::TableSetupColumn("##label", ImGuiTableColumnFlags_WidthFixed, 230.0f);
				ImGui::TableSetupColumn("##control", ImGuiTableColumnFlags_WidthStretch);

				ImGui::TableNextRow();
				ImGui::TableSetColumnIndex(0);
				ImGui::AlignTextToFramePadding();
				ImGui::TextUnformatted("Jitter threshold");
				ImGui::TableSetColumnIndex(1);
				ImGui::PushID("adv_jitter_threshold");
				ImGui::SetNextItemWidth(-FLT_MIN);
				ImGui::SliderFloat("##adv_jitter_threshold_slider", &CalCtx.jitterThreshold, 0.1f, 10.0f, "%1.1f", 0);
				if (ImGui::IsItemHovered()) {
					ImGui::SetTooltip("Controls how much jitter will be allowed for calibration.\n"
						"Higher values allow worse tracking to calibrate, but may result in poorer tracking.");
				}
				AddResetContextMenu("adv_jitter_threshold_ctx", [] { CalCtx.jitterThreshold = 3.0f; });
				ImGui::PopID();

				ImGui::TableNextRow();
				ImGui::TableSetColumnIndex(0);
				ImGui::AlignTextToFramePadding();
				ImGui::TextUnformatted("Recalibration threshold");
				ImGui::TableSetColumnIndex(1);
				ImGui::PushID("adv_recalibration_threshold");
				ImGui::SetNextItemWidth(-FLT_MIN);
				ImGui::SliderFloat("##adv_recalibration_threshold_slider", &CalCtx.continuousCalibrationThreshold, 1.01f, 10.0f, "%1.1f", 0);
				if (ImGui::IsItemHovered()) {
					ImGui::SetTooltip("Controls how good the calibration must be before realigning the trackers.\n"
						"Higher values cause calibration to happen less often, and may be useful for systems with lots of tracking drift.");
				}
				AddResetContextMenu("adv_recalibration_threshold_ctx", [] { CalCtx.continuousCalibrationThreshold = 1.5f; });
				ImGui::PopID();

				ImGui::TableNextRow();
				ImGui::TableSetColumnIndex(0);
				ImGui::AlignTextToFramePadding();
				ImGui::TextUnformatted("Max relative error threshold");
				ImGui::TableSetColumnIndex(1);
				ImGui::PushID("adv_max_relative_error_threshold");
				ImGui::SetNextItemWidth(-FLT_MIN);
				ImGui::SliderFloat("##adv_max_relative_error_threshold_slider", &CalCtx.maxRelativeErrorThreshold, 0.01f, 1.0f, "%1.1f", 0);
				if (ImGui::IsItemHovered()) {
					ImGui::SetTooltip("Controls the maximum acceptable relative error. If the error from the relative calibration is too poor, the calibration will be discarded.");
				}
				AddResetContextMenu("adv_max_rel_err_ctx", [] { CalCtx.maxRelativeErrorThreshold = 0.005f; });
				ImGui::PopID();

				ImGui::EndTable();
			}
			ImGui::EndGroupPanel();
		}

		// Latency tuning (one knob, less of a "Thresholds" grouping)
		{
			ImGui::BeginGroupPanel("Continuous calibration (advanced)", panel_size);

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
					"Default 0 disables the feature. Auto-detect below overrides this when on.");
			}
			AddResetContextMenu("target_latency_ctx", [] { CalCtx.targetLatencyOffsetMs = 0.0; });
			ImGui::PopID();

			// Auto-detect runs cross-correlation across the rolling sample
			// buffer that only fills during continuous calibration. In one-shot
			// mode the calibration is frozen and the estimator never runs, so
			// grey the checkbox out to make the constraint explicit.
			const bool kLatencyAutoDetectActive = (CalCtx.state == CalibrationState::Continuous);
			ImGui::BeginDisabled(!kLatencyAutoDetectActive);
			ImGui::Checkbox("Auto-detect target latency", &CalCtx.latencyAutoDetect);
			ImGui::EndDisabled();
			if (ImGui::IsItemHovered(0)) {
				if (kLatencyAutoDetectActive) {
					ImGui::SetTooltip("Uses cross-correlation of tracker velocities to estimate the inter-system\n"
						"latency once per second when both devices are moving. Overrides the manual\n"
						"offset above when on.");
				} else {
					ImGui::SetTooltip("Active only during continuous calibration. The cross-correlation\n"
						"estimator runs against the rolling sample buffer continuous mode populates;\n"
						"in one-shot mode the calibration is frozen and the estimator never fires.\n"
						"The manual offset slider above still applies in either mode.");
				}
			}

			ImGui::EndGroupPanel();
		}

		// Experimental opt-in toggles. New flags ship here default-off until real-
		// world session evidence shows they are an improvement (or a regression to
		// roll back). Header is collapsed by default so the panel doesn't grow
		// noisy as more flags accumulate. Plain English on labels; engineering
		// terms (algorithm names, paper refs) live in the tooltip if anywhere.
		//
		// Each toggle is wrapped in a previous-value compare so that a user flip
		// emits a one-shot log annotation. This gives anyone reading the
		// session log direct evidence of "behavior changed at time T because the
		// user enabled X". The previous-value statics are function-local so they
		// don't leak; first-frame initialization captures whatever the loaded
		// profile says, and subsequent flips fire the annotation.
		auto logToggleFlip = [](const char* key, bool& prev, bool current) {
			if (prev != current) {
				char buf[128];
				snprintf(buf, sizeof buf, "experimental_toggle_flip: key=%s value=%d", key, (int)current);
				Metrics::WriteLogAnnotation(buf);
				prev = current;
			}
		};
		{
			ImGui::BeginGroupPanel("Experimental (opt-in, may break)", panel_size);
			ImGui::TextWrapped("Off by default. Enable only if you want to help validate a new path. "
				"Each toggle changes one specific code path; if tracking regresses after you enable "
				"one, turn it back off and the old behavior returns.");
			ImGui::Spacing();

			// Mode-availability flags. Several toggles only affect runtime
			// behavior in specific calibration states; greying them out in the
			// modes where they would be silent no-ops makes the constraint
			// visible instead of leaving the user to wonder why a flip
			// produced no observable change.
			const bool continuousActive = (CalCtx.state == CalibrationState::Continuous);
			const bool oneShotInProgress = (CalCtx.state == CalibrationState::Begin
				|| CalCtx.state == CalibrationState::Rotation
				|| CalCtx.state == CalibrationState::Translation);
			const bool restLockedActive = !continuousActive && !oneShotInProgress;

			// Status banner: one line stating which subset of toggles will
			// actually act on the running calibration right now. Avoids the
			// guessing game of "I flipped it but tracking did not change."
			ImGui::PushStyleColor(ImGuiCol_Text, ImGui::GetStyleColorVec4(ImGuiCol_TextDisabled));
			if (continuousActive) {
				ImGui::TextWrapped("Continuous calibration is active. Continuous-only toggles below are interactive; "
					"continuous-OFF-only toggles (rest-locked yaw) are inactive while continuous runs.");
			} else if (oneShotInProgress) {
				ImGui::TextWrapped("One-shot calibration is in progress. Most toggles are inactive until the "
					"one-shot finishes or you start continuous calibration.");
			} else {
				ImGui::TextWrapped("Continuous calibration is OFF. Continuous-only toggles below are inactive; "
					"continuous-OFF-only toggles (rest-locked yaw, predictive recovery) are interactive.");
			}
			ImGui::PopStyleColor();
			ImGui::Spacing();

			// Whitened-spectrum latency estimator (GCC-PHAT, Knapp-Carter 1976).
			// Doubly gated: requires Auto-detect target latency above (which itself
			// requires continuous mode). Whichever gate is closed greys this one too.
			const bool gccPhatActive = continuousActive && CalCtx.latencyAutoDetect;
			ImGui::BeginDisabled(!gccPhatActive);
			ImGui::Checkbox("Whitened-spectrum latency estimator", &CalCtx.useGccPhatLatency);
			ImGui::EndDisabled();
			if (ImGui::IsItemHovered(0)) {
				if (gccPhatActive) {
					ImGui::SetTooltip("Replaces the default time-domain cross-correlator with a GCC-PHAT\n"
						"variant (Knapp-Carter 1976). May be sharper when the two tracking systems'\n"
						"velocity signals have very different frequency content (e.g. heavy IMU\n"
						"low-pass on one side). The default time-domain estimator is well-validated;\n"
						"turn this on only if your auto-detected offset reads as jumpy.\n\n"
						"Active in: continuous calibration with auto-detect target latency on.");
				} else if (continuousActive) {
					ImGui::SetTooltip("Requires Auto-detect target latency above. Off by default.\n\n"
						"Active in: continuous calibration with auto-detect target latency on.");
				} else {
					ImGui::SetTooltip("Active in: continuous calibration with auto-detect target latency on.\n"
						"Currently inactive because continuous calibration is off.");
				}
			}

			// CUSUM geometry-shift detector (Page 1954) replaces the 5x-rolling-
			// median rule with a cumulative-sum statistical test. Same recovery
			// action; tunable false-alarm rate via standard ARL tables.
			// Continuous-only: the geometry-shift watchdog runs inside the
			// continuous-cal tick path.
			ImGui::BeginDisabled(!continuousActive);
			ImGui::Checkbox("CUSUM geometry-shift detector", &CalCtx.useCusumGeometryShift);
			ImGui::EndDisabled();
			if (ImGui::IsItemHovered(0)) {
				if (continuousActive) {
					ImGui::SetTooltip("Statistical change-point test (CUSUM, Page 1954) instead of the fixed\n"
						"5x-rolling-median rule. Uses standard Average-Run-Length tables for a\n"
						"tunable false-alarm rate. Same recovery action when fired (clear cal,\n"
						"demote to standby). Off by default; the existing rolling-median rule\n"
						"has not misfired in observed sessions.\n\n"
						"Active in: continuous calibration only.");
				} else {
					ImGui::SetTooltip("Active in: continuous calibration only.\n"
						"The geometry-shift watchdog runs inside the continuous-cal tick path; in\n"
						"one-shot mode the calibration is frozen and the watchdog never fires.");
				}
			}

			// Velocity-aware outlier weighting in the IRLS translation solve.
			// Down-weights residuals taken during fast motion as likely
			// glitches; preserves residuals taken at rest as legitimate
			// "cal is wrong here" signal. IRLS runs in both ComputeOneshot
			// and ComputeIncremental, so this toggle applies in both modes.
			ImGui::Checkbox("Velocity-aware outlier weighting", &CalCtx.useVelocityAwareWeighting);
			if (ImGui::IsItemHovered(0)) {
				ImGui::SetTooltip("Scales the per-pair IRLS Cauchy threshold inversely with motion magnitude\n"
					"so high-residual rows from fast-moving frames are suppressed as likely glitches,\n"
					"while high-residual rows from stationary frames remain informative (the cal\n"
					"genuinely needs an update there). Off by default; the dominant rejection in\n"
					"observed logs is axis_variance_low which this does not address.\n\n"
					"Active in: both one-shot and continuous calibration.");
			}

			// Tukey biweight + Qn-scale alternative robust kernel. Same
			// IRLS path as velocity-aware weighting: applies in both modes.
			ImGui::Checkbox("Tukey biweight robust kernel", &CalCtx.useTukeyBiweight);
			if (ImGui::IsItemHovered(0)) {
				ImGui::SetTooltip("Replaces the default IRLS Cauchy + MAD with Tukey biweight + Qn-scale\n"
					"(Rousseeuw-Croux 1993). Tukey is redescending: residuals beyond the threshold\n"
					"get exactly zero weight, so a single bad-frame outlier cannot drag the fit.\n"
					"Qn does not saturate at the MAD floor and does not assume residual symmetry.\n"
					"Off by default; the standard Cauchy + MAD has been adequate on observed data.\n\n"
					"Active in: both one-shot and continuous calibration.");
			}

			// Kalman-filter blend at publish. Replaces the EMA in
			// ComputeIncremental; ComputeOneshot does not use the EMA path,
			// so this toggle is a no-op in one-shot mode.
			ImGui::BeginDisabled(!continuousActive);
			ImGui::Checkbox("Kalman-filter blend at publish", &CalCtx.useBlendFilter);
			ImGui::EndDisabled();
			if (ImGui::IsItemHovered(0)) {
				if (continuousActive) {
					ImGui::SetTooltip("Replaces the single-step EMA (alpha=0.3) at the publish point with a\n"
						"4-state Kalman filter on (yaw, tx, ty, tz). Process noise is tuned to typical\n"
						"long-term Quest-SLAM drift; measurement noise to validation-gate-pass quality.\n"
						"On a candidate that diverges from the filter prediction (post-relocalize snap,\n"
						"geometry-shift recovery), the filter resets and falls back to the EMA path for\n"
						"that tick. Off by default; the EMA is the validated default.\n\n"
						"Active in: continuous calibration only.");
				} else {
					ImGui::SetTooltip("Active in: continuous calibration only.\n"
						"The blend filter sits in the ComputeIncremental publish path; one-shot mode\n"
						"does not run incremental publishes, so the filter never sees a candidate.");
				}
			}

			// Rest-locked yaw drift correction. Inverse gating: only fires
			// when continuous-cal is OFF and one-shot is not in progress.
			ImGui::BeginDisabled(!restLockedActive);
			ImGui::Checkbox("Rest-locked yaw drift correction", &CalCtx.restLockedYawEnabled);
			ImGui::EndDisabled();
			if (ImGui::IsItemHovered(0)) {
				if (restLockedActive) {
					ImGui::SetTooltip("When a tracker stays still for 1 s, lock its orientation as an absolute\n"
						"reference. On every subsequent at-rest tick, compare predicted vs locked yaw\n"
						"and apply a bounded-rate correction to the active calibration (per-class cap,\n"
						"global ceiling 0.5 deg/s). Off by default; flips on after a real session test\n"
						"passes the four-tier success criterion (p90 error_currentCal, recovery-fire\n"
						"frequency, time-to-first-failure, subjective Likert).\n\n"
						"Active in: continuous calibration OFF (one-shot, idle, editing, standby).");
				} else if (continuousActive) {
					ImGui::SetTooltip("Active in: continuous calibration OFF only.\n"
						"Continuous-cal already corrects drift in its own loop; running both would\n"
						"produce two integrators acting on the same error and risk oscillation.");
				} else {
					ImGui::SetTooltip("Active in: continuous calibration OFF only.\n"
						"Currently inactive because a one-shot calibration is in progress.");
				}
			}

			// Predictive recovery pre-correction. Buffer fills on the 30 cm
			// relocalization fire (continuous mode); per-tick apply runs in
			// any state. Mostly continuous-relevant.
			ImGui::Checkbox("Predictive recovery pre-correction", &CalCtx.predictiveRecoveryEnabled);
			if (ImGui::IsItemHovered(0)) {
				ImGui::SetTooltip("Each Quest re-anchor event (the 30 cm HMD-jump trigger) pushes its direction\n"
					"and magnitude into a 6-deep rolling buffer. Once 3+ events accumulate with a consistent\n"
					"direction, apply 10 percent of the predicted next-jump per tick as a bounded-rate\n"
					"translation nudge to the active calibration. Bounded twice (10 percent fraction +\n"
					"per-tick rate cap) so a misfire cannot reproduce the deleted Phase 1+2 silent-recal\n"
					"failure mode. Off by default.\n\n"
					"Active in: any mode. Buffer fills only when the 30 cm detector fires (continuous\n"
					"mode); the per-tick predictive nudge applies whenever the buffer has enough events.");
			}

			// Chi-square re-anchor sub-detector. Pose-stream-only; works in
			// any state.
			ImGui::Checkbox("Chi-square re-anchor sub-detector", &CalCtx.reanchorChiSquareEnabled);
			if (ImGui::IsItemHovered(0)) {
				ImGui::SetTooltip("Mahalanobis distance between HMD-pose-from-rolling-velocity and observed\n"
					"HMD pose. Threshold at chi-square 6 DoF p<1e-4 (about 27.86). When fired,\n"
					"freezes rec A and rec C corrections for 500 ms so the existing 30 cm detector\n"
					"can confirm without our nudges contaminating the signal. Detection-only:\n"
					"never triggers recovery itself. Off by default.\n\n"
					"Active in: any mode.");
			}

			// Toggle-flip diagnostic. Compare each flag to its previous value
			// and emit a one-shot annotation on change. Statics are initialized
			// to the loaded-profile values on first frame so we do not log a
			// spurious flip just because the panel rendered for the first time.
			static bool s_prevAutoDetect = CalCtx.latencyAutoDetect;
			static bool s_prevGccPhat    = CalCtx.useGccPhatLatency;
			static bool s_prevCusum      = CalCtx.useCusumGeometryShift;
			static bool s_prevVelAware   = CalCtx.useVelocityAwareWeighting;
			static bool s_prevTukey      = CalCtx.useTukeyBiweight;
			static bool s_prevKalman     = CalCtx.useBlendFilter;
			static bool s_prevRestYaw    = CalCtx.restLockedYawEnabled;
			static bool s_prevPredRecov  = CalCtx.predictiveRecoveryEnabled;
			static bool s_prevChiSq      = CalCtx.reanchorChiSquareEnabled;
			logToggleFlip("latency_auto_detect",       s_prevAutoDetect, CalCtx.latencyAutoDetect);
			logToggleFlip("latency_use_gcc_phat",      s_prevGccPhat,    CalCtx.useGccPhatLatency);
			logToggleFlip("geometry_shift_use_cusum",  s_prevCusum,      CalCtx.useCusumGeometryShift);
			logToggleFlip("irls_velocity_aware",       s_prevVelAware,   CalCtx.useVelocityAwareWeighting);
			logToggleFlip("irls_use_tukey",            s_prevTukey,      CalCtx.useTukeyBiweight);
			logToggleFlip("blend_use_kalman",          s_prevKalman,     CalCtx.useBlendFilter);
			logToggleFlip("rest_locked_yaw",           s_prevRestYaw,    CalCtx.restLockedYawEnabled);
			logToggleFlip("predictive_recovery",       s_prevPredRecov,  CalCtx.predictiveRecoveryEnabled);
			logToggleFlip("reanchor_chi_square",       s_prevChiSq,      CalCtx.reanchorChiSquareEnabled);

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

	// Maintenance buttons grouped in their own panel so they don't read as
	// floating buttons under the speed/threshold matrix.
	ImGui::Spacing();
	ImGui::BeginGroupPanel("Maintenance", panel_size);
	if (ImGui::Button("Reset settings")) {
		CalCtx.ResetConfig();
	}
	if (ImGui::IsItemHovered()) {
		ImGui::SetTooltip("Reset all settings (jitter / speed / lock / etc.) to defaults.\n"
		                  "Does NOT clear your calibrated profile -- only the tunables.");
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
	ImGui::EndGroupPanel();

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

	// Recording status + toggle. The log file IS the recording -- there's no
	// separate "start/stop recording" notion. Putting the toggle here means
	// the user can flip logging on right where they're managing the log
	// files, instead of having to dig into Settings.
	ImGui::Checkbox("Enable debug logging", &Metrics::enableLogs);
	if (ImGui::IsItemHovered()) {
		ImGui::SetTooltip("Write a per-tick CSV of calibration state to %%LocalAppDataLow%%\\SpaceCalibrator\\Logs\\\n"
		                  "while this is on. The new log shows up in the list below as soon as the next calibration tick fires.");
	}
	ImGui::SameLine();
	if (Metrics::enableLogs) {
		ImGui::TextColored(ImVec4(0.45f, 0.85f, 0.45f, 1.0f),
			" ● a fresh CSV is being written this session");
	} else {
		ImGui::TextDisabled(" (off)");
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
		// Selectable list of log files, newest first. Each row has a small
		// Delete button on the right; clicking it removes the file from disk
		// and rebuilds the list. Selection state stays consistent (clamped
		// to the new file count) so the action buttons below don't reference
		// a stale index.
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

				// Reserve room for the Delete button on the right edge so
				// the Selectable doesn't fill the whole line.
				const float deleteBtnWidth = 70.0f;
				const float rowWidth = ImGui::GetContentRegionAvail().x - deleteBtnWidth - 8.0f;
				if (ImGui::Selectable(label, state.selectedIdx == i, 0, ImVec2(rowWidth, 0))) {
					state.selectedIdx = i;
				}
				ImGui::SameLine();
				char delId[64];
				snprintf(delId, sizeof delId, "Delete##log%d", i);
				if (ImGui::SmallButton(delId)) {
					// Best-effort delete -- DeleteFileW returns non-zero on
					// success. If it fails (file in use, permission), surface
					// the error in the same transient hint slot the Copy path
					// button uses.
					BOOL ok = DeleteFileW(state.files[i].fullPath.c_str());
					if (ok) {
						state.copyHint = "Deleted " + state.files[i].name;
					} else {
						state.copyHint = "Could not delete (file may be in use)";
					}
					state.copyHintExpireTime = ImGui::GetTime() + 2.5;
					RebuildLogsList();
					if (state.selectedIdx >= (int)state.files.size()) {
						state.selectedIdx = -1;
					}
					break; // list mutated, bail this iteration
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

	// Long-stall counter: HMD stalled for ≥30 ticks (~1.5 s). Previously the
	// calibration tick purged the sample buffer at this point; reverted
	// 2026-05-04 because the purge + warm-start re-anchor caused cumulative
	// drift on every HMD off/on cycle. The counter remains as a diagnostic
	// — frequent long-stalls still indicate a tracking-environment problem.
	const int kHmdLongStallThreshold = 30;
	const int curStalls = CalCtx.consecutiveHmdStalls;
	if (curStalls >= kHmdLongStallThreshold && s_lastSeenStallCount < kHmdLongStallThreshold) {
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
		ImGui::Text("HMD long-stall %.0fs ago (count: %d)",
			now - s_lastStallPurgeTime, s_stallPurgeCount);
		ImGui::PopStyleColor();
	} else if (s_stallPurgeCount == 0) {
		ImGui::TextDisabled("HMD long-stalls: 0 (last: never)");
	} else {
		ImGui::TextDisabled("HMD long-stalls: %d (last: %.0fs ago)", s_stallPurgeCount,
			now - s_lastStallPurgeTime);
	}
	if (ImGui::IsItemHovered()) {
		ImGui::SetTooltip("HMD long-stall: the headset stopped reporting fresh poses for ~1.5 seconds or more\n"
		                  "(SteamVR hiccup, tracking loss, headset taken off). Diagnostic counter only —\n"
		                  "calibration is no longer disturbed during a stall (rolling sample buffer ages out\n"
		                  "stale samples naturally on recovery). Frequent long-stalls suggest a tracking-\n"
		                  "environment problem (lighting, USB bandwidth, etc).");
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

		// (Jitter threshold moved to the Advanced tab -- it's a rarely-touched
		// knob, surfaced there alongside the rest of the deeper math settings.)

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

		// (Static recalibration toggle removed -- it's now always-on, gated
		// implicitly by Lock relative position. Independent devices have
		// nothing locked to snap to, so the feature is a no-op for them;
		// rigid setups get the snap-back automatically.)

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

		// (Enable debug logs toggle removed -- it lives in the Logs tab now,
		// where the user can flip it on right where they're managing the
		// log files.)

		ImGui::EndTable();
	}

	ImGui::EndGroupPanel(); // Settings

	// Calibration speed -- moved here from above the tab bar (was being
	// rendered inline in BuildMenu, stacking the speed picker on top of the
	// device + action rows and pushing the tabs off-screen on small
	// windows). Lives in Settings rather than Advanced because it's a
	// common knob most users want one click away.
	ImGui::Spacing();
	ImGui::BeginGroupPanel("Calibration speed", panelSize);
	{
		auto speed = CalCtx.calibrationSpeed;
		struct Opt { const char* label; CalibrationContext::Speed value; const char* tooltip; };
		const Opt opts[] = {
			{ "Auto",      CalibrationContext::AUTO,
				"Pick FAST / SLOW / VERY SLOW automatically from observed jitter. Recommended." },
			{ "Fast",      CalibrationContext::FAST,
				"100 samples (~5 s buffer). Best for low-jitter setups where motion is varied." },
			{ "Slow",      CalibrationContext::SLOW,
				"250 samples (~12 s). Better for moderately noisy trackers." },
			{ "Very Slow", CalibrationContext::VERY_SLOW,
				"500 samples (~25 s). Use for noisy IMU-based body trackers." },
		};
		for (size_t i = 0; i < sizeof(opts) / sizeof(opts[0]); ++i) {
			if (i > 0) ImGui::SameLine();
			if (ImGui::RadioButton(opts[i].label, speed == opts[i].value)) {
				CalCtx.calibrationSpeed = opts[i].value;
				SaveProfile(CalCtx);
			}
			if (ImGui::IsItemHovered()) {
				ImGui::SetTooltip("%s", opts[i].tooltip);
			}
		}
	}
	ImGui::EndGroupPanel();

	// Chaperone -- moved here from BuildMenu for the same reason. Copy/Paste
	// + auto-apply checkbox; Paste only meaningful when the profile already
	// has stored bounds.
	ImGui::Spacing();
	ImGui::BeginGroupPanel("Chaperone bounds", panelSize);
	{
		ImGui::BeginDisabled(!IsVRReady());
		if (ImGui::Button("Copy chaperone bounds to profile")) {
			LoadChaperoneBounds();
			SaveProfile(CalCtx);
		}
		ImGui::EndDisabled();
		if (!IsVRReady() && ImGui::IsItemHovered()) {
			ImGui::SetTooltip("Waiting for SteamVR.");
		}
		if (CalCtx.chaperone.valid) {
			ImGui::SameLine();
			ImGui::BeginDisabled(!IsVRReady());
			if (ImGui::Button("Paste chaperone bounds")) {
				ApplyChaperoneBounds();
			}
			ImGui::EndDisabled();
			if (!IsVRReady() && ImGui::IsItemHovered()) {
				ImGui::SetTooltip("Waiting for SteamVR.");
			}

			if (ImGui::Checkbox("Paste automatically when geometry resets", &CalCtx.chaperone.autoApply)) {
				SaveProfile(CalCtx);
			}
		} else {
			ImGui::TextDisabled("(No bounds saved in profile yet -- press Copy first.)");
		}
	}
	ImGui::EndGroupPanel();

	// Recenter playspace -- manual chaperone shift for "the headset
	// re-localized and now the chaperone is in the wrong place around me."
	// User-triggered (not automatic) because they can see the misalignment
	// directly. We don't claim to detect "when" they should click; we just
	// surface the most recent detected event below as a nudge so they know
	// roughly when their tracking last shifted.
	//
	// Two-step interaction: button opens a confirm popup that tells the
	// user where to stand FIRST, since the recenter snapshot uses their
	// current position as the new chaperone centre. Without the prompt the
	// user might click before getting into position and end up with the
	// chaperone offset from where they actually meant.
	ImGui::Spacing();
	ImGui::BeginGroupPanel("Recenter playspace", panelSize);
	{
		ImGui::TextWrapped(
			"If your headset has re-localized and the chaperone bounds are now "
			"in the wrong place relative to where you actually are, click the "
			"button below. Floor height and yaw are preserved -- only the "
			"chaperone's X/Z origin shifts.");
		ImGui::Spacing();

		ImGui::BeginDisabled(!IsVRReady());
		if (ImGui::Button("Recenter playspace to my current position")) {
			ImGui::OpenPopup("Recenter playspace?");
		}
		ImGui::EndDisabled();
		if (!IsVRReady() && ImGui::IsItemHovered()) {
			ImGui::SetTooltip("Waiting for SteamVR.");
		}

		// Confirm modal. Keeps the click intentional: the user has to
		// physically position themselves before the snapshot, which is the
		// part the modal copy is for.
		if (ImGui::BeginPopupModal("Recenter playspace?", nullptr,
				ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoMove)) {
			ImGui::TextWrapped(
				"Stand at the centre of your play area, facing forward, then "
				"click Confirm. Your headset's current position will become "
				"the new chaperone centre.");
			ImGui::Spacing();
			ImGui::TextDisabled(
				"Floor height and yaw are preserved. Only X/Z (where you "
				"are in the room) shifts. Reversible: click again later if "
				"you over-shoot.");
			ImGui::Spacing();
			if (ImGui::Button("Confirm##recenter_confirm",
					ImVec2(180, 0))) {
				const bool ok = RecenterPlayspaceToCurrentHmd();
				if (!ok) {
					CalCtx.Log("Recenter playspace failed -- check the log for details.\n");
				}
				ImGui::CloseCurrentPopup();
			}
			ImGui::SameLine();
			if (ImGui::Button("Cancel##recenter_cancel",
					ImVec2(120, 0))) {
				ImGui::CloseCurrentPopup();
			}
			ImGui::EndPopup();
		}

		// Surface the detector's most recent event. The detector is
		// logging-only; this is the user-visible side of the same data.
		double age = 0, deltaM = 0, deltaDeg = 0;
		if (LastDetectedRelocalization(age, deltaM, deltaDeg)) {
			ImGui::TextDisabled(
				"Last detected re-localization: %.0f s ago (%.1f cm, %.1f deg).",
				age, deltaM * 100.0, deltaDeg);
			if (ImGui::IsItemHovered()) {
				ImGui::SetTooltip("The drift detector logs HMD re-localization events when it sees the\n"
				                  "HMD's reported pose jump while base stations and body trackers stayed put.\n"
				                  "If this number is small (< 60 s) and you're noticing chaperone drift,\n"
				                  "clicking the button above is probably the right fix.");
			}
		} else {
			ImGui::TextDisabled(
				"No re-localization detected this session.");
			if (ImGui::IsItemHovered()) {
				ImGui::SetTooltip("Detector requires at least 2 Lighthouse base stations to fire.\n"
				                  "If you don't have base stations, the detector is silent and the\n"
				                  "button above is your only signal.");
			}
		}
	}
	ImGui::EndGroupPanel();

	// Wizard / reset actions, grouped in their own panel so they don't read
	// as floating buttons under the Settings table.
	ImGui::Spacing();
	ImGui::BeginGroupPanel("Maintenance", panelSize);
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
	ImGui::EndGroupPanel();
}

void CCal_BasicInfo() {
	ImVec2 panelSize{ ImGui::GetWindowContentRegionMax().x - ImGui::GetWindowContentRegionMin().x, 0 };

	// (Tip moved to global footer; mismatch banner moved below the Actions
	// panel per the user request -- placement preserves visibility but stops
	// pushing the device readout off-screen on small windows.)

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

	// (Removed 2026-05-04: "Recalibrate from scratch" button. The wedge case
	// it served as escape-hatch for is now handled silently by the load-time
	// guard in Configuration.cpp::ParseProfile and the runtime detector in
	// Calibration.cpp::CalibrationTick. Per
	// feedback_no_button_to_recover_broken_tracking.md (memory): a recovery
	// flow whose precondition is broken tracking can't require interaction
	// from the user via that same broken tracking — auto-detect + auto-fix
	// is the only correct shape.)

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

	// Profile-mismatch banner lives inside the Actions panel. The banner
	// only renders when the active HMD's tracking system disagrees with
	// the saved profile, so users without a mismatch see no extra rows.
	// Inside-the-panel placement keeps the recovery actions (Clear
	// profile / Recalibrate) visually grouped with the rest of the
	// session-control buttons above.
	DrawProfileMismatchBanner();

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

		// (Jitter threshold and Recalibration threshold moved to the Advanced
		// tab -- they're rarely-touched knobs and were padding the Basic
		// settings table without justifying their space here.)

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

		// (Enable debug logs toggle moved to the Logs tab where the user is
		// already managing log files. The checkbox here was redundant.)

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
		// (Profile-mismatch banner moved below the action buttons -- it used
		// to push the Start / Continuous / Edit / Clear row down with a
		// multi-line warning. The buttons are what the user is actually
		// reaching for; surface them first.)

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

		// Profile-mismatch banner (relocated): renders only when the saved
		// profile expects a different HMD tracking system than the current
		// HMD reports. Sits below the action buttons so the buttons stay
		// at the top -- the banner is informative + recovery actions
		// (Clear profile / Recalibrate), not a blocking modal.
		if (CalCtx.validProfile && !CalCtx.enabled)
		{
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
			ImGui::Spacing();
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
		}

		// (Chaperone Copy/Paste buttons + autoApply moved to the Settings
		// tab as the "Chaperone bounds" group panel.)
		// (Calibration Speed picker moved to the Settings tab as the
		// "Calibration speed" group panel.)
		// Both used to render inline here, stacking on top of the action
		// buttons and pushing the tab bar off-screen on small windows.
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
			// Phase banner. The two-phase one-shot flow (Rotation → Translation)
			// is invisible to the user otherwise; surfacing the active phase and
			// what motion to do removes the "I waved but the bar didn't move"
			// confusion that the old combined-gate flow created.
			if (CalCtx.state == CalibrationState::Rotation) {
				ImGui::TextDisabled("Phase 1 of 2: Rotation");
				ImGui::TextWrapped("Rotate the tracker through different orientations (≥ 90° between some pair).");
			} else if (CalCtx.state == CalibrationState::Translation) {
				ImGui::TextDisabled("Phase 2 of 2: Translation");
				ImGui::TextWrapped("Wave the tracker through ~30 cm on each of left/right, up/down, and forward/back.");
			} else {
				ImGui::TextDisabled("Motion coverage");
			}
			ImGui::Spacing();

			const float trDiv = (float)Metrics::translationDiversity.last();
			// Latch the rotation bar at 100 % once we've moved into Translation
			// phase: the rotation samples are frozen on CalibrationCalc and the
			// live Metrics::rotationDiversity now reflects only fresh translation
			// samples (which initially drops to 0 and stays low). Showing the
			// raw metric here would make the just-achieved rotation appear to
			// regress -- correct from the metric's POV but wrong UX.
			const float rotDiv = (CalCtx.state == CalibrationState::Translation)
				? 1.0f
				: (float)Metrics::rotationDiversity.last();

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
				// Per-axis ranges of the target tracker across the live sample
				// buffer. Whichever axis is smallest is what's pinning the
				// Translation% score (= min / 30 cm). Naming axes in user
				// terms (left/right, up/down, fwd/back) is more useful than
				// raw X/Y/Z because the user is moving a hand, not thinking
				// in tracker-space coordinates.
				const Eigen::Vector3d r = Metrics::translationAxisRangesCm.last();
				int minIdx = 0; double minR = r(0);
				for (int i = 1; i < 3; ++i) if (r(i) < minR) { minR = r(i); minIdx = i; }
				static const char* kAxisName[] = {
					"X (left/right)", "Y (up/down)", "Z (forward/back)"
				};
				ImGui::SetTooltip(
					"Translation coverage: how much you've moved the tracker along all three axes.\n"
					"Wave it ~30 cm in each of left/right, up/down, and forward/back to fill this bar.\n"
					"Green = enough variety for a clean calibration.\n"
					"\n"
					"Current ranges: X=%.0f cm, Y=%.0f cm, Z=%.0f cm\n"
					"Weakest axis: %s",
					r(0), r(1), r(2), kAxisName[minIdx]);
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

// =============================================================================
// Finger Smoothing tab — Index Knuckles per-bone slerp smoothing.
//
// The driver hooks IVRDriverInputInternal::UpdateSkeletonComponent (a private
// Valve interface, reachable via GetGenericInterface; layout recovered by
// parsing the public IVRDriverInput pimpl thunks at install time -- see
// SkeletalHookInjector.cpp). On every UpdateSkeleton call for a recognised
// /skeleton/hand/{left,right} component, each bone is slerp'd toward the
// incoming pose by a factor derived from the smoothness slider. Per-finger
// disable bits let the user isolate one finger if its smoothing produces an
// artifact without disabling the whole feature.
//
// Default-OFF and skip-if-default-on-save: a user who never opens this tab
// sees byte-identical profile JSON and zero behaviour change.
// =============================================================================
// Non-static so Calibration.cpp's InitCalibrator() (and any future caller)
// can push the current persisted config without going through the slider
// handler. Forward-declared in Calibration.cpp.
void SendFingerSmoothingConfig(CalibrationContext &ctx)
{
	if (!Driver.IsConnected()) return;
	protocol::Request req(protocol::RequestSetFingerSmoothing);
	req.setFingerSmoothing.master_enabled = ctx.fingerSmoothingEnabled;
	int s = ctx.fingerSmoothingStrength;
	if (s < 0) s = 0;
	if (s > 100) s = 100;
	req.setFingerSmoothing.smoothness = (uint8_t)s;
	req.setFingerSmoothing.finger_mask = ctx.fingerSmoothingMask;
	req.setFingerSmoothing._reserved = 0;
	try {
		Driver.SendBlocking(req);
	} catch (const std::exception &e) {
		std::cerr << "[FingerSmoothing] IPC send failed: " << e.what() << std::endl;
	}
}

void CCal_DrawFingerSmoothing()
{
	auto &ctx = CalCtx;

	ImGui::Text("Finger smoothing");
	ImGui::TextDisabled("(Index Knuckles only — smooths per-frame finger bone updates before VRChat sees them.)");
	ImGui::Spacing();

	bool dirty = false;

	bool master = ctx.fingerSmoothingEnabled;
	if (ImGui::Checkbox("Enable finger smoothing", &master)) {
		ctx.fingerSmoothingEnabled = master;
		dirty = true;
	}
	if (ImGui::IsItemHovered()) {
		ImGui::SetTooltip(
			"Master kill switch. When off, the driver passes Knuckles bone\n"
			"arrays through untouched -- exactly the same behaviour as a build\n"
			"without the finger-smoothing feature compiled in.");
	}

	ImGui::Spacing();
	ImGui::BeginDisabled(!ctx.fingerSmoothingEnabled);

	int strength = ctx.fingerSmoothingStrength;
	if (ImGui::SliderInt("Strength##fingers", &strength, 0, 100, "%d%%")) {
		if (strength < 0) strength = 0;
		if (strength > 100) strength = 100;
		ctx.fingerSmoothingStrength = strength;
		dirty = true;
	}
	if (ImGui::IsItemHovered()) {
		ImGui::SetTooltip(
			"0   = no smoothing (each frame snaps to incoming bones).\n"
			"50  = moderate (good starting point).\n"
			"100 = heavy lag (slerp factor 0.05 per frame). Never fully freezes.\n"
			"Drag the slider live and feel the change immediately in-game.");
	}

	ImGui::Spacing();
	ImGui::Text("Per-finger toggles (uncheck to bypass that finger only)");
	ImGui::TextDisabled("Useful for isolating a finger whose smoothing produces an artifact.");

	const char *fingerLabels[5] = { "Thumb", "Index", "Middle", "Ring", "Pinky" };
	const char *handLabels[2]   = { "Left", "Right" };

	if (ImGui::BeginTable("fingers_grid", 6, ImGuiTableFlags_BordersInnerH | ImGuiTableFlags_BordersInnerV)) {
		ImGui::TableSetupColumn("Hand");
		for (int f = 0; f < 5; ++f) ImGui::TableSetupColumn(fingerLabels[f]);
		ImGui::TableHeadersRow();

		for (int hand = 0; hand < 2; ++hand) {
			ImGui::TableNextRow();
			ImGui::TableNextColumn();
			ImGui::Text("%s", handLabels[hand]);
			for (int finger = 0; finger < 5; ++finger) {
				int bit = hand * 5 + finger;
				ImGui::TableNextColumn();
				bool enabled = ((ctx.fingerSmoothingMask >> bit) & 1) != 0;
				ImGui::PushID(bit);
				if (ImGui::Checkbox("##fingerbit", &enabled)) {
					if (enabled) ctx.fingerSmoothingMask |= (uint16_t)(1u << bit);
					else         ctx.fingerSmoothingMask &= (uint16_t)~(1u << bit);
					dirty = true;
				}
				ImGui::PopID();
			}
		}
		ImGui::EndTable();
	}

	ImGui::Spacing();
	if (ImGui::Button("Enable all fingers")) {
		if (ctx.fingerSmoothingMask != protocol::kAllFingersMask) {
			ctx.fingerSmoothingMask = protocol::kAllFingersMask;
			dirty = true;
		}
	}
	ImGui::SameLine();
	if (ImGui::Button("Disable all fingers")) {
		if (ctx.fingerSmoothingMask != 0) {
			ctx.fingerSmoothingMask = 0;
			dirty = true;
		}
	}

	ImGui::EndDisabled();

	ImGui::Spacing();
	ImGui::Separator();
	if (Driver.IsConnected()) {
		ImGui::TextColored(ImVec4(0.20f, 0.80f, 0.30f, 1.0f),
			"Driver connected -- changes apply live as you drag the slider.");
	} else {
		ImGui::TextColored(ImVec4(0.85f, 0.30f, 0.25f, 1.0f),
			"Driver not connected. Settings will save locally and apply once SteamVR is running.");
	}

	if (dirty) {
		SaveProfile(ctx);
		SendFingerSmoothingConfig(ctx);
	}
}

