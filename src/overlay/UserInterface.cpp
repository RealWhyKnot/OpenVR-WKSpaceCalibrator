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
static void CCal_DrawRecordingsPanel();

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

	// First-run auto-open of the setup wizard. Once the user finishes or
	// dismisses it, wizardCompleted is persisted to the profile and we
	// never auto-show again. The user can re-open it from the Advanced tab.
	{
		static bool s_firstRunChecked = false;
		if (!s_firstRunChecked) {
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

	// Driver-connection dot. Green when the IPC pipe is alive; red otherwise.
	// Text after the dot reflects the same state. The version we show is the
	// build-time client protocol version — IPCClient::Connect() throws unless
	// the driver reports the same number, so when IsConnected() is true that
	// number is also the live driver version.
	const bool driverConnected = Driver.IsConnected();
	if (driverConnected) {
		DrawStatusDot(IM_COL32(80, 200, 120, 255));
		ImGui::TextColored(ImVec4(0.5f, 0.85f, 0.55f, 1.0f),
			"Driver: connected (v%u)", (unsigned)protocol::Version);
	} else {
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

void CCal_BasicInfo();
void CCal_DrawSettings();
void CCal_DrawPredictionSuppression();

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
	//   - Recordings: only when debug logs are on -- replay captured motion
	//               against the live math.
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

		// Recordings tab: only shown when debug logging is enabled, since the
		// whole point is to load and replay debug-log files.  Always visible
		// once logs are on -- the choice "I want to debug" is the user signalling
		// they're ready for the more advanced surface.
		if (Metrics::enableLogs) {
			if (ImGui::BeginTabItem("Recordings")) {
				CCal_DrawRecordingsPanel();
				ImGui::EndTabItem();
			}
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

	ImGui::BeginGroupPanel("Tip", panel_size);
	ImGui::Text("Hover over settings to learn more about them! Right-click any slider to reset it to default.");
	ImGui::EndGroupPanel();

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

// === Recordings (debug log replay) ========================================
// State persists for the lifetime of the process. The recordings list is
// rebuilt on demand (when the user opens the tab or hits "Refresh"); the
// loaded recording + last replay result are kept until the user picks a
// different file or closes the panel.
namespace {

struct RecordingsPanelState {
	std::vector<spacecal::replay::LogFileEntry> files;
	int selectedIdx = -1;
	bool listBuilt = false;
	std::string loadError;
	spacecal::replay::LoadedRecording loaded;
	bool loadedValid = false;
	spacecal::replay::ReplayOptions replayOpts;
	spacecal::replay::ReplayResult lastResult;
	bool resultPresent = false;
};

RecordingsPanelState& RecordingsState() {
	static RecordingsPanelState s;
	return s;
}

void RebuildRecordingsList() {
	auto& s = RecordingsState();
	s.files = spacecal::replay::ListRecordings();
	s.listBuilt = true;
	if (s.selectedIdx >= (int)s.files.size()) s.selectedIdx = -1;
}

// Format file age relative to "now" using FILETIME math. We don't bother with
// localized strings — "5 min ago" / "2 hours ago" / "3 days ago" is plenty
// detail for picking the right recording out of a list.
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

} // namespace

static void CCal_DrawRecordingsPanel() {
	auto& state = RecordingsState();

	ImGui::TextWrapped(
		"Record a motion sequence by leaving \"Enable debug logs\" on while you reproduce a problem. "
		"Each session writes a CSV file at\n"
		"  %%LocalAppDataLow%%\\SpaceCalibrator\\Logs\\spacecal_log.<date>T<time>.txt\n"
		"Pick one below to replay the captured raw poses through a fresh calibration math instance — "
		"useful for confirming a fix changes behaviour against the same input.");
	ImGui::Spacing();

	// Recording status: just reflects the live debug-log toggle. We don't have
	// a separate "recording" notion; the log file IS the recording.
	if (Metrics::enableLogs) {
		ImGui::TextColored(ImVec4(0.45f, 0.85f, 0.45f, 1.0f),
			"● Recording: ON  (debug logs being written)");
	} else {
		ImGui::TextColored(ImVec4(0.85f, 0.55f, 0.45f, 1.0f),
			"○ Recording: OFF  (enable \"Enable debug logs\" on the Settings tab to start)");
	}

	ImGui::Spacing();
	ImGui::Separator();
	ImGui::TextDisabled("Available recordings");

	ImGui::SameLine();
	if (ImGui::SmallButton("Refresh##recordings")) {
		RebuildRecordingsList();
	}

	if (!state.listBuilt) RebuildRecordingsList();

	if (state.files.empty()) {
		ImGui::TextDisabled("(No recordings found in the Logs directory.)");
	} else {
		// Compact selectable list. Newest at top (sorted by ListRecordings).
		const float listHeight = ImGui::GetTextLineHeightWithSpacing() * 6.5f;
		if (ImGui::BeginChild("##recordings_list",
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
					// Reset previous load/replay state so the user sees a clean
					// "loaded but not yet replayed" view.
					state.loadError.clear();
					state.loaded = {};
					state.loadedValid = false;
					state.resultPresent = false;
				}
			}
		}
		ImGui::EndChild();
	}

	ImGui::Spacing();

	// Action row: Load + Replay buttons, gated on a selection being present.
	const bool haveSelection = state.selectedIdx >= 0
		&& state.selectedIdx < (int)state.files.size();

	ImGui::BeginDisabled(!haveSelection);
	if (ImGui::Button("Load selected##recordings")) {
		const auto& f = state.files[state.selectedIdx];
		// Convert wide path to UTF-8 for std::ifstream below.
		const int n = WideCharToMultiByte(CP_UTF8, 0, f.fullPath.c_str(), -1, nullptr, 0, nullptr, nullptr);
		std::string utf8Path(n > 1 ? n - 1 : 0, '\0');
		if (n > 1) WideCharToMultiByte(CP_UTF8, 0, f.fullPath.c_str(), -1, utf8Path.data(), n, nullptr, nullptr);
		state.loaded = spacecal::replay::LoadRecording(utf8Path);
		state.loadedValid = state.loaded.error.empty();
		state.loadError = state.loaded.error;
		state.resultPresent = false;
	}
	ImGui::SameLine();
	ImGui::BeginDisabled(!state.loadedValid);
	if (ImGui::Button("Replay##recordings")) {
		state.lastResult = spacecal::replay::RunReplay(state.loaded, state.replayOpts);
		state.resultPresent = true;
	}
	ImGui::EndDisabled();
	ImGui::EndDisabled();

	if (!state.loadError.empty()) {
		ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 0.55f, 0.55f, 1.0f));
		ImGui::TextWrapped("Load error: %s", state.loadError.c_str());
		ImGui::PopStyleColor();
	}

	if (state.loadedValid) {
		ImGui::Spacing();
		ImGui::Separator();
		ImGui::TextDisabled("Loaded recording");
		const auto& m = state.loaded.meta;
		ImGui::Text("Rows:        %d", (int)state.loaded.rows.size());
		if (!m.buildStamp.empty())  ImGui::Text("Captured by: build %s (%s)",
			m.buildStamp.c_str(),
			m.buildChannel.empty() ? "?" : m.buildChannel.c_str());
		if (!m.hmdModel.empty())    ImGui::Text("HMD:         %s [%s]",
			m.hmdModel.c_str(),
			m.hmdTrackingSystem.empty() ? "?" : m.hmdTrackingSystem.c_str());
		if (!m.windowsVersion.empty()) ImGui::Text("OS:          Windows %s", m.windowsVersion.c_str());

		ImGui::Spacing();
		ImGui::Separator();
		ImGui::TextDisabled("Replay options");
		// SliderFloat takes float& but ReplayOptions stores double for API symmetry with
		// the live calibration knobs; bounce through float locals.
		float thr = (float)state.replayOpts.threshold;
		if (ImGui::SliderFloat("Recalibration threshold", &thr, 1.01f, 10.0f, "%1.2f")) {
			state.replayOpts.threshold = thr;
		}
		float maxRel = (float)state.replayOpts.maxRelError;
		if (ImGui::SliderFloat("Max rel-pose error (m)", &maxRel, 0.001f, 0.05f, "%1.4f")) {
			state.replayOpts.maxRelError = maxRel;
		}
		ImGui::Checkbox("Continuous mode (vs. one-shot)", &state.replayOpts.continuous);
		ImGui::Checkbox("Ignore outliers", &state.replayOpts.ignoreOutliers);
	}

	if (state.resultPresent) {
		ImGui::Spacing();
		ImGui::Separator();
		ImGui::TextDisabled("Replay result");

		const auto& r = state.lastResult;
		if (!r.error.empty()) {
			ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 0.55f, 0.55f, 1.0f));
			ImGui::TextWrapped("%s", r.error.c_str());
			ImGui::PopStyleColor();
		} else {
			ImGui::Text("Rows replayed:    %d", r.rowsReplayed);
			if (state.replayOpts.continuous) {
				ImGui::Text("Accepts:          %d", r.accepts);
				ImGui::Text("Rejects:          %d", r.rejects);
			}
			ImGui::Text("Watchdog resets:  %d", r.watchdogResets);

			if (r.finalTransformValid) {
				const auto t = r.finalTransform.translation();
				const auto eul = r.finalTransform.rotation().eulerAngles(2, 1, 0) * (180.0 / EIGEN_PI);
				ImGui::Text("Final translation (m):   x=%.4f  y=%.4f  z=%.4f", t.x(), t.y(), t.z());
				ImGui::Text("Final rotation ZYX (deg): yaw=%.3f  pitch=%.3f  roll=%.3f",
					eul.x(), eul.y(), eul.z());
				ImGui::Text("Final RMS error:  %.3f mm", r.finalErrorMm);
			} else {
				ImGui::TextColored(ImVec4(0.95f, 0.65f, 0.45f, 1.0f),
					"No valid calibration produced from this replay.");
			}
		}
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

void CCal_BasicInfo() {
	// Mirror the profile-mismatch banner from BuildMenu so it's visible while
	// the user is in continuous mode. Returns true if drawn — we just let it
	// stack above the device-info table either way.
	DrawProfileMismatchBanner();

	// --- Watchdog/HMD-stall visibility row ---------------------------------
	// Show the running stuck-loop watchdog count so users (and bug reports)
	// have ground truth about whether the safety net has been firing. Flip
	// the indicator amber for ~15 s after a fresh increment, similar to the
	// continuous-recalibration banner pattern we use elsewhere.
	{
		static int s_lastSeenWatchdog = -1;
		static double s_lastWatchdogResetTime = 0.0;
		static int s_lastSeenStallCount = 0;
		static int s_stallPurgeCount = 0;
		static double s_lastStallPurgeTime = 0.0;

		const int wdResets = GetWatchdogResetCount();
		const double now = ImGui::GetTime();
		if (s_lastSeenWatchdog < 0) {
			// First UI tick: initialise without flagging the existing count as
			// "just happened".
			s_lastSeenWatchdog = wdResets;
		} else if (wdResets != s_lastSeenWatchdog) {
			s_lastSeenWatchdog = wdResets;
			s_lastWatchdogResetTime = now;
		}

		// HMD-stall purge is detected by watching consecutiveHmdStalls cross
		// the threshold the calibration tick uses (30). We can't read the
		// constant directly without touching Calibration.cpp, but the only
		// thing we care about is "the value is high enough that the purge
		// already fired" — pick a value at-or-above the source MaxHmdStalls.
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
			ImGui::Text("Watchdog reset %.0fs ago — recollecting samples (count: %d)",
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
			ImGui::Text("HMD-stall purge %.0fs ago — recollecting samples (count: %d)",
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
		ImGui::Separator();
	}

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

	float width = ImGui::GetWindowContentRegionWidth(), scale = 1.0f;

	// Recovery affordances: Cancel, Restart sampling, Pause. The old
	// "Debug: Mark logs" debug button was removed — annotations are written
	// automatically when the watchdog fires or the user presses the
	// recalibrate buttons; that's enough to grep the log.
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

	// === Common settings ===================================================
	// The handful of settings most users actually touch.  Moved here from the
	// old "Settings" tab so a basic user has everything they need on one page
	// without having to know what a "calibration speed" is.
	ImVec2 panelSize{ ImGui::GetWindowContentRegionMax().x - ImGui::GetWindowContentRegionMin().x, 0 };

	ImGui::BeginGroupPanel("Common settings", panelSize);

	// Jitter threshold
	ImGui::Text("Jitter threshold");
	ImGui::SameLine();
	ImGui::PushID("basic_jitter_threshold");
	ImGui::SliderFloat("##basic_jitter_threshold_slider", &CalCtx.jitterThreshold, 0.1f, 10.0f, "%1.1f", 0);
	if (ImGui::IsItemHovered()) {
		ImGui::SetTooltip("Controls how much jitter will be allowed for calibration.\n"
			"Higher values allow worse tracking to calibrate, but may result in poorer tracking.");
	}
	AddResetContextMenu("basic_jitter_threshold_ctx", [] { CalCtx.jitterThreshold = 3.0f; });
	ImGui::PopID();

	// Recalibration threshold
	ImGui::Text("Recalibration threshold");
	ImGui::SameLine();
	ImGui::PushID("basic_recalibration_threshold");
	ImGui::SliderFloat("##basic_recalibration_threshold_slider", &CalCtx.continuousCalibrationThreshold, 1.01f, 10.0f, "%1.1f", 0);
	if (ImGui::IsItemHovered()) {
		ImGui::SetTooltip("Controls how good the calibration must be before realigning the trackers.\n"
			"Higher values cause calibration to happen less often, and may be useful for systems with lots of tracking drift.");
	}
	AddResetContextMenu("basic_recalibration_threshold_ctx", [] { CalCtx.continuousCalibrationThreshold = 1.5f; });
	ImGui::PopID();

	// Lock relative position -- tristate (Auto / On / Off).
	// Auto detects rigid attachment from observed motion; we default to it
	// because most users don't know whether their target is glued to the
	// HMD or not, and the failure mode of an undetected rigid setup is
	// "calibration slowly drifts as the math chases sensor noise".
	{
		ImGui::Text("Lock relative position");
		ImGui::SameLine();
		const char* labels[] = { "Off##lock", "On##lock", "Auto##lock" };
		const auto modes = {
			CalibrationContext::LockMode::OFF,
			CalibrationContext::LockMode::ON,
			CalibrationContext::LockMode::AUTO
		};
		int i = 0;
		for (auto m : modes) {
			ImGui::SameLine();
			if (ImGui::RadioButton(labels[i], CalCtx.lockRelativePositionMode == m)) {
				CalCtx.lockRelativePositionMode = m;
				SaveProfile(CalCtx);
			}
			++i;
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

		// Show resolved state when in Auto so the user can see what the detector decided.
		if (CalCtx.lockRelativePositionMode == CalibrationContext::LockMode::AUTO) {
			ImGui::PushStyleColor(ImGuiCol_Text, ImGui::GetStyleColorVec4(ImGuiCol_TextDisabled));
			if (CalCtx.autoLockEffectivelyLocked) {
				ImGui::TextWrapped("    Auto: locked (detected as rigidly attached, %d samples)",
					(int)CalCtx.autoLockHistory.size());
			} else if (CalCtx.autoLockHistory.size() < 30) {
				ImGui::TextWrapped("    Auto: collecting motion data (%d/30 samples)",
					(int)CalCtx.autoLockHistory.size());
			} else {
				ImGui::TextWrapped("    Auto: unlocked (devices move independently)");
			}
			ImGui::PopStyleColor();
		}
	}

	// Require triggers
	if (ImGui::Checkbox("Require trigger press to apply", &CalCtx.requireTriggerPressToApply)) {
		SaveProfile(CalCtx);
	}
	if (ImGui::IsItemHovered()) {
		ImGui::SetTooltip("If on, only apply the calibrated offset while a controller trigger is held.\n"
			"Useful for verifying the result before committing.");
	}

	// Recalibrate on movement (default on)
	if (ImGui::Checkbox("Recalibrate on movement", &CalCtx.recalibrateOnMovement)) {
		SaveProfile(CalCtx);
	}
	if (ImGui::IsItemHovered()) {
		ImGui::SetTooltip("When the calibration math updates, only blend the new offset in while you're actually moving.\n"
			"Stationary users (e.g. lying down) won't see phantom body shifts; the catch-up happens during natural motion.\n"
			"Default ON. Turn off to get instantaneous time-based blending regardless of motion state.");
	}

	ImGui::EndGroupPanel();

	// Debug logs toggle.  Outside the Common settings panel so it's clearly
	// a separate concern (writes a file vs. tweaks a runtime knob).  Stays
	// in Basic because a typical bug report starts with "enable logs and
	// reproduce" — hiding it under Advanced would force every reporter to
	// read a manual first.
	ImGui::Spacing();
	ImGui::Checkbox("Enable debug logs", &Metrics::enableLogs);
	if (ImGui::IsItemHovered()) {
		ImGui::SetTooltip("Write a per-tick CSV log of calibration state to\n"
		                  "%%LocalAppDataLow%%\\SpaceCalibrator\\Logs\\spacecal_log.<date>.txt\n"
		                  "Useful for bug reports.  Also unlocks the Recordings tab where you can replay\n"
		                  "a captured session against the live calibration math.");
	}

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
			if (ImGui::Button("Recalibrate")) {
				ImGui::OpenPopup("Calibration Progress");
				StartCalibration();
			}
			ImGui::Text("");
		}

		float width = ImGui::GetWindowContentRegionWidth(), scale = 1.0f;
		if (CalCtx.validProfile)
		{
			width -= style.FramePadding.x * 4.0f;
			scale = 1.0f / 4.0f;
		}

		if (ImGui::Button("Start Calibration", ImVec2(width * scale, ImGui::GetTextLineHeight() * 2)))
		{
			ImGui::OpenPopup("Calibration Progress");
			StartCalibration();
		}

		ImGui::SameLine();
		if (ImGui::Button("Continuous Calibration", ImVec2(width * scale, ImGui::GetTextLineHeight() * 2))) {
			StartContinuousCalibration();
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
		if (ImGui::Button("Copy Chaperone Bounds to profile", ImVec2(width * scale, ImGui::GetTextLineHeight() * 2)))
		{
			LoadChaperoneBounds();
			SaveProfile(CalCtx);
		}

		if (CalCtx.chaperone.valid)
		{
			ImGui::SameLine();
			if (ImGui::Button("Paste Chaperone Bounds", ImVec2(width * scale, ImGui::GetTextLineHeight() * 2)))
			{
				ApplyChaperoneBounds();
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

