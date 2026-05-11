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
#include "UserInterfaceFooter.h"
#include "UserInterfaceBanners.h"

#include <thread>
#include <string>
#include <vector>
#include <algorithm>
#include <shellapi.h>
#include <shlobj_core.h>
#include <imgui/imgui.h>
#include "imgui_extensions.h"

extern SCIPCClient Driver;

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
void CCal_DrawLogsPanel();
static void BuildMainWindowContents(bool runningInOverlay_);

// Forward decls for the tab content called from both modes. CCal_BasicInfo /
// CCal_DrawSettings were declared near the continuous-mode tab bar; the
// non-continuous flow needs them in scope at BuildMainWindow time too, so the
// decls live at file scope. Prediction and finger smoothing relocated to the
// Smoothing overlay (Protocol v12 migration, 2026-05-11).
void CCal_BasicInfo();
void CCal_DrawSettings();
static void OneShot_DrawSettings();

bool runningInOverlay;
bool s_inUmbrella = false;

void CCal_SetInUmbrella(bool inUmbrella)
{
	s_inUmbrella = inUmbrella;
}

// Update-check state. Lives for the lifetime of the process. We kick off the
// initial check on the first BuildMainWindow tick so we don't slow startup.
// The Updater is reused across retries (it transitions back to Idle on
// failure when DownloadAndLaunch is called again -- see Updater::DownloadAndLaunch).
spacecal::updates::UpdateChecker s_updateChecker;
spacecal::updates::Updater s_updater;
bool s_updateInitialKicked = false;
bool s_updateBannerDismissed = false;
double s_updateBannerHiddenUntil = 0.0;

void BuildMainWindow(bool runningInOverlay_)
{
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

	BuildMainWindowContents(runningInOverlay_);
	ImGui::End();
}

void CCal_DrawTab()
{
	BuildMainWindowContents(false);
}

static void BuildMainWindowContents(bool runningInOverlay_)
{
	runningInOverlay = runningInOverlay_;
	bool continuousCalibration = CalCtx.state == CalibrationState::Continuous || CalCtx.state == CalibrationState::ContinuousStandby;

	ImGui::PushStyleColor(ImGuiCol_PlotHistogram, ImGui::GetStyleColorVec4(ImGuiCol_Button));

	// Drawn before everything else so it's always visible; the banner is a
	// no-op until the first GitHub check completes (a few seconds after
	// startup) and an update is actually available.
	spacecal::ui::DrawUpdateBanner();

	// "Waiting for SteamVR" banner -- visible whenever the program is up
	// without a connected VR stack (e.g. user launched us before starting
	// SteamVR). Disappears the moment the connection lands. Renders after
	// the update banner because update checks work without VR; the VR
	// banner is the more transient state.
	spacecal::ui::DrawVRWaitingBanner();

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
				if (ImGui::BeginTabItem("Logs")) {
					CCal_DrawLogsPanel();
					ImGui::EndTabItem();
				}
				ImGui::EndTabBar();
			}
		}
	}

	spacecal::ui::ShowVersionLine();

	ImGui::PopStyleColor();
}

// DrawStatusDot, ShowVersionLine, GetModeStatus moved to UserInterfaceFooter.cpp

// ShowVersionLine, GetModeStatus bodies moved to UserInterfaceFooter.cpp

// FormatBytes, DrawVRWaitingBanner, DrawUpdateBanner moved to UserInterfaceBanners.cpp

void BuildContinuousCalDisplay() {
	if (!s_inUmbrella) {
		ImGui::SetNextWindowPos(ImVec2(0, 0));
		ImGui::SetNextWindowSize(ImGui::GetWindowSize());
		ImGui::SetNextWindowBgAlpha(1);
		if (!ImGui::Begin("Continuous Calibration", nullptr,
			bareWindowFlags & ~ImGuiWindowFlags_NoTitleBar
		)) {
			ImGui::End();
			return;
		}
	}

	ImVec2 contentRegion;
	contentRegion.x = ImGui::GetWindowContentRegionWidth();
	contentRegion.y = ImGui::GetWindowHeight() - ImGui::GetFrameHeightWithSpacing() * 2.1f;

	if (!ImGui::BeginChild("CCalDisplayFrame", contentRegion, ImGuiChildFlags_None)) {
		ImGui::EndChild();
		if (!s_inUmbrella) ImGui::End();
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

	spacecal::ui::ShowVersionLine();

	if (!s_inUmbrella) ImGui::End();
}

// ScaledDragFloat, AddResetContextMenu, CCal_DrawSettings moved to UserInterfaceTabsAdvanced.cpp


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

// CCal_DrawPredictionSuppression: relocated to the Smoothing overlay's
// SmoothingPrediction sub-tab as part of the Protocol v12 migration on
// 2026-05-11. Per-tracker pose-prediction smoothness is now owned by the
// Smoothing plugin and pushed via RequestSetDevicePrediction; SC no longer
// holds trackerSmoothness state or sends those fields inside
// RequestSetDeviceTransform.



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
	spacecal::ui::DrawChaperoneLoadFailedBanner();
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
			// Phase banner. The two-phase one-shot flow (Rotation -> Translation)
			// is invisible to the user otherwise; surfacing the active phase and
			// what motion to do removes the "I waved but the bar didn't move"
			// confusion that the old combined-gate flow created.
			if (CalCtx.state == CalibrationState::Rotation) {
				ImGui::TextDisabled("Phase 1 of 2: Rotation");
				ImGui::TextWrapped("Rotate the tracker through different orientations (>= 90 deg between some pair).");
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
		// Guard: TriggerHapticPulse with an invalid device index is undefined
		// behaviour (driver crash or silent no-op depending on the runtime).
		// Skip the entire loop if either ID hasn't been assigned yet.
		if (CalCtx.targetID != vr::k_unTrackedDeviceIndexInvalid
			&& CalCtx.referenceID != vr::k_unTrackedDeviceIndexInvalid)
		{
			for (unsigned i = 0; i < 100; ++i)
			{
				vr::VRSystem()->TriggerHapticPulse(CalCtx.targetID, 0, 2000);
				vr::VRSystem()->TriggerHapticPulse(CalCtx.referenceID, 0, 2000);
				std::this_thread::sleep_for(std::chrono::milliseconds(5));
			}
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

// SendFingerSmoothingConfig + CCal_DrawFingerSmoothing relocated to the
// Smoothing overlay's SmoothingFingers sub-tab on 2026-05-11 (Protocol v12
// migration). The Smoothing plugin owns the on-disk config and the
// RequestSetFingerSmoothing IPC send now; SC no longer carries finger state
// in CalibrationContext.