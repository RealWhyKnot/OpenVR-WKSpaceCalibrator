#include "stdafx.h"
#include "Calibration.h"
#include "Configuration.h"
#include "VRState.h"
#include "CalibrationMetrics.h"

#include <string>
#include <openvr.h>
#include <imgui/imgui.h>
#include "imgui_extensions.h"

static inline const char* GetPrettyTrackingSystemName(const std::string& value) {
	// To comply with SteamVR branding guidelines (page 29), we rename devices under lighthouse tracking to SteamVR Tracking.
	if (value == "lighthouse" || value == "aapvr") {
		return "SteamVR Tracking";
	}
	return value.c_str();
}

// Mirror of the "Reference HMD not detected" banner from BuildMenu, rendered
// inside the continuous-cal Status tab so the user sees it even when they jump
// straight into continuous mode. Returns true if the banner was drawn.
static bool DrawProfileMismatchBanner() {
	if (!CalCtx.validProfile || CalCtx.enabled) return false;
	const char* refSystem = GetPrettyTrackingSystemName(CalCtx.referenceTrackingSystem);
	// The "actual" tracking system is whatever the current HMD reports -- fall
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
		// Same trigger BuildMenu uses for "Start Calibration" -- kicks the state
		// machine into Begin via StartCalibration. The popup isn't relevant here
		// since the user is already inside the continuous-cal window.
		StartCalibration();
	}
	ImGui::Separator();
	return true;
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
	// from the user via that same broken tracking -- auto-detect + auto-fix
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
		// button. Same underlying call -- pushing a random offset forces the
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
