#include "stdafx.h"
#include "Wizard.h"
#include "Calibration.h"
#include "Configuration.h"
#include "VRState.h"
#include "UserInterface.h"

#include <imgui/imgui.h>
#include <openvr.h>

#include <algorithm>
#include <string>
#include <vector>

// Helpers from UserInterface.cpp -- exposed via the calibration namespace for
// the wizard's own use. These already exist; we just forward-declare to avoid
// creating circular header dependencies.
extern "C++" const char* GetPrettyTrackingSystemName(const std::string& value);

void StartCalibration();
void StartContinuousCalibration();
void EndContinuousCalibration();
void SaveProfile(CalibrationContext& ctx);

namespace spacecal::wizard {

namespace {

// Wizard state machine. All state lives in this single static struct so the
// wizard can be opened, dismissed, and reopened without leaking partial state
// between sessions.
enum class Step {
	Inactive,           // wizard not running
	Welcome,            // intro screen
	NoCalibrationNeeded,// detected only one tracking system; nothing to do
	PickTarget,         // user picks the target tracker for the current system
	Calibrating,        // continuous calibration is running for the current system
	SystemDone,         // current system done; offer to do another or finish
	AllDone,            // all systems calibrated
};

struct State {
	Step step = Step::Inactive;

	// HMD tracking system (the implicit reference for every calibration).
	std::string hmdSystem;

	// Tracking systems still to calibrate. We pop entries off the front as
	// each is completed.
	std::vector<std::string> pendingSystems;

	// The currently-selected target device for the current pendingSystem.
	// Reset between systems.
	int selectedDeviceIdx = -1;

	// Snapshot of the VR state, refreshed on entering PickTarget so the user
	// sees a stable list while choosing.
	VRState vrState;

	// True after Calibrating successfully starts continuous calibration so
	// our progress watcher can detect completion.
	bool calibrationStarted = false;

	// We start out treating the first non-HMD system as the "primary"
	// (uses the singular CalCtx fields). Subsequent systems get appended
	// to additionalCalibrations. firstSystem flips to false after the
	// first finishes.
	bool firstSystem = true;
};

State& S() {
	static State s;
	return s;
}

// Pretty-print a tracking system name. Falls through to GetPrettyTrackingSystemName
// when defined; otherwise echoes the raw name.
const char* Pretty(const std::string& sys) {
	return GetPrettyTrackingSystemName(sys);
}

// Modal sizing helper: centred fixed-width box.
void OpenCentredPopup(const char* id, ImVec2 size) {
	ImVec2 vp = ImGui::GetMainViewport()->Size;
	ImGui::SetNextWindowPos(ImVec2((vp.x - size.x) * 0.5f, (vp.y - size.y) * 0.5f),
	                       ImGuiCond_Appearing);
	ImGui::SetNextWindowSize(size, ImGuiCond_Always);
	ImGui::OpenPopup(id);
}

// Step transitions.

void EnterPickTarget() {
	auto& s = S();
	s.vrState = VRState::Load();
	s.selectedDeviceIdx = -1;
	s.calibrationStarted = false;
	s.step = Step::PickTarget;
}

void EnterSystemDone() {
	S().step = Step::SystemDone;
}

void EnterAllDone() {
	S().step = Step::AllDone;
	CalCtx.wizardCompleted = true;
	SaveProfile(CalCtx);
}

void EnterNextSystem() {
	auto& s = S();
	if (s.pendingSystems.empty()) {
		EnterAllDone();
		return;
	}
	EnterPickTarget();
}

// Drawn step-by-step.

void DrawWelcome() {
	ImGui::TextWrapped(
		"Welcome to Space Calibrator!");
	ImGui::Spacing();
	ImGui::TextWrapped(
		"This wizard will set you up in a few clicks.  We'll detect what tracking "
		"systems you have, walk you through aligning each one to your headset, and "
		"keep them aligned automatically afterwards.");
	ImGui::Spacing();
	ImGui::TextWrapped(
		"You can skip this and configure things yourself in the tabs below at any time.");
	ImGui::Spacing();
	ImGui::Separator();
	ImGui::Spacing();
	if (ImGui::Button("Continue", ImVec2(140, 0))) {
		// Detect tracking systems.
		auto& s = S();
		s.vrState = VRState::Load();
		s.hmdSystem.clear();
		std::vector<std::string> nonHmd;
		// Find HMD's tracking system.
		if (auto vrSystem = vr::VRSystem()) {
			char buf[vr::k_unMaxPropertyStringSize] = {};
			vr::ETrackedPropertyError err = vr::TrackedProp_Success;
			vrSystem->GetStringTrackedDeviceProperty(
				vr::k_unTrackedDeviceIndex_Hmd,
				vr::Prop_TrackingSystemName_String, buf, sizeof buf, &err);
			if (err == vr::TrackedProp_Success) s.hmdSystem = buf;
		}
		for (const auto& sys : s.vrState.trackingSystems) {
			if (sys != s.hmdSystem) nonHmd.push_back(sys);
		}
		s.pendingSystems = nonHmd;
		s.firstSystem = true;

		if (s.pendingSystems.empty()) {
			s.step = Step::NoCalibrationNeeded;
		} else {
			EnterPickTarget();
		}
	}
	ImGui::SameLine();
	if (ImGui::Button("Skip wizard", ImVec2(140, 0))) {
		CalCtx.wizardCompleted = true;
		SaveProfile(CalCtx);
		S().step = Step::Inactive;
	}
}

void DrawNoCalibrationNeeded() {
	const char* hmd = S().hmdSystem.empty() ? "your HMD" : Pretty(S().hmdSystem);
	ImGui::TextWrapped(
		"Only one tracking system detected (%s).", hmd);
	ImGui::Spacing();
	ImGui::TextWrapped(
		"All your tracked devices share a single coordinate space already, so "
		"calibration would not help and could actually add noise.  You can close "
		"this app -- it is not needed for this setup.");
	ImGui::Spacing();
	ImGui::Spacing();
	if (ImGui::Button("Got it -- close wizard", ImVec2(220, 0))) {
		CalCtx.wizardCompleted = true;
		SaveProfile(CalCtx);
		S().step = Step::Inactive;
	}
}

void DrawPickTarget() {
	auto& s = S();
	if (s.pendingSystems.empty()) {
		EnterAllDone();
		return;
	}
	const std::string& currentSys = s.pendingSystems.front();

	ImGui::TextWrapped(
		"Pick a device from the %s system to use as the target.  We will align "
		"this device to your %s headset, and that alignment automatically applies "
		"to every other device in the same system (so you only need to pick one).",
		Pretty(currentSys), Pretty(s.hmdSystem));
	ImGui::Spacing();

	// Device list -- only devices in the currently selected system.
	std::vector<int> candidates;
	for (size_t i = 0; i < s.vrState.devices.size(); ++i) {
		const auto& d = s.vrState.devices[i];
		if (d.trackingSystem != currentSys) continue;
		if (d.deviceClass == vr::TrackedDeviceClass_HMD) continue;
		if (d.deviceClass == vr::TrackedDeviceClass_Invalid) continue;
		candidates.push_back((int)i);
	}

	if (candidates.empty()) {
		ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.95f, 0.55f, 0.45f, 1.0f));
		ImGui::TextWrapped(
			"No devices available in this system right now.  Make sure the trackers "
			"are powered on and visible to SteamVR, then refresh.");
		ImGui::PopStyleColor();
		if (ImGui::Button("Refresh##wiz_pick")) {
			s.vrState = VRState::Load();
		}
		ImGui::SameLine();
	} else {
		ImGui::BeginChild("##wiz_devices",
			ImVec2(0, ImGui::GetTextLineHeightWithSpacing() * 6.0f),
			ImGuiChildFlags_Border);
		for (int idx : candidates) {
			const auto& d = s.vrState.devices[idx];
			char label[256];
			snprintf(label, sizeof label, "%s (%s)",
				d.model.c_str(), d.serial.c_str());
			if (ImGui::Selectable(label, s.selectedDeviceIdx == idx)) {
				s.selectedDeviceIdx = idx;
			}
		}
		ImGui::EndChild();
	}

	ImGui::Spacing();
	const bool canContinue = s.selectedDeviceIdx >= 0;
	ImGui::BeginDisabled(!canContinue);
	if (ImGui::Button("Start calibration", ImVec2(180, 0))) {
		// Wire the pick into the singular CalCtx fields. For first-system, this
		// IS the primary; for additional systems we also create an additional
		// calibration entry once continuous mode is running so its sample
		// buffer accumulates fresh data.
		const auto& d = s.vrState.devices[s.selectedDeviceIdx];

		// Find HMD device record.
		int hmdIdx = -1;
		for (size_t i = 0; i < s.vrState.devices.size(); ++i) {
			if (s.vrState.devices[i].deviceClass == vr::TrackedDeviceClass_HMD) {
				hmdIdx = (int)i;
				break;
			}
		}
		if (hmdIdx < 0) {
			ImGui::EndDisabled();
			return; // no HMD -- punt; user can refresh
		}
		const auto& hmd = s.vrState.devices[hmdIdx];

		if (s.firstSystem) {
			// Set up primary.
			CalCtx.referenceTrackingSystem = hmd.trackingSystem;
			CalCtx.targetTrackingSystem    = d.trackingSystem;
			CalCtx.referenceStandby.trackingSystem = hmd.trackingSystem;
			CalCtx.referenceStandby.model = hmd.model;
			CalCtx.referenceStandby.serial = hmd.serial;
			CalCtx.targetStandby.trackingSystem = d.trackingSystem;
			CalCtx.targetStandby.model = d.model;
			CalCtx.targetStandby.serial = d.serial;
			CalCtx.referenceID = hmd.id;
			CalCtx.targetID = d.id;
			StartContinuousCalibration();
		} else {
			// Append a new entry. The continuous tick processes it in
			// parallel with the primary.
			AdditionalCalibration extra;
			extra.targetTrackingSystem = d.trackingSystem;
			extra.targetStandby.trackingSystem = d.trackingSystem;
			extra.targetStandby.model = d.model;
			extra.targetStandby.serial = d.serial;
			extra.referenceID = hmd.id;
			extra.targetID = d.id;
			extra.enabled = true;
			CalCtx.additionalCalibrations.push_back(std::move(extra));
		}
		s.calibrationStarted = true;
		s.step = Step::Calibrating;
	}
	ImGui::EndDisabled();
	ImGui::SameLine();
	if (ImGui::Button("Skip this system##wiz_pick")) {
		s.pendingSystems.erase(s.pendingSystems.begin());
		s.firstSystem = false;
		EnterNextSystem();
	}
	ImGui::SameLine();
	if (ImGui::Button("Skip wizard##wiz_pick_skip")) {
		CalCtx.wizardCompleted = true;
		SaveProfile(CalCtx);
		S().step = Step::Inactive;
	}
}

void DrawCalibrating() {
	auto& s = S();
	const std::string currentSys = s.pendingSystems.empty() ? "" : s.pendingSystems.front();

	ImGui::TextWrapped(
		"Calibrating %s.  Slowly move and rotate the chosen tracker (or walk "
		"around if it is on your body) for ~30 seconds.  The math figures out "
		"the alignment from your motion.",
		Pretty(currentSys));
	ImGui::Spacing();

	// Progress is a soft estimate -- there is no fixed end-of-calibration in
	// continuous mode. We treat "we have at least one valid result" as done.
	bool hasResult = false;
	if (s.firstSystem) {
		hasResult = CalCtx.validProfile && CalCtx.enabled;
	} else if (!CalCtx.additionalCalibrations.empty()) {
		hasResult = CalCtx.additionalCalibrations.back().valid;
	}

	if (hasResult) {
		ImGui::TextColored(ImVec4(0.45f, 0.85f, 0.45f, 1.0f),
			"Calibration accepted.");
	} else {
		ImGui::TextDisabled("Waiting for first valid calibration...");
	}

	ImGui::Spacing();
	if (ImGui::Button("This system is done", ImVec2(180, 0))) {
		s.pendingSystems.erase(s.pendingSystems.begin());
		s.firstSystem = false;
		EnterSystemDone();
	}
	ImGui::SameLine();
	if (ImGui::Button("Cancel##wiz_calib")) {
		// Don't tear down what we already started -- the user can fix it via
		// the tabs. Just exit the wizard.
		S().step = Step::Inactive;
		CalCtx.wizardCompleted = true;
		SaveProfile(CalCtx);
	}
}

void DrawSystemDone() {
	auto& s = S();
	if (s.pendingSystems.empty()) {
		EnterAllDone();
		return;
	}
	const std::string& nextSys = s.pendingSystems.front();
	ImGui::TextWrapped(
		"Done!  We still see %s waiting to be calibrated.  Want to do that next?",
		Pretty(nextSys));
	ImGui::TextWrapped("(%d system(s) still pending after this one.)",
		(int)s.pendingSystems.size() - 1);
	ImGui::Spacing();
	if (ImGui::Button("Yes, calibrate next", ImVec2(200, 0))) {
		EnterNextSystem();
	}
	ImGui::SameLine();
	if (ImGui::Button("No, finish wizard")) {
		EnterAllDone();
	}
}

void DrawAllDone() {
	ImGui::TextWrapped(
		"All set!  Continuous calibration is now running for every system you "
		"chose; it will keep them aligned automatically as long as the overlay "
		"is open.");
	ImGui::Spacing();
	ImGui::TextWrapped(
		"You can re-run this wizard from the Advanced tab if you change your "
		"hardware setup, and tweak individual settings in the Basic / Advanced "
		"tabs at any time.");
	ImGui::Spacing();
	if (ImGui::Button("Close", ImVec2(140, 0))) {
		S().step = Step::Inactive;
	}
}

} // namespace

bool IsActive() {
	return S().step != Step::Inactive;
}

void Open() {
	auto& s = S();
	s = State{}; // reset
	s.step = Step::Welcome;
}

void Draw() {
	auto& s = S();
	if (s.step == Step::Inactive) return;

	const char* popupId = "##spacecal_wizard";
	OpenCentredPopup(popupId, ImVec2(640, 380));

	if (ImGui::BeginPopupModal(popupId, nullptr,
		ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove))
	{
		// Header banner.
		ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.95f, 0.95f, 1.0f, 1.0f));
		ImGui::Text("Space Calibrator -- Setup Wizard");
		ImGui::PopStyleColor();
		ImGui::Separator();
		ImGui::Spacing();

		switch (s.step) {
		case Step::Welcome:              DrawWelcome();              break;
		case Step::NoCalibrationNeeded:  DrawNoCalibrationNeeded();  break;
		case Step::PickTarget:           DrawPickTarget();           break;
		case Step::Calibrating:          DrawCalibrating();          break;
		case Step::SystemDone:           DrawSystemDone();           break;
		case Step::AllDone:              DrawAllDone();              break;
		default: break;
		}

		// If the step transitioned to Inactive mid-frame (Skip/Close clicked),
		// close the popup so the next frame doesn't re-open it on us.
		if (s.step == Step::Inactive) {
			ImGui::CloseCurrentPopup();
		}

		ImGui::EndPopup();
	}
}

} // namespace spacecal::wizard
