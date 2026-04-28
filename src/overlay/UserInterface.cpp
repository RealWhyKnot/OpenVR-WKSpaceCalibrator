#include "stdafx.h"
#include "UserInterface.h"
#include "Calibration.h"
#include "Configuration.h"
#include "VRState.h"
#include "CalibrationMetrics.h"
#include "IPCClient.h"
#include "Protocol.h"
#include "Version.h"

#include <thread>
#include <string>
#include <vector>
#include <algorithm>
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

static bool runningInOverlay;

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
	ImVec4 textColor;
	ImVec4 bgColor;

	if (!validProfile) {
		label = "[NO PROFILE]";
		textColor = ImVec4(0.85f, 0.85f, 0.85f, 1.0f);
		bgColor   = ImVec4(0.30f, 0.30f, 0.30f, 1.0f);
	} else if (state == CalibrationState::ContinuousStandby) {
		label = "[STANDBY — waiting for tracking]";
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
			textColor = ImVec4(0.10f, 0.10f, 0.10f, 1.0f);
			bgColor   = ImVec4(0.95f, 0.70f, 0.20f, 1.0f);
		} else if (recentlyUpdated) {
			label = "[LIVE — updating]";
			textColor = ImVec4(0.95f, 0.95f, 1.00f, 1.0f);
			bgColor   = ImVec4(0.20f, 0.50f, 0.85f, 1.0f);
		} else {
			label = "[LIVE]";
			textColor = ImVec4(0.95f, 0.95f, 1.00f, 1.0f);
			bgColor   = ImVec4(0.25f, 0.45f, 0.75f, 1.0f);
		}
	} else if (enabled && state == CalibrationState::None) {
		label = "[FIXED OFFSET ACTIVE]";
		textColor = ImVec4(0.10f, 0.20f, 0.10f, 1.0f);
		bgColor   = ImVec4(0.45f, 0.80f, 0.45f, 1.0f);
	} else {
		// validProfile but not enabled and not in a continuous state — treat as
		// idle/no-op rather than hide the pill entirely so the user always has
		// a status they can point to in a bug report.
		label = "[IDLE]";
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
	// Reserve layout space.
	ImGui::Dummy(ImVec2(textSize.x + padding.x * 2.0f, textSize.y + padding.y * 2.0f));
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

	// Persistent mode pill above the tab bar — surfaces the high-level state
	// (LIVE updating, searching, standby, fixed-offset) at a glance regardless
	// of which tab the user is looking at.
	DrawModePill();
	ImGui::Spacing();

	if (ImGui::BeginTabBar("CCalTabs", 0)) {
		if (ImGui::BeginTabItem("Status")) {
			CCal_BasicInfo();
			ImGui::EndTabItem();
		}

		if (ImGui::BeginTabItem("More Graphs")) {
			ShowCalibrationDebug(2, 3);
			ImGui::EndTabItem();
		}
		
		if (ImGui::BeginTabItem("Settings")) {
			CCal_DrawSettings();
			ImGui::EndTabItem();
		}

		if (ImGui::BeginTabItem("Prediction")) {
			CCal_DrawPredictionSuppression();
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

	ImGui::BeginGroupPanel("Tip", panel_size);
	ImGui::Text("Hover over settings to learn more about them! Right-click any slider to reset it to default.");
	ImGui::EndGroupPanel();

	// === COMMON SETTINGS ====================================================
	// Settings the typical user actually wants to touch. Kept always visible.
	{
		ImGui::BeginGroupPanel("Common settings", panel_size);

		// Jitter threshold
		ImGui::Text("Jitter threshold");
		ImGui::SameLine();
		ImGui::PushID("jitter_threshold");
		ImGui::SliderFloat("##jitter_threshold_slider", &CalCtx.jitterThreshold, 0.1f, 10.0f, "%1.1f", 0);
		if (ImGui::IsItemHovered(0)) {
			ImGui::SetTooltip("Controls how much jitter will be allowed for calibration.\n"
				"Higher values allow worse tracking to calibrate, but may result in poorer tracking.");
		}
		AddResetContextMenu("jitter_threshold_ctx", [] { CalCtx.jitterThreshold = 3.0f; });
		ImGui::PopID();

		// Recalibration threshold (continuous calibration)
		ImGui::Text("Recalibration threshold");
		ImGui::SameLine();
		ImGui::PushID("recalibration_threshold");
		ImGui::SliderFloat("##recalibration_threshold_slider", &CalCtx.continuousCalibrationThreshold, 1.01f, 10.0f, "%1.1f", 0);
		if (ImGui::IsItemHovered(0)) {
			ImGui::SetTooltip("Controls how good the calibration must be before realigning the trackers.\n"
				"Higher values cause calibration to happen less often, and may be useful for systems with lots of tracking drift.");
		}
		AddResetContextMenu("recalibration_threshold_ctx", [] { CalCtx.continuousCalibrationThreshold = 1.5f; });
		ImGui::PopID();

		// Lock relative position toggle.
		if (ImGui::Checkbox("Lock relative position", &CalCtx.lockRelativePosition)) {
			SaveProfile(CalCtx);
		}
		if (ImGui::IsItemHovered(0)) {
			ImGui::SetTooltip("Lock the calibrated relative pose between reference and target so it doesn't get re-solved during continuous calibration.");
		}
		ImGui::PushStyleColor(ImGuiCol_Text, ImGui::GetStyleColorVec4(ImGuiCol_TextDisabled));
		ImGui::TextWrapped("When locked, static-recalibration settings have no effect.");
		ImGui::PopStyleColor();

		// Require trigger press toggle.
		if (ImGui::Checkbox("Require trigger press to apply", &CalCtx.requireTriggerPressToApply)) {
			SaveProfile(CalCtx);
		}
		if (ImGui::IsItemHovered(0)) {
			ImGui::SetTooltip("If on, only apply the calibrated offset while a controller trigger is held.\n"
				"Useful for verifying the result before committing.");
		}

		ImGui::EndGroupPanel();
	}

	// === ADVANCED SETTINGS ==================================================
	// Behind a CollapsingHeader so they're out of the way for casual use but
	// still reachable for power users / bug reports.
	if (ImGui::CollapsingHeader("Advanced settings", CalCtx.showAdvancedSettings ? ImGuiTreeNodeFlags_DefaultOpen : 0)) {
		// Persist the open/closed state for the rest of this session. Note: the
		// tree-node header doesn't itself expose its open state, so we infer it
		// from "we're inside the if-true branch" — flipping the flag here means
		// next time CCal_DrawSettings runs we'll DefaultOpen on the way in.
		CalCtx.showAdvancedSettings = true;

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

				ImGui::Columns(3, nullptr, false);
				if (ImGui::RadioButton(" Fast          ", speed == CalibrationContext::FAST)) {
					CalCtx.calibrationSpeed = CalibrationContext::FAST;
				}
				ImGui::NextColumn();
				if (ImGui::RadioButton(" Slow          ", speed == CalibrationContext::SLOW)) {
					CalCtx.calibrationSpeed = CalibrationContext::SLOW;
				}
				ImGui::NextColumn();
				if (ImGui::RadioButton(" Very Slow     ", speed == CalibrationContext::VERY_SLOW)) {
					CalCtx.calibrationSpeed = CalibrationContext::VERY_SLOW;
				}
				ImGui::Columns(1);

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
			DrawVectorElement("cc_playspace_scale", "PLayspace Scale", &CalCtx.calibratedScale, 1, " 1 ");
			ImGui::EndGroupPanel();
		}
	} else {
		// Header is collapsed — record that so the next pass starts collapsed.
		CalCtx.showAdvancedSettings = false;
	}

	ImGui::NewLine();
	ImGui::Indent();
	if (ImGui::Button("Reset settings")) {
		CalCtx.ResetConfig();
	}
	ImGui::Unindent();
	ImGui::NewLine();

	// Section: Contributors credits
	{
		ImGui::BeginGroupPanel("Credits", panel_size);

		ImGui::TextDisabled("tach");
		ImGui::TextDisabled("pushrax");
		ImGui::TextDisabled("bd_");
		ImGui::TextDisabled("ArcticFox");
		ImGui::TextDisabled("hekky");
		ImGui::TextDisabled("pimaker");

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

	// External-tool detection banner. Shown only when something matched the
	// process scan AND auto-suppress is on (or the user has explicitly disabled
	// auto-suppress; we show a different message there).
	if (ctx.externalSmoothingDetected) {
		const char* tool = ctx.externalSmoothingToolName.empty() ? "an external smoothing tool" : ctx.externalSmoothingToolName.c_str();
		if (ctx.autoSuppressOnExternalTool) {
			ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 0.85f, 0.4f, 1.0f));
			ImGui::TextWrapped(
				"%s is running. Auto-applying built-in prediction suppression to the calibration "
				"reference and target trackers so the math stays clean. You can disable %s — this "
				"fork has the same fix natively (see the per-device list below).",
				tool, tool);
			ImGui::PopStyleColor();
		} else {
			ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 0.55f, 0.55f, 1.0f));
			ImGui::TextWrapped(
				"%s is running but auto-suppress is off. Calibration math may be disturbed by its "
				"velocity scaling. Either re-enable auto-suppress, manually pick devices in the list "
				"below, or stop %s.",
				tool, tool);
			ImGui::PopStyleColor();
		}
		ImGui::Separator();
	}

	ImGui::TextWrapped(
		"Native prediction suppression replaces external tools like OVR-SmoothTracking. When a "
		"device is enabled below, the SteamVR driver zeroes its velocity/acceleration on every pose "
		"update — defeating SteamVR's pose extrapolation and any external smoothing tool that "
		"scales the same fields. Calibration math then sees clean (un-extrapolated) pose data.");
	ImGui::Spacing();

	bool autoSup = ctx.autoSuppressOnExternalTool;
	if (ImGui::Checkbox("Auto-apply on calibration trackers when an external smoothing tool is detected", &autoSup)) {
		ctx.autoSuppressOnExternalTool = autoSup;
		SaveProfile(ctx);
	}
	ImGui::Spacing();
	ImGui::Separator();
	ImGui::TextDisabled("Per-device suppression");
	ImGui::Spacing();

	// Enumerate currently-known devices. Show a checkbox per device. The
	// suppression set is keyed by serial — that survives ID reassignment, so a
	// tracker that disconnects and reconnects keeps its suppression setting.
	auto vrSystem = vr::VRSystem();
	if (!vrSystem) {
		ImGui::TextDisabled("(VR system not available)");
		return;
	}

	bool anyShown = false;
	char buffer[vr::k_unMaxPropertyStringSize];
	for (uint32_t id = 0; id < vr::k_unMaxTrackedDeviceCount; ++id) {
		auto deviceClass = vrSystem->GetTrackedDeviceClass(id);
		if (deviceClass == vr::TrackedDeviceClass_Invalid) continue;
		// HMD shouldn't be suppressed (would degrade reprojection); skip.
		if (deviceClass == vr::TrackedDeviceClass_HMD) continue;

		vr::ETrackedPropertyError err = vr::TrackedProp_Success;
		vrSystem->GetStringTrackedDeviceProperty(id, vr::Prop_SerialNumber_String, buffer, sizeof buffer, &err);
		if (err != vr::TrackedProp_Success || buffer[0] == 0) continue;
		std::string serial = buffer;

		vrSystem->GetStringTrackedDeviceProperty(id, vr::Prop_RenderModelName_String, buffer, sizeof buffer, &err);
		std::string model = (err == vr::TrackedProp_Success) ? buffer : "";

		vrSystem->GetStringTrackedDeviceProperty(id, vr::Prop_TrackingSystemName_String, buffer, sizeof buffer, &err);
		std::string sys = (err == vr::TrackedProp_Success) ? GetPrettyTrackingSystemName(buffer) : "";

		bool enabled = ctx.suppressedSerials.find(serial) != ctx.suppressedSerials.end();
		std::string label = "##suppress_" + serial;
		if (ImGui::Checkbox(label.c_str(), &enabled)) {
			if (enabled) ctx.suppressedSerials.insert(serial);
			else ctx.suppressedSerials.erase(serial);
			SaveProfile(ctx);
		}
		ImGui::SameLine();
		ImGui::Text("%s  [%s]  serial: %s", model.c_str(), sys.c_str(), serial.c_str());
		anyShown = true;
	}
	if (!anyShown) {
		ImGui::TextDisabled("(No tracked devices found.)");
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

	// Recovery affordances. Two columns of user-facing buttons (Cancel +
	// Restart sampling), plus an inline Pause toggle and an optional
	// Mark-logs column when debug logs are on.
	const int columns = Metrics::enableLogs ? 4 : 3;
	if (ImGui::BeginTable("##CCal_Cancel", columns, 0, ImVec2(width * scale, ImGui::GetTextLineHeight() * 2))) {
		ImGui::TableNextRow();
		ImGui::TableSetColumnIndex(0);
		if (ImGui::Button("Cancel Continuous Calibration", ImVec2(-FLT_MIN, 0.0f))) {
			EndContinuousCalibration();
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
			ImGui::SetTooltip("Discards the current incremental estimate and forces continuous calibration to recollect samples from scratch.");
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

		if (Metrics::enableLogs) {
			ImGui::TableSetColumnIndex(3);
			if (ImGui::Button("Debug: Mark logs", ImVec2(-FLT_MIN, 0.0f))) {
				Metrics::WriteLogAnnotation("MARK LOGS");
			}
		}

		ImGui::EndTable();
	}

	ImGui::Checkbox("Hide tracker", &CalCtx.quashTargetInContinuous);
	ImGui::SameLine();
	ImGui::Checkbox("Static recalibration", &CalCtx.enableStaticRecalibration);
	ImGui::SameLine();
	ImGui::Checkbox("Enable debug logs", &Metrics::enableLogs);
	ImGui::SameLine();
	ImGui::Checkbox("Lock relative transform", &CalCtx.lockRelativePosition);
	ImGui::SameLine();
	ImGui::Checkbox("Require triggers", &CalCtx.requireTriggerPressToApply);
	ImGui::Checkbox("Ignore outliers", &CalCtx.ignoreOutliers);

	// Status field...

	ImGui::PushStyleColor(ImGuiCol_FrameBg, ImVec4(0, 0, 0, 1));

	for (const auto& msg : CalCtx.messages) {
		if (msg.type == CalibrationContext::Message::String) {
			ImGui::TextWrapped("> %s", msg.str.c_str());
		}
	}

	ImGui::PopStyleColor();

	ShowCalibrationDebug(1, 3);
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

		ImGui::Columns(4, nullptr, false);
		ImGui::Text("Calibration Speed");

		ImGui::NextColumn();
		if (ImGui::RadioButton(" Fast          ", speed == CalibrationContext::FAST))
			CalCtx.calibrationSpeed = CalibrationContext::FAST;

		ImGui::NextColumn();
		if (ImGui::RadioButton(" Slow          ", speed == CalibrationContext::SLOW))
			CalCtx.calibrationSpeed = CalibrationContext::SLOW;

		ImGui::NextColumn();
		if (ImGui::RadioButton(" Very Slow     ", speed == CalibrationContext::VERY_SLOW))
			CalCtx.calibrationSpeed = CalibrationContext::VERY_SLOW;

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

