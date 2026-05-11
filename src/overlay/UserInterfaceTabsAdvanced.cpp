#include "stdafx.h"
#include "Calibration.h"
#include "Configuration.h"
#include "CalibrationMetrics.h"
#include "VRState.h"
#include "Wizard.h"

#include <string>
#include <imgui/imgui.h>
#include "imgui_extensions.h"

// Forward decl for DrawVectorElement defined in UserInterface.cpp
void DrawVectorElement(const std::string id, const char* text, double* value, int defaultValue = 0, const char* defaultValueStr = " 0 ");

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

	// Long-stall counter: HMD stalled for >=30 ticks (~1.5 s). Previously the
	// calibration tick purged the sample buffer at this point; reverted
	// 2026-05-04 because the purge + warm-start re-anchor caused cumulative
	// drift on every HMD off/on cycle. The counter remains as a diagnostic
	// -- frequent long-stalls still indicate a tracking-environment problem.
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
		                  "(SteamVR hiccup, tracking loss, headset taken off). Diagnostic counter only --\n"
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

			// Legacy translation solve. The default path is the direct O(N)
			// latent-offset solve; flipping this on reverts to the prior
			// pairwise O(N^2) IRLS as a safety hatch.
			ImGui::Checkbox("Legacy translation solve (pairwise)", &CalCtx.useLegacyMath);
			if (ImGui::IsItemHovered(0)) {
				ImGui::SetTooltip("Reverts the translation solve to the pre-revamp pairwise O(N^2) IRLS\n"
					"path. The default direct O(N) latent-offset solver jointly estimates the\n"
					"calibration translation and the reference-to-target offset, then runs\n"
					"per-sample Cauchy IRLS; it is faster and statistically cleaner. Flip this\n"
					"on only if a real session shows a regression vs the prior release -- and\n"
					"please file a session log so the direct path can be fixed.\n\n"
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
			static bool s_prevLegacy     = CalCtx.useLegacyMath;
			static bool s_prevKalman     = CalCtx.useBlendFilter;
			static bool s_prevRestYaw    = CalCtx.restLockedYawEnabled;
			static bool s_prevPredRecov  = CalCtx.predictiveRecoveryEnabled;
			static bool s_prevChiSq      = CalCtx.reanchorChiSquareEnabled;
			logToggleFlip("latency_auto_detect",       s_prevAutoDetect, CalCtx.latencyAutoDetect);
			logToggleFlip("latency_use_gcc_phat",      s_prevGccPhat,    CalCtx.useGccPhatLatency);
			logToggleFlip("geometry_shift_use_cusum",  s_prevCusum,      CalCtx.useCusumGeometryShift);
			logToggleFlip("irls_velocity_aware",       s_prevVelAware,   CalCtx.useVelocityAwareWeighting);
			logToggleFlip("irls_use_tukey",            s_prevTukey,      CalCtx.useTukeyBiweight);
			logToggleFlip("translation_use_legacy",    s_prevLegacy,     CalCtx.useLegacyMath);
			logToggleFlip("blend_use_kalman",          s_prevKalman,     CalCtx.useBlendFilter);
			logToggleFlip("rest_locked_yaw",           s_prevRestYaw,    CalCtx.restLockedYawEnabled);
			logToggleFlip("predictive_recovery",       s_prevPredRecov,  CalCtx.predictiveRecoveryEnabled);
			logToggleFlip("reanchor_chi_square",       s_prevChiSq,      CalCtx.reanchorChiSquareEnabled);

			ImGui::EndGroupPanel();
		}

		// Tracker offset / Playspace scale stay in advanced -- these are rarely
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
