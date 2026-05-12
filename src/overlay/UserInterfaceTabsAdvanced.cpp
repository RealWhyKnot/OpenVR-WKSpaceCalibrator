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
	// Hide tracker + Ignore outliers also live in the one-shot Settings tab
	// (UserInterface.cpp). When state == None both tabs are visible at once,
	// so duplicating them here would render the same checkbox twice. Gate on
	// continuous mode: in continuous, the Settings tab is hidden and Advanced
	// is the user's only access point; in non-continuous, the Settings tab
	// owns these.
	const bool kInContinuous = (CalCtx.state == CalibrationState::Continuous);
	if (kInContinuous) {
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
	}

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

			// Calibration Speed radio. Also rendered in the one-shot Settings
			// tab; gate on continuous mode here to avoid the same radio appearing
			// twice when state == None (both tabs visible at once).
			if (kInContinuous) {
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
			// buffer that only fills during continuous calibration; in one-shot
			// mode the estimator never fires. This is a persisted preference,
			// so it's always interactive -- the tooltip explains when the path
			// actually runs.
			ImGui::Checkbox("Auto-detect target latency", &CalCtx.latencyAutoDetect);
			if (ImGui::IsItemHovered(0)) {
				ImGui::SetTooltip("Uses cross-correlation of tracker velocities to estimate the inter-system\n"
					"latency once per second when both devices are moving. Overrides the manual\n"
					"offset above when on.\n\n"
					"Active in: continuous calibration only. In one-shot mode the rolling sample\n"
					"buffer that this estimator reads doesn't fill, so the value freezes; the\n"
					"manual offset slider above still applies.");
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
		// ---------------------------------------------------------------
		// Panel 1: Legacy (pre-fork upstream math).
		//
		// One master switch for pre-fork upstream behavior. This is broader
		// than the current-math kill switches below: it selects the bare
		// upstream pairwise BDCSVD translation path and bypasses fork-only
		// helpers at their call sites.
		// ---------------------------------------------------------------
		{
			ImGui::BeginGroupPanel("Legacy (pre-fork upstream math)", panel_size);
			ImGui::TextWrapped("Use this only as a broad compatibility mode. When on, the translation "
				"solve uses the pre-fork upstream bare pairwise BDCSVD path and fork-only helpers "
				"such as latency correction, velocity-aware weighting, and rest-locked yaw are bypassed.");
			ImGui::Spacing();

			ImGui::Checkbox("Use pre-fork upstream math", &CalCtx.useUpstreamMath);
			if (ImGui::IsItemHovered(0)) {
				ImGui::SetTooltip("Master compatibility switch. ON selects the pre-fork upstream translation\n"
					"solve and bypasses fork-only math helpers without changing their saved settings.\n"
					"Turn it off to return to the current fork math stack.");
			}
			ImGui::EndGroupPanel();
		}

		ImGui::Spacing();

		// ---------------------------------------------------------------
		// Panel 2: Current math (disable individual features).
		//
		// Per-feature kill switches for the current fork math stack. The
		// labels are feature-disable wording; the stored bool names keep
		// existing profile semantics.
		// ---------------------------------------------------------------
		{
			ImGui::BeginGroupPanel("Current math (disable individual features)", panel_size);
			ImGui::TextWrapped("These switches disable individual pieces of the current fork math stack. "
				"The pre-fork upstream switch above takes precedence while it is on.");
			ImGui::Spacing();

			// Legacy translation solve. The default path is the direct O(N)
			// latent-offset solve; flipping this on reverts to the prior
			// pairwise O(N^2) IRLS as a safety hatch.
			ImGui::Checkbox("Disable direct translation solve (use pairwise IRLS)", &CalCtx.useLegacyMath);
			if (ImGui::IsItemHovered(0)) {
				ImGui::SetTooltip("Reverts the translation solve to the prior pairwise O(N^2) IRLS path.\n"
					"The default direct O(N) latent-offset solver jointly estimates the calibration\n"
					"translation and the reference-to-target offset, then runs per-sample Cauchy IRLS;\n"
					"it is faster and statistically cleaner. Flip this on only if a real session shows\n"
					"a regression vs the prior release -- and please file a session log so the direct\n"
					"path can be fixed.\n\n"
					"Active in: both one-shot and continuous calibration.");
			}

			// Legacy time-domain latency estimator. Inverts useGccPhatLatency:
			// checked = revert to time-domain CC. Always interactive; this is
			// a persisted preference, the tooltip explains when it takes effect.
			bool legacyTimeDomainLatency = !CalCtx.useGccPhatLatency;
			if (ImGui::Checkbox("Disable GCC-PHAT latency estimator (use time-domain CC)", &legacyTimeDomainLatency)) {
				CalCtx.useGccPhatLatency = !legacyTimeDomainLatency;
			}
			if (ImGui::IsItemHovered(0)) {
				ImGui::SetTooltip("Reverts the latency estimator from the default whitened-spectrum\n"
					"cross-correlator (GCC-PHAT, Knapp-Carter 1976) to the prior time-domain CC.\n"
					"GCC-PHAT is drop-in better across the latency-relevant range, especially when\n"
					"the two tracking systems' velocity signals have different frequency content.\n"
					"Flip this on if your auto-detected offset reads as jumpy.\n\n"
					"Active in: continuous calibration with auto-detect target latency on.");
			}

			// Legacy uniform IRLS weighting. Inverts useVelocityAwareWeighting:
			// checked = drop the velocity scaling of the per-pair threshold.
			bool legacyUniformIRLS = !CalCtx.useVelocityAwareWeighting;
			if (ImGui::Checkbox("Disable velocity-aware IRLS weighting", &legacyUniformIRLS)) {
				CalCtx.useVelocityAwareWeighting = !legacyUniformIRLS;
			}
			if (ImGui::IsItemHovered(0)) {
				ImGui::SetTooltip("Reverts the IRLS translation solve to the prior uniform per-pair\n"
					"threshold. The default velocity-aware path scales the threshold DOWN with motion\n"
					"magnitude so fast-motion glitches get a sharper cutoff while stationary\n"
					"high-residual rows stay informative. Flip this on if you suspect the velocity\n"
					"scaling is biasing the fit on your setup.\n\n"
					"Active in: both one-shot and continuous calibration.");
			}

			// Disable rest-locked yaw drift correction. Inverts
			// restLockedYawEnabled: checked = revert to no rest-yaw correction.
			// Always interactive; tooltip explains when the path runs.
			bool disableRestLockedYaw = !CalCtx.restLockedYawEnabled;
			if (ImGui::Checkbox("Disable rest-locked yaw drift correction", &disableRestLockedYaw)) {
				CalCtx.restLockedYawEnabled = !disableRestLockedYaw;
			}
			if (ImGui::IsItemHovered(0)) {
				ImGui::SetTooltip("Disables the rest-locked yaw drift correction. When a tracker stays\n"
					"still for 1 s, the default path locks its orientation as an absolute reference\n"
					"and applies bounded-rate yaw nudges on subsequent at-rest ticks. Flip this on\n"
					"to remove the correction entirely and rely on continuous-cal alone.\n\n"
					"Active in: continuous calibration OFF (one-shot, idle, editing, standby).");
			}
			ImGui::EndGroupPanel();
		}

		ImGui::Spacing();

		// ---------------------------------------------------------------
		// Panel 3: Experimental / research (opt-in).
		//
		// Fork-added code paths that have not graduated. Each has a
		// documented risk that kept it from default-on. Off by default.
		// ---------------------------------------------------------------
		{
			ImGui::BeginGroupPanel("Experimental / research (opt-in)", panel_size);
			ImGui::TextWrapped("Not-yet-validated research code paths. Off by default; opt in only if "
				"you want to help validate one. Documented risks per tooltip; if tracking regresses "
				"after you flip one, flip it back.");
			ImGui::Spacing();

			// CUSUM geometry-shift detector (Page 1954) replaces the 5x-rolling-
			// median rule with a cumulative-sum statistical test. Same recovery
			// action; tunable false-alarm rate via standard ARL tables.
			// Always interactive; tooltip explains when the path runs.
			ImGui::Checkbox("CUSUM geometry-shift detector", &CalCtx.useCusumGeometryShift);
			if (ImGui::IsItemHovered(0)) {
				ImGui::SetTooltip("Statistical change-point test (CUSUM, Page 1954) instead of the\n"
					"default 5x-rolling-median rule. Same recovery action when fired (clear cal,\n"
					"demote to standby). Documented ARL_0 ~ 10^4 ticks (~3 min at 60 Hz at noise-\n"
					"floor convergence) means roughly one false alarm per quiet session, which is\n"
					"why it has not graduated to default-on.\n\n"
					"Active in: continuous calibration only.");
			}

			// Tukey biweight + Qn-scale alternative robust kernel. Same
			// IRLS path as velocity-aware weighting: applies in both modes.
			ImGui::Checkbox("Tukey biweight robust kernel", &CalCtx.useTukeyBiweight);
			if (ImGui::IsItemHovered(0)) {
				ImGui::SetTooltip("Replaces the default IRLS Cauchy + MAD with Tukey biweight + Qn-scale\n"
					"(Rousseeuw-Croux 1993). Tukey is redescending: residuals beyond the threshold get\n"
					"exactly zero weight, which can swallow real geometry-shift evidence; that risk\n"
					"is why it has not graduated. Cauchy + MAD has been adequate on observed data.\n\n"
					"Active in: both one-shot and continuous calibration.");
			}

			// Kalman-filter blend at publish. Replaces the EMA in
			// ComputeIncremental; ComputeOneshot does not use the EMA path,
			// so this toggle is a no-op in one-shot mode. Always interactive;
			// tooltip explains when the path runs.
			ImGui::Checkbox("Kalman-filter blend at publish", &CalCtx.useBlendFilter);
			if (ImGui::IsItemHovered(0)) {
				ImGui::SetTooltip("Replaces the single-step EMA (alpha=0.3) at the publish point with a\n"
					"4-state Kalman filter on (yaw, tx, ty, tz). Off by default: measurement noise\n"
					"is currently fixed and has not been adapted to the new direct translation\n"
					"solver's covariance output. Until that plumbing lands, the EMA is the safer\n"
					"default.\n\n"
					"Active in: continuous calibration only.");
			}

			// Predictive recovery pre-correction. Buffer fills on the 30 cm
			// relocalization fire (continuous mode); per-tick apply runs in
			// any state. Mostly continuous-relevant.
			ImGui::Checkbox("Predictive recovery pre-correction", &CalCtx.predictiveRecoveryEnabled);
			if (ImGui::IsItemHovered(0)) {
				ImGui::SetTooltip("Each Quest re-anchor event (the 30 cm HMD-jump trigger) pushes its direction\n"
					"and magnitude into a 6-deep rolling buffer. Once 3+ events accumulate with a consistent\n"
					"direction, apply 10 percent of the predicted next-jump per tick as a bounded-rate\n"
					"translation nudge to the active calibration. Speculative -- can inject bias before a\n"
					"real event, which is why it has not graduated.\n\n"
					"Active in: any mode. Buffer fills only when the 30 cm detector fires (continuous\n"
					"mode); the per-tick predictive nudge applies whenever the buffer has enough events.");
			}

			// Chi-square re-anchor sub-detector. Pose-stream-only; works in
			// any state.
			ImGui::Checkbox("Chi-square re-anchor sub-detector", &CalCtx.reanchorChiSquareEnabled);
			if (ImGui::IsItemHovered(0)) {
				ImGui::SetTooltip("Mahalanobis distance between HMD-pose-from-rolling-velocity and observed\n"
					"HMD pose. Threshold at chi-square 6 DoF p<1e-4 (about 27.86). Needs angular-velocity\n"
					"prediction before it can trust rotation residuals during normal head turns; without\n"
					"that it can false-fire on routine motion. Detection-only: never triggers recovery\n"
					"itself.\n\n"
					"Active in: any mode.");
			}

			// Toggle-flip diagnostic. Compare each flag to its previous value
			// and emit a one-shot annotation on change. Statics are initialized
			// to the loaded-profile values on first frame so we do not log a
			// spurious flip just because the panel rendered for the first time.
			static bool s_prevAutoDetect = CalCtx.latencyAutoDetect;
			static bool s_prevUpstream   = CalCtx.useUpstreamMath;
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
			logToggleFlip("translation_use_upstream",  s_prevUpstream,   CalCtx.useUpstreamMath);
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
