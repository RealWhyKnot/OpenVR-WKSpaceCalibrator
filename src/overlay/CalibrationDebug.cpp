#include "stdafx.h"

#include <vector>

#include <implot/implot.h>
#include "Calibration.h"
#include "CalibrationCalc.h"
#include "CalibrationMetrics.h"
#include "UserInterface.h"
// ImPlotPoint (*ImPlotGetter)(int idx, void* user_data);
namespace {
	double refTime;

	template<typename F>
	ImPlotPoint VPIndexer(int idx, void* ptr) {
		auto point = (*reinterpret_cast<const F*>(ptr))(idx);
		point.x -= refTime;
		return point;
	}

	template<typename F>
	void PlotLineG(const char* name, const F& f, int points) {
		const void* vp_f = &f;

		if (points > 0) {
			ImPlot::PlotLineG(name, VPIndexer<F>, const_cast<void*>(vp_f), points);
		}
		else {
			double x = -INFINITY;
			double y = 0;
			ImPlot::PlotLine(name, &x, &y, 1);
		}
	}
	
	template<typename F, typename G>
	void PlotShadedG(const char* name, const F& data, const G& reference, int count) {
		const void* vp_data = &data;
		const void* vp_reference = &reference;

		if (count > 0) {
			ImPlot::PlotShadedG(name,
				VPIndexer<F>, const_cast<void*>(vp_data),
				VPIndexer<G>, const_cast<void*>(vp_reference),
				count
			);
		}
		else {
			double x = -INFINITY;
			double y = 0;
			ImPlot::PlotShaded(name, &x, &y, &y, 1);
		}
	}

	void PlotLineG(const char* name, const Metrics::TimeSeries<double>& ts) {
		PlotLineG(name, [&](int index) {
				const auto& p = ts[index];
				return ImPlotPoint(p.first, p.second);
			},
			ts.size()
		);
	}

	void PlotVector(const char* namePrefix, const Metrics::TimeSeries<Eigen::Vector3d>& ts) {
		std::string name(namePrefix);
		name += "X";
		PlotLineG(name.c_str(), [&](int index) {
			const auto& p = ts[index];
			return ImPlotPoint(p.first, p.second(0));
		}, ts.size());

		name.pop_back();
		name += "Y";
		PlotLineG(name.c_str(), [&](int index) {
			const auto& p = ts[index];
			return ImPlotPoint(p.first, p.second(1));
		}, ts.size());

		name.pop_back();
		name += "Z";
		PlotLineG(name.c_str(), [&](int index) {
			const auto& p = ts[index];
			return ImPlotPoint(p.first, p.second(2));
		}, ts.size());
	}

	double lastMouseX = -INFINITY;
	bool wasHovered;

	std::vector<double> calAppliedTimeBuffer, calByRelPoseTimeBuffer;

	void PrepApplyTicks() {
		calAppliedTimeBuffer.clear();
		calByRelPoseTimeBuffer.clear();

		for (auto t : Metrics::calibrationApplied.data()) {
			if (t.second) {
				calAppliedTimeBuffer.push_back(t.first - refTime);
			}
			else {
				calByRelPoseTimeBuffer.push_back(t.first - refTime);
			}
		}
	}

	void AddApplyTicks() {
		if (calAppliedTimeBuffer.empty()) {
			double x = -INFINITY;
			ImPlot::PlotInfLines("##CalibrationAppliedTime", &x, 1);
		} else {
			ImPlot::PlotInfLines("##CalibrationAppliedTime", &calAppliedTimeBuffer[0], (int)calAppliedTimeBuffer.size());
		}

		if (calByRelPoseTimeBuffer.empty()) {
			double x = -INFINITY;
			ImPlot::PlotInfLines("##CalibrationAppliedTimeByRelPose", &x, 1);
		}
		else {
			ImPlot::PlotInfLines("##CalibrationAppliedTimeByRelPose", &calByRelPoseTimeBuffer[0], (int)calByRelPoseTimeBuffer.size());
		}

		ImPlot::SetNextLineStyle(ImVec4(0.5, 0.5, 1, 1));
		ImPlot::PlotInfLines("##TagLine", &lastMouseX, 1);

		if (ImPlot::IsPlotHovered()) {
			auto mousePos = ImPlot::GetPlotMousePos();
			lastMouseX = mousePos.x;
			wasHovered = true;
		}
	}

	struct GraphInfo {
		const char* name;
		void (*callback)();
		const char* tooltip; // shown on hover for users who don't know what
		                     // the metric means.
	};

	void SetupXAxis() {
		ImPlot::SetupAxisLimits(ImAxis_X1, -Metrics::TimeSpan, 0, ImGuiCond_Always);
	}

	void G_PosOffset_RawComputed() {
		if (ImPlot::BeginPlot("##posOffsetRawComputed")) {
			ImPlot::SetupAxes(nullptr, "mm", 0, ImPlotAxisFlags_AutoFit | ImPlotAxisFlags_RangeFit);
			SetupXAxis();
			ImPlot::SetupAxisLimits(ImAxis_Y1, -200, 200, ImGuiCond_Appearing);

			AddApplyTicks();

			PlotVector("", Metrics::posOffset_rawComputed);

			ImPlot::EndPlot();
		}
	}

	void G_PosOffset_CurrentCal() {
		if (ImPlot::BeginPlot("##posOffsetCurrentCal")) {
			ImPlot::SetupAxes(nullptr, "mm", 0, ImPlotAxisFlags_AutoFit | ImPlotAxisFlags_RangeFit);
			SetupXAxis();
			ImPlot::SetupAxisLimits(ImAxis_Y1, -200, 200, ImGuiCond_Appearing);

			AddApplyTicks();

			PlotVector("", Metrics::posOffset_currentCal);

			ImPlot::EndPlot();
		}
	}

	void G_PosOffset_LastSample() {
		if (ImPlot::BeginPlot("##posOffsetLastSample")) {
			ImPlot::SetupAxes(nullptr, "mm", 0, ImPlotAxisFlags_AutoFit | ImPlotAxisFlags_RangeFit);
			SetupXAxis();
			ImPlot::SetupAxisLimits(ImAxis_Y1, -200, 200, ImGuiCond_Appearing);

			AddApplyTicks();

			PlotVector("", Metrics::posOffset_lastSample);

			ImPlot::EndPlot();
		}
	}

	void G_PosOffset_ByRelPose() {
		if (ImPlot::BeginPlot("##posOffsetByRelPose")) {
			ImPlot::SetupAxes(nullptr, "mm", 0, ImPlotAxisFlags_AutoFit | ImPlotAxisFlags_RangeFit);
			SetupXAxis();
			ImPlot::SetupAxisLimits(ImAxis_Y1, -200, 200, ImGuiCond_Appearing);

			AddApplyTicks();

			PlotVector("", Metrics::posOffset_byRelPose);

			ImPlot::EndPlot();
		}
	}

	void G_PosOffset_PosError() {
		if (ImPlot::BeginPlot("##Position error")) {
			ImPlot::SetupAxes(nullptr, "mm (RMS)");
			SetupXAxis();
			ImPlot::SetupAxisLimits(ImAxis_Y1, 0, 25, ImGuiCond_Appearing);

			AddApplyTicks();

			PlotLineG("Candidate", Metrics::error_rawComputed);
			PlotLineG("Active", Metrics::error_currentCal);
			PlotLineG("By Rel Pose", Metrics::error_byRelPose);
			PlotLineG("CC Rel Pose", Metrics::error_currentCalRelPose);
			ImPlot::EndPlot();
		}
	}

	void G_ComputationTime() {
		if (ImPlot::BeginPlot("##Computation Time", ImVec2(-1, 0), ImPlotFlags_NoLegend)) {
			ImPlot::SetupAxes(nullptr, "ms", 0, ImPlotAxisFlags_AutoFit | ImPlotAxisFlags_RangeFit);
			SetupXAxis();
			ImPlot::SetupAxisLimits(ImAxis_Y1, 0, 200, ImGuiCond_Appearing);

			AddApplyTicks();

			PlotLineG("Time", Metrics::computationTime);
			ImPlot::EndPlot();
		}
	}

	void G_JitterReference() {
		if (ImPlot::BeginPlot("##JitterReference", ImVec2(-1, 0), ImPlotFlags_NoLegend)) {
			ImPlot::SetupAxes(nullptr, "", 0, ImPlotAxisFlags_AutoFit | ImPlotAxisFlags_RangeFit);
			SetupXAxis();
			ImPlot::SetupAxisLimits(ImAxis_Y1, 0, 10, ImGuiCond_Appearing);

			AddApplyTicks();

			PlotLineG("Reference Jitter", Metrics::jitterRef);
			ImPlot::EndPlot();
		}
	}

	void G_JitterTarget() {
		if (ImPlot::BeginPlot("##JitterTarget", ImVec2(-1, 0), ImPlotFlags_NoLegend)) {
			ImPlot::SetupAxes(nullptr, "", 0, ImPlotAxisFlags_AutoFit | ImPlotAxisFlags_RangeFit);
			SetupXAxis();
			ImPlot::SetupAxisLimits(ImAxis_Y1, 0, 10, ImGuiCond_Appearing);

			AddApplyTicks();

			PlotLineG("Target Jitter", Metrics::jitterTarget);
			ImPlot::EndPlot();
		}
	}

	void G_ApplyRate() {
		if (ImPlot::BeginPlot("##ApplyRate")) {
			ImPlot::SetupAxes(nullptr, "Hz", 0, ImPlotAxisFlags_AutoFit | ImPlotAxisFlags_RangeFit);
			SetupXAxis();
			ImPlot::SetupAxisLimits(ImAxis_Y1, 0, 200, ImGuiCond_Appearing);

			AddApplyTicks();

			PlotLineG("Per-ID", Metrics::perIdApplyRate);
			PlotLineG("Fallback", Metrics::fallbackApplyRate);
			PlotLineG("Quash", Metrics::quashApplyRate);

			ImPlot::EndPlot();
		}
	}

	void G_AxisVariance() {
		static bool firstrun = true;
		static ImPlotColormap axisVarianceColormap;
		if (firstrun) {
			firstrun = false;

			auto defaultFirst = ImPlot::GetColormapColor(0);

			ImVec4 colors[] = {
				ImPlot::GetColormapColor(0),
				ImPlot::GetColormapColor(1),
				{ 1, 0, 0, 1 },
				{ 0, 1, 0, 1 },
				{ 0.5, 0.5, 0.5, 1 },
			};

			axisVarianceColormap = ImPlot::AddColormap("AxisVarianceColormap", colors, sizeof(colors) / sizeof(colors[0]));
		}

		if (ImPlot::BeginPlot("##Axis variance", ImVec2(-1, 0), ImPlotFlags_NoLegend)) {
			ImPlot::SetupAxes(nullptr, nullptr, 0, 0);
			SetupXAxis();
			ImPlot::SetupAxisLimits(ImAxis_Y1, 0, 0.003, ImGuiCond_Always);

			AddApplyTicks();

			ImPlot::PushColormap(axisVarianceColormap);
			ImPlot::PushStyleVar(ImPlotStyleVar_FillAlpha, 0.5f);
			ImPlot::SetNextLineStyle(ImVec4(1, 0, 0, 1));
			PlotShadedG("##VarianceLow",
				[&](int index) {
					auto p = Metrics::axisIndependence[index];
					p.second = std::min(p.second, CalibrationCalc::AxisVarianceThreshold);
					return ImPlotPoint(p.first, p.second);
				},
				[&](int index) {
					auto p = Metrics::axisIndependence[index];
					return ImPlotPoint(p.first, 0);
				},
				Metrics::axisIndependence.size()
			);

			ImPlot::SetNextLineStyle(ImVec4(0, 1, 0, 1));

			PlotShadedG("##VarianceHigh",
				[&](int index) {
					auto p = Metrics::axisIndependence[index];
					p.second = std::max(p.second, CalibrationCalc::AxisVarianceThreshold);
					return ImPlotPoint(p.first, p.second);
				},
				[&](int index) {
					auto p = Metrics::axisIndependence[index];
					return ImPlotPoint(p.first, CalibrationCalc::AxisVarianceThreshold);
				},
				Metrics::axisIndependence.size()
			);

			PlotLineG("Datapoint", Metrics::axisIndependence);

			ImPlot::PopStyleVar(1);
			ImPlot::PopColormap(1);

			ImPlot::EndPlot();
		}
	}

	// Mirror of CalibrationCalc.cpp's local RotationConditionMin/MaxConsecutiveRejections.
	// Kept in sync manually — used only as reference lines on the watchdog plots.
	constexpr double kRotationConditionMin = 0.05;
	constexpr double kMaxConsecutiveRejections = 50.0;

	void G_RotationConditionRatio() {
		if (ImPlot::BeginPlot("##RotationConditionRatio", ImVec2(-1, 0))) {
			ImPlot::SetupAxes(nullptr, "min/max SV");
			SetupXAxis();
			ImPlot::SetupAxisLimits(ImAxis_Y1, 0, 1, ImGuiCond_Appearing);

			AddApplyTicks();

			// Reference line at the rejection threshold.
			ImPlot::SetNextLineStyle(ImVec4(1, 0.4f, 0.4f, 1));
			double thresholdY[2] = { kRotationConditionMin, kRotationConditionMin };
			ImPlot::PlotInfLines("##RotConditionThreshold", &thresholdY[0], 1, ImPlotInfLinesFlags_Horizontal);

			PlotLineG("Rotation Condition", Metrics::rotationConditionRatio);

			ImPlot::EndPlot();
		}
	}

	void G_ConsecutiveRejections() {
		if (ImPlot::BeginPlot("##ConsecutiveRejections", ImVec2(-1, 0))) {
			ImPlot::SetupAxes(nullptr, "count");
			SetupXAxis();
			ImPlot::SetupAxisLimits(ImAxis_Y1, 0, 50, ImGuiCond_Appearing);

			AddApplyTicks();

			// Reference line at the watchdog trigger count.
			ImPlot::SetNextLineStyle(ImVec4(1, 0.4f, 0.4f, 1));
			double watchdogY = kMaxConsecutiveRejections;
			ImPlot::PlotInfLines("##WatchdogThreshold", &watchdogY, 1, ImPlotInfLinesFlags_Horizontal);

			PlotLineG("Rejections", Metrics::consecutiveRejections);

			ImPlot::EndPlot();
		}
	}

	const struct GraphInfo graphs[] = {
		{ "Position Error", G_PosOffset_PosError,
			"RMS position error of the various calibration estimates against the live samples (mm).\n"
			"'Active' is the offset currently applied to your body trackers. 'Candidate' is the latest\n"
			"value the solver computed before the acceptance gate. Lower is tighter; bumps usually\n"
			"correspond to the math finding (and gating on) a fresh estimate." },
		{ "Axis Variance", G_AxisVariance,
			"4D quaternion-PCA variance of the rotation samples in the buffer.\n"
			"Drops toward zero when rotational motion is too planar (e.g. only yaw, no pitch/roll),\n"
			"which is the math telling you it can't reliably fit a 3D rotation from your samples." },
		{ "Offset: Raw Computed", G_PosOffset_RawComputed,
			"The most recent position offset the math computed, in mm, before the acceptance gate.\n"
			"Can be noisy as the solver experiments. Compare to Current Calibration to see how often\n"
			"a candidate beats the gate." },
		{ "Offset: Current Calibration", G_PosOffset_CurrentCal,
			"The position offset (mm) currently applied to body trackers.\n"
			"This is what the driver is publishing, blended toward each accepted candidate." },
		{ "Offset: Last Sample", G_PosOffset_LastSample,
			"Per-tick offset (mm) between the reference HMD pose and the target tracker pose on\n"
			"the most recent sample. Mostly useful for sanity-checking the input stream." },
		{ "Offset: By Rel Pose", G_PosOffset_ByRelPose,
			"Position offset (mm) implied by the locked reference->target relative pose, when\n"
			"Lock relative position is on or AUTO has detected a rigid attachment.\n"
			"Stays flat for rigid setups; diverges from Current Calibration when the live solver\n"
			"is fighting the lock." },
		{ "Processing time", G_ComputationTime,
			"Per-tick math computation time (ms). Useful for performance debugging on slower\n"
			"machines or after large profile changes -- normal values are well under 5 ms." },
		{ "Reference Jitter", G_JitterReference,
			"Welford std-dev of the reference HMD's translation over the sample buffer (mm).\n"
			"High values indicate unstable tracking on the reference (e.g. lighthouse occlusion,\n"
			"wireless dropout) and feed AUTO calibration speed's bucketing." },
		{ "Target Jitter", G_JitterTarget,
			"Welford std-dev of the target tracker's translation over the sample buffer (mm).\n"
			"Same shape as Reference Jitter, for the target side. Big differences between the\n"
			"two suggest one tracking system is noisier than the other." },
		{ "Rotation Condition Ratio", G_RotationConditionRatio,
			"2D Kabsch min/max singular-value ratio. A measure of how varied the rotation samples\n"
			"are -- drops toward zero when motion is single-axis. The math gates on this; values\n"
			"under ~0.05 mean the rotation fit isn't trustworthy." },
		{ "Consecutive Rejections", G_ConsecutiveRejections,
			"Number of consecutive ticks the acceptance gate has refused a new estimate.\n"
			"Climbs when the user isn't moving enough or the samples are too noisy. Watchdog\n"
			"resets when this gets very high (~50)." },
		{ "Apply rate (Hz)", G_ApplyRate,
			"Per-tracking-system rate at which the driver applies pose transforms, in Hz.\n"
			"Should hover around 100-150 Hz on a healthy session. Drops indicate the driver\n"
			"is dedup-suppressing identical transforms (Per-ID), or falling back to the\n"
			"system-wide transform (Fallback)." }
	};

	const int N_GRAPHS = sizeof(graphs) / sizeof(graphs[0]);
}

void PushCalibrationApplyTime() {
	Metrics::calibrationApplied.Push(true);
}


void ShowCalibrationDebug(int rows, int cols) {
	static std::vector<int> curIndexes;

	//ImGui::ShowDemoWindow();
	//ImPlot::ShowDemoWindow();

	double initMouseX = lastMouseX;
	wasHovered = false;

	for (int i = (int)curIndexes.size(); i < rows * cols; i++) {
		curIndexes.push_back(i % N_GRAPHS);
	}

	auto avail = ImGui::GetContentRegionAvail();

	auto bgCol = ImGui::GetStyleColorVec4(ImGuiCol_FrameBg);

	ImGui::PushStyleColor(ImGuiCol_TableRowBg, bgCol);
	ImGui::PushStyleColor(ImGuiCol_TableRowBgAlt, bgCol);
	ImPlot::PushStyleColor(ImPlotCol_FrameBg, ImVec4(0,0,0,0));

	ImGui::SetNextWindowBgAlpha(1);
	if (!ImGui::BeginChild("##CalibrationDebug", avail, false,
		ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoFocusOnAppearing | ImGuiWindowFlags_NoTitleBar)) {
		ImGui::EndChild();
		return;
	}

	if (!ImGui::BeginTable("##CalibrationDebug", cols, ImGuiTableFlags_RowBg)) {
		return;
	}

	double t = refTime = Metrics::timestamp();
	PrepApplyTicks();

	for (int r = 0; r < rows; r++) {
		ImGui::TableNextRow();
		for (int c = 0; c < cols; c++) {
			int i = r * cols + c;
			ImGui::TableSetColumnIndex(c);

			ImGui::PushID(i);

			ImGui::SetNextItemWidth(ImGui::GetColumnWidth());
			if (ImGui::BeginCombo("", graphs[curIndexes[i]].name, 0)) {
				for (int j = 0; j < N_GRAPHS; j++) {
					bool isSelected = j == curIndexes[i];
					if (ImGui::Selectable(graphs[j].name, isSelected)) {
						curIndexes[i] = j;
					}
					// Per-row tooltip in the dropdown -- so the user can read
					// the description of any graph before switching to it.
					if (graphs[j].tooltip && ImGui::IsItemHovered()) {
						ImGui::SetTooltip("%s", graphs[j].tooltip);
					}

					if (isSelected) ImGui::SetItemDefaultFocus();
				}
				ImGui::EndCombo();
			}
			// Tooltip on the closed combo too, so the user doesn't have to
			// open it to see what the currently-displayed graph means.
			if (graphs[curIndexes[i]].tooltip && ImGui::IsItemHovered()) {
				ImGui::SetTooltip("%s", graphs[curIndexes[i]].tooltip);
			}

			graphs[curIndexes[i]].callback();

			ImGui::PopID();
		}
	}
	
	ImGui::EndTable();

	// (Watchdog reset count moved to the Advanced tab's Diagnostics panel --
	// it's a session counter, not a per-tick time series, and was sitting
	// alone at the bottom of the graphs page.)

	ImGui::EndChild();

	ImPlot::PopStyleColor(1);
	ImGui::PopStyleColor(2);

	if (!wasHovered) {
		lastMouseX = -INFINITY;
	}
}