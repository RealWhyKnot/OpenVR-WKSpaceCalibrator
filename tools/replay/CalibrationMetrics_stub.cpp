// Stub implementations of the symbols CalibrationCalc.cpp pulls in from the
// overlay's metrics + calibration-context layer. The real implementations are
// tightly coupled to the overlay process (Win32 file I/O for log rotation, a
// CalibrationContext singleton that owns the live device-pose table, etc.) and
// don't make sense in an offline replay tool. Everything here is a no-op shim
// or a default-initialized global.
//
// If new Metrics::TimeSeries or CalibrationContext members start being touched
// from CalibrationCalc.cpp, add the corresponding stub definition here. Linker
// errors will spell out exactly what's needed.

// <iostream> must come before Calibration.h: that header's CalibrationContext::Log
// inlines `std::cerr << msg`, but doesn't include <iostream> itself (the overlay
// project relies on stdafx.h to provide it). The replay tool has no precompiled
// header, so we pull it in here.
#include <iostream>

#include "Calibration.h"
#include "CalibrationMetrics.h"

#include <cstdio>

// CalCtx — the global CalibrationContext that CalibrationCalc.cpp logs to.
// We only need its Log() method to be callable; the live overlay populates
// the rest via CalibrationTick(). Default-construct it here and let
// CalibrationContext::Log() in Calibration.h push messages onto its `messages`
// deque (and write to std::cerr) — that's fine for a CLI tool.
CalibrationContext CalCtx;

namespace Metrics {
	double TimeSpan = 30.0;
	double CurrentTime = 0.0;

	TimeSeries<Eigen::Vector3d> posOffset_rawComputed;
	TimeSeries<Eigen::Vector3d> posOffset_currentCal;
	TimeSeries<Eigen::Vector3d> posOffset_lastSample;
	TimeSeries<Eigen::Vector3d> posOffset_byRelPose;

	TimeSeries<double> error_rawComputed, error_currentCal, error_byRelPose, error_currentCalRelPose;
	TimeSeries<double> axisIndependence;
	TimeSeries<double> computationTime;
	TimeSeries<double> jitterRef, jitterTarget;
	TimeSeries<double> rotationConditionRatio;
	TimeSeries<double> consecutiveRejections;
	TimeSeries<double> samplesInBuffer;
	TimeSeries<double> watchdogResetCount;
	TimeSeries<double> translationDiversity;
	TimeSeries<double> rotationDiversity;
	TimeSeries<Eigen::Vector3d> translationAxisRangesCm;
	TimeSeries<double> watchdogHealthySkip;
	TimeSeries<double> effectivePriorMm;
	TimeSeries<double> validateRmsThresholdMm;
	std::string lastRejectReason;

	// Driver-side apply-rate counters. Not used by the offline replay tool but
	// CalibrationCalc.cpp doesn't reference them; leave them out unless a future
	// CalibrationCalc change needs them.

	TimeSeries<bool> calibrationApplied;

	bool enableLogs = false;

	double timestamp() { return 0.0; }
	void RecordTimestamp() { /* CurrentTime is advanced by the tool itself per replayed tick. */ }

	void SetTickRawPoses(
		const Eigen::Vector3d&, const Eigen::Quaterniond&,
		const Eigen::Vector3d&, const Eigen::Quaterniond&,
		TickPhase) {
		// Unused offline — we already have the raw poses in the input CSV.
	}

	void WriteLogAnnotation(const char* s) {
		// Surface watchdog / state-transition annotations to stderr so the user
		// sees them during replay. The live overlay writes these into the CSV
		// itself; in replay we don't write a CSV back out.
		std::fprintf(stderr, "[annotation] %s\n", s);
	}

	void WriteLogEntry() { /* The replay tool never writes log rows back. */ }
}
