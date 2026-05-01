// Minimal symbol stubs to satisfy the linker when compiling CalibrationCalc.cpp
// in isolation for unit tests. CalibrationCalc references:
//   - CalCtx (a CalibrationContext) for logging.
//   - The Metrics::TimeSeries<> globals declared in CalibrationMetrics.h.
//   - Metrics::RecordTimestamp() and Metrics::WriteLogAnnotation().
//
// We provide just enough to make the link succeed without dragging in the full
// overlay (OpenGL, ImGui, OpenVR runtime, Windows shell APIs, etc).

// Calibration.h transitively expects <iostream> to be visible (for std::cerr
// in CalibrationContext::Log). The overlay gets it via stdafx.h; we need to
// include it ourselves before pulling Calibration.h in.
#include <iostream>

#include "Calibration.h"
#include "CalibrationMetrics.h"

// Storage for the global calibration context that CalibrationCalc.cpp logs
// through. The default constructor pre-populates the alignment-speed thresholds
// and zeroes the device-pose array, which is exactly what we want for test
// runs (no live VR system).
CalibrationContext CalCtx;

namespace Metrics {
    // Definitions for the storage of the time-series globals declared in
    // CalibrationMetrics.h. The overlay uses these for live plots; the tests
    // only need them to exist so Push() calls inside CalibrationCalc don't
    // produce undefined references.
    double TimeSpan = 30.0;
    double CurrentTime = 0.0;

    TimeSeries<Eigen::Vector3d> posOffset_rawComputed;
    TimeSeries<Eigen::Vector3d> posOffset_currentCal;
    TimeSeries<Eigen::Vector3d> posOffset_lastSample;
    TimeSeries<Eigen::Vector3d> posOffset_byRelPose;

    TimeSeries<double> error_rawComputed;
    TimeSeries<double> error_currentCal;
    TimeSeries<double> error_byRelPose;
    TimeSeries<double> error_currentCalRelPose;
    TimeSeries<double> axisIndependence;
    TimeSeries<double> computationTime;
    TimeSeries<double> jitterRef;
    TimeSeries<double> jitterTarget;
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

    TimeSeries<bool> calibrationApplied;

    bool enableLogs = false;

    double timestamp() {
        // Tests don't rely on wall-clock timing — return a monotonic counter
        // so successive calls are ordered but cheap.
        static double t = 0.0;
        t += 1.0;
        return t;
    }

    void RecordTimestamp() {
        CurrentTime = timestamp();
    }

    void WriteLogAnnotation(const char* /*s*/) {
        // No-op: the tests don't open a CSV log file.
    }

    void WriteLogEntry() {
        // No-op.
    }
}
