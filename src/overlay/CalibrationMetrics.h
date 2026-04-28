#pragma once

#include <deque>
#include <utility>
#include <Eigen/Dense>

namespace Metrics {
	extern double TimeSpan, CurrentTime;

	double timestamp();
	void RecordTimestamp();

	template<typename T>
	class TimeSeries {
		std::deque<std::pair<double, T>> Data;
		
	public:
		const std::deque<std::pair<double, T>> &data() const { return Data; }

		void Push(const T& data) {
			Data.push_back(std::make_pair(CurrentTime, data));

			double cutoff = CurrentTime - TimeSpan;
			while (!Data.empty() && (Data.front().first < cutoff || Data.size() > INT_MAX)) {
				Data.pop_front();
			}
		}

		int size() const { return (int)Data.size(); }
		const std::pair<double, T>& operator[](int index) const { return Data[index]; }

		const T& last() const {
			static const T fallback;
			return Data.size() > 0 ? Data.back().second : fallback;
		}

		const double lastTs() const {
			return Data.size() > 0 ? Data.back().first : 0;
		}
	};


	extern TimeSeries<Eigen::Vector3d> posOffset_rawComputed; // , rotOffset_rawComputed;
	extern TimeSeries<Eigen::Vector3d> posOffset_currentCal; // , rotOffset_currentCal;
	extern TimeSeries<Eigen::Vector3d> posOffset_lastSample; // , rotOffset_lastSample;
	extern TimeSeries<Eigen::Vector3d> posOffset_byRelPose;
	
	extern TimeSeries<double> error_rawComputed, error_currentCal, error_byRelPose, error_currentCalRelPose;
	extern TimeSeries<double> axisIndependence;
	extern TimeSeries<double> computationTime;
	extern TimeSeries<double> jitterRef, jitterTarget;

	// Smallest/largest singular value ratio of the 2D Kabsch yaw covariance.
	// Drops near zero when the user moves on a single axis only.
	extern TimeSeries<double> rotationConditionRatio;

	// Running count of ComputeIncremental rejections since the last successful
	// accept. The stuck-loop watchdog fires when this exceeds its threshold.
	extern TimeSeries<double> consecutiveRejections;

	extern TimeSeries<bool> calibrationApplied;

	extern bool enableLogs;

	// Phase of the per-tick CalibrationTick state machine at the moment WriteLogEntry()
	// is called. Stored in the v2 CSV "tick_phase" column to let the replay harness
	// reproduce the same control-flow path that produced each row. Mirrors the
	// CalibrationState enum but is held independently so CalibrationMetrics.h doesn't
	// have to pull in Calibration.h.
	enum class TickPhase {
		None,
		Begin,
		Rotation,
		Translation,
		Editing,
		Continuous,
		ContinuousStandby,
	};

	// Set the raw reference and target pose (translation + quaternion) and the tick
	// phase that will be written by the next WriteLogEntry() call. Caller is expected
	// to invoke this once per tick, just before WriteLogEntry(), so the v2 columns
	// stay consistent with the metrics already snapshotted into the TimeSeries above.
	void SetTickRawPoses(
		const Eigen::Vector3d& refTrans, const Eigen::Quaterniond& refRot,
		const Eigen::Vector3d& targetTrans, const Eigen::Quaterniond& targetRot,
		TickPhase phase);

	void WriteLogAnnotation(const char* s);
	void WriteLogEntry();
}