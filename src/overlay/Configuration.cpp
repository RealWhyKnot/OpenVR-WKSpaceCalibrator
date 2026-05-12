#include "stdafx.h"
#include "Configuration.h"
#include "CalibrationMetrics.h"   // WriteLogAnnotation -- profile_loaded_calibration
                                  // diagnostic line on launch.
#include "WedgeDetector.h"        // kMaxPlausibleCalibrationMagnitudeCm — shared
                                  // with the runtime wedge detector in Calibration.cpp.

#include <picojson.h>

#include <cmath>     // std::sqrt for magnitude computation in the launch log
#include <cstdio>    // snprintf for the profile_loaded_calibration log buffer
#include <string>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <limits>

// Profile schema versioning.
//
// Persisted profiles include a "schema_version" integer at the top level. This
// lets us evolve the JSON shape -- rename keys, change defaults, drop deprecated
// fields -- without writing one-off migration code in the load path every time.
//
// Two design rules keep migrations cheap:
//
//   1. Default-only fields are not written. SaveProfile compares each setting
//      against a freshly-constructed CalibrationContext (the defaults) and only
//      writes the fields the user actually changed. Adding a new setting needs
//      no migration: old profiles silently pick up the new default; new profiles
//      that don't customise it write nothing.
//
//   2. The load path is tolerant of missing keys for any field with a sensible
//      default. We only need a migration step when the meaning of a field
//      changes -- e.g. a renamed key, a removed enum value the old default
//      pointed at, or a unit change.
//
// Version history:
//   0 - Implicit version of legacy profiles (no "schema_version" key present).
//   1 - First explicitly-versioned schema. Same field set as 0; just writes the
//       version key so future migrations have a starting point.
//   2 - calibrationSpeed: SLOW (1) was the default before AUTO (3) was added.
//       Profiles whose stored value is SLOW were almost certainly never
//       customised away from the old default; rewrite them to AUTO so the user
//       gets the new sensible default automatically.
//       Also drops the deprecated "auto_suppress_on_external_tool" key, since
//       we no longer try to interop with external smoothing tools.
//   3 - Adds "additional_calibrations" array for multi-ecosystem (3+ tracking
//       system) setups. v2 profiles transparently load with an empty array;
//       no field renames or removals.
//       Adds "lock_relative_position_mode" (int 0/1/2 = OFF/ON/AUTO) replacing
//       the legacy "lock_relative_position" bool. Migration: bool true -> ON,
//       bool false -> AUTO (the new safer default that detects rigidity).
//
// When you change the schema:
//   1. Bump kProfileSchemaVersion below.
//   2. Add a step inside MigrateProfile() for the new bump.
//   3. Keep the load path tolerant of missing keys for any new field.
static const int kProfileSchemaVersion = 3;

// Set to true when a chaperone geometry array with a non-multiple-of-12 length
// is encountered during ParseProfile. The UI reads this to show a banner
// ("corrupted size -- auto-apply disabled") rather than silently doing nothing.
// Persists for the lifetime of the process (a second SaveProfile clears the
// corruption, so the banner is only shown after a load, not during normal use).
bool g_chaperoneGeometrySizeMismatch = false;

// Forward-migrate an already-parsed profile object in place from `from_version`
// up to kProfileSchemaVersion. Called between JSON parse and field population.
// Each future schema bump should add a `if (from_version < N) { ... }` block.
// Steps must be cumulative -- a v0 profile passes through every step on its way
// up to the current version.
static void MigrateProfile(int from_version, picojson::object& profile)
{
	// 0 -> 1: no field changes; only the addition of schema_version itself.
	// Nothing to rewrite.

	// 1 -> 2: SLOW was the default calibration speed before AUTO existed.
	// Treat any stored SLOW (=1) on a v0/v1 profile as "user never customised
	// this" and rewrite to AUTO (=3). Users who explicitly chose SLOW lose
	// their preference here -- a one-time cost we accept because the alternative
	// is leaving most users on the worse default forever.
	//
	// Also remove "auto_suppress_on_external_tool": we no longer interop with
	// external smoothing tools. Leaving the key around would just confuse the
	// next person reading the JSON.
	if (from_version < 2) {
		auto it = profile.find("calibration_speed");
		if (it != profile.end() && it->second.is<double>()) {
			const int raw = (int)it->second.get<double>();
			constexpr int kOldSlow = 1;
			constexpr int kAuto = 3;
			if (raw == kOldSlow) {
				double newSpeed = (double)kAuto;
				it->second.set<double>(newSpeed);
			}
		}
		profile.erase("auto_suppress_on_external_tool");
		// Skip-if-default also deprecates "suppressed_serials" (replaced by
		// "tracker_smoothness"). The load path already handles either, so
		// don't strip it here -- if the user downgrades the build they'll
		// keep that data. The next save with kProfileSchemaVersion >= 2 will
		// drop it for new profiles.
	}
}

static picojson::array FloatArray(const float *buf, size_t numFloats)
{
	picojson::array arr;

	for (int i = 0; i < numFloats; i++) {
		arr.push_back(picojson::value(double(buf[i])));
	}

	return arr;
}

static void LoadFloatArray(const picojson::value &obj, float *buf, size_t numFloats)
{
	if (!obj.is<picojson::array>()) {
		throw std::runtime_error("expected array, got " + obj.to_str());
	}

	auto &arr = obj.get<picojson::array>();
	if (arr.size() != numFloats) {
		throw std::runtime_error("wrong buffer size");
	}

	for (int i = 0; i < numFloats; i++) {
		buf[i] = (float) arr[i].get<double>();
	}
}

static void LoadStandby(StandbyDevice& device, picojson::value& value) {
	if (!value.is<picojson::object>()) {
		return;
	}
	auto& obj = value.get<picojson::object>();
	
	const auto &system = obj["tracking_system"];
	if (system.is<std::string>()) {
		device.trackingSystem = system.get<std::string>();
	}

	const auto& model = obj["model"];
	if (model.is<std::string>()) {
		device.model = model.get<std::string>();
	}

	const auto& serial = obj["serial"];
	if (serial.is<std::string>()) {
		device.serial = serial.get<std::string>();
	}
}

static void VisitAlignmentParams(CalibrationContext& ctx, std::function<void(const char *, double&)> MapParam) {
#define P(s) MapParam(#s, ctx.alignmentSpeedParams.s)
	P(align_speed_tiny);
	P(align_speed_small);
	P(align_speed_large);
	P(thr_trans_tiny);
	P(thr_trans_small);
	P(thr_trans_large);
	P(thr_rot_tiny);
	P(thr_rot_small);
	P(thr_rot_large);
	
	// Convert to double and back
	double tmp = ctx.continuousCalibrationThreshold;
	MapParam("continuousCalibrationThreshold", tmp);
	ctx.continuousCalibrationThreshold = (float)tmp;
}

static void LoadAlignmentParams(CalibrationContext& ctx, picojson::value& value) {
	ctx.ResetConfig();
	
	if (!value.is<picojson::object>()) {
		return;
	}
	auto& obj = value.get<picojson::object>();
	
	VisitAlignmentParams(ctx, [&](auto name, auto& param) {
		const picojson::value& node = obj[name];
		if (node.is<double>()) {
			param = (float)node.get<double>();
		}
	});
}

static picojson::object SaveAlignmentParams(CalibrationContext& ctx) {
	picojson::object obj;

	VisitAlignmentParams(ctx, [&](auto name, auto& param) {
		obj[name].set<double>(param);
	});

	return obj;
}

void ParseProfile(CalibrationContext &ctx, std::istream &stream)
{
	picojson::value v;
	std::string err = picojson::parse(v, stream);
	if (!err.empty()) {
		throw std::runtime_error(err);
	}

	auto arr = v.get<picojson::array>();
	if (arr.size() < 1) {
		throw std::runtime_error("no profiles in file");
	}

	auto obj = arr[0].get<picojson::object>();

	// Determine the profile schema version. Legacy profiles (written before schema
	// versioning was introduced) have no "schema_version" key and are treated as
	// version 0. A profile from a NEWER overlay than this build is refused — we'd
	// rather leave validProfile=false than silently load partial data and overwrite
	// the user's newer profile on the next save.
	int profileVersion = 0;
	if (obj["schema_version"].is<double>()) {
		profileVersion = (int)obj["schema_version"].get<double>();
	}

	if (profileVersion > kProfileSchemaVersion) {
		std::cerr << "Refusing to load profile: schema_version " << profileVersion
			<< " is newer than this build supports (" << kProfileSchemaVersion << ")."
			<< " Update OpenVR-WKSpaceCalibrator to use this profile." << std::endl;
		ctx.validProfile = false;
		return;
	}

	if (profileVersion < kProfileSchemaVersion) {
		MigrateProfile(profileVersion, obj);
	}

	LoadAlignmentParams(ctx, obj["alignment_params"]);
	ctx.referenceTrackingSystem = obj["reference_tracking_system"].get<std::string>();
	ctx.targetTrackingSystem = obj["target_tracking_system"].get<std::string>();
	ctx.calibratedRotation(0) = obj["roll"].get<double>();
	ctx.calibratedRotation(1) = obj["yaw"].get<double>();
	ctx.calibratedRotation(2) = obj["pitch"].get<double>();
	ctx.calibratedTranslation(0) = obj["x"].get<double>();
	ctx.calibratedTranslation(1) = obj["y"].get<double>();
	ctx.calibratedTranslation(2) = obj["z"].get<double>();

	// Diagnostic: surface the loaded calibration immediately at startup so
	// the user's "tracking is insanely off on launch" symptom is visible
	// in the log without needing to wait for any further events. A wedged
	// saved cal (large translation magnitude) loading on every launch is a
	// known failure mode -- having this line at startup turns "I think the
	// profile loaded wrong" into a one-grep confirmation.
	bool wedgedProfileCleared = false;
	{
		const double tx = ctx.calibratedTranslation(0);
		const double ty = ctx.calibratedTranslation(1);
		const double tz = ctx.calibratedTranslation(2);
		const double magnitude = std::sqrt(tx*tx + ty*ty + tz*tz);
		char loadbuf[256];
		snprintf(loadbuf, sizeof loadbuf,
			"profile_loaded_calibration: t=(%.3f,%.3f,%.3f) magnitude=%.3f rot_deg=(roll=%.2f, yaw=%.2f, pitch=%.2f) ref_system='%s' tgt_system='%s'",
			tx, ty, tz, magnitude,
			ctx.calibratedRotation(0), ctx.calibratedRotation(1), ctx.calibratedRotation(2),
			ctx.referenceTrackingSystem.c_str(), ctx.targetTrackingSystem.c_str());
		Metrics::WriteLogAnnotation(loadbuf);

		// Load-time wedge guard DISABLED 2026-05-05 — caused reset loops on
		// the user's Quest+Lighthouse setup where legitimate convergence
		// values fall above any plausible fixed magnitude bound. See
		// project_wedge_guard_removed_2026-05-05.md (memory). The clearing
		// logic and the wedgedProfileCleared cleanup at the bottom of
		// ParseProfile are inert when this branch is gated false. Quest
		// re-localization auto-recovery (TickHmdRelocalizationDetector)
		// remains active — that uses HMD-jump signals, not magnitude.
	}
	LoadStandby(ctx.referenceStandby, obj["reference_device"]);
	LoadStandby(ctx.targetStandby, obj["target_device"]);
	if (obj["autostart_continuous_calibration"].evaluate_as_boolean()) {
		ctx.state = CalibrationState::ContinuousStandby;
	}
	ctx.quashTargetInContinuous = obj["quash_target_in_continuous"].evaluate_as_boolean();
	ctx.requireTriggerPressToApply = obj["require_trigger_press_to_apply"].evaluate_as_boolean();
	ctx.ignoreOutliers = obj["ignore_outliers"].evaluate_as_boolean();
	ctx.continuousCalibrationOffset(0) = obj["continuous_calibration_target_offset_x"].get<double>();
	ctx.continuousCalibrationOffset(1) = obj["continuous_calibration_target_offset_y"].get<double>();
	ctx.continuousCalibrationOffset(2) = obj["continuous_calibration_target_offset_z"].get<double>();
	if (obj["static_calibration"].is<bool>()) {
		ctx.enableStaticRecalibration = obj["static_calibration"].get<bool>();
	}
	// Skip-if-default: missing keys mean "use the in-code default". ResetConfig
	// (called via LoadAlignmentParams above) already populated the defaults, so
	// each block here is "if present in JSON, override; else keep the default".
	// The previous code had explicit `else` clauses that re-set defaults; they
	// were redundant at best and had at least one bug -- jitter_threshold's
	// missing-key fallback was 0.1f, contradicting the 3.0f default set by
	// ResetConfig.
	if (obj["jitter_threshold"].is<double>())
		ctx.jitterThreshold = (float)obj["jitter_threshold"].get<double>();
	if (obj["max_relative_error_threshold"].is<double>())
		ctx.maxRelativeErrorThreshold = (float)obj["max_relative_error_threshold"].get<double>();
	if (obj["target_latency_offset_ms"].is<double>())
		ctx.targetLatencyOffsetMs = obj["target_latency_offset_ms"].get<double>();
	if (obj["latency_auto_detect"].is<bool>())
		ctx.latencyAutoDetect = obj["latency_auto_detect"].get<bool>();
	if (obj["estimated_latency_offset_ms"].is<double>())
		ctx.estimatedLatencyOffsetMs = obj["estimated_latency_offset_ms"].get<double>();
	if (obj["latency_use_gcc_phat"].is<bool>())
		ctx.useGccPhatLatency = obj["latency_use_gcc_phat"].get<bool>();
	if (obj["geometry_shift_use_cusum"].is<bool>())
		ctx.useCusumGeometryShift = obj["geometry_shift_use_cusum"].get<bool>();
	if (obj["irls_velocity_aware"].is<bool>())
		ctx.useVelocityAwareWeighting = obj["irls_velocity_aware"].get<bool>();
	if (obj["irls_use_tukey"].is<bool>())
		ctx.useTukeyBiweight = obj["irls_use_tukey"].get<bool>();
	if (obj["translation_use_upstream"].is<bool>())
		ctx.useUpstreamMath = obj["translation_use_upstream"].get<bool>();
	if (obj["translation_use_legacy"].is<bool>())
		ctx.useLegacyMath = obj["translation_use_legacy"].get<bool>();
	if (obj["blend_use_kalman"].is<bool>())
		ctx.useBlendFilter = obj["blend_use_kalman"].get<bool>();
	if (obj["rest_locked_yaw"].is<bool>())
		ctx.restLockedYawEnabled = obj["rest_locked_yaw"].get<bool>();
	if (obj["predictive_recovery"].is<bool>())
		ctx.predictiveRecoveryEnabled = obj["predictive_recovery"].get<bool>();
	if (obj["reanchor_chi_square"].is<bool>())
		ctx.reanchorChiSquareEnabled = obj["reanchor_chi_square"].get<bool>();

	// Native prediction-suppression settings.
	//
	// New schema (v2026.4.28+): "tracker_smoothness" is an object mapping
	// each tracker's serial number to a 0..100 strength.  Anything missing
	// or 0 means the tracker isn't suppressed.
	//
	// Old schema (pre-2026.4.28): "suppressed_serials" was a string array
	// (binary on/off).  Migrate by mapping every entry to smoothness=100,
	// which matches the old "fully zero velocity" behaviour.  We don't
	// rewrite the JSON on load -- the next SaveProfile will emit the new
	// schema -- so a downgrade keeps the legacy data readable too.
	// tracker_smoothness, suppressed_serials (legacy), finger_smoothing_*
	// removed from SC profiles on 2026-05-11 (Protocol v12 migration).
	// Per-tracker prediction smoothness and finger-smoothing settings now
	// live in %LocalAppDataLow%\OpenVR-Pair\profiles\smoothing.txt under
	// the Smoothing overlay. We silently drop the legacy keys on load so
	// pre-migration profiles don't error; the values themselves are
	// migrated on first launch by hand or by reimport in the Smoothing UI.

	if (obj["recalibrate_on_movement"].is<bool>()) {
		ctx.recalibrateOnMovement = obj["recalibrate_on_movement"].get<bool>();
	} else {
		ctx.recalibrateOnMovement = true;
	}

	// AUTO/OFF for the base station drift detector. Default AUTO so users
	// with Lighthouse setups get the universe-shift correction without
	// having to know it exists. Persists across launches.
	if (obj["base_station_drift_correction"].is<bool>()) {
		ctx.baseStationDriftCorrectionEnabled = obj["base_station_drift_correction"].get<bool>();
	} else {
		ctx.baseStationDriftCorrectionEnabled = true;
	}

	if (obj["scale"].is<double>()) {
		ctx.calibratedScale = obj["scale"].get<double>();
	} else {
		ctx.calibratedScale = 1.0;
	}

	if (obj["calibration_speed"].is<double>()) {
		const int raw = (int)obj["calibration_speed"].get<double>();
		// Validate the loaded enum value. AUTO was added in 2026.4.28.x; older
		// profiles only stored 0..2 (FAST/SLOW/VERY_SLOW). Anything outside
		// 0..3 is corrupt — fall back to AUTO so users get sane behavior.
		if (raw >= CalibrationContext::FAST && raw <= CalibrationContext::AUTO) {
			ctx.calibrationSpeed = (CalibrationContext::Speed)raw;
		} else {
			ctx.calibrationSpeed = CalibrationContext::AUTO;
		}
	}
	// Note: when calibration_speed is absent (older profile or new install) the
	// CalibrationContext default of AUTO applies.

	// "view_mode" was a per-profile UI density preference (BASIC/GRAPH/ADVANCED).
	// Replaced by actual top-level tabs (Basic/Graphs/Advanced) in 2026.4.28+;
	// the JSON key is dead. Skip-if-default save means new profiles won't emit
	// it; old profiles just leave the key on disk where future loads ignore it.

	if (obj["chaperone"].is<picojson::object>()) {
		auto chaperone = obj["chaperone"].get<picojson::object>();
		ctx.chaperone.autoApply = chaperone["auto_apply"].get<bool>();

		LoadFloatArray(chaperone["play_space_size"], ctx.chaperone.playSpaceSize.v, 2);

		LoadFloatArray(
			chaperone["standing_center"],
			(float *) ctx.chaperone.standingCenter.m,
			sizeof(ctx.chaperone.standingCenter.m) / sizeof(float)
		);

		if (!chaperone["geometry"].is<picojson::array>()) {
			throw std::runtime_error("chaperone geometry is not an array");
		}

		auto &geometry = chaperone["geometry"].get<picojson::array>();

		// Each chaperone quad is HmdQuad_t = 4 corners * 3 floats = 12 floats. A
		// geometry array whose length isn't a multiple of 12 is corrupt — almost
		// always a partial-write from a previous overlay crash. Loading it anyway
		// would either over-read the JSON array (LoadFloatArray throws) or store a
		// truncated final quad (silent garbage that we'd then paint as a chaperone
		// boundary). Better to skip the chaperone load and warn.
		if (geometry.size() > 0 && (geometry.size() % 12) != 0) {
			std::cerr << "Chaperone geometry length (" << geometry.size()
				<< ") is not a multiple of 12 -- skipping chaperone load." << std::endl;
			g_chaperoneGeometrySizeMismatch = true;
		} else if (geometry.size() > 0) {
			ctx.chaperone.geometry.resize(geometry.size() * sizeof(float) / sizeof(ctx.chaperone.geometry[0]));
			LoadFloatArray(chaperone["geometry"], (float *) ctx.chaperone.geometry.data(), geometry.size());

			ctx.chaperone.valid = true;
		}
	}
	if (obj["relative_pos_calibrated"].is<bool>()) {
		ctx.relativePosCalibrated = obj["relative_pos_calibrated"].get<bool>();
	}
	// Lock-mode tristate. New schema (v3+) stores "lock_relative_position_mode"
	// as a 0/1/2 int (OFF/ON/AUTO).  Legacy schemas stored a bool
	// "lock_relative_position" -- map true -> ON, false -> AUTO (was the old
	// implicit behaviour when locked).  No write of the legacy key in v3+.
	if (obj["lock_relative_position_mode"].is<double>()) {
		const int raw = (int)obj["lock_relative_position_mode"].get<double>();
		if (raw >= 0 && raw <= 2) {
			ctx.lockRelativePositionMode = (CalibrationContext::LockMode)raw;
		}
	} else if (obj["lock_relative_position"].is<bool>()) {
		ctx.lockRelativePositionMode = obj["lock_relative_position"].get<bool>()
			? CalibrationContext::LockMode::ON
			: CalibrationContext::LockMode::AUTO;
	}
	if (obj["relative_transform"].is<picojson::object>()) {
		auto relTransform = obj["relative_transform"].get<picojson::object>();
		Eigen::Vector3d refToTargetTranslation;
		refToTargetTranslation(0) = relTransform["x"].get<double>();
		refToTargetTranslation(1) = relTransform["y"].get<double>();
		refToTargetTranslation(2) = relTransform["z"].get<double>();

		Eigen::Matrix3d rotationMatrix = Eigen::Matrix3d::Identity();

		// New (quaternion) form: prefer quat_w/x/y/z when present. The previous Euler
		// form (eulerAngles(0,1,2) with `roll/yaw/pitch` keys) is fragile: Eigen's
		// eulerAngles can return values in unexpected ranges/branches when the
		// rotation passes near gimbal-lock, so a save→load round-trip can silently
		// drift. Quaternions round-trip exactly.
		if (relTransform["quat_w"].is<double>() && relTransform["quat_x"].is<double>()
			&& relTransform["quat_y"].is<double>() && relTransform["quat_z"].is<double>())
		{
			Eigen::Quaterniond q(
				relTransform["quat_w"].get<double>(),
				relTransform["quat_x"].get<double>(),
				relTransform["quat_y"].get<double>(),
				relTransform["quat_z"].get<double>());
			q.normalize();
			rotationMatrix = q.toRotationMatrix();
		}
		else if (relTransform["roll"].is<double>() && relTransform["yaw"].is<double>() && relTransform["pitch"].is<double>())
		{
			// Legacy Euler form. Kept for backward compat with profiles written by
			// older overlay builds; we never write it again.
			Eigen::Vector3d refToTargetRotation;
			refToTargetRotation(0) = relTransform["roll"].get<double>();
			refToTargetRotation(1) = relTransform["yaw"].get<double>();
			refToTargetRotation(2) = relTransform["pitch"].get<double>();
			rotationMatrix =
				Eigen::AngleAxisd(refToTargetRotation[0], Eigen::Vector3d::UnitX()) *
				Eigen::AngleAxisd(refToTargetRotation[1], Eigen::Vector3d::UnitY()) *
				Eigen::AngleAxisd(refToTargetRotation[2], Eigen::Vector3d::UnitZ());
		}

		ctx.refToTargetPose = Eigen::AffineCompact3d::Identity();
		ctx.refToTargetPose.linear() = rotationMatrix;
		ctx.refToTargetPose.translation() = refToTargetTranslation;
	}

	// Multi-ecosystem extras. Each entry mirrors the calibration data fields
	// of the primary plus its own per-extra lock mode. Standby device records
	// stay tied to the entry so we can re-resolve referenceID/targetID at
	// next scan tick after a restart.
	ctx.additionalCalibrations.clear();
	if (obj["additional_calibrations"].is<picojson::array>()) {
		for (auto& v : obj["additional_calibrations"].get<picojson::array>()) {
			if (!v.is<picojson::object>()) continue;
			auto& extraObj = v.get<picojson::object>();
			AdditionalCalibration extra;
			if (extraObj["target_tracking_system"].is<std::string>()) {
				extra.targetTrackingSystem = extraObj["target_tracking_system"].get<std::string>();
			}
			if (extraObj["target_device"].is<picojson::object>()) {
				LoadStandby(extra.targetStandby, extraObj["target_device"]);
			}
			if (extraObj["roll"].is<double>())  extra.calibratedRotation(0) = extraObj["roll"].get<double>();
			if (extraObj["yaw"].is<double>())   extra.calibratedRotation(1) = extraObj["yaw"].get<double>();
			if (extraObj["pitch"].is<double>()) extra.calibratedRotation(2) = extraObj["pitch"].get<double>();
			if (extraObj["x"].is<double>()) extra.calibratedTranslation(0) = extraObj["x"].get<double>();
			if (extraObj["y"].is<double>()) extra.calibratedTranslation(1) = extraObj["y"].get<double>();
			if (extraObj["z"].is<double>()) extra.calibratedTranslation(2) = extraObj["z"].get<double>();
			if (extraObj["scale"].is<double>()) extra.calibratedScale = extraObj["scale"].get<double>();
			if (extraObj["lock_mode"].is<double>()) {
				int raw = (int)extraObj["lock_mode"].get<double>();
				if (raw < 0 || raw > 2) raw = 2;
				extra.lockMode = raw;
			}
			if (extraObj["valid"].is<bool>()) extra.valid = extraObj["valid"].get<bool>();
			if (extraObj["enabled"].is<bool>()) extra.enabled = extraObj["enabled"].get<bool>();
			ctx.additionalCalibrations.push_back(std::move(extra));
		}
	}

	if (obj["wizard_completed"].is<bool>()) {
		ctx.wizardCompleted = obj["wizard_completed"].get<bool>();
	}

	// Load-time wedge guard, completion. The relative-pose state and
	// refToTargetPose are read further down in ParseProfile, so we can only
	// override them here, after all reads are done. End-state mirrors what
	// the runtime recovery helper in Calibration.cpp produces: a cold start
	// from identity, with continuous-cal armed to converge from fresh samples.
	if (wedgedProfileCleared) {
		ctx.refToTargetPose = Eigen::AffineCompact3d::Identity();
		ctx.relativePosCalibrated = false;
	}

	ctx.validProfile = true;
}


static void WriteStandby(StandbyDevice& device, picojson::value& value) {
	auto obj = picojson::object();

	obj["tracking_system"].set<std::string>(device.trackingSystem);
	obj["model"].set<std::string>(device.model);
	obj["serial"].set<std::string>(device.serial);

	value.set<picojson::object>(obj);
}


void WriteProfile(CalibrationContext &ctx, std::ostream &out)
{
	if (!ctx.validProfile) {
		return;
	}

	picojson::object profile;
	// Stamp the schema version first so it's prominent at the top of the serialized
	// output and so any future loader can branch on it before touching other fields.
	// picojson's set<double>() takes an lvalue reference, so the constant must be
	// stored in a local before being passed.
	double schemaVersionDouble = (double)kProfileSchemaVersion;
	profile["schema_version"].set<double>(schemaVersionDouble);

	// "Defaults" reference -- a freshly-constructed CalibrationContext.  Settings
	// fields that match this reference are NOT written to the JSON: the loader
	// will fall back to the same defaults next time, and we save disk space and
	// cognitive load on humans reading the file.  Only fields the user has
	// actually customised hit disk.
	//
	// Calibration data (translation, rotation, calibrated_scale, the standby
	// device records) is always written -- it IS the calibration, not a tunable.
	const CalibrationContext defaults;

	// --- Calibration data (always written) ------------------------------------
	profile["alignment_params"].set<picojson::object>(SaveAlignmentParams(ctx));
	profile["reference_tracking_system"].set<std::string>(ctx.referenceTrackingSystem);
	profile["target_tracking_system"].set<std::string>(ctx.targetTrackingSystem);
	profile["roll"].set<double>(ctx.calibratedRotation(0));
	profile["yaw"].set<double>(ctx.calibratedRotation(1));
	profile["pitch"].set<double>(ctx.calibratedRotation(2));
	profile["x"].set<double>(ctx.calibratedTranslation(0));
	profile["y"].set<double>(ctx.calibratedTranslation(1));
	profile["z"].set<double>(ctx.calibratedTranslation(2));
	profile["scale"].set<double>(ctx.calibratedScale);
	WriteStandby(ctx.referenceStandby, profile["reference_device"]);
	WriteStandby(ctx.targetStandby, profile["target_device"]);
	profile["continuous_calibration_target_offset_x"].set<double>(ctx.continuousCalibrationOffset(0));
	profile["continuous_calibration_target_offset_y"].set<double>(ctx.continuousCalibrationOffset(1));
	profile["continuous_calibration_target_offset_z"].set<double>(ctx.continuousCalibrationOffset(2));

	// --- State that's tied to the profile (not really a "default-able" knob) --
	bool isInContinuousCalibrationMode = ctx.state == CalibrationState::Continuous
		|| ctx.state == CalibrationState::ContinuousStandby;
	if (isInContinuousCalibrationMode) {
		// Only write the autostart flag when it's true; absent => don't autostart.
		// picojson's set<bool> is only specialized for the const-ref overload --
		// passing a literal `true` would resolve to the un-defined rvalue
		// template. Bind to a local first.
		bool autostart = true;
		profile["autostart_continuous_calibration"].set<bool>(autostart);
	}

	// --- Settings: skip-if-default ---------------------------------------------
	// Local helper macros keep the boilerplate readable. WRITE_IF_CHANGED writes
	// `field` to `key` only when ctx.field differs from defaults.field. picojson's
	// set<T> takes a non-const T& (no rvalue overload), so the value must be
	// bound to a local before the call -- the inner braces also hide the local
	// from the caller's scope.
#define WRITE_IF_CHANGED_BOOL(KEY, FIELD) \
	do { if (ctx.FIELD != defaults.FIELD) { bool _v = ctx.FIELD; profile[KEY].set<bool>(_v); } } while (0)
#define WRITE_IF_CHANGED_DOUBLE(KEY, FIELD) \
	do { if (ctx.FIELD != defaults.FIELD) { double _v = (double)ctx.FIELD; profile[KEY].set<double>(_v); } } while (0)

	WRITE_IF_CHANGED_BOOL  ("quash_target_in_continuous",   quashTargetInContinuous);
	WRITE_IF_CHANGED_BOOL  ("require_trigger_press_to_apply", requireTriggerPressToApply);
	WRITE_IF_CHANGED_BOOL  ("ignore_outliers",              ignoreOutliers);
	WRITE_IF_CHANGED_BOOL  ("static_calibration",           enableStaticRecalibration);
	WRITE_IF_CHANGED_DOUBLE("jitter_threshold",             jitterThreshold);
	WRITE_IF_CHANGED_DOUBLE("max_relative_error_threshold", maxRelativeErrorThreshold);
	WRITE_IF_CHANGED_DOUBLE("target_latency_offset_ms",     targetLatencyOffsetMs);
	WRITE_IF_CHANGED_BOOL  ("latency_auto_detect",          latencyAutoDetect);
	WRITE_IF_CHANGED_DOUBLE("estimated_latency_offset_ms",  estimatedLatencyOffsetMs);
	WRITE_IF_CHANGED_BOOL  ("latency_use_gcc_phat",         useGccPhatLatency);
	WRITE_IF_CHANGED_BOOL  ("geometry_shift_use_cusum",     useCusumGeometryShift);
	WRITE_IF_CHANGED_BOOL  ("irls_velocity_aware",          useVelocityAwareWeighting);
	WRITE_IF_CHANGED_BOOL  ("irls_use_tukey",               useTukeyBiweight);
	WRITE_IF_CHANGED_BOOL  ("translation_use_upstream",     useUpstreamMath);
	WRITE_IF_CHANGED_BOOL  ("translation_use_legacy",       useLegacyMath);
	WRITE_IF_CHANGED_BOOL  ("blend_use_kalman",             useBlendFilter);
	WRITE_IF_CHANGED_BOOL  ("rest_locked_yaw",              restLockedYawEnabled);
	WRITE_IF_CHANGED_BOOL  ("predictive_recovery",          predictiveRecoveryEnabled);
	WRITE_IF_CHANGED_BOOL  ("reanchor_chi_square",          reanchorChiSquareEnabled);
	WRITE_IF_CHANGED_BOOL  ("recalibrate_on_movement",      recalibrateOnMovement);
	WRITE_IF_CHANGED_BOOL  ("base_station_drift_correction", baseStationDriftCorrectionEnabled);
	WRITE_IF_CHANGED_DOUBLE("calibration_speed",            calibrationSpeed);

	// finger_smoothing_* and tracker_smoothness moved out of SC profiles
	// on 2026-05-11 (Protocol v12 migration). The Smoothing overlay owns
	// that state in its own config file now -- see OpenVR-WKSmoothing's
	// Config.cpp.

#undef WRITE_IF_CHANGED_BOOL
#undef WRITE_IF_CHANGED_DOUBLE

	if (ctx.chaperone.valid) {
		picojson::object chaperone;
		chaperone["auto_apply"].set<bool>(ctx.chaperone.autoApply);
		chaperone["play_space_size"].set<picojson::array>(FloatArray(ctx.chaperone.playSpaceSize.v, 2));

		chaperone["standing_center"].set<picojson::array>(FloatArray(
			(float *) ctx.chaperone.standingCenter.m,
			sizeof(ctx.chaperone.standingCenter.m) / sizeof(float)
		));

		chaperone["geometry"].set<picojson::array>(FloatArray(
			(float *) ctx.chaperone.geometry.data(),
			sizeof(ctx.chaperone.geometry[0]) / sizeof(float) * ctx.chaperone.geometry.size()
		));

		profile["chaperone"].set<picojson::object>(chaperone);
	}

	// Serialize the relative pose as a quaternion (exact round-trip) instead of
	// Eigen's eulerAngles, which is convention-fragile near gimbal lock and can
	// silently drift across save/load cycles. The legacy roll/yaw/pitch keys are
	// still understood by the reader (see ParseProfile) for backward compat with
	// older profiles.
	Eigen::Quaterniond refToTargetQuat(ctx.refToTargetPose.rotation());
	refToTargetQuat.normalize();
	Eigen::Vector3d refToTargetTranslation = ctx.refToTargetPose.translation();
	picojson::object refToTarget;
	refToTarget["x"].set<double>(refToTargetTranslation(0));
	refToTarget["y"].set<double>(refToTargetTranslation(1));
	refToTarget["z"].set<double>(refToTargetTranslation(2));
	refToTarget["quat_w"].set<double>(refToTargetQuat.w());
	refToTarget["quat_x"].set<double>(refToTargetQuat.x());
	refToTarget["quat_y"].set<double>(refToTargetQuat.y());
	refToTarget["quat_z"].set<double>(refToTargetQuat.z());
	// relative_pos_calibrated is profile state (was the rel-pose calibrated?), not a
	// user-tunable setting -- always emit so a load can tell the difference between
	// "no rel-pose data" and "rel-pose data, freshly initialised". The lock mode
	// is a setting; skip-if-default applies.
	profile["relative_pos_calibrated"].set<bool>(ctx.relativePosCalibrated);
	if (ctx.lockRelativePositionMode != defaults.lockRelativePositionMode) {
		double lockMode = (double)(int)ctx.lockRelativePositionMode;
		profile["lock_relative_position_mode"].set<double>(lockMode);
	}
	profile["relative_transform"].set<picojson::object>(refToTarget);

	// Multi-ecosystem extras. Always emit when non-empty; skip the key entirely
	// when the user hasn't added any (keeps profiles for the typical 2-system
	// case looking identical to v2 except for the schema_version bump).
	if (!ctx.additionalCalibrations.empty()) {
		picojson::array extrasArr;
		for (const auto& extra : ctx.additionalCalibrations) {
			picojson::object e;
			e["target_tracking_system"].set<std::string>(extra.targetTrackingSystem);
			picojson::value tgtDev;
			WriteStandby(const_cast<StandbyDevice&>(extra.targetStandby), tgtDev);
			e["target_device"] = tgtDev;
			e["roll"].set<double>(extra.calibratedRotation(0));
			e["yaw"].set<double>(extra.calibratedRotation(1));
			e["pitch"].set<double>(extra.calibratedRotation(2));
			e["x"].set<double>(extra.calibratedTranslation(0));
			e["y"].set<double>(extra.calibratedTranslation(1));
			e["z"].set<double>(extra.calibratedTranslation(2));
			e["scale"].set<double>(extra.calibratedScale);
			double lockMode = (double)extra.lockMode;
			e["lock_mode"].set<double>(lockMode);
			bool eValid = extra.valid;
			bool eEnabled = extra.enabled;
			e["valid"].set<bool>(eValid);
			e["enabled"].set<bool>(eEnabled);
			picojson::value entry;
			entry.set<picojson::object>(e);
			extrasArr.push_back(entry);
		}
		profile["additional_calibrations"].set<picojson::array>(extrasArr);
	}

	// Wizard-completion flag. Skipped when default (false) per skip-if-default,
	// emitted as true once the user has finished or dismissed the wizard.
	if (ctx.wizardCompleted) {
		bool wc = true;
		profile["wizard_completed"].set<bool>(wc);
	}

	picojson::value profileV;
	profileV.set<picojson::object>(profile);

	picojson::array profiles;
	profiles.push_back(profileV);

	picojson::value profilesV;
	profilesV.set<picojson::array>(profiles);

	out << profilesV.serialize(true);
}

static void LogRegistryResult(LSTATUS result)
{
	char *message;
	FormatMessageA(FORMAT_MESSAGE_FROM_SYSTEM | FORMAT_MESSAGE_ALLOCATE_BUFFER, 0, result, LANG_USER_DEFAULT, (LPSTR)&message, 0, nullptr);
	std::cerr << "Opening registry key: " << message << std::endl;
}

static const char *RegistryKey = "Software\\OpenVR-WKSpaceCalibrator";

// Strip the trailing null terminator that RegGetValueA reports as part of
// the byte count for REG_SZ. Pure for testability — see
// tests/test_configuration.cpp::ReadRegistryKey_StripNullTerminator_*.
//
// The reported `size` may legitimately be 0 if the registry value exists
// but is empty (or in a malformed/tampered state). Without this guard,
// the original code did `str.resize(size - 1)` which underflowed to
// (DWORD)0xFFFFFFFF and the resize threw std::bad_alloc, propagating up
// through LoadProfile and crashing the overlay before ParseProfile could
// even run. Returns 0 on size==0 (caller treats as "no profile, bootstrap
// fresh"); otherwise returns size-1 (drop the null).
size_t StripRegistryNullTerminator(DWORD reportedSize) {
	return reportedSize > 0 ? (size_t)reportedSize - 1 : 0;
}

// Outcome of the last ReadRegistryKey() call. Distinguishes "key simply
// absent (first run)" from "key present but value read failed (I/O error,
// permissions)". Callers use this to decide whether to surface an error
// banner vs. silently bootstrapping a fresh profile.
enum class RegistryReadOutcome {
	Absent,       // Key or value not found -- normal first-run condition.
	ReadFailed,   // Key found but data read failed -- surface to user.
	Success,      // Data read successfully.
};
static RegistryReadOutcome g_lastRegistryReadOutcome = RegistryReadOutcome::Absent;

static std::string ReadRegistryKey()
{
	DWORD size = 0;
	auto result = RegGetValueA(HKEY_CURRENT_USER_LOCAL_SETTINGS, RegistryKey, "Config", RRF_RT_REG_SZ, 0, 0, &size);
	if (result != ERROR_SUCCESS) {
		// ERROR_FILE_NOT_FOUND / ERROR_PATH_NOT_FOUND: key genuinely absent
		// (first run or profile wiped). Anything else is an I/O / permission
		// failure that should be surfaced to the user.
		if (result == ERROR_FILE_NOT_FOUND || result == ERROR_PATH_NOT_FOUND) {
			g_lastRegistryReadOutcome = RegistryReadOutcome::Absent;
		} else {
			g_lastRegistryReadOutcome = RegistryReadOutcome::ReadFailed;
			std::cerr << "Registry size query failed (not a first-run absence): ";
			LogRegistryResult(result);
		}
		return "";
	}

	// Empty / malformed registry value: short-circuit before allocating.
	if (size == 0) {
		g_lastRegistryReadOutcome = RegistryReadOutcome::Absent;
		return "";
	}

	std::string str;
	str.resize(size);

	result = RegGetValueA(HKEY_CURRENT_USER_LOCAL_SETTINGS, RegistryKey, "Config", RRF_RT_REG_SZ, 0, &str[0], &size);
	if (result != ERROR_SUCCESS) {
		// Key was present and queryable but the data read itself failed.
		// This is distinct from "key missing" -- log clearly and flag for
		// the UI so it can show a "failed to read saved calibration" banner
		// rather than silently zeroing out and pretending it's a fresh start.
		g_lastRegistryReadOutcome = RegistryReadOutcome::ReadFailed;
		std::cerr << "Registry value read failed after successful size query: ";
		LogRegistryResult(result);
		return "";
	}

	g_lastRegistryReadOutcome = RegistryReadOutcome::Success;
	// `size` is re-populated by the data-fetch call. Use the helper so
	// the truncation logic stays unit-testable.
	str.resize(StripRegistryNullTerminator(size));
	return str;
}

static void WriteRegistryKey(std::string str)
{
	HKEY hkey;
	auto result = RegCreateKeyExA(HKEY_CURRENT_USER_LOCAL_SETTINGS, RegistryKey, 0, REG_NONE, 0, KEY_ALL_ACCESS, 0, &hkey, 0);
	if (result != ERROR_SUCCESS) {
		LogRegistryResult(result);
		return;
	}

	DWORD size = static_cast<DWORD>(str.size() + 1);

	result = RegSetValueExA(hkey, "Config", 0, REG_SZ, reinterpret_cast<const BYTE*>(str.c_str()), size);
	if (result != ERROR_SUCCESS) {
		LogRegistryResult(result);
	}

	RegCloseKey(hkey);
}

void LoadProfile(CalibrationContext &ctx)
{
	// @TODO: Rewrite this to migrate configs from the registry to the spacecal directory
	//        I don't know why whoever wrote this thought writing to the registry in the 2020s was a good idea...
	//        NOTE: HKEY_CURRENT_USER_LOCAL_SETTINGS evaluates to	HKCU\Software\Classes\Local Settings
	//              Settings are currently stored at				HKCU\Software\Classes\Local Settings\Software\OpenVR-WKSpaceCalibrator
	//
	//        Profiles stored at this registry path are now versioned via the top-level
	//        "schema_version" integer (see kProfileSchemaVersion). Legacy registry blobs
	//        written before versioning are treated as version 0 and migrated on read.

	ctx.validProfile = false;

	auto str = ReadRegistryKey();
	if (str == "") {
		if (g_lastRegistryReadOutcome == RegistryReadOutcome::ReadFailed) {
			// Registry key exists but the value could not be read (I/O error,
			// ACL change, corruption). Warn visibly rather than silently
			// zeroing the calibration.
			std::cerr << "LoadProfile: failed to read saved calibration from registry; "
				"running with empty profile. Check registry permissions at "
				"HKCU\\Software\\Classes\\Local Settings\\" << RegistryKey << std::endl;
		} else {
			std::cout << "Profile is empty" << std::endl;
		}
		ctx.Clear();
		return;
	}

	try {
		std::stringstream io(str);
		ParseProfile(ctx, io);
		std::cout << "Loaded profile" << std::endl;
		// Capture the load event in the spacecal log so anyone reading the
		// session can correlate any post-load behavior change with the
		// load itself (rather than guessing whether the user re-applied a
		// stale profile mid-session).
		// NOTE: ctx.calibratedTranslation is stored in centimetres (see
		// Calibration.cpp:2800 -- "convert to cm units for profile storage").
		// Do NOT multiply by 100; the value is already in cm.
		const double transMagCm =
			std::sqrt(ctx.calibratedTranslation.x() * ctx.calibratedTranslation.x()
			        + ctx.calibratedTranslation.y() * ctx.calibratedTranslation.y()
			        + ctx.calibratedTranslation.z() * ctx.calibratedTranslation.z());
		char loadBuf[320];
		snprintf(loadBuf, sizeof loadBuf,
			"profile_loaded: bytes=%zu valid=%d trans_mag_cm=%.2f euler_deg=(%.2f,%.2f,%.2f)"
			" ref_sys=%s tgt_sys=%s",
			str.size(), (int)ctx.validProfile, transMagCm,
			ctx.calibratedRotation.x(), ctx.calibratedRotation.y(), ctx.calibratedRotation.z(),
			ctx.referenceTrackingSystem.c_str(), ctx.targetTrackingSystem.c_str());
		Metrics::WriteLogAnnotation(loadBuf);
	} catch (const std::runtime_error &e) {
		std::cerr << "Error loading profile: " << e.what() << std::endl;
		char errBuf[256];
		snprintf(errBuf, sizeof errBuf,
			"profile_load_failed: bytes=%zu reason=%s", str.size(), e.what());
		Metrics::WriteLogAnnotation(errBuf);
	}
}

void SaveProfile(CalibrationContext &ctx)
{
	std::cout << "Saving profile to registry" << std::endl;

	std::stringstream io;
	WriteProfile(ctx, io);
	const std::string serialized = io.str();
	WriteRegistryKey(serialized);

	// Annotate the save event so the log lets a reader correlate writes
	// with the cal-state changes that triggered them. The wedged-state-saved
	// bug from 2026-05-03 was hard to debug because saves were silent; this
	// closes that gap. Includes the magnitude of what we just persisted so
	// a "saved a wedged cal at time T" question can be spot-checked.
	const double transMagCm =
		std::sqrt(ctx.calibratedTranslation.x() * ctx.calibratedTranslation.x()
		        + ctx.calibratedTranslation.y() * ctx.calibratedTranslation.y()
		        + ctx.calibratedTranslation.z() * ctx.calibratedTranslation.z()) * 100.0;
	char saveBuf[256];
	snprintf(saveBuf, sizeof saveBuf,
		"profile_saved: bytes=%zu valid=%d trans_mag_cm=%.2f",
		serialized.size(), (int)ctx.validProfile, transMagCm);
	Metrics::WriteLogAnnotation(saveBuf);
}
