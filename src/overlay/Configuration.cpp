#include "stdafx.h"
#include "Configuration.h"

#include <picojson.h>

#include <string>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <limits>

// Profile schema versioning.
//
// Persisted profiles now include a "schema_version" integer at the top level of each
// profile object. This lets us evolve the on-disk JSON shape without silently breaking
// or corrupting profiles written by older overlay builds.
//
// Version history:
//   0 - Implicit version of legacy profiles (no "schema_version" key present). All
//       fields prior to this commit. Loaded in compatibility mode.
//   1 - First explicitly-versioned schema. Same field set as version 0, but writes
//       the version key so future migrations have a starting point.
//
// When you change the schema:
//   1. Bump kProfileSchemaVersion below.
//   2. Add a step inside MigrateProfile() that transforms a parsed picojson::object
//      from the previous version to the new one (rename keys, fill defaults, etc.).
//   3. Keep the load path tolerant of missing keys for fields you add.
static const int kProfileSchemaVersion = 1;

// Forward-migrate an already-parsed profile object in place from `from_version`
// up to kProfileSchemaVersion. Called between JSON parse and field population.
//
// Each future schema bump should add a `if (from_version < N) { ... }` block here
// that rewrites the object to the version-N shape. Steps must be cumulative: a
// profile at version 0 should pass through every step on its way to current.
static void MigrateProfile(int from_version, picojson::object &profile)
{
	// from_version 0 -> 1: no field changes; the only difference is the presence
	// of the "schema_version" key itself, so nothing to rewrite here.
	(void)from_version;
	(void)profile;
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

static void ParseProfile(CalibrationContext &ctx, std::istream &stream)
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
			<< " Update OpenVR-SpaceCalibrator to use this profile." << std::endl;
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
	if (obj["jitter_threshold"].is<double>()) {
		ctx.jitterThreshold = ((float) obj["jitter_threshold"].get<double>());
	} else {
		ctx.jitterThreshold = 0.1f;
	}
	if (obj["max_relative_error_threshold"].is<double>()) {
		ctx.maxRelativeErrorThreshold = ((float) obj["max_relative_error_threshold"].get<double>());
	} else {
		ctx.maxRelativeErrorThreshold = 0.005f;
	}

	if (obj["target_latency_offset_ms"].is<double>()) {
		ctx.targetLatencyOffsetMs = obj["target_latency_offset_ms"].get<double>();
	} else {
		ctx.targetLatencyOffsetMs = 0.0;
	}

	if (obj["latency_auto_detect"].is<bool>()) {
		ctx.latencyAutoDetect = obj["latency_auto_detect"].get<bool>();
	} else {
		ctx.latencyAutoDetect = false;
	}
	if (obj["estimated_latency_offset_ms"].is<double>()) {
		ctx.estimatedLatencyOffsetMs = obj["estimated_latency_offset_ms"].get<double>();
	} else {
		ctx.estimatedLatencyOffsetMs = 0.0;
	}

	// Native prediction-suppression settings.
	ctx.suppressedSerials.clear();
	if (obj["suppressed_serials"].is<picojson::array>()) {
		for (auto& v : obj["suppressed_serials"].get<picojson::array>()) {
			if (v.is<std::string>()) {
				ctx.suppressedSerials.insert(v.get<std::string>());
			}
		}
	}
	if (obj["auto_suppress_on_external_tool"].is<bool>()) {
		ctx.autoSuppressOnExternalTool = obj["auto_suppress_on_external_tool"].get<bool>();
	} else {
		ctx.autoSuppressOnExternalTool = true;
	}

	if (obj["recalibrate_on_movement"].is<bool>()) {
		ctx.recalibrateOnMovement = obj["recalibrate_on_movement"].get<bool>();
	} else {
		ctx.recalibrateOnMovement = true;
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

	// View mode preference. New in 2026.4.28.x; older profiles get the GRAPH
	// default. Bounds-check so a hand-edited profile with garbage doesn't
	// crash on next render.
	if (obj["view_mode"].is<double>()) {
		const int raw = (int)obj["view_mode"].get<double>();
		if (raw >= 0 && raw <= 2) {
			ctx.viewMode = (CalibrationContext::ViewMode)raw;
		}
	}

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
				<< ") is not a multiple of 12 — skipping chaperone load." << std::endl;
		} else if (geometry.size() > 0) {
			ctx.chaperone.geometry.resize(geometry.size() * sizeof(float) / sizeof(ctx.chaperone.geometry[0]));
			LoadFloatArray(chaperone["geometry"], (float *) ctx.chaperone.geometry.data(), geometry.size());

			ctx.chaperone.valid = true;
		}
	}
	if (obj["relative_pos_calibrated"].is<bool>()) {
		ctx.relativePosCalibrated = obj["relative_pos_calibrated"].get<bool>();
	}
	if (obj["lock_relative_position"].is<bool>()) {
		ctx.lockRelativePosition = obj["lock_relative_position"].get<bool>();
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

	ctx.validProfile = true;
}


static void WriteStandby(StandbyDevice& device, picojson::value& value) {
	auto obj = picojson::object();

	obj["tracking_system"].set<std::string>(device.trackingSystem);
	obj["model"].set<std::string>(device.model);
	obj["serial"].set<std::string>(device.serial);

	value.set<picojson::object>(obj);
}


static void WriteProfile(CalibrationContext &ctx, std::ostream &out)
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
	bool isInContinuousCalibrationMode = ctx.state == CalibrationState::Continuous || ctx.state == CalibrationState::ContinuousStandby;
	profile["autostart_continuous_calibration"].set<bool>(isInContinuousCalibrationMode);
	profile["quash_target_in_continuous"].set<bool>(ctx.quashTargetInContinuous);
	profile["require_trigger_press_to_apply"].set<bool>(ctx.requireTriggerPressToApply);
	profile["ignore_outliers"].set<bool>(ctx.ignoreOutliers);
	profile["continuous_calibration_target_offset_x"].set<double>(ctx.continuousCalibrationOffset(0));
	profile["continuous_calibration_target_offset_y"].set<double>(ctx.continuousCalibrationOffset(1));
	profile["continuous_calibration_target_offset_z"].set<double>(ctx.continuousCalibrationOffset(2));
	profile["static_calibration"].set<bool>(ctx.enableStaticRecalibration);
	double jitterThreshold = (double)ctx.jitterThreshold;
	profile["jitter_threshold"].set<double>(jitterThreshold);
	double maxRelErrorThresTmp = (double)ctx.maxRelativeErrorThreshold;
	profile["max_relative_error_threshold"].set<double>(maxRelErrorThresTmp);
	profile["target_latency_offset_ms"].set<double>(ctx.targetLatencyOffsetMs);
	profile["latency_auto_detect"].set<bool>(ctx.latencyAutoDetect);
	profile["estimated_latency_offset_ms"].set<double>(ctx.estimatedLatencyOffsetMs);

	// Native prediction-suppression settings.
	picojson::array suppressedSerials;
	for (const auto& serial : ctx.suppressedSerials) {
		picojson::value v;
		v.set<std::string>(serial);
		suppressedSerials.push_back(v);
	}
	profile["suppressed_serials"].set<picojson::array>(suppressedSerials);
	profile["auto_suppress_on_external_tool"].set<bool>(ctx.autoSuppressOnExternalTool);
	profile["recalibrate_on_movement"].set<bool>(ctx.recalibrateOnMovement);

	double speed = (int) ctx.calibrationSpeed;
	profile["calibration_speed"].set<double>(speed);

	double viewMode = (int) ctx.viewMode;
	profile["view_mode"].set<double>(viewMode);

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
	profile["relative_pos_calibrated"].set<bool>(ctx.relativePosCalibrated);
	profile["lock_relative_position"].set<bool>(ctx.lockRelativePosition);
	profile["relative_transform"].set<picojson::object>(refToTarget);

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

static const char *RegistryKey = "Software\\OpenVR-SpaceCalibrator";

static std::string ReadRegistryKey()
{
	DWORD size = 0;
	auto result = RegGetValueA(HKEY_CURRENT_USER_LOCAL_SETTINGS, RegistryKey, "Config", RRF_RT_REG_SZ, 0, 0, &size);
	if (result != ERROR_SUCCESS) {
		LogRegistryResult(result);
		return "";
	}

	std::string str;
	str.resize(size);

	result = RegGetValueA(HKEY_CURRENT_USER_LOCAL_SETTINGS, RegistryKey, "Config", RRF_RT_REG_SZ, 0, &str[0], &size);
	if (result != ERROR_SUCCESS) {
		LogRegistryResult(result);
		return "";
	}
	
	str.resize(size - 1);
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
	//              Settings are currently stored at				HKCU\Software\Classes\Local Settings\Software\OpenVR-SpaceCalibrator
	//
	//        Profiles stored at this registry path are now versioned via the top-level
	//        "schema_version" integer (see kProfileSchemaVersion). Legacy registry blobs
	//        written before versioning are treated as version 0 and migrated on read.

	ctx.validProfile = false;

	auto str = ReadRegistryKey();
	if (str == "") {
		std::cout << "Profile is empty" << std::endl;
		ctx.Clear();
		return;
	}

	try {
		std::stringstream io(str);
		ParseProfile(ctx, io);
		std::cout << "Loaded profile" << std::endl;
	} catch (const std::runtime_error &e) {
		std::cerr << "Error loading profile: " << e.what() << std::endl;
	}
}

void SaveProfile(CalibrationContext &ctx)
{
	std::cout << "Saving profile to registry" << std::endl;

	std::stringstream io;
	WriteProfile(ctx, io);
	WriteRegistryKey(io.str());
}
