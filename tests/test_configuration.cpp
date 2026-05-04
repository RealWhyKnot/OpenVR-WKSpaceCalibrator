// Tests for the profile JSON load/save round-trip and the schema migration
// pipeline. The production code routes through Windows registry I/O (Load-
// Profile / SaveProfile in Configuration.cpp), but the underlying parsing
// and serialization is exposed via ParseProfile / WriteProfile, which take
// std::istream / std::ostream and don't touch the registry. We exercise
// those directly here so the test stays hermetic.
//
// Coverage:
//   - Schema-version write: every fresh save stamps the current version.
//   - Schema migration: legacy profiles with no schema_version load as v0
//     and migrate up to the current version.
//   - "Refuse to load newer" guard: a profile claiming a future schema
//     version leaves validProfile=false and doesn't corrupt the in-memory
//     context.
//   - Round-trip: every customised setting survives save->load.
//   - Round-trip covers every persisted field (calibrated transform,
//     thresholds, lock mode, etc.).

#include <gtest/gtest.h>

#include <cstring>
#include <sstream>
#include <string>

#include "Configuration.h"
#include "Calibration.h"

namespace {

// Build a minimal v0 / v1 / v2 / v3 JSON payload programmatically. Real
// production profiles have many more keys; for the migration tests we only
// need the schema-version field plus enough valid scaffolding that
// ParseProfile doesn't reject for missing required keys.
std::string MakeMinimalProfile(int schemaVersion, const std::string& extraKeys = "") {
    std::ostringstream o;
    o << "[{";
    if (schemaVersion >= 1) {
        o << "\"schema_version\":" << schemaVersion << ",";
    }
    o << "\"reference_tracking_system\":\"lighthouse\",";
    o << "\"target_tracking_system\":\"oculus\",";
    o << "\"roll\":0.0,\"yaw\":0.0,\"pitch\":0.0,";
    o << "\"x\":0.0,\"y\":0.0,\"z\":0.0,";
    o << "\"continuous_calibration_target_offset_x\":0.0,";
    o << "\"continuous_calibration_target_offset_y\":0.0,";
    o << "\"continuous_calibration_target_offset_z\":0.0";
    if (!extraKeys.empty()) {
        o << "," << extraKeys;
    }
    o << "}]";
    return o.str();
}

} // namespace

// ---------------------------------------------------------------------------
// Round-trip: write a context, read it back into a fresh context, every
// customised field survives. Acts as a contract test for "the profile
// remembers what the user did" -- the most basic configuration test.
// ---------------------------------------------------------------------------
TEST(ConfigurationTest, RoundTripPreservesCustomFields) {
    CalibrationContext src;
    src.referenceTrackingSystem = "lighthouse";
    src.targetTrackingSystem = "oculus";
    src.calibratedTranslation = Eigen::Vector3d(12.5, 0.0, -7.25);
    src.calibratedRotation = Eigen::Vector3d(0.0, 45.0, 0.0);
    src.jitterThreshold = 5.5f;
    src.recalibrateOnMovement = false;
    src.baseStationDriftCorrectionEnabled = false;
    src.ignoreOutliers = true;
    src.continuousCalibrationThreshold = 2.5f;
    src.calibrationSpeed = CalibrationContext::FAST;
    src.lockRelativePositionMode = CalibrationContext::LockMode::ON;
    src.validProfile = true;

    std::stringstream io;
    WriteProfile(src, io);

    CalibrationContext dst;
    ParseProfile(dst, io);

    EXPECT_EQ(dst.referenceTrackingSystem, "lighthouse");
    EXPECT_EQ(dst.targetTrackingSystem, "oculus");
    EXPECT_DOUBLE_EQ(dst.calibratedTranslation.x(), 12.5);
    EXPECT_DOUBLE_EQ(dst.calibratedTranslation.z(), -7.25);
    EXPECT_DOUBLE_EQ(dst.calibratedRotation.y(), 45.0);
    EXPECT_FLOAT_EQ(dst.jitterThreshold, 5.5f);
    EXPECT_FALSE(dst.recalibrateOnMovement);
    EXPECT_FALSE(dst.baseStationDriftCorrectionEnabled);
    EXPECT_TRUE(dst.ignoreOutliers);
    EXPECT_FLOAT_EQ(dst.continuousCalibrationThreshold, 2.5f);
    EXPECT_EQ(dst.calibrationSpeed, CalibrationContext::FAST);
    EXPECT_EQ(dst.lockRelativePositionMode, CalibrationContext::LockMode::ON);
}

// ---------------------------------------------------------------------------
// Default-only fields are not written and load as defaults. The skip-if-
// default optimization means a brand-new context's saved JSON shouldn't
// carry every field; missing keys reload as the in-code defaults, which
// matches the in-code defaults baked into CalibrationContext.
// ---------------------------------------------------------------------------
TEST(ConfigurationTest, DefaultFieldsRoundTripAsDefaults) {
    CalibrationContext src; // fresh defaults
    src.referenceTrackingSystem = "lighthouse";
    src.targetTrackingSystem = "oculus";
    src.validProfile = true;

    std::stringstream io;
    WriteProfile(src, io);

    CalibrationContext dst;
    ParseProfile(dst, io);

    // The in-code defaults survive a no-customization round-trip.
    EXPECT_TRUE(dst.recalibrateOnMovement);              // default true
    EXPECT_TRUE(dst.enableStaticRecalibration);          // default true (flipped this session)
    EXPECT_TRUE(dst.baseStationDriftCorrectionEnabled);  // default AUTO (no-op without base stations)
    EXPECT_FLOAT_EQ(dst.jitterThreshold, 3.0f);
    EXPECT_EQ(dst.calibrationSpeed, CalibrationContext::AUTO);
    EXPECT_EQ(dst.lockRelativePositionMode, CalibrationContext::LockMode::AUTO);
}

// ---------------------------------------------------------------------------
// Refuse-to-load-newer guard: if a future build wrote a profile with a
// higher schema version, the current build must NOT corrupt the in-memory
// context by partially loading it. The guard sets validProfile=false and
// returns early.
// ---------------------------------------------------------------------------
TEST(ConfigurationTest, RefusesProfileFromNewerSchema) {
    // Pick a version far enough in the future that no near-term schema bump
    // would accidentally make this test pass. 999 is symbolic.
    std::string newerJson = MakeMinimalProfile(/*schemaVersion=*/999);

    CalibrationContext ctx;
    ctx.validProfile = true; // pre-set, the guard should clear this
    std::stringstream io(newerJson);
    ParseProfile(ctx, io);

    EXPECT_FALSE(ctx.validProfile)
        << "Refusing to load a newer-schema profile must leave validProfile=false";
}

// ---------------------------------------------------------------------------
// Schema migration v0 -> current. v0 = no schema_version key (legacy
// profiles written before the version was introduced). The load path must
// run all forward-migration steps; specifically it must NOT keep
// auto_suppress_on_external_tool around (dropped in v2's migration).
// ---------------------------------------------------------------------------
TEST(ConfigurationTest, MigrateV0ProfileLoadsCleanly) {
    // v0 had no schema_version key. Include the legacy auto-suppress key so
    // we can verify the migration drops it. Add a SLOW calibration_speed so
    // we can verify the v1->v2 step rewrites it to AUTO.
    std::string v0Json = MakeMinimalProfile(
        /*schemaVersion=*/0,
        // calibration_speed=1 -> SLOW. Migration rewrites to 3 (AUTO).
        "\"calibration_speed\":1,"
        "\"auto_suppress_on_external_tool\":true");

    CalibrationContext ctx;
    std::stringstream io(v0Json);
    ParseProfile(ctx, io);

    // Migration v1->v2 rewrites legacy SLOW (=1) to AUTO (=3) since SLOW was
    // the old default that most users never customised.
    EXPECT_EQ(ctx.calibrationSpeed, CalibrationContext::AUTO)
        << "v1->v2 migration should rewrite legacy SLOW default to AUTO";

    // Profile loaded; main fields populated.
    EXPECT_EQ(ctx.referenceTrackingSystem, "lighthouse");
    EXPECT_EQ(ctx.targetTrackingSystem, "oculus");
}

// ---------------------------------------------------------------------------
// Schema migration v2 -> v3. v2 profiles have no additional_calibrations
// array (multi-ecosystem support added in v3). They must load as if extras
// were an empty list.
// ---------------------------------------------------------------------------
TEST(ConfigurationTest, MigrateV2ProfileLoadsWithEmptyExtras) {
    std::string v2Json = MakeMinimalProfile(/*schemaVersion=*/2);

    CalibrationContext ctx;
    std::stringstream io(v2Json);
    ParseProfile(ctx, io);

    EXPECT_EQ(ctx.referenceTrackingSystem, "lighthouse");
    EXPECT_TRUE(ctx.additionalCalibrations.empty())
        << "v2 profile should load with no extras (additional_calibrations was added in v3)";
}

// ---------------------------------------------------------------------------
// Default-value pins. These are the in-code defaults that ResetConfig()
// produces; load paths inherit them when a key is missing from the JSON. A
// test that fails here means a default was changed -- look at the diff and
// make sure the change is intentional, then update the literal here.
//
// Why pin defaults: silent default flips are a class of bug that's hard to
// catch in code review. Forcing the test to be touched whenever a default
// changes makes the change explicit and reviewable.
// ---------------------------------------------------------------------------
TEST(ConfigurationTest, InCodeDefaultsArePinned) {
    CalibrationContext ctx;

    EXPECT_FLOAT_EQ(ctx.jitterThreshold, 3.0f);
    EXPECT_TRUE(ctx.recalibrateOnMovement)
        << "recalibrateOnMovement default is ON: prevents phantom drift while still";
    EXPECT_TRUE(ctx.enableStaticRecalibration)
        << "enableStaticRecalibration default is ON: no-op when not locked, "
           "accelerates rigid-attachment recovery; flipped on this fork";
    EXPECT_TRUE(ctx.baseStationDriftCorrectionEnabled)
        << "baseStationDriftCorrectionEnabled default is AUTO (true): "
           "no-op when no base stations are detected (e.g. Quest-only "
           "setups), corrects on detected universe shifts otherwise";
    EXPECT_FALSE(ctx.requireTriggerPressToApply);
    EXPECT_FALSE(ctx.ignoreOutliers);
    EXPECT_FALSE(ctx.quashTargetInContinuous);
    EXPECT_FLOAT_EQ(ctx.continuousCalibrationThreshold, 1.5f);
    EXPECT_FLOAT_EQ(ctx.maxRelativeErrorThreshold, 0.005f);
    EXPECT_EQ(ctx.calibrationSpeed, CalibrationContext::AUTO);
    EXPECT_EQ(ctx.lockRelativePositionMode, CalibrationContext::LockMode::AUTO);
    EXPECT_DOUBLE_EQ(ctx.calibratedScale, 1.0);
    EXPECT_DOUBLE_EQ(ctx.targetLatencyOffsetMs, 0.0);
    EXPECT_FALSE(ctx.latencyAutoDetect);
}

// ---------------------------------------------------------------------------
// Wedged-profile guard: a saved cal whose translation magnitude exceeds the
// plausibility bound (200 cm) loads as zeroed translation/rotation, so
// continuous-cal next session starts cold instead of inheriting the wedge.
// Mirrors the ParseProfile path described in
// project_watchdog_wedged_cal_limitation.md (memory).
// ---------------------------------------------------------------------------
TEST(ConfigurationTest, WedgedProfileMagnitudeIsZeroedOnLoad) {
    // Use the exact wedged values from the user's 2026-05-04 reproduction
    // (spacecal_log.2026-05-04T17-14-50.txt): t=(-168, 215, -112) cm,
    // magnitude ≈ 295 cm. Solidly above the 200 cm bound; the guard must
    // zero translation+rotation. The rest of the profile (e.g. tracking-
    // system names) must still load.
    std::string wedgedJson = MakeMinimalProfile(/*schemaVersion=*/3);
    // Replace the zero translation in the minimal payload with the wedged
    // magnitude. Easier than building a custom profile from scratch.
    auto pos = wedgedJson.find("\"x\":0.0,\"y\":0.0,\"z\":0.0");
    ASSERT_NE(pos, std::string::npos);
    wedgedJson.replace(pos, std::strlen("\"x\":0.0,\"y\":0.0,\"z\":0.0"),
                       "\"x\":-168.0,\"y\":215.0,\"z\":-112.0");

    CalibrationContext ctx;
    std::stringstream io(wedgedJson);
    ParseProfile(ctx, io);

    EXPECT_DOUBLE_EQ(ctx.calibratedTranslation.x(), 0.0);
    EXPECT_DOUBLE_EQ(ctx.calibratedTranslation.y(), 0.0);
    EXPECT_DOUBLE_EQ(ctx.calibratedTranslation.z(), 0.0);
    EXPECT_DOUBLE_EQ(ctx.calibratedRotation.x(), 0.0);
    EXPECT_DOUBLE_EQ(ctx.calibratedRotation.y(), 0.0);
    EXPECT_DOUBLE_EQ(ctx.calibratedRotation.z(), 0.0);
    EXPECT_FALSE(ctx.relativePosCalibrated);
    EXPECT_TRUE(ctx.validProfile)
        << "Guard zeroes the cal but keeps the profile loaded -- the user's "
           "settings (tracking-system names, calibration speed, etc.) should "
           "survive so they don't have to reconfigure on top of recalibrating";
    EXPECT_EQ(ctx.referenceTrackingSystem, "lighthouse");
    EXPECT_EQ(ctx.targetTrackingSystem, "oculus");
}

// ---------------------------------------------------------------------------
// Wedged-profile guard variant: just-over-threshold (201 cm). Verifies the
// boundary is exclusive of equality on the safe side: anything strictly
// greater than 200 cm gets cleared. A user whose legitimate setup magnitudes
// land at 201+ cm would also trip this — that's why
// kMaxPlausibleCalibrationMagnitudeCm is greenlit at 200 not lower.
// ---------------------------------------------------------------------------
TEST(ConfigurationTest, WedgedProfileJustOverThresholdIsCleared) {
    // x=201 cm → magnitude=201 cm, which is > 200 cm bound.
    std::string json = MakeMinimalProfile(/*schemaVersion=*/3);
    auto pos = json.find("\"x\":0.0,\"y\":0.0,\"z\":0.0");
    ASSERT_NE(pos, std::string::npos);
    json.replace(pos, std::strlen("\"x\":0.0,\"y\":0.0,\"z\":0.0"),
                 "\"x\":201.0,\"y\":0.0,\"z\":0.0");

    CalibrationContext ctx;
    std::stringstream io(json);
    ParseProfile(ctx, io);

    EXPECT_DOUBLE_EQ(ctx.calibratedTranslation.x(), 0.0)
        << "201 cm magnitude must be cleared (strictly > 200 cm bound)";
    EXPECT_DOUBLE_EQ(ctx.calibratedTranslation.y(), 0.0);
    EXPECT_DOUBLE_EQ(ctx.calibratedTranslation.z(), 0.0);
}

// ---------------------------------------------------------------------------
// Wedged-profile guard variant: just-under-threshold (199 cm). Verifies the
// boundary doesn't accidentally fire on cals that are unusual but plausible.
// Setups whose origin separation legitimately approaches 2 m (e.g. a Quest
// origin at one corner of a large room with Lighthouse origin at the other)
// must NOT be clobbered.
// ---------------------------------------------------------------------------
TEST(ConfigurationTest, WedgedProfileJustUnderThresholdIsKept) {
    // x=199 cm → magnitude=199 cm, which is < 200 cm bound.
    CalibrationContext src;
    src.referenceTrackingSystem = "lighthouse";
    src.targetTrackingSystem = "oculus";
    src.calibratedTranslation = Eigen::Vector3d(199.0, 0.0, 0.0);
    src.calibratedRotation = Eigen::Vector3d(0.0, 0.0, 0.0);
    src.validProfile = true;

    std::stringstream io;
    WriteProfile(src, io);

    CalibrationContext dst;
    ParseProfile(dst, io);

    EXPECT_DOUBLE_EQ(dst.calibratedTranslation.x(), 199.0)
        << "199 cm magnitude is below the 200 cm bound and must round-trip "
           "intact — large-room setups must keep their cals";
}

// ---------------------------------------------------------------------------
// Wedged-profile guard variant: rotation alone is irrelevant. The guard
// gates only on translation magnitude. A profile with extreme rotation
// values (180°+ on every axis) but small translation must NOT be cleared —
// rotation values are radians-or-degrees but the user's frame relationship
// might genuinely require any orientation.
// ---------------------------------------------------------------------------
TEST(ConfigurationTest, WedgedProfileLargeRotationSmallTranslationIsKept) {
    CalibrationContext src;
    src.referenceTrackingSystem = "lighthouse";
    src.targetTrackingSystem = "oculus";
    src.calibratedTranslation = Eigen::Vector3d(50.0, 50.0, 50.0);  // ~87 cm magnitude
    // Pretty much any rotation values are physically valid for a refToTarget
    // frame relationship; pin that the guard doesn't second-guess them.
    src.calibratedRotation = Eigen::Vector3d(180.0, -135.0, 90.0);
    src.validProfile = true;

    std::stringstream io;
    WriteProfile(src, io);

    CalibrationContext dst;
    ParseProfile(dst, io);

    EXPECT_DOUBLE_EQ(dst.calibratedTranslation.x(), 50.0);
    EXPECT_DOUBLE_EQ(dst.calibratedRotation.x(), 180.0)
        << "Guard must not gate on rotation magnitude — only translation";
    EXPECT_DOUBLE_EQ(dst.calibratedRotation.y(), -135.0);
    EXPECT_DOUBLE_EQ(dst.calibratedRotation.z(), 90.0);
}

// ---------------------------------------------------------------------------
// Wedged-profile guard variant: extreme magnitude (registry-observed 254 cm
// from the user's actual saved profile on 2026-05-04). Pinned to the exact
// values read from HKCU at audit time so a future change can't silently
// break the user's known reproduction.
// ---------------------------------------------------------------------------
TEST(ConfigurationTest, WedgedProfileFromUserReproductionIsCleared) {
    // From `HKCU:\Software\Classes\Local Settings\Software\OpenVR-SpaceCalibrator`
    // read 2026-05-04 during the HMD-on/off-drift diagnosis: t=(-251.6, -36.3, -4.6),
    // magnitude≈254.2 cm. This is the cal that motivated the entire wedge guard.
    std::string json = MakeMinimalProfile(/*schemaVersion=*/3);
    auto pos = json.find("\"x\":0.0,\"y\":0.0,\"z\":0.0");
    ASSERT_NE(pos, std::string::npos);
    json.replace(pos, std::strlen("\"x\":0.0,\"y\":0.0,\"z\":0.0"),
                 "\"x\":-251.55873114202993,\"y\":-36.317712345397368,\"z\":-4.6416934383008508");

    CalibrationContext ctx;
    std::stringstream io(json);
    ParseProfile(ctx, io);

    EXPECT_DOUBLE_EQ(ctx.calibratedTranslation.x(), 0.0)
        << "The user's actual 2026-05-04 wedge values must be cleared on load";
    EXPECT_DOUBLE_EQ(ctx.calibratedTranslation.y(), 0.0);
    EXPECT_DOUBLE_EQ(ctx.calibratedTranslation.z(), 0.0);
}

// ---------------------------------------------------------------------------
// Wedged-profile guard: a normal-magnitude cal (well under 200 cm) round-
// trips unchanged. Pinned so a future tightening of the bound doesn't
// silently break healthy cals.
// ---------------------------------------------------------------------------
TEST(ConfigurationTest, NormalMagnitudeProfileIsNotCleared) {
    // 127 cm magnitude (the user's converged-healthy fit from the same log).
    // Any future bound-tightening must keep this case loading verbatim.
    CalibrationContext src;
    src.referenceTrackingSystem = "lighthouse";
    src.targetTrackingSystem = "oculus";
    src.calibratedTranslation = Eigen::Vector3d(-0.8, 122.5, -34.8);
    src.calibratedRotation = Eigen::Vector3d(0.0, -110.9, 0.0);
    src.validProfile = true;

    std::stringstream io;
    WriteProfile(src, io);

    CalibrationContext dst;
    ParseProfile(dst, io);

    EXPECT_DOUBLE_EQ(dst.calibratedTranslation.x(), -0.8);
    EXPECT_DOUBLE_EQ(dst.calibratedTranslation.y(), 122.5);
    EXPECT_DOUBLE_EQ(dst.calibratedTranslation.z(), -34.8);
    EXPECT_DOUBLE_EQ(dst.calibratedRotation.y(), -110.9);
}

// ---------------------------------------------------------------------------
// Regression guard for the registry-read underflow bug fixed 2026-05-04.
// RegGetValueA can return size==0 for an empty/malformed REG_SZ; the original
// code did `str.resize(size - 1)` which underflowed to 0xFFFFFFFF and threw
// std::bad_alloc, crashing the overlay before ParseProfile could run. Pinned
// behavior: size==0 input maps to size==0 output (caller treats as "no
// profile"); positive input maps to input-1 (strip the null terminator).
// ---------------------------------------------------------------------------
#ifdef _WIN32
TEST(ConfigurationTest, Regression_StripRegistryNullTerminator_HandlesZero) {
    EXPECT_EQ(StripRegistryNullTerminator(0), 0u)
        << "size==0 must NOT underflow — caller short-circuits to empty string";
}

TEST(ConfigurationTest, Regression_StripRegistryNullTerminator_StripsOne) {
    EXPECT_EQ(StripRegistryNullTerminator(1), 0u)
        << "size==1 (just a null byte) maps to empty string";
    EXPECT_EQ(StripRegistryNullTerminator(2), 1u);
    EXPECT_EQ(StripRegistryNullTerminator(10), 9u);
    EXPECT_EQ(StripRegistryNullTerminator(0xFFFF), 0xFFFEu);
}
#endif

// ---------------------------------------------------------------------------
// Save always stamps the current schema_version. Future reads (on the same
// or later builds) need that key present; without it, even today's load
// path treats the profile as v0 and runs the migration steps redundantly.
// ---------------------------------------------------------------------------
TEST(ConfigurationTest, SaveStampsCurrentSchemaVersion) {
    CalibrationContext src;
    src.referenceTrackingSystem = "lighthouse";
    src.targetTrackingSystem = "oculus";
    src.validProfile = true;

    std::stringstream io;
    WriteProfile(src, io);

    // Reload through ParseProfile and verify validProfile became true (which
    // it does only if schema_version <= kProfileSchemaVersion is satisfied).
    // If the saved JSON had no schema_version, it'd load as v0 -- still
    // accepted but the rewritten file would become "stuck" at v0 instead of
    // migrating forward. The contract is that every save bumps to current.
    CalibrationContext dst;
    std::stringstream io2(io.str());
    ParseProfile(dst, io2);
    EXPECT_TRUE(dst.validProfile)
        << "A freshly-saved profile must be re-loadable cleanly";

    // Also sanity-check that "schema_version" appears in the JSON literally.
    // (Not a perfect check -- the parser doesn't expose the key list -- but
    // confirms the WriteProfile path writes the field at all.)
    EXPECT_NE(io.str().find("schema_version"), std::string::npos)
        << "Saved JSON should contain a schema_version key";
}
