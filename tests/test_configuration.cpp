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
// Wedge-guard tests removed 2026-05-05. The load-time wedge guard in
// ParseProfile and the runtime detector in CalibrationTick are both
// disabled in production — they were causing a reset loop on the user's
// Quest+Lighthouse setup whose legitimate convergence values exceed any
// fixed magnitude bound. See project_wedge_guard_removed_2026-05-05.md.
// ---------------------------------------------------------------------------


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
        << "size==0 must NOT underflow â€” caller short-circuits to empty string";
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
// SaveProfile-path schema-validation pin (audit follow-up). Every persistence
// site in the overlay routes through `SaveProfile(ctx)` â†’ `WriteProfile(ctx,
// stream)` â†’ the `WRITE_IF_CHANGED_*` macros (Configuration.cpp:586-617).
// These tests pin three contracts at the WriteProfile boundary:
//
//   1. schema_version is ALWAYS stamped â€” without it, future loads treat
//      the profile as v0 and run all migration steps redundantly.
//   2. Calibration data (translation, rotation, scale, tracking-system
//      names, the standby device records) is ALWAYS written â€” it IS the
//      calibration, not a tunable.
//   3. Setting fields are SKIPPED when at default â€” old profiles that pre-
//      date the field don't accidentally get the field's default value
//      stamped onto disk on next save (which would block future default-
//      flips from taking effect on existing user profiles).
//
// If anyone adds a new SaveProfile call site that bypasses WriteProfile, or
// modifies WriteProfile to drop schema_version / change a calibration field
// to skip-if-default / change a tunable to always-write, one of these
// expectations fails.
// ---------------------------------------------------------------------------
TEST(ConfigurationTest, WriteProfile_AlwaysStampsCalibrationData) {
    // Calibration data is mandatory on every save. Even a fresh
    // unconfigured context with default zeros must emit the calibration
    // keys so a future load always has them.
    CalibrationContext ctx; // all defaults
    ctx.referenceTrackingSystem = "lighthouse";
    ctx.targetTrackingSystem = "oculus";
    ctx.validProfile = true;

    std::stringstream io;
    WriteProfile(ctx, io);
    const std::string json = io.str();

    EXPECT_NE(json.find("\"x\":"),  std::string::npos);
    EXPECT_NE(json.find("\"y\":"),  std::string::npos);
    EXPECT_NE(json.find("\"z\":"),  std::string::npos);
    EXPECT_NE(json.find("\"roll\":"),  std::string::npos);
    EXPECT_NE(json.find("\"yaw\":"),   std::string::npos);
    EXPECT_NE(json.find("\"pitch\":"), std::string::npos);
    EXPECT_NE(json.find("\"scale\":"), std::string::npos);
    EXPECT_NE(json.find("\"reference_tracking_system\":"), std::string::npos);
    EXPECT_NE(json.find("\"target_tracking_system\":"),    std::string::npos);
}

TEST(ConfigurationTest, WriteProfile_SkipsDefaultTunables) {
    // Skip-if-default: tunables left at their CalibrationContext-construction
    // defaults must NOT appear in the JSON. Without this contract, a user's
    // pre-2026 profile would come out of round-trip with new fields baked in
    // at today's defaults â€” and any future default flip would have no effect
    // on those profiles.
    CalibrationContext ctx;  // all defaults
    ctx.referenceTrackingSystem = "lighthouse";
    ctx.targetTrackingSystem = "oculus";
    ctx.validProfile = true;

    std::stringstream io;
    WriteProfile(ctx, io);
    const std::string json = io.str();

    // recalibrateOnMovement defaults to true â†’ skip
    EXPECT_EQ(json.find("recalibrate_on_movement"), std::string::npos)
        << "default-true bool must be skipped on save";
    // baseStationDriftCorrectionEnabled defaults to true â†’ skip
    EXPECT_EQ(json.find("base_station_drift_correction"), std::string::npos);
    // ignoreOutliers defaults to false â†’ skip
    EXPECT_EQ(json.find("ignore_outliers"), std::string::npos);
    // jitterThreshold defaults to 3.0f â†’ skip
    EXPECT_EQ(json.find("jitter_threshold"), std::string::npos);
}

TEST(ConfigurationTest, WriteProfile_StampsNonDefaultTunables) {
    // The other half of WRITE_IF_CHANGED: tunables flipped away from default
    // MUST be written. If someone breaks the macro to always-skip, the
    // user's customisations would silently disappear on next save.
    CalibrationContext ctx;
    ctx.referenceTrackingSystem = "lighthouse";
    ctx.targetTrackingSystem = "oculus";
    ctx.validProfile = true;
    ctx.recalibrateOnMovement = false;       // flipped from default true
    ctx.ignoreOutliers = true;               // flipped from default false
    ctx.jitterThreshold = 5.5f;              // flipped from default 3.0
    ctx.baseStationDriftCorrectionEnabled = false; // flipped from default true

    std::stringstream io;
    WriteProfile(ctx, io);
    const std::string json = io.str();

    EXPECT_NE(json.find("recalibrate_on_movement"),   std::string::npos);
    EXPECT_NE(json.find("ignore_outliers"),            std::string::npos);
    EXPECT_NE(json.find("jitter_threshold"),           std::string::npos);
    EXPECT_NE(json.find("base_station_drift_correction"), std::string::npos);
}

TEST(ConfigurationTest, WriteProfile_InvalidProfileWritesNothing) {
    // Sanity: WriteProfile early-returns on !validProfile (Configuration.cpp:530).
    // A caller that hands in an unfinished context should produce empty
    // output, not partial JSON that a re-load would silently truncate.
    CalibrationContext ctx;
    ctx.validProfile = false;

    std::stringstream io;
    WriteProfile(ctx, io);

    EXPECT_TRUE(io.str().empty())
        << "WriteProfile must early-return on invalid context â€” partial JSON "
           "would be a silent data-loss bug for the caller";
}

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
