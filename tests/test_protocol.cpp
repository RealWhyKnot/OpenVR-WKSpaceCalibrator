// Tests for the IPC protocol structures shared between overlay and driver.
//
// These are pure-data assertions: the protocol's behavioural correctness is
// covered by the live IPC handshake and end-to-end testing, but having the
// version constant + struct sizes pinned by tests catches accidental on-wire
// breakage at PR time -- not at "I started SteamVR and the driver refuses
// to handshake" time.
//
// The whole header is included as-is; the test binary doesn't link against
// the IPC client/server, just the layout definitions.

#include <gtest/gtest.h>

#include <cstring>
#include <openvr.h>
#include "Protocol.h"

// ---------------------------------------------------------------------------
// Pin the protocol version. If you legitimately bump it (added/changed an
// IPC field), update this number AND add a row to Driver-Protocol.md's
// version table. The point of the test is to make the wire-protocol bump
// require a deliberate two-touch acknowledgement, not to gate releases.
// ---------------------------------------------------------------------------
TEST(ProtocolTest, VersionPinnedToCurrent) {
    EXPECT_EQ(protocol::Version, 14u)
        << "Protocol version changed without updating the test pin. If this is "
           "intentional: bump the literal here and add a row to wiki/Driver-"
           "Protocol.md describing the new version's wire-format changes.";
}

// ---------------------------------------------------------------------------
// MaxTrackingSystemNameLen is the size of the fixed-length char buffer that
// carries tracking-system identifiers ("lighthouse", "oculus", etc.) across
// the IPC boundary. Pin it so any change has to be deliberate -- shrinking
// it would silently truncate "Pimax Crystal HMD" and friends.
// ---------------------------------------------------------------------------
TEST(ProtocolTest, MaxTrackingSystemNameLenIs32) {
    EXPECT_EQ(protocol::MaxTrackingSystemNameLen, 32u);
}

// ---------------------------------------------------------------------------
// SetDeviceTransform default constructor: all-disabled, identity rotation,
// scale=1, no per-system tag, no smoothness, no movement gating. This is the
// "blank slate" payload the overlay starts from before deciding which fields
// to override -- a regression in the defaults silently leaks unintended
// state to the driver.
// ---------------------------------------------------------------------------
TEST(ProtocolTest, SetDeviceTransformDefaults) {
    protocol::SetDeviceTransform t(/*id=*/42u, /*enabled=*/false);
    EXPECT_EQ(t.openVRID, 42u);
    EXPECT_FALSE(t.enabled);
    EXPECT_FALSE(t.updateTranslation);
    EXPECT_FALSE(t.updateRotation);
    EXPECT_FALSE(t.updateScale);
    EXPECT_FALSE(t.lerp);
    EXPECT_FALSE(t.quash);
    EXPECT_DOUBLE_EQ(t.scale, 1.0);
    // Identity quaternion (w=1, x=y=z=0).
    EXPECT_DOUBLE_EQ(t.rotation.w, 1.0);
    EXPECT_DOUBLE_EQ(t.rotation.x, 0.0);
    EXPECT_DOUBLE_EQ(t.rotation.y, 0.0);
    EXPECT_DOUBLE_EQ(t.rotation.z, 0.0);
    // Zero translation.
    EXPECT_DOUBLE_EQ(t.translation.v[0], 0.0);
    EXPECT_DOUBLE_EQ(t.translation.v[1], 0.0);
    EXPECT_DOUBLE_EQ(t.translation.v[2], 0.0);
    // Empty target_system.
    EXPECT_EQ(t.target_system[0], '\0');
    EXPECT_EQ(t.predictionSmoothness, 0u);
    EXPECT_FALSE(t.recalibrateOnMovement);
}

// ---------------------------------------------------------------------------
// SetDeviceTransform translation+rotation overload. Verifies both fields
// land where they should and the corresponding update flags get set; the
// other fields stay at defaults.
// ---------------------------------------------------------------------------
TEST(ProtocolTest, SetDeviceTransformTransRotConstructor) {
    vr::HmdVector3d_t v{ 0.5, 1.0, -1.5 };
    vr::HmdQuaternion_t q{ 0.7071, 0.0, 0.7071, 0.0 };
    protocol::SetDeviceTransform t(/*id=*/7u, /*enabled=*/true, v, q);

    EXPECT_TRUE(t.updateTranslation);
    EXPECT_TRUE(t.updateRotation);
    EXPECT_FALSE(t.updateScale);
    EXPECT_DOUBLE_EQ(t.translation.v[0], 0.5);
    EXPECT_DOUBLE_EQ(t.translation.v[1], 1.0);
    EXPECT_DOUBLE_EQ(t.translation.v[2], -1.5);
    EXPECT_DOUBLE_EQ(t.rotation.w, 0.7071);
    EXPECT_DOUBLE_EQ(t.rotation.y, 0.7071);
}

// ---------------------------------------------------------------------------
// Request types are still in the expected order. Adding a new request type
// in the middle of the enum would silently shift every later integer value
// and confuse old driver builds reading new payloads (or vice versa).
// ---------------------------------------------------------------------------
TEST(ProtocolTest, RequestTypeOrdinals) {
    EXPECT_EQ((int)protocol::RequestInvalid, 0);
    EXPECT_EQ((int)protocol::RequestHandshake, 1);
    EXPECT_EQ((int)protocol::RequestSetDeviceTransform, 2);
    EXPECT_EQ((int)protocol::RequestSetAlignmentSpeedParams, 3);
    EXPECT_EQ((int)protocol::RequestDebugOffset, 4);
    EXPECT_EQ((int)protocol::RequestSetTrackingSystemFallback, 5);
    EXPECT_EQ((int)protocol::RequestSetFingerSmoothing, 6);
    EXPECT_EQ((int)protocol::RequestSetInputHealthConfig, 7);
    EXPECT_EQ((int)protocol::RequestResetInputHealthStats, 8);
}

// ---------------------------------------------------------------------------
// Response types: same regression intent.
// ---------------------------------------------------------------------------
TEST(ProtocolTest, ResponseTypeOrdinals) {
    EXPECT_EQ((int)protocol::ResponseInvalid, 0);
    EXPECT_EQ((int)protocol::ResponseHandshake, 1);
    EXPECT_EQ((int)protocol::ResponseSuccess, 2);
}

// ---------------------------------------------------------------------------
// Default Protocol struct stamps the current version. Used by the IPC
// handshake -- a mismatch here would let stale code report an old version.
// ---------------------------------------------------------------------------
TEST(ProtocolTest, ProtocolDefaultVersionMatchesConstant) {
    protocol::Protocol p;
    EXPECT_EQ(p.version, protocol::Version);
}

// ---------------------------------------------------------------------------
// SetTrackingSystemFallback: defaults zero out enable + system_name, leaving
// the driver to refuse the fallback until a real payload arrives.
// ---------------------------------------------------------------------------
TEST(ProtocolTest, SetTrackingSystemFallbackZeroInitDefault) {
    protocol::SetTrackingSystemFallback fb{};
    EXPECT_FALSE(fb.enabled);
    EXPECT_EQ(fb.system_name[0], '\0');
    EXPECT_DOUBLE_EQ(fb.scale, 0.0); // POD zero-init
}
