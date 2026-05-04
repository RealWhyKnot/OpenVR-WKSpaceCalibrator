#include "SkeletalHookInjector.h"
#include "Hooking.h"
#include "InterfaceHookInjector.h"   // InterfaceHooks::DetourScope — bracket
                                     // each detour body so DisableHooks can
                                     // drain in-flight callers before the DLL
                                     // is unmapped on driver unload.
#include "Logging.h"
#include "ServerTrackedDeviceProvider.h"

#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <mutex>
#include <unordered_map>

#define WIN32_LEAN_AND_MEAN
#include <windows.h>

// =============================================================================
// File-scope state.
// =============================================================================

// Cached driver pointer; set from skeletal::Init(), read on every UpdateSkeleton
// detour call. Matches SC's existing InterfaceHookInjector.cpp pattern (file-
// scope raw pointer, set before any hook can fire). Cleared by Shutdown but
// only after IHook::DestroyAll has removed our hooks, so no in-flight detour
// can race the clear.
static ServerTrackedDeviceProvider *g_driver = nullptr;

// Per-hand smoothing state. Index 0 = left, 1 = right. The "previous" buffer
// is the last bone array we wrote out for that hand — the next frame's slerp
// target is the raw incoming pose, with the previous as the source.
struct HandState
{
    bool                  initialized = false;
    vr::VRBoneTransform_t previous[31] = {};
};
static HandState g_handState[2];

// VRInputComponentHandle_t -> handedness (0=left, 1=right). Populated by the
// CreateSkeletonComponent hook by inspecting `pchSkeletonPath` (which is
// "/skeleton/hand/left" or "/skeleton/hand/right" for Index Knuckles). Handles
// not in this map are passed through unsmoothed — covers any non-Index
// skeletal device whose path doesn't match.
static std::unordered_map<vr::VRInputComponentHandle_t, int> g_handleToHandedness;

// Single mutex protects g_handState and g_handleToHandedness. The hot path
// is one acquire+release per UpdateSkeletonComponent call (~340 Hz/hand);
// the writer (CreateSkeletonComponent) fires twice per session. Single mutex
// keeps the design obvious; if profiling later shows contention we can split.
static std::mutex g_skeletalMutex;

// =============================================================================
// Diagnostic counters. UpdateSkeletonComponent fires ~340 Hz/hand, so a per-
// call LOG would balloon the log file. Instead, we tally outcomes in atomics
// and dump a summary every kStatsLogIntervalSec seconds whenever an
// UpdateSkeleton call lands. The first frame on each hand also gets a one-
// time "first call seen" log so the user can verify the hook is alive end-
// to-end after enabling smoothing in the overlay.
//
// This is the diagnostic surface the user asked for: when finger smoothing
// "doesn't appear to work", these counters answer the three failure modes:
//   1. zero left/right total           -> hook never sees frames
//   2. nonzero total but zero smoothed -> config never reaches driver, or
//                                         master_enabled false / smoothness 0
//   3. nonzero unknown_handle          -> handedness map missed, finger paths
//                                         don't match "/left" or "/right"
// =============================================================================
struct PerHandStats
{
    std::atomic<uint64_t> totalCalls{0};
    std::atomic<uint64_t> smoothedCalls{0};
    std::atomic<uint64_t> passthroughCalls{0};
    std::atomic<bool>     firstCallLogged{false};
};
static PerHandStats          g_stats[2];
static std::atomic<uint64_t> g_unknownHandleCalls{0};
static std::atomic<uint64_t> g_invalidTransformCalls{0};
static std::atomic<int64_t>  g_lastStatsLogQpc{0};
static std::atomic<int64_t>  g_lastDeepStateLogQpc{0};
static LARGE_INTEGER         g_qpcFreq{};
static constexpr double      kStatsLogIntervalSec = 30.0;
static constexpr double      kDeepStateLogIntervalSec = 60.0;

// First-N-calls verbose logging. After install succeeds (or any UpdateSkeleton
// arrives), the first kVerboseFirstCalls calls per hand emit a full parameter
// dump including a sample of the bone array. Beyond that we fall back to the
// throttled stats summary. Catches: incorrect bone-count/layout assumptions,
// detour seeing garbage params (= MinHook patched the wrong target), wrong
// handedness lookup propagating into the smoothing buffer.
static constexpr int         kVerboseFirstCalls = 3;
static std::atomic<int>      g_verboseCallsRemaining[2] = {{kVerboseFirstCalls}, {kVerboseFirstCalls}};

// Track first-time unknown-handle for an extra-detailed log. After the first
// unknown handle hits, subsequent ones just bump the counter (already done).
// The first one gets a snapshot of the entire g_handleToHandedness map so we
// can see what HANDLES we DO have mapped vs what we're being asked about.
static std::atomic<bool>     g_firstUnknownHandleLogged{false};

// Also log the first CreateSkeleton call regardless of whether the path
// matches /left or /right, so we can see if there are paths we're not
// recognising. Beyond that, only matched paths get logged (existing behavior).
static std::atomic<bool>     g_firstCreateSkeletonLogged{false};

static void MaybeLogStats(const char *callerTag)
{
    if (g_qpcFreq.QuadPart == 0) return;
    LARGE_INTEGER now;
    QueryPerformanceCounter(&now);
    int64_t last = g_lastStatsLogQpc.load(std::memory_order_relaxed);
    if (last == 0) {
        // First call: just seed the timestamp so the next log fires after a
        // full window rather than immediately.
        g_lastStatsLogQpc.compare_exchange_strong(last, now.QuadPart);
        return;
    }
    double elapsedSec = (double)(now.QuadPart - last) / (double)g_qpcFreq.QuadPart;
    if (elapsedSec < kStatsLogIntervalSec) return;
    if (!g_lastStatsLogQpc.compare_exchange_strong(last, now.QuadPart)) return;

    uint64_t l_total  = g_stats[0].totalCalls.load();
    uint64_t l_smooth = g_stats[0].smoothedCalls.load();
    uint64_t l_pass   = g_stats[0].passthroughCalls.load();
    uint64_t r_total  = g_stats[1].totalCalls.load();
    uint64_t r_smooth = g_stats[1].smoothedCalls.load();
    uint64_t r_pass   = g_stats[1].passthroughCalls.load();
    uint64_t unknown  = g_unknownHandleCalls.load();
    uint64_t invalid  = g_invalidTransformCalls.load();

    LOG("[skeletal] stats(%s, %.1fs window) L:%llu(s%llu/p%llu) R:%llu(s%llu/p%llu) unknown_handle=%llu invalid_transforms=%llu",
        callerTag, elapsedSec,
        (unsigned long long)l_total, (unsigned long long)l_smooth, (unsigned long long)l_pass,
        (unsigned long long)r_total, (unsigned long long)r_smooth, (unsigned long long)r_pass,
        (unsigned long long)unknown, (unsigned long long)invalid);
}

// Forward-declare for use in MaybeLogDeepState below — defined later in this
// file once we have access to the driver class member.
static void DumpHandleMap(const char *callerTag);

// Comprehensive periodic dump every kDeepStateLogIntervalSec. Includes:
// - Hook registry status (Create/Update hooks present)
// - Current finger config snapshot (driver-side cache)
// - All known handle->handedness mappings
// - Per-hand init state (have we seen a first frame?)
// - Stats snapshot (same numbers as the 30s stats line, with derived rates)
// Fires from the UpdateSkeleton hot path so it only runs when traffic is
// flowing — silent during driver idle.
static void MaybeLogDeepState(const char *callerTag)
{
    if (g_qpcFreq.QuadPart == 0) return;
    LARGE_INTEGER now;
    QueryPerformanceCounter(&now);
    int64_t last = g_lastDeepStateLogQpc.load(std::memory_order_relaxed);
    if (last == 0) {
        g_lastDeepStateLogQpc.compare_exchange_strong(last, now.QuadPart);
        return;
    }
    double elapsedSec = (double)(now.QuadPart - last) / (double)g_qpcFreq.QuadPart;
    if (elapsedSec < kDeepStateLogIntervalSec) return;
    if (!g_lastDeepStateLogQpc.compare_exchange_strong(last, now.QuadPart)) return;

    uint64_t l_total  = g_stats[0].totalCalls.load();
    uint64_t l_smooth = g_stats[0].smoothedCalls.load();
    uint64_t r_total  = g_stats[1].totalCalls.load();
    uint64_t r_smooth = g_stats[1].smoothedCalls.load();
    bool l_init, r_init;
    {
        std::lock_guard<std::mutex> lk(g_skeletalMutex);
        l_init = g_handState[0].initialized;
        r_init = g_handState[1].initialized;
    }
    double l_hz = (double)l_total / elapsedSec;
    double r_hz = (double)r_total / elapsedSec;

    LOG("[skeletal] deep_state(%s, %.1fs window):", callerTag, elapsedSec);
    LOG("[skeletal]   verboseRemaining=L%d/R%d",
        g_verboseCallsRemaining[0].load(), g_verboseCallsRemaining[1].load());

    // Hook registry snapshot — confirm both hooks are still present (they
    // shouldn't disappear, but a SteamVR-side reload could in theory drop
    // them; this catches that case). Names are duplicated string literals
    // here so MaybeLogDeepState can be defined above the Hook<> static
    // declarations without forward-declaration gymnastics; a single refactor
    // would have to update both sites if the names ever change.
    LOG("[skeletal]   hooks: createInRegistry=%d updateInRegistry=%d",
        (int)IHook::Exists("IVRDriverInput::CreateSkeletonComponent"),
        (int)IHook::Exists("IVRDriverInput::UpdateSkeletonComponent"));

    // Current driver-side finger config snapshot. If this disagrees with what
    // the overlay last sent (visible in the SetFingerSmoothingConfig log),
    // there's an IPC sync bug.
    if (g_driver) {
        auto cfg = g_driver->GetFingerSmoothingConfig();
        LOG("[skeletal]   cfg: enabled=%d smoothness=%u mask=0x%04x",
            (int)cfg.master_enabled, (unsigned)cfg.smoothness, (unsigned)cfg.finger_mask);
    } else {
        LOG("[skeletal]   cfg: g_driver is NULL (subsystem un-Init'd?)");
    }

    LOG("[skeletal]   per_hand: L{init=%d total=%llu(%.1fHz) smoothed=%llu} R{init=%d total=%llu(%.1fHz) smoothed=%llu}",
        (int)l_init, (unsigned long long)l_total, l_hz, (unsigned long long)l_smooth,
        (int)r_init, (unsigned long long)r_total, r_hz, (unsigned long long)r_smooth);

    DumpHandleMap("deep_state");
}

// Dump the entire handle->handedness map. Defined as a separate function so
// it can be called from multiple diagnostic sites (deep_state, first-unknown-
// handle log).
static void DumpHandleMap(const char *callerTag)
{
    std::lock_guard<std::mutex> lk(g_skeletalMutex);
    if (g_handleToHandedness.empty()) {
        LOG("[skeletal]   handle_map(%s): EMPTY -- CreateSkeleton never matched /left or /right",
            callerTag);
        return;
    }
    LOG("[skeletal]   handle_map(%s): %zu entries:", callerTag, g_handleToHandedness.size());
    for (const auto& kv : g_handleToHandedness) {
        LOG("[skeletal]     handle=%llu -> %s",
            (unsigned long long)kv.first,
            kv.second == 0 ? "left" : (kv.second == 1 ? "right" : "?"));
    }
}

// =============================================================================
// Smoothing math. Per-bone slerp toward incoming, with linear interpolation
// for position. Independent per-bone smoothing (not curl/splay derive +
// reconstruct) — this is the first-iteration math; we may promote to summary-
// scalar smoothing later. Quaternion validity is preserved (slerp + normalize).
// =============================================================================

static inline float Lerpf(float a, float b, float t)
{
    return a + (b - a) * t;
}

static vr::HmdQuaternionf_t SlerpQuat(const vr::HmdQuaternionf_t &a, vr::HmdQuaternionf_t b, float t)
{
    float dot = a.x*b.x + a.y*b.y + a.z*b.z + a.w*b.w;
    // Take the shorter arc.
    if (dot < 0.0f) {
        b.x = -b.x; b.y = -b.y; b.z = -b.z; b.w = -b.w;
        dot = -dot;
    }
    // Nearly aligned: linear blend is numerically safer than acos at small angles.
    if (dot > 0.9995f) {
        vr::HmdQuaternionf_t r{
            Lerpf(a.w, b.w, t),
            Lerpf(a.x, b.x, t),
            Lerpf(a.y, b.y, t),
            Lerpf(a.z, b.z, t)
        };
        float len = std::sqrt(r.w*r.w + r.x*r.x + r.y*r.y + r.z*r.z);
        if (len > 0.0f) {
            float inv = 1.0f / len;
            r.w *= inv; r.x *= inv; r.y *= inv; r.z *= inv;
        }
        return r;
    }
    float theta_0     = std::acos(dot);
    float sin_theta_0 = std::sin(theta_0);
    float theta       = theta_0 * t;
    float sin_theta   = std::sin(theta);
    float s1 = std::cos(theta) - dot * sin_theta / sin_theta_0;
    float s2 = sin_theta / sin_theta_0;
    return vr::HmdQuaternionf_t{
        s1*a.w + s2*b.w,
        s1*a.x + s2*b.x,
        s1*a.y + s2*b.y,
        s1*a.z + s2*b.z
    };
}

// OpenVR hand skeleton bone layout (vr::HandSkeletonBone enum):
//   0 root, 1 wrist, 2-5 thumb, 6-10 index, 11-15 middle,
//   16-20 ring, 21-25 pinky, 26-30 aux markers.
// Returns 0..4 (thumb..pinky) for finger bones, -1 otherwise.
static int FingerIndexForBone(uint32_t bone)
{
    if (bone >= 2  && bone <= 5)  return 0;
    if (bone >= 6  && bone <= 10) return 1;
    if (bone >= 11 && bone <= 15) return 2;
    if (bone >= 16 && bone <= 20) return 3;
    if (bone >= 21 && bone <= 25) return 4;
    return -1;
}

// Determine whether bone `bone` on hand `handedness` (0=left, 1=right) should
// be smoothed given the per-finger mask. Non-finger bones (root, wrist, aux)
// always pass through.
static bool ShouldSmoothBone(uint32_t bone, int handedness, uint16_t mask)
{
    int finger = FingerIndexForBone(bone);
    if (finger < 0) return false;
    int bit = handedness * 5 + finger;
    return ((mask >> bit) & 1) != 0;
}

// User-visible smoothness 0..100 -> slerp factor 1.0..0.05.
//   0   -> alpha 1.0   (raw passthrough; current frame snaps)
//   100 -> alpha 0.05  (heavy smoothing; never fully freezes)
// Intermediate values lerp linearly. The 0.95 multiplier preserves a tiny
// per-frame nudge at maximum smoothness so a finger never visually locks.
static float SmoothnessToAlpha(uint8_t smoothness)
{
    float s = (float)smoothness;
    if (s < 0.0f)   s = 0.0f;
    if (s > 100.0f) s = 100.0f;
    return 1.0f - (s / 100.0f) * 0.95f;
}

// =============================================================================
// VirtualQuery-guarded memory probes. Used to safely walk the iface pointer
// before patching its vtable. Faulting on a stray vtable read would crash
// vrserver — these probes turn that into a logged-and-bailed soft failure
// instead. Used by TryInstallPublicHooks below.
// =============================================================================

static bool IsAddressReadable(const void *addr, size_t bytes)
{
    if (!addr) return false;
    MEMORY_BASIC_INFORMATION mbi{};
    if (!VirtualQuery(addr, &mbi, sizeof mbi)) return false;
    if (mbi.State != MEM_COMMIT) return false;
    DWORD prot = mbi.Protect & 0xFF;
    if (prot == 0 || (prot & PAGE_NOACCESS) || (prot & PAGE_GUARD)) return false;
    auto regionEnd = (uintptr_t)mbi.BaseAddress + mbi.RegionSize;
    auto needEnd   = (uintptr_t)addr + bytes;
    return needEnd <= regionEnd;
}

// Log a one-line region descriptor: base, size, state, protection, type. Used
// post-install to confirm that the patched vtable slots actually point into
// vrserver's loaded image (.text section, MEM_IMAGE) rather than something
// non-executable. If we ever see slot 5 or slot 6 land in a heap or mapped
// region, the iface we hooked wasn't a real interface and the hook patched
// the wrong target.
static void LogVirtualQueryRegion(const char *tag, const void *addr)
{
    if (!addr) {
        LOG("[skeletal-probe] %s: addr=NULL", tag);
        return;
    }
    MEMORY_BASIC_INFORMATION mbi{};
    if (!VirtualQuery(addr, &mbi, sizeof mbi)) {
        LOG("[skeletal-probe] %s: addr=%p VirtualQuery FAILED (err=%lu)", tag, addr, GetLastError());
        return;
    }
    LOG("[skeletal-probe] %s: addr=%p base=%p size=0x%llx state=0x%lx prot=0x%lx type=0x%lx",
        tag, addr, mbi.BaseAddress, (unsigned long long)mbi.RegionSize,
        mbi.State, mbi.Protect, mbi.Type);
}

// =============================================================================
// Hook<> instances. Names must be unique against everything else registered
// in IHook::hooks (see SC's existing InterfaceHookInjector.cpp). Two slots
// only — UpdateSkeleton (the smoothing target) and CreateSkeleton (used to
// learn which handle is left vs right). The other 5 IVRDriverInput methods
// are intentionally NOT hooked to keep the surface minimal.
// =============================================================================

static Hook<vr::EVRInputError(*)(vr::IVRDriverInput *, vr::VRInputComponentHandle_t, vr::EVRSkeletalMotionRange, const vr::VRBoneTransform_t *, uint32_t)>
    PublicUpdateSkeletonHook("IVRDriverInput::UpdateSkeletonComponent");
static Hook<vr::EVRInputError(*)(vr::IVRDriverInput *, vr::PropertyContainerHandle_t, const char *, const char *, const char *, vr::EVRSkeletalTrackingLevel, const vr::VRBoneTransform_t *, uint32_t, vr::VRInputComponentHandle_t *)>
    PublicCreateSkeletonHook("IVRDriverInput::CreateSkeletonComponent");

// =============================================================================
// Detours.
// =============================================================================

static vr::EVRInputError DetourPublicCreateSkeletonComponent(
    vr::IVRDriverInput *_this,
    vr::PropertyContainerHandle_t ulContainer,
    const char *pchName,
    const char *pchSkeletonPath,
    const char *pchBasePosePath,
    vr::EVRSkeletalTrackingLevel eSkeletalTrackingLevel,
    const vr::VRBoneTransform_t *pGripLimitTransforms,
    uint32_t unGripLimitTransformCount,
    vr::VRInputComponentHandle_t *pHandle)
{
    InterfaceHooks::DetourScope _scope;
    auto result = PublicCreateSkeletonHook.originalFunc(
        _this, ulContainer, pchName, pchSkeletonPath, pchBasePosePath,
        eSkeletalTrackingLevel, pGripLimitTransforms, unGripLimitTransformCount, pHandle);

    // First-CreateSkeleton-ever log: regardless of left/right match, dump
    // every parameter. Catches the case where the path uses an unexpected
    // form (not "/left" or "/right" — maybe "/hand/L", "/skeleton/left_hand",
    // or some other variant we haven't seen) and our handedness map ends up
    // empty as a result.
    bool firstCreateExpected = false;
    if (g_firstCreateSkeletonLogged.compare_exchange_strong(firstCreateExpected, true)) {
        LOG("[skeletal] FIRST CreateSkeleton call: result=%d this=%p container=%llu name='%s' path='%s' basePosePath='%s' level=%d gripCount=%u outHandle=%llu",
            (int)result, (void*)_this,
            (unsigned long long)ulContainer,
            pchName ? pchName : "(null)",
            pchSkeletonPath ? pchSkeletonPath : "(null)",
            pchBasePosePath ? pchBasePosePath : "(null)",
            (int)eSkeletalTrackingLevel,
            unGripLimitTransformCount,
            pHandle ? (unsigned long long)*pHandle : 0ULL);
    }

    // Only learn handedness when the underlying create succeeded and we got
    // a valid handle + a recognizable left/right path. Anything else is a
    // skeletal device we don't smooth.
    if (result == vr::VRInputError_None
        && pHandle && *pHandle != vr::k_ulInvalidInputComponentHandle
        && pchSkeletonPath)
    {
        int handedness = -1;
        if (std::strstr(pchSkeletonPath, "/left"))       handedness = 0;
        else if (std::strstr(pchSkeletonPath, "/right")) handedness = 1;
        if (handedness >= 0) {
            std::lock_guard<std::mutex> lk(g_skeletalMutex);
            g_handleToHandedness[*pHandle] = handedness;
            // A re-create for the same hand needs to re-seed from incoming.
            // Mark uninitialised so the first UpdateSkeleton after this snaps
            // to the new pose instead of slerp-blending out of stale state.
            g_handState[handedness].initialized = false;
            // Reset the verbose-call counter for this hand so the user sees
            // detailed dumps after a re-create (driver reload, hand reconnect).
            g_verboseCallsRemaining[handedness].store(kVerboseFirstCalls);
            LOG("[skeletal] CreateSkeleton MAPPED handle=%llu -> %s (path=%s name=%s map_size_now=%zu)",
                (unsigned long long)*pHandle,
                handedness == 0 ? "left" : "right",
                pchSkeletonPath,
                pchName ? pchName : "(null)",
                g_handleToHandedness.size());
        } else {
            // Path didn't match left/right — log so we can see what other
            // skeletal components are being created (e.g., non-Index trackers
            // with skeletal output, custom drivers).
            LOG("[skeletal] CreateSkeleton SKIPPED (no /left or /right in path): handle=%llu path='%s' name='%s'",
                (unsigned long long)*pHandle,
                pchSkeletonPath,
                pchName ? pchName : "(null)");
        }
    } else if (result != vr::VRInputError_None) {
        LOG("[skeletal] CreateSkeleton FAILED: result=%d path='%s'",
            (int)result, pchSkeletonPath ? pchSkeletonPath : "(null)");
    }
    return result;
}

static vr::EVRInputError DetourPublicUpdateSkeletonComponent(
    vr::IVRDriverInput *_this,
    vr::VRInputComponentHandle_t ulComponent,
    vr::EVRSkeletalMotionRange eMotionRange,
    const vr::VRBoneTransform_t *pTransforms,
    uint32_t unTransformCount)
{
    InterfaceHooks::DetourScope _scope;
    // Fast passthrough: feature-off, unrecognised inputs, or no driver pointer.
    // This is the dominant code path when finger smoothing isn't enabled.
    if (!g_driver || !pTransforms || unTransformCount != 31) {
        if (!pTransforms || unTransformCount != 31) {
            g_invalidTransformCalls.fetch_add(1, std::memory_order_relaxed);
        }
        return PublicUpdateSkeletonHook.originalFunc(_this, ulComponent, eMotionRange, pTransforms, unTransformCount);
    }
    auto cfg = g_driver->GetFingerSmoothingConfig();

    // Look up which hand this is. Unknown handles (non-Index skeletal devices)
    // are counted but NOT smoothed — they fast-path through. We do this lookup
    // before the master_enabled gate so the per-hand stats counters give a
    // meaningful "frames seen per hand" reading even when the feature is off,
    // which is the diagnostic the user needs to know whether the hook is
    // actually receiving the lighthouse skeleton stream.
    int handedness = -1;
    {
        std::lock_guard<std::mutex> lk(g_skeletalMutex);
        auto it = g_handleToHandedness.find(ulComponent);
        if (it != g_handleToHandedness.end()) handedness = it->second;
    }
    if (handedness < 0) {
        g_unknownHandleCalls.fetch_add(1, std::memory_order_relaxed);

        // First-time unknown-handle deep log. Snapshots the entire handle
        // map so we can compare what handle the lighthouse driver is asking
        // about vs what handles we DID see come through CreateSkeleton.
        // Mismatch tells us our hook missed the CreateSkeleton call (install
        // happened too late) or the path didn't match left/right.
        bool expectedFirstUnknown = false;
        if (g_firstUnknownHandleLogged.compare_exchange_strong(expectedFirstUnknown, true)) {
            LOG("[skeletal] FIRST unknown-handle UpdateSkeleton: requested handle=%llu count=%u motionRange=%d (this hand was never seen by CreateSkeleton OR path didn't match /left|/right)",
                (unsigned long long)ulComponent, unTransformCount, (int)eMotionRange);
            DumpHandleMap("first_unknown");
        }

        // Without this, an "all unknown handles" failure (CreateSkeleton hook
        // missed the lighthouse skeleton creation, so the handedness map is
        // empty) would never log stats — there'd be no other path to MaybeLogStats
        // and the user would see silence in the log instead of the diagnostic
        // that points at the real problem (unknown_handle != 0).
        MaybeLogStats("UpdateSkeleton/unknown");
        return PublicUpdateSkeletonHook.originalFunc(_this, ulComponent, eMotionRange, pTransforms, unTransformCount);
    }

    g_stats[handedness].totalCalls.fetch_add(1, std::memory_order_relaxed);

    // First call per hand: emit a one-time "we're alive" log so the user can
    // confirm in the driver log that the hook is actually receiving frames
    // for each hand. Includes the live config snapshot so the log shows what
    // state the feature is in at the moment the first frame arrived.
    bool expected = false;
    if (g_stats[handedness].firstCallLogged.compare_exchange_strong(expected, true)) {
        LOG("[skeletal] first UpdateSkeleton on %s hand: handle=%llu count=%u motionRange=%d cfg{enabled=%d smoothness=%u mask=0x%04x}",
            handedness == 0 ? "left" : "right",
            (unsigned long long)ulComponent, unTransformCount, (int)eMotionRange,
            (int)cfg.master_enabled, (unsigned)cfg.smoothness, (unsigned)cfg.finger_mask);
    }

    // Verbose first-N-calls dump (per hand). After CreateSkeleton mapped this
    // handle (or after install) the counter is set to kVerboseFirstCalls; each
    // call decrements + dumps. After the count exhausts we go silent (the
    // throttled stats line is sufficient for steady-state). Dumps a small
    // sample of bone positions so we can sanity-check the array isn't garbage
    // (root + wrist should be near-identity; thumb metacarpal should be a
    // small offset). Garbage in pTransforms = our detour is patched at the
    // wrong target.
    int verboseRem = g_verboseCallsRemaining[handedness].fetch_sub(1, std::memory_order_relaxed);
    if (verboseRem > 0 && pTransforms) {
        const auto& bone0 = pTransforms[0];
        const auto& bone1 = pTransforms[1];
        const auto& bone2 = pTransforms[2];
        LOG("[skeletal] verbose %s call %d/%d: handle=%llu count=%u motion=%d this=%p bones[0..2]={pos=(%.4f,%.4f,%.4f) (%.4f,%.4f,%.4f) (%.4f,%.4f,%.4f) rot[0]=(%.3f,%.3f,%.3f,%.3f)}",
            handedness == 0 ? "L" : "R",
            kVerboseFirstCalls - verboseRem + 1, kVerboseFirstCalls,
            (unsigned long long)ulComponent, unTransformCount, (int)eMotionRange, (void*)_this,
            bone0.position.v[0], bone0.position.v[1], bone0.position.v[2],
            bone1.position.v[0], bone1.position.v[1], bone1.position.v[2],
            bone2.position.v[0], bone2.position.v[1], bone2.position.v[2],
            bone0.orientation.w, bone0.orientation.x, bone0.orientation.y, bone0.orientation.z);
    }

    // Periodic deep-state dump. Cheap when not firing (atomic load + arithmetic),
    // ~150 chars when it does. Runs from the hot path so it only fires when
    // skeleton traffic is actually flowing — silent when driver is idle.
    MaybeLogDeepState("UpdateSkeleton");

    if (!cfg.master_enabled || cfg.smoothness == 0) {
        g_stats[handedness].passthroughCalls.fetch_add(1, std::memory_order_relaxed);
        MaybeLogStats("UpdateSkeleton");
        return PublicUpdateSkeletonHook.originalFunc(_this, ulComponent, eMotionRange, pTransforms, unTransformCount);
    }

    float alpha = SmoothnessToAlpha(cfg.smoothness);
    vr::VRBoneTransform_t smoothed[31];

    {
        std::lock_guard<std::mutex> lk(g_skeletalMutex);
        HandState &state = g_handState[handedness];
        if (!state.initialized) {
            // First frame for this hand: seed previous-state from the incoming
            // pose and forward unmodified. Avoids a visible "snap from
            // identity quaternion" on the first call after CreateSkeleton.
            std::memcpy(state.previous, pTransforms, sizeof(state.previous));
            std::memcpy(smoothed,        pTransforms, sizeof(smoothed));
            state.initialized = true;
        } else {
            for (uint32_t i = 0; i < 31; ++i) {
                if (!ShouldSmoothBone(i, handedness, cfg.finger_mask)) {
                    smoothed[i] = pTransforms[i];
                    state.previous[i] = pTransforms[i];
                    continue;
                }
                const auto &in   = pTransforms[i];
                const auto &prev = state.previous[i];
                vr::VRBoneTransform_t out{};
                out.position.v[0] = Lerpf(prev.position.v[0], in.position.v[0], alpha);
                out.position.v[1] = Lerpf(prev.position.v[1], in.position.v[1], alpha);
                out.position.v[2] = Lerpf(prev.position.v[2], in.position.v[2], alpha);
                out.position.v[3] = in.position.v[3];
                out.orientation   = SlerpQuat(prev.orientation, in.orientation, alpha);
                smoothed[i]       = out;
                state.previous[i] = out;
            }
        }
    }

    g_stats[handedness].smoothedCalls.fetch_add(1, std::memory_order_relaxed);
    MaybeLogStats("UpdateSkeleton");
    return PublicUpdateSkeletonHook.originalFunc(_this, ulComponent, eMotionRange, smoothed, unTransformCount);
}

// =============================================================================
// Public API.
// =============================================================================

namespace skeletal {

void Init(ServerTrackedDeviceProvider *driver)
{
    g_driver = driver;
    QueryPerformanceFrequency(&g_qpcFreq);
    g_lastStatsLogQpc.store(0);
    g_lastDeepStateLogQpc.store(0);
    g_unknownHandleCalls.store(0);
    g_invalidTransformCalls.store(0);
    g_firstUnknownHandleLogged.store(false);
    g_firstCreateSkeletonLogged.store(false);
    for (int h = 0; h < 2; ++h) {
        g_stats[h].totalCalls.store(0);
        g_stats[h].smoothedCalls.store(0);
        g_stats[h].passthroughCalls.store(0);
        g_stats[h].firstCallLogged.store(false);
        g_verboseCallsRemaining[h].store(kVerboseFirstCalls);
    }
    {
        std::lock_guard<std::mutex> lk(g_skeletalMutex);
        g_handleToHandedness.clear();
        for (int h = 0; h < 2; ++h) {
            g_handState[h].initialized = false;
            std::memset(g_handState[h].previous, 0, sizeof(g_handState[h].previous));
        }
    }
    LOG("[skeletal] Init: subsystem armed (driver=%p), awaiting IVRDriverInput interface queries", (void*)driver);
}

void Shutdown()
{
    // Called after IHook::DestroyAll + InterfaceHooks::DrainInFlightDetours
    // from the existing DisableHooks(). Our detours are guaranteed to have
    // exited before we get here (drain is the previous step), so no in-flight
    // caller can race anything we do here.
    MaybeLogStats("Shutdown");
    // Force a final deep-state dump regardless of throttle so even a short-
    // lived session leaves us a full snapshot in the log. Bypass the throttle
    // by zeroing the last-log timestamp.
    g_lastDeepStateLogQpc.store(0);
    // Then bump it forward so the elapsed check passes — we want the dump,
    // not just the seed.
    LARGE_INTEGER fakeOld;
    QueryPerformanceCounter(&fakeOld);
    fakeOld.QuadPart -= (LONGLONG)(kDeepStateLogIntervalSec * (double)g_qpcFreq.QuadPart) + 1;
    g_lastDeepStateLogQpc.store(fakeOld.QuadPart);
    MaybeLogDeepState("Shutdown");
    LOG("[skeletal] Shutdown: subsystem disarmed");

    // Intentionally do NOT clear g_driver. ServerTrackedDeviceProvider
    // outlives this DLL: SteamVR holds the provider object alive across the
    // entire driver session, and Init() will overwrite g_driver on the next
    // load. Nulling it here used to crash vrserver on every reload that
    // coincided with a 340Hz UpdateSkeleton -- the detour reads g_driver
    // after the !g_driver guard and a window between guard and use let the
    // pointer go to NULL mid-call. Item 2's in-flight drain closes that
    // window, but defending the pointer itself keeps the invariant local
    // to this file in case a future detour is added without the scope
    // guard.
    {
        std::lock_guard<std::mutex> lk(g_skeletalMutex);
        g_handleToHandedness.clear();
        for (int h = 0; h < 2; ++h) g_handState[h].initialized = false;
    }
}

void TryInstallPublicHooks(void *iface)
{
    if (!iface) return;

    // Idempotent: once both hooks are registered, additional invocations
    // (one per driver context query of IVRDriverInput_*) no-op cheaply.
    // The MinHook target functions are static across all queries returning
    // the same singleton vtable in vrserver, so re-patching would be harmless
    // but we'd rather skip the work + the noisy log lines.
    bool createAlready = IHook::Exists(PublicCreateSkeletonHook.name);
    bool updateAlready = IHook::Exists(PublicUpdateSkeletonHook.name);
    if (createAlready && updateAlready) return;

    LOG("[skeletal] TryInstallPublicHooks invoked: iface=%p createInRegistry=%d updateInRegistry=%d",
        iface, (int)createAlready, (int)updateAlready);

    // Defensive readability + sanity check. A real C++ object has its vtable
    // pointer at offset 0; the vtable itself is a contiguous array of
    // function pointers in the DLL's .rdata. If the pointer SteamVR returned
    // is something else (e.g. a JsonCpp settings struct, like the
    // IVRDriverInputInternal_XXX chase encountered — see memory file), the
    // dereferenced "vtable" will not be readable for 7 slots, OR slot 0 and
    // slot 6 will be wildly different addresses (not even in the same
    // module). Both checks must pass before we patch anything.
    if (!IsAddressReadable(iface, sizeof(void *))) {
        LOG("[skeletal] iface %p not readable; aborting install", iface);
        return;
    }
    void **vtable = *((void ***)iface);
    if (!IsAddressReadable(vtable, sizeof(void *) * 7)) {
        LOG("[skeletal] vtable %p not readable for 7 slots; aborting install (iface=%p — likely garbage, e.g. settings memory)",
            (void *)vtable, iface);
        return;
    }
    // BattleAxeVR/PSVR2 shim defensive pattern: real vrserver vtables are a
    // contiguous block of function pointers, all into the same .text section
    // of vrserver.exe. Slot 0 (CreateBoolean) and slot 6 (UpdateSkeleton) are
    // virtual functions of the same C++ class; their addresses are within a
    // few hundred bytes of each other in practice. A spread of more than
    // 64 KB means either (a) the iface pointer is bogus and "vtable" points
    // at random data, or (b) the OpenVR ABI has shifted in a way we don't
    // understand and we should bail rather than patch the wrong target.
    intptr_t spread = (intptr_t)vtable[6] - (intptr_t)vtable[0];
    if (spread < 0) spread = -spread;
    if (spread > 0x10000) {
        LOG("[skeletal] vtable spread |slot6 - slot0| = 0x%llx bytes (>64KB); refusing to install (iface=%p slot0=%p slot6=%p — likely garbage)",
            (unsigned long long)spread, iface, vtable[0], vtable[6]);
        return;
    }

    // Pre-install snapshot for the post-install diff. The slot values
    // themselves typically don't change with MinHook (it patches the
    // function body, not the pointer in the vtable), so we expect post == pre
    // here. The change should be in originalFunc (which is the MinHook
    // trampoline, distinct from both the original target and our detour).
    void *preCreate = vtable[5];
    void *preUpdate = vtable[6];
    LOG("[skeletal] pre-install snapshot: vtable[5] (Create) = %p, vtable[6] (Update) = %p, spread=0x%llx",
        preCreate, preUpdate, (unsigned long long)spread);

    if (!createAlready) {
        PublicCreateSkeletonHook.CreateHookInObjectVTable(iface, 5, &DetourPublicCreateSkeletonComponent);
        IHook::Register(&PublicCreateSkeletonHook);
        LOG("[skeletal]   Create hook installed at vtable[5]: originalFunc=%p, detour=%p",
            (void *)PublicCreateSkeletonHook.originalFunc, (void *)&DetourPublicCreateSkeletonComponent);
    }
    if (!updateAlready) {
        PublicUpdateSkeletonHook.CreateHookInObjectVTable(iface, 6, &DetourPublicUpdateSkeletonComponent);
        IHook::Register(&PublicUpdateSkeletonHook);
        LOG("[skeletal]   Update hook installed at vtable[6]: originalFunc=%p, detour=%p",
            (void *)PublicUpdateSkeletonHook.originalFunc, (void *)&DetourPublicUpdateSkeletonComponent);
    }

    LogVirtualQueryRegion("public_vtable_slot5", vtable[5]);
    LogVirtualQueryRegion("public_vtable_slot6", vtable[6]);

    LOG("[skeletal] installed PUBLIC IVRDriverInput hooks: vtable[5]=Create, vtable[6]=Update -- waiting for first calls");
}

} // namespace skeletal
