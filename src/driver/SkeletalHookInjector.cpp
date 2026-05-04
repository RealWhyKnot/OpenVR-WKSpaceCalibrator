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

// Recovered IVRDriverInputInternal vtable indices, populated by parsing the
// public IVRDriverInput_003 stub thunks in skeletal::CapturePublicVTable.
// The mapping turned out to be 1:1 with public method order in M1, but we
// still parse rather than hardcode in case a SteamVR update reshuffles the
// internal vtable independently of the public one.
//
// Index of -1 means "not yet known"; we refuse to install Internal hooks
// against any negative index.
static int     g_internalVTableIndex[7] = {-1, -1, -1, -1, -1, -1, -1};
static uint8_t g_publicPimplOffset      = 0;
static bool    g_publicThunksParsed     = false;

// First non-null IVRDriverInputInternal pointer we've installed against.
// Subsequent calls (additional GetGenericInterface queries during a single
// session — multiple driver contexts) point into the same static vtable
// functions in vrserver.exe; MinHook would refuse to re-patch the same target,
// so we just no-op on duplicates.
static std::atomic<void *> g_installedAgainstInternal{nullptr};

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
// - Install state (publicParsed, installedAgainstInternal, hook registry status)
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
    LOG("[skeletal]   install: publicParsed=%d installedAgainst=%p verboseRemaining=L%d/R%d",
        (int)g_publicThunksParsed, g_installedAgainstInternal.load(),
        g_verboseCallsRemaining[0].load(), g_verboseCallsRemaining[1].load());

    // Hook registry snapshot — confirm both Internal hooks are still present
    // (they shouldn't disappear, but a SteamVR-side reload could in theory
    // drop them; this catches that case). Names are duplicated string
    // literals here so MaybeLogDeepState can be defined above the Hook<>
    // static declarations without forward-declaration gymnastics; a single
    // refactor would have to update both sites if the names ever change.
    LOG("[skeletal]   hooks: createInRegistry=%d updateInRegistry=%d",
        (int)IHook::Exists("IVRDriverInputInternal::CreateSkeletonComponent"),
        (int)IHook::Exists("IVRDriverInputInternal::UpdateSkeletonComponent"));

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
// VirtualQuery-guarded memory probes. Used for safely walking unknown
// vtables and dereferencing the inner pointer recovered from the public
// pimpl. Faulting on a stray vtable read would crash vrserver — these
// probes turn that into a logged-and-bailed soft failure instead.
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

// Tighter check used by DeepProbeInterface before iterating "what we hope
// is a vtable" pointer slots. Real C++ vtables for objects from a DLL live
// in the loaded image's .rdata section -- mbi.Type == MEM_IMAGE. Mapped
// readable regions like KUSER_SHARED_DATA (0x7FFE0000), private heap pages
// holding plain data, or stray stack pages are MEM_PRIVATE / MEM_MAPPED
// and never carry a real vtable. Without this filter, a candidate that
// happened to be readable (via IsAddressReadable) but isn't a real vtable
// would have its 8 "slots" iterated and logged as if they were function
// pointers, producing 8 lines of misleading garbage in the diagnostic log
// on every failed install.
static bool IsPlausibleVTableRegion(const void *addr)
{
    if (!addr) return false;
    // Cheap obvious-bogus filter: vtable pointers below the first 64KB
    // (MM_HIGHEST_USER_ADDRESS leaves this range as a NULL trap on Windows)
    // can never be real. Catches the most common garbage-pointer case
    // without paying for a VirtualQuery syscall.
    if ((uintptr_t)addr < 0x10000) return false;
    MEMORY_BASIC_INFORMATION mbi{};
    if (!VirtualQuery(addr, &mbi, sizeof mbi)) return false;
    if (mbi.State != MEM_COMMIT) return false;
    if (mbi.Type != MEM_IMAGE)   return false;
    return true;
}

// =============================================================================
// Deep memory anatomy probe. Used when an interface install fails its
// readability check — dumps everything we know about the iface, the
// putative vtable, and the surrounding memory regions so we can diagnose
// from the log what the actual memory layout looks like. Output is
// intentionally verbose; gated by callers so it only fires on first
// failure, not every time the same iface comes through.
//
// Reveals:
//   - The full memory region the iface lives in (heap? code? stack?)
//   - The first 8 pointer slots at iface (so we can see if iface[0]
//     is actually the vtable pointer, or if the layout is different)
//   - The memory region the vtable pointer points into
//   - Per-slot readability + value for vtable[0..7]
//   - For each readable slot, a probe of the function pointer's region
//     (so we can confirm slots really do point at code)
// =============================================================================
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

static void DeepProbeInterface(const char *who, void *iface)
{
    if (!iface) {
        LOG("[skeletal-probe] %s: iface=NULL, nothing to probe", who);
        return;
    }
    LogVirtualQueryRegion(who, iface);

    // Try to read the first 8 pointer slots at iface. If iface is a
    // standard C++ vtable-having object, slot 0 is the vtable pointer
    // and slots 1+ are member fields. If iface is something else (e.g.
    // a struct of function pointers, or a thunk wrapper) the layout
    // will look different.
    if (!IsAddressReadable(iface, sizeof(void *) * 8)) {
        LOG("[skeletal-probe] %s: cannot read 8 slots at iface=%p; trying smaller reads", who, iface);
        for (int i = 0; i < 8; ++i) {
            void **slot = ((void **)iface) + i;
            if (IsAddressReadable(slot, sizeof(void *))) {
                LOG("[skeletal-probe] %s: iface[%d] readable, value=%p", who, i, *slot);
            } else {
                LOG("[skeletal-probe] %s: iface[%d] at %p NOT readable", who, i, (void*)slot);
            }
        }
        return;
    }
    void **iface_as_slots = (void **)iface;
    LOG("[skeletal-probe] %s: iface[0..7] = %p %p %p %p %p %p %p %p",
        who,
        iface_as_slots[0], iface_as_slots[1], iface_as_slots[2], iface_as_slots[3],
        iface_as_slots[4], iface_as_slots[5], iface_as_slots[6], iface_as_slots[7]);

    // Standard C++ assumption: iface[0] is the vtable pointer. Probe the
    // region it points into.
    void *vtableCandidate = iface_as_slots[0];
    LogVirtualQueryRegion("vtable_ptr_region", vtableCandidate);

    // Bail before slot iteration if vtableCandidate isn't in a plausible
    // image region. Without this, a candidate that happens to land in a
    // readable-but-not-actually-a-vtable area (KUSER_SHARED_DATA at
    // 0x7FFE0000, a heap page that happens to contain pointer-sized values,
    // etc.) would have its "slots" dereferenced and logged as if they were
    // function pointers -- producing 8 lines of misleading garbage on every
    // failed install. Real vtables live in MEM_IMAGE pages from a loaded
    // DLL's .rdata.
    if (!IsPlausibleVTableRegion(vtableCandidate)) {
        LOG("[skeletal-probe] %s: vtableCandidate=%p not in MEM_IMAGE region; skipping per-slot probe (likely not a real vtable)",
            who, vtableCandidate);
        return;
    }

    // Per-slot probe. Print whether each is readable, the slot's value
    // (which should be a function pointer if it's a real vtable), and
    // for the readable ones, the region the function pointer points into.
    for (int i = 0; i < 8; ++i) {
        void **slotAddr = ((void **)vtableCandidate) + i;
        if (!IsAddressReadable(slotAddr, sizeof(void *))) {
            LOG("[skeletal-probe] %s: vtable[%d] at %p NOT readable", who, i, (void*)slotAddr);
            continue;
        }
        void *fn = *slotAddr;
        // Check the function pointer points into committed memory too.
        bool fnReadable = IsAddressReadable(fn, 8);
        LOG("[skeletal-probe] %s: vtable[%d] at %p = %p (fn region readable=%d)",
            who, i, (void*)slotAddr, fn, (int)fnReadable);
        if (fnReadable) {
            // Print first 8 bytes of the function — useful for sanity-
            // checking that this really is code (look for opcode prefixes
            // like 48 8B... indicating x64 mov).
            const uint8_t *p = (const uint8_t *)fn;
            LOG("[skeletal-probe] %s: vtable[%d] fn first8 = %02x %02x %02x %02x %02x %02x %02x %02x",
                who, i, p[0], p[1], p[2], p[3], p[4], p[5], p[6], p[7]);
        }
    }
}

// =============================================================================
// Public-stub thunk parser. Every IVRDriverInput_003 vtable entry in vrserver
// is an 11-14 byte pimpl forwarder of the shape:
//
//   48 8B 49 NN     mov rcx, [rcx + NN]        ; NN = pimpl offset (observed = 8)
//   48 8B 01     OR 4C 8B 11   mov rax/r10, [rcx]
//   48 FF 60 MM  OR 49 FF 62 MM  jmp [rax/r10 + MM]   ; MM/8 = inner vtable index
//   (special-cased: 48 FF 20 / 49 FF 22 = jmp [reg] = inner vtable index 0)
//
// Returns {valid=true, pimplOffset, innerVtableIndex} on a clean parse, or
// valid=false if any byte doesn't match. Used at install time to recover
// the Internal vtable layout for free without a private header.
// =============================================================================

struct ThunkParse
{
    bool    valid;
    uint8_t pimplOffset;
    uint8_t innerVtableIndex;
};

static ThunkParse ParsePublicVTableThunk(const void *fn)
{
    ThunkParse out{false, 0, 0};
    if (!IsAddressReadable(fn, 16)) return out;
    auto *p = (const uint8_t *)fn;

    // mov rcx, [rcx + disp8]: 48 8B 49 NN
    if (p[0] != 0x48 || p[1] != 0x8B || p[2] != 0x49) return out;
    out.pimplOffset = p[3];
    p += 4;

    // mov rax, [rcx] (48 8B 01) or mov r10, [rcx] (4C 8B 11)
    if (p[0] == 0x48 && p[1] == 0x8B && p[2] == 0x01)      p += 3;
    else if (p[0] == 0x4C && p[1] == 0x8B && p[2] == 0x11) p += 3;
    else return out;

    if      (p[0] == 0x48 && p[1] == 0xFF && p[2] == 0x20) out.innerVtableIndex = 0;
    else if (p[0] == 0x48 && p[1] == 0xFF && p[2] == 0x60) out.innerVtableIndex = (uint8_t)(p[3] / 8);
    else if (p[0] == 0x49 && p[1] == 0xFF && p[2] == 0x22) out.innerVtableIndex = 0;
    else if (p[0] == 0x49 && p[1] == 0xFF && p[2] == 0x62) out.innerVtableIndex = (uint8_t)(p[3] / 8);
    else return out;

    out.valid = true;
    return out;
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
// Hook<> instances. Names must be unique against everything else registered
// in IHook::hooks (see SC's existing InterfaceHookInjector.cpp). Two slots
// only — UpdateSkeleton (the smoothing target) and CreateSkeleton (used to
// learn which handle is left vs right). The other 5 IVRDriverInput methods
// are intentionally NOT hooked to keep the surface minimal.
// =============================================================================

static Hook<vr::EVRInputError(*)(vr::IVRDriverInput *, vr::VRInputComponentHandle_t, vr::EVRSkeletalMotionRange, const vr::VRBoneTransform_t *, uint32_t)>
    InternalUpdateSkeletonHook("IVRDriverInputInternal::UpdateSkeletonComponent");
static Hook<vr::EVRInputError(*)(vr::IVRDriverInput *, vr::PropertyContainerHandle_t, const char *, const char *, const char *, vr::EVRSkeletalTrackingLevel, const vr::VRBoneTransform_t *, uint32_t, vr::VRInputComponentHandle_t *)>
    InternalCreateSkeletonHook("IVRDriverInputInternal::CreateSkeletonComponent");

// =============================================================================
// Detours.
// =============================================================================

static vr::EVRInputError DetourInternalCreateSkeletonComponent(
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
    auto result = InternalCreateSkeletonHook.originalFunc(
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

static vr::EVRInputError DetourInternalUpdateSkeletonComponent(
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
        return InternalUpdateSkeletonHook.originalFunc(_this, ulComponent, eMotionRange, pTransforms, unTransformCount);
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
        return InternalUpdateSkeletonHook.originalFunc(_this, ulComponent, eMotionRange, pTransforms, unTransformCount);
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
        return InternalUpdateSkeletonHook.originalFunc(_this, ulComponent, eMotionRange, pTransforms, unTransformCount);
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
    return InternalUpdateSkeletonHook.originalFunc(_this, ulComponent, eMotionRange, smoothed, unTransformCount);
}

// =============================================================================
// Public API.
// =============================================================================

namespace skeletal {

void Init(ServerTrackedDeviceProvider *driver)
{
    g_driver = driver;
    g_installedAgainstInternal.store(nullptr);
    g_publicThunksParsed = false;
    g_publicPimplOffset  = 0;
    for (int i = 0; i < 7; ++i) g_internalVTableIndex[i] = -1;
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
    LOG("[skeletal] Shutdown: subsystem disarmed (publicThunksParsed=%d, installedAgainstInternal=%p)",
        (int)g_publicThunksParsed, g_installedAgainstInternal.load());

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
    g_installedAgainstInternal.store(nullptr);
    {
        std::lock_guard<std::mutex> lk(g_skeletalMutex);
        g_handleToHandedness.clear();
        for (int h = 0; h < 2; ++h) g_handState[h].initialized = false;
    }
}

bool IsPublicVTableCaptured()
{
    return g_publicThunksParsed;
}

void CapturePublicVTable(void *iface)
{
    if (!iface) return;
    if (g_publicThunksParsed) {
        // Idempotent re-entry — common: every IVRDriverInput_003 query (one
        // per driver per session) calls in here. After the first success we
        // just no-op silently to avoid log spam.
        return;
    }
    LOG("[skeletal] CapturePublicVTable invoked: iface=%p", iface);
    // Deep-probe the public interface as a working baseline. The Internal
    // probe (in TryInstallInternalHook on failure) compares against this —
    // if their layouts look fundamentally different that tells us the
    // Internal returned by GetGenericInterface isn't a standard vtable-
    // having object.
    DeepProbeInterface("public", iface);
    if (!IsAddressReadable(iface, sizeof(void *))) {
        LOG("[skeletal] CapturePublicVTable: iface %p not readable, aborting", iface);
        return;
    }
    void **vtable = *((void ***)iface);
    if (!IsAddressReadable(vtable, sizeof(void *) * 7)) {
        LOG("[skeletal] CapturePublicVTable: vtable %p not readable for 7 slots, aborting", (void*)vtable);
        return;
    }

    ThunkParse parses[7];
    bool       allValid  = true;
    uint8_t    pimplSeen = 0;
    for (int i = 0; i < 7; ++i) {
        parses[i] = ParsePublicVTableThunk(vtable[i]);
        if (!parses[i].valid) { allValid = false; continue; }
        if (pimplSeen == 0) pimplSeen = parses[i].pimplOffset;
        else if (pimplSeen != parses[i].pimplOffset) {
            LOG("[skeletal] pimpl-offset mismatch at public method %d (got %u, expected %u); aborting parse",
                i, (unsigned)parses[i].pimplOffset, (unsigned)pimplSeen);
            allValid = false;
        }
    }
    if (!allValid || pimplSeen == 0) {
        LOG("[skeletal] public IVRDriverInput thunk parse failed; finger smoothing will not install");
        return;
    }
    for (int i = 0; i < 7; ++i) g_internalVTableIndex[i] = parses[i].innerVtableIndex;
    g_publicPimplOffset  = pimplSeen;
    g_publicThunksParsed = true;
    LOG("[skeletal] public IVRDriverInput thunks parsed: pimplOffset=%u indices={%d,%d,%d,%d,%d,%d,%d}",
        (unsigned)g_publicPimplOffset,
        g_internalVTableIndex[0], g_internalVTableIndex[1], g_internalVTableIndex[2],
        g_internalVTableIndex[3], g_internalVTableIndex[4], g_internalVTableIndex[5],
        g_internalVTableIndex[6]);
}

void TryInstallInternalHook(void *iface)
{
    if (!iface) return;

    LOG("[skeletal] TryInstallInternalHook invoked: iface=%p publicParsed=%d prevInstalled=%p",
        iface, (int)g_publicThunksParsed, g_installedAgainstInternal.load());

    // First-install only. If we've already installed against any Internal
    // pointer this session, bail — subsequent pointers map to the same
    // static vtable functions and MinHook would refuse anyway.
    void *prev = nullptr;
    if (!g_installedAgainstInternal.compare_exchange_strong(prev, iface)) {
        LOG("[skeletal] TryInstallInternalHook: already installed against %p, no-op", prev);
        return;
    }

    if (!g_publicThunksParsed) {
        LOG("[skeletal] Internal pointer arrived before public-stub thunks were parsed; install will retry on next query (caller should have proactively fetched IVRDriverInput_003 — see InterfaceHookInjector::DetourGetGenericInterface)");
        // Roll back the install marker so a subsequent invocation could retry.
        g_installedAgainstInternal.store(nullptr);
        return;
    }
    if (!IsAddressReadable(iface, sizeof(void *))) {
        LOG("[skeletal] Internal interface pointer %p not readable; aborting install", iface);
        g_installedAgainstInternal.store(nullptr);
        return;
    }
    void **vtable = *((void ***)iface);
    int updateIdx = g_internalVTableIndex[6];   // public[6] = UpdateSkeletonComponent
    int createIdx = g_internalVTableIndex[5];   // public[5] = CreateSkeletonComponent
    if (updateIdx < 0 || createIdx < 0) {
        LOG("[skeletal] recovered Internal indices invalid (update=%d create=%d); aborting", updateIdx, createIdx);
        g_installedAgainstInternal.store(nullptr);
        return;
    }
    // ALWAYS deep-probe the Internal interface, regardless of whether the
    // readability check below passes or fails. Captures the iface anatomy
    // for both the working-install and failing-install cases, so we can
    // diff the two if behaviour changes between SteamVR builds.
    DeepProbeInterface("internal", iface);

    if (!IsAddressReadable(&vtable[updateIdx], sizeof(void *)) ||
        !IsAddressReadable(&vtable[createIdx], sizeof(void *)))
    {
        LOG("[skeletal] Internal vtable slot pointers out of range; aborting (updateIdx=%d createIdx=%d vtable=%p)",
            updateIdx, createIdx, (void*)vtable);

        // Per-slot drilldown — log readability AND value (if readable) for
        // every slot 0..updateIdx. If slot N is unreadable but slot 0 is,
        // the vtable might be smaller than 7 entries. If slot 0 is also
        // unreadable, the vtable pointer itself is bogus.
        for (int i = 0; i <= updateIdx + 1 && i < 16; ++i) {
            void **slotAddr = vtable + i;
            bool readable = IsAddressReadable(slotAddr, sizeof(void *));
            if (readable) {
                LOG("[skeletal]   per_slot vtable[%d] @ %p readable, value=%p", i, (void*)slotAddr, *slotAddr);
            } else {
                LOG("[skeletal]   per_slot vtable[%d] @ %p NOT readable", i, (void*)slotAddr);
            }
        }
        g_installedAgainstInternal.store(nullptr);
        return;
    }

    // Pre-install vtable snapshot. After install we re-read these slots and
    // compare; the values should change to point at MinHook trampolines.
    void *preCreate = vtable[createIdx];
    void *preUpdate = vtable[updateIdx];
    LOG("[skeletal] pre-install snapshot: vtable[%d] (Create) = %p, vtable[%d] (Update) = %p",
        createIdx, preCreate, updateIdx, preUpdate);

    if (!IHook::Exists(InternalCreateSkeletonHook.name)) {
        InternalCreateSkeletonHook.CreateHookInObjectVTable(iface, createIdx, &DetourInternalCreateSkeletonComponent);
        IHook::Register(&InternalCreateSkeletonHook);
        LOG("[skeletal]   Create hook installed: originalFunc=%p, detour=%p (orig should NOT == detour)",
            (void*)InternalCreateSkeletonHook.originalFunc, (void*)&DetourInternalCreateSkeletonComponent);
    } else {
        LOG("[skeletal]   Create hook ALREADY EXISTS in registry; skipping install");
    }
    if (!IHook::Exists(InternalUpdateSkeletonHook.name)) {
        InternalUpdateSkeletonHook.CreateHookInObjectVTable(iface, updateIdx, &DetourInternalUpdateSkeletonComponent);
        IHook::Register(&InternalUpdateSkeletonHook);
        LOG("[skeletal]   Update hook installed: originalFunc=%p, detour=%p (orig should NOT == detour)",
            (void*)InternalUpdateSkeletonHook.originalFunc, (void*)&DetourInternalUpdateSkeletonComponent);
    } else {
        LOG("[skeletal]   Update hook ALREADY EXISTS in registry; skipping install");
    }

    // Post-install verification. The vtable slot values themselves typically
    // DON'T change with MinHook (it patches the function body, not the
    // pointer in the vtable), so we expect post == pre here. The change
    // should instead be in originalFunc (which is the MinHook trampoline,
    // distinct from both the original target and our detour).
    void *postCreate = vtable[createIdx];
    void *postUpdate = vtable[updateIdx];
    LOG("[skeletal] post-install snapshot: vtable[%d] (Create) = %p (changed=%d), vtable[%d] (Update) = %p (changed=%d)",
        createIdx, postCreate, (int)(postCreate != preCreate),
        updateIdx, postUpdate, (int)(postUpdate != preUpdate));

    // Probe the trampolines that MinHook stored in originalFunc — they should
    // be inside MinHook's allocated trampoline pool (typically a separate
    // VirtualAlloc'd region near the target). Confirm they're readable code.
    if (InternalCreateSkeletonHook.originalFunc) {
        LogVirtualQueryRegion("create_originalFunc_region", (void*)InternalCreateSkeletonHook.originalFunc);
    }
    if (InternalUpdateSkeletonHook.originalFunc) {
        LogVirtualQueryRegion("update_originalFunc_region", (void*)InternalUpdateSkeletonHook.originalFunc);
    }

    LOG("[skeletal] installed Internal hooks: CreateSkeleton @ vtable[%d], UpdateSkeleton @ vtable[%d] -- waiting for first calls",
        createIdx, updateIdx);
}

} // namespace skeletal
