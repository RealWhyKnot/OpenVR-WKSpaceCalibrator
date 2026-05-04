#pragma once

#include <windows.h>
#include <cstdint>
#include <atomic>
#include <stdexcept>
#include <functional>

#ifndef _OPENVR_API
#include <openvr_driver.h>
#endif

#define OPENVR_SPACECALIBRATOR_PIPE_NAME "\\\\.\\pipe\\OpenVRSpaceCalibratorDriver"
#define OPENVR_SPACECALIBRATOR_SHMEM_NAME "OpenVRSpaceCalibratorPoseMemoryV1"

#ifdef _OPENVR_API 

namespace vr {
	// We can't include openvr_driver.h as it will result in multiple definition of some structures.
	// However, we need to share driver-specific structures with the client application, so duplicate them
	// here.

	struct DriverPose_t
	{
		/* Time offset of this pose, in seconds from the actual time of the pose,
		 * relative to the time of the PoseUpdated() call made by the driver.
		 */
		double poseTimeOffset;

		/* Generally, the pose maintained by a driver
		 * is in an inertial coordinate system different
		 * from the world system of x+ right, y+ up, z+ back.
		 * Also, the driver is not usually tracking the "head" position,
		 * but instead an internal IMU or another reference point in the HMD.
		 * The following two transforms transform positions and orientations
		 * to app world space from driver world space,
		 * and to HMD head space from driver local body space.
		 *
		 * We maintain the driver pose state in its internal coordinate system,
		 * so we can do the pose prediction math without having to
		 * use angular acceleration.  A driver's angular acceleration is generally not measured,
		 * and is instead calculated from successive samples of angular velocity.
		 * This leads to a noisy angular acceleration values, which are also
		 * lagged due to the filtering required to reduce noise to an acceptable level.
		 */
		vr::HmdQuaternion_t qWorldFromDriverRotation;
		double vecWorldFromDriverTranslation[3];

		vr::HmdQuaternion_t qDriverFromHeadRotation;
		double vecDriverFromHeadTranslation[3];

		/* State of driver pose, in meters and radians. */
		/* Position of the driver tracking reference in driver world space
		* +[0] (x) is right
		* +[1] (y) is up
		* -[2] (z) is forward
		*/
		double vecPosition[3];

		/* Velocity of the pose in meters/second */
		double vecVelocity[3];

		/* Acceleration of the pose in meters/second */
		double vecAcceleration[3];

		/* Orientation of the tracker, represented as a quaternion */
		vr::HmdQuaternion_t qRotation;

		/* Angular velocity of the pose in axis-angle
		* representation. The direction is the angle of
		* rotation and the magnitude is the angle around
		* that axis in radians/second. */
		double vecAngularVelocity[3];

		/* Angular acceleration of the pose in axis-angle
		* representation. The direction is the angle of
		* rotation and the magnitude is the angle around
		* that axis in radians/second^2. */
		double vecAngularAcceleration[3];

		ETrackingResult result;

		bool poseIsValid;
		bool willDriftInYaw;
		bool shouldApplyHeadModel;
		bool deviceIsConnected;
	};

}

#endif

namespace protocol
{
	// v8 (2026-04-28): freezePrediction (bool) -> predictionSmoothness (uint8 0..100).
	// Old field gave a binary on/off; new field is a strength knob. Driver scales
	// velocity / acceleration / poseTimeOffset by (1 - smoothness/100) instead of
	// zeroing them when the bool was true. smoothness=100 reproduces the old freeze
	// behaviour; smoothness=0 leaves the pose untouched (off).
	// v9 (2026-05-02): adds RequestSetFingerSmoothing + FingerSmoothingConfig union
	// member. Driver-side hook on IVRDriverInputInternal::UpdateSkeletonComponent
	// applies per-bone slerp smoothing to Index Knuckles bone arrays before they
	// reach OpenVR consumers (proven 2026-05-02 in VRChat). Default OFF; the union
	// and Request sizeof are unchanged because the new struct is much smaller than
	// SetDeviceTransform — the bump is to force a paired overlay+driver reinstall
	// so the user sees a clean handshake error instead of a silently-ignored new
	// request type if they upgrade only one half.
	const uint32_t Version = 9;

	// Maximum length of a tracking-system-name string (e.g., "lighthouse", "oculus",
	// "Pimax Crystal HMD"). 32 bytes is more than enough for known systems and keeps
	// the IPC payload compact.
	static const size_t MaxTrackingSystemNameLen = 32;

	enum RequestType
	{
		RequestInvalid,
		RequestHandshake,
		RequestSetDeviceTransform,
		RequestSetAlignmentSpeedParams,
		RequestDebugOffset,
		RequestSetTrackingSystemFallback,
		// v9 (2026-05-02): finger-smoothing config push. Driver caches the
		// payload behind a small mutex; the IVRDriverInputInternal hook reads
		// it on every UpdateSkeletonComponent call (~340 Hz/hand).
		RequestSetFingerSmoothing,
	};

	enum ResponseType
	{
		ResponseInvalid,
		ResponseHandshake,
		ResponseSuccess,
	};

	struct Protocol
	{
		uint32_t version = Version;
	};

	struct AlignmentSpeedParams
	{
		/**
		 * The threshold at which we adjust the alignment speed based on the position offset
		 * between current and target calibrations. Generally, we increase the speed if we go
		 * above small/large, and decrease it only once it's under tiny.
		 * 
		 * These values are expressed as distance squared
		 */
		double thr_trans_tiny, thr_trans_small, thr_trans_large;

		/**
		 * Similar thresholds for rotation offsets, in radians
		 */
		double thr_rot_tiny, thr_rot_small, thr_rot_large;
		
		/**
		 * The speed of alignment, expressed as a lerp/slerp factor. 1 will blend most of the way in <1 second.
		 * (We actually do a lerp(s * delta_t) where s is the speed factor here)
		 */
		double align_speed_tiny, align_speed_small, align_speed_large;
	};

	struct SetDeviceTransform
	{
		uint32_t openVRID;
		bool enabled;
		bool updateTranslation;
		bool updateRotation;
		bool updateScale;
		vr::HmdVector3d_t translation;
		vr::HmdQuaternion_t rotation;
		double scale;
		bool lerp;
		bool quash;

		// Tracking system name of the device with this OpenVR ID, populated by the
		// overlay so the driver can match it against per-system fallbacks without
		// querying VR properties on every pose update. Empty string means "unknown".
		char target_system[MaxTrackingSystemNameLen];

		// Strength of native pose-prediction suppression for this device, on a
		// 0..100 scale. 0 = pose untouched (no suppression). 100 = velocity,
		// acceleration, angular velocity, angular acceleration and poseTimeOffset
		// are all zeroed before the pose ships -- defeating SteamVR's
		// extrapolation entirely (the old binary "freeze" behaviour). Values in
		// between scale velocity / acceleration by (1 - smoothness/100), giving
		// the user a smooth knob from "raw motion" to "fully suppressed".
		//
		// The overlay enforces a hard block on suppressing the calibration
		// reference / target trackers and the HMD: those values are forced to 0
		// regardless of what the user picks in the UI, because suppressing them
		// would corrupt either the calibration math or the user's view.
		uint8_t predictionSmoothness;

		// When true, BlendTransform's lerp toward targetTransform only advances
		// when the device is actually moving — instantaneous per-frame motion
		// gates the blend rate. The result: a stationary user (e.g. lying down)
		// sees no calibration drift even if the math is updating; the offset
		// catches up only when they next move, hidden by the natural motion.
		// Without this, sudden math updates (especially after watchdog clears)
		// cause visible "phantom body movement" while the user is still. Default
		// off in the struct — the overlay toggles it on per-device when the
		// recalibrateOnMovement profile setting is enabled.
		bool recalibrateOnMovement;

		SetDeviceTransform(uint32_t id, bool enabled) :
			openVRID(id), enabled(enabled), updateTranslation(false), updateRotation(false), updateScale(false), translation({}), rotation({1,0,0,0}), scale(1), lerp(false), quash(false), target_system{}, predictionSmoothness(0), recalibrateOnMovement(false) { }

		SetDeviceTransform(uint32_t id, bool enabled, vr::HmdVector3d_t translation) :
			openVRID(id), enabled(enabled), updateTranslation(true), updateRotation(false), updateScale(false), translation(translation), rotation({ 1,0,0,0 }), scale(1), lerp(false), quash(false), target_system{}, predictionSmoothness(0), recalibrateOnMovement(false) { }

		SetDeviceTransform(uint32_t id, bool enabled, vr::HmdQuaternion_t rotation) :
			openVRID(id), enabled(enabled), updateTranslation(false), updateRotation(true), updateScale(false), translation({}), rotation(rotation), scale(1), lerp(false), quash(false), target_system{}, predictionSmoothness(0), recalibrateOnMovement(false) { }

		SetDeviceTransform(uint32_t id, bool enabled, double scale) :
			openVRID(id), enabled(enabled), updateTranslation(false), updateRotation(false), updateScale(true), translation({}), rotation({ 1,0,0,0 }), scale(scale), lerp(false), quash(false), target_system{}, predictionSmoothness(0), recalibrateOnMovement(false) { }

		SetDeviceTransform(uint32_t id, bool enabled, vr::HmdVector3d_t translation, vr::HmdQuaternion_t rotation) :
			openVRID(id), enabled(enabled), updateTranslation(true), updateRotation(true), updateScale(false), translation(translation), rotation(rotation), scale(1), lerp(false), quash(false), target_system{}, predictionSmoothness(0), recalibrateOnMovement(false) { }

		SetDeviceTransform(uint32_t id, bool enabled, vr::HmdVector3d_t translation, vr::HmdQuaternion_t rotation, double scale) :
			openVRID(id), enabled(enabled), updateTranslation(true), updateRotation(true), updateScale(true), translation(translation), rotation(rotation), scale(scale), lerp(false), quash(false), target_system{}, predictionSmoothness(0), recalibrateOnMovement(false) { }
	};

	// Per-finger enable mask layout for FingerSmoothingConfig::finger_mask.
	// Bit (LSB-first):
	//   0  left thumb     5  right thumb
	//   1  left index     6  right index
	//   2  left middle    7  right middle
	//   3  left ring      8  right ring
	//   4  left pinky     9  right pinky
	// All 10 bits set = every finger smoothed (the typical opt-in case). Setting
	// a bit to 0 makes that finger pass through raw, useful when one finger's
	// smoothing produces an artifact and the user wants to isolate it without
	// disabling the whole feature.
	static const uint16_t kAllFingersMask = 0x03FF;

	// POD payload for RequestSetFingerSmoothing. Plain values + flag so the
	// struct is trivially memcpy-safe across the pipe with no marshalling.
	// Default-constructed instance encodes "feature off" (master_enabled = false)
	// — passthrough behaviour. The hook detour does zero work in that state, so
	// shipping with this struct in the union has zero performance cost when the
	// feature is unused.
	struct FingerSmoothingConfig
	{
		// Master kill-switch. When false the IVRDriverInputInternal::
		// UpdateSkeletonComponent detour forwards bone arrays untouched —
		// no per-bone slerp, no state lookup. Default false (opt-in).
		bool     master_enabled;

		// Smoothing strength on a 0..100 scale. 0 = pass-through (alpha=1.0,
		// each frame snaps to the incoming bones). 100 = heavy smoothing
		// (alpha clamped near 0, bones lag behind incoming significantly).
		// Linearly mapped to slerp factor by alpha = 1.0 - (smoothness/100)*0.95
		// so 100 still has a tiny per-frame nudge (alpha=0.05) — never fully
		// freezes a finger.
		uint8_t  smoothness;

		// Per-finger enable bits (see kAllFingersMask above). Disabled fingers
		// pass through unsmoothed. Bones outside the 5 finger chains
		// (root, wrist, aux markers) always pass through.
		uint16_t finger_mask;

		// 1 byte of trailing padding here on x64 to round to the natural
		// alignment of the struct. Explicit name so a future reader doesn't
		// mistake it for a meaningful flag — left zero by the overlay; the
		// driver MUST NOT read it.
		uint8_t  _reserved;
	};

	// Per-tracking-system fallback transform. Applied to any device whose tracking
	// system matches `system_name` and that doesn't currently have an active per-ID
	// transform. Lets newly connected trackers inherit the calibrated offset
	// immediately, without waiting for the overlay's next scan tick.
	struct SetTrackingSystemFallback
	{
		char system_name[MaxTrackingSystemNameLen];
		bool enabled;
		vr::HmdVector3d_t translation;
		vr::HmdQuaternion_t rotation;
		double scale;
		// Same semantics as SetDeviceTransform::predictionSmoothness (0..100).
		// Applied to every device that picks up this fallback (i.e. every device
		// of the matching tracking system that doesn't have an active per-ID
		// transform). The HMD/ref/target hard-block also applies here.
		uint8_t predictionSmoothness;
		// Same semantics as SetDeviceTransform::recalibrateOnMovement. Applies to
		// every device that picks up this fallback so newly-connected matching-
		// system trackers also get motion-gated blend instead of an instant snap
		// when their first per-ID transform later arrives.
		bool recalibrateOnMovement;
	};

	struct Request
	{
		RequestType type;

		union {
			SetDeviceTransform setDeviceTransform;
			AlignmentSpeedParams setAlignmentSpeedParams;
			SetTrackingSystemFallback setTrackingSystemFallback;
			// v9: finger smoothing. FingerSmoothingConfig is much smaller than
			// SetDeviceTransform so this addition does not grow the union and
			// therefore does not change sizeof(Request). The Version bump
			// is purely to force paired install — the wire layout is otherwise
			// backwards-compatible.
			FingerSmoothingConfig setFingerSmoothing;
		};

		Request() : type(RequestInvalid), setAlignmentSpeedParams({}) { }
		Request(RequestType type) : type(type), setAlignmentSpeedParams({}) { }
		Request(AlignmentSpeedParams params) : type(RequestType::RequestSetAlignmentSpeedParams), setAlignmentSpeedParams(params) {}
	};

	struct Response
	{
		ResponseType type;

		union {
			Protocol protocol;
		};

		Response() : type(ResponseInvalid), protocol({}) {}
		Response(ResponseType type) : type(type), protocol({}) { }
	};

	class DriverPoseShmem {
	public:
		struct AugmentedPose {
			LARGE_INTEGER sample_time;
			int deviceId;
			vr::DriverPose_t pose;
		};

		// Driver-side telemetry counters. Each is a monotonically increasing count of
		// the number of times the driver took the corresponding code path while
		// processing a tracked-device pose update. Relaxed-ordered atomic increments
		// are sufficient because the overlay only consumes deltas — there is no
		// cross-counter consistency requirement.
		struct Telemetry {
			std::atomic<uint64_t> fallback_apply_count;
			std::atomic<uint64_t> per_id_apply_count;
			std::atomic<uint64_t> quash_apply_count;
			std::atomic<uint64_t> reserved[5];
		};

		// Names of the individual telemetry counters, used by IncrementTelemetry to
		// pick which atomic to bump. Kept inside the class so we don't pollute the
		// `protocol` namespace with another enum.
		enum TelemetryField {
			TELEMETRY_FALLBACK_APPLY,
			TELEMETRY_PER_ID_APPLY,
			TELEMETRY_QUASH_APPLY,
		};

		// Sentinel written by the driver into ShmemData::magic. The overlay
		// rejects any segment whose magic doesn't match — guards against
		// reading a stale or wrong-format mapping left behind by a different
		// build.
		static const uint32_t SHMEM_MAGIC = 0xCA11B8A7;

		// Version of the shmem layout. Bump on any field addition / reordering
		// / removal so a driver/overlay version skew is rejected at Open()
		// instead of corrupting poses with mismatched offsets.
		static const uint32_t SHMEM_VERSION = 1;

	private:
		static const uint32_t SYNC_ACTIVE_POSE_B = 0x80000000;
		static const uint32_t BUFFERED_SAMPLES = 64 * 1024;

		struct ShmemData {
			// Magic + version sit first so the overlay can validate a mapping
			// before it touches anything else. Driver Create() writes them; the
			// overlay Open() reads and verifies. A mismatch means the overlay
			// is paired with a different driver build — far better to throw
			// than to silently feed it garbage poses.
			uint32_t magic;
			uint32_t shmem_version;
			// Telemetry sits right after the header so any future growth of the
			// pose ring buffer doesn't shift its address. The shmem layout is
			// regenerated on driver startup and the overlay/driver IPC handshake
			// already gates compatible builds, so a layout change is acceptable
			// — but keeping telemetry pinned to a stable offset keeps debugging
			// dumps readable.
			Telemetry telemetry;
			std::atomic<uint64_t> index;
			AugmentedPose poses[BUFFERED_SAMPLES];
		};

		// Compile-time guard against silent layout drift between driver and
		// overlay builds. If a field is added / reordered / repacked this
		// assertion fires at compile time instead of producing corrupted poses
		// at runtime. Sum of every field's sizeof — the trailing struct ends
		// 8-aligned and every field starts on its natural boundary so the sum
		// matches the struct size exactly. If you intentionally change the
		// layout, bump SHMEM_VERSION and update this assertion.
		static_assert(
			sizeof(ShmemData) ==
				sizeof(uint32_t) + sizeof(uint32_t) +
				sizeof(Telemetry) +
				sizeof(std::atomic<uint64_t>) +
				sizeof(AugmentedPose) * BUFFERED_SAMPLES,
			"DriverPoseShmem::ShmemData layout has drifted; bump SHMEM_VERSION and update the static_assert"
		);
		
	private:
		HANDLE hMapFile;
		ShmemData* pData;
		uint64_t cursor;

		AugmentedPose lastPose[vr::k_unMaxTrackedDeviceCount] = {0};

		std::string LastErrorString(DWORD lastError)
		{
			LPSTR buffer = nullptr;
			size_t size = FormatMessageA(
				FORMAT_MESSAGE_ALLOCATE_BUFFER | FORMAT_MESSAGE_FROM_SYSTEM | FORMAT_MESSAGE_IGNORE_INSERTS,
				NULL, lastError, MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT), (LPSTR)&buffer, 0, NULL
			);

			std::string message(buffer, size);
			LocalFree(buffer);
			return message;
		}

	public:
		operator bool() const {
			return pData != nullptr;
		}

		bool operator!() const {
			return pData == nullptr;
		}

		DriverPoseShmem() {
			hMapFile = INVALID_HANDLE_VALUE;
			pData = nullptr;
			cursor = 0;
		}

		~DriverPoseShmem() {
			Close();
		}

		void Close() {
			if (pData) UnmapViewOfFile(pData);
			if (hMapFile) CloseHandle(hMapFile);
		}

		bool Create(LPCSTR segment_name) {
			Close();

			hMapFile = CreateFileMappingA(
				INVALID_HANDLE_VALUE,
				NULL,
				PAGE_READWRITE,
				0,
				sizeof(ShmemData),
				segment_name
			);

			if (!hMapFile) return false;

			pData = reinterpret_cast<ShmemData*>(MapViewOfFile(
				hMapFile,
				FILE_MAP_ALL_ACCESS,
				0,
				0,
				sizeof(ShmemData)
			));

			if (!pData) return false;

			// Stamp the header so the overlay can verify it's looking at a
			// driver-compatible mapping. New file mappings are zero-initialised
			// by the OS, so simply writing the magic/version is enough.
			pData->magic = SHMEM_MAGIC;
			pData->shmem_version = SHMEM_VERSION;

			return true;
		}


		void Open(LPCSTR segment_name) {
			Close();

			hMapFile = OpenFileMappingA(
				FILE_MAP_ALL_ACCESS,
				FALSE,
				segment_name
			);

			if (!hMapFile) {
				throw std::runtime_error("Failed to open pose data shared memory segment: " + LastErrorString(GetLastError()));
			}

			pData = reinterpret_cast<ShmemData*>(MapViewOfFile(
				hMapFile,
				FILE_MAP_ALL_ACCESS,
				0,
				0,
				sizeof(ShmemData)
			));

			if (!pData) {
				throw std::runtime_error("Failed to map pose data shared memory segment: " + LastErrorString(GetLastError()));
			}

			// Validate the header. A mismatch means the overlay is paired with a
			// driver build that doesn't share this layout — much safer to throw
			// here than to silently corrupt poses by reading at the wrong offsets.
			if (pData->magic != SHMEM_MAGIC) {
				char buf[128];
				snprintf(buf, sizeof buf,
					"Pose shmem segment magic mismatch: got 0x%08X, expected 0x%08X (driver/overlay version skew?)",
					pData->magic, SHMEM_MAGIC);
				UnmapViewOfFile(pData); pData = nullptr;
				CloseHandle(hMapFile); hMapFile = nullptr;
				throw std::runtime_error(buf);
			}
			if (pData->shmem_version != SHMEM_VERSION) {
				char buf[128];
				snprintf(buf, sizeof buf,
					"Pose shmem segment version mismatch: got %u, expected %u (driver/overlay version skew?)",
					pData->shmem_version, SHMEM_VERSION);
				UnmapViewOfFile(pData); pData = nullptr;
				CloseHandle(hMapFile); hMapFile = nullptr;
				throw std::runtime_error(buf);
			}

			char tmp[256];
			snprintf(tmp, sizeof tmp, "Opened shmem segment: %p\n", pData);
			OutputDebugStringA(tmp);
		}

		void ReadNewPoses(std::function<void(AugmentedPose const&)> cb) {
			if (!pData) throw std::runtime_error("Not open");
			
			uint64_t cur_index = pData->index.load(std::memory_order_acquire);
			if (cur_index < cursor || cur_index - cursor > BUFFERED_SAMPLES / 2) {
				if (cur_index < BUFFERED_SAMPLES / 2)
					cursor = cur_index;
				else
					cursor = cur_index - BUFFERED_SAMPLES / 2;
			}

			while (cursor < cur_index) {
				cb(pData->poses[cursor % BUFFERED_SAMPLES]);
				cursor++;
			}

			std::atomic_thread_fence(std::memory_order_release);
		}

		bool GetPose(int index, vr::DriverPose_t& pose, LARGE_INTEGER *pSampleTime = NULL) {
			ReadNewPoses([this](AugmentedPose const& pose) {
				if (pose.pose.poseIsValid && pose.pose.result == vr::ETrackingResult::TrackingResult_Running_OK) {
					this->lastPose[pose.deviceId] = pose;
				}
			});

			if (index >= 0 && index < vr::k_unMaxTrackedDeviceCount) {
				pose = lastPose[index].pose;
				if (pSampleTime) *pSampleTime = lastPose[index].sample_time;
				return true;
			}
			// Out-of-range index: don't touch `pose`, signal failure. The caller
			// must check the return value (the prior version fell off the end of
			// a non-void function — undefined behaviour, papered over only by the
			// fact that nothing actually called this with an out-of-range index).
			return false;
		}

		void SetPose(int index, const vr::DriverPose_t& pose) {
			if (index >= vr::k_unMaxTrackedDeviceCount) return;
			if (pData == nullptr) return;

			AugmentedPose augPose = {0};
			augPose.deviceId = index;
			augPose.pose = pose;
			QueryPerformanceCounter(&augPose.sample_time);

			uint64_t cur_index = pData->index.load(std::memory_order_relaxed) + 1;
			pData->poses[cur_index % BUFFERED_SAMPLES] = augPose;
			pData->index.store(cur_index, std::memory_order_release);
		}

		// Bump the named telemetry counter by one. Safe to call from the driver's
		// pose-update path — a relaxed atomic increment is sub-ns on x86 and there
		// are no cross-counter ordering requirements.
		void IncrementTelemetry(TelemetryField field) {
			if (pData == nullptr) return;
			switch (field) {
			case TELEMETRY_FALLBACK_APPLY:
				pData->telemetry.fallback_apply_count.fetch_add(1, std::memory_order_relaxed);
				break;
			case TELEMETRY_PER_ID_APPLY:
				pData->telemetry.per_id_apply_count.fetch_add(1, std::memory_order_relaxed);
				break;
			case TELEMETRY_QUASH_APPLY:
				pData->telemetry.quash_apply_count.fetch_add(1, std::memory_order_relaxed);
				break;
			default:
				// Unknown enum value: silently ignore. Explicit default silences
				// implicit-fall-through warnings and documents that no other
				// counters exist today; new fields must be added above.
				break;
			}
		}

		// Snapshot the telemetry counters. Returns true if the shmem segment is open.
		// The values are loaded with relaxed ordering — the overlay only ever
		// computes deltas across snapshots, so torn reads relative to other counters
		// don't matter.
		bool GetTelemetry(uint64_t& fallback_apply, uint64_t& per_id_apply, uint64_t& quash_apply) {
			if (pData == nullptr) return false;
			fallback_apply = pData->telemetry.fallback_apply_count.load(std::memory_order_relaxed);
			per_id_apply = pData->telemetry.per_id_apply_count.load(std::memory_order_relaxed);
			quash_apply = pData->telemetry.quash_apply_count.load(std::memory_order_relaxed);
			return true;
		}
	};
}