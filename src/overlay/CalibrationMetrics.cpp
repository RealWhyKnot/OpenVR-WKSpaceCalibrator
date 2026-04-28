#include "stdafx.h"
#include "CalibrationMetrics.h"
#include "BuildStamp.h"
#include <shlobj_core.h>
#include <fstream>
#include <openvr.h>
#include <string>
#include <vector>

namespace Metrics {
	double TimeSpan = 30, CurrentTime = 0;

	TimeSeries<Eigen::Vector3d> posOffset_rawComputed; // , rotOffset_rawComputed;
	TimeSeries<Eigen::Vector3d> posOffset_currentCal; // , rotOffset_currentCal;
	TimeSeries<Eigen::Vector3d> posOffset_lastSample; // , rotOffset_lastSample;
	TimeSeries<Eigen::Vector3d> posOffset_byRelPose;

	TimeSeries<double> error_rawComputed, error_currentCal, error_byRelPose, error_currentCalRelPose;
	TimeSeries<double> axisIndependence;
	TimeSeries<double> computationTime;
	TimeSeries<double> jitterRef, jitterTarget;
	TimeSeries<double> rotationConditionRatio;
	TimeSeries<double> consecutiveRejections;
	TimeSeries<double> samplesInBuffer;
	TimeSeries<double> watchdogResetCount;
	TimeSeries<double> translationDiversity;
	TimeSeries<double> rotationDiversity;
	std::string lastRejectReason;

	TimeSeries<double> fallbackApplyRate;
	TimeSeries<double> perIdApplyRate;
	TimeSeries<double> quashApplyRate;

	// true - full calibration, false - static calibration
	TimeSeries<bool> calibrationApplied;

	// https://stackoverflow.com/a/17827724
	bool IsBrowsePath(const std::wstring& path)
	{
		return (path == L"." || path == L"..");
	}

	uint64_t CalculateDirSize(const std::wstring& path, uint64_t size = 0)
	{
		WIN32_FIND_DATA data;
		HANDLE sh = NULL;
		sh = FindFirstFile((path + L"\\*").c_str(), &data);

		if (sh == INVALID_HANDLE_VALUE)
		{
			// We should probably return an error, but we don't for the sake of minimising memory allocations
			return size;
		}

		do
		{
			// skip current and parent
			if (!IsBrowsePath(data.cFileName))
			{
				// if found object is ...
				if ((data.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY) == FILE_ATTRIBUTE_DIRECTORY)
					// directory, then search it recursievly
					size = CalculateDirSize(path + L"\\" + data.cFileName, size);
				else
					// otherwise get object size and add it to directory size
					size += (uint64_t)(data.nFileSizeHigh * (MAXDWORD)+data.nFileSizeLow);
			}

		} while (FindNextFile(sh, &data)); // do

		FindClose(sh);

		return size;
	}

	double timestamp() {
		static long long ts_start = ~0LL;
		
		LARGE_INTEGER ts, freq;
		QueryPerformanceCounter(&ts);
		QueryPerformanceFrequency(&freq);

		if (ts_start == ~0LL) ts_start = ts.QuadPart;

		ts.QuadPart -= ts_start;

		return ts.QuadPart / (double)freq.QuadPart;
	}

	void RecordTimestamp() {
		CurrentTime = timestamp();
	}

	bool enableLogs = false;

	static std::ofstream logFile;
	static bool logFileIsOpen = false;
	static bool failedToOpenLogFile = false;

	// v2 wire-format addition: per-tick raw reference and target poses plus the tick
	// phase. Filled by SetTickRawPoses() each tick and consumed by the field writers
	// below. Defaults are an identity pose with phase=None so that any unexpected
	// WriteLogEntry call (i.e. one not preceded by SetTickRawPoses) emits a syntactically
	// valid row rather than uninitialized memory.
	struct TickRawPoses {
		Eigen::Vector3d refTrans = Eigen::Vector3d::Zero();
		Eigen::Quaterniond refRot = Eigen::Quaterniond::Identity();
		Eigen::Vector3d targetTrans = Eigen::Vector3d::Zero();
		Eigen::Quaterniond targetRot = Eigen::Quaterniond::Identity();
		TickPhase phase = TickPhase::None;
	};
	static TickRawPoses g_tickRaw;

	static const char* TickPhaseName(TickPhase p) {
		switch (p) {
		case TickPhase::None: return "None";
		case TickPhase::Begin: return "Begin";
		case TickPhase::Rotation: return "Rotation";
		case TickPhase::Translation: return "Translation";
		case TickPhase::Editing: return "Editing";
		case TickPhase::Continuous: return "Continuous";
		case TickPhase::ContinuousStandby: return "ContinuousStandby";
		}
		return "None";
	}

	void SetTickRawPoses(
		const Eigen::Vector3d& refTrans, const Eigen::Quaterniond& refRot,
		const Eigen::Vector3d& targetTrans, const Eigen::Quaterniond& targetRot,
		TickPhase phase)
	{
		g_tickRaw.refTrans = refTrans;
		g_tickRaw.refRot = refRot;
		g_tickRaw.targetTrans = targetTrans;
		g_tickRaw.targetRot = targetRot;
		g_tickRaw.phase = phase;
	}

	struct CsvField {
		const char* name;
		void (*writer)(std::ofstream& s);
	};

#define TS_FIELD(n) \
	{ #n, [](auto &s) { s << n.last(); } }
	
#define TS_VECTOR_FIELD(n) \
	{ #n ".x", [](auto &s) { s << n.last()(0); } }, \
	{ #n ".y", [](auto &s) { s << n.last()(1); } }, \
	{ #n ".z", [](auto &s) { s << n.last()(2); } }

	static const CsvField fields[] = {
		{
			"Timestamp",
			[](auto& s) { s << CurrentTime; }
		},

		TS_VECTOR_FIELD(posOffset_rawComputed),
		TS_VECTOR_FIELD(posOffset_currentCal),
		TS_VECTOR_FIELD(posOffset_lastSample),
		TS_VECTOR_FIELD(posOffset_byRelPose),
		
		TS_FIELD(error_rawComputed),
		TS_FIELD(error_currentCal),
		TS_FIELD(error_byRelPose),
		TS_FIELD(error_currentCalRelPose),
		TS_FIELD(axisIndependence),
		TS_FIELD(rotationConditionRatio),
		TS_FIELD(consecutiveRejections),
		TS_FIELD(samplesInBuffer),
		TS_FIELD(watchdogResetCount),
		TS_FIELD(computationTime),
		TS_FIELD(jitterRef),
		TS_FIELD(jitterTarget),
		TS_FIELD(fallbackApplyRate),
		TS_FIELD(perIdApplyRate),
		TS_FIELD(quashApplyRate),
		// String-valued reject reason. Empty when the last ComputeIncremental
		// accepted; otherwise one of: "below_floor_or_worse", "axis_variance_low",
		// "rotation_planar", "rotation_no_deltas", "translation_planar",
		// "translation_no_deltas", "validate_failed", "healthy_below_floor".
		{ "reject_reason", [](auto& s) { s << lastRejectReason; } },

		{
			"calibrationApplied",
			[](auto& s) {
				if (calibrationApplied.lastTs() == CurrentTime) {
					if (calibrationApplied.last()) {
						s << "FULL";
					}
					else {
						s << "STATIC";
					}
				}
			}
		},

		// --- v2 columns: raw reference + target poses and tick phase ---------------
		// Translations are in meters, rotations are unit quaternions in (w,x,y,z) order.
		// These are written with full double precision so the replay harness can
		// reconstruct the exact `Sample` values that fed CalibrationCalc::PushSample.
		{ "ref_tx", [](auto& s) { s.precision(17); s << g_tickRaw.refTrans.x(); } },
		{ "ref_ty", [](auto& s) { s.precision(17); s << g_tickRaw.refTrans.y(); } },
		{ "ref_tz", [](auto& s) { s.precision(17); s << g_tickRaw.refTrans.z(); } },
		{ "ref_qw", [](auto& s) { s.precision(17); s << g_tickRaw.refRot.w(); } },
		{ "ref_qx", [](auto& s) { s.precision(17); s << g_tickRaw.refRot.x(); } },
		{ "ref_qy", [](auto& s) { s.precision(17); s << g_tickRaw.refRot.y(); } },
		{ "ref_qz", [](auto& s) { s.precision(17); s << g_tickRaw.refRot.z(); } },
		{ "tgt_tx", [](auto& s) { s.precision(17); s << g_tickRaw.targetTrans.x(); } },
		{ "tgt_ty", [](auto& s) { s.precision(17); s << g_tickRaw.targetTrans.y(); } },
		{ "tgt_tz", [](auto& s) { s.precision(17); s << g_tickRaw.targetTrans.z(); } },
		{ "tgt_qw", [](auto& s) { s.precision(17); s << g_tickRaw.targetRot.w(); } },
		{ "tgt_qx", [](auto& s) { s.precision(17); s << g_tickRaw.targetRot.x(); } },
		{ "tgt_qy", [](auto& s) { s.precision(17); s << g_tickRaw.targetRot.y(); } },
		{ "tgt_qz", [](auto& s) { s.precision(17); s << g_tickRaw.targetRot.z(); } },
		{ "tick_phase", [](auto& s) { s << TickPhaseName(g_tickRaw.phase); } },
	};
	
	
	static void ClearOldLogs(const std::wstring& path) {
		std::wstring search_path = path + L"\\spacecal_log.*.txt";
		WIN32_FIND_DATA find_data;

		SYSTEMTIME st_now;
		FILETIME ft_now;
		GetSystemTime(&st_now);
		SystemTimeToFileTime(&st_now, &ft_now);

		ULARGE_INTEGER ft_tmp;
		ft_tmp.HighPart = ft_now.dwHighDateTime;
		ft_tmp.LowPart = ft_now.dwLowDateTime;
		// one day ago
		uint64_t limit = ft_tmp.QuadPart - (uint64_t)(24LL * 3600LL * 10LL * 1000LL * 1000LL);

		HANDLE find_handle = FindFirstFile(search_path.c_str(), &find_data);
		if (find_handle != INVALID_HANDLE_VALUE) {
			do {
				ft_tmp.HighPart = find_data.ftLastWriteTime.dwHighDateTime;
				ft_tmp.LowPart = find_data.ftLastWriteTime.dwLowDateTime;
				if (ft_tmp.QuadPart < limit) {
					std::wstring file_path = path + L"\\" + find_data.cFileName;
					DeleteFile(file_path.c_str());
				}
			} while (FindNextFile(find_handle, &find_data));
			FindClose(find_handle);
		}
	}

	// %userprofile%\LocalLow\SpaceCalibrator\Logs
	static bool OpenLogFile() {
		PWSTR RootPath = nullptr;
		if (S_OK != SHGetKnownFolderPath(FOLDERID_LocalAppDataLow, 0, nullptr, &RootPath)) {
			CoTaskMemFree(RootPath);
			return false;
		}

		std::wstring path(RootPath);
		CoTaskMemFree(RootPath);
		
		path += LR"(\SpaceCalibrator)";
		if (CreateDirectoryW(path.c_str(), 0) == 0 && GetLastError() != ERROR_ALREADY_EXISTS) {
			return false;
		}

		path += LR"(\Logs)";
		if (CreateDirectoryW(path.c_str(), 0) == 0 && GetLastError() != ERROR_ALREADY_EXISTS) {
			return false;
		}

		ClearOldLogs(path);

		SYSTEMTIME now{};
		GetSystemTime(&now);

		size_t dateBufLen = GetDateFormatW(LOCALE_USER_DEFAULT, 0, &now, L"yyyy-MM-dd", nullptr, 0);
		std::vector<WCHAR> dateBuf(dateBufLen);
		if (!GetDateFormatEx(LOCALE_NAME_INVARIANT, 0, &now, L"yyyy-MM-dd", &dateBuf[0], static_cast<int>(dateBufLen), nullptr)) return false;
		
		size_t timeBufLen = GetTimeFormatW(LOCALE_USER_DEFAULT, 0, &now, L"HH-mm-ss", nullptr, 0);
		std::vector<WCHAR> timeBuf(timeBufLen);
		if (!GetTimeFormatEx(LOCALE_NAME_INVARIANT, 0, &now, L"HH-mm-ss", &timeBuf[0], static_cast<int>(timeBufLen))) return false;

		path += LR"(\spacecal_log.)";
		path += &dateBuf[0];
		path += L"T";
		path += &timeBuf[0];
		path += L".txt";

		logFile.open(path);
		if (logFile.fail()) {
			return false;
		}

		// Wire-format version annotation. v2 added per-tick raw reference + target
		// poses (ref_t{x,y,z}, ref_q{w,x,y,z}, tgt_*) and tick_phase. The replay
		// harness in tools/replay/ rejects logs that don't begin with this banner so
		// older v1 captures (which lacked raw poses) fail loud rather than silently
		// being interpreted with the wrong column layout. New columns added later
		// (samplesInBuffer, watchdogResetCount, reject_reason) are still v2 because
		// the replay harness looks columns up by name, not position.
		logFile << "# spacecal_log_v2\n";

		// === Self-describing header ============================================
		// Triage from a debug log starts with "what build was this, on what
		// hardware, with what profile". Embed that up-front so a single log file
		// is sufficient for a bug report — no need to ask the reporter to run a
		// dozen follow-up commands.
		logFile << "# build_stamp=" SPACECAL_BUILD_STAMP "\n";
		logFile << "# build_channel=" SPACECAL_BUILD_CHANNEL "\n";

		// HMD identification — model + tracking system. Driver-side issues often
		// correlate to a specific HMD or runtime, so this is the first thing
		// anyone reading the log wants to know.
		if (auto vrSystem = vr::VRSystem()) {
			char buf[vr::k_unMaxPropertyStringSize] = {};
			vr::ETrackedPropertyError pe = vr::TrackedProp_Success;
			vrSystem->GetStringTrackedDeviceProperty(
				vr::k_unTrackedDeviceIndex_Hmd,
				vr::Prop_TrackingSystemName_String,
				buf, sizeof buf, &pe);
			if (pe == vr::TrackedProp_Success && buf[0]) {
				logFile << "# hmd_tracking_system=" << buf << "\n";
			}
			buf[0] = 0;
			vrSystem->GetStringTrackedDeviceProperty(
				vr::k_unTrackedDeviceIndex_Hmd,
				vr::Prop_ModelNumber_String,
				buf, sizeof buf, &pe);
			if (pe == vr::TrackedProp_Success && buf[0]) {
				logFile << "# hmd_model=" << buf << "\n";
			}
			buf[0] = 0;
			vrSystem->GetStringTrackedDeviceProperty(
				vr::k_unTrackedDeviceIndex_Hmd,
				vr::Prop_SerialNumber_String,
				buf, sizeof buf, &pe);
			if (pe == vr::TrackedProp_Success && buf[0]) {
				logFile << "# hmd_serial=" << buf << "\n";
			}

			// SteamVR runtime version, when reachable. Useful for filtering issues
			// to a specific Steam beta/stable lineage.
			char rtPath[MAX_PATH] = {};
			unsigned int rtLen = 0;
			vr::VR_GetRuntimePath(rtPath, MAX_PATH, &rtLen);
			if (rtLen > 0) {
				logFile << "# steamvr_runtime_path=" << rtPath << "\n";
			}
		} else {
			logFile << "# vr_system=unavailable_at_log_open\n";
		}

		// CPU + memory floor for context. SHGetKnownFolderPath needed Win10+
		// already so OS version reporting is just for triage; not gated.
		{
			OSVERSIONINFOEXW osv{ sizeof(OSVERSIONINFOEXW) };
			using RtlGetVersionPtr = LONG(WINAPI*)(LPOSVERSIONINFOEXW);
			HMODULE ntdll = GetModuleHandleW(L"ntdll.dll");
			if (ntdll) {
				auto fn = (RtlGetVersionPtr)GetProcAddress(ntdll, "RtlGetVersion");
				if (fn && fn(&osv) == 0) {
					logFile << "# windows=" << osv.dwMajorVersion << "."
					        << osv.dwMinorVersion << "." << osv.dwBuildNumber << "\n";
				}
			}

			SYSTEM_INFO si{};
			GetSystemInfo(&si);
			logFile << "# logical_processors=" << si.dwNumberOfProcessors << "\n";
		}

		// Banner divider so a human grepping the file can see where header ends
		// and the column row begins. Replay harness ignores any line starting with #.
		logFile << "# === columns ===\n";

		for (int i = 0; i < sizeof fields / sizeof fields[0]; i++) {
			if (i > 0) logFile << ",";
			logFile << fields[i].name;
		}
		logFile << "\n";

		logFileIsOpen = true;

		return true;
	}
	
	static bool CheckLogOpen() {
		if (!enableLogs) {
			if (logFileIsOpen) {
				logFile.close();
			}
			logFileIsOpen = false;
			failedToOpenLogFile = false;

			return false;
		}

		if (failedToOpenLogFile) return false;
		if (!logFileIsOpen && !OpenLogFile()) {
			failedToOpenLogFile = true;
			return false;
		}
		return true;
	}

	void WriteLogAnnotation(const char *s) {
		if (!CheckLogOpen()) return;

		logFile << "# [" << timestamp() << "] " << s << "\n";
		logFile.flush();
	}

	void WriteLogEntry() {
		if (!CheckLogOpen()) return;

		if (logFileIsOpen) {
			for (int i = 0; i < sizeof fields / sizeof fields[0]; i++) {
				if (i > 0) logFile << ",";
				fields[i].writer(logFile);
			}
			logFile << "\n";
		}
		logFile.flush();
	}
}
