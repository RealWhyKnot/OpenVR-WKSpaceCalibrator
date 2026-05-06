#include "stdafx.h"
#include "CalibrationMetrics.h"
#include "BuildStamp.h"
#include <shlobj_core.h>
#include <fstream>
#include <openvr.h>
#include <string>
#include <vector>
#include <tlhelp32.h>   // Toolhelp32 snapshot — used for parent-process lookup
                        // in the launch-context banner block below.
#include <psapi.h>      // QueryFullProcessImageNameW for parent exe path.
#include <sddl.h>       // ConvertSidToStringSidW — token user SID stringification.
#pragma comment(lib, "Advapi32.lib")  // OpenProcessToken / GetTokenInformation /
                                      // ConvertSidToStringSidW. Already linked
                                      // transitively but explicit is safer.

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
	TimeSeries<Eigen::Vector3d> translationAxisRangesCm;
	TimeSeries<double> watchdogHealthySkip;
	TimeSeries<double> effectivePriorMm;
	TimeSeries<double> validateRmsThresholdMm;
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

	// Debug-log toggle. Default depends on the build channel:
	//   - dev builds (build.ps1 without -Version, SPACECAL_BUILD_CHANNEL="dev"):
	//     ON. Local development sessions reproduce issues that benefit from
	//     a CSV trail without the developer having to remember to flip the
	//     toggle every launch.
	//   - release builds (CI tag publish, SPACECAL_BUILD_CHANNEL="release"):
	//     OFF. End users only opt in when investigating a bug.
	// The user can flip it either direction from the Logs tab regardless;
	// this is just the boot default.
#if defined(SPACECAL_BUILD_CHANNEL_IS_DEV)
	bool enableLogs = true;
#else
	bool enableLogs = (std::string(SPACECAL_BUILD_CHANNEL) == "dev");
#endif

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
		// Motion-coverage scores for the live sample buffer (0..1 each). Pushed by
		// CollectSample. The Calibration Progress popup reads these for the live
		// "Translation %" / "Rotation %" bars; logging them here lets post-hoc
		// triage see exactly what the bars showed when a one-shot calibration
		// got stuck below the auto-finish threshold.
		TS_FIELD(translationDiversity),
		TS_FIELD(rotationDiversity),
		TS_VECTOR_FIELD(translationAxisRangesCm),
		// Wedge-detection diagnostics. watchdogHealthySkip flags ticks where the
		// watchdog wanted to clear but couldn't (prior in healthy band).
		// effectivePriorMm is the actual prior the 1.5× gate compared against.
		// validateRmsThresholdMm is the dynamic noise-floor threshold the
		// validate gate used this tick.
		TS_FIELD(watchdogHealthySkip),
		TS_FIELD(effectivePriorMm),
		TS_FIELD(validateRmsThresholdMm),
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

	// =============================================================================
	// Launch-context banner. Writes a block of `# key=value` lines describing
	// EVERYTHING that could plausibly differ between a Start-menu launch and a
	// script-launched instance: process identity, parent process, command line,
	// cwd, token/elevation/integrity/session, window station + desktop, console
	// attachment, startup flags, and any VR/Steam/OpenVR env vars.
	//
	// Goal: make it possible to take two log files (one launched from Start
	// menu, one from quick.ps1) and diff the banners to see what's actually
	// different at the process level. Without this we're left speculating.
	//
	// Each Win32 call that fails logs the error code rather than being silently
	// dropped — a missing parent_exe with err=5 (access denied) is a meaningful
	// data point, not noise.
	// =============================================================================
	static std::string WideToUtf8(const wchar_t* w)
	{
		if (!w) return std::string();
		int len = WideCharToMultiByte(CP_UTF8, 0, w, -1, nullptr, 0, nullptr, nullptr);
		if (len <= 0) return std::string();
		std::string out(len - 1, '\0'); // -1 to drop the trailing NUL count
		WideCharToMultiByte(CP_UTF8, 0, w, -1, &out[0], len, nullptr, nullptr);
		return out;
	}

	static void WriteLaunchContextBanner(std::ofstream& out)
	{
		// --- Process identity ----------------------------------------------------
		const DWORD pid = GetCurrentProcessId();
		out << "# launch_pid=" << pid << "\n";

		wchar_t exePath[MAX_PATH] = {};
		if (GetModuleFileNameW(nullptr, exePath, MAX_PATH)) {
			out << "# launch_exe_path=" << WideToUtf8(exePath) << "\n";
		} else {
			out << "# launch_exe_path=GetModuleFileNameW_failed_err=" << GetLastError() << "\n";
		}

		LPWSTR cmdline = GetCommandLineW();
		if (cmdline) {
			out << "# launch_command_line=" << WideToUtf8(cmdline) << "\n";
		}

		wchar_t cwd[MAX_PATH] = {};
		DWORD cwdLen = GetCurrentDirectoryW(MAX_PATH, cwd);
		if (cwdLen > 0 && cwdLen < MAX_PATH) {
			out << "# launch_cwd=" << WideToUtf8(cwd) << "\n";
		} else {
			out << "# launch_cwd=GetCurrentDirectoryW_failed_err=" << GetLastError() << "\n";
		}

		// --- Parent process (THE big question for the script-vs-manual mystery) -
		// Toolhelp32 is documented + reliable on every Win10/11 build. The
		// alternative (NtQueryInformationProcess + ProcessBasicInformation) is
		// undocumented and overkill here. Snapshot is one process-wide enumeration,
		// runs in <1ms.
		DWORD parentPid = 0;
		{
			HANDLE snap = CreateToolhelp32Snapshot(TH32CS_SNAPPROCESS, 0);
			if (snap != INVALID_HANDLE_VALUE) {
				PROCESSENTRY32W pe{};
				pe.dwSize = sizeof pe;
				if (Process32FirstW(snap, &pe)) {
					do {
						if (pe.th32ProcessID == pid) {
							parentPid = pe.th32ParentProcessID;
							break;
						}
					} while (Process32NextW(snap, &pe));
				}
				CloseHandle(snap);
			} else {
				out << "# launch_parent_lookup=snapshot_failed_err=" << GetLastError() << "\n";
			}
		}
		out << "# launch_parent_pid=" << parentPid << "\n";

		// Parent exe path. We need PROCESS_QUERY_LIMITED_INFORMATION (Win Vista+);
		// works across integrity levels for processes the current token can see.
		if (parentPid != 0) {
			HANDLE hParent = OpenProcess(PROCESS_QUERY_LIMITED_INFORMATION, FALSE, parentPid);
			if (hParent) {
				wchar_t parentExe[MAX_PATH] = {};
				DWORD len = MAX_PATH;
				if (QueryFullProcessImageNameW(hParent, 0, parentExe, &len)) {
					out << "# launch_parent_exe=" << WideToUtf8(parentExe) << "\n";
				} else {
					out << "# launch_parent_exe=QueryFullProcessImageNameW_failed_err="
					    << GetLastError() << "\n";
				}
				CloseHandle(hParent);
			} else {
				out << "# launch_parent_exe=OpenProcess_failed_err=" << GetLastError() << "\n";
			}
		}

		// --- Token: elevation, integrity, session, user SID --------------------
		HANDLE token = nullptr;
		if (OpenProcessToken(GetCurrentProcess(), TOKEN_QUERY, &token) && token) {
			// Integrity level. The SID's last sub-authority is the IL constant
			// (0x1000=Low, 0x2000=Medium, 0x3000=High, 0x4000=System). A
			// script-launched-from-elevated-shell process would show High;
			// a Start-menu launch normally shows Medium.
			DWORD size = 0;
			GetTokenInformation(token, TokenIntegrityLevel, nullptr, 0, &size);
			if (size > 0) {
				std::vector<BYTE> buf(size);
				if (GetTokenInformation(token, TokenIntegrityLevel, buf.data(), size, &size)) {
					auto* tml = reinterpret_cast<TOKEN_MANDATORY_LABEL*>(buf.data());
					DWORD subAuth = *GetSidSubAuthority(
						tml->Label.Sid,
						(DWORD)(UCHAR)(*GetSidSubAuthorityCount(tml->Label.Sid) - 1));
					const char* levelName = "unknown";
					if (subAuth < 0x1000)      levelName = "untrusted";
					else if (subAuth < 0x2000) levelName = "low";
					else if (subAuth < 0x3000) levelName = "medium";
					else if (subAuth < 0x4000) levelName = "high";
					else                       levelName = "system";
					char tmp[64];
					sprintf_s(tmp, "%s(0x%lx)", levelName, subAuth);
					out << "# launch_token_integrity=" << tmp << "\n";
				}
			}

			TOKEN_ELEVATION elevation{};
			DWORD elevSize = sizeof elevation;
			if (GetTokenInformation(token, TokenElevation, &elevation, elevSize, &elevSize)) {
				out << "# launch_token_elevated=" << (elevation.TokenIsElevated ? "yes" : "no") << "\n";
			}

			TOKEN_ELEVATION_TYPE elevType{};
			DWORD elevTypeSize = sizeof elevType;
			if (GetTokenInformation(token, TokenElevationType, &elevType, elevTypeSize, &elevTypeSize)) {
				const char* etName = "unknown";
				switch (elevType) {
					case TokenElevationTypeDefault: etName = "default"; break;
					case TokenElevationTypeFull:    etName = "full"; break;
					case TokenElevationTypeLimited: etName = "limited"; break;
				}
				out << "# launch_token_elevation_type=" << etName << "\n";
			}

			DWORD sessionId = 0;
			DWORD sessSize = sizeof sessionId;
			if (GetTokenInformation(token, TokenSessionId, &sessionId, sessSize, &sessSize)) {
				out << "# launch_session_id=" << sessionId << "\n";
			}

			// User SID — convert to string form for log readability.
			DWORD tuSize = 0;
			GetTokenInformation(token, TokenUser, nullptr, 0, &tuSize);
			if (tuSize > 0) {
				std::vector<BYTE> tuBuf(tuSize);
				if (GetTokenInformation(token, TokenUser, tuBuf.data(), tuSize, &tuSize)) {
					auto* tu = reinterpret_cast<TOKEN_USER*>(tuBuf.data());
					LPWSTR sidStr = nullptr;
					if (ConvertSidToStringSidW(tu->User.Sid, &sidStr)) {
						out << "# launch_user_sid=" << WideToUtf8(sidStr) << "\n";
						LocalFree(sidStr);
					}
				}
			}

			CloseHandle(token);
		} else {
			out << "# launch_token=OpenProcessToken_failed_err=" << GetLastError() << "\n";
		}

		// --- Window station + desktop ------------------------------------------
		// "WinSta0\Default" is the normal interactive desktop. A service-spawned
		// process or one in a non-interactive context would show different.
		HWINSTA hwinsta = GetProcessWindowStation();
		if (hwinsta) {
			wchar_t wsname[256] = {};
			DWORD needed = 0;
			if (GetUserObjectInformationW(hwinsta, UOI_NAME, wsname, sizeof wsname, &needed)) {
				out << "# launch_winstation=" << WideToUtf8(wsname) << "\n";
			}
		}
		HDESK hdesk = GetThreadDesktop(GetCurrentThreadId());
		if (hdesk) {
			wchar_t deskname[256] = {};
			DWORD needed = 0;
			if (GetUserObjectInformationW(hdesk, UOI_NAME, deskname, sizeof deskname, &needed)) {
				out << "# launch_desktop=" << WideToUtf8(deskname) << "\n";
			}
		}

		// --- Console + startup info --------------------------------------------
		HWND console = GetConsoleWindow();
		out << "# launch_console_attached=" << (console != nullptr ? "yes" : "no") << "\n";

		STARTUPINFOW si{};
		si.cb = sizeof si;
		GetStartupInfoW(&si);
		char sibuf[128];
		sprintf_s(sibuf, "0x%lx show_window=0x%x has_title=%d",
			si.dwFlags, (unsigned)si.wShowWindow, si.lpTitle ? 1 : 0);
		out << "# launch_startup_info=" << sibuf << "\n";
		if (si.lpTitle) {
			out << "# launch_startup_title=" << WideToUtf8(si.lpTitle) << "\n";
		}

		// --- Environment variable summary --------------------------------------
		// Don't dump full environment (may contain credentials in CI / non-
		// interactive contexts). Count total + dump only known-safe
		// VR/Steam-related ones.
		LPWCH env = GetEnvironmentStringsW();
		int envCount = 0;
		std::string vrEnvDump;
		if (env) {
			LPWCH p = env;
			while (*p) {
				envCount++;
				std::wstring entry(p);
				// Whitelist: log full key=value for vars matching VR-relevant
				// prefixes. Common ones that affect SteamVR / OpenVR runtime
				// behavior: VR_*, OPENVR_*, XR_*, STEAM_*, OVR_*. Excludes
				// things like PATH (huge) or generic user-secret containers.
				if (entry.rfind(L"VR_", 0) == 0
				 || entry.rfind(L"OPENVR_", 0) == 0
				 || entry.rfind(L"XR_", 0) == 0
				 || entry.rfind(L"STEAM_", 0) == 0
				 || entry.rfind(L"OVR_", 0) == 0
				 || entry.rfind(L"VRPATH", 0) == 0)
				{
					vrEnvDump += "# launch_vr_env=";
					vrEnvDump += WideToUtf8(p);
					vrEnvDump += "\n";
				}
				while (*p) p++;
				p++;
			}
			FreeEnvironmentStringsW(env);
		}
		out << "# launch_env_var_count=" << envCount << "\n";
		out << vrEnvDump;
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
		// (samplesInBuffer, watchdogResetCount, reject_reason, translationDiversity,
		// rotationDiversity, translationAxisRangesCm.{x,y,z}) are still v2 because
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

		// === Launch context ====================================================
		// Captures everything that could differ between a Start-menu launch and
		// a script-launched instance. Diffing these blocks across two log files
		// (one from each launch path) tells us exactly what's environmentally
		// different about the process — without this, we're guessing about
		// elevation / parent-process / cwd / session etc.
		//
		// Triggered by the 2026-05-03 user report: tracking consistently breaks
		// when SpaceCalibrator.exe is launched by quick.ps1 -Install's auto-
		// relaunch, but works fine when the same exe is launched from the
		// Start menu. Same exe, same driver, same registry profile. Some part
		// of the process context must differ; this block captures the candidates.
		//
		// All values are best-effort — Win32 calls that fail are logged with
		// the error code rather than skipped, so a missing field doesn't hide
		// the failure mode itself.
		WriteLaunchContextBanner(logFile);

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
