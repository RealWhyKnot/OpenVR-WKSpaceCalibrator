#define _CRT_SECURE_NO_DEPRECATE
#include "Logging.h"

#include <chrono>
#include <string>
#include <vector>

#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <shlobj.h>      // SHGetKnownFolderPath, FOLDERID_LocalAppDataLow
#include <objbase.h>     // CoTaskMemFree

FILE *LogFile;

namespace {

// Mirrors the overlay's ClearOldLogs (CalibrationMetrics.cpp:260): nuke any
// driver_log.*.txt older than 24 hours so the LocalAppDataLow\SpaceCalibrator
// \Logs directory doesn't accumulate forever. Files newer than the cutoff are
// retained so a triage session can grab the recent few.
void ClearOldDriverLogs(const std::wstring& dir)
{
	std::wstring search = dir + L"\\driver_log.*.txt";
	WIN32_FIND_DATAW find_data{};

	SYSTEMTIME stNow{};
	FILETIME   ftNow{};
	GetSystemTime(&stNow);
	SystemTimeToFileTime(&stNow, &ftNow);

	ULARGE_INTEGER nowQ;
	nowQ.HighPart = ftNow.dwHighDateTime;
	nowQ.LowPart  = ftNow.dwLowDateTime;
	// 24h in 100ns units
	const uint64_t cutoff = nowQ.QuadPart - (uint64_t)(24LL * 3600LL * 10LL * 1000LL * 1000LL);

	HANDLE h = FindFirstFileW(search.c_str(), &find_data);
	if (h == INVALID_HANDLE_VALUE) return;
	do {
		ULARGE_INTEGER fileQ;
		fileQ.HighPart = find_data.ftLastWriteTime.dwHighDateTime;
		fileQ.LowPart  = find_data.ftLastWriteTime.dwLowDateTime;
		if (fileQ.QuadPart < cutoff) {
			std::wstring path = dir + L"\\" + find_data.cFileName;
			DeleteFileW(path.c_str());
		}
	} while (FindNextFileW(h, &find_data));
	FindClose(h);
}

// Build %LocalAppDataLow%\SpaceCalibrator\Logs\driver_log.<date>T<time>.txt.
// Returns the wide path on success, empty on any failure (caller falls back
// to the legacy cwd path so a missing AppDataLow doesn't lose logs entirely).
std::wstring BuildLogPath()
{
	PWSTR rootRaw = nullptr;
	if (S_OK != SHGetKnownFolderPath(FOLDERID_LocalAppDataLow, 0, nullptr, &rootRaw)) {
		if (rootRaw) CoTaskMemFree(rootRaw);
		return {};
	}
	std::wstring root(rootRaw);
	CoTaskMemFree(rootRaw);

	std::wstring dir = root + L"\\SpaceCalibrator";
	if (!CreateDirectoryW(dir.c_str(), nullptr) && GetLastError() != ERROR_ALREADY_EXISTS) return {};
	dir += L"\\Logs";
	if (!CreateDirectoryW(dir.c_str(), nullptr) && GetLastError() != ERROR_ALREADY_EXISTS) return {};

	ClearOldDriverLogs(dir);

	SYSTEMTIME now{};
	GetSystemTime(&now);

	const int dateLen = GetDateFormatEx(LOCALE_NAME_INVARIANT, 0, &now, L"yyyy-MM-dd", nullptr, 0, nullptr);
	const int timeLen = GetTimeFormatEx(LOCALE_NAME_INVARIANT, 0, &now, L"HH-mm-ss", nullptr, 0);
	if (dateLen <= 0 || timeLen <= 0) return {};

	std::vector<wchar_t> date(dateLen);
	std::vector<wchar_t> time(timeLen);
	if (!GetDateFormatEx(LOCALE_NAME_INVARIANT, 0, &now, L"yyyy-MM-dd", date.data(), dateLen, nullptr)) return {};
	if (!GetTimeFormatEx(LOCALE_NAME_INVARIANT, 0, &now, L"HH-mm-ss", time.data(), timeLen)) return {};

	// Filename: driver_log.<date>T<time>.txt -- distinct prefix from the
	// overlay's spacecal_log.* so the two sit side-by-side in the same dir
	// without ClearOldLogs cross-deleting them. The overlay's ClearOldLogs
	// matches "spacecal_log.*.txt" only; ours matches "driver_log.*.txt"
	// only.
	std::wstring path = dir + L"\\driver_log." + date.data() + L"T" + time.data() + L".txt";
	return path;
}

} // namespace

void OpenLogFile()
{
	// Prefer the LocalAppDataLow path so the user's diagnostic flow ("diff
	// overlay log + driver log side-by-side") works without having to hunt
	// for the driver log under whatever cwd vrserver happened to inherit
	// from Steam (typically the Steam install dir, sometimes non-writable).
	std::wstring path = BuildLogPath();
	if (!path.empty()) {
		// _wfopen takes a wide path, which lets us cope with usernames
		// containing non-ASCII characters that fopen would mangle.
		LogFile = _wfopen(path.c_str(), L"a");
		if (LogFile) return;
	}

	// Fallback: legacy behavior. Better than nothing if SHGetKnownFolderPath
	// isn't available or AppDataLow isn't writable.
	LogFile = fopen("space_calibrator_driver.log", "a");
	if (!LogFile) {
		LogFile = stderr;
	}
}

tm TimeForLog()
{
	auto now = std::chrono::system_clock::now();
	auto nowTime = std::chrono::system_clock::to_time_t(now);
	tm value;
	auto tm = localtime_s(&value, &nowTime);
	return value;
}

void LogFlush()
{
	fflush(LogFile);
}
