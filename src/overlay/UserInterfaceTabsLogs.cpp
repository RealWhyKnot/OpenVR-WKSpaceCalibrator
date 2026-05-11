#include "stdafx.h"
#include "Calibration.h"
#include "CalibrationMetrics.h"
#include "MotionRecording.h"

#include <string>
#include <vector>
#include <shellapi.h>
#include <shlobj_core.h>
#include <imgui/imgui.h>

namespace {

struct LogsPanelState {
	std::vector<spacecal::replay::LogFileEntry> files;
	bool listBuilt = false;
	int selectedIdx = -1;
	std::string copyHint;        // "Path copied" / errors -- transient feedback
	double copyHintExpireTime = 0.0;
};

LogsPanelState& LogsState() {
	static LogsPanelState s;
	return s;
}

void RebuildLogsList() {
	auto& s = LogsState();
	s.files = spacecal::replay::ListRecordings();
	s.listBuilt = true;
	if (s.selectedIdx >= (int)s.files.size()) s.selectedIdx = -1;
}

// Format file age relative to "now" using FILETIME math. We don't bother with
// localized strings -- "5 min ago" / "2 hours ago" / "3 days ago" is plenty
// detail for picking the right log out of a list.
std::string FormatFileAge(uint64_t mtimeFt) {
	FILETIME nowFt{};
	GetSystemTimeAsFileTime(&nowFt);
	const uint64_t now = ((uint64_t)nowFt.dwHighDateTime << 32) | nowFt.dwLowDateTime;
	if (mtimeFt > now) return "in the future"; // clock skew sentinel
	const uint64_t deltaTicks = now - mtimeFt; // 100-ns ticks
	const uint64_t deltaSec = deltaTicks / 10'000'000ull;
	char buf[64];
	if (deltaSec < 60)             snprintf(buf, sizeof buf, "%llus ago", (unsigned long long)deltaSec);
	else if (deltaSec < 3600)      snprintf(buf, sizeof buf, "%llum ago", (unsigned long long)(deltaSec / 60));
	else if (deltaSec < 86400)     snprintf(buf, sizeof buf, "%lluh ago", (unsigned long long)(deltaSec / 3600));
	else                           snprintf(buf, sizeof buf, "%llud ago", (unsigned long long)(deltaSec / 86400));
	return buf;
}

std::string FormatBytesShort(uint64_t n) {
	char buf[64];
	if (n >= (1ull << 20)) snprintf(buf, sizeof buf, "%.1f MB", (double)n / (double)(1ull << 20));
	else if (n >= (1ull << 10)) snprintf(buf, sizeof buf, "%.0f KB", (double)n / (double)(1ull << 10));
	else snprintf(buf, sizeof buf, "%llu B", (unsigned long long)n);
	return buf;
}

// Resolve the logs directory. ListRecordings discovers it internally for
// scanning; we want the parent path for the Explorer button. If the list is
// empty, derive from the first entry's full path; if THAT's empty too, fall
// back to the standard %LOCALAPPDATA%\Low\SpaceCalibrator\Logs path.
std::wstring GetLogsDirectory() {
	auto& s = LogsState();
	if (!s.files.empty()) {
		const auto& full = s.files.front().fullPath;
		const size_t lastSlash = full.find_last_of(L"\\/");
		if (lastSlash != std::wstring::npos) {
			return full.substr(0, lastSlash);
		}
	}
	// Fallback: standard Windows AppDataLow path. This matches what
	// Metrics::enableLogs writes into. We don't take a hard dependency on
	// shlobj here -- the path is stable and well-documented for the lifetime
	// of this app.
	wchar_t* appDataLow = nullptr;
	if (SUCCEEDED(SHGetKnownFolderPath(FOLDERID_LocalAppDataLow, 0, nullptr, &appDataLow)) && appDataLow) {
		std::wstring path(appDataLow);
		CoTaskMemFree(appDataLow);
		path += L"\\SpaceCalibrator\\Logs";
		return path;
	}
	return L"";
}

// Copy a UTF-16 string to the Windows clipboard. Returns true on success.
bool CopyToClipboardW(const std::wstring& text) {
	if (!OpenClipboard(nullptr)) return false;
	EmptyClipboard();
	const size_t bytes = (text.size() + 1) * sizeof(wchar_t);
	HGLOBAL h = GlobalAlloc(GMEM_MOVEABLE, bytes);
	if (!h) { CloseClipboard(); return false; }
	if (auto* buf = (wchar_t*)GlobalLock(h)) {
		memcpy(buf, text.c_str(), bytes);
		GlobalUnlock(h);
		SetClipboardData(CF_UNICODETEXT, h);
	} else {
		GlobalFree(h);
		CloseClipboard();
		return false;
	}
	CloseClipboard();
	return true;
}

} // namespace

// === Logs panel ============================================================
// User-friendly view of the debug-log files written when "Enable debug logs"
// is on. The previous Recordings tab was dev-tooling -- it loaded a CSV and
// replayed it through the calibration math, which only made sense for the
// developer iterating on a fix. Most users just want a way to find and copy
// log files to attach to a bug report. So we surface the file list, a button
// to open the logs folder in Explorer, and a per-row copy-path action.
//
// The replay machinery (spacecal::replay::LoadRecording / RunReplay) is still
// in MotionRecording.cpp for the standalone replay CLI tool and the test
// suite; it just isn't surfaced in the overlay UI anymore.
void CCal_DrawLogsPanel() {
	auto& state = LogsState();

	ImGui::TextWrapped(
		"Debug logs are CSV files written one row per calibration tick. They're the "
		"first thing to attach to a bug report -- the team can replay the captured "
		"poses against the live math to reproduce what you saw.");
	ImGui::Spacing();

	// Recording status + toggle. The log file IS the recording -- there's no
	// separate "start/stop recording" notion. Putting the toggle here means
	// the user can flip logging on right where they're managing the log
	// files, instead of having to dig into Settings.
	ImGui::Checkbox("Enable debug logging", &Metrics::enableLogs);
	if (ImGui::IsItemHovered()) {
		ImGui::SetTooltip("Write a per-tick CSV of calibration state to %%LocalAppDataLow%%\\SpaceCalibrator\\Logs\\\n"
		                  "while this is on. The new log shows up in the list below as soon as the next calibration tick fires.");
	}
	ImGui::SameLine();
	if (Metrics::enableLogs) {
		ImGui::TextColored(ImVec4(0.45f, 0.85f, 0.45f, 1.0f),
			" -- a fresh CSV is being written this session");
	} else {
		ImGui::TextDisabled(" (off)");
	}

	// Drift-subsystem state dump. Captures rec A rest detector, rec C
	// recovery delta buffer, rec F chi-square detector state to the log
	// in a single batch of [drift][state-dump] annotations. Useful to
	// attach to a bug report alongside the rolling annotations.
	ImGui::SameLine();
	if (ImGui::Button("Dump drift state")) {
		DumpDriftSubsystemState();
	}
	if (ImGui::IsItemHovered()) {
		ImGui::SetTooltip("Write a one-shot snapshot of the drift-correction subsystems\n"
			"(rest-locked yaw rec A, predictive recovery rec C, chi-square rec F)\n"
			"to the current debug log. Includes per-device rest state, recovery\n"
			"event ring contents, and chi-square detector counters. Greppable\n"
			"by [drift][state-dump] for triage.");
	}

	ImGui::Spacing();
	ImGui::Separator();

	ImGui::TextDisabled("Log files");
	ImGui::SameLine();
	if (ImGui::SmallButton("Refresh##logs")) {
		RebuildLogsList();
	}
	ImGui::SameLine();
	if (ImGui::SmallButton("Open folder##logs")) {
		const std::wstring dir = GetLogsDirectory();
		if (!dir.empty()) {
			ShellExecuteW(nullptr, L"open", dir.c_str(), nullptr, nullptr, SW_SHOWNORMAL);
		}
	}

	if (!state.listBuilt) RebuildLogsList();

	if (state.files.empty()) {
		ImGui::TextDisabled("(No log files found in the Logs directory.)");
	} else {
		// Selectable list of log files, newest first. Each row has a small
		// Delete button on the right; clicking it removes the file from disk
		// and rebuilds the list. Selection state stays consistent (clamped
		// to the new file count) so the action buttons below don't reference
		// a stale index.
		const float listHeight = ImGui::GetTextLineHeightWithSpacing() * 8.0f;
		if (ImGui::BeginChild("##logs_list",
				ImVec2(0, listHeight), ImGuiChildFlags_Border)) {
			for (int i = 0; i < (int)state.files.size(); ++i) {
				const auto& f = state.files[i];
				char label[512];
				snprintf(label, sizeof label, "%s   (%s, %s)",
					f.name.c_str(),
					FormatBytesShort(f.sizeBytes).c_str(),
					FormatFileAge(f.mtimeFileTime).c_str());

				// Reserve room for the Delete button on the right edge so
				// the Selectable doesn't fill the whole line.
				const float deleteBtnWidth = 70.0f;
				const float rowWidth = ImGui::GetContentRegionAvail().x - deleteBtnWidth - 8.0f;
				if (ImGui::Selectable(label, state.selectedIdx == i, 0, ImVec2(rowWidth, 0))) {
					state.selectedIdx = i;
				}
				ImGui::SameLine();
				char delId[64];
				snprintf(delId, sizeof delId, "Delete##log%d", i);
				if (ImGui::SmallButton(delId)) {
					// Best-effort delete -- DeleteFileW returns non-zero on
					// success. If it fails (file in use, permission), surface
					// the error in the same transient hint slot the Copy path
					// button uses.
					BOOL ok = DeleteFileW(state.files[i].fullPath.c_str());
					if (ok) {
						state.copyHint = "Deleted " + state.files[i].name;
					} else {
						state.copyHint = "Could not delete (file may be in use)";
					}
					state.copyHintExpireTime = ImGui::GetTime() + 2.5;
					RebuildLogsList();
					if (state.selectedIdx >= (int)state.files.size()) {
						state.selectedIdx = -1;
					}
					break; // list mutated, bail this iteration
				}
			}
		}
		ImGui::EndChild();
	}

	ImGui::Spacing();

	const bool haveSelection = state.selectedIdx >= 0
		&& state.selectedIdx < (int)state.files.size();
	ImGui::BeginDisabled(!haveSelection);
	if (ImGui::Button("Open selected##logs")) {
		ShellExecuteW(nullptr, L"open",
			state.files[state.selectedIdx].fullPath.c_str(),
			nullptr, nullptr, SW_SHOWNORMAL);
	}
	ImGui::SameLine();
	if (ImGui::Button("Copy path##logs")) {
		if (CopyToClipboardW(state.files[state.selectedIdx].fullPath)) {
			state.copyHint = "Path copied to clipboard";
		} else {
			state.copyHint = "Failed to copy path (clipboard busy?)";
		}
		state.copyHintExpireTime = ImGui::GetTime() + 2.5;
	}
	ImGui::EndDisabled();

	// Transient feedback row -- shown for ~2.5s after a clipboard action.
	if (!state.copyHint.empty() && ImGui::GetTime() < state.copyHintExpireTime) {
		ImGui::SameLine();
		ImGui::PushStyleColor(ImGuiCol_Text, ImGui::GetStyleColorVec4(ImGuiCol_TextDisabled));
		ImGui::TextUnformatted(state.copyHint.c_str());
		ImGui::PopStyleColor();
	} else if (!state.copyHint.empty()) {
		state.copyHint.clear();
	}
}
