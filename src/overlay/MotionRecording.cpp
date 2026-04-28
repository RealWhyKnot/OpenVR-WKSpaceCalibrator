#include "stdafx.h"
#include "MotionRecording.h"

#include <windows.h>
#include <shlobj_core.h>

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <unordered_map>

namespace spacecal::replay {

namespace {

// Strict split-on-comma. Mirrors the live CSV writer (no quoting, no embedded
// newlines). Identical to the parser in tools/replay/main.cpp; deliberately
// duplicated rather than shared so the in-app version can evolve without
// dragging the standalone CLI's link line along.
std::vector<std::string> SplitCsv(const std::string& line) {
	std::vector<std::string> out;
	std::string cur;
	for (char c : line) {
		if (c == ',') { out.push_back(cur); cur.clear(); }
		else { cur.push_back(c); }
	}
	out.push_back(cur);
	return out;
}

void RTrim(std::string& s) {
	while (!s.empty() && (s.back() == '\r' || s.back() == '\n' ||
		s.back() == ' ' || s.back() == '\t')) {
		s.pop_back();
	}
}

bool ReadDouble(const std::string& s, double& out) {
	if (s.empty()) return false;
	char* end = nullptr;
	const double v = std::strtod(s.c_str(), &end);
	if (end == s.c_str()) return false;
	out = v;
	return true;
}

int ColIndex(const std::unordered_map<std::string, int>& cols, const char* name) {
	const auto it = cols.find(name);
	return (it == cols.end()) ? -1 : it->second;
}

// Strip a leading "# key=" prefix from an annotation line. Returns the value
// part (post-=) on match, or empty string when the line doesn't have that key.
std::string ParseHeaderKv(const std::string& line, const char* key) {
	const std::string prefix = std::string("# ") + key + "=";
	if (line.rfind(prefix, 0) != 0) return {};
	return line.substr(prefix.size());
}

// %LocalAppDataLow%\SpaceCalibrator\Logs — same path the live logger writes to.
std::wstring GetLogsDir() {
	PWSTR rootPath = nullptr;
	if (SHGetKnownFolderPath(FOLDERID_LocalAppDataLow, 0, nullptr, &rootPath) != S_OK) {
		if (rootPath) CoTaskMemFree(rootPath);
		return {};
	}
	std::wstring p(rootPath);
	CoTaskMemFree(rootPath);
	p += LR"(\SpaceCalibrator\Logs)";
	return p;
}

} // namespace

LoadedRecording LoadRecording(const std::string& path) {
	LoadedRecording rec;
	rec.sourcePath = path;

	std::ifstream in(path);
	if (!in) {
		rec.error = "Failed to open recording: " + path;
		return rec;
	}

	std::string line;
	bool sawVersionBanner = false;
	std::vector<std::string> header;

	while (std::getline(in, line)) {
		RTrim(line);
		if (line.empty()) continue;
		if (line.rfind("#", 0) == 0) {
			if (line.find("spacecal_log_v2") != std::string::npos) {
				sawVersionBanner = true;
				continue;
			}
			if (line.find("spacecal_log_v") != std::string::npos) {
				rec.error = "Recording is not v2 (saw '" + line + "'). Replay needs v2.";
				return rec;
			}
			// Pull metadata KVs out of the header annotations the live logger emits.
			std::string v;
			if (!(v = ParseHeaderKv(line, "build_stamp")).empty())            rec.meta.buildStamp = v;
			else if (!(v = ParseHeaderKv(line, "build_channel")).empty())     rec.meta.buildChannel = v;
			else if (!(v = ParseHeaderKv(line, "hmd_tracking_system")).empty()) rec.meta.hmdTrackingSystem = v;
			else if (!(v = ParseHeaderKv(line, "hmd_model")).empty())         rec.meta.hmdModel = v;
			else if (!(v = ParseHeaderKv(line, "hmd_serial")).empty())        rec.meta.hmdSerial = v;
			else if (!(v = ParseHeaderKv(line, "steamvr_runtime_path")).empty()) rec.meta.steamvrRuntime = v;
			else if (!(v = ParseHeaderKv(line, "windows")).empty())           rec.meta.windowsVersion = v;
			else if (!(v = ParseHeaderKv(line, "logical_processors")).empty()) {
				rec.meta.logicalProcessors = std::atoi(v.c_str());
			}
			continue;
		}
		header = SplitCsv(line);
		break;
	}

	if (header.empty()) {
		rec.error = "Recording has no column header row";
		return rec;
	}
	if (!sawVersionBanner) {
		rec.error = "Recording missing '# spacecal_log_v2' banner — refusing to replay (v1 logs lack raw poses).";
		return rec;
	}

	std::unordered_map<std::string, int> cols;
	for (size_t i = 0; i < header.size(); ++i) cols[header[i]] = (int)i;

	const int idxTimestamp = ColIndex(cols, "Timestamp");
	const int idxRefTx = ColIndex(cols, "ref_tx");
	const int idxRefTy = ColIndex(cols, "ref_ty");
	const int idxRefTz = ColIndex(cols, "ref_tz");
	const int idxRefQw = ColIndex(cols, "ref_qw");
	const int idxRefQx = ColIndex(cols, "ref_qx");
	const int idxRefQy = ColIndex(cols, "ref_qy");
	const int idxRefQz = ColIndex(cols, "ref_qz");
	const int idxTgtTx = ColIndex(cols, "tgt_tx");
	const int idxTgtTy = ColIndex(cols, "tgt_ty");
	const int idxTgtTz = ColIndex(cols, "tgt_tz");
	const int idxTgtQw = ColIndex(cols, "tgt_qw");
	const int idxTgtQx = ColIndex(cols, "tgt_qx");
	const int idxTgtQy = ColIndex(cols, "tgt_qy");
	const int idxTgtQz = ColIndex(cols, "tgt_qz");
	const int idxTickPhase = ColIndex(cols, "tick_phase");

	const int required[] = {
		idxRefTx, idxRefTy, idxRefTz, idxRefQw, idxRefQx, idxRefQy, idxRefQz,
		idxTgtTx, idxTgtTy, idxTgtTz, idxTgtQw, idxTgtQx, idxTgtQy, idxTgtQz,
	};
	for (int idx : required) {
		if (idx < 0) {
			rec.error = "Recording is missing one of the required raw-pose columns (ref_t{x,y,z}, ref_q{w,x,y,z}, tgt_*).";
			return rec;
		}
	}

	rec.rows.reserve(1024); // avoid early reallocations on typical recordings.

	while (std::getline(in, line)) {
		RTrim(line);
		if (line.empty()) continue;
		if (line.rfind("#", 0) == 0) continue;

		const auto fields = SplitCsv(line);
		if ((int)fields.size() < (int)header.size()) {
			// Likely a partial row (log rotated mid-tick). Skip silently — the
			// summary's "rows replayed" already tells the user how many were used.
			continue;
		}

		ReplayRow row;
		double ts = 0.0;
		if (idxTimestamp >= 0) ReadDouble(fields[idxTimestamp], ts);
		row.timestamp = ts;

		double rt[3], rq[4], tt[3], tq[4];
		if (!ReadDouble(fields[idxRefTx], rt[0]) ||
			!ReadDouble(fields[idxRefTy], rt[1]) ||
			!ReadDouble(fields[idxRefTz], rt[2]) ||
			!ReadDouble(fields[idxRefQw], rq[0]) ||
			!ReadDouble(fields[idxRefQx], rq[1]) ||
			!ReadDouble(fields[idxRefQy], rq[2]) ||
			!ReadDouble(fields[idxRefQz], rq[3]) ||
			!ReadDouble(fields[idxTgtTx], tt[0]) ||
			!ReadDouble(fields[idxTgtTy], tt[1]) ||
			!ReadDouble(fields[idxTgtTz], tt[2]) ||
			!ReadDouble(fields[idxTgtQw], tq[0]) ||
			!ReadDouble(fields[idxTgtQx], tq[1]) ||
			!ReadDouble(fields[idxTgtQy], tq[2]) ||
			!ReadDouble(fields[idxTgtQz], tq[3])) {
			continue;
		}

		Eigen::Quaterniond rqQ(rq[0], rq[1], rq[2], rq[3]);
		Eigen::Quaterniond tqQ(tq[0], tq[1], tq[2], tq[3]);
		// Zero-quaternion sentinel = "device wasn't tracking this tick" — ignore.
		if (rqQ.norm() < 1e-9 || tqQ.norm() < 1e-9) continue;
		rqQ.normalize();
		tqQ.normalize();

		row.ref.rot = rqQ.toRotationMatrix();
		row.ref.trans = Eigen::Vector3d(rt[0], rt[1], rt[2]);
		row.target.rot = tqQ.toRotationMatrix();
		row.target.trans = Eigen::Vector3d(tt[0], tt[1], tt[2]);

		if (idxTickPhase >= 0) row.tickPhase = fields[idxTickPhase];

		rec.rows.push_back(std::move(row));
	}

	return rec;
}

ReplayResult RunReplay(const LoadedRecording& rec, const ReplayOptions& opts) {
	ReplayResult res;
	if (!rec.error.empty()) {
		res.error = rec.error;
		return res;
	}
	if (rec.rows.empty()) {
		res.error = "Recording has no replayable rows.";
		return res;
	}

	CalibrationCalc calc;
	calc.enableStaticRecalibration = false;
	calc.lockRelativePosition = false;

	res.trace.reserve(rec.rows.size());

	for (const auto& row : rec.rows) {
		Sample s(row.ref, row.target, row.timestamp);
		calc.PushSample(s);

		ReplayTickResult tick;
		tick.timestamp = row.timestamp;

		if (opts.continuous) {
			bool lerp = false;
			const bool ok = calc.ComputeIncremental(lerp, opts.threshold,
				opts.maxRelError, opts.ignoreOutliers);
			tick.accepted = ok;
			if (ok) ++res.accepts; else ++res.rejects;
			tick.consecutiveRejections = calc.m_consecutiveRejections;
			// CalibrationCalc doesn't expose its post-validate prior error
			// directly, so use the public Validate path against the current
			// estimate — only valid after the first accept, otherwise we
			// leave the error fields at 0.
			if (calc.isValid()) {
				double err = 0.0;
				Eigen::Vector3d offset;
				if (calc.ValidateCalibration(calc.Transformation(), &err, &offset)) {
					tick.currentCalErrMm = err * 1000.0;
				}
			}
			// Tag mirrors CalibrationCalc::m_rejectReasonTag, which is private.
			// We approximate from the public state: empty when the tick accepted,
			// "rejected" otherwise. The detailed tag is already streamed to the
			// debug log if logging is on; this trace is the lighter-weight UI view.
			if (!ok) tick.rejectReason = "rejected";
		} else {
			// Oneshot mode just keeps appending samples. The single Compute below
			// runs after the loop. Per-tick trace stays as samples-only.
		}

		res.trace.push_back(std::move(tick));
	}

	if (!opts.continuous) {
		const bool ok = calc.ComputeOneshot(opts.ignoreOutliers);
		if (ok) ++res.accepts; else ++res.rejects;
	}

	res.rowsReplayed = (int)rec.rows.size();
	res.watchdogResets = calc.m_watchdogResets;
	res.finalTransformValid = calc.isValid();
	if (calc.isValid()) {
		res.finalTransform = calc.Transformation();
		double err = 0.0;
		Eigen::Vector3d offset;
		if (calc.ValidateCalibration(calc.Transformation(), &err, &offset)) {
			res.finalErrorMm = err * 1000.0;
		}
	}
	res.succeeded = true;
	return res;
}

std::vector<LogFileEntry> ListRecordings() {
	std::vector<LogFileEntry> out;
	const std::wstring dir = GetLogsDir();
	if (dir.empty()) return out;

	const std::wstring pattern = dir + L"\\spacecal_log.*.txt";
	WIN32_FIND_DATAW find{};
	HANDLE h = FindFirstFileW(pattern.c_str(), &find);
	if (h == INVALID_HANDLE_VALUE) return out;

	do {
		if (find.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY) continue;

		LogFileEntry entry;
		// File name only, in UTF-8 for ImGui display.
		const int n = WideCharToMultiByte(CP_UTF8, 0, find.cFileName, -1, nullptr, 0, nullptr, nullptr);
		if (n > 1) {
			entry.name.resize(n - 1);
			WideCharToMultiByte(CP_UTF8, 0, find.cFileName, -1, entry.name.data(), n, nullptr, nullptr);
		}
		entry.fullPath = dir + L"\\" + find.cFileName;
		entry.sizeBytes = ((uint64_t)find.nFileSizeHigh << 32) | (uint64_t)find.nFileSizeLow;
		entry.mtimeFileTime = ((uint64_t)find.ftLastWriteTime.dwHighDateTime << 32)
			| (uint64_t)find.ftLastWriteTime.dwLowDateTime;
		out.push_back(std::move(entry));
	} while (FindNextFileW(h, &find));
	FindClose(h);

	// Newest-first so the recording the user just made is at the top.
	std::sort(out.begin(), out.end(), [](const LogFileEntry& a, const LogFileEntry& b) {
		return a.mtimeFileTime > b.mtimeFileTime;
	});
	return out;
}

} // namespace spacecal::replay
