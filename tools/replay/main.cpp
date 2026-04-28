// Offline replay harness for spacecal_log_v2 captures.
//
// Usage:
//   spacecal_replay <log.csv> [--mode oneshot|continuous] [--threshold 1.5]
//                              [--max-rel-error 0.005] [--ignore-outliers]
//
// Re-feeds raw reference + target poses from a captured CSV log into a fresh
// CalibrationCalc instance and reports what the calibration produces. Useful
// for asking "what would changing this threshold do to this user's data?"
// without needing the user's hardware. The CSV must be v2 (the header line
// begins with `# spacecal_log_v2`); v1 logs lack the raw-pose columns.

// <iostream> first — CalibrationCalc.h transitively pulls Calibration.h via
// CalibrationCalc.cpp, but here we include CalibrationCalc.h directly which
// also exposes std::cerr usage in headers; pull it in explicitly.
#include <iostream>

#include "CalibrationCalc.h"

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

namespace {

struct Options {
	std::string csvPath;
	enum class Mode { Oneshot, Continuous } mode = Mode::Continuous;
	double threshold = 1.5;
	double maxRelError = 0.005;
	bool ignoreOutliers = false;
};

void PrintUsage(const char* argv0) {
	std::fprintf(stderr,
		"Usage: %s <log.csv> [--mode oneshot|continuous] [--threshold N] [--max-rel-error N] [--ignore-outliers]\n",
		argv0);
}

bool ParseArgs(int argc, char** argv, Options& out) {
	for (int i = 1; i < argc; i++) {
		std::string a = argv[i];
		auto needNext = [&](const char* name) -> const char* {
			if (i + 1 >= argc) {
				std::fprintf(stderr, "%s: missing value for %s\n", argv[0], name);
				return nullptr;
			}
			return argv[++i];
		};

		if (a == "--mode") {
			const char* v = needNext("--mode");
			if (!v) return false;
			std::string mv(v);
			if (mv == "oneshot") out.mode = Options::Mode::Oneshot;
			else if (mv == "continuous") out.mode = Options::Mode::Continuous;
			else {
				std::fprintf(stderr, "Unknown --mode '%s' (expected oneshot|continuous)\n", v);
				return false;
			}
		} else if (a == "--threshold") {
			const char* v = needNext("--threshold");
			if (!v) return false;
			out.threshold = std::strtod(v, nullptr);
		} else if (a == "--max-rel-error") {
			const char* v = needNext("--max-rel-error");
			if (!v) return false;
			out.maxRelError = std::strtod(v, nullptr);
		} else if (a == "--ignore-outliers") {
			out.ignoreOutliers = true;
		} else if (a == "-h" || a == "--help") {
			PrintUsage(argv[0]);
			return false;
		} else if (!a.empty() && a[0] == '-') {
			std::fprintf(stderr, "Unknown option '%s'\n", a.c_str());
			PrintUsage(argv[0]);
			return false;
		} else {
			if (!out.csvPath.empty()) {
				std::fprintf(stderr, "Multiple input files supplied; only one is allowed\n");
				return false;
			}
			out.csvPath = a;
		}
	}

	if (out.csvPath.empty()) {
		PrintUsage(argv[0]);
		return false;
	}
	return true;
}

// Tiny CSV splitter. The metrics writer always emits plain numeric/identifier
// fields and never embeds commas, quotes, or newlines, so a strict split-on-comma
// is sufficient. We deliberately don't add quote handling to avoid masking a
// future log-format change that we should notice.
std::vector<std::string> SplitCsv(const std::string& line) {
	std::vector<std::string> out;
	std::string cur;
	for (char c : line) {
		if (c == ',') {
			out.push_back(cur);
			cur.clear();
		} else {
			cur.push_back(c);
		}
	}
	out.push_back(cur);
	return out;
}

// Strip CR (the metrics writer uses "\n", but a user might re-save the file
// through a tool that adds CRLF). Trim trailing whitespace too.
void RTrim(std::string& s) {
	while (!s.empty() && (s.back() == '\r' || s.back() == '\n' ||
		s.back() == ' ' || s.back() == '\t')) {
		s.pop_back();
	}
}

bool ReadDouble(const std::string& s, double& out) {
	if (s.empty()) return false;
	char* end = nullptr;
	double v = std::strtod(s.c_str(), &end);
	if (end == s.c_str()) return false;
	out = v;
	return true;
}

struct ReplayRow {
	double timestamp = 0.0;
	Pose ref;
	Pose target;
	std::string tickPhase;
};

// Look up a column index by name; returns -1 if missing.
int ColIndex(const std::unordered_map<std::string, int>& cols, const char* name) {
	auto it = cols.find(name);
	return (it == cols.end()) ? -1 : it->second;
}

bool LoadCsv(const std::string& path, std::vector<ReplayRow>& rows, std::string& errOut) {
	std::ifstream in(path);
	if (!in) {
		errOut = "Failed to open input file: " + path;
		return false;
	}

	std::string line;
	bool sawVersionBanner = false;
	std::vector<std::string> header;

	// Walk past leading comment / annotation lines until we find the column header.
	// The format-version banner is the very first line by convention.
	while (std::getline(in, line)) {
		RTrim(line);
		if (line.empty()) continue;
		if (line.rfind("#", 0) == 0) {
			if (line.find("spacecal_log_v2") != std::string::npos) {
				sawVersionBanner = true;
			} else if (line.find("spacecal_log_v") != std::string::npos) {
				errOut = "Input log is not v2 (saw '" + line + "'). Replay requires v2.";
				return false;
			}
			continue;
		}
		header = SplitCsv(line);
		break;
	}

	if (header.empty()) {
		errOut = "Input file has no header row";
		return false;
	}
	if (!sawVersionBanner) {
		errOut = "Input file does not begin with '# spacecal_log_v2' — refusing to replay a v1 (no raw poses) capture.";
		return false;
	}

	std::unordered_map<std::string, int> cols;
	for (size_t i = 0; i < header.size(); i++) {
		cols[header[i]] = (int)i;
	}

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
			errOut = "Header is missing one of the required raw-pose columns (ref_t{x,y,z}, ref_q{w,x,y,z}, tgt_*).";
			return false;
		}
	}

	int rowNum = 0;
	while (std::getline(in, line)) {
		rowNum++;
		RTrim(line);
		if (line.empty()) continue;
		if (line.rfind("#", 0) == 0) continue;

		auto fields = SplitCsv(line);
		if ((int)fields.size() < (int)header.size()) {
			// Allow a trailing blank field but otherwise reject — partial rows
			// most likely indicate the log was rotated mid-tick.
			std::fprintf(stderr, "warning: row %d has %zu fields (expected %zu) — skipping\n",
				rowNum, fields.size(), header.size());
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
			!ReadDouble(fields[idxTgtQz], tq[3]))
		{
			std::fprintf(stderr, "warning: row %d has unparsable pose fields — skipping\n", rowNum);
			continue;
		}

		// A pose with a zero-length quaternion is a sentinel for "device not
		// tracking" (the live capture happens on devicePoses[], which can be all
		// zeros for a missing slot). Don't feed those into the calc.
		Eigen::Quaterniond rqQ(rq[0], rq[1], rq[2], rq[3]);
		Eigen::Quaterniond tqQ(tq[0], tq[1], tq[2], tq[3]);
		if (rqQ.norm() < 1e-9 || tqQ.norm() < 1e-9) continue;
		rqQ.normalize();
		tqQ.normalize();

		row.ref.rot = rqQ.toRotationMatrix();
		row.ref.trans = Eigen::Vector3d(rt[0], rt[1], rt[2]);
		row.target.rot = tqQ.toRotationMatrix();
		row.target.trans = Eigen::Vector3d(tt[0], tt[1], tt[2]);

		if (idxTickPhase >= 0) row.tickPhase = fields[idxTickPhase];

		rows.push_back(row);
	}

	return true;
}

void PrintCalibration(const Eigen::AffineCompact3d& xform) {
	const auto t = xform.translation();
	const auto eulerDeg = xform.rotation().eulerAngles(2, 1, 0) * (180.0 / EIGEN_PI);
	std::printf("  translation (m):   x=%.6f  y=%.6f  z=%.6f\n", t.x(), t.y(), t.z());
	std::printf("  rotation (deg ZYX): yaw=%.4f  pitch=%.4f  roll=%.4f\n",
		eulerDeg.x(), eulerDeg.y(), eulerDeg.z());
}

} // namespace

int main(int argc, char** argv) {
	Options opts;
	if (!ParseArgs(argc, argv, opts)) return 1;

	std::vector<ReplayRow> rows;
	std::string err;
	if (!LoadCsv(opts.csvPath, rows, err)) {
		std::fprintf(stderr, "Error: %s\n", err.c_str());
		return 2;
	}

	std::printf("Loaded %zu replayable ticks from %s\n", rows.size(), opts.csvPath.c_str());
	if (rows.empty()) {
		std::fprintf(stderr, "No usable rows in input — nothing to do.\n");
		return 3;
	}

	CalibrationCalc calc;
	calc.enableStaticRecalibration = false;
	calc.lockRelativePosition = false;

	int accepts = 0;
	int rejects = 0;
	int oneshotRuns = 0;

	for (const auto& row : rows) {
		Sample s(row.ref, row.target, row.timestamp);
		calc.PushSample(s);

		if (opts.mode == Options::Mode::Continuous) {
			bool lerp = false;
			bool ok = calc.ComputeIncremental(lerp, opts.threshold, opts.maxRelError, opts.ignoreOutliers);
			if (ok) accepts++; else rejects++;
		}
	}

	if (opts.mode == Options::Mode::Oneshot) {
		bool ok = calc.ComputeOneshot(opts.ignoreOutliers);
		oneshotRuns = 1;
		if (ok) accepts++; else rejects++;
	}

	std::printf("\nReplay summary\n");
	std::printf("  mode:            %s\n",
		opts.mode == Options::Mode::Oneshot ? "oneshot" : "continuous");
	std::printf("  threshold:       %.6f\n", opts.threshold);
	std::printf("  max rel error:   %.6f\n", opts.maxRelError);
	std::printf("  ignore outliers: %s\n", opts.ignoreOutliers ? "yes" : "no");
	std::printf("  ticks replayed:  %zu\n", rows.size());
	if (opts.mode == Options::Mode::Continuous) {
		std::printf("  accepts:         %d\n", accepts);
		std::printf("  rejects:         %d\n", rejects);
	} else {
		std::printf("  oneshot runs:    %d (accepts=%d, rejects=%d)\n",
			oneshotRuns, accepts, rejects);
	}
	std::printf("  watchdog resets: %d\n", calc.m_watchdogResets);

	if (calc.isValid()) {
		std::printf("\nFinal calibration:\n");
		PrintCalibration(calc.Transformation());
	} else {
		std::printf("\nNo valid calibration produced.\n");
	}

	return calc.isValid() ? 0 : 4;
}
