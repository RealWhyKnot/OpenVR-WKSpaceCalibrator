#include "stdafx.h"
#include "InputHealthCapture.h"

#include "inputhealth/SnapshotDiagnostics.h"

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <exception>
#include <shlobj_core.h>
#include <stdexcept>
#include <vector>

namespace
{
	namespace diag = inputhealth::diagnostics;

	std::string WideToUtf8(const std::wstring &w)
	{
		if (w.empty()) return {};
		const int needed = WideCharToMultiByte(CP_UTF8, 0, w.data(), (int)w.size(), nullptr, 0, nullptr, nullptr);
		std::string out(needed, '\0');
		if (needed > 0) {
			WideCharToMultiByte(CP_UTF8, 0, w.data(), (int)w.size(), out.data(), needed, nullptr, nullptr);
		}
		return out;
	}

	std::wstring CurrentTimestampForFilename()
	{
		SYSTEMTIME t;
		GetLocalTime(&t);
		wchar_t buf[64];
		swprintf_s(buf, L"%04u-%02u-%02uT%02u-%02u-%02u",
			(unsigned)t.wYear, (unsigned)t.wMonth, (unsigned)t.wDay,
			(unsigned)t.wHour, (unsigned)t.wMinute, (unsigned)t.wSecond);
		return buf;
	}

	bool EnsureDirectory(const std::wstring &dir)
	{
		if (CreateDirectoryW(dir.c_str(), nullptr)) return true;
		return GetLastError() == ERROR_ALREADY_EXISTS;
	}

	std::wstring CaptureDir()
	{
		PWSTR rootPath = nullptr;
		if (FAILED(SHGetKnownFolderPath(FOLDERID_LocalAppDataLow, 0, nullptr, &rootPath)) || !rootPath) {
			throw std::runtime_error("Could not locate LocalAppDataLow for InputHealth captures.");
		}
		std::wstring root(rootPath);
		CoTaskMemFree(rootPath);

		const std::wstring appDir = root + L"\\SpaceCalibrator";
		const std::wstring captureDir = appDir + L"\\InputHealth";
		if (!EnsureDirectory(appDir) || !EnsureDirectory(captureDir)) {
			throw std::runtime_error("Could not create InputHealth capture directory.");
		}
		return captureDir;
	}

	void WriteHeader(std::ofstream &out)
	{
		out
			<< "time_s,publish_tick,handle,container_handle,device_serial_hash,path,kind,axis_role,"
			<< "is_scalar,is_boolean,welford_count,welford_mean,welford_m2,stddev,"
			<< "range_initialized,observed_min,observed_max,rest_min_initialized,rest_min,"
			<< "last_value,last_update_us,press_count,last_boolean,"
			<< "ph_initialized,ph_mean,ph_pos,ph_neg,ph_triggered,ph_triggered_positive,"
			<< "polar_global_max_r,polar_coverage,polar_entropy,polar_weak_bins,"
			<< "polar_counts,polar_max_r\n";
	}

	void WriteSemicolonU16Array(std::ofstream &out, const uint16_t *values, size_t count)
	{
		for (size_t i = 0; i < count; ++i) {
			if (i != 0) out << ';';
			out << values[i];
		}
	}

	void WriteSemicolonFloatArray(std::ofstream &out, const float *values, size_t count)
	{
		for (size_t i = 0; i < count; ++i) {
			if (i != 0) out << ';';
			out << values[i];
		}
	}
}

void InputHealthCaptureWriter::EnsureOpen()
{
	if (file_.is_open()) return;

	const std::wstring path = CaptureDir() +
		L"\\inputhealth_capture." + CurrentTimestampForFilename() + L".csv";
	file_.open(path, std::ios::out | std::ios::trunc);
	if (!file_.is_open()) {
		throw std::runtime_error("Could not open InputHealth capture file.");
	}
	path_ = WideToUtf8(path);
	last_error_.clear();
	WriteHeader(file_);
	file_.flush();
}

void InputHealthCaptureWriter::Capture(
	const InputHealthSnapshotReader &reader,
	bool monitoringEnabled,
	double now,
	bool force)
{
	if (!monitoringEnabled || !reader.IsOpen()) return;
	if (!force && now < next_capture_time_) return;

	try {
		EnsureOpen();
		next_capture_time_ = now + 5.0;

		std::vector<const InputHealthSnapshotReader::Entry *> sorted;
		const auto &entries = reader.EntriesByHandle();
		sorted.reserve(entries.size());
		for (const auto &kv : entries) sorted.push_back(&kv.second);
		std::sort(sorted.begin(), sorted.end(),
			[](const InputHealthSnapshotReader::Entry *a, const InputHealthSnapshotReader::Entry *b) {
				if (a->body.device_serial_hash != b->body.device_serial_hash) {
					return a->body.device_serial_hash < b->body.device_serial_hash;
				}
				return std::strncmp(a->body.path, b->body.path,
					protocol::INPUTHEALTH_PATH_LEN) < 0;
			});

		for (const auto *entry : sorted) {
			const auto &b = entry->body;
			const diag::PolarSummary polar = diag::SummarizePolar(b);
			const size_t pathLen = diag::BoundedPathLength(b.path);

			file_
				<< now << ','
				<< reader.LastPublishTick() << ','
				<< b.handle << ','
				<< b.container_handle << ','
				<< b.device_serial_hash << ','
				<< std::string(b.path, b.path + pathLen) << ','
				<< diag::KindName(b) << ','
				<< diag::AxisRoleName(b.axis_role) << ','
				<< (unsigned)b.is_scalar << ','
				<< (unsigned)b.is_boolean << ','
				<< b.welford_count << ','
				<< b.welford_mean << ','
				<< b.welford_m2 << ','
				<< diag::SampleStdDev(b) << ','
				<< (unsigned)b.scalar_range_initialized << ','
				<< b.observed_min << ','
				<< b.observed_max << ','
				<< (unsigned)b.rest_min_initialized << ','
				<< b.rest_min << ','
				<< b.last_value << ','
				<< b.last_update_us << ','
				<< b.press_count << ','
				<< (unsigned)b.last_boolean << ','
				<< (unsigned)b.ph_initialized << ','
				<< b.ph_mean << ','
				<< b.ph_pos << ','
				<< b.ph_neg << ','
				<< (unsigned)b.ph_triggered << ','
				<< (unsigned)b.ph_triggered_positive << ','
				<< b.polar_global_max_r << ','
				<< polar.coverage << ','
				<< polar.entropy << ','
				<< polar.weak_bins << ',';
			WriteSemicolonU16Array(file_, b.polar_count, protocol::INPUTHEALTH_POLAR_BIN_COUNT);
			file_ << ',';
			WriteSemicolonFloatArray(file_, b.polar_max_r, protocol::INPUTHEALTH_POLAR_BIN_COUNT);
			file_ << '\n';
		}
		file_.flush();
	} catch (const std::exception &e) {
		last_error_ = e.what();
		if (file_.is_open()) file_.close();
	}
}
