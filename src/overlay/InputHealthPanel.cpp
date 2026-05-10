#include "stdafx.h"
#include "InputHealthPanel.h"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <exception>
#include <fstream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <unordered_map>
#include <vector>

#include <imgui/imgui.h>
#include <shlobj_core.h>

#include "Protocol.h"

namespace
{
	class BrokenPipeException : public std::runtime_error
	{
	public:
		BrokenPipeException(const std::string &msg, DWORD code)
			: std::runtime_error(msg), errorCode(code) {}

		DWORD errorCode;
	};

	bool IsBrokenPipeError(DWORD code)
	{
		return code == ERROR_BROKEN_PIPE ||
			code == ERROR_PIPE_NOT_CONNECTED ||
			code == ERROR_NO_DATA;
	}

	std::string LastErrorString(DWORD lastError)
	{
		LPWSTR buffer = nullptr;
		size_t size = FormatMessageW(
			FORMAT_MESSAGE_ALLOCATE_BUFFER | FORMAT_MESSAGE_FROM_SYSTEM | FORMAT_MESSAGE_IGNORE_INSERTS,
			nullptr, lastError, MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT), (LPWSTR)&buffer, 0, nullptr);
		if (!buffer) return {};

		int needed = WideCharToMultiByte(CP_UTF8, 0, buffer, (int)size, nullptr, 0, nullptr, nullptr);
		std::string out(needed, '\0');
		if (needed > 0) {
			WideCharToMultiByte(CP_UTF8, 0, buffer, (int)size, out.data(), needed, nullptr, nullptr);
		}
		LocalFree(buffer);
		return out;
	}

	class InputHealthIpcClient
	{
	public:
		~InputHealthIpcClient()
		{
			Close();
		}

		void Close()
		{
			if (pipe_ && pipe_ != INVALID_HANDLE_VALUE) {
				CloseHandle(pipe_);
				pipe_ = INVALID_HANDLE_VALUE;
			}
		}

		void Connect()
		{
			Close();

			LPCSTR pipeName = OPENVR_PAIRDRIVER_INPUTHEALTH_PIPE_NAME;
			WaitNamedPipeA(pipeName, 100);
			pipe_ = CreateFileA(pipeName, GENERIC_READ | GENERIC_WRITE, 0, 0, OPEN_EXISTING, 0, 0);
			if (pipe_ == INVALID_HANDLE_VALUE) {
				throw std::runtime_error(
					"InputHealth driver unavailable. Check SteamVR, the shared driver, and enable_inputhealth.flag.");
			}

			DWORD mode = PIPE_READMODE_MESSAGE;
			if (!SetNamedPipeHandleState(pipe_, &mode, 0, 0)) {
				const DWORD err = GetLastError();
				Close();
				throw std::runtime_error("Could not set InputHealth pipe mode. Error " +
					std::to_string(err) + ": " + LastErrorString(err));
			}

			Send(protocol::Request(protocol::RequestHandshake));
			const protocol::Response response = Receive();
			if (response.type != protocol::ResponseHandshake || response.protocol.version != protocol::Version) {
				const uint32_t driverVersion = response.protocol.version;
				Close();
				throw std::runtime_error(
					"InputHealth protocol mismatch. Reinstall SpaceCalibrator/shared driver. (Overlay: " +
					std::to_string(protocol::Version) + ", driver: " + std::to_string(driverVersion) + ")");
			}
			++connection_generation_;
		}

		bool IsConnected() const
		{
			return pipe_ != INVALID_HANDLE_VALUE;
		}

		uint64_t ConnectionGeneration() const
		{
			return connection_generation_;
		}

		protocol::Response SendBlocking(const protocol::Request &request)
		{
			try {
				Send(request);
				return Receive();
			} catch (const BrokenPipeException &e) {
				if (in_reconnect_) throw;

				Close();

				in_reconnect_ = true;
				try {
					Connect();
				} catch (...) {
					in_reconnect_ = false;
					throw;
				}
				in_reconnect_ = false;

				Send(request);
				return Receive();
			}
		}

	private:
		void Send(const protocol::Request &request)
		{
			DWORD bytesWritten = 0;
			const BOOL ok = WriteFile(pipe_, &request, sizeof(request), &bytesWritten, 0);
			if (!ok) {
				const DWORD err = GetLastError();
				const std::string msg = "InputHealth IPC write error " + std::to_string(err) +
					": " + LastErrorString(err);
				if (IsBrokenPipeError(err)) throw BrokenPipeException(msg, err);
				throw std::runtime_error(msg);
			}
			if (bytesWritten != sizeof(request)) {
				throw std::runtime_error("InputHealth IPC short write");
			}
		}

		protocol::Response Receive()
		{
			protocol::Response response(protocol::ResponseInvalid);
			DWORD bytesRead = 0;
			const BOOL ok = ReadFile(pipe_, &response, sizeof(response), &bytesRead, 0);
			if (!ok) {
				const DWORD err = GetLastError();
				if (err != ERROR_MORE_DATA) {
					const std::string msg = "InputHealth IPC read error " + std::to_string(err) +
						": " + LastErrorString(err);
					if (IsBrokenPipeError(err)) throw BrokenPipeException(msg, err);
					throw std::runtime_error(msg);
				}

				char drain[1024];
				while (true) {
					DWORD drained = 0;
					const BOOL drainOk = ReadFile(pipe_, drain, sizeof(drain), &drained, 0);
					if (drainOk) break;
					const DWORD drainErr = GetLastError();
					if (drainErr == ERROR_MORE_DATA) continue;
					if (IsBrokenPipeError(drainErr)) {
						throw BrokenPipeException("Pipe broken while draining oversized response", drainErr);
					}
					break;
				}
				throw std::runtime_error("Invalid InputHealth IPC response: message too large");
			}

			if (bytesRead != sizeof(response)) {
				throw std::runtime_error("Invalid InputHealth IPC response size: " + std::to_string(bytesRead));
			}

			return response;
		}

		HANDLE pipe_ = INVALID_HANDLE_VALUE;
		bool in_reconnect_ = false;
		uint64_t connection_generation_ = 0;
	};

	class InputHealthSnapshotReader
	{
	public:
		struct Entry
		{
			protocol::InputHealthSnapshotBody body;
			uint64_t last_seen_publish_tick = 0;
		};

		bool TryOpen()
		{
			try {
				shmem_.Open(OPENVR_PAIRDRIVER_INPUTHEALTH_SHMEM_NAME);
				last_error_.clear();
				entries_by_handle_.clear();
				last_publish_tick_ = shmem_.LoadPublishTick();
				last_publish_tick_change_ = std::chrono::steady_clock::now();
				return true;
			} catch (const std::exception &e) {
				last_error_ = e.what();
				return false;
			}
		}

		void Close()
		{
			shmem_.Close();
			entries_by_handle_.clear();
			last_publish_tick_ = 0;
			last_publish_tick_change_ = {};
		}

		void Refresh()
		{
			if (!shmem_) {
				TryOpen();
				if (!shmem_) return;
			}

			const uint64_t publishTick = shmem_.LoadPublishTick();
			const auto now = std::chrono::steady_clock::now();
			if (publishTick != last_publish_tick_) {
				last_publish_tick_ = publishTick;
				last_publish_tick_change_ = now;
			} else if (last_publish_tick_change_ != std::chrono::steady_clock::time_point{} &&
				now - last_publish_tick_change_ > std::chrono::seconds(2)) {
				last_error_ = "InputHealth shared memory is stale; waiting for driver restart";
				Close();
				return;
			}

			for (uint32_t i = 0; i < protocol::INPUTHEALTH_SLOT_COUNT; ++i) {
				protocol::InputHealthSnapshotBody body;
				if (!shmem_.TryReadSlot(i, body)) continue;
				if (body.handle == 0) continue;
				Entry &entry = entries_by_handle_[body.handle];
				std::memcpy(&entry.body, &body, sizeof(body));
				entry.last_seen_publish_tick = last_publish_tick_;
			}
		}

		bool IsOpen() const
		{
			return shmem_;
		}

		const std::string &LastError() const
		{
			return last_error_;
		}

		uint64_t LastPublishTick() const
		{
			return last_publish_tick_;
		}

		const std::unordered_map<uint64_t, Entry> &EntriesByHandle() const
		{
			return entries_by_handle_;
		}

	private:
		protocol::InputHealthSnapshotShmem shmem_;
		std::unordered_map<uint64_t, Entry> entries_by_handle_;
		std::string last_error_;
		uint64_t last_publish_tick_ = 0;
		std::chrono::steady_clock::time_point last_publish_tick_change_{};
	};

	const char *AxisRoleName(uint8_t role)
	{
		switch (role) {
			case 0: return "-";
			case 1: return "stick.x";
			case 2: return "stick.y";
			default: return "?";
		}
	}

	const char *KindName(const protocol::InputHealthSnapshotBody &b)
	{
		if (b.is_scalar)  return "scalar";
		if (b.is_boolean) return "bool";
		return "?";
	}

	double SampleStdDev(const protocol::InputHealthSnapshotBody &b)
	{
		if (b.welford_count < 2) return 0.0;
		return std::sqrt(b.welford_m2 / static_cast<double>(b.welford_count - 1));
	}

	size_t BoundedPathLength(const char *path)
	{
		size_t n = 0;
		while (n < protocol::INPUTHEALTH_PATH_LEN && path[n] != '\0') ++n;
		return n;
	}

	std::string LowerPath(const protocol::InputHealthSnapshotBody &b)
	{
		const size_t n = BoundedPathLength(b.path);
		std::string out(b.path, b.path + n);
		for (char &c : out) {
			c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
		}
		return out;
	}

	bool LooksLikeTriggerValue(const protocol::InputHealthSnapshotBody &b)
	{
		const std::string p = LowerPath(b);
		return p.find("trigger") != std::string::npos &&
			(p.find("value") != std::string::npos || p.rfind("/input/trigger", 0) == 0);
	}

	struct PolarSummary
	{
		int    bins_with_samples = 0;
		int    weak_bins = 0;
		double coverage = 0.0;
		double entropy = 0.0;
		float  min_r = 0.0f;
		float  mean_r = 0.0f;
		bool   enough_coverage = false;
	};

	PolarSummary SummarizePolar(const protocol::InputHealthSnapshotBody &b)
	{
		PolarSummary s;
		constexpr uint16_t kMinSamplesPerBin = 3;
		const int binCount = static_cast<int>(protocol::INPUTHEALTH_POLAR_BIN_COUNT);

		uint64_t totalCount = 0;
		for (int i = 0; i < binCount; ++i) totalCount += b.polar_count[i];

		float minR = std::numeric_limits<float>::max();
		float sumR = 0.0f;
		for (int i = 0; i < binCount; ++i) {
			if (b.polar_count[i] >= kMinSamplesPerBin && b.polar_max_r[i] > 0.0f) {
				++s.bins_with_samples;
				minR = std::min(minR, b.polar_max_r[i]);
				sumR += b.polar_max_r[i];
			}
			if (totalCount > 0 && b.polar_count[i] > 0) {
				const double p = static_cast<double>(b.polar_count[i]) / static_cast<double>(totalCount);
				s.entropy -= p * std::log(p);
			}
		}

		if (totalCount > 0) {
			s.entropy /= std::log(static_cast<double>(binCount));
		}
		s.coverage = static_cast<double>(s.bins_with_samples) / static_cast<double>(binCount);
		if (s.bins_with_samples > 0) {
			s.min_r = minR;
			s.mean_r = sumR / static_cast<float>(s.bins_with_samples);
		}

		s.enough_coverage = s.coverage >= 0.70 && s.entropy >= 0.75 && b.polar_global_max_r >= 0.60f;
		if (s.enough_coverage) {
			const float weakThreshold = b.polar_global_max_r * 0.75f;
			for (int i = 0; i < binCount; ++i) {
				if (b.polar_count[i] >= kMinSamplesPerBin &&
					b.polar_max_r[i] > 0.0f &&
					b.polar_max_r[i] < weakThreshold) {
					++s.weak_bins;
				}
			}
		}
		return s;
	}

	void DrawRangeCell(const protocol::InputHealthSnapshotBody &b)
	{
		if (!b.is_scalar || !b.scalar_range_initialized) {
			ImGui::TextDisabled("-");
			return;
		}
		ImGui::Text("%.3f..%.3f", b.observed_min, b.observed_max);
	}

	void DrawHintCell(const protocol::InputHealthSnapshotBody &b)
	{
		if (b.is_boolean) {
			if (b.press_count == 0) ImGui::TextDisabled("no presses");
			else                   ImGui::Text("seen");
			return;
		}

		if (!b.is_scalar || !b.scalar_range_initialized) {
			ImGui::TextDisabled("waiting");
			return;
		}

		if (b.ph_triggered) {
			ImGui::TextColored(ImVec4(0.95f, 0.70f, 0.40f, 1.0f),
				"drift %s", b.ph_triggered_positive ? "+" : "-");
			return;
		}

		if (LooksLikeTriggerValue(b)) {
			if (b.welford_count >= 20 && b.observed_min > 0.08f) {
				ImGui::TextColored(ImVec4(0.95f, 0.70f, 0.40f, 1.0f), "rest high?");
			} else if (b.welford_count >= 20 && b.observed_max < 0.85f) {
				ImGui::TextColored(ImVec4(0.95f, 0.70f, 0.40f, 1.0f), "max low?");
			} else {
				ImGui::TextDisabled("range");
			}
			return;
		}

		if (b.axis_role == 1) {
			const PolarSummary p = SummarizePolar(b);
			if (!p.enough_coverage) {
				ImGui::TextDisabled("sweep %.0f%% H%.2f", p.coverage * 100.0, p.entropy);
			} else if (p.weak_bins > 0) {
				ImGui::TextColored(ImVec4(0.95f, 0.70f, 0.40f, 1.0f),
					"weak arc? %d", p.weak_bins);
			} else {
				ImGui::TextDisabled("coverage ok");
			}
			return;
		}

		ImGui::TextDisabled("range");
	}

	struct InputHealthUiState
	{
		InputHealthIpcClient ipc;
		InputHealthSnapshotReader reader;
		protocol::InputHealthConfig config{};
		bool initialized = false;
		uint64_t observed_generation = 0;
		double next_heartbeat_time = 0.0;
		double next_snapshot_time = 0.0;
		double next_capture_time = 0.0;
		std::ofstream capture_file;
		std::string capture_path;
		std::string capture_error;
		std::string last_error;
	};

	InputHealthUiState &State()
	{
		static InputHealthUiState state;
		if (!state.initialized) {
			state.config.master_enabled = true;
			state.config.diagnostics_only = true;
			state.config.enable_rest_recenter = false;
			state.config.enable_trigger_remap = false;
			state.config.notification_cooldown_s = 0;
			state.config._reserved = 0;
			state.initialized = true;
		}
		return state;
	}

	void MaintainDriverConnection(InputHealthUiState &state);

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

	std::wstring InputHealthCaptureDir()
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

	void WriteCaptureHeader(std::ofstream &out)
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

	void EnsureCaptureOpen(InputHealthUiState &state)
	{
		if (state.capture_file.is_open()) return;

		const std::wstring path = InputHealthCaptureDir() +
			L"\\inputhealth_capture." + CurrentTimestampForFilename() + L".csv";
		state.capture_file.open(path, std::ios::out | std::ios::trunc);
		if (!state.capture_file.is_open()) {
			throw std::runtime_error("Could not open InputHealth capture file.");
		}
		state.capture_path = WideToUtf8(path);
		state.capture_error.clear();
		WriteCaptureHeader(state.capture_file);
		state.capture_file.flush();
	}

	void CaptureSnapshot(InputHealthUiState &state, bool force)
	{
		if (!state.config.master_enabled || !state.reader.IsOpen()) return;
		if (!force && ImGui::GetTime() < state.next_capture_time) return;

		try {
			EnsureCaptureOpen(state);
			const double now = ImGui::GetTime();
			state.next_capture_time = now + 5.0;

			std::vector<const InputHealthSnapshotReader::Entry *> sorted;
			const auto &entries = state.reader.EntriesByHandle();
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
				const PolarSummary polar = SummarizePolar(b);
				const size_t pathLen = BoundedPathLength(b.path);

				state.capture_file
					<< now << ','
					<< state.reader.LastPublishTick() << ','
					<< b.handle << ','
					<< b.container_handle << ','
					<< b.device_serial_hash << ','
					<< std::string(b.path, b.path + pathLen) << ','
					<< KindName(b) << ','
					<< AxisRoleName(b.axis_role) << ','
					<< (unsigned)b.is_scalar << ','
					<< (unsigned)b.is_boolean << ','
					<< b.welford_count << ','
					<< b.welford_mean << ','
					<< b.welford_m2 << ','
					<< SampleStdDev(b) << ','
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
				WriteSemicolonU16Array(state.capture_file, b.polar_count, protocol::INPUTHEALTH_POLAR_BIN_COUNT);
				state.capture_file << ',';
				WriteSemicolonFloatArray(state.capture_file, b.polar_max_r, protocol::INPUTHEALTH_POLAR_BIN_COUNT);
				state.capture_file << '\n';
			}
			state.capture_file.flush();
		} catch (const std::exception &e) {
			state.capture_error = e.what();
			if (state.capture_file.is_open()) state.capture_file.close();
		}
	}

	void Tick(InputHealthUiState &state)
	{
		const double now = ImGui::GetTime();

		if (now >= state.next_heartbeat_time) {
			MaintainDriverConnection(state);
			state.next_heartbeat_time = now + 1.0;
		}
		if (now >= state.next_snapshot_time) {
			state.reader.Refresh();
			state.next_snapshot_time = now + 0.10;
		}
		CaptureSnapshot(state, false);
	}

	void PushConfigToDriver(InputHealthUiState &state)
	{
		state.config.diagnostics_only = true;
		state.config.enable_rest_recenter = false;
		state.config.enable_trigger_remap = false;

		if (!state.ipc.IsConnected()) {
			state.last_error = "InputHealth driver is not connected.";
			return;
		}

		try {
			protocol::Request req(protocol::RequestSetInputHealthConfig);
			req.setInputHealthConfig = state.config;
			const protocol::Response response = state.ipc.SendBlocking(req);
			if (response.type != protocol::ResponseSuccess) {
				state.last_error = "InputHealth driver rejected config.";
				return;
			}
			state.last_error.clear();
		} catch (const std::exception &e) {
			state.last_error = std::string("InputHealth IPC: ") + e.what();
			state.ipc.Close();
		}
	}

	void MaintainDriverConnection(InputHealthUiState &state)
	{
		try {
			if (!state.ipc.IsConnected()) {
				state.ipc.Connect();
			}

			const protocol::Response response =
				state.ipc.SendBlocking(protocol::Request(protocol::RequestHandshake));
			if (response.type != protocol::ResponseHandshake || response.protocol.version != protocol::Version) {
				state.last_error = "InputHealth protocol mismatch during heartbeat.";
				state.ipc.Close();
				return;
			}

			const uint64_t generation = state.ipc.ConnectionGeneration();
			if (generation != state.observed_generation) {
				state.observed_generation = generation;
				PushConfigToDriver(state);
			}
			if (state.last_error.find("InputHealth driver") == 0 ||
				state.last_error.find("InputHealth IPC") == 0) {
				state.last_error.clear();
			}
		} catch (const std::exception &e) {
			state.last_error = std::string("InputHealth driver: ") + e.what();
			state.ipc.Close();
		}
	}

	void SendReset(InputHealthUiState &state, uint64_t serialHash)
	{
		if (!state.ipc.IsConnected()) {
			state.last_error = "InputHealth driver is not connected.";
			return;
		}

		try {
			protocol::Request req(protocol::RequestResetInputHealthStats);
			req.resetInputHealthStats.device_serial_hash = serialHash;
			req.resetInputHealthStats.reset_passive = 1;
			req.resetInputHealthStats.reset_active = 0;
			req.resetInputHealthStats.reset_curves = 0;
			const protocol::Response response = state.ipc.SendBlocking(req);
			if (response.type != protocol::ResponseSuccess) {
				state.last_error = "InputHealth driver rejected reset.";
				return;
			}
			state.last_error.clear();
		} catch (const std::exception &e) {
			state.last_error = std::string("InputHealth IPC: ") + e.what();
			state.ipc.Close();
		}
	}

	void DrawStatusBanner(InputHealthUiState &state)
	{
		const bool ipcOk = state.ipc.IsConnected();
		const bool shmemOk = state.reader.IsOpen();

		ImGui::TextColored(ipcOk ? ImVec4(0.40f, 0.85f, 0.40f, 1.0f) : ImVec4(0.85f, 0.40f, 0.40f, 1.0f),
			"IPC %s", ipcOk ? "connected" : "disconnected");
		ImGui::SameLine();
		ImGui::TextDisabled("|");
		ImGui::SameLine();
		ImGui::TextColored(shmemOk ? ImVec4(0.40f, 0.85f, 0.40f, 1.0f) : ImVec4(0.85f, 0.40f, 0.40f, 1.0f),
			"Shmem %s", shmemOk ? "open" : "closed");
		ImGui::SameLine();
		ImGui::TextDisabled("|");
		ImGui::SameLine();
		ImGui::Text("publish_tick=%llu", (unsigned long long)state.reader.LastPublishTick());

		if (!state.last_error.empty()) {
			ImGui::TextColored(ImVec4(0.95f, 0.40f, 0.40f, 1.0f), "%s", state.last_error.c_str());
		}
		if (!shmemOk && !state.reader.LastError().empty()) {
			ImGui::TextDisabled("Shmem: %s", state.reader.LastError().c_str());
		}
		if (!state.capture_path.empty()) {
			ImGui::TextDisabled("Capture: %s", state.capture_path.c_str());
		}
		if (!state.capture_error.empty()) {
			ImGui::TextColored(ImVec4(0.95f, 0.40f, 0.40f, 1.0f),
				"Capture: %s", state.capture_error.c_str());
		}
	}

	void DrawComponentTable(InputHealthUiState &state)
	{
		const auto &entries = state.reader.EntriesByHandle();
		if (entries.empty()) {
			ImGui::TextDisabled("No components seen yet. Enable monitoring and move a controller.");
			return;
		}

		std::vector<const InputHealthSnapshotReader::Entry *> sorted;
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

		const ImGuiTableFlags flags =
			ImGuiTableFlags_SizingStretchProp |
			ImGuiTableFlags_RowBg |
			ImGuiTableFlags_BordersInnerH |
			ImGuiTableFlags_Resizable;

		if (ImGui::BeginTable("inputhealth_components", 10, flags)) {
			ImGui::TableSetupColumn("device");
			ImGui::TableSetupColumn("path");
			ImGui::TableSetupColumn("kind");
			ImGui::TableSetupColumn("role");
			ImGui::TableSetupColumn("n");
			ImGui::TableSetupColumn("range");
			ImGui::TableSetupColumn("mean");
			ImGui::TableSetupColumn("stddev");
			ImGui::TableSetupColumn("rest / press");
			ImGui::TableSetupColumn("hint");
			ImGui::TableHeadersRow();

			for (const auto *e : sorted) {
				const auto &b = e->body;
				ImGui::TableNextRow();

				char serialShort[20];
				snprintf(serialShort, sizeof(serialShort), "%016llx",
					(unsigned long long)b.device_serial_hash);

				ImGui::TableSetColumnIndex(0);
				ImGui::TextUnformatted(serialShort);
				ImGui::TableSetColumnIndex(1);
				const size_t pathLen = BoundedPathLength(b.path);
				ImGui::TextUnformatted(b.path, b.path + pathLen);
				ImGui::TableSetColumnIndex(2);
				ImGui::TextUnformatted(KindName(b));
				ImGui::TableSetColumnIndex(3);
				ImGui::TextUnformatted(AxisRoleName(b.axis_role));
				ImGui::TableSetColumnIndex(4);
				ImGui::Text("%llu", (unsigned long long)b.welford_count);
				ImGui::TableSetColumnIndex(5);
				DrawRangeCell(b);
				ImGui::TableSetColumnIndex(6);
				if (b.welford_count > 0) ImGui::Text("%.4f", b.welford_mean);
				else                     ImGui::TextDisabled("-");
				ImGui::TableSetColumnIndex(7);
				if (b.welford_count > 1) ImGui::Text("%.4f", SampleStdDev(b));
				else                     ImGui::TextDisabled("-");
				ImGui::TableSetColumnIndex(8);
				if (b.is_boolean) {
					ImGui::Text("%llu / %s",
						(unsigned long long)b.press_count,
						b.last_boolean ? "down" : "up");
				} else if (b.rest_min_initialized) {
					ImGui::Text("%.4f", b.rest_min);
				} else {
					ImGui::TextDisabled("-");
				}
				ImGui::TableSetColumnIndex(9);
				DrawHintCell(b);
			}

			ImGui::EndTable();
		}
	}
}

void InputHealth_Tick()
{
	InputHealthUiState &state = State();
	Tick(state);
}

void CCal_DrawInputHealth()
{
	InputHealthUiState &state = State();
	Tick(state);

	ImGui::Text("InputHealth");
	ImGui::TextDisabled("Live OpenVR button and axis diagnostics. Monitoring is diagnostics-only and on by default.");
	ImGui::Spacing();

	DrawStatusBanner(state);
	ImGui::Separator();

	bool master = state.config.master_enabled;
	if (ImGui::Checkbox("Enable input monitoring", &master)) {
		state.config.master_enabled = master;
		PushConfigToDriver(state);
	}
	if (ImGui::IsItemHovered()) {
		ImGui::SetTooltip("When enabled, the driver records component observations and publishes diagnostics snapshots.");
	}

	ImGui::SameLine();
	bool diagnosticsOnly = true;
	ImGui::BeginDisabled(true);
	ImGui::Checkbox("Diagnostics-only", &diagnosticsOnly);
	ImGui::EndDisabled();
	if (ImGui::IsItemHovered()) {
		ImGui::SetTooltip("Forced on for now. Correction controls will come after the live stats are validated.");
	}

	ImGui::SameLine();
	if (ImGui::Button("Reset stats")) {
		SendReset(state, 0);
	}
	if (ImGui::IsItemHovered()) {
		ImGui::SetTooltip("Clears accumulated passive stats for all observed input components.");
	}

	ImGui::SameLine();
	if (ImGui::Button("Capture now")) {
		CaptureSnapshot(state, true);
	}
	if (ImGui::IsItemHovered()) {
		ImGui::SetTooltip("Writes the current component snapshot to the capture CSV immediately.");
	}

	ImGui::Separator();
	DrawComponentTable(state);
}
