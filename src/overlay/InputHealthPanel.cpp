#include "stdafx.h"
#include "InputHealthPanel.h"

#include "InputHealthCapture.h"
#include "InputHealthIpcClient.h"
#include "InputHealthSnapshotReader.h"
#include "inputhealth/SnapshotDiagnostics.h"

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <exception>
#include <string>
#include <vector>

#include <imgui/imgui.h>

namespace
{
	namespace diag = inputhealth::diagnostics;

	struct InputHealthUiState
	{
		InputHealthIpcClient ipc;
		InputHealthSnapshotReader reader;
		InputHealthCaptureWriter capture;
		protocol::InputHealthConfig config{};
		bool initialized = false;
		uint64_t observed_generation = 0;
		double next_heartbeat_time = 0.0;
		double next_snapshot_time = 0.0;
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
		state.capture.Capture(state.reader, state.config.master_enabled, now, false);
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

		if (diag::LooksLikeTriggerValue(b)) {
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
			const diag::PolarSummary p = diag::SummarizePolar(b);
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
		if (!state.capture.Path().empty()) {
			ImGui::TextDisabled("Capture: %s", state.capture.Path().c_str());
		}
		if (!state.capture.LastError().empty()) {
			ImGui::TextColored(ImVec4(0.95f, 0.40f, 0.40f, 1.0f),
				"Capture: %s", state.capture.LastError().c_str());
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
				const size_t pathLen = diag::BoundedPathLength(b.path);
				ImGui::TextUnformatted(b.path, b.path + pathLen);
				ImGui::TableSetColumnIndex(2);
				ImGui::TextUnformatted(diag::KindName(b));
				ImGui::TableSetColumnIndex(3);
				ImGui::TextUnformatted(diag::AxisRoleName(b.axis_role));
				ImGui::TableSetColumnIndex(4);
				ImGui::Text("%llu", (unsigned long long)b.welford_count);
				ImGui::TableSetColumnIndex(5);
				DrawRangeCell(b);
				ImGui::TableSetColumnIndex(6);
				if (b.welford_count > 0) ImGui::Text("%.4f", b.welford_mean);
				else                     ImGui::TextDisabled("-");
				ImGui::TableSetColumnIndex(7);
				if (b.welford_count > 1) ImGui::Text("%.4f", diag::SampleStdDev(b));
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
		state.capture.Capture(state.reader, true, ImGui::GetTime(), true);
	}
	if (ImGui::IsItemHovered()) {
		ImGui::SetTooltip("Writes the current component snapshot to the capture CSV immediately.");
	}

	ImGui::Separator();
	DrawComponentTable(state);
}
