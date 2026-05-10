#include "stdafx.h"
#include "InputHealthSnapshotReader.h"

#include <cstring>
#include <exception>

bool InputHealthSnapshotReader::TryOpen()
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

void InputHealthSnapshotReader::Close()
{
	shmem_.Close();
	entries_by_handle_.clear();
	last_publish_tick_ = 0;
	last_publish_tick_change_ = {};
}

void InputHealthSnapshotReader::Refresh()
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
