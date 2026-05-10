#pragma once

#include "Protocol.h"

#include <chrono>
#include <string>
#include <unordered_map>

class InputHealthSnapshotReader
{
public:
	struct Entry
	{
		protocol::InputHealthSnapshotBody body;
		uint64_t last_seen_publish_tick = 0;
	};

	bool TryOpen();
	void Refresh();
	void Close();

	bool IsOpen() const { return shmem_; }
	const std::string &LastError() const { return last_error_; }
	uint64_t LastPublishTick() const { return last_publish_tick_; }

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
