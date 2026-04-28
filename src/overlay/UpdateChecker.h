#pragma once

// Background poller for the latest GitHub release. Spawns a worker thread on
// CheckAsync(), hits the GitHub API with WinHttp, parses the response, and
// publishes the result to a thread-safe state struct that the UI thread reads
// every frame to decide whether to show the "update available" banner.
//
// All fields on the result are valid until the next CheckAsync() call.

#include <atomic>
#include <mutex>
#include <string>
#include <thread>

namespace spacecal::updates {

struct UpdateInfo {
	bool available = false;          // True if remote tag > local build stamp.
	std::string latestTag;            // e.g. "v2026.4.28.1"
	std::string latestVersion;        // tag with leading "v" stripped
	std::string installerUrl;         // direct asset download URL
	std::string installerName;        // filename only
	std::string installerSha256;      // hex digest from the GitHub asset metadata
	uint64_t    installerSizeBytes = 0;
	std::string releaseNotesUrl;      // human-readable release page
	std::string errorMessage;         // non-empty on failure
};

enum class State : int {
	Idle,        // No check has run yet.
	Checking,    // Worker thread is in flight.
	HasResult,   // Result is populated (available true OR false).
	Failed,      // errorMessage is populated.
};

class UpdateChecker {
public:
	UpdateChecker() = default;
	~UpdateChecker();

	// Kick off a background check. Returns immediately. Subsequent calls while
	// already checking are no-ops.
	void CheckAsync();

	// Snapshot the current result + state. Cheap; takes the lock briefly.
	State GetState() const;
	UpdateInfo GetResult() const;

private:
	void RunCheck();

	mutable std::mutex m_mutex;
	std::atomic<State> m_state{ State::Idle };
	UpdateInfo m_result;
	std::thread m_worker;
};

} // namespace spacecal::updates
