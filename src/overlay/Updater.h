#pragma once

// Downloads the installer published by UpdateChecker, verifies its SHA-256
// against the digest GitHub gave us in the asset metadata, then launches the
// installer (with a UAC fallback if SmartScreen blocks the un-elevated run)
// and exits the overlay so the installer can replace the running EXE.
//
// All state is kept on the Updater itself; the UI thread reads it every frame
// without taking the worker's lock — only published bytes/total/state cross
// threads, and they're atomics. The error message is the only string field
// the UI thread reads, and it's only written once on terminal failure.

#include <atomic>
#include <mutex>
#include <string>
#include <thread>

namespace spacecal::updates {

enum class DownloadState : int {
	Idle,         // No download in flight.
	Downloading,  // WinHttp transfer in progress.
	Verifying,    // SHA-256 hash check running.
	Launching,    // ShellExecuteEx in flight (briefly).
	Done,         // Installer launched; caller should exit.
	Failed,       // errorMessage populated.
};

struct DownloadProgress {
	DownloadState state = DownloadState::Idle;
	uint64_t bytesReceived = 0;
	uint64_t bytesTotal = 0;       // 0 if unknown.
	std::string errorMessage;      // populated only on Failed.
	std::string downloadedPath;    // local path of the verified installer.
};

class Updater {
public:
	Updater() = default;
	~Updater();

	// Kick off the download + verify + launch chain. The arguments come from
	// UpdateChecker::GetResult(): the asset URL, expected SHA-256 hex (lower
	// or upper case), and the filename (used for the local copy and so the
	// installer keeps its expected name on disk).
	//
	// expectedSha256 may be empty — in that case, verification is skipped
	// (older releases that pre-date GitHub's `digest` field). The UI should
	// warn the user before calling DownloadAndLaunch in that case.
	void DownloadAndLaunch(
		const std::string& url,
		const std::string& expectedSha256,
		const std::string& filename);

	// Cheap snapshot for the UI thread.
	DownloadProgress GetProgress() const;

private:
	void Run(std::string url, std::string expectedSha256, std::string filename);

	mutable std::mutex m_mutex;
	std::atomic<DownloadState> m_state{ DownloadState::Idle };
	std::atomic<uint64_t> m_received{ 0 };
	std::atomic<uint64_t> m_total{ 0 };
	std::string m_errorMessage;
	std::string m_downloadedPath;
	std::thread m_worker;
};

} // namespace spacecal::updates
