#pragma once

#include "Protocol.h"

#include <atomic>
#include <thread>
#include <set>
#include <mutex>

#define WIN32_LEAN_AND_MEAN
#include <windows.h>

class ServerTrackedDeviceProvider;

class IPCServer
{
public:
	IPCServer(ServerTrackedDeviceProvider *driver) : driver(driver) { }
	~IPCServer();

	void Run();
	void Stop();

private:
	void HandleRequest(const protocol::Request &request, protocol::Response &response);

	struct PipeInstance
	{
		OVERLAPPED overlap; // Used by the API
		HANDLE pipe;
		IPCServer *server;

		protocol::Request request;
		protocol::Response response;
	};

	PipeInstance *CreatePipeInstance(HANDLE pipe);
	void ClosePipeInstance(PipeInstance *pipeInst);

	static void RunThread(IPCServer *_this);
	static BOOL CreateAndConnectInstance(LPOVERLAPPED overlap, HANDLE &pipe);
	static void WINAPI CompletedReadCallback(DWORD err, DWORD bytesRead, LPOVERLAPPED overlap);
	static void WINAPI CompletedWriteCallback(DWORD err, DWORD bytesWritten, LPOVERLAPPED overlap);

	std::thread mainThread;

	// Atomic because `stop` is written by the driver-shutdown thread and
	// read by RunThread on every loop iter; `running` is written by both
	// threads on lifecycle transitions and inspected by Stop()'s early-
	// return guard. Bare bool worked in practice on x86 (single-byte R/W
	// is naturally atomic there) but is technically a data race per the
	// C++ memory model. Relaxed ordering is sufficient: there's no other
	// state these flags need to fence against.
	std::atomic<bool> running { false };
	std::atomic<bool> stop { false };

	std::set<PipeInstance *> pipes;
	// Created by RunThread on entry, signalled by Stop() to break the wait,
	// and closed in Stop() once the worker has joined so the kernel handle
	// doesn't leak across driver reload.
	HANDLE connectEvent = nullptr;

	ServerTrackedDeviceProvider *driver;
};
