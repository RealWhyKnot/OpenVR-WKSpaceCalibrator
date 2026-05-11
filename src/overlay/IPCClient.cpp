#include "stdafx.h"
#include "IPCClient.h"
#include "CalibrationMetrics.h"   // WriteLogAnnotation -- see Connect() for why
                                  // we trace the IPC handshake outcomes.

#include <cstdio>
#include <string>

// Forward-declared rather than #include "Calibration.h" because Calibration.h
// pulls in <openvr.h> while SCIPCClient.cpp's translation unit (via Protocol.h)
// already includes <openvr_driver.h>; the two openvr headers redefine common
// constants and conflict at compile time. The single function we need is
// signature-stable, so a local forward declaration is the safe minimum coupling.
void ReopenShmem();

namespace
{
	class BrokenPipeException : public std::runtime_error
	{
	public:
		BrokenPipeException(const std::string& msg, DWORD code)
			: std::runtime_error(msg), errorCode(code) {}

		DWORD errorCode;
	};

	bool IsBrokenPipeError(DWORD code)
	{
		return code == ERROR_BROKEN_PIPE          // 0x6D
			|| code == ERROR_PIPE_NOT_CONNECTED   // 0xE9
			|| code == ERROR_NO_DATA;             // 0xE8
	}
}

std::string WStringToString(const std::wstring& wstr)
{
	int size_needed = WideCharToMultiByte(CP_UTF8, 0, &wstr[0], (int)wstr.size(), nullptr, 0, nullptr, nullptr);
	std::string str_to(size_needed, 0);
	WideCharToMultiByte(CP_UTF8, 0, &wstr[0], (int)wstr.size(), &str_to[0], size_needed, nullptr, nullptr);
	return str_to;
}

static std::string LastErrorString(DWORD lastError)
{
	LPWSTR buffer = nullptr;
	size_t size = FormatMessageW(
		FORMAT_MESSAGE_ALLOCATE_BUFFER | FORMAT_MESSAGE_FROM_SYSTEM | FORMAT_MESSAGE_IGNORE_INSERTS,
		nullptr, lastError, MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT), (LPWSTR)&buffer, 0, nullptr
	);
	std::wstring message(buffer, size);
	LocalFree(buffer);
	return WStringToString(message);
}

SCIPCClient::~SCIPCClient()
{
	if (pipe && pipe != INVALID_HANDLE_VALUE)
		CloseHandle(pipe);
}

void SCIPCClient::Connect()
{
	LPCTSTR pipeName = TEXT(OPENVR_PAIRDRIVER_CALIBRATION_PIPE_NAME);

	WaitNamedPipe(pipeName, 1000);
	pipe = CreateFile(pipeName, GENERIC_READ | GENERIC_WRITE, 0, 0, OPEN_EXISTING, 0, 0);

	// Annotate every pipe-open outcome -- gives us evidence of whether the
	// IPC connection itself behaves differently across launch contexts. A
	// script-launched instance with elevated parent might hit ACCESS_DENIED
	// (5) where a Start-menu launch succeeds; a stale-pipe-from-prev-instance
	// case might hit PIPE_BUSY (231) etc.
	{
		DWORD lastError = (pipe == INVALID_HANDLE_VALUE) ? GetLastError() : 0;
		char annot[256];
		snprintf(annot, sizeof annot, "ipc_pipe_open: handle=%p err=%lu",
			pipe, lastError);
		Metrics::WriteLogAnnotation(annot);
	}

	if (pipe == INVALID_HANDLE_VALUE)
	{
		throw std::runtime_error("Space Calibrator driver unavailable. Make sure SteamVR is running, and the Space Calibrator addon is enabled in SteamVR settings.");
	}

	DWORD mode = PIPE_READMODE_MESSAGE;
	if (!SetNamedPipeHandleState(pipe, &mode, 0, 0))
	{
		DWORD lastError = GetLastError();
		throw std::runtime_error("Couldn't set pipe mode. Error " + std::to_string(lastError) + ": " + LastErrorString(lastError));
	}

	auto response = SendBlocking(protocol::Request(protocol::RequestHandshake));
	{
		char annot[160];
		snprintf(annot, sizeof annot, "ipc_handshake: response_type=%d server_version=%u client_version=%u",
			(int)response.type,
			(unsigned)response.protocol.version,
			(unsigned)protocol::Version);
		Metrics::WriteLogAnnotation(annot);
	}
	if (response.type != protocol::ResponseHandshake || response.protocol.version != protocol::Version)
	{
		throw std::runtime_error(
			"Incorrect driver version installed, try reinstalling Space Calibrator. (Client: " +
			std::to_string(protocol::Version) +
			", Driver: " +
			std::to_string(response.protocol.version) +
			")"
		);
	}
}

protocol::Response SCIPCClient::SendBlocking(const protocol::Request &request)
{
	try
	{
		Send(request);
		return Receive();
	}
	catch (const BrokenPipeException &e)
	{
		// If we're already inside a reconnect attempt (e.g. handshake-after-reconnect
		// hit another broken pipe), don't recurse -- let the caller handle it.
		if (inReconnect)
			throw;

		// SteamVR's vrserver may have restarted, leaving us with a stale pipe.
		// Try to reconnect once and re-issue the request transparently.
		fprintf(stderr,
			"[SCIPCClient] Broken pipe (error %lu) during request; attempting reconnect...\n",
			(unsigned long)e.errorCode);

		if (pipe && pipe != INVALID_HANDLE_VALUE)
		{
			CloseHandle(pipe);
			pipe = INVALID_HANDLE_VALUE;
		}

		inReconnect = true;
		try
		{
			Connect();
		}
		catch (const std::exception &reconnectErr)
		{
			inReconnect = false;
			fprintf(stderr, "[SCIPCClient] Reconnect failed: %s\n", reconnectErr.what());
			throw std::runtime_error(std::string("IPC reconnect failed after broken pipe: ") + reconnectErr.what());
		}
		inReconnect = false;

		// Re-open the pose shared-memory segment. vrserver crashing also destroys the
		// named file mapping that backs it; without this, the overlay's mapped view
		// silently detaches and ReadNewPoses returns zeros forever -- the overlay loops
		// "healthy" with no data. Done after Connect() (and its handshake) succeeds so
		// we know the new vrserver is up.
		ReopenShmem();

		// Retry once. If this also fails (broken pipe or otherwise),
		// propagate the exception to the caller as the original would have.
		Send(request);
		return Receive();
	}
}

void SCIPCClient::Send(const protocol::Request &request)
{
	DWORD bytesWritten;
	BOOL success = WriteFile(pipe, &request, sizeof request, &bytesWritten, 0);
	if (!success)
	{
		DWORD lastError = GetLastError();
		std::string msg = "Error writing IPC request. Error " + std::to_string(lastError) + ": " + LastErrorString(lastError);
		if (IsBrokenPipeError(lastError))
		{
			throw BrokenPipeException(msg, lastError);
		}
		throw std::runtime_error(msg);
	}
}

protocol::Response SCIPCClient::Receive()
{
	protocol::Response response(protocol::ResponseInvalid);
	DWORD bytesRead;

	BOOL success = ReadFile(pipe, &response, sizeof response, &bytesRead, 0);
	if (!success)
	{
		DWORD lastError = GetLastError();
		if (lastError != ERROR_MORE_DATA)
		{
			std::string msg = "Error reading IPC response. Error " + std::to_string(lastError) + ": " + LastErrorString(lastError);
			if (IsBrokenPipeError(lastError))
			{
				throw BrokenPipeException(msg, lastError);
			}
			throw std::runtime_error(msg);
		}

		// ERROR_MORE_DATA: the message in the pipe is larger than our Response buffer.
		// The kernel keeps the unread tail queued for the next ReadFile, which would
		// desync subsequent messages -- every following Receive() would parse part of
		// this message's tail as a fresh response. Drain the rest of the message
		// before throwing so the next caller starts on a clean message boundary.
		char drainBuf[1024];
		while (true)
		{
			DWORD drained = 0;
			BOOL drainSuccess = ReadFile(pipe, drainBuf, sizeof drainBuf, &drained, 0);
			if (drainSuccess) break; // last fragment of the oversized message
			DWORD drainErr = GetLastError();
			if (drainErr == ERROR_MORE_DATA) continue; // more to drain
			if (IsBrokenPipeError(drainErr))
			{
				throw BrokenPipeException("Pipe broken while draining oversized IPC response", drainErr);
			}
			// Some other error during drain -- give up draining and surface the original
			// SIZE_MISMATCH error so the caller knows something was wrong.
			break;
		}
		throw std::runtime_error(
			"Invalid IPC response. Error MESSAGE_TOO_LARGE, expected " + std::to_string(sizeof response) +
			" bytes but message was larger (drained the rest)."
		);
	}

	if (bytesRead != sizeof response)
	{
		throw std::runtime_error("Invalid IPC response. Error SIZE_MISMATCH, got size " + std::to_string(bytesRead));
	}

	return response;
}
