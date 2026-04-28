#include "stdafx.h"
#include "IPCClient.h"

#include <cstdio>
#include <string>

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

IPCClient::~IPCClient()
{
	if (pipe && pipe != INVALID_HANDLE_VALUE)
		CloseHandle(pipe);
}

void IPCClient::Connect()
{
	LPCTSTR pipeName = TEXT(OPENVR_SPACECALIBRATOR_PIPE_NAME);

	WaitNamedPipe(pipeName, 1000);
	pipe = CreateFile(pipeName, GENERIC_READ | GENERIC_WRITE, 0, 0, OPEN_EXISTING, 0, 0);

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

protocol::Response IPCClient::SendBlocking(const protocol::Request &request)
{
	try
	{
		Send(request);
		return Receive();
	}
	catch (const BrokenPipeException &e)
	{
		// If we're already inside a reconnect attempt (e.g. handshake-after-reconnect
		// hit another broken pipe), don't recurse — let the caller handle it.
		if (inReconnect)
			throw;

		// SteamVR's vrserver may have restarted, leaving us with a stale pipe.
		// Try to reconnect once and re-issue the request transparently.
		fprintf(stderr,
			"[IPCClient] Broken pipe (error %lu) during request; attempting reconnect...\n",
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
			fprintf(stderr, "[IPCClient] Reconnect failed: %s\n", reconnectErr.what());
			throw std::runtime_error(std::string("IPC reconnect failed after broken pipe: ") + reconnectErr.what());
		}
		inReconnect = false;

		// Retry once. If this also fails (broken pipe or otherwise),
		// propagate the exception to the caller as the original would have.
		Send(request);
		return Receive();
	}
}

void IPCClient::Send(const protocol::Request &request)
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

protocol::Response IPCClient::Receive()
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
	}

	if (bytesRead != sizeof response)
	{
		throw std::runtime_error("Invalid IPC response. Error SIZE_MISMATCH, got size " + std::to_string(bytesRead));
	}

	return response;
}
