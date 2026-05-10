#include "stdafx.h"
#include "InputHealthIpcClient.h"

#include <stdexcept>
#include <string>

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
}

InputHealthIpcClient::~InputHealthIpcClient()
{
	Close();
}

void InputHealthIpcClient::Close()
{
	if (pipe_ && pipe_ != INVALID_HANDLE_VALUE) {
		CloseHandle(pipe_);
		pipe_ = INVALID_HANDLE_VALUE;
	}
}

void InputHealthIpcClient::Connect()
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

protocol::Response InputHealthIpcClient::SendBlocking(const protocol::Request &request)
{
	try {
		Send(request);
		return Receive();
	} catch (const BrokenPipeException &) {
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

void InputHealthIpcClient::Send(const protocol::Request &request)
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

protocol::Response InputHealthIpcClient::Receive()
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
