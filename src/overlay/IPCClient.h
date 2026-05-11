#pragma once

#include "Protocol.h"

class SCIPCClient
{
public:
	~SCIPCClient();

	void Connect();
	protocol::Response SendBlocking(const protocol::Request &request);

	void Send(const protocol::Request &request);
	protocol::Response Receive();

	// True once Connect() has completed and the pipe handle is still alive.
	// Goes back to false if a broken-pipe error closed the handle and the
	// transparent reconnect attempt failed. The overlay UI uses this to
	// show a connection-status dot in the version line -- when false, the
	// user has likely uninstalled or disabled the SteamVR driver.
	bool IsConnected() const { return pipe != INVALID_HANDLE_VALUE; }

private:
	HANDLE pipe = INVALID_HANDLE_VALUE;
	bool inReconnect = false;
};