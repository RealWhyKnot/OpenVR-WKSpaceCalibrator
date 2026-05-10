#pragma once

#include "Protocol.h"

class InputHealthIpcClient
{
public:
	~InputHealthIpcClient();

	void Connect();
	void Close();
	protocol::Response SendBlocking(const protocol::Request &request);

	bool IsConnected() const { return pipe_ != INVALID_HANDLE_VALUE; }
	uint64_t ConnectionGeneration() const { return connection_generation_; }

private:
	void Send(const protocol::Request &request);
	protocol::Response Receive();

	HANDLE pipe_ = INVALID_HANDLE_VALUE;
	bool in_reconnect_ = false;
	uint64_t connection_generation_ = 0;
};
