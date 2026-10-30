// SPDX-License-Identifier: AGPL-3.0-only

#pragma once

#include "Platform.h"

#include "Protocol.h"

class IPCClient
{
public:
	~IPCClient();

	void Connect();
	bool TryConnect();
#ifdef _WIN32
	bool IsConnected() const { return pipe && pipe != INVALID_HANDLE_VALUE; }
#else
	bool IsConnected() const { return fd >= 0; }
#endif
	protocol::Response SendBlocking(const protocol::Request &request);

	void Send(const protocol::Request &request);
	protocol::Response Receive();

private:
#ifdef _WIN32
	void ConnectInternal(DWORD waitMs = 1000);
#else
	void ConnectInternal(unsigned long waitMs = 1000);
#endif
	void Disconnect();

#ifdef _WIN32
	HANDLE pipe = INVALID_HANDLE_VALUE;
#else
	int fd = -1;
#endif
};