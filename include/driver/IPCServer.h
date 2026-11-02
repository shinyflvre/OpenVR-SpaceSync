// SPDX-License-Identifier: AGPL-3.0-only

#pragma once

#include "Protocol.h"

#include <thread>
#include <set>
#include <mutex>

#include "Platform.h"

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

#ifndef _WIN32
	static void RunThread(IPCServer *_this);

	std::thread mainThread;
	bool running = false;
	int listenFd = -1;
	int wakeFd = -1; // eventfd, poked by Stop()
	std::set<int> clients;

	ServerTrackedDeviceProvider *driver;
};
#else
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

	bool running = false;
	bool stop = false;

	std::set<PipeInstance *> pipes;
	HANDLE connectEvent;

	ServerTrackedDeviceProvider *driver;
};
#endif
