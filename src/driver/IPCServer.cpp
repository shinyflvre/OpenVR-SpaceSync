// SPDX-License-Identifier: AGPL-3.0-only
// Modified by Shinyflvres, 2026-08-23. Part of SpaceSync, a modified version of OpenVR-SpaceOverride by Nyabsi (AGPL-3.0). See NOTICE.md

#include "IPCServer.h"
#include "Logging.h"
#include "ServerTrackedDeviceProvider.h"

void IPCServer::HandleRequest(const protocol::Request &request, protocol::Response &response)
{
	switch (request.type)
	{
	case protocol::RequestHandshake:
		response.type = protocol::ResponseHandshake;
		response.protocol.version = protocol::Version;
		break;

	case protocol::RequestSetDeviceTransform:
		driver->SetDeviceTransform(request.setDeviceTransform);
		response.type = protocol::ResponseSuccess;
		break;

	case protocol::RequestSetHmdTracker:
		driver->SetHmdTracker(request.setHmdTracker);
		response.type = protocol::ResponseSuccess;
		break;

	case protocol::RequestSetSlamSync:
		driver->SetSlamSync(request.setSlamSync);
		response.type = protocol::ResponseSuccess;
		break;

	case protocol::RequestSetOneEuro:
		driver->SetOneEuro(request.setOneEuro);
		response.type = protocol::ResponseSuccess;
		break;

	case protocol::RequestGetStatus:
		driver->GetStatus(response.status);
		response.type = protocol::ResponseStatus;
		break;

	case protocol::RequestSetUniverseLock:
		driver->SetUniverseLock(request.setUniverseLock);
		response.type = protocol::ResponseSuccess;
		break;

	default:
		LOG("Invalid IPC request: %d", request.type);
		break;
	}
}

#ifndef _WIN32

#include <poll.h>
#include <sys/eventfd.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <vector>

// Linux: a SOCK_SEQPACKET unix socket in the abstract namespace keeps the
// message framing the Windows named pipe had (one request in, one response out).

IPCServer::~IPCServer()
{
	Stop();
}

void IPCServer::Run()
{
	listenFd = socket(AF_UNIX, SOCK_SEQPACKET | SOCK_CLOEXEC, 0);
	if (listenFd < 0)
	{
		LOG("IPC socket() failed: %s", strerror(errno));
		return;
	}

	sockaddr_un addr = {};
	addr.sun_family = AF_UNIX;
	// Leading NUL = abstract namespace: nothing on disk to clean up after a crash.
	const char name[] = SPACESYNC_SOCKET_NAME;
	memcpy(addr.sun_path + 1, name, sizeof name - 1);
	socklen_t len = offsetof(sockaddr_un, sun_path) + 1 + sizeof name - 1;

	if (bind(listenFd, (sockaddr *)&addr, len) != 0 || listen(listenFd, 8) != 0)
	{
		LOG("IPC bind/listen failed: %s", strerror(errno));
		close(listenFd);
		listenFd = -1;
		return;
	}

	wakeFd = eventfd(0, EFD_CLOEXEC);
	running = true;
	mainThread = std::thread(RunThread, this);
}

void IPCServer::Stop()
{
	TRACE("IPCServer::Stop()");
	if (!running)
		return;

	uint64_t one = 1;
	if (write(wakeFd, &one, sizeof one) < 0)
		LOG("IPC wake write failed: %s", strerror(errno));
	mainThread.join();
	running = false;

	for (int fd : clients)
		close(fd);
	clients.clear();
	close(listenFd);
	close(wakeFd);
	listenFd = wakeFd = -1;
	TRACE("IPCServer::Stop() finished");
}

void IPCServer::RunThread(IPCServer *_this)
{
	for (;;)
	{
		std::vector<pollfd> fds;
		fds.push_back({ _this->wakeFd, POLLIN, 0 });
		fds.push_back({ _this->listenFd, POLLIN, 0 });
		for (int fd : _this->clients)
			fds.push_back({ fd, POLLIN, 0 });

		if (poll(fds.data(), fds.size(), -1) < 0)
		{
			if (errno == EINTR)
				continue;
			LOG("IPC poll failed: %s", strerror(errno));
			return;
		}

		if (fds[0].revents)
			return;

		if (fds[1].revents & POLLIN)
		{
			int fd = accept4(_this->listenFd, nullptr, nullptr, SOCK_CLOEXEC);
			if (fd >= 0)
			{
				LOG("IPC client connected");
				_this->clients.insert(fd);
			}
		}

		for (size_t i = 2; i < fds.size(); i++)
		{
			if (!fds[i].revents)
				continue;

			int fd = fds[i].fd;
			protocol::Request request;
			protocol::Response response;
			ssize_t n = recv(fd, &request, sizeof request, 0);
			bool ok = n == (ssize_t)sizeof request;
			if (ok)
			{
				_this->HandleRequest(request, response);
				ok = send(fd, &response, sizeof response, MSG_NOSIGNAL) == (ssize_t)sizeof response;
			}

			if (!ok)
			{
				if (n == 0)
					LOG("IPC client disconnecting normally");
				else
					LOG("IPC client disconnecting due to error, bytes: %zd, error: %s", n, strerror(errno));
				close(fd);
				_this->clients.erase(fd);
			}
		}
	}
}

#else

IPCServer::~IPCServer()
{
	Stop();
}

void IPCServer::Run()
{
	mainThread = std::thread(RunThread, this);
}

void IPCServer::Stop()
{
	TRACE("IPCServer::Stop()");
	if (!running)
		return;

	stop = true;
	SetEvent(connectEvent);
	mainThread.join();
	running = false;
	TRACE("IPCServer::Stop() finished");
}

IPCServer::PipeInstance *IPCServer::CreatePipeInstance(HANDLE pipe)
{
	auto pipeInst = new PipeInstance;
	pipeInst->pipe = pipe;
	pipeInst->server = this;
	pipes.insert(pipeInst);
	return pipeInst;
}

void IPCServer::ClosePipeInstance(PipeInstance *pipeInst)
{
	DisconnectNamedPipe(pipeInst->pipe);
	CloseHandle(pipeInst->pipe);
	pipes.erase(pipeInst);
	delete pipeInst;
}

void IPCServer::RunThread(IPCServer *_this)
{
	_this->running = true;
	LPCTSTR pipeName = TEXT(SPACESYNC_PIPE_NAME);

	HANDLE connectEvent = _this->connectEvent = CreateEvent(0, TRUE, TRUE, 0);
	if (!connectEvent)
	{
		LOG("CreateEvent failed in RunThread. Error: %d", GetLastError());
		return;
	}

	OVERLAPPED connectOverlap;
	connectOverlap.hEvent = connectEvent;

	HANDLE nextPipe;
	BOOL connectPending = CreateAndConnectInstance(&connectOverlap, nextPipe);

	while (!_this->stop)
	{
		DWORD wait = WaitForSingleObjectEx(connectEvent, INFINITE, TRUE);

		if (_this->stop)
		{
			break;
		}
		else if (wait == 0)
		{
			// When connectPending is false, the last call to CreateAndConnectInstance
			// picked up a connected client and triggered this event, so we can simply
			// create a new pipe instance for it. If true, the client was still pending
			// connection when CreateAndConnectInstance returned, so this event was triggered
			// internally and we need to flush out the result, or something like that.
			if (connectPending)
			{
				DWORD bytesConnect;
				BOOL success = GetOverlappedResult(nextPipe, &connectOverlap, &bytesConnect, FALSE);
				if (!success)
				{
					LOG("GetOverlappedResult failed in RunThread. Error: %d", GetLastError());
					return;
				}
			}

			LOG("IPC client connected");

			auto pipeInst = _this->CreatePipeInstance(nextPipe);
			CompletedWriteCallback(0, sizeof protocol::Response, (LPOVERLAPPED) pipeInst);

			connectPending = CreateAndConnectInstance(&connectOverlap, nextPipe);
		}
		else if (wait != WAIT_IO_COMPLETION)
		{
			printf("WaitForSingleObjectEx failed in RunThread. Error %d", GetLastError());
			return;
		}
	}

	for (auto &pipeInst : _this->pipes)
	{
		_this->ClosePipeInstance(pipeInst);
	}
	_this->pipes.clear();
}

BOOL IPCServer::CreateAndConnectInstance(LPOVERLAPPED overlap, HANDLE &pipe)
{
	pipe = CreateNamedPipe(
		TEXT(SPACESYNC_PIPE_NAME),
		PIPE_ACCESS_DUPLEX | FILE_FLAG_OVERLAPPED,
		PIPE_TYPE_MESSAGE | PIPE_READMODE_MESSAGE | PIPE_WAIT,
		PIPE_UNLIMITED_INSTANCES,
		sizeof protocol::Request,
		sizeof protocol::Response,
		1000,
		0
	);

	if (pipe == INVALID_HANDLE_VALUE)
	{
		LOG("CreateNamedPipe failed. Error: %d", GetLastError());
		return FALSE;
	}

	ConnectNamedPipe(pipe, overlap);

	switch(GetLastError())
	{
	case ERROR_IO_PENDING:
		// Mark a pending connection by returning true, and when the connection
		// completes an event will trigger automatically.
		return TRUE;

	case ERROR_PIPE_CONNECTED:
		// Signal the event loop that a client is connected.
		if (SetEvent(overlap->hEvent))
			return FALSE;
	}

	LOG("ConnectNamedPipe failed. Error: %d", GetLastError());
	return FALSE;
}

void IPCServer::CompletedReadCallback(DWORD err, DWORD bytesRead, LPOVERLAPPED overlap)
{
	PipeInstance *pipeInst = (PipeInstance *) overlap;
	BOOL success = FALSE;

	if (err == 0 && bytesRead > 0)
	{
		pipeInst->server->HandleRequest(pipeInst->request, pipeInst->response);
		success = WriteFileEx(
			pipeInst->pipe,
			&pipeInst->response,
			sizeof protocol::Response,
			overlap,
			(LPOVERLAPPED_COMPLETION_ROUTINE) CompletedWriteCallback
		);
	}

	if (!success)
	{
		if (err == ERROR_BROKEN_PIPE)
		{
			LOG("IPC client disconnecting normally");
		}
		else
		{
			LOG("IPC client disconnecting due to error (via CompletedReadCallback), error: %d, bytesRead: %d", err, bytesRead);
		}
		pipeInst->server->ClosePipeInstance(pipeInst);
	}
}

void IPCServer::CompletedWriteCallback(DWORD err, DWORD bytesWritten, LPOVERLAPPED overlap)
{
	PipeInstance *pipeInst = (PipeInstance *) overlap;
	BOOL success = FALSE;

	if (err == 0 && bytesWritten == sizeof protocol::Response)
	{
		success = ReadFileEx(
			pipeInst->pipe,
			&pipeInst->request,
			sizeof protocol::Request,
			overlap,
			(LPOVERLAPPED_COMPLETION_ROUTINE) CompletedReadCallback
		);
	}

	if (!success)
	{
		LOG("IPC client disconnecting due to error (via CompletedWriteCallback), error: %d, bytesWritten: %d", err, bytesWritten);
		pipeInst->server->ClosePipeInstance(pipeInst);
	}
}

#endif
