// SPDX-License-Identifier: AGPL-3.0-only
// Modified by Shinyflvres, 2026-08-23. Part of SpaceSync, a modified version of OpenVR-SpaceOverride by Nyabsi (AGPL-3.0). See NOTICE.md

#include "IPCClient.h"

#include <string>
#include <stdexcept>

#ifndef _WIN32

#include <sys/socket.h>
#include <sys/un.h>
#include <sys/time.h>
#include <unistd.h>

#include <cerrno>
#include <cstddef>
#include <cstring>

// Linux side of the driver connection: same request/response messages as the
// Windows named pipe, carried over the abstract unix socket the driver listens on.

IPCClient::~IPCClient()
{
	Disconnect();
}

void IPCClient::Connect()
{
	const int maxAttempts = 3;
	unsigned long delayMs = 2000;

	for (int attempt = 1; attempt <= maxAttempts; ++attempt)
	{
		try
		{
			ConnectInternal();
			return;
		}
		catch (const std::runtime_error& e)
		{
			Disconnect();

			if (attempt >= 3)
				throw;

			fprintf(stderr, "IPC connect failed (attempt %d/3), retrying in %lums: %s\n", attempt, delayMs, e.what());
			Sleep(delayMs);
			delayMs *= 2;
		}
	}
}

bool IPCClient::TryConnect()
{
	try
	{
		ConnectInternal(100);
		return true;
	}
	catch (const std::runtime_error&)
	{
		Disconnect();
		return false;
	}
}

void IPCClient::Disconnect()
{
	if (fd >= 0)
		close(fd);
	fd = -1;
}

protocol::Response IPCClient::SendBlocking(const protocol::Request &request)
{
	if (!IsConnected())
		return protocol::Response(protocol::ResponseInvalid);
	try
	{
		Send(request);
		return Receive();
	}
	catch (const std::runtime_error&)
	{
		Disconnect();
		return protocol::Response(protocol::ResponseInvalid);
	}
}

void IPCClient::Send(const protocol::Request &request)
{
	if (send(fd, &request, sizeof request, MSG_NOSIGNAL) != (ssize_t)sizeof request)
		throw std::runtime_error(std::string("Error writing IPC request. Error: ") + strerror(errno));
}

protocol::Response IPCClient::Receive()
{
	protocol::Response response(protocol::ResponseInvalid);
	ssize_t n = recv(fd, &response, sizeof response, 0);
	if (n < 0)
		throw std::runtime_error(std::string("Error reading IPC response. Error: ") + strerror(errno));
	if (n != (ssize_t)sizeof response)
		throw std::runtime_error("Invalid IPC response with size " + std::to_string(n));
	return response;
}

void IPCClient::ConnectInternal(unsigned long waitMs)
{
	fd = socket(AF_UNIX, SOCK_SEQPACKET | SOCK_CLOEXEC, 0);
	if (fd < 0)
		throw std::runtime_error(std::string("socket() failed: ") + strerror(errno));

	sockaddr_un addr = {};
	addr.sun_family = AF_UNIX;
	const char name[] = SPACESYNC_SOCKET_NAME;
	memcpy(addr.sun_path + 1, name, sizeof name - 1);
	socklen_t len = offsetof(sockaddr_un, sun_path) + 1 + sizeof name - 1;

	if (connect(fd, (sockaddr *)&addr, len) != 0)
	{
		throw std::runtime_error("SpaceSync driver unavailable. Make sure SteamVR is running and the SpaceSync add-on is enabled in SteamVR settings.");
	}

	// The handshake must not hang the UI if the driver is wedged.
	timeval tv = { (time_t)(waitMs / 1000), (suseconds_t)((waitMs % 1000) * 1000) };
	setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof tv);

	Send(protocol::Request(protocol::RequestHandshake));
	auto response = Receive();

	// Normal requests can wait longer than the handshake.
	timeval longTv = { 2, 0 };
	setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &longTv, sizeof longTv);

	if (response.type != protocol::ResponseHandshake || response.protocol.version != protocol::Version)
	{
		throw std::runtime_error(
			"Incorrect driver version installed, try reinstalling SpaceSync. (Client: " +
			std::to_string(protocol::Version) + ", Driver: " +
			std::to_string(response.protocol.version) + ")"
		);
	}
}

#else

static std::string LastErrorString(DWORD lastError)
{
	LPSTR buffer = nullptr;
	size_t size = FormatMessageA(
		FORMAT_MESSAGE_ALLOCATE_BUFFER | FORMAT_MESSAGE_FROM_SYSTEM | FORMAT_MESSAGE_IGNORE_INSERTS,
		NULL, lastError, MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT), (LPSTR)&buffer, 0, NULL
	);

	std::string message(buffer, size);
	LocalFree(buffer);
	return message;
}

IPCClient::~IPCClient()
{
	if (pipe && pipe != INVALID_HANDLE_VALUE)
		CloseHandle(pipe);
}

void IPCClient::Connect()
{
	const int maxAttempts = 3;
	DWORD delayMs = 2000;

	for (int attempt = 1; attempt <= maxAttempts; ++attempt)
	{
		try
		{
			ConnectInternal();
			return;
		}
		catch (const std::runtime_error& e)
		{
			if (pipe && pipe != INVALID_HANDLE_VALUE)
				CloseHandle(pipe);
			pipe = INVALID_HANDLE_VALUE;

			if (attempt >= 3)
				throw;

			fprintf(stderr, "IPC connect failed (attempt %d/3), retrying in %lums: %s\n", attempt, delayMs, e.what());
			Sleep(delayMs);
			delayMs *= 2;
		}
	}
}

bool IPCClient::TryConnect()
{
	try
	{
		ConnectInternal(100);
		return true;
	}
	catch (const std::runtime_error&)
	{
		Disconnect();
		return false;
	}
}

void IPCClient::Disconnect()
{
	if (pipe && pipe != INVALID_HANDLE_VALUE)
		CloseHandle(pipe);
	pipe = INVALID_HANDLE_VALUE;
}

protocol::Response IPCClient::SendBlocking(const protocol::Request &request)
{
	if (!IsConnected())
		return protocol::Response(protocol::ResponseInvalid);
	try
	{
		Send(request);
		return Receive();
	}
	catch (const std::runtime_error&)
	{
		Disconnect();
		return protocol::Response(protocol::ResponseInvalid);
	}
}

void IPCClient::Send(const protocol::Request &request)
{
	DWORD bytesWritten;
	BOOL success = WriteFile(pipe, &request, sizeof request, &bytesWritten, 0);
	if (!success)
	{
		throw std::runtime_error("Error writing IPC request. Error: " + LastErrorString(GetLastError()));
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
			throw std::runtime_error("Error reading IPC response. Error: " + LastErrorString(lastError));
		}
	}

	if (bytesRead != sizeof response)
	{
		throw std::runtime_error("Invalid IPC response with size " + std::to_string(bytesRead));
	}

	return response;
}

void IPCClient::ConnectInternal(DWORD waitMs)
{
	LPCTSTR pipeName = TEXT(SPACESYNC_PIPE_NAME);
	WaitNamedPipe(pipeName, waitMs);
	pipe = CreateFile(pipeName, GENERIC_READ | GENERIC_WRITE, 0, 0, OPEN_EXISTING, 0, 0);
	if (pipe == INVALID_HANDLE_VALUE)
	{
		throw std::runtime_error("SpaceSync driver unavailable. Make sure SteamVR is running and the SpaceSync add-on is enabled in SteamVR settings.");
	}

	DWORD mode = PIPE_READMODE_MESSAGE;
	if (!SetNamedPipeHandleState(pipe, &mode, 0, 0))
	{
		throw std::runtime_error("Couldn't set pipe mode. Error: " + LastErrorString(GetLastError()));
	}

	Send(protocol::Request(protocol::RequestHandshake));
	auto response = Receive();

	if (response.type != protocol::ResponseHandshake || response.protocol.version != protocol::Version)
	{
		throw std::runtime_error(
			"Incorrect driver version installed, try reinstalling SpaceSync. (Client: " +
			std::to_string(protocol::Version) + ", Driver: " +
			std::to_string(response.protocol.version) + ")"
		);
	}
}

#endif
