// SPDX-License-Identifier: AGPL-3.0-only
// Added by Lokinya (Linux port), 2026-10-30. Part of SpaceSync, a modified version of OpenVR-SpaceOverride by Nyabsi (AGPL-3.0). See NOTICE.md
//
// Small stand-ins for the handful of Win32 calls SpaceSync uses, so the same
// code builds on Linux. On Windows this just pulls in <windows.h>.

#pragma once

#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#else

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <string>
#include <strings.h>
#include <sys/stat.h>
#include <unistd.h>

union LARGE_INTEGER
{
	struct { uint32_t LowPart; int32_t HighPart; };
	int64_t QuadPart;
};

// Nanosecond monotonic clock; frequency is therefore 1e9.
inline bool QueryPerformanceCounter(LARGE_INTEGER *out)
{
	timespec ts;
	clock_gettime(CLOCK_MONOTONIC, &ts);
	out->QuadPart = (int64_t)ts.tv_sec * 1000000000ll + ts.tv_nsec;
	return true;
}

inline bool QueryPerformanceFrequency(LARGE_INTEGER *out)
{
	out->QuadPart = 1000000000ll;
	return true;
}

inline void Sleep(unsigned long ms)
{
	usleep((useconds_t)ms * 1000);
}

inline int localtime_s(tm *out, const time_t *t)
{
	return localtime_r(t, out) ? 0 : 1;
}

#define _stricmp strcasecmp
#define _fseeki64 fseeko
#define _ftelli64 ftello

struct SYSTEMTIME
{
	int wYear, wMonth, wDayOfWeek, wDay, wHour, wMinute, wSecond, wMilliseconds;
};

inline void GetLocalTime(SYSTEMTIME *out)
{
	timespec ts;
	clock_gettime(CLOCK_REALTIME, &ts);
	tm t;
	localtime_r(&ts.tv_sec, &t);
	*out = { t.tm_year + 1900, t.tm_mon + 1, t.tm_wday, t.tm_mday, t.tm_hour, t.tm_min, t.tm_sec, (int)(ts.tv_nsec / 1000000) };
}

namespace platform
{
	// ~/.local/share/SpaceSync (created on demand). Holds logs and the profile.
	inline std::string DataDir()
	{
		std::string dir;
		if (const char *xdg = getenv("XDG_DATA_HOME"); xdg && *xdg)
			dir = std::string(xdg) + "/SpaceSync";
		else if (const char *home = getenv("HOME"); home && *home)
			dir = std::string(home) + "/.local/share/SpaceSync";
		else
			dir = "/tmp/SpaceSync";
		for (size_t i = 1; i <= dir.size(); i++)
			if (i == dir.size() || dir[i] == '/')
				mkdir(dir.substr(0, i).c_str(), 0755);
		return dir;
	}
}

#endif

#ifndef _WIN32
// Abstract-namespace unix socket shared by the driver and the app.
#define SPACESYNC_SOCKET_NAME "SpaceSyncCom"
#endif
