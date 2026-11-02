// SPDX-License-Identifier: AGPL-3.0-only
// Modified by Shinyflvres, 2026-08-23. Part of SpaceSync, a modified version of OpenVR-SpaceOverride by Nyabsi (AGPL-3.0). See NOTICE.md

#define _CRT_SECURE_NO_DEPRECATE
#include "Logging.h"
#include "Platform.h"
#include <chrono>

FILE *LogFile;

void OpenLogFile()
{
#ifdef _WIN32
	LogFile = fopen("spacesync_driver.log", "a");
#else
	LogFile = fopen((platform::DataDir() + "/spacesync_driver.log").c_str(), "a");
#endif
	if (LogFile == nullptr)
	{
		LogFile = stderr;
	}
}

void CloseLogFile()
{
	int result = fclose(LogFile);
	if (result != 0)
		std::exit(EXIT_FAILURE);
}

tm TimeForLog()
{
	auto now = std::chrono::system_clock::now();
	auto nowTime = std::chrono::system_clock::to_time_t(now);
	tm value;
	auto tm = localtime_s(&value, &nowTime);
	return value;
}

void LogFlush()
{
	fflush(LogFile);
}