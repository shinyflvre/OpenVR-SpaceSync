// SPDX-License-Identifier: AGPL-3.0-only

#pragma once

#include <cstdio>
#include <ctime>

extern FILE *LogFile;

void OpenLogFile();
void CloseLogFile();

tm TimeForLog();
void LogFlush();

#ifndef LOG
#define LOG(fmt, ...) do { \
	tm logNow = TimeForLog(); \
	fprintf(LogFile, "[%02d:%02d:%02d] " fmt "\n", logNow.tm_hour, logNow.tm_min, logNow.tm_sec __VA_OPT__(,) __VA_ARGS__); \
	LogFlush(); \
} while (0)
#endif

#define TRACE(...) {}

#ifndef TRACE
#define TRACE LOG
#endif
