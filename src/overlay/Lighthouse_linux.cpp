// SPDX-License-Identifier: AGPL-3.0-only
// Added by Lokinya (Linux port), 2026-10-30. Part of SpaceSync, a modified version of OpenVR-SpaceOverride by Nyabsi (AGPL-3.0). See NOTICE.md
//
// Linux stand-in for Lighthouse.cpp. Base station power control there talks
// Bluetooth LE through WinRT, which Linux doesn't have. Until a BlueZ backend
// exists, management reports itself unavailable and the rest of the app keeps
// working. Note() still logs, because Main.cpp uses it as the app log.

#include "Lighthouse.h"
#include "Platform.h"

#include <cstdarg>
#include <cstdio>
#include <ctime>
#include <mutex>

namespace lighthouse
{
	namespace
	{
		std::mutex logMutex;
		bool managed = false;
	}

	void Init() { }
	void Shutdown() { }
	void EnsureScanning() { }
	bool Scanning() { return false; }
	bool Available() { return false; }

	std::string AvailabilityError()
	{
		return "Base station power control is not available on Linux yet. Use Base Station Manager instead.";
	}

	std::vector<Station> Stations() { return {}; }
	void RequestPower(uint64_t, Power) { }
	void RequestPowerAll(Power) { }
	void RequestRefresh(uint64_t) { }
	void SetAutoWake(bool) { }
	void SetManaged(bool enabled) { managed = enabled; }
	bool Managed() { return managed; }
	void PowerDownAllAndWait(Power, int) { }
	void BeginPowerDownAll(Power) { }
	bool Idle() { return true; }

	void Note(const char *message)
	{
		std::lock_guard<std::mutex> lock(logMutex);
		static std::string path = platform::DataDir() + "/lighthouse.log";
		FILE *f = fopen(path.c_str(), "a");
		if (!f)
			return;
		time_t now = time(nullptr);
		tm t;
		localtime_r(&now, &t);
		fprintf(f, "%04d-%02d-%02d %02d:%02d:%02d app: %s\n",
			t.tm_year + 1900, t.tm_mon + 1, t.tm_mday, t.tm_hour, t.tm_min, t.tm_sec, message);
		fclose(f);
	}
}
