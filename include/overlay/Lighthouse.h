// SPDX-License-Identifier: AGPL-3.0-only
// Added by Shinyflvres, 2026-09-25. Part of SpaceSync, a modified version of OpenVR-SpaceOverride by Nyabsi (AGPL-3.0). See NOTICE.md

#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace lighthouse
{
	enum class Power { Unknown = -1, Sleep = 0, Awake = 1, Standby = 2 };

	struct Station
	{
		uint64_t address = 0;
		std::string name;
		Power state = Power::Unknown;
		bool busy = false;
		std::string error;
		int rssi = 0;
	};

	void Init();
	void Shutdown();
	void EnsureScanning();
	bool Scanning();
	bool Available();
	std::string AvailabilityError();
	std::vector<Station> Stations();
	void RequestPower(uint64_t address, Power mode);
	void RequestPowerAll(Power mode);
	void RequestRefresh(uint64_t address);
}
