// SPDX-License-Identifier: AGPL-3.0-only
// Added by Shinyflvres, 2026-09-25. Part of SpaceSync, a modified version of OpenVR-SpaceOverride by Nyabsi (AGPL-3.0). See NOTICE.md

#include "Lighthouse.h"

#define WIN32_LEAN_AND_MEAN
#include <windows.h>

#include <winrt/Windows.Foundation.h>
#include <winrt/Windows.Foundation.Collections.h>
#include <winrt/Windows.Devices.Bluetooth.h>
#include <winrt/Windows.Devices.Bluetooth.Advertisement.h>
#include <winrt/Windows.Devices.Bluetooth.GenericAttributeProfile.h>
#include <winrt/Windows.Storage.Streams.h>

#include <chrono>
#include <condition_variable>
#include <cstdarg>
#include <cstdio>
#include <deque>
#include <map>
#include <mutex>
#include <thread>

using namespace winrt;
using namespace winrt::Windows::Devices::Bluetooth;
using namespace winrt::Windows::Devices::Bluetooth::Advertisement;
using namespace winrt::Windows::Devices::Bluetooth::GenericAttributeProfile;
using namespace winrt::Windows::Storage::Streams;

namespace lighthouse
{
	namespace
	{
		const guid kControlService{ 0x00001523, 0x1212, 0xefde, { 0x15, 0x23, 0x78, 0x5f, 0xea, 0xbc, 0xd1, 0x24 } };
		const guid kPowerCharacteristic{ 0x00001525, 0x1212, 0xefde, { 0x15, 0x23, 0x78, 0x5f, 0xea, 0xbc, 0xd1, 0x24 } };

		struct Command
		{
			uint64_t address;
			int mode;
		};

		std::mutex logMutex;

		std::string LogDirectory()
		{
			const char* localAppData = getenv("LOCALAPPDATA");
			if (localAppData)
			{
				std::string dir = std::string(localAppData) + "\\SpaceSync";
				CreateDirectoryA(dir.c_str(), NULL);
				return dir;
			}
			char path[1024] = {};
			GetModuleFileNameA(nullptr, path, sizeof path);
			std::string dir(path);
			size_t slash = dir.find_last_of("\\/");
			return slash == std::string::npos ? std::string(".") : dir.substr(0, slash);
		}

		void Log(const char* fmt, ...)
		{
			std::lock_guard<std::mutex> lock(logMutex);
			static std::string dir = LogDirectory();
			FILE* f = fopen((dir + "\\lighthouse.log").c_str(), "a");
			if (!f)
				return;
			SYSTEMTIME st;
			GetLocalTime(&st);
			fprintf(f, "[%02d:%02d:%02d.%03d] ", st.wHour, st.wMinute, st.wSecond, st.wMilliseconds);
			va_list args;
			va_start(args, fmt);
			vfprintf(f, fmt, args);
			va_end(args);
			fprintf(f, "\n");
			fclose(f);
		}

		std::mutex mutex;
		std::condition_variable wake;
		std::deque<Command> queue;
		std::map<uint64_t, Station> stations;
		std::thread worker;
		bool running = false;
		bool scanning = false;
		bool autoWake = false;
		bool enabled = true;
		bool available = true;
		std::string availabilityError;
		BluetoothLEAdvertisementWatcher watcher{ nullptr };

		struct PassiveInfo
		{
			std::chrono::steady_clock::time_point discovered;
			std::chrono::steady_clock::time_point commanded;
			Power commandedState = Power::Unknown;
			bool refreshRequested = false;
			int advertised = -1;
		};
		std::map<uint64_t, PassiveInfo> passive;

		void SetBusy(uint64_t address, bool busy, const char* error)
		{
			std::lock_guard<std::mutex> lock(mutex);
			auto it = stations.find(address);
			if (it == stations.end())
				return;
			it->second.busy = busy;
			it->second.error = error ? error : "";
		}

		void SetState(uint64_t address, Power state)
		{
			std::lock_guard<std::mutex> lock(mutex);
			auto it = stations.find(address);
			if (it != stations.end())
				it->second.state = state;
		}

		template <typename TAsync>
		static auto AwaitTimeout(TAsync const& op, int seconds)
		{
			if (op.wait_for(std::chrono::seconds(seconds)) != winrt::Windows::Foundation::AsyncStatus::Completed)
			{
				op.Cancel();
				throw hresult_error(HRESULT_FROM_WIN32(ERROR_TIMEOUT));
			}
			return op.GetResults();
		}

		Power DecodePower(uint8_t raw)
		{
			if (raw == 0x00)
				return Power::Sleep;
			if (raw == 0x02)
				return Power::Standby;
			return Power::Awake;
		}

		GattCharacteristic FindPowerCharacteristic(BluetoothLEDevice& device)
		{
			auto services = AwaitTimeout(device.GetGattServicesForUuidAsync(kControlService, BluetoothCacheMode::Uncached), 6);
			if (services.Status() != GattCommunicationStatus::Success || services.Services().Size() == 0)
				return nullptr;
			for (auto const& service : services.Services())
			{
				auto chars = AwaitTimeout(service.GetCharacteristicsForUuidAsync(kPowerCharacteristic, BluetoothCacheMode::Uncached), 6);
				if (chars.Status() == GattCommunicationStatus::Success && chars.Characteristics().Size() > 0)
					return chars.Characteristics().GetAt(0);
			}
			return nullptr;
		}

		const char* Attempt(const Command& cmd)
		{
			const char* failure = nullptr;
			try
			{
				BluetoothLEDevice device = AwaitTimeout(BluetoothLEDevice::FromBluetoothAddressAsync(cmd.address), 6);
				if (!device)
					failure = "not reachable";
				else
				{
					GattCharacteristic ch = FindPowerCharacteristic(device);
					if (!ch)
						failure = "no power control found";
					else if (cmd.mode < 0)
					{
						auto read = AwaitTimeout(ch.ReadValueAsync(BluetoothCacheMode::Uncached), 6);
						if (read.Status() != GattCommunicationStatus::Success)
							failure = "state read failed";
						else
						{
							auto buffer = read.Value();
							if (buffer && buffer.Length() > 0)
							{
								DataReader reader = DataReader::FromBuffer(buffer);
								uint8_t raw = reader.ReadByte();
								Power state = DecodePower(raw);
								SetState(cmd.address, state);
								Log("%012llx reports state 0x%02x (%s)", (unsigned long long)cmd.address, raw,
									state == Power::Sleep ? "sleeping" : (state == Power::Standby ? "standby" : "awake"));
							}
						}
					}
					else
					{
						uint8_t value = cmd.mode == (int)Power::Awake ? 0x01 : (cmd.mode == (int)Power::Standby ? 0x02 : 0x00);
						DataWriter writer;
						writer.WriteByte(value);
						auto status = AwaitTimeout(ch.WriteValueAsync(writer.DetachBuffer(), GattWriteOption::WriteWithResponse), 6);
						if (status != GattCommunicationStatus::Success)
							failure = "write rejected (old firmware?)";
						else
						{
							SetState(cmd.address, (Power)cmd.mode);
							std::lock_guard<std::mutex> lock(mutex);
							auto& info = passive[cmd.address];
							info.commanded = std::chrono::steady_clock::now();
							info.commandedState = (Power)cmd.mode;
						}
					}
				}
				if (device)
					device.Close();
			}
			catch (...)
			{
				failure = "bluetooth error";
			}
			return failure;
		}

		void Process(const Command& cmd)
		{
			SetBusy(cmd.address, true, nullptr);
			const char* what = cmd.mode < 0 ? "refresh" : (cmd.mode == (int)Power::Awake ? "wake" : (cmd.mode == (int)Power::Standby ? "standby" : "sleep"));
			const char* failure = "cancelled";
			for (int attempt = 0; attempt < 3; attempt++)
			{
				if (attempt > 0)
					std::this_thread::sleep_for(std::chrono::milliseconds(400));
				{
					std::lock_guard<std::mutex> lock(mutex);
					if (!running || !enabled)
						break;
				}
				failure = Attempt(cmd);
				if (!failure)
					break;
				Log("%012llx %s attempt %d failed: %s", (unsigned long long)cmd.address, what, attempt + 1, failure);
			}
			Log("%012llx %s -> %s", (unsigned long long)cmd.address, what, failure ? failure : "ok");
			SetBusy(cmd.address, false, failure);
		}

		void WorkerLoop()
		{
			init_apartment(apartment_type::multi_threaded);
			for (;;)
			{
				Command cmd{};
				{
					std::unique_lock<std::mutex> lock(mutex);
					wake.wait(lock, [] { return !running || !queue.empty(); });
					if (!running)
						break;
					cmd = queue.front();
					queue.pop_front();
				}
				Process(cmd);
			}
			uninit_apartment();
		}

		int AdvertisedPower(BluetoothLEAdvertisementReceivedEventArgs const& args)
		{
			for (auto const& data : args.Advertisement().ManufacturerData())
			{
				if (data.CompanyId() != 0x055D || data.Data().Length() < 7)
					continue;
				DataReader reader = DataReader::FromBuffer(data.Data());
				uint8_t bytes[7];
				for (auto& b : bytes)
					b = reader.ReadByte();
				return bytes[4];
			}
			return -1;
		}

		void ApplyAdvertisedPower(uint64_t address, int raw)
		{
			bool changed = false;
			Power state = DecodePower((uint8_t)raw);
			{
				std::lock_guard<std::mutex> lock(mutex);
				auto it = stations.find(address);
				if (it == stations.end())
					return;
				auto& info = passive[address];
				auto now = std::chrono::steady_clock::now();
				bool recentCommand = info.commandedState != Power::Unknown && now - info.commanded < std::chrono::seconds(8);
				if (it->second.busy || (recentCommand && state != info.commandedState))
					return;
				changed = info.advertised != raw;
				info.advertised = raw;
				it->second.state = state;
			}
			if (changed)
				Log("%012llx advertises state 0x%02x (%s)", (unsigned long long)address, raw,
					state == Power::Sleep ? "sleeping" : (state == Power::Standby ? "standby" : "awake"));
		}

		void OnAdvertisement(BluetoothLEAdvertisementWatcher const&, BluetoothLEAdvertisementReceivedEventArgs const& args)
		{
			uint64_t address = args.BluetoothAddress();
			int advertised = AdvertisedPower(args);

			hstring localName = args.Advertisement().LocalName();
			std::wstring w(localName.c_str());
			bool named = localName.size() >= 5 && w.compare(0, 4, L"LHB-") == 0 && w != L"LHB-00000000";

			bool fresh = false;
			bool wake = false;
			bool refresh = false;
			std::string name(w.begin(), w.end());
			{
				std::lock_guard<std::mutex> lock(mutex);
				if (!enabled)
					return;
				auto it = stations.find(address);
				if (it == stations.end())
				{
					if (!named)
						return;
					Station s;
					s.address = address;
					s.name = name;
					s.rssi = args.RawSignalStrengthInDBm();
					stations[address] = s;
					passive[address].discovered = std::chrono::steady_clock::now();
					fresh = true;
					wake = autoWake;
				}
				else
				{
					it->second.rssi = args.RawSignalStrengthInDBm();
					auto& info = passive[address];
					if (it->second.state == Power::Unknown && advertised < 0 && !info.refreshRequested && !it->second.busy
						&& std::chrono::steady_clock::now() - info.discovered > std::chrono::seconds(3))
					{
						info.refreshRequested = true;
						refresh = true;
					}
				}
			}
			if (advertised >= 0)
				ApplyAdvertisedPower(address, advertised);
			if (fresh)
			{
				Log("found %s (%012llx, %d dBm)%s", name.c_str(), (unsigned long long)address, args.RawSignalStrengthInDBm(), wake ? ", auto-wake" : "");
				if (wake)
					RequestPower(address, Power::Awake);
			}
			if (refresh)
			{
				Log("%012llx sends no power state in its advertisement, reading it over GATT", (unsigned long long)address);
				RequestRefresh(address);
			}
		}
	}

	void Init()
	{
		std::lock_guard<std::mutex> lock(mutex);
		if (running)
			return;
		running = true;
		worker = std::thread(WorkerLoop);
	}

	void Shutdown()
	{
		{
			std::lock_guard<std::mutex> lock(mutex);
			if (!running)
				return;
			running = false;
			queue.clear();
		}
		wake.notify_all();
		if (worker.joinable())
		{
			HANDLE h = (HANDLE)worker.native_handle();
			DWORD rc = WaitForSingleObject(h, 10000);
			if (rc == WAIT_OBJECT_0)
				worker.join();
			else
			{
				Log("shutdown: worker still blocked in bluetooth, detaching");
				worker.detach();
			}
		}
		try
		{
			if (scanning && watcher)
				watcher.Stop();
		}
		catch (...) {}
		watcher = nullptr;
		scanning = false;
	}

	void EnsureScanning()
	{
		{
			std::lock_guard<std::mutex> lock(mutex);
			if (scanning || !available || !enabled)
				return;
		}
		bool ok = true;
		BluetoothLEAdvertisementWatcher created{ nullptr };
		try
		{
			created = BluetoothLEAdvertisementWatcher();
			created.ScanningMode(BluetoothLEScanningMode::Active);
			created.Received(&OnAdvertisement);
			created.Start();
		}
		catch (...)
		{
			ok = false;
		}
		std::lock_guard<std::mutex> lock(mutex);
		if (ok)
		{
			watcher = created;
			scanning = true;
			Log("scanning started");
		}
		else
		{
			available = false;
			availabilityError = "Bluetooth LE is not available on this system";
			Log("bluetooth LE unavailable");
		}
	}

	bool Scanning()
	{
		std::lock_guard<std::mutex> lock(mutex);
		return scanning;
	}

	bool Available()
	{
		std::lock_guard<std::mutex> lock(mutex);
		return available;
	}

	std::string AvailabilityError()
	{
		std::lock_guard<std::mutex> lock(mutex);
		return availabilityError;
	}

	std::vector<Station> Stations()
	{
		std::lock_guard<std::mutex> lock(mutex);
		std::vector<Station> out;
		for (auto const& kv : stations)
			out.push_back(kv.second);
		return out;
	}

	void RequestPower(uint64_t address, Power mode)
	{
		{
			std::lock_guard<std::mutex> lock(mutex);
			if (!running || !enabled)
				return;
			queue.push_back({ address, (int)mode });
		}
		wake.notify_one();
	}

	void RequestPowerAll(Power mode)
	{
		{
			std::lock_guard<std::mutex> lock(mutex);
			if (!running || !enabled)
				return;
			for (auto const& kv : stations)
				queue.push_back({ kv.first, (int)mode });
		}
		wake.notify_one();
	}

	void RequestRefresh(uint64_t address)
	{
		{
			std::lock_guard<std::mutex> lock(mutex);
			if (!running || !enabled)
				return;
			queue.push_back({ address, -1 });
		}
		wake.notify_one();
	}

	void Note(const char* message)
	{
		Log("app: %s", message);
	}

	void BeginStandbyAll()
	{
		size_t known = 0;
		{
			std::lock_guard<std::mutex> lock(mutex);
			known = stations.size();
		}
		Log("exit: standby initiated for %d station(s), window stays until done", (int)known);
		RequestPowerAll(Power::Standby);
	}

	bool Idle()
	{
		std::lock_guard<std::mutex> lock(mutex);
		if (!queue.empty())
			return false;
		for (auto const& kv : stations)
			if (kv.second.busy)
				return false;
		return true;
	}

	void SetAutoWake(bool enabled)
	{
		std::lock_guard<std::mutex> lock(mutex);
		autoWake = enabled;
		Log("auto-wake %s", enabled ? "enabled" : "disabled");
	}

	void SetEnabled(bool value)
	{
		BluetoothLEAdvertisementWatcher stopped{ nullptr };
		{
			std::lock_guard<std::mutex> lock(mutex);
			if (enabled == value)
				return;
			enabled = value;
			if (!enabled)
			{
				queue.clear();
				stations.clear();
				passive.clear();
				stopped = watcher;
				watcher = nullptr;
				scanning = false;
			}
		}
		Log("basestation control %s", value ? "enabled" : "disabled");
		try
		{
			if (stopped)
				stopped.Stop();
		}
		catch (...) {}
	}

	void StandbyAllAndWait(int timeoutMs)
	{
		size_t known = 0;
		{
			std::lock_guard<std::mutex> lock(mutex);
			known = stations.size();
		}
		int budget = (int)known * 6000;
		if (budget < timeoutMs) budget = timeoutMs;
		timeoutMs = budget;
		Log("exit: sending standby to %d known station(s), budget %d ms", (int)known, timeoutMs);
		RequestPowerAll(Power::Standby);
		auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeoutMs);
		for (;;)
		{
			{
				std::lock_guard<std::mutex> lock(mutex);
				if (!running)
					return;
				bool pending = !queue.empty();
				for (auto const& kv : stations)
					pending = pending || kv.second.busy;
				if (!pending)
					break;
			}
			if (std::chrono::steady_clock::now() > deadline)
			{
				Log("exit: standby wait timed out");
				return;
			}
			std::this_thread::sleep_for(std::chrono::milliseconds(50));
		}
		Log("exit: standby commands completed, verifying states");

		{
			std::lock_guard<std::mutex> lock(mutex);
			if (!running)
				return;
			for (auto const& kv : stations)
				queue.push_back({ kv.first, -1 });
		}
		wake.notify_one();
		auto verifyDeadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeoutMs);
		for (;;)
		{
			{
				std::lock_guard<std::mutex> lock(mutex);
				if (!running)
					return;
				bool pending = !queue.empty();
				for (auto const& kv : stations)
					pending = pending || kv.second.busy;
				if (!pending)
				{
					Log("exit: state verification done");
					return;
				}
			}
			if (std::chrono::steady_clock::now() > verifyDeadline)
			{
				Log("exit: state verification timed out");
				return;
			}
			std::this_thread::sleep_for(std::chrono::milliseconds(50));
		}
	}
}
