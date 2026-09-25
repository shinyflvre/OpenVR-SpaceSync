// SPDX-License-Identifier: AGPL-3.0-only
// Added by Shinyflvres, 2026-09-25. Part of SpaceSync, a modified version of OpenVR-SpaceOverride by Nyabsi (AGPL-3.0). See NOTICE.md

#include "Lighthouse.h"

#include <winrt/Windows.Foundation.h>
#include <winrt/Windows.Foundation.Collections.h>
#include <winrt/Windows.Devices.Bluetooth.h>
#include <winrt/Windows.Devices.Bluetooth.Advertisement.h>
#include <winrt/Windows.Devices.Bluetooth.GenericAttributeProfile.h>
#include <winrt/Windows.Storage.Streams.h>

#include <condition_variable>
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

		std::mutex mutex;
		std::condition_variable wake;
		std::deque<Command> queue;
		std::map<uint64_t, Station> stations;
		std::thread worker;
		bool running = false;
		bool scanning = false;
		bool available = true;
		std::string availabilityError;
		BluetoothLEAdvertisementWatcher watcher{ nullptr };

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
			auto services = device.GetGattServicesForUuidAsync(kControlService, BluetoothCacheMode::Uncached).get();
			if (services.Status() != GattCommunicationStatus::Success || services.Services().Size() == 0)
				return nullptr;
			for (auto const& service : services.Services())
			{
				auto chars = service.GetCharacteristicsForUuidAsync(kPowerCharacteristic, BluetoothCacheMode::Uncached).get();
				if (chars.Status() == GattCommunicationStatus::Success && chars.Characteristics().Size() > 0)
					return chars.Characteristics().GetAt(0);
			}
			return nullptr;
		}

		void Process(const Command& cmd)
		{
			SetBusy(cmd.address, true, nullptr);
			const char* failure = nullptr;
			try
			{
				BluetoothLEDevice device = BluetoothLEDevice::FromBluetoothAddressAsync(cmd.address).get();
				if (!device)
					failure = "not reachable";
				else
				{
					GattCharacteristic ch = FindPowerCharacteristic(device);
					if (!ch)
						failure = "no power control found";
					else if (cmd.mode < 0)
					{
						auto read = ch.ReadValueAsync(BluetoothCacheMode::Uncached).get();
						if (read.Status() != GattCommunicationStatus::Success)
							failure = "state read failed";
						else
						{
							auto buffer = read.Value();
							if (buffer && buffer.Length() > 0)
							{
								DataReader reader = DataReader::FromBuffer(buffer);
								SetState(cmd.address, DecodePower(reader.ReadByte()));
							}
						}
					}
					else
					{
						uint8_t value = cmd.mode == (int)Power::Awake ? 0x01 : (cmd.mode == (int)Power::Standby ? 0x02 : 0x00);
						DataWriter writer;
						writer.WriteByte(value);
						auto status = ch.WriteValueAsync(writer.DetachBuffer(), GattWriteOption::WriteWithResponse).get();
						if (status != GattCommunicationStatus::Success)
							failure = "write rejected (old firmware?)";
						else
							SetState(cmd.address, (Power)cmd.mode);
					}
				}
				if (device)
					device.Close();
			}
			catch (...)
			{
				failure = "bluetooth error";
			}
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

		void OnAdvertisement(BluetoothLEAdvertisementWatcher const&, BluetoothLEAdvertisementReceivedEventArgs const& args)
		{
			hstring localName = args.Advertisement().LocalName();
			if (localName.size() < 5)
				return;
			std::wstring w(localName.c_str());
			if (w.compare(0, 4, L"LHB-") != 0)
				return;
			if (w == L"LHB-00000000")
				return;

			std::string name(w.begin(), w.end());
			uint64_t address = args.BluetoothAddress();
			bool fresh = false;
			{
				std::lock_guard<std::mutex> lock(mutex);
				auto it = stations.find(address);
				if (it == stations.end())
				{
					Station s;
					s.address = address;
					s.name = name;
					s.rssi = args.RawSignalStrengthInDBm();
					stations[address] = s;
					fresh = true;
				}
				else
					it->second.rssi = args.RawSignalStrengthInDBm();
			}
			if (fresh)
				RequestRefresh(address);
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
			worker.join();
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
			if (scanning || !available)
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
		}
		else
		{
			available = false;
			availabilityError = "Bluetooth LE is not available on this system";
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
			if (!running)
				return;
			queue.push_back({ address, (int)mode });
		}
		wake.notify_one();
	}

	void RequestPowerAll(Power mode)
	{
		{
			std::lock_guard<std::mutex> lock(mutex);
			if (!running)
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
			if (!running)
				return;
			queue.push_back({ address, -1 });
		}
		wake.notify_one();
	}
}
