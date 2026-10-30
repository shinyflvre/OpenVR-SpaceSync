// SPDX-License-Identifier: AGPL-3.0-only
// Added by Shinyflvres, 2026-08-25. Part of SpaceSync, a modified version of OpenVR-SpaceOverride by Nyabsi (AGPL-3.0). See NOTICE.md

#include "Sound.h"

#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <mmsystem.h>
#else
#include <SDL3/SDL.h>
#include <atomic>
#include <unistd.h>
#endif

#include <condition_variable>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <fstream>
#include <map>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace sound
{
	namespace
	{
		const double kVolume = 0.5;
		const double kTailSeconds = 0.1;
		const double kSilence = 0.02;

		std::mutex mutex;
		std::condition_variable wake;
		std::deque<std::pair<std::string, uint64_t>> queue;
		uint64_t generation = 0;
		std::map<std::string, std::vector<uint8_t>> cache;
		std::thread worker;
		bool running = false;
		std::string soundDir;

		std::string ExeDirectory()
		{
#ifdef _WIN32
			char path[MAX_PATH] = {};
			DWORD n = GetModuleFileNameA(nullptr, path, MAX_PATH);
#else
			char path[4096] = {};
			ssize_t n = readlink("/proc/self/exe", path, sizeof path - 1);
			if (n < 0) n = 0;
#endif
			std::string s(path, n);
			size_t slash = s.find_last_of("\\/");
			return slash == std::string::npos ? "." : s.substr(0, slash);
		}

		uint32_t ReadU32(const uint8_t* p) { return p[0] | (p[1] << 8) | (p[2] << 16) | (p[3] << 24); }
		uint16_t ReadU16(const uint8_t* p) { return (uint16_t)(p[0] | (p[1] << 8)); }

		void WriteU32(uint8_t* p, uint32_t v) { p[0] = v & 0xff; p[1] = (v >> 8) & 0xff; p[2] = (v >> 16) & 0xff; p[3] = (v >> 24) & 0xff; }

		void PrepareWav(std::vector<uint8_t>& wav)
		{
			if (wav.size() < 12 || std::memcmp(wav.data(), "RIFF", 4) != 0 || std::memcmp(wav.data() + 8, "WAVE", 4) != 0)
				return;
			size_t pos = 12;
			uint16_t format = 0, bits = 0, channels = 1;
			uint32_t rate = 44100;
			while (pos + 8 <= wav.size())
			{
				uint32_t size = ReadU32(wav.data() + pos + 4);
				const uint8_t* body = wav.data() + pos + 8;
				if (pos + 8 + size > wav.size())
					size = (uint32_t)(wav.size() - pos - 8);
				if (std::memcmp(wav.data() + pos, "fmt ", 4) == 0 && size >= 16)
				{
					format = ReadU16(body);
					channels = ReadU16(body + 2);
					rate = ReadU32(body + 4);
					bits = ReadU16(body + 14);
				}
				else if (std::memcmp(wav.data() + pos, "data", 4) == 0)
				{
					if (format == 1 && bits == 16 && channels > 0)
					{
						int16_t* samples = reinterpret_cast<int16_t*>(wav.data() + pos + 8);
						size_t count = size / 2;
						int peak = 0;
						for (size_t i = 0; i < count; i++)
							if (std::abs((int)samples[i]) > peak) peak = std::abs((int)samples[i]);
						size_t last = 0;
						int threshold = (int)(peak * kSilence);
						for (size_t i = 0; i < count; i++)
							if (std::abs((int)samples[i]) > threshold) last = i;
						size_t keep = last + 1 + (size_t)(kTailSeconds * rate) * channels;
						keep -= keep % channels;
						if (keep < count)
						{
							size = (uint32_t)(keep * 2);
							WriteU32(wav.data() + pos + 4, size);
							wav.resize(pos + 8 + size + (size & 1));
							WriteU32(wav.data() + 4, (uint32_t)(wav.size() - 8));
						}
						for (size_t i = 0; i < keep && i < count; i++)
							samples[i] = (int16_t)(samples[i] * kVolume);
					}
					return;
				}
				pos += 8 + size + (size & 1);
			}
		}

		const std::vector<uint8_t>* Load(const std::string& name)
		{
			auto it = cache.find(name);
			if (it != cache.end())
				return it->second.empty() ? nullptr : &it->second;

			std::vector<uint8_t>& buffer = cache[name];
			std::ifstream file(soundDir + "/sound/" + name + ".wav", std::ios::binary);
			if (file)
			{
				buffer.assign(std::istreambuf_iterator<char>(file), std::istreambuf_iterator<char>());
				PrepareWav(buffer);
			}
			return buffer.empty() ? nullptr : &buffer;
		}

#ifndef _WIN32
		// PlaySound(SND_MEMORY | SND_SYNC) stand-in: play one in-memory WAV and
		// return when it has finished, or as soon as Stop() asks us to.
		std::atomic<bool> abortPlayback{ false };

		void StopPlayback()
		{
			abortPlayback = true;
		}

		void PlayBlocking(const std::vector<uint8_t> &wav)
		{
			SDL_IOStream *io = SDL_IOFromConstMem(wav.data(), wav.size());
			SDL_AudioSpec spec;
			Uint8 *buf = nullptr;
			Uint32 len = 0;
			if (!io || !SDL_LoadWAV_IO(io, true, &spec, &buf, &len))
				return;

			SDL_AudioStream *stream = SDL_OpenAudioDeviceStream(SDL_AUDIO_DEVICE_DEFAULT_PLAYBACK, &spec, nullptr, nullptr);
			if (stream)
			{
				SDL_PutAudioStreamData(stream, buf, (int)len);
				SDL_FlushAudioStream(stream);
				SDL_ResumeAudioStreamDevice(stream);
				while (!abortPlayback && SDL_GetAudioStreamQueued(stream) > 0)
					SDL_Delay(10);
				// Let the device drain its own buffer before closing it.
				if (!abortPlayback)
					SDL_Delay(80);
				SDL_DestroyAudioStream(stream);
			}
			SDL_free(buf);
		}
#endif

		void WorkerLoop()
		{
			for (;;)
			{
				std::string name;
				uint64_t gen = 0;
				{
					std::unique_lock<std::mutex> lock(mutex);
					wake.wait(lock, [] { return !running || !queue.empty(); });
					if (!running)
						return;
					name = queue.front().first;
					gen = queue.front().second;
					queue.pop_front();
				}

				const std::vector<uint8_t>* wav = Load(name);
				if (!wav)
					continue;

				// Stop() between the pop above and here would otherwise let a superseded cue play
				// and push everything after it a step out of sync.
				{
					std::lock_guard<std::mutex> lock(mutex);
					if (!running || gen != generation)
						continue;
				}

#ifdef _WIN32
				PlaySoundA(reinterpret_cast<LPCSTR>(wav->data()), nullptr, SND_MEMORY | SND_SYNC | SND_NODEFAULT);
#else
				abortPlayback = false;
				PlayBlocking(*wav);
#endif
			}
		}
	}

	void Init()
	{
		std::lock_guard<std::mutex> lock(mutex);
		if (running)
			return;
		soundDir = ExeDirectory();
#ifndef _WIN32
		if (!SDL_InitSubSystem(SDL_INIT_AUDIO))
			return;
#endif
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
#ifdef _WIN32
		PlaySoundA(nullptr, nullptr, 0);
#else
		StopPlayback();
#endif
		if (worker.joinable())
			worker.join();
	}

	void Play(const char* name)
	{
		{
			std::lock_guard<std::mutex> lock(mutex);
			if (!running)
				return;
			queue.push_back({ name, generation });
		}
		wake.notify_one();
	}

	void ClearQueue()
	{
		std::lock_guard<std::mutex> lock(mutex);
		queue.clear();
		generation++;
	}

	void Stop()
	{
		ClearQueue();
#ifdef _WIN32
		PlaySoundA(nullptr, nullptr, 0);
#else
		StopPlayback();
#endif
	}
}
