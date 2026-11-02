// SPDX-License-Identifier: AGPL-3.0-only

#pragma once

#include "Logging.h"
#ifdef _WIN32
#include <MinHook.h>
#else
#include <sys/mman.h>
#include <unistd.h>
#endif
#include <cstdint>
#include <map>
#include <string>

class IHook
{
public:
	const std::string name;

	IHook(const std::string &name) : name(name) { }
	virtual ~IHook() { }

	virtual void Destroy() = 0;

	static bool Exists(const std::string &name);
	static void Register(IHook *hook);
	static void Unregister(IHook *hook);
	static void DestroyAll();

private:
	static std::map<std::string, IHook *> hooks;
};

template<class FuncType> class Hook : public IHook
{
public:
	FuncType originalFunc = nullptr;
	Hook(const std::string &name) : IHook(name) { }

#ifndef _WIN32
	// Linux: no MinHook, so swap the function pointer in the vtable slot itself.
	// Every object of that class shares the vtable, which is what we want here.
	bool CreateHookInObjectVTable(void *object, int vtableOffset, void *detourFunction)
	{
		void **vtable = *((void ***)object);
		slot = &vtable[vtableOffset];
		if (*slot == detourFunction)
			return false;

		originalFunc = (FuncType)*slot;
		if (!WriteSlot(detourFunction))
		{
			LOG("Failed to make vtable writable for %s", name.c_str());
			originalFunc = nullptr;
			slot = nullptr;
			return false;
		}

		LOG("Enabled hook for %s", name.c_str());
		enabled = true;
		detour = detourFunction;
		return true;
	}

	void Destroy()
	{
		// Only put the original back if nobody hooked on top of us since.
		if (enabled && slot && *slot == detour)
			WriteSlot((void *)originalFunc);
		enabled = false;
	}

private:
	bool WriteSlot(void *value)
	{
		long page = sysconf(_SC_PAGESIZE);
		void *start = (void *)((uintptr_t)slot & ~(uintptr_t)(page - 1));
		// The slot may straddle a page boundary, so unprotect two pages.
		if (mprotect(start, page * 2, PROT_READ | PROT_WRITE) != 0)
			return false;
		*slot = value;
		mprotect(start, page * 2, PROT_READ);
		return true;
	}

	void **slot = nullptr;
	void *detour = nullptr;
	bool enabled = false;
#else
	bool CreateHookInObjectVTable(void *object, int vtableOffset, void *detourFunction)
	{
		// For virtual objects, VC++ adds a pointer to the vtable as the first member.
		// To access the vtable, we simply dereference the object.
		void **vtable = *((void ***)object);

		// The vtable itself is an array of pointers to member functions,
		// in the order they were declared in.
		targetFunc = vtable[vtableOffset];

		auto err = MH_CreateHook(targetFunc, detourFunction, (LPVOID *)&originalFunc);
		if (err != MH_OK)
		{
			LOG("Failed to create hook for %s, error: %s", name.c_str(), MH_StatusToString(err));
			return false;
		}

		err = MH_EnableHook(targetFunc);
		if (err != MH_OK)
		{
			LOG("Failed to enable hook for %s, error: %s", name.c_str(), MH_StatusToString(err));
			MH_RemoveHook(targetFunc);
			return false;
		}

		LOG("Enabled hook for %s", name.c_str());
		enabled = true;
		return true;
	}

	void Destroy()
	{
		if (enabled)
		{
			MH_RemoveHook(targetFunc);
			enabled = false;
		}
	}

private:
	bool enabled = false;
	void* targetFunc = nullptr;
#endif
};