// SPDX-License-Identifier: AGPL-3.0-only
// Added by Shinyflvres, 2026-09-29. Part of SpaceSync, a modified version of OpenVR-SpaceOverride by Nyabsi (AGPL-3.0). See NOTICE.md

#pragma once

#include "PoseMath.h"

#include <openvr_driver.h>
#include "Platform.h"

#include <cstdio>
#include <deque>
#include <functional>
#include <mutex>
#include <string>
#include <vector>

namespace universe
{
	struct Sample
	{
		double t = 0.0;
		vr::HmdVector3d_t world = { 0, 0, 0 };
		vr::HmdQuaternion_t worldRot = { 1, 0, 0, 0 };
		vr::HmdVector3d_t frameT = { 0, 0, 0 };
		vr::HmdQuaternion_t frameR = { 1, 0, 0, 0 };
		vr::HmdVector3d_t local = { 0, 0, 0 };
		int result = 0;
		bool valid = false;
	};

	struct Summary
	{
		uint32_t baseUpdates = 0;
		uint32_t baseChanges = 0;
		uint32_t frameChanges = 0;
		uint32_t frameJumps = 0;
		uint32_t referenceSwitches = 0;
		uint32_t inconsistentSwitches = 0;
		uint32_t unmatched = 0;
		uint32_t events = 0;
	};

	class Probe
	{
	public:
		std::function<void(uint32_t, int32_t&, std::string&, std::string&, std::string&)> properties;
		std::string path = "spacesync_universe.log";

		~Probe()
		{
			if (file)
				fclose(file);
		}

		bool takeSummary(Summary& out, double now)
		{
			std::lock_guard<std::mutex> lock(mutex);
			if (now - lastSummary < 60.0)
				return false;
			lastSummary = now;
			out = minute;
			minute = Summary{};
			writeSummary(now);
			return true;
		}

		void onPose(uint32_t id, const vr::DriverPose_t& pose, double now, uint32_t hmdID)
		{
			if (id >= vr::k_unMaxTrackedDeviceCount)
				return;
			std::lock_guard<std::mutex> lock(mutex);
			if (!ensureOpen())
				return;
			Dev& d = devs[id];
			if (d.kind == Unknown || (d.kind != Ignored && d.serial.empty() && now - d.classifiedAt > 2.0))
				classify(id, d, now, hmdID);
			if (d.kind == Ignored || d.kind == Unknown)
				return;

			Sample s;
			s.t = now;
			s.valid = pose.poseIsValid;
			s.result = (int)pose.result;
			s.frameR = quaternionNormalize(pose.qWorldFromDriverRotation);
			s.frameT = vecFromArray(pose.vecWorldFromDriverTranslation);
			vr::HmdVector3d_t headLocal = quaternionRotateVector(pose.qRotation, pose.vecDriverFromHeadTranslation);
			s.local = vecAdd(vecFromArray(pose.vecPosition), headLocal);
			s.world = vecAdd(quaternionRotateVector(s.frameR, s.local), s.frameT);
			s.worldRot = quaternionNormalize(s.frameR * pose.qRotation * pose.qDriverFromHeadRotation);
			d.updates++;

			if (d.kind == Base)
				onBase(id, d, s, now);
			else if (d.kind == Device)
				onDevice(id, d, s, pose, now);

			if (d.ring.empty() || now - d.ring.back().t >= 0.004)
			{
				d.ring.push_back(s);
				while (!d.ring.empty() && now - d.ring.front().t > 2.0)
					d.ring.pop_front();
				if (now < captureUntil)
					writeSample(id, s);
			}
		}

	private:
		enum Kind { Unknown, Ignored, Base, Device, Hmd };

		struct Dev
		{
			Kind kind = Unknown;
			std::string serial;
			double classifiedAt = -1e9;
			bool primed = false;
			Sample last;
			std::deque<Sample> ring;
			std::vector<Sample> history;
			uint32_t updates = 0, frameChanges = 0, jumps = 0, switches = 0, badSwitches = 0, unmatched = 0;
			int lastRef = -1;
		};

		std::mutex mutex;
		FILE* file = nullptr;
		bool openTried = false;
		long long written = 0;
		const long long maxBytes = 200ll * 1024 * 1024;
		Dev devs[vr::k_unMaxTrackedDeviceCount];
		double captureUntil = -1.0;
		double lastSummary = 0.0;
		Summary minute;

		bool ensureOpen()
		{
			if (file)
				return true;
			if (openTried)
				return false;
			openTried = true;
			FILE* existing = fopen(path.c_str(), "rb");
			if (existing)
			{
				long long size = 0;
				if (_fseeki64(existing, 0, SEEK_END) == 0)
					size = _ftelli64(existing);
				fclose(existing);
				if (size > 50ll * 1024 * 1024)
				{
					std::string old = path.substr(0, path.rfind('.')) + ".old.log";
					remove(old.c_str());
					rename(path.c_str(), old.c_str());
				}
			}
			file = fopen(path.c_str(), "a");
			if (!file)
				return false;
			line("SESSION universe probe started (raw SteamVR poses, before any SpaceSync transform)");
			return true;
		}

		void stamp(char* buf, size_t n)
		{
			SYSTEMTIME st;
			GetLocalTime(&st);
			std::snprintf(buf, n, "%02d:%02d:%02d.%03d", st.wHour, st.wMinute, st.wSecond, st.wMilliseconds);
		}

		template <typename... Args>
		void line(const char* fmt, Args... args)
		{
			if (!file || written > maxBytes)
				return;
			char ts[32];
			stamp(ts, sizeof ts);
			int a = std::fprintf(file, "%s ", ts);
			int b = std::fprintf(file, fmt, args...);
			int c = std::fprintf(file, "\n");
			written += (a > 0 ? a : 0) + (b > 0 ? b : 0) + (c > 0 ? c : 0);
			fflush(file);
		}

		void classify(uint32_t id, Dev& d, double now, uint32_t hmdID)
		{
			d.classifiedAt = now;
			int32_t cls = 0;
			std::string system, serial, type;
			if (properties)
				properties(id, cls, system, serial, type);
			else
			{
				vr::PropertyContainerHandle_t c = vr::VRProperties()->TrackedDeviceToPropertyContainer(id);
				vr::ETrackedPropertyError err = vr::TrackedProp_Success;
				cls = vr::VRProperties()->GetInt32Property(c, vr::Prop_DeviceClass_Int32, &err);
				system = vr::VRProperties()->GetStringProperty(c, vr::Prop_TrackingSystemName_String, &err);
				serial = vr::VRProperties()->GetStringProperty(c, vr::Prop_SerialNumber_String, &err);
				type = vr::VRProperties()->GetStringProperty(c, vr::Prop_ControllerType_String, &err);
			}
			Kind kind = Ignored;
			if (id == hmdID)
				kind = Hmd;
			else if (system == "lighthouse")
				kind = cls == vr::TrackedDeviceClass_TrackingReference ? Base : Device;
			bool changed = kind != d.kind || serial != d.serial;
			d.kind = kind;
			d.serial = serial;
			if (changed)
				line("DEV id %u class %d system %s serial %s type %s -> %s", id, cls, system.c_str(), serial.c_str(), type.c_str(),
					kind == Base ? "base station" : (kind == Device ? "lighthouse device" : (kind == Hmd ? "headset" : "ignored")));
		}

		static double angleDeg(const vr::HmdQuaternion_t& a, const vr::HmdQuaternion_t& b)
		{
			return quaternionAngleRad(quaternionNormalize(a * quaternionConjugate(b))) * 180.0 / POSE_PI;
		}

		void onBase(uint32_t id, Dev& d, const Sample& s, double now)
		{
			minute.baseUpdates++;
			if (!d.primed)
			{
				d.primed = true;
				d.last = s;
				d.history.push_back(s);
				line("BASE id %u serial %s first pose pos %.4f %.4f %.4f rot %.6f %.6f %.6f %.6f", id, d.serial.c_str(),
					s.world.v[0], s.world.v[1], s.world.v[2], s.worldRot.w, s.worldRot.x, s.worldRot.y, s.worldRot.z);
				return;
			}
			double dp = vecNorm(vecSub(s.world, d.last.world));
			double da = angleDeg(s.worldRot, d.last.worldRot);
			if (dp < 0.0005 && da < 0.02)
			{
				d.last = s;
				return;
			}
			minute.baseChanges++;
			line("BASE id %u serial %s moved %.1f mm %.3f deg pos %.4f %.4f %.4f rot %.6f %.6f %.6f %.6f", id, d.serial.c_str(), dp * 1000.0, da,
				s.world.v[0], s.world.v[1], s.world.v[2], s.worldRot.w, s.worldRot.x, s.worldRot.y, s.worldRot.z);
			d.last = s;
			d.history.push_back(s);
			if (d.history.size() > 16)
				d.history.erase(d.history.begin());
			startCapture(now, "base", id);
		}

		void matchReference(const Sample& s, int& bestId, int& bestAge, double& bestMm, double& bestDeg)
		{
			bestId = -1;
			bestAge = -1;
			bestMm = 1e9;
			bestDeg = 1e9;
			for (uint32_t b = 0; b < vr::k_unMaxTrackedDeviceCount; b++)
			{
				const Dev& bd = devs[b];
				if (bd.kind != Base)
					continue;
				for (int h = (int)bd.history.size() - 1; h >= 0; h--)
				{
					const Sample& bs = bd.history[h];
					double mm = vecNorm(vecSub(bs.world, s.frameT)) * 1000.0;
					double deg = angleDeg(bs.worldRot, s.frameR);
					if (mm + deg * 10.0 < bestMm + bestDeg * 10.0)
					{
						bestId = (int)b;
						bestAge = (int)bd.history.size() - 1 - h;
						bestMm = mm;
						bestDeg = deg;
					}
				}
			}
		}

		void onDevice(uint32_t id, Dev& d, const Sample& s, const vr::DriverPose_t& pose, double now)
		{
			if (!d.primed)
			{
				d.primed = true;
				d.last = s;
				int ref, age;
				double mm, deg;
				matchReference(s, ref, age, mm, deg);
				d.lastRef = ref;
				line("FRAME id %u serial %s first, matches base %d (history %d) by %.1f mm / %.3f deg", id, d.serial.c_str(), ref, age, mm, deg);
				return;
			}
			const Sample& p = d.last;
			vr::HmdVector3d_t movedByFrame = vecSub(vecAdd(quaternionRotateVector(s.frameR, p.local), s.frameT), p.world);
			double frameMm = vecNorm(movedByFrame) * 1000.0;
			double frameDeg = angleDeg(s.frameR, p.frameR);
			bool frameChanged = frameMm > 0.5 || frameDeg > 0.02;
			if (frameChanged && s.valid && p.valid)
			{
				double dt = s.t - p.t;
				if (dt < 0.0 || dt > 0.1)
					dt = 0.0;
				vr::HmdVector3d_t vLocal = vecFromArray(pose.vecVelocity);
				vr::HmdVector3d_t vWorld = quaternionRotateVector(s.frameR, vLocal);
				double worldJump = vecNorm(vecSub(s.world, vecAdd(p.world, vecScale(vWorld, dt)))) * 1000.0;
				double localJump = vecNorm(vecSub(s.local, vecAdd(p.local, vecScale(vLocal, dt)))) * 1000.0;
				int ref, age;
				double mm, deg;
				matchReference(s, ref, age, mm, deg);
				bool matched = mm <= 5.0 && deg <= 0.2;
				bool worldJumped = worldJump > 5.0;
				bool sameRef = ref == d.lastRef;
				const char* verdict;
				d.frameChanges++;
				minute.frameChanges++;
				if (!matched)
				{
					d.unmatched++;
					minute.unmatched++;
					verdict = worldJumped ? "UNEXPLAINED JUMP (no base matches)" : "unmatched frame, world continuous";
				}
				else if (sameRef)
				{
					if (worldJumped) { d.jumps++; minute.frameJumps++; }
					verdict = worldJumped ? "UNIVERSE JUMP (reference base moved)" : "reference base moved, world continuous";
				}
				else
				{
					d.switches++;
					minute.referenceSwitches++;
					if (worldJumped) { d.badSwitches++; minute.inconsistentSwitches++; }
					verdict = worldJumped ? "REFERENCE SWITCH BETWEEN INCONSISTENT BASES" : "reference switch, world continuous";
				}
				line("FRAME id %u serial %s changed: moves device %.1f mm / %.3f deg, world jump %.1f mm, local jump %.1f mm, reference base %d -> %d (history %d, match %.1f mm / %.3f deg) -> %s",
					id, d.serial.c_str(), frameMm, frameDeg, worldJump, localJump, d.lastRef, ref, age, mm, deg, verdict);
				d.lastRef = ref;
				if (worldJumped)
					startCapture(now, "device", id);
			}
			d.last = s;
		}

		void startCapture(double now, const char* why, uint32_t id)
		{
			minute.events++;
			if (now >= captureUntil)
			{
				line("CAPTURE start (%s %u), dumping last 2 s of all devices", why, id);
				for (uint32_t k = 0; k < vr::k_unMaxTrackedDeviceCount; k++)
					for (const Sample& s : devs[k].ring)
						writeSample(k, s);
			}
			captureUntil = now + 2.0;
		}

		void writeSample(uint32_t id, const Sample& s)
		{
			line("S %u %.6f %d %d w %.5f %.5f %.5f q %.6f %.6f %.6f %.6f f %.5f %.5f %.5f fq %.6f %.6f %.6f %.6f l %.5f %.5f %.5f", id, s.t, (int)s.valid, s.result,
				s.world.v[0], s.world.v[1], s.world.v[2], s.worldRot.w, s.worldRot.x, s.worldRot.y, s.worldRot.z,
				s.frameT.v[0], s.frameT.v[1], s.frameT.v[2], s.frameR.w, s.frameR.x, s.frameR.y, s.frameR.z,
				s.local.v[0], s.local.v[1], s.local.v[2]);
		}

		void writeSummary(double now)
		{
			(void)now;
			for (uint32_t k = 0; k < vr::k_unMaxTrackedDeviceCount; k++)
			{
				Dev& d = devs[k];
				if (d.kind != Base && d.kind != Device && d.kind != Hmd)
					continue;
				line("SUM id %u serial %s kind %s updates %u frame changes %u universe jumps %u reference switches %u (inconsistent %u) unmatched %u reference %d",
					k, d.serial.c_str(), d.kind == Base ? "base" : (d.kind == Hmd ? "headset" : "device"), d.updates, d.frameChanges, d.jumps, d.switches, d.badSwitches, d.unmatched, d.lastRef);
				d.updates = d.frameChanges = d.jumps = d.switches = d.badSwitches = d.unmatched = 0;
			}
		}
	};
}
