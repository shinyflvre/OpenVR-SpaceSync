// SPDX-License-Identifier: AGPL-3.0-only
// Modified by Shinyflvres, 2026-08-23. Part of SpaceSync, a modified version of OpenVR-SpaceOverride by Nyabsi (AGPL-3.0). See NOTICE.md

#include "Configuration.h"
#include "Localization.h"

#include <Windows.h>

#include <picojson.h>

#include <string>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <limits>

static picojson::array FloatArray(const float *buf, int numFloats)
{
	picojson::array arr;

	for (int i = 0; i < numFloats; i++)
		arr.push_back(picojson::value(double(buf[i])));

	return arr;
}

static void LoadFloatArray(const picojson::value &obj, float *buf, int numFloats)
{
	if (!obj.is<picojson::array>())
		throw std::runtime_error("expected array, got " + obj.to_str());

	auto &arr = obj.get<picojson::array>();
	if (arr.size() != numFloats)
		throw std::runtime_error("wrong buffer size");

	for (int i = 0; i < numFloats; i++)
		buf[i] = (float) arr[i].get<double>();
}

static void ParseProfile(CalibrationContext &ctx, std::istream &stream)
{
	picojson::value v;
	std::string err = picojson::parse(v, stream);
	if (!err.empty())
		throw std::runtime_error(err);

	auto arr = v.get<picojson::array>();
	if (arr.size() < 1)
		throw std::runtime_error("no profiles in file");

	auto obj = arr[0].get<picojson::object>();

	ctx.targetTrackingSystem = obj["target_tracking_system"].get<std::string>();

	if (obj["hmd_serial"].is<std::string>())
		ctx.hmdSerial = obj["hmd_serial"].get<std::string>();
	if (obj["tracker_serial"].is<std::string>())
		ctx.trackerSerial = obj["tracker_serial"].get<std::string>();
	ctx.calibratedRotation(0) = obj["roll"].get<double>();
	ctx.calibratedRotation(1) = obj["yaw"].get<double>();
	ctx.calibratedRotation(2) = obj["pitch"].get<double>();
	ctx.calibratedTranslation(0) = obj["x"].get<double>();
	ctx.calibratedTranslation(1) = obj["y"].get<double>();
	ctx.calibratedTranslation(2) = obj["z"].get<double>();

	if (obj["scale"].is<double>())
		ctx.calibratedScale = obj["scale"].get<double>();
	else
		ctx.calibratedScale = 1.0;

	if (obj["targetModelScale"].is<double>())
		ctx.targetModelScale = obj["targetModelScale"].get<double>();
	else
		ctx.targetModelScale = ctx.calibratedScale;

	if (obj["hmdScale"].is<double>())
		ctx.hmdScale = obj["hmdScale"].get<double>();
	else
		ctx.hmdScale = 1.0;

	if (ctx.targetModelScale <= 0.0)
		ctx.targetModelScale = 1.0;
	if (ctx.hmdScale <= 0.0)
		ctx.hmdScale = 1.0;

	ctx.calibrationLatencyValid = obj["calLatencyRot"].is<double>() && obj["calLatencyPos"].is<double>();
	ctx.calibrationLatencyRot = ctx.calibrationLatencyValid ? obj["calLatencyRot"].get<double>() : 0.0;
	ctx.calibrationLatencyPos = ctx.calibrationLatencyValid ? obj["calLatencyPos"].get<double>() : 0.0;

	ctx.calibrationSpotValid = obj["calSpotX"].is<double>() && obj["calSpotY"].is<double>() && obj["calSpotZ"].is<double>();
	ctx.calibrationSpot = ctx.calibrationSpotValid
		? Eigen::Vector3d(obj["calSpotX"].get<double>(), obj["calSpotY"].get<double>(), obj["calSpotZ"].get<double>())
		: Eigen::Vector3d::Zero();

	ctx.fallbackToSlam = obj["fallbackSlam"].get<bool>();
	ctx.enableAngularVelocity = obj["eAngVel"].get<bool>();

	if (obj["continuousSync"].is<bool>())
		ctx.continuousSync = obj["continuousSync"].get<bool>();
	else
		ctx.continuousSync = true;

	if (obj["followSlam"].is<bool>())
		ctx.followSlamHmd = obj["followSlam"].get<bool>();
	else
		ctx.followSlamHmd = false;

	if (obj["noHeadTracker"].is<bool>())
		ctx.noHeadTracker = obj["noHeadTracker"].get<bool>();
	else
		ctx.noHeadTracker = false;

	if (obj["calibratedNoTracker"].is<bool>())
		ctx.calibratedNoTracker = obj["calibratedNoTracker"].get<bool>();
	else
		ctx.calibratedNoTracker = ctx.noHeadTracker;

	if (obj["mountRefined"].is<bool>())
		ctx.mountRefined = obj["mountRefined"].get<bool>();
	else
		ctx.mountRefined = false;

	if (obj["dynPower"].is<bool>())
		ctx.dynamicBasestationPower = obj["dynPower"].get<bool>();
	else
		ctx.dynamicBasestationPower = false;

	ctx.dynamicPowerMode = obj["dynPowerMode"].is<double>() && obj["dynPowerMode"].get<double>() == 1.0 ? 1 : 0;

	if (obj["bsManagement"].is<bool>())
		ctx.basestationManagement = obj["bsManagement"].get<bool>();
	else
		ctx.basestationManagement = true;

	if (obj["lhSmoothing"].is<double>())
	{
		ctx.lighthouseSmoothing = obj["lhSmoothing"].get<double>();
		if (ctx.lighthouseSmoothing < 0.0) ctx.lighthouseSmoothing = 0.0;
		if (ctx.lighthouseSmoothing > 100.0) ctx.lighthouseSmoothing = 100.0;
	}
	else
		ctx.lighthouseSmoothing = 0.0;

	if (obj["latencyComp"].is<double>())
	{
		ctx.latencyCompensation = obj["latencyComp"].get<double>();
		if (ctx.latencyCompensation < 0.0) ctx.latencyCompensation = 0.0;
		if (ctx.latencyCompensation > 100.0) ctx.latencyCompensation = 100.0;
	}
	else
		ctx.latencyCompensation = 40.0;

	if (obj["hideHeadTracker"].is<bool>())
		ctx.hideHeadTracker = obj["hideHeadTracker"].get<bool>();
	else
		ctx.hideHeadTracker = false;

	if (obj["stayAligned"].is<bool>())
		ctx.stayAligned = obj["stayAligned"].get<bool>();
	else
		ctx.stayAligned = false;

	if (obj["lockBaseStations"].is<bool>())
		ctx.lockBaseStations = obj["lockBaseStations"].get<bool>();
	else
		ctx.lockBaseStations = false;

	if (obj["disableVoice"].is<bool>())
		ctx.disableVoiceHelp = obj["disableVoice"].get<bool>();
	else
		ctx.disableVoiceHelp = false;

	if (obj["language"].is<double>())
		ctx.language = (int)obj["language"].get<double>();
	else
		ctx.language = 0;
	if (ctx.language < 0 || ctx.language > 3)
		ctx.language = 0;
	loc::SetLanguage((loc::Lang)ctx.language);

	if (obj["uiScale"].is<double>())
		ctx.uiScale = (float)obj["uiScale"].get<double>();
	else
		ctx.uiScale = 1.25f;
	if (ctx.uiScale < 0.8f) ctx.uiScale = 0.8f;
	if (ctx.uiScale > 2.0f) ctx.uiScale = 2.0f;

	if (obj["predictionTime"].is<double>())
		ctx.predictionTime = obj["predictionTime"].get<double>();
	else
		ctx.predictionTime = 1.0;

	auto loadOneEuro = [&](const char *key, protocol::OneEuroParams &out, protocol::OneEuroParams def) {
		out = def;
		if (!obj[key].is<picojson::object>())
			return;
		auto o = obj[key].get<picojson::object>();
		if (o["minCutoff"].is<double>()) out.minCutoff = o["minCutoff"].get<double>();
		if (o["beta"].is<double>())      out.beta = o["beta"].get<double>();
		if (o["dCutoff"].is<double>())   out.dCutoff = o["dCutoff"].get<double>();
	};

	ctx.headFilterEnabled = obj["headFilterEnabled"].is<bool>() ? obj["headFilterEnabled"].get<bool>() : true;
	loadOneEuro("headFilter", ctx.headFilterParams, { 2.0, 0.5, 1.0 });
	loadOneEuro("driftFilter", ctx.driftFilterParams, { 1.0, 0.4, 0.85 });

	if (obj["rel_qw"].is<double>())
	{
		ctx.relativeRotation.w = obj["rel_qw"].get<double>();
		ctx.relativeRotation.x = obj["rel_qx"].get<double>();
		ctx.relativeRotation.y = obj["rel_qy"].get<double>();
		ctx.relativeRotation.z = obj["rel_qz"].get<double>();
		ctx.relativeTranslation.v[0] = obj["rel_tx"].get<double>();
		ctx.relativeTranslation.v[1] = obj["rel_ty"].get<double>();
		ctx.relativeTranslation.v[2] = obj["rel_tz"].get<double>();
		ctx.validRelativeOffset = true;
	}
	else
	{
		ctx.validRelativeOffset = false;
	}

	if (obj["calibration_speed"].is<double>())
		ctx.calibrationSpeed = (CalibrationContext::Speed)(int) obj["calibration_speed"].get<double>();

	ctx.tiltHistory.clear();
	if (obj["tiltHistory"].is<picojson::array>())
	{
		for (auto &entry : obj["tiltHistory"].get<picojson::array>())
		{
			if (!entry.is<picojson::array>())
				continue;
			auto &v = entry.get<picojson::array>();
			if (v.size() != 3 || !v[0].is<double>() || !v[1].is<double>() || !v[2].is<double>())
				continue;
			Eigen::Vector3d g(v[0].get<double>(), v[1].get<double>(), v[2].get<double>());
			if (g.norm() > 0.5)
				ctx.tiltHistory.push_back(g.normalized());
		}
	}
	ctx.tiltFingerprint = obj["tiltFingerprint"].is<std::string>() ? obj["tiltFingerprint"].get<std::string>() : std::string();
	ctx.tiltSessionEntry = -1;
	ctx.tiltStepsRecorded = 0;

	if (obj["chaperone"].is<picojson::object>())
	{
		auto chaperone = obj["chaperone"].get<picojson::object>();
		ctx.chaperone.autoApply = chaperone["auto_apply"].get<bool>();

		LoadFloatArray(chaperone["play_space_size"], ctx.chaperone.playSpaceSize.v, 2);

		LoadFloatArray(
			chaperone["standing_center"],
			(float *) ctx.chaperone.standingCenter.m,
			sizeof(ctx.chaperone.standingCenter.m) / sizeof(float)
		);

		if (!chaperone["geometry"].is<picojson::array>())
			throw std::runtime_error("chaperone geometry is not an array");

		auto &geometry = chaperone["geometry"].get<picojson::array>();

		if (geometry.size() > 0)
		{
			ctx.chaperone.geometry.resize(geometry.size() * sizeof(float) / sizeof(ctx.chaperone.geometry[0]));
			LoadFloatArray(chaperone["geometry"], (float *) ctx.chaperone.geometry.data(), geometry.size());

			ctx.chaperone.valid = true;
		}
	}

	ctx.validProfile = true;
}

static void WriteProfile(CalibrationContext &ctx, std::ostream &out)
{
	if (!ctx.validProfile)
		return;

	picojson::object profile;
	profile["target_tracking_system"].set<std::string>(ctx.targetTrackingSystem);
	profile["hmd_serial"].set<std::string>(ctx.hmdSerial);
	profile["tracker_serial"].set<std::string>(ctx.trackerSerial);
	profile["roll"].set<double>(ctx.calibratedRotation(0));
	profile["yaw"].set<double>(ctx.calibratedRotation(1));
	profile["pitch"].set<double>(ctx.calibratedRotation(2));
	profile["x"].set<double>(ctx.calibratedTranslation(0));
	profile["y"].set<double>(ctx.calibratedTranslation(1));
	profile["z"].set<double>(ctx.calibratedTranslation(2));
	profile["scale"].set<double>(ctx.calibratedScale);
	profile["targetModelScale"].set<double>(ctx.targetModelScale);
	profile["hmdScale"].set<double>(ctx.hmdScale);
	if (ctx.calibrationLatencyValid)
	{
		profile["calLatencyRot"].set<double>(ctx.calibrationLatencyRot);
		profile["calLatencyPos"].set<double>(ctx.calibrationLatencyPos);
	}
	if (ctx.calibrationSpotValid)
	{
		profile["calSpotX"].set<double>(ctx.calibrationSpot.x());
		profile["calSpotY"].set<double>(ctx.calibrationSpot.y());
		profile["calSpotZ"].set<double>(ctx.calibrationSpot.z());
	}
	profile["calibratedNoTracker"].set<bool>(ctx.calibratedNoTracker);

	profile["fallbackSlam"].set<bool>(ctx.fallbackToSlam);
	profile["eAngVel"].set<bool>(ctx.enableAngularVelocity);
	profile["continuousSync"].set<bool>(ctx.continuousSync);
	profile["followSlam"].set<bool>(ctx.followSlamHmd);
	profile["noHeadTracker"].set<bool>(ctx.noHeadTracker);
	profile["mountRefined"].set<bool>(ctx.mountRefined);
	profile["dynPower"].set<bool>(ctx.dynamicBasestationPower);
	double dynPowerMode = (double)ctx.dynamicPowerMode;
	profile["dynPowerMode"].set<double>(dynPowerMode);
	profile["bsManagement"].set<bool>(ctx.basestationManagement);
	double lhSmoothing = ctx.lighthouseSmoothing;
	profile["lhSmoothing"].set<double>(lhSmoothing);
	double latencyComp = ctx.latencyCompensation;
	profile["latencyComp"].set<double>(latencyComp);
	profile["hideHeadTracker"].set<bool>(ctx.hideHeadTracker);
	profile["stayAligned"].set<bool>(ctx.stayAligned);
	profile["lockBaseStations"].set<bool>(ctx.lockBaseStations);
	profile["disableVoice"].set<bool>(ctx.disableVoiceHelp);
	double language = (double)ctx.language;
	profile["language"].set<double>(language);
	double uiScale = ctx.uiScale;
	profile["uiScale"].set<double>(uiScale);

	double time = ctx.predictionTime;
	profile["predictionTime"].set<double>(time);

	profile["headFilterEnabled"].set<bool>(ctx.headFilterEnabled);

	auto saveOneEuro = [](const protocol::OneEuroParams &p) {
		picojson::object o;
		o["minCutoff"].set<double>(p.minCutoff);
		o["beta"].set<double>(p.beta);
		o["dCutoff"].set<double>(p.dCutoff);
		return o;
	};
	profile["headFilter"].set<picojson::object>(saveOneEuro(ctx.headFilterParams));
	profile["driftFilter"].set<picojson::object>(saveOneEuro(ctx.driftFilterParams));

	if (ctx.validRelativeOffset)
	{
		profile["rel_qw"].set<double>(ctx.relativeRotation.w);
		profile["rel_qx"].set<double>(ctx.relativeRotation.x);
		profile["rel_qy"].set<double>(ctx.relativeRotation.y);
		profile["rel_qz"].set<double>(ctx.relativeRotation.z);
		profile["rel_tx"].set<double>(ctx.relativeTranslation.v[0]);
		profile["rel_ty"].set<double>(ctx.relativeTranslation.v[1]);
		profile["rel_tz"].set<double>(ctx.relativeTranslation.v[2]);
	}

	double speed = (int) ctx.calibrationSpeed;
	profile["calibration_speed"].set<double>(speed);

	picojson::array tiltHistory;
	for (auto &g : ctx.tiltHistory)
	{
		picojson::array v;
		v.push_back(picojson::value(g.x()));
		v.push_back(picojson::value(g.y()));
		v.push_back(picojson::value(g.z()));
		tiltHistory.push_back(picojson::value(v));
	}
	profile["tiltHistory"].set<picojson::array>(tiltHistory);
	profile["tiltFingerprint"].set<std::string>(ctx.tiltFingerprint);

	if (ctx.chaperone.valid)
	{
		picojson::object chaperone;
		chaperone["auto_apply"].set<bool>(ctx.chaperone.autoApply);
		chaperone["play_space_size"].set<picojson::array>(FloatArray(ctx.chaperone.playSpaceSize.v, 2));

		chaperone["standing_center"].set<picojson::array>(FloatArray(
			(float *) ctx.chaperone.standingCenter.m,
			sizeof(ctx.chaperone.standingCenter.m) / sizeof(float)
		));

		chaperone["geometry"].set<picojson::array>(FloatArray(
			(float *) ctx.chaperone.geometry.data(),
			sizeof(ctx.chaperone.geometry[0]) / sizeof(float) * ctx.chaperone.geometry.size()
		));

		profile["chaperone"].set<picojson::object>(chaperone);
	}

	picojson::value profileV;
	profileV.set<picojson::object>(profile);

	picojson::array profiles;
	profiles.push_back(profileV);

	picojson::value profilesV;
	profilesV.set<picojson::array>(profiles);

	out << profilesV.serialize(true);
}

static void LogRegistryResult(LSTATUS result)
{
	char *message;
	FormatMessageA(FORMAT_MESSAGE_FROM_SYSTEM | FORMAT_MESSAGE_ALLOCATE_BUFFER, 0, result, LANG_USER_DEFAULT, (LPSTR)&message, 0, NULL);
	std::cerr << "Opening registry key: " << message << std::endl;
}

static const char *RegistryKey = "Software\\SpaceSync";
// Old OpenVR-SpaceOverride profiles are read as fallback so nobody has to recalibrate.
static const char *LegacyRegistryKey = "Software\\OpenVR-SpaceOverride";

static std::string ReadRegistryValue(const char *key)
{
	DWORD size = 0;
	auto result = RegGetValueA(HKEY_CURRENT_USER_LOCAL_SETTINGS, key, "Config", RRF_RT_REG_SZ, 0, 0, &size);
	if (result != ERROR_SUCCESS)
	{
		LogRegistryResult(result);
		return "";
	}

	std::string str;
	str.resize(size);

	result = RegGetValueA(HKEY_CURRENT_USER_LOCAL_SETTINGS, key, "Config", RRF_RT_REG_SZ, 0, &str[0], &size);
	if (result != ERROR_SUCCESS)
	{
		LogRegistryResult(result);
		return "";
	}

	str.resize(size - 1);
	return str;
}

static std::string ReadRegistryKey()
{
	std::string str = ReadRegistryValue(RegistryKey);
	if (str.empty())
		str = ReadRegistryValue(LegacyRegistryKey);
	return str;
}

static void WriteRegistryKey(std::string str)
{
	HKEY hkey;
	auto result = RegCreateKeyExA(HKEY_CURRENT_USER_LOCAL_SETTINGS, RegistryKey, 0, REG_NONE, 0, KEY_ALL_ACCESS, 0, &hkey, 0);
	if (result != ERROR_SUCCESS)
	{
		LogRegistryResult(result);
		return;
	}

	DWORD size = str.size() + 1;

	result = RegSetValueExA(hkey, "Config", 0, REG_SZ, reinterpret_cast<const BYTE*>(str.c_str()), size);
	if (result != ERROR_SUCCESS)
		LogRegistryResult(result);

	RegCloseKey(hkey);
}

void LoadProfile(CalibrationContext &ctx)
{
	ctx.validProfile = false;

	auto str = ReadRegistryKey();
	if (str == "")
	{
		std::cout << "Profile is empty" << std::endl;
		ctx.Clear();
		return;
	}

	try
	{
		std::stringstream io(str);
		ParseProfile(ctx, io);
		std::cout << "Loaded profile" << std::endl;
	}
	catch (const std::runtime_error &e)
	{
		std::cerr << "Error loading profile: " << e.what() << std::endl;
	}
}

void SaveProfile(CalibrationContext &ctx)
{
	std::cout << "Saving profile to registry" << std::endl;

	std::stringstream io;
	WriteProfile(ctx, io);
	WriteRegistryKey(io.str());
}
