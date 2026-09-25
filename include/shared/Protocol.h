// SPDX-License-Identifier: AGPL-3.0-only
// Modified by Shinyflvres, 2026-08-23. Part of SpaceSync, a modified version of OpenVR-SpaceOverride by Nyabsi (AGPL-3.0). See NOTICE.md

#pragma once

#include <cstdint>

#ifndef _OPENVR_API
#include <openvr_driver.h>
#endif

#define SPACESYNC_PIPE_NAME "\\\\.\\pipe\\SpaceSyncCom"

namespace protocol
{
	const uint32_t Version = 14;

	enum RequestType
	{
		RequestInvalid,
		RequestHandshake,
		RequestSetDeviceTransform,
		RequestSetHmdTracker,
		RequestSetSlamSync,
		RequestSetOneEuro,
		RequestGetStatus,
	};

	enum ResponseType
	{
		ResponseInvalid,
		ResponseHandshake,
		ResponseSuccess,
		ResponseStatus,
	};

	struct Protocol
	{
		uint32_t version = Version;
	};

	struct SetDeviceTransform
	{
		uint32_t openVRID;
		bool enabled;
		bool updateTranslation;
		bool updateRotation;
		bool updateScale;
		vr::HmdVector3d_t translation;
		vr::HmdQuaternion_t rotation;
		double scale;

		SetDeviceTransform(uint32_t id, bool enabled) :
			openVRID(id), enabled(enabled), updateTranslation(false), updateRotation(false), updateScale(false) { }

		SetDeviceTransform(uint32_t id, bool enabled, vr::HmdVector3d_t translation) :
			openVRID(id), enabled(enabled), updateTranslation(true), updateRotation(false), updateScale(false), translation(translation) { }

		SetDeviceTransform(uint32_t id, bool enabled, vr::HmdQuaternion_t rotation) :
			openVRID(id), enabled(enabled), updateTranslation(false), updateRotation(true), updateScale(false), rotation(rotation) { }

		SetDeviceTransform(uint32_t id, bool enabled, double scale) :
			openVRID(id), enabled(enabled), updateTranslation(false), updateRotation(false), updateScale(true), scale(scale) { }

		SetDeviceTransform(uint32_t id, bool enabled, vr::HmdVector3d_t translation, vr::HmdQuaternion_t rotation) :
			openVRID(id), enabled(enabled), updateTranslation(true), updateRotation(true), updateScale(false), translation(translation), rotation(rotation) { }

		SetDeviceTransform(uint32_t id, bool enabled, vr::HmdVector3d_t translation, vr::HmdQuaternion_t rotation, double scale) :
			openVRID(id), enabled(enabled), updateTranslation(true), updateRotation(true), updateScale(true), translation(translation), rotation(rotation), scale(scale) { }
	};

	struct SetHmdTracker
	{
		uint32_t hmdID;
		uint32_t trackerID;
		bool enabled;
		bool slamFallback;
		bool enableAngularVelocity;
		float predictionTime;
		vr::HmdQuaternion_t offsetRotation;
		vr::HmdVector3d_t offsetTranslation;
		vr::HmdQuaternion_t calibrationRotation;
		vr::HmdVector3d_t calibrationTranslation;
		double calibrationScale;
		// Headset tracking space scale relative to the head tracker's lighthouse space.
		double hmdScale;
		// Follow SLAM HMD: headset keeps its SLAM pose, lighthouse devices (incl. head tracker) follow it.
		bool followSlamHmd;
		// Follow mode only.
		bool hideHeadTracker;
	};

	struct SetSlamSync
	{
		uint32_t openVRID;
		bool enabled;
	};

	struct OneEuroParams
	{
		double minCutoff;
		double beta;
		double dCutoff;
	};

	struct SetOneEuro
	{
		bool headEnabled;
		OneEuroParams head;
		OneEuroParams drift;
		double deviceSmoothing;
	};

	struct DriverStatus
	{
		bool driftValid;
		bool mountShiftSuspected;
		double tiltDeg;
		double translationDeviationM;
		double latencyMs;
		double latencyPosMs;
		uint32_t jumpsCompensated;
		double sigmaYawDeg;
		double sigmaTranslationM;
		double calmSeconds;
		uint32_t refinementSolves;
		uint32_t refinementTranslationSolves;
		bool refinementValid;
		vr::HmdQuaternion_t offsetRotation;
		vr::HmdVector3d_t offsetTranslation;
		double hmdScale;
	};

	struct Request
	{
		RequestType type;

		union {
			SetDeviceTransform setDeviceTransform;
			SetHmdTracker setHmdTracker;
			SetSlamSync setSlamSync;
			SetOneEuro setOneEuro;
		};

		Request() : type(RequestInvalid) { }
		Request(RequestType type) : type(type) { }
	};

	struct Response
	{
		ResponseType type;

		union {
			Protocol protocol;
			DriverStatus status;
		};

		Response() : type(ResponseInvalid) { }
		Response(ResponseType type) : type(type) { }
	};
}