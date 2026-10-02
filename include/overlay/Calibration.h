// SPDX-License-Identifier: AGPL-3.0-only
// Modified by Shinyflvres, 2026-08-23. Part of SpaceSync, a modified version of OpenVR-SpaceOverride by Nyabsi (AGPL-3.0). See NOTICE.md

#pragma once

#include <Core>
#include <openvr.h>
#include <vector>
#include <string>
#include <iostream>

#include "Protocol.h"

enum class CalibrationState
{
	None,
	Begin,
	Detect,
	Sampling,
	Editing,
};

struct CalibrationContext
{
	CalibrationState state = CalibrationState::None;
	uint32_t targetID;

	Eigen::Vector3d calibratedRotation;
	Eigen::Vector3d calibratedTranslation;
	double calibratedScale = 1.0;
	double targetModelScale = 1.0;
	double hmdScale = 1.0;
	bool calibrationLatencyValid = false;
	double calibrationLatencyRot = 0.0;
	double calibrationLatencyPos = 0.0;
	bool calibrationSpotValid = false;
	Eigen::Vector3d calibrationSpot = Eigen::Vector3d::Zero();
	bool calibratedNoTracker = false;

	vr::HmdQuaternion_t relativeRotation = { 1, 0, 0, 0 };
	vr::HmdVector3d_t relativeTranslation = { 0, 0, 0 };
	bool validRelativeOffset = false;

	std::string targetTrackingSystem;

	std::string hmdSerial;
	std::string trackerSerial;

	bool enabled = false;
	bool validProfile = false;
	bool lastCalibrationOk = false;   // did the last calibration run succeed
	protocol::DriverStatus driverStatus = {};
	bool refinementDirty = false;
	bool mountRefined = false;
	std::vector<Eigen::Vector3d> tiltHistory;
	std::string tiltFingerprint;
	int tiltSessionEntry = -1;
	uint32_t tiltStepsRecorded = 0;
	double timeRefinementSaved = 0.0;
	double timeLastTick = 0, timeLastScan = 0;
	double wantedUpdateInterval = 1.0;

	bool fallbackToSlam = true;
	bool enableAngularVelocity = false;
	bool continuousSync = false;
	bool followSlamHmd = true;
	bool noHeadTracker = false;
	bool hideHeadTracker = true;
	bool stayAligned = false;
	bool lockBaseStations = false;
	bool lockCommitPending = false;
	float predictionTime = 1.0f;
	float uiScale = 1.25f;            // UI content scale

	bool headFilterEnabled = false;
	protocol::OneEuroParams headFilterParams = { 5.0, 0.8, 1.0 };
	protocol::OneEuroParams driftFilterParams = { 3.0, 1.3, 0.6 };
	double lighthouseSmoothing = 0.0;
	double latencyCompensation = 40.0;
	bool basestationControl = true;
	bool dynamicBasestationPower = false;
	bool disableVoiceHelp = false;
	int language = 0;

	vr::VRNotificationId notificationId = 0;

	enum Speed
	{
		FAST = 0,
		SLOW = 1,
		VERY_SLOW = 2
	};
	Speed calibrationSpeed = FAST;

	vr::TrackedDevicePose_t devicePoses[vr::k_unMaxTrackedDeviceCount];

	struct Chaperone
	{
		bool valid = false;
		bool autoApply = true;
		std::vector<vr::HmdQuad_t> geometry;
		vr::HmdMatrix34_t standingCenter;
		vr::HmdVector2_t playSpaceSize;
	} chaperone;

	void Clear()
	{
		chaperone.geometry.clear();
		chaperone.standingCenter = vr::HmdMatrix34_t();
		chaperone.playSpaceSize = vr::HmdVector2_t();
		chaperone.valid = false;

		calibratedRotation = Eigen::Vector3d();
		calibratedTranslation = Eigen::Vector3d();
		calibratedScale = 1.0;
		targetModelScale = 1.0;
		hmdScale = 1.0;
		calibrationLatencyValid = false;
		calibrationLatencyRot = 0.0;
		calibrationLatencyPos = 0.0;
		calibrationSpotValid = false;
		calibrationSpot = Eigen::Vector3d::Zero();
		calibratedNoTracker = false;
		relativeRotation = { 1, 0, 0, 0 };
		relativeTranslation = { 0, 0, 0 };
		validRelativeOffset = false;
		mountRefined = false;
		tiltHistory.clear();
		tiltFingerprint.clear();
		tiltSessionEntry = -1;
		tiltStepsRecorded = 0;
		targetTrackingSystem = "";
		hmdSerial = "";
		trackerSerial = "";
		enabled = false;
		validProfile = false;
		continuousSync = false;
	}

	// The look-around sequence drives both the wizard and the voice cues, one second per step.
	static const int SequenceCycle = 8;
	static constexpr double StepSeconds = 1.5;

	double sequenceStart = 0.0;
	int sequenceStep = 0;
	int sequenceSteps = 0;

	int SequencePasses() const
	{
		switch (calibrationSpeed)
		{
		case FAST:
			return 1;
		case SLOW:
			return 2;
		case VERY_SLOW:
			return 3;
		}
		return 1;
	}

	int SequenceStepCount() const { return SequenceCycle * SequencePasses(); }
	double SequenceSeconds() const { return SequenceStepCount() * StepSeconds; }

	struct Message
	{
		enum Type
		{
			String,
			Progress
		} type = String;

		Message(Type type) : type(type) { }

		std::string str;
		int progress, target;
	};

	std::vector<Message> messages;

	void Log(const std::string &msg)
	{
		if (messages.empty() || messages.back().type == Message::Progress)
			messages.push_back(Message(Message::String));

		messages.back().str += msg;
		std::cerr << msg;
	}

	void Progress(int current, int target)
	{
		if (messages.empty() || messages.back().type == Message::String)
			messages.push_back(Message(Message::Progress));

		messages.back().progress = current;
		messages.back().target = target;
	}
};

extern CalibrationContext CalCtx;

void InitCalibrator();
bool DriverConnected();
void CalibrationTick(double time);
void StartCalibration();
void CancelCalibration();   // abort a running calibration, restore the saved profile
void LoadChaperoneBounds();
void ApplyChaperoneBounds();
void SendOneEuroParams();
void SendUniverseLock();
void KeepCalibrationSpotAfterRotationEdit(const Eigen::Vector3d &previousRotation);