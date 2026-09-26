// SPDX-License-Identifier: AGPL-3.0-only
// Modified by Shinyflvres, 2026-08-23. Part of SpaceSync, a modified version of OpenVR-SpaceOverride by Nyabsi (AGPL-3.0). See NOTICE.md

#pragma once

#include "IPCServer.h"
#include "OneEuroFilter.h"
#include "AlignmentEstimator.h"
#include "KalmanFilter.h"

#include <openvr_driver.h>

#include <atomic>
#include <cmath>
#include <mutex>

class ServerTrackedDeviceProvider : public vr::IServerTrackedDeviceProvider
{
public:
	////// Start vr::IServerTrackedDeviceProvider functions

	/** initializes the driver. This will be called before any other methods are called. */
	virtual vr::EVRInitError Init(vr::IVRDriverContext *pDriverContext) override;

	/** cleans up the driver right before it is unloaded */
	virtual void Cleanup() override;

	/** Returns the version of the ITrackedDeviceServerDriver interface used by this driver */
	virtual const char * const *GetInterfaceVersions() { return vr::k_InterfaceVersions; }

	/** Allows the driver do to some work in the main loop of the server. */
	virtual void RunFrame() { }

	/** Returns true if the driver wants to block Standby mode. */
	virtual bool ShouldBlockStandbyMode() { return false; }

	/** Called when the system is entering Standby mode. The driver should switch itself into whatever sort of low-power
	* state it has. */
	virtual void EnterStandby() { }

	/** Called when the system is leaving Standby mode. The driver should switch itself back to
	full operation. */
	virtual void LeaveStandby() { }

	////// End vr::IServerTrackedDeviceProvider functions

	ServerTrackedDeviceProvider() : server(this) { }
	void SetDeviceTransform(const protocol::SetDeviceTransform &newTransform);
	void SetHmdTracker(const protocol::SetHmdTracker &cmd);
	void SetSlamSync(const protocol::SetSlamSync &cmd);
	void SetOneEuro(const protocol::SetOneEuro &cmd);
	void GetStatus(protocol::DriverStatus &status);
	bool HandleDevicePoseUpdated(uint32_t openVRID, vr::DriverPose_t &pose);

private:
	// confidence 0..1 = how much this sample may move the drift estimate.
	void UpdateDrift(const vr::HmdQuaternion_t &correctedRotation, const double (&correctedPosition)[3],
		const vr::HmdQuaternion_t &rawRotation, const double (&rawPosition)[3], double confidence, double quality = 1.0);

	// Drift changes slowly, but fast head motion makes a single sample unreliable, so weight by speed.
	static double DriftSampleConfidence(double linSpeed, double angSpeed)
	{
		// Fast head motion: don't touch the drift at all, it gets re-measured once the head is calm.
		const double linFreeze = 1.0;  // m/s
		const double angFreeze = 1.0;  // rad/s
		if (linSpeed > linFreeze || angSpeed > angFreeze)
			return 0.0;

		const double linRef = 0.5;   // m/s   -> weight 0.5
		const double angRef = 0.4;   // rad/s -> weight 0.5
		double l = linSpeed / linRef, a = angSpeed / angRef;
		return 1.0 / (1.0 + l * l + a * a);
	}

	bool DetectHmdFrameJump(const vr::DriverPose_t &pose, double &jumpYaw, vr::HmdVector3d_t &jumpTranslation);

	align::ClockAligner clock;
	double clockLogTime = 0.0;
	double lastHmdTime = -1.0;
	double tauRotTrim = 0.0;
	double refineTauRot = 0.0;
	double refineTauPos = 0.0;

	struct FrameConvention
	{
		bool primed = false;
		double time = 0.0;
		vr::HmdQuaternion_t rotation = { 1, 0, 0, 0 };
		vr::HmdVector3d_t position = { 0, 0, 0 };
		double angWorld = 0.0, angDevice = 0.0, angNorm = 0.0;
		double velWorld = 0.0, velDevice = 0.0, velNorm = 0.0;
		bool angularDevice = false;
		bool velocityDevice = false;
		bool decidedAngular = false;
		bool decidedVelocity = false;

		static const int VelRing = 4;
		vr::HmdVector3d_t velHistory[VelRing] = {};
		double velHistoryTime[VelRing] = {};
		int velCount = 0;
		int velNext = 0;
		double accNum = 0.0, accDen = 0.0;
		double accGain = 0.0;
		bool accDecided = false;

		void reset()
		{
			primed = false;
			angWorld = angDevice = angNorm = 0.0;
			velWorld = velDevice = velNorm = 0.0;
			angularDevice = velocityDevice = false;
			decidedAngular = decidedVelocity = false;
			velCount = velNext = 0;
			accNum = accDen = 0.0;
			accGain = 0.0;
			accDecided = false;
		}
	} frames;

	struct HmdFrameWatch
	{
		bool primed = false;
		vr::HmdQuaternion_t rotation = { 1, 0, 0, 0 };
		vr::HmdVector3d_t translation = { 0, 0, 0 };
		uint32_t jumps = 0;

		void reset() { primed = false; }
	} hmdFrame;

	struct ResidualDiag
	{
		double yawNum = 0.0, yawDen = 0.0, posNum = 0.0, posDen = 0.0;
		int yawFrames = 0, posFrames = 0;
		double lastLog = 0.0;

		void reset() { yawNum = yawDen = posNum = posDen = 0.0; yawFrames = posFrames = 0; }
	} residualDiag;

	struct YawBins
	{
		static const int Count = 8;
		double sumWorld[Count][3] = {};
		double sumHead[Count][3] = {};
		double sumMeas[Count][3] = {};
		int frames[Count] = {};
		double lastLog = 0.0;

		void reset()
		{
			for (int i = 0; i < Count; i++)
			{
				frames[i] = 0;
				for (int k = 0; k < 3; k++) { sumWorld[i][k] = 0.0; sumHead[i][k] = 0.0; sumMeas[i][k] = 0.0; }
			}
		}
	} yawBins;

	align::MountRefiner refine;
	LARGE_INTEGER refineLast = {};
	bool refinePrimed = false;
	double refineLogTime = 0.0;
	double refineTauMovedSince = -1.0;
	vr::HmdQuaternion_t worldTilt = { 1, 0, 0, 0 };
	double trackerNotOKTime = -1e9;
	int measurementTrustState = -1;
	double trustLogTime = 0.0;

	struct EffectiveOffsets
	{
		vr::HmdQuaternion_t rotation = { 1, 0, 0, 0 };
		vr::HmdVector3d_t translation = { 0, 0, 0 };
		double hmdScale = 1.0;
	};
	EffectiveOffsets effective;
	std::mutex effectiveMutex;
	EffectiveOffsets effectiveShared;
	bool effectiveSharedValid = false;
	EffectiveOffsets published;
	bool publishedValid = false;
	void UpdateEffectiveOffsets();

	static bool OffsetsEqual(const vr::HmdQuaternion_t& qa, const vr::HmdVector3d_t& ta, double sa,
		const vr::HmdQuaternion_t& qb, const vr::HmdVector3d_t& tb, double sb)
	{
		const double eps = 1e-12;
		return std::fabs(qa.w - qb.w) < eps && std::fabs(qa.x - qb.x) < eps && std::fabs(qa.y - qb.y) < eps && std::fabs(qa.z - qb.z) < eps
			&& std::fabs(ta.v[0] - tb.v[0]) < eps && std::fabs(ta.v[1] - tb.v[1]) < eps && std::fabs(ta.v[2] - tb.v[2]) < eps
			&& std::fabs(sa - sb) < eps;
	}
	void ApplyDrift(vr::DriverPose_t &pose) const;
	void ApplyInverseDrift(vr::DriverPose_t &pose) const;
	void NoteTrackerState(bool trackerOK, bool usingSlam, const vr::TrackedDevicePose_t &tp,
		double trackerYawDeg, bool relativeYawValid, double relativeYawDeg);

	// Raw head tracker pose grabbed in the hook. Follow mode needs it because the tracker is
	// re-aligned in vrserver and can't be read back raw. Lighthouse thread writes, HMD thread reads.
	struct TrackerSample
	{
		bool valid = false;
		bool poseIsValid = false;
		bool deviceIsConnected = false;
		vr::ETrackingResult result = vr::TrackingResult_Uninitialized;
		vr::HmdQuaternion_t rotation = { 1, 0, 0, 0 };
		double position[3] = { 0, 0, 0 };
		double velocity[3] = { 0, 0, 0 };         // world space, m/s
		double angularVelocity[3] = { 0, 0, 0 };  // world space, rad/s (axis-angle rate)
		double acceleration[3] = { 0, 0, 0 };     // world space, m/s^2
		double linSpeed = 0.0;
		double angSpeed = 0.0;
		double poseTimeOffset = 0.0;              // seconds, as reported by the driver
		LARGE_INTEGER time = {};                  // when the sample was received
	};
	std::mutex trackerSampleMutex;
	TrackerSample trackerSample;

	void StoreTrackerSample(const vr::DriverPose_t &pose);
	TrackerSample LoadTrackerSample();

	double SlamToCorrectedScaleBase() const
	{
		double k = hmdTracker.hmdScale > 0.0 ? 1.0 / hmdTracker.hmdScale : 1.0;
		return k * hmdTracker.calibrationScale;
	}

	double SlamToCorrectedScale() const { return SlamToCorrectedScaleBase() * (1.0 + refine.scale); }

	IPCServer server;

	struct DeviceTransform
	{
		bool enabled = false;
		vr::HmdVector3d_t translation;
		vr::HmdQuaternion_t rotation;
		double scale;
	};

	DeviceTransform transforms[vr::k_unMaxTrackedDeviceCount];

	struct HmdTracker
	{
		// Written last / read first so the pose thread never sees a half-written config.
		std::atomic<bool> enabled{ false };
		// Follow SLAM HMD: headset keeps its SLAM pose, lighthouse devices follow it.
		bool followSlam = false;
		bool hideHeadTracker = false;
		bool slamFallback = true;
		bool enableAngularVelocity = false;
		float predictionTime = 1.0f;
		uint32_t hmdID = vr::k_unTrackedDeviceIndex_Hmd;
		uint32_t trackerID = vr::k_unTrackedDeviceIndexInvalid;
		vr::HmdQuaternion_t offsetRotation = { 1, 0, 0, 0 };
		vr::HmdVector3d_t offsetTranslation = { 0, 0, 0 };
		vr::HmdQuaternion_t calibrationRotation = { 1, 0, 0, 0 };
		vr::HmdVector3d_t calibrationTranslation = { 0, 0, 0 };
		double calibrationScale = 1.0;
		double hmdScale = 1.0;
	} hmdTracker;

	bool slamSync[vr::k_unMaxTrackedDeviceCount];

	struct DriftCorrection
	{
		bool valid = false;
		vr::HmdQuaternion_t rotation = { 1, 0, 0, 0 };
		vr::HmdVector3d_t translation = { 0, 0, 0 };

		LARGE_INTEGER lastUpdate = {};
		align::YawTranslationEstimator estimator;
	} drift;

	struct HeadFilter
	{
		bool enabled = false;
		bool valid = false;
		LARGE_INTEGER lastUpdate = {};
		oneeuro::Quat rotationFilter;
		oneeuro::Vec3 translationFilter;

		void reset() { valid = false; rotationFilter.reset(); translationFilter.reset(); }
	} headFilter;

	struct DeviceFilter
	{
		bool valid = false;
		LARGE_INTEGER lastUpdate = {};
		oneeuro::Quat rotationFilter;
		oneeuro::Vec3 translationFilter;
		oneeuro::Vec3 velocityFilter;
		oneeuro::Vec3 angularVelocityFilter;
		oneeuro::Vec3 accelerationFilter;
		oneeuro::Vec3 angularAccelerationFilter;

		void reset()
		{
			valid = false;
			rotationFilter.reset();
			translationFilter.reset();
			velocityFilter.reset();
			angularVelocityFilter.reset();
			accelerationFilter.reset();
			angularAccelerationFilter.reset();
		}
	};
	DeviceFilter deviceFilters[vr::k_unMaxTrackedDeviceCount];
	std::atomic<double> deviceSmoothing{ 0.0 };
	double deviceSmoothingLogTime = 0.0;

	struct TrackerFilter
	{
		KalmanFilterXYZ translation;
		void reset() { translation.reset(); }
	} trackerFilter;

	struct HeadVelocity
	{
		bool valid = false;
		LARGE_INTEGER lastUpdate = {};
		vr::HmdQuaternion_t prevRotation = { 1, 0, 0, 0 };
		oneeuro::Vec3 filter;

		void reset() { valid = false; filter.reset(); }
	} headVel;

	// Only for logging tracker state changes (OK <-> lost) with the yaw numbers around them.
	struct TrackerState
	{
		bool primed = false;
		bool wasOK = false;
		bool wasFallback = false;
		vr::ETrackingResult lastResult = vr::TrackingResult_Uninitialized;
		LARGE_INTEGER lostAt = {};
		double trackerYawAtLoss = 0.0;      // raw tracker yaw (deg)
		double relativeYawAtLoss = 0.0;     // drift yaw (deg)
		bool relativeYawAtLossValid = false;

		void reset() { primed = false; wasOK = false; wasFallback = false; lastResult = vr::TrackingResult_Uninitialized; relativeYawAtLossValid = false; }
	} trackerState;

	struct MountCheck
	{
		double tiltDeg = 0.0;
		double translationDeviation = 0.0;
		bool suspected = false;

		void reset() { tiltDeg = 0.0; translationDeviation = 0.0; suspected = false; }
	} mount;

	// Last logged drift, so a jump of the SLAM<->lighthouse relation shows up in the log.
	struct DriftLog
	{
		bool primed = false;
		double yawDeg = 0.0;
		vr::HmdVector3d_t translation = { 0, 0, 0 };
		double tauMs = 0.0;
		LARGE_INTEGER lastLog = {};

		void reset() { primed = false; }
	} driftLog;
};