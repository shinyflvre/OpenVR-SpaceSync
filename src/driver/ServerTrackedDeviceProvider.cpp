// SPDX-License-Identifier: AGPL-3.0-only
// Modified by Shinyflvres, 2026-08-23. Part of SpaceSync, a modified version of OpenVR-SpaceOverride by Nyabsi (AGPL-3.0). See NOTICE.md

#include "ServerTrackedDeviceProvider.h"
#include "Logging.h"
#include "InterfaceHookInjector.h"

#include "Version.h"

#include <cmath>
#include <cstdio>

static double QpcSeconds(const LARGE_INTEGER& t)
{
	LARGE_INTEGER freq;
	QueryPerformanceFrequency(&freq);
	return t.QuadPart / (double)freq.QuadPart;
}

static double FilterStep(LARGE_INTEGER& lastUpdate, bool primed)
{
	LARGE_INTEGER now, freq;
	QueryPerformanceCounter(&now);
	QueryPerformanceFrequency(&freq);

	double dt = primed ? (now.QuadPart - lastUpdate.QuadPart) / (double)freq.QuadPart : 0.0;
	lastUpdate = now;
	if (dt <= 0.0 || isnan(dt)) dt = 1.0 / 90.0;
	if (dt > 0.1) dt = 0.1;
	return dt;
}

vr::EVRInitError ServerTrackedDeviceProvider::Init(vr::IVRDriverContext* pDriverContext)
{
	TRACE("ServerTrackedDeviceProvider::Init()");
	VR_INIT_SERVER_DRIVER_CONTEXT(pDriverContext);

	OpenLogFile();
	LOG("SpaceSync driver " SPACECAL_VERSION_STRING " loaded");

	memset(transforms, 0, vr::k_unMaxTrackedDeviceCount * sizeof(DeviceTransform));
	memset(slamSync, 0, sizeof slamSync);

	headFilter.rotationFilter.params = { 5.0, 0.8, 1.0 };
	headFilter.translationFilter.params = { 5.0, 0.8, 1.0 };
	headVel.filter.params = { 8.0, 1.0, 1.0 };

	trackerFilter.translation.SetQ(2.5e-7);
  	trackerFilter.translation.SetR(1.0e-5);
	trackerFilter.translation.SetAdaptiveGain(4.0);

	InjectHooks(pDriverContext);
	server.Run();

	return vr::VRInitError_None;
}

void ServerTrackedDeviceProvider::Cleanup()
{
	LOG("SpaceSync driver unloaded");
	CloseLogFile();

	TRACE("ServerTrackedDeviceProvider::Cleanup()");
	server.Stop();
	DisableHooks();
	VR_CLEANUP_SERVER_DRIVER_CONTEXT();
}

void ServerTrackedDeviceProvider::SetDeviceTransform(const protocol::SetDeviceTransform& newTransform)
{
	if (newTransform.openVRID >= vr::k_unMaxTrackedDeviceCount)
		return;

	auto& tf = transforms[newTransform.openVRID];

	// Values first, enabled last, so the pose thread never sees enabled with a half-written transform.
	if (newTransform.updateTranslation)
		tf.translation = newTransform.translation;

	if (newTransform.updateRotation)
		tf.rotation = newTransform.rotation;

	if (newTransform.updateScale)
		tf.scale = newTransform.scale;

	tf.enabled = newTransform.enabled;
}

void ServerTrackedDeviceProvider::SetHmdTracker(const protocol::SetHmdTracker& cmd)
{
	const bool wasEnabled = hmdTracker.enabled.load(std::memory_order_acquire);
	if (!cmd.enabled)
		hmdTracker.enabled.store(false, std::memory_order_release);

	double cmdScale = cmd.hmdScale > 0.0 ? cmd.hmdScale : 1.0;
	bool keepOffsets = false;
	if (cmd.enabled && wasEnabled)
	{
		bool sameBase = OffsetsEqual(cmd.offsetRotation, cmd.offsetTranslation, cmdScale, hmdTracker.offsetRotation, hmdTracker.offsetTranslation, hmdTracker.hmdScale);
		bool samePublished;
		{
			std::lock_guard<std::mutex> lock(effectiveMutex);
			samePublished = publishedValid && OffsetsEqual(cmd.offsetRotation, cmd.offsetTranslation, cmdScale, published.rotation, published.translation, published.hmdScale);
		}
		keepOffsets = sameBase || samePublished;
	}

	vr::HmdQuaternion_t baseRotation = hmdTracker.offsetRotation;
	vr::HmdVector3d_t baseTranslation = hmdTracker.offsetTranslation;
	double baseScale = hmdTracker.hmdScale;

	if (hmdTracker.followSlam != cmd.followSlamHmd)
		hmdFrame.reset();

	hmdTracker.followSlam = cmd.followSlamHmd;
	hmdTracker.hideHeadTracker = cmd.hideHeadTracker && cmd.followSlamHmd;
	hmdTracker.slamFallback = cmd.slamFallback;
	hmdTracker.enableAngularVelocity = cmd.enableAngularVelocity;
	hmdTracker.predictionTime = cmd.predictionTime;
	hmdTracker.hmdID = cmd.hmdID;
	hmdTracker.trackerID = cmd.trackerID;
	hmdTracker.offsetRotation = cmd.offsetRotation;
	hmdTracker.offsetTranslation = cmd.offsetTranslation;
	hmdTracker.calibrationRotation = cmd.calibrationRotation;
	hmdTracker.calibrationTranslation = cmd.calibrationTranslation;
	hmdTracker.calibrationScale = cmd.calibrationScale > 0.0 ? cmd.calibrationScale : 1.0;
	hmdTracker.hmdScale = cmdScale;

	if (keepOffsets)
	{
		hmdTracker.offsetRotation = baseRotation;
		hmdTracker.offsetTranslation = baseTranslation;
		hmdTracker.hmdScale = baseScale;
	}

	if (!cmd.enabled)
	{
		drift.valid = false;
		drift.estimator.reset();
		headFilter.reset();
		headVel.reset();
		trackerFilter.reset();
		trackerState.reset();
		driftLog.reset();
		clock.reset();
		lastHmdTime = -1.0;
		tauRotTrim = 0.0;
		frames.reset();
		hmdFrame.reset();
		residualDiag.reset();
		yawBins.reset();
		mount.reset();
		refine.reset();
		refinePrimed = false;
		worldTilt = { 1, 0, 0, 0 };
		{
			std::lock_guard<std::mutex> lock(trackerSampleMutex);
			trackerSample.valid = false;
		}
		{
			std::lock_guard<std::mutex> lock(effectiveMutex);
			effectiveSharedValid = false;
			publishedValid = false;
		}
		memset(slamSync, 0, sizeof slamSync);
	}
	else
	{
		if (!wasEnabled || !keepOffsets)
		{
			refine.reset();
			mount.reset();
			refinePrimed = false;
			UpdateEffectiveOffsets();
			{
				std::lock_guard<std::mutex> lock(effectiveMutex);
				publishedValid = false;
			}
			if (wasEnabled)
				LOG("Head tracker offsets changed while running, mount refinement restarted");
			if (cmd.followSlamHmd && cmd.trackerID >= vr::k_unMaxTrackedDeviceCount)
			{
				drift.estimator.reset();
				drift.estimator.valid = true;
				drift.estimator.yaw = 0.0;
				drift.estimator.translation = { 0, 0, 0 };
				drift.rotation = { 1, 0, 0, 0 };
				drift.translation = { 0, 0, 0 };
				worldTilt = { 1, 0, 0, 0 };
				drift.valid = true;
				LOG("Follow mode without head tracker: static alignment from calibration, headset recenters are carried over");
			}
		}
		hmdTracker.enabled.store(true, std::memory_order_release);
	}
}

void ServerTrackedDeviceProvider::UpdateEffectiveOffsets()
{
	effective.rotation = refine.effectiveRotation(hmdTracker.offsetRotation);
	effective.translation = refine.effectiveTranslation(hmdTracker.offsetRotation, hmdTracker.offsetTranslation);
	effective.hmdScale = hmdTracker.hmdScale / (1.0 + refine.scale);
	std::lock_guard<std::mutex> lock(effectiveMutex);
	effectiveShared = effective;
	effectiveSharedValid = true;
}

void ServerTrackedDeviceProvider::SetSlamSync(const protocol::SetSlamSync& cmd)
{
	if (cmd.openVRID < vr::k_unMaxTrackedDeviceCount)
		slamSync[cmd.openVRID] = cmd.enabled;
}

void ServerTrackedDeviceProvider::SetOneEuro(const protocol::SetOneEuro& cmd)
{
	auto toParams = [](const protocol::OneEuroParams& p) {
		oneeuro::Params out;
		out.minCutoff = p.minCutoff < 0.01 ? 0.01 : p.minCutoff;
		out.beta = p.beta < 0.0 ? 0.0 : p.beta;
		out.dCutoff = p.dCutoff < 0.01 ? 0.01 : p.dCutoff;
		return out;
	};

	headFilter.rotationFilter.params = toParams(cmd.head);
	headFilter.translationFilter.params = toParams(cmd.head);

	if (headFilter.enabled && !cmd.headEnabled)
		headFilter.reset();
	headFilter.enabled = cmd.headEnabled;

	double ds = cmd.deviceSmoothing;
	if (ds < 0.0) ds = 0.0;
	if (ds > 100.0) ds = 100.0;
	double prev = deviceSmoothing.exchange(ds);
	if (prev != ds)
	{
		double minCutoff = 12.0 * std::pow(1.0 / 30.0, ds / 100.0);
		oneeuro::Params p = { minCutoff, 0.8, 1.0 };
		for (auto& f : deviceFilters)
		{
			f.rotationFilter.params = p;
			f.translationFilter.params = p;
			f.velocityFilter.params = p;
			f.angularVelocityFilter.params = p;
			f.accelerationFilter.params = p;
			f.angularAccelerationFilter.params = p;
			f.reset();
		}
		LARGE_INTEGER smoothNow;
		QueryPerformanceCounter(&smoothNow);
		LARGE_INTEGER smoothFreq;
		QueryPerformanceFrequency(&smoothFreq);
		double smoothNowSeconds = smoothNow.QuadPart / (double)smoothFreq.QuadPart;
		if (smoothNowSeconds - deviceSmoothingLogTime > 1.0)
		{
			deviceSmoothingLogTime = smoothNowSeconds;
			LOG("Lighthouse device smoothing %s: %.0f %% (cutoff %.2f Hz)", ds >= 0.5 ? "enabled" : "disabled", ds, minCutoff);
		}
	}
}

void ServerTrackedDeviceProvider::UpdateDrift(const vr::HmdQuaternion_t& correctedRotation, const double(&correctedPosition)[3],
	const vr::HmdQuaternion_t& rawRotation, const double(&rawPosition)[3], double confidence, double quality)
{
	vr::HmdQuaternion_t invTilt = quaternionConjugate(worldTilt);
	vr::HmdQuaternion_t correctedRotationT = quaternionNormalize(invTilt * correctedRotation);
	vr::HmdVector3d_t correctedPositionT = quaternionRotateVector(invTilt, correctedPosition);

	vr::HmdQuaternion_t relation = quaternionNormalize(correctedRotationT * quaternionConjugate(rawRotation));
	vr::HmdQuaternion_t instRot = quaternionProjectYaw(relation);

	double dt = FilterStep(drift.lastUpdate, drift.valid);
	double slamScale = SlamToCorrectedScale();

	auto distance = [](const vr::HmdVector3d_t& a, const vr::HmdVector3d_t& b) {
		return vecNorm(vecSub(a, b));
	};

	auto& est = drift.estimator;
	double yawBefore = est.valid ? est.yaw : 0.0;
	bool snapped = est.update(quaternionYawRad(instRot), correctedPositionT, vecFromArray(rawPosition), rawRotation, slamScale, confidence, dt, quality);
	drift.rotation = quaternionNormalize(worldTilt * est.rotation());
	drift.translation = quaternionRotateVector(worldTilt, est.translation);
	drift.valid = true;

	if (snapped)
	{
		if (est.lastSnapReverted)
			LOG("Alignment flip-flop reverted: yaw back to %.2f deg (was heading to %.2f), measurement latched unstable",
				est.yaw * 180.0 / POSE_PI, yawBefore * 180.0 / POSE_PI);
		else
			LOG("Drift jump compensated: yaw %.2f -> %.2f deg, translation delta %.1f cm, %d frames, sigma %.2f deg / %.1f mm",
				yawBefore * 180.0 / POSE_PI, est.yaw * 180.0 / POSE_PI, vecNorm(est.lastJumpTranslation) * 100.0, est.lastJumpFrames,
				est.sigmaYaw() * 180.0 / POSE_PI, est.sigmaTranslation() * 1000.0);
		if (std::fabs(est.lastJumpYaw) > 0.05 || vecNorm(est.lastJumpTranslation) > 0.15)
			clock.noteHmdDiscontinuity();
		refine.shift(est.lastJumpYaw, est.lastJumpTranslation);
		driftLog.yawDeg = quaternionYawDeg(drift.rotation);
		driftLog.translation = drift.translation;
		return;
	}

	double yawDeg = quaternionYawDeg(drift.rotation);
	LARGE_INTEGER now, freq;
	QueryPerformanceCounter(&now);
	QueryPerformanceFrequency(&freq);
	double tauMs = clock.tauRot() * 1000.0;
	if (!driftLog.primed)
	{
		driftLog.primed = true;
		driftLog.yawDeg = yawDeg;
		driftLog.translation = drift.translation;
		driftLog.tauMs = tauMs;
		driftLog.lastLog = now;
		LOG("Drift (SLAM->tracker) initialised: yaw %.2f deg, translation (%.3f, %.3f, %.3f) m, latency offset rot %.1f ms / pos %.1f ms",
			yawDeg, drift.translation.v[0], drift.translation.v[1], drift.translation.v[2], tauMs, clock.tauPos() * 1000.0);
	}
	else
	{
		double dYaw = wrapDeg(yawDeg - driftLog.yawDeg);
		double dTrans = distance(drift.translation, driftLog.translation);
		double dTau = tauMs - driftLog.tauMs;
		double sinceLog = (now.QuadPart - driftLog.lastLog.QuadPart) / (double)freq.QuadPart;

		if ((std::fabs(dYaw) > 1.0 || dTrans > 0.05 || std::fabs(dTau) > 5.0) && sinceLog > 1.0)
		{
			LOG("Drift (SLAM->tracker) changed: yaw %.2f -> %.2f deg (delta %.2f), translation (%.3f, %.3f, %.3f) -> (%.3f, %.3f, %.3f) m (delta %.1f cm), latency offset rot %.1f -> %.1f ms, pos %.1f ms",
				driftLog.yawDeg, yawDeg, dYaw,
				driftLog.translation.v[0], driftLog.translation.v[1], driftLog.translation.v[2],
				drift.translation.v[0], drift.translation.v[1], drift.translation.v[2], dTrans * 100.0,
				driftLog.tauMs, tauMs, clock.tauPos() * 1000.0);
			driftLog.yawDeg = yawDeg;
			driftLog.translation = drift.translation;
			driftLog.tauMs = tauMs;
			driftLog.lastLog = now;
		}
	}
}

void ServerTrackedDeviceProvider::GetStatus(protocol::DriverStatus& status)
{
	status.driftValid = drift.valid;
	status.mountShiftSuspected = mount.suspected;
	status.tiltDeg = mount.tiltDeg;
	status.translationDeviationM = mount.translationDeviation;
	status.latencyMs = clock.tauRot() * 1000.0;
	status.latencyPosMs = clock.tauPos() * 1000.0;
	status.jumpsCompensated = drift.estimator.jumps;
	status.sigmaYawDeg = drift.estimator.sigmaYaw() * 180.0 / POSE_PI;
	status.sigmaTranslationM = drift.estimator.sigmaTranslation();
	status.calmSeconds = refine.weight();
	status.refinementSolves = refine.applied;
	status.refinementTranslationSolves = refine.appliedTranslation;

	std::lock_guard<std::mutex> lock(effectiveMutex);
	status.refinementValid = effectiveSharedValid && hmdTracker.enabled.load(std::memory_order_acquire);
	status.offsetRotation = effectiveShared.rotation;
	status.offsetTranslation = effectiveShared.translation;
	status.hmdScale = effectiveShared.hmdScale;
	if (status.refinementValid)
	{
		published = effectiveShared;
		publishedValid = true;
	}
}

void ServerTrackedDeviceProvider::ApplyDrift(vr::DriverPose_t& pose) const
{
	double slamScale = SlamToCorrectedScale();

	pose.qWorldFromDriverRotation = quaternionNormalize(drift.rotation * pose.qWorldFromDriverRotation);

	pose.vecPosition[0] *= slamScale;
	pose.vecPosition[1] *= slamScale;
	pose.vecPosition[2] *= slamScale;
	for (int i = 0; i < 3; i++)
	{
		pose.vecVelocity[i] *= slamScale;
		pose.vecAcceleration[i] *= slamScale;
	}

	double scaledTranslation[3] = {
		pose.vecWorldFromDriverTranslation[0] * slamScale,
		pose.vecWorldFromDriverTranslation[1] * slamScale,
		pose.vecWorldFromDriverTranslation[2] * slamScale
	};
	vr::HmdVector3d_t rotatedTranslation = quaternionRotateVector(drift.rotation, scaledTranslation);
	pose.vecWorldFromDriverTranslation[0] = rotatedTranslation.v[0] + drift.translation.v[0];
	pose.vecWorldFromDriverTranslation[1] = rotatedTranslation.v[1] + drift.translation.v[1];
	pose.vecWorldFromDriverTranslation[2] = rotatedTranslation.v[2] + drift.translation.v[2];
}

// Follow mode: exact inverse of ApplyDrift, moves a calibrated lighthouse pose into SLAM space.
void ServerTrackedDeviceProvider::ApplyInverseDrift(vr::DriverPose_t& pose) const
{
	double slamScale = SlamToCorrectedScale();
	double invScale = slamScale > 0.0 ? 1.0 / slamScale : 1.0;

	vr::HmdQuaternion_t invRotation = quaternionConjugate(drift.rotation);

	pose.qWorldFromDriverRotation = quaternionNormalize(invRotation * pose.qWorldFromDriverRotation);

	pose.vecPosition[0] *= invScale;
	pose.vecPosition[1] *= invScale;
	pose.vecPosition[2] *= invScale;
	for (int i = 0; i < 3; i++)
	{
		pose.vecVelocity[i] *= invScale;
		pose.vecAcceleration[i] *= invScale;
	}

	double shifted[3] = {
		(pose.vecWorldFromDriverTranslation[0] - drift.translation.v[0]) * invScale,
		(pose.vecWorldFromDriverTranslation[1] - drift.translation.v[1]) * invScale,
		(pose.vecWorldFromDriverTranslation[2] - drift.translation.v[2]) * invScale
	};
	vr::HmdVector3d_t rotated = quaternionRotateVector(invRotation, shifted);
	pose.vecWorldFromDriverTranslation[0] = rotated.v[0];
	pose.vecWorldFromDriverTranslation[1] = rotated.v[1];
	pose.vecWorldFromDriverTranslation[2] = rotated.v[2];
}

bool ServerTrackedDeviceProvider::DetectHmdFrameJump(const vr::DriverPose_t& pose, double& jumpYaw, vr::HmdVector3d_t& jumpTranslation)
{
	vr::HmdQuaternion_t rotation = quaternionNormalize(pose.qWorldFromDriverRotation);
	vr::HmdVector3d_t translation = vecFromArray(pose.vecWorldFromDriverTranslation);

	auto& w = hmdFrame;
	if (!w.primed)
	{
		w.primed = true;
		w.rotation = rotation;
		w.translation = translation;
		return false;
	}

	vr::HmdQuaternion_t step = quaternionNormalize(rotation * quaternionConjugate(w.rotation));
	vr::HmdVector3d_t stepTranslation = vecSub(translation, quaternionRotateVector(step, w.translation));
	double angle = quaternionAngleRad(step);
	double distance = vecNorm(stepTranslation);

	w.rotation = rotation;
	w.translation = translation;

	if (angle < 0.25 * POSE_PI / 180.0 && distance < 0.003)
		return false;

	double tilt = quaternionAngleRad(quaternionNormalize(step * quaternionConjugate(quaternionProjectYaw(step))));
	if (tilt > 0.5 * POSE_PI / 180.0 || distance > 10.0)
		return false;

	jumpYaw = quaternionYawRad(quaternionProjectYaw(step));
	jumpTranslation = stepTranslation;
	w.jumps++;
	return true;
}

void ServerTrackedDeviceProvider::StoreTrackerSample(const vr::DriverPose_t& pose)
{
	TrackerSample s;
	s.valid = true;
	s.poseIsValid = pose.poseIsValid;
	s.deviceIsConnected = pose.deviceIsConnected;
	s.result = pose.result;

	// Same math vrserver uses for mDeviceToAbsoluteTracking.
	s.rotation = quaternionNormalize(pose.qWorldFromDriverRotation * pose.qRotation * pose.qDriverFromHeadRotation);

	vr::HmdVector3d_t headLocal = quaternionRotateVector(pose.qRotation, pose.vecDriverFromHeadTranslation);
	double driverLocal[3] = {
		pose.vecPosition[0] + headLocal.v[0],
		pose.vecPosition[1] + headLocal.v[1],
		pose.vecPosition[2] + headLocal.v[2]
	};
	vr::HmdVector3d_t world = quaternionRotateVector(pose.qWorldFromDriverRotation, driverLocal);
	s.position[0] = world.v[0] + pose.vecWorldFromDriverTranslation[0];
	s.position[1] = world.v[1] + pose.vecWorldFromDriverTranslation[1];
	s.position[2] = world.v[2] + pose.vecWorldFromDriverTranslation[2];

	// Velocities come in driver space, rotate them into world space.
	vr::HmdVector3d_t vel = quaternionRotateVector(pose.qWorldFromDriverRotation, pose.vecVelocity);
	vr::HmdVector3d_t angVel = quaternionRotateVector(pose.qWorldFromDriverRotation, pose.vecAngularVelocity);
	vr::HmdVector3d_t accel = quaternionRotateVector(pose.qWorldFromDriverRotation, pose.vecAcceleration);
	vr::HmdQuaternion_t deviceToWorld = quaternionNormalize(pose.qWorldFromDriverRotation * pose.qRotation);
	vr::HmdVector3d_t velDevice = quaternionRotateVector(deviceToWorld, pose.vecVelocity);
	vr::HmdVector3d_t angVelDevice = quaternionRotateVector(deviceToWorld, pose.vecAngularVelocity);
	vr::HmdVector3d_t accelDevice = quaternionRotateVector(deviceToWorld, pose.vecAcceleration);
	s.poseTimeOffset = pose.poseTimeOffset;
	QueryPerformanceCounter(&s.time);
	double nominal = QpcSeconds(s.time) + s.poseTimeOffset;

	if (frames.primed)
	{
		double span = nominal - frames.time;
		if (span > 0.001 && span < 0.05)
		{
			vr::HmdVector3d_t omegaFd = vecScale(quaternionToRotationVector(s.rotation * quaternionConjugate(frames.rotation)), 1.0 / span);
			vr::HmdVector3d_t velFd = vecScale(vecSub(vecFromArray(s.position), frames.position), 1.0 / span);
			double on = vecDot(omegaFd, omegaFd);
			if (on > 0.25)
			{
				frames.angWorld += vecDot(omegaFd, angVel);
				frames.angDevice += vecDot(omegaFd, angVelDevice);
				frames.angNorm += on;
			}
			double vn = vecDot(velFd, velFd);
			if (vn > 0.09)
			{
				frames.velWorld += vecDot(velFd, vel);
				frames.velDevice += vecDot(velFd, velDevice);
				frames.velNorm += vn;
			}
			if (frames.angNorm > 500.0)
			{
				double w = frames.angWorld / frames.angNorm, d = frames.angDevice / frames.angNorm;
				bool device = d > w;
				if ((!frames.decidedAngular || device != frames.angularDevice) && std::fabs(d - w) > 0.2)
				{
					frames.angularDevice = device;
					frames.decidedAngular = true;
					LOG("Tracker angular velocity frame: %s (agreement world %.2f, device %.2f)", device ? "device" : "world", w, d);
				}
				frames.angWorld *= 0.5; frames.angDevice *= 0.5; frames.angNorm *= 0.5;
			}
			if (frames.velNorm > 100.0)
			{
				double w = frames.velWorld / frames.velNorm, d = frames.velDevice / frames.velNorm;
				bool device = d > w;
				if ((!frames.decidedVelocity || device != frames.velocityDevice) && std::fabs(d - w) > 0.2)
				{
					frames.velocityDevice = device;
					frames.decidedVelocity = true;
					LOG("Tracker velocity frame: %s (agreement world %.2f, device %.2f)", device ? "device" : "world", w, d);
				}
				frames.velWorld *= 0.5; frames.velDevice *= 0.5; frames.velNorm *= 0.5;
			}
		}
	}
	frames.primed = true;
	frames.time = nominal;
	frames.rotation = s.rotation;
	frames.position = vecFromArray(s.position);

	if (frames.angularDevice) angVel = angVelDevice;
	if (frames.velocityDevice) { vel = velDevice; accel = accelDevice; }

	// Gain is the regression slope of a velocity difference onto the reported acceleration, so it is
	// the MSE-optimal shrinkage: 0 on drivers whose acceleration is noise, 1 where it is clean.
	auto& f = frames;
	vr::HmdVector3d_t accelUsed = { 0, 0, 0 };
	if (f.velCount >= FrameConvention::VelRing)
	{
		int oldest = f.velNext;
		double span = nominal - f.velHistoryTime[oldest];
		if (span > 0.01 && span < 0.1)
		{
			vr::HmdVector3d_t accelFd = vecScale(vecSub(vel, f.velHistory[oldest]), 1.0 / span);
			f.accNum += vecDot(accelFd, accel);
			f.accDen += vecDot(accel, accel);
			if (f.accDen > 500.0)
			{
				double gain = f.accNum / f.accDen;
				if (gain < 0.0) gain = 0.0;
				if (gain > 1.0) gain = 1.0;
				f.accGain = gain;
				f.accDecided = true;
				f.accNum *= 0.5;
				f.accDen *= 0.5;
			}
		}
	}
	if (f.accDecided)
		accelUsed = vecScale(accel, f.accGain);

	f.velHistory[f.velNext] = vel;
	f.velHistoryTime[f.velNext] = nominal;
	f.velNext = (f.velNext + 1) % FrameConvention::VelRing;
	if (f.velCount < FrameConvention::VelRing) f.velCount++;

	for (int i = 0; i < 3; i++)
	{
		s.velocity[i] = vel.v[i];
		s.angularVelocity[i] = angVel.v[i];
		s.acceleration[i] = accelUsed.v[i];
	}
	s.linSpeed = vecNorm(vel);
	s.angSpeed = vecNorm(angVel);

	vr::HmdVector3d_t headPoint = vecAdd(vecFromArray(s.position), quaternionRotateVector(s.rotation, hmdTracker.offsetTranslation));

	std::lock_guard<std::mutex> lock(trackerSampleMutex);
	trackerSample = s;
	clock.addTrackerPose(nominal, s.rotation, headPoint);
}

ServerTrackedDeviceProvider::TrackerSample ServerTrackedDeviceProvider::LoadTrackerSample()
{
	std::lock_guard<std::mutex> lock(trackerSampleMutex);
	return trackerSample;
}

void ServerTrackedDeviceProvider::NoteTrackerState(bool trackerOK, bool usingSlam, const vr::TrackedDevicePose_t& tp,
	double trackerYawDeg, bool relativeYawValid, double relativeYawDeg)
{
	auto& st = trackerState;

	LARGE_INTEGER now;
	QueryPerformanceCounter(&now);

	const char* source = hmdTracker.followSlam ? "SLAM (follow mode, lighthouse devices follow the HMD)"
		: (usingSlam ? "SLAM+drift" : (trackerOK ? "tracker" : (tp.bPoseIsValid ? "tracker (dead-reckoned, drift frozen)" : "none")));

	if (!st.primed)
	{
		st.primed = true;
		st.wasOK = trackerOK;
		st.wasFallback = usingSlam;
		st.lastResult = tp.eTrackingResult;
		st.lostAt = now;
		st.trackerYawAtLoss = trackerYawDeg;
		st.relativeYawAtLoss = relativeYawDeg;
		st.relativeYawAtLossValid = relativeYawValid;
		LOG("Head tracker: initial state result=%d poseValid=%d connected=%d source=%s", (int)tp.eTrackingResult, (int)tp.bPoseIsValid, (int)tp.bDeviceIsConnected, source);
		return;
	}

	if (trackerOK && !st.wasOK)
	{
		LARGE_INTEGER freq;
		QueryPerformanceFrequency(&freq);
		double lostFor = (now.QuadPart - st.lostAt.QuadPart) / (double)freq.QuadPart;
		double trackerDelta = wrapDeg(trackerYawDeg - st.trackerYawAtLoss);

		if (relativeYawValid && st.relativeYawAtLossValid)
		{
			// If the SLAM->tracker yaw delta stays non-zero, the lighthouse frame moved relative to the room.
			LOG("Head tracker: OK again after %.2f s (last result=%d), tracker yaw %.2f -> %.2f deg (delta %.2f), SLAM->tracker yaw %.2f -> %.2f deg (delta %.2f)",
				lostFor, (int)st.lastResult, st.trackerYawAtLoss, trackerYawDeg, trackerDelta,
				st.relativeYawAtLoss, relativeYawDeg, wrapDeg(relativeYawDeg - st.relativeYawAtLoss));
		}
		else
		{
			LOG("Head tracker: OK again after %.2f s (last result=%d), tracker yaw %.2f -> %.2f deg (delta %.2f)",
				lostFor, (int)st.lastResult, st.trackerYawAtLoss, trackerYawDeg, trackerDelta);
		}
	}
	else if (!trackerOK && st.wasOK)
	{
		st.lostAt = now;
		st.trackerYawAtLoss = trackerYawDeg;
		st.relativeYawAtLoss = relativeYawDeg;
		st.relativeYawAtLossValid = relativeYawValid;
		LOG("Head tracker: lost, result=%d poseValid=%d connected=%d, tracker yaw %.2f deg, SLAM->tracker yaw %.2f deg, head pose source now: %s",
			(int)tp.eTrackingResult, (int)tp.bPoseIsValid, (int)tp.bDeviceIsConnected, trackerYawDeg, relativeYawDeg, source);
	}
	else if (!trackerOK && (tp.eTrackingResult != st.lastResult || usingSlam != st.wasFallback))
	{
		LOG("Head tracker: still lost, result %d -> %d, poseValid=%d, head pose source: %s",
			(int)st.lastResult, (int)tp.eTrackingResult, (int)tp.bPoseIsValid, source);
	}

	st.wasOK = trackerOK;
	st.wasFallback = usingSlam;
	st.lastResult = tp.eTrackingResult;
}

bool ServerTrackedDeviceProvider::HandleDevicePoseUpdated(uint32_t openVRID, vr::DriverPose_t& pose)
{
	if (openVRID >= vr::k_unMaxTrackedDeviceCount)
		return true;

	const bool overrideEnabled = hmdTracker.enabled.load(std::memory_order_acquire);
	const bool followSlam = overrideEnabled && hmdTracker.followSlam;

	// Follow mode: stash the raw head tracker pose before any transform touches it.
	if (followSlam && openVRID == hmdTracker.trackerID)
		StoreTrackerSample(pose);

	auto& tf = transforms[openVRID];
	if (tf.enabled)
	{
		pose.qWorldFromDriverRotation = tf.rotation * pose.qWorldFromDriverRotation;

		pose.vecPosition[0] *= tf.scale;
		pose.vecPosition[1] *= tf.scale;
		pose.vecPosition[2] *= tf.scale;

		vr::HmdVector3d_t rotatedTranslation = quaternionRotateVector(tf.rotation, pose.vecWorldFromDriverTranslation);
		pose.vecWorldFromDriverTranslation[0] = rotatedTranslation.v[0] + tf.translation.v[0];
		pose.vecWorldFromDriverTranslation[1] = rotatedTranslation.v[1] + tf.translation.v[1];
		pose.vecWorldFromDriverTranslation[2] = rotatedTranslation.v[2] + tf.translation.v[2];

		// Follow mode: move the calibrated lighthouse device into the headset's SLAM space.
		if (followSlam && drift.valid)
			ApplyInverseDrift(pose);

		if (deviceSmoothing.load(std::memory_order_relaxed) >= 0.5 && !(followSlam && openVRID == hmdTracker.trackerID))
		{
			auto& df = deviceFilters[openVRID];
			if (!pose.poseIsValid || pose.result != vr::TrackingResult_Running_OK)
				df.reset();
			else
			{
				double fdt = FilterStep(df.lastUpdate, df.valid);
				df.valid = true;
				pose.qRotation = df.rotationFilter.filter(pose.qRotation, fdt);
				vr::HmdVector3d_t smoothed = df.translationFilter.filter(vecFromArray(pose.vecPosition), fdt);
				vr::HmdVector3d_t vel = df.velocityFilter.filter(vecFromArray(pose.vecVelocity), fdt);
				vr::HmdVector3d_t angVel = df.angularVelocityFilter.filter(vecFromArray(pose.vecAngularVelocity), fdt);
				vr::HmdVector3d_t acc = df.accelerationFilter.filter(vecFromArray(pose.vecAcceleration), fdt);
				vr::HmdVector3d_t angAcc = df.angularAccelerationFilter.filter(vecFromArray(pose.vecAngularAcceleration), fdt);
				for (int i = 0; i < 3; i++)
				{
					pose.vecPosition[i] = smoothed.v[i];
					pose.vecVelocity[i] = vel.v[i];
					pose.vecAngularVelocity[i] = angVel.v[i];
					pose.vecAcceleration[i] = acc.v[i];
					pose.vecAngularAcceleration[i] = angAcc.v[i];
				}
			}
		}
	}

	// Alignment is unaffected, it reads the raw sample stashed above.
	if (followSlam && hmdTracker.hideHeadTracker && openVRID == hmdTracker.trackerID)
	{
		const vr::HmdVector3d_t parked = { 0.0, 9001.0, 0.0 };
		vr::HmdVector3d_t local = quaternionRotateVector(
			quaternionConjugate(quaternionNormalize(pose.qWorldFromDriverRotation)),
			vecSub(parked, vecFromArray(pose.vecWorldFromDriverTranslation)));
		local = vecSub(local, quaternionRotateVector(pose.qRotation, pose.vecDriverFromHeadTranslation));

		for (int i = 0; i < 3; i++)
		{
			pose.vecPosition[i] = local.v[i];
			pose.vecVelocity[i] = 0.0;
			pose.vecAcceleration[i] = 0.0;
			pose.vecAngularVelocity[i] = 0.0;
			pose.vecAngularAcceleration[i] = 0.0;
		}
	}

	if (overrideEnabled)
	{
		if (openVRID == hmdTracker.hmdID)
		{
			bool rawValid = pose.poseIsValid && pose.deviceIsConnected && pose.result == vr::TrackingResult_Running_OK;
			vr::HmdQuaternion_t rawRotation = { 1, 0, 0, 0 };
			double rawPosition[3] = { 0, 0, 0 };
			if (rawValid)
			{
				rawRotation = quaternionNormalize(pose.qWorldFromDriverRotation * pose.qRotation * pose.qDriverFromHeadRotation);

				vr::HmdVector3d_t headLocal = quaternionRotateVector(pose.qRotation, pose.vecDriverFromHeadTranslation);
				double driverLocal[3] = {
					pose.vecPosition[0] + headLocal.v[0],
					pose.vecPosition[1] + headLocal.v[1],
					pose.vecPosition[2] + headLocal.v[2]
				};
				vr::HmdVector3d_t world = quaternionRotateVector(pose.qWorldFromDriverRotation, driverLocal);
				rawPosition[0] = world.v[0] + pose.vecWorldFromDriverTranslation[0];
				rawPosition[1] = world.v[1] + pose.vecWorldFromDriverTranslation[1];
				rawPosition[2] = world.v[2] + pose.vecWorldFromDriverTranslation[2];
			}

			if (followSlam)
			{
				// Follow mode: headset pose stays untouched, we only measure the drift here.
				TrackerSample ts = LoadTrackerSample();

				LARGE_INTEGER now, freq;
				QueryPerformanceCounter(&now);
				QueryPerformanceFrequency(&freq);
				double age = ts.valid ? (now.QuadPart - ts.time.QuadPart) / (double)freq.QuadPart : 1e9;
				double nowSeconds = now.QuadPart / (double)freq.QuadPart;
				double hmdTime = nowSeconds + pose.poseTimeOffset;
				bool freshPose = !rawValid || lastHmdTime < 0.0 || hmdTime - lastHmdTime > 0.001;
				if (rawValid)
				{
					// Before the pose enters the clock series, a step there reads as a velocity spike.
					double jumpYaw = 0.0;
					vr::HmdVector3d_t jumpTranslation = { 0, 0, 0 };
					if (DetectHmdFrameJump(pose, jumpYaw, jumpTranslation))
					{
						clock.noteHmdDiscontinuity();

						if (drift.valid)
						{
							drift.estimator.rebase(jumpYaw, jumpTranslation, SlamToCorrectedScale());
							drift.rotation = quaternionNormalize(worldTilt * drift.estimator.rotation());
							drift.translation = quaternionRotateVector(worldTilt, drift.estimator.translation);

							// Its sums are built from raw poses, which are now in a different frame.
							refine.clearSums();

							driftLog.yawDeg = quaternionYawDeg(drift.rotation);
							driftLog.translation = drift.translation;
						}
					}

					clock.addHmdPose(hmdTime, rawRotation, vecFromArray(rawPosition));
					lastHmdTime = hmdTime;
				}

				const bool trackerHasPose = ts.valid && age < 0.25 && ts.deviceIsConnected && ts.poseIsValid;
				const bool trackerOK = trackerHasPose && ts.result == vr::TrackingResult_Running_OK;

				vr::TrackedDevicePose_t tpLog = {};
				tpLog.bPoseIsValid = trackerHasPose;
				tpLog.bDeviceIsConnected = ts.valid && ts.deviceIsConnected;
				tpLog.eTrackingResult = ts.valid ? ts.result : vr::TrackingResult_Uninitialized;

				double relativeYaw = drift.valid ? quaternionYawDeg(drift.rotation) : 0.0;
				bool relativeYawValid = drift.valid;

				if (trackerOK && rawValid && freshPose)
				{
					// Predict the tracker sample to the headset pose's time (reported offsets + learned tau),
					// otherwise head motion leaks into the drift as speed x time gap.
					double dtBase = (pose.poseTimeOffset) - (ts.poseTimeOffset - age);
					auto clampAlign = [](double v) { return v > 0.30 ? 0.30 : (v < -0.10 ? -0.10 : v); };
					double tauRotNow = clock.tauRot() + tauRotTrim;
					double tauPosNow = clock.pos.primed ? clock.tauPos() : tauRotNow;
					double dtAlignRot = clampAlign(dtBase + tauRotNow);
					double dtAlignPos = clampAlign(dtBase + tauPosNow);

					vr::HmdQuaternion_t sampleRotation = quaternionNormalize(quaternionFromAngularVelocity(ts.angularVelocity, dtAlignRot) * ts.rotation);

					// s = u*t + a*t^2/2, already shrunk by the measured gain.
					vr::HmdVector3d_t secondOrder = vecScale(vecFromArray(ts.acceleration), 0.5 * dtAlignPos * dtAlignPos);
					double secondOrderNorm = vecNorm(secondOrder);
					if (secondOrderNorm > 0.05)
						secondOrder = vecScale(secondOrder, 0.05 / secondOrderNorm);

					double samplePosition[3] = {
						ts.position[0] + ts.velocity[0] * dtAlignPos + secondOrder.v[0],
						ts.position[1] + ts.velocity[1] * dtAlignPos + secondOrder.v[1],
						ts.position[2] + ts.velocity[2] * dtAlignPos + secondOrder.v[2]
					};

					vr::HmdQuaternion_t trackerRefRotation = quaternionNormalize(hmdTracker.calibrationRotation * sampleRotation);
					vr::HmdVector3d_t trackerRefPosition = quaternionRotateVector(hmdTracker.calibrationRotation, samplePosition);
					trackerRefPosition.v[0] += hmdTracker.calibrationTranslation.v[0];
					trackerRefPosition.v[1] += hmdTracker.calibrationTranslation.v[1];
					trackerRefPosition.v[2] += hmdTracker.calibrationTranslation.v[2];

					vr::HmdQuaternion_t headRotationBase = quaternionNormalize(trackerRefRotation * hmdTracker.offsetRotation);
					vr::HmdVector3d_t headPositionBase = vecAdd(trackerRefPosition, quaternionRotateVector(trackerRefRotation, hmdTracker.offsetTranslation));
					vr::HmdQuaternion_t headRotation = quaternionNormalize(trackerRefRotation * effective.rotation);
					vr::HmdVector3d_t headPositionVec = vecAdd(trackerRefPosition, quaternionRotateVector(trackerRefRotation, effective.translation));
					double headPosition[3] = { headPositionVec.v[0], headPositionVec.v[1], headPositionVec.v[2] };

					vr::HmdQuaternion_t instRot = quaternionProjectYaw(quaternionNormalize(headRotation * quaternionConjugate(rawRotation)));
					relativeYaw = quaternionYawDeg(instRot);
					relativeYawValid = true;

					if (clock.due(nowSeconds))
					{
						bool updated;
						{
							std::lock_guard<std::mutex> lock(trackerSampleMutex);
							updated = clock.solve(nowSeconds);
						}
						if (updated && nowSeconds - clockLogTime > 10.0)
						{
							clockLogTime = nowSeconds;
							LOG("Clock alignment: rot %.1f ms (found %.1f, corr %.2f), pos %.1f ms (found %.1f, corr %.2f)",
								clock.tauRot() * 1000.0, clock.rot.lastFound * 1000.0, clock.rot.lastCorrelation,
								clock.tauPos() * 1000.0, clock.pos.lastFound * 1000.0, clock.pos.lastCorrelation);
						}
					}

					if (!trackerOK)
						trackerNotOKTime = nowSeconds;
					double sinceNotOK = nowSeconds - trackerNotOKTime;
					double fReacquire = sinceNotOK < 2.0 ? 0.0 : (sinceNotOK < 5.0 ? (sinceNotOK - 2.0) / 3.0 : 1.0);

					vr::HmdVector3d_t headFwd = quaternionRotateVector(rawRotation, { 0.0, 0.0, -1.0 });
					double horizontal = std::sqrt(headFwd.v[0] * headFwd.v[0] + headFwd.v[2] * headFwd.v[2]);
					double fPitch = (horizontal - 0.26) / (0.70 - 0.26);
					if (fPitch < 0.15) fPitch = 0.15;
					if (fPitch > 1.0) fPitch = 1.0;

					double fStable = drift.estimator.unstable() ? 0.1 : 1.0;
					double quality = fPitch * fReacquire * fStable;

					int trust = quality >= 0.5 ? 0 : (drift.estimator.unstable() ? 2 : 1);
					if (trust != measurementTrustState && nowSeconds - trustLogTime > 2.0)
					{
						trustLogTime = nowSeconds;
						measurementTrustState = trust;
						double pitchDeg = std::atan2(headFwd.v[1], horizontal) * 180.0 / POSE_PI;
						if (trust == 0)
							LOG("Measurement trust restored (head pitch %+.0f deg)", pitchDeg);
						else
							LOG("Measurement trust reduced: %s (head pitch %+.0f deg, tracker reacquired %.1f s ago, quality %.2f) - alignment holds instead of following",
								trust == 2 ? "measurement unstable" : "weak geometry", pitchDeg, sinceNotOK, quality);
					}

					double confidence = drift.valid ? DriftSampleConfidence(ts.linSpeed, ts.angSpeed) * quality : 1.0;
					const double maxLinSpeed = 2.75;
					const double maxAngSpeed = 3.5;
					const bool usable = age <= 0.05 && (!drift.valid || (ts.linSpeed < maxLinSpeed && ts.angSpeed < maxAngSpeed));
					if (usable)
						UpdateDrift(headRotation, headPosition, rawRotation, rawPosition, confidence, quality);

					if (usable && drift.valid && drift.estimator.weightSum >= 1.0)
					{
						vr::HmdVector3d_t angVelCal = quaternionRotateVector(hmdTracker.calibrationRotation, ts.angularVelocity);
						double omegaYaw = angVelCal.v[1];
						if (quality > 0.5 && std::fabs(omegaYaw) > 1.0)
						{
							double r = wrapRad(quaternionYawRad(instRot) - drift.estimator.yaw);
							if (std::fabs(r) < 0.35)
							{
								residualDiag.yawNum += omegaYaw * r;
								residualDiag.yawDen += omegaYaw * omegaYaw;
								residualDiag.yawFrames++;
							}
						}
						vr::HmdVector3d_t headVel = quaternionRotateVector(hmdTracker.calibrationRotation, ts.velocity);
						double speed = vecNorm(headVel);
						if (quality > 0.5 && speed > 0.5)
						{
							vr::HmdVector3d_t predicted = vecAdd(quaternionRotateVector(drift.rotation, vecScale(vecFromArray(rawPosition), SlamToCorrectedScale())), drift.translation);
							vr::HmdVector3d_t e = vecSub(headPositionVec, predicted);
							if (vecNorm(e) < 0.3)
							{
								residualDiag.posNum += vecDot(e, headVel);
								residualDiag.posDen += speed * speed;
								residualDiag.posFrames++;
							}
						}
						if (nowSeconds - residualDiag.lastLog > 10.0 && (residualDiag.yawFrames > 30 || residualDiag.posFrames > 30))
						{
							double yawSlope = residualDiag.yawDen > 0.0 ? residualDiag.yawNum / residualDiag.yawDen : 0.0;
							if (residualDiag.yawFrames >= 50)
							{
								tauRotTrim -= 0.3 * yawSlope;
								if (tauRotTrim > 0.025) tauRotTrim = 0.025;
								if (tauRotTrim < -0.025) tauRotTrim = -0.025;
							}
							LOG("Alignment residual: yaw slope %+.1f ms (%d frames), position slope %+.1f ms (%d frames), tau rot %.1f / pos %.1f ms, trim %+.1f ms",
								yawSlope * 1000.0, residualDiag.yawFrames,
								residualDiag.posDen > 0.0 ? residualDiag.posNum / residualDiag.posDen * 1000.0 : 0.0, residualDiag.posFrames,
								clock.tauRot() * 1000.0, clock.tauPos() * 1000.0, tauRotTrim * 1000.0);
							residualDiag.reset();
							residualDiag.lastLog = nowSeconds;
						}

						if (confidence > 0.5)
						{
							vr::HmdVector3d_t predictedHead = vecAdd(quaternionRotateVector(drift.rotation, vecScale(vecFromArray(rawPosition), SlamToCorrectedScale())), drift.translation);
							vr::HmdVector3d_t residualWorld = vecSub(headPositionVec, predictedHead);
							if (vecNorm(residualWorld) < 0.2)
							{
								vr::HmdVector3d_t residualHead = quaternionRotateVector(quaternionConjugate(rawRotation), quaternionRotateVector(quaternionConjugate(drift.rotation), residualWorld));
								double headYaw = quaternionYawRad(rawRotation);
								int bin = (int)std::floor((headYaw + POSE_PI) / (2.0 * POSE_PI) * YawBins::Count);
								if (bin < 0) bin = 0;
								if (bin >= YawBins::Count) bin = YawBins::Count - 1;
								for (int k = 0; k < 3; k++)
								{
									yawBins.sumWorld[bin][k] += residualWorld.v[k];
									yawBins.sumHead[bin][k] += residualHead.v[k];
									yawBins.sumMeas[bin][k] += residualWorld.v[k] + drift.translation.v[k];
								}
								yawBins.frames[bin]++;
							}
							if (nowSeconds - yawBins.lastLog > 60.0)
							{
								yawBins.lastLog = nowSeconds;
								char line[1024];
								int used = std::snprintf(line, sizeof line, "Head residual by yaw (world mm / head mm, n):");
								for (int i = 0; i < YawBins::Count && used < (int)sizeof line - 1; i++)
								{
									int n = yawBins.frames[i];
									if (n < 30)
										used += std::snprintf(line + used, sizeof line - used, " [%d: -]", i * 45 - 180);
									else
										used += std::snprintf(line + used, sizeof line - used, " [%d: %.0f,%.0f,%.0f / %.0f,%.0f,%.0f n%d]", i * 45 - 180,
											yawBins.sumWorld[i][0] / n * 1000.0, yawBins.sumWorld[i][1] / n * 1000.0, yawBins.sumWorld[i][2] / n * 1000.0,
											yawBins.sumHead[i][0] / n * 1000.0, yawBins.sumHead[i][1] / n * 1000.0, yawBins.sumHead[i][2] / n * 1000.0, n);
								}
								LOG("%s", line);
								double totalMeas[3] = { 0.0, 0.0, 0.0 };
								long totalFrames = 0;
								for (int i = 0; i < YawBins::Count; i++)
								{
									if (yawBins.frames[i] < 30) continue;
									totalFrames += yawBins.frames[i];
									for (int k = 0; k < 3; k++) totalMeas[k] += yawBins.sumMeas[i][k];
								}
								if (totalFrames > 0)
								{
									double meanMeas[3] = { totalMeas[0] / totalFrames, totalMeas[1] / totalFrames, totalMeas[2] / totalFrames };
									used = std::snprintf(line, sizeof line, "Head measurement offset by yaw (mm vs mean):");
									for (int i = 0; i < YawBins::Count && used < (int)sizeof line - 1; i++)
									{
										int n = yawBins.frames[i];
										if (n < 30)
											used += std::snprintf(line + used, sizeof line - used, " [%d: -]", i * 45 - 180);
										else
											used += std::snprintf(line + used, sizeof line - used, " [%d: %.0f,%.0f,%.0f]", i * 45 - 180,
												(yawBins.sumMeas[i][0] / n - meanMeas[0]) * 1000.0,
												(yawBins.sumMeas[i][1] / n - meanMeas[1]) * 1000.0,
												(yawBins.sumMeas[i][2] / n - meanMeas[2]) * 1000.0);
									}
									LOG("%s", line);
								}
								yawBins.reset();
							}
						}
					}

					double dtRefine = FilterStep(refineLast, refinePrimed);
					refinePrimed = true;
					bool refineTauMoved = std::fabs(tauRotNow - refineTauRot) > 0.015 || std::fabs(tauPosNow - refineTauPos) > 0.015;
					if (refine.weight() > 0.0 && refineTauMoved)
					{
						if (refineTauMovedSince < 0.0)
							refineTauMovedSince = nowSeconds;
						else if (nowSeconds - refineTauMovedSince > 10.0)
						{
							LOG("Mount refinement restarted, time alignment moved (rot %.1f -> %.1f ms, pos %.1f -> %.1f ms)",
								refineTauRot * 1000.0, tauRotNow * 1000.0, refineTauPos * 1000.0, tauPosNow * 1000.0);
							refine.clearSums();
							refineTauMovedSince = -1.0;
						}
					}
					else
						refineTauMovedSince = -1.0;
					if (refine.weight() <= 0.0)
					{
						refineTauRot = tauRotNow;
						refineTauPos = tauPosNow;
					}
					if (usable && drift.valid && confidence > 0.5 && clock.rot.primed && clock.pos.primed)
					{
						vr::HmdQuaternion_t invTiltRef = quaternionConjugate(worldTilt);
						refine.add(quaternionNormalize(invTiltRef * headRotationBase), quaternionRotateVector(invTiltRef, headPositionBase), rawRotation, vecFromArray(rawPosition), drift.estimator.yaw, SlamToCorrectedScaleBase(), confidence * dtRefine);
						align::MountRefiner::Delta applied;
						if (refine.evaluate(applied))
						{
							double scaleBefore = SlamToCorrectedScale();
							bool changed = vecNorm(applied.rotation) > 0.0 || vecNorm(applied.translation) > 0.0 || applied.scale != 0.0;
							if (vecNorm(applied.tilt) > 0.0)
							{
								vr::HmdQuaternion_t stepQ = quaternionFromRotationVector(applied.tilt);
								worldTilt = quaternionNormalize(worldTilt * stepQ);
								drift.estimator.absorbTiltStep(stepQ);
								drift.rotation = quaternionNormalize(worldTilt * drift.estimator.rotation());
								drift.translation = quaternionRotateVector(worldTilt, drift.estimator.translation);
								refine.clearSums();
								vr::HmdVector3d_t tiltTotal = quaternionToRotationVector(worldTilt);
								LOG("World tilt refined: step (%.3f, %.3f) deg, total (%.3f, %.3f) deg = %.3f deg, holdout rms %.3f -> %.3f deg, obs (%.2f, %.2f)",
									applied.tilt.v[0] * 180.0 / POSE_PI, applied.tilt.v[2] * 180.0 / POSE_PI,
									tiltTotal.v[0] * 180.0 / POSE_PI, tiltTotal.v[2] * 180.0 / POSE_PI,
									quaternionAngleRad(worldTilt) * 180.0 / POSE_PI,
									refine.lastTiltBefore * 180.0 / POSE_PI, refine.lastTiltAfter * 180.0 / POSE_PI,
									refine.obsTilt[0], refine.obsTilt[1]);
							}
							if (changed)
							{
								drift.estimator.absorbRefinement(applied.rotation, applied.translation, applied.scale, rawRotation, vecFromArray(rawPosition), headRotationBase, SlamToCorrectedScaleBase(), scaleBefore);
								UpdateEffectiveOffsets();
							}

							bool was = mount.suspected;
							mount.suspected = refine.suspected;
							mount.tiltDeg = refine.rotationAngle() * 180.0 / POSE_PI;
							mount.translationDeviation = refine.translationDistance();
							if (was != mount.suspected)
								LOG("Mount check: %s (solution wants %.2f deg, %.1f cm)", mount.suspected ? "tracker may have moved on the headset" : "back to normal",
									vecNorm(refine.solvedRotation) * 180.0 / POSE_PI, vecNorm(refine.solvedTranslation) * 100.0);

							if (refine.lastHadReference)
								LOG("Mount refinement block %u: translation rms %.1f -> %.1f mm (%s), rotation %.2f -> %.2f deg (%s), solved (%.1f, %.1f, %.1f) mm / (%.2f, %.2f, %.2f) deg / %+.3f %%, applied total (%.1f, %.1f, %.1f) mm / (%.2f, %.2f, %.2f) deg / scale %+.3f %%, obs (%.2f, %.2f, %.2f | %.2f, %.2f, %.2f | %.2f m2), tilt cand (%.2f, %.2f) deg %s obs (%.2f, %.2f)",
									refine.solves, refine.lastRmsBefore * 1000.0, refine.lastRmsAfter * 1000.0, refine.lastAcceptedTranslation ? "accepted" : "rejected",
									refine.lastRotBefore * 180.0 / POSE_PI, refine.lastRotAfter * 180.0 / POSE_PI, refine.lastAcceptedRotation ? "accepted" : "rejected",
									refine.solvedTranslation.v[0] * 1000.0, refine.solvedTranslation.v[1] * 1000.0, refine.solvedTranslation.v[2] * 1000.0,
									refine.solvedRotation.v[0] * 180.0 / POSE_PI, refine.solvedRotation.v[1] * 180.0 / POSE_PI, refine.solvedRotation.v[2] * 180.0 / POSE_PI,
									refine.solvedScale * 100.0,
									refine.translation.v[0] * 1000.0, refine.translation.v[1] * 1000.0, refine.translation.v[2] * 1000.0,
									refine.rotation.v[0] * 180.0 / POSE_PI, refine.rotation.v[1] * 180.0 / POSE_PI, refine.rotation.v[2] * 180.0 / POSE_PI,
									refine.scale * 100.0,
									refine.obsTranslation[0], refine.obsTranslation[1], refine.obsTranslation[2],
									refine.obsRotation[0], refine.obsRotation[1], refine.obsRotation[2], refine.obsScale,
									refine.solvedTilt.v[0] * 180.0 / POSE_PI, refine.solvedTilt.v[2] * 180.0 / POSE_PI,
									refine.lastAcceptedTilt ? "applied" : "held",
									refine.obsTilt[0], refine.obsTilt[1]);
							else
								LOG("Mount refinement block %u: reference block collected, obs (%.2f, %.2f, %.2f | %.2f, %.2f, %.2f | %.2f m2), tilt obs (%.2f, %.2f)", refine.solves,
									refine.obsTranslation[0], refine.obsTranslation[1], refine.obsTranslation[2],
									refine.obsRotation[0], refine.obsRotation[1], refine.obsRotation[2], refine.obsScale,
									refine.obsTilt[0], refine.obsTilt[1]);
						}
					}
				}

				NoteTrackerState(trackerOK, true, tpLog, quaternionYawDeg(ts.rotation), relativeYawValid, relativeYaw);
				return true;
			}

			vr::PropertyContainerHandle_t container = vr::VRProperties()->TrackedDeviceToPropertyContainer(openVRID);

			float displayFrequency = vr::VRProperties()->GetFloatProperty(container, vr::Prop_DisplayFrequency_Float);
			if (!(displayFrequency > 0.0f))
				displayFrequency = 90.0f;

			vr::TrackedDevicePose_t poses[vr::k_unMaxTrackedDeviceCount];
			vr::VRServerDriverHost()->GetRawTrackedDevicePoses((float)((1.0 / displayFrequency) * hmdTracker.predictionTime), poses, vr::k_unMaxTrackedDeviceCount);

			vr::TrackedDevicePose_t tp = {};
			if (hmdTracker.trackerID < vr::k_unMaxTrackedDeviceCount)
				tp = poses[hmdTracker.trackerID];

			// Lighthouse keeps bPoseIsValid true while dead-reckoning on the IMU (OutOfRange),
			// only Running_OK is a real optical pose.
			const bool trackerHasPose = tp.bDeviceIsConnected && tp.bPoseIsValid;
			const bool trackerOK = trackerHasPose && tp.eTrackingResult == vr::TrackingResult_Running_OK;
			const bool slamAvailable = hmdTracker.slamFallback && drift.valid && rawValid;
			// Dead-reckoned poses only when there's no SLAM fallback.
			const bool useTracker = trackerOK || (trackerHasPose && !slamAvailable);

			vr::HmdQuaternion_t trackerQuat = HmdQuaternion_FromMatrix(tp.mDeviceToAbsoluteTracking);

			if (useTracker)
			{
				vr::HmdQuaternion_t trackerRefRotation = quaternionNormalize(hmdTracker.calibrationRotation * trackerQuat);

				vr::HmdVector3d_t filteredTrackerPos = trackerFilter.translation.update({
					tp.mDeviceToAbsoluteTracking.m[0][3],
					tp.mDeviceToAbsoluteTracking.m[1][3],
					tp.mDeviceToAbsoluteTracking.m[2][3]
				});
				double trackerPos[3] = {
					filteredTrackerPos.v[0],
					filteredTrackerPos.v[1],
					filteredTrackerPos.v[2]
				};

				vr::HmdVector3d_t trackerRefPosition = quaternionRotateVector(hmdTracker.calibrationRotation, trackerPos);

				trackerRefPosition.v[0] += hmdTracker.calibrationTranslation.v[0];
				trackerRefPosition.v[1] += hmdTracker.calibrationTranslation.v[1];
				trackerRefPosition.v[2] += hmdTracker.calibrationTranslation.v[2];

				vr::HmdQuaternion_t hmdRotation = quaternionNormalize(trackerRefRotation * hmdTracker.offsetRotation);
				vr::HmdVector3d_t offset = quaternionRotateVector(trackerRefRotation, hmdTracker.offsetTranslation.v);

				pose.qWorldFromDriverRotation = { 1, 0, 0, 0 };
				pose.vecWorldFromDriverTranslation[0] = 0;
				pose.vecWorldFromDriverTranslation[1] = 0;
				pose.vecWorldFromDriverTranslation[2] = 0;

				pose.qDriverFromHeadRotation = { 1, 0, 0, 0 };
				pose.vecDriverFromHeadTranslation[0] = 0;
				pose.vecDriverFromHeadTranslation[1] = 0;
				pose.vecDriverFromHeadTranslation[2] = 0;

				pose.qRotation = hmdRotation;
				pose.vecPosition[0] = trackerRefPosition.v[0] + offset.v[0];
				pose.vecPosition[1] = trackerRefPosition.v[1] + offset.v[1];
				pose.vecPosition[2] = trackerRefPosition.v[2] + offset.v[2];

				if (headFilter.enabled)
				{
					double dt = FilterStep(headFilter.lastUpdate, headFilter.valid);
					headFilter.valid = true;

					pose.qRotation = headFilter.rotationFilter.filter(pose.qRotation, dt);

					vr::HmdVector3d_t headPos = headFilter.translationFilter.filter(
						{ pose.vecPosition[0], pose.vecPosition[1], pose.vecPosition[2] }, dt);
					pose.vecPosition[0] = headPos.v[0];
					pose.vecPosition[1] = headPos.v[1];
					pose.vecPosition[2] = headPos.v[2];
				}

				double trackerVel[3] = {
					tp.vVelocity.v[0],
					tp.vVelocity.v[1],
					tp.vVelocity.v[2]
				};

				double trackerAngVel[3] = {
					tp.vAngularVelocity.v[0],
					tp.vAngularVelocity.v[1],
					tp.vAngularVelocity.v[2]
				};

				vr::HmdVector3d_t vel = quaternionRotateVector(hmdTracker.calibrationRotation, trackerVel);
				vel.v[0] *= hmdTracker.calibrationScale;
				vel.v[1] *= hmdTracker.calibrationScale;
				vel.v[2] *= hmdTracker.calibrationScale;

				double dtAng = FilterStep(headVel.lastUpdate, headVel.valid);
				vr::HmdVector3d_t headAngVel = { 0, 0, 0 };
				if (headVel.valid)
					headAngVel = headVel.filter.filter(quaternionAngularVelocity(pose.qRotation, headVel.prevRotation, dtAng), dtAng);
				headVel.prevRotation = pose.qRotation;
				headVel.valid = true;

				vr::HmdVector3d_t tangential = {
					headAngVel.v[1] * offset.v[2] - headAngVel.v[2] * offset.v[1],
					headAngVel.v[2] * offset.v[0] - headAngVel.v[0] * offset.v[2],
					headAngVel.v[0] * offset.v[1] - headAngVel.v[1] * offset.v[0]
				};

				for (int i = 0; i < 3; i++)
				{
					pose.vecVelocity[i] = vel.v[i] + tangential.v[i];
					pose.vecAngularVelocity[i] = hmdTracker.enableAngularVelocity ? headAngVel.v[i] : 0.0;
				}

				pose.poseIsValid = true;
				pose.deviceIsConnected = true;
				// Report the real tracking state.
				pose.result = trackerOK ? vr::TrackingResult_Running_OK : tp.eTrackingResult;
				pose.shouldApplyHeadModel = false;
				pose.poseTimeOffset = 0;

				// Log state transitions with the current SLAM->tracker yaw.
				double relativeYaw = drift.valid ? quaternionYawDeg(drift.rotation) : 0.0;
				bool relativeYawValid = drift.valid;
				if (trackerOK && rawValid)
				{
					relativeYaw = quaternionYawDeg(quaternionProjectYaw(quaternionNormalize(pose.qRotation * quaternionConjugate(rawRotation))));
					relativeYawValid = true;
				}
				NoteTrackerState(trackerOK, false, tp, quaternionYawDeg(trackerQuat), relativeYawValid, relativeYaw);

				// Drift only learns from Running_OK poses, so a bad frame can never become the new reference.
				if (trackerOK && rawValid)
				{
					double linSpeed = sqrt(
						trackerVel[0] * trackerVel[0] +
						trackerVel[1] * trackerVel[1] +
						trackerVel[2] * trackerVel[2]);

					double angSpeed = sqrt(
						trackerAngVel[0] * trackerAngVel[0] +
						trackerAngVel[1] * trackerAngVel[1] +
						trackerAngVel[2] * trackerAngVel[2]);

					const double maxLinSpeed = 2.75;
					const double maxAngSpeed = 3.5;

					if (!drift.valid || (linSpeed < maxLinSpeed && angSpeed < maxAngSpeed))
						UpdateDrift(pose.qRotation, pose.vecPosition, rawRotation, rawPosition,
							drift.valid ? DriftSampleConfidence(linSpeed, angSpeed) : 1.0);
				}
			}
			else {
				headVel.reset();
				trackerFilter.reset();

				NoteTrackerState(false, hmdTracker.slamFallback && drift.valid, tp, quaternionYawDeg(trackerQuat), drift.valid, drift.valid ? quaternionYawDeg(drift.rotation) : 0.0);

				if (!hmdTracker.slamFallback) {
					pose.qWorldFromDriverRotation = hmdTracker.calibrationRotation;
					pose.vecWorldFromDriverTranslation[0] = hmdTracker.calibrationTranslation.v[0];
					pose.vecWorldFromDriverTranslation[1] = hmdTracker.calibrationTranslation.v[1];
					pose.vecWorldFromDriverTranslation[2] = hmdTracker.calibrationTranslation.v[2];

					pose.qDriverFromHeadRotation = { 1, 0, 0, 0 };
					pose.vecDriverFromHeadTranslation[0] = 0;
					pose.vecDriverFromHeadTranslation[1] = 0;
					pose.vecDriverFromHeadTranslation[2] = 0;

					pose.qRotation = { 1, 0, 0, 0 };
					pose.vecPosition[0] = 0;
					pose.vecPosition[1] = 0;
					pose.vecPosition[2] = 0;

					for (int i = 0; i < 3; i++)
					{
						pose.vecVelocity[i] = 0;
						pose.vecAngularVelocity[i] = 0;
					}

					pose.poseIsValid = false;
					pose.deviceIsConnected = true;
					pose.result = vr::TrackingResult_Running_OutOfRange;
					pose.shouldApplyHeadModel = false;
					pose.poseTimeOffset = 0;
				}
				else if (drift.valid) {
					// SLAM fallback with the last good drift.
					ApplyDrift(pose);
				}
			}
		}
		else if (slamSync[openVRID] && drift.valid && !followSlam)
		{
			ApplyDrift(pose);
		}
	}

	return true;
}
