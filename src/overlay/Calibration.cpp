// SPDX-License-Identifier: AGPL-3.0-only
// Modified by Shinyflvres, 2026-08-23. Part of SpaceSync, a modified version of OpenVR-SpaceOverride by Nyabsi (AGPL-3.0). See NOTICE.md

#define WIN32_LEAN_AND_MEAN

#include "Calibration.h"
#include "Configuration.h"
#include "IPCClient.h"
#include "Sound.h"

#include <string>
#include <vector>
#include <iostream>
#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <map>
#include <sstream>

#include <Dense>


static IPCClient Driver;
CalibrationContext CalCtx;

void InitCalibrator()
{
	Driver.TryConnect();
}

bool DriverConnected()
{
	return Driver.IsConnected();
}

struct Pose
{
	Eigen::Matrix3d rot;
	Eigen::Vector3d trans;

	Pose() { }
	Pose(vr::HmdMatrix34_t hmdMatrix)
	{
		for (int i = 0; i < 3; i++) {
			for (int j = 0; j < 3; j++) {
				rot(i,j) = hmdMatrix.m[i][j];
			}
		}
		trans = Eigen::Vector3d(hmdMatrix.m[0][3], hmdMatrix.m[1][3], hmdMatrix.m[2][3]);
	}
	Pose(double x, double y, double z) : trans(Eigen::Vector3d(x,y,z)) { }
};

struct Sample
{
	Pose ref, target;
	bool valid;
	double time = 0.0;
	Sample() : valid(false) { }
	Sample(Pose ref, Pose target) : valid(true), ref(ref), target(target) { }
};

struct DSample
{
	bool valid;
	Eigen::Vector3d ref, target;
};

bool StartsWith(const std::string &str, const std::string &prefix)
{
	if (str.length() < prefix.length())
		return false;

	return str.compare(0, prefix.length(), prefix) == 0;
}

bool EndsWith(const std::string &str, const std::string &suffix)
{
	if (str.length() < suffix.length())
		return false;

	return str.compare(str.length() - suffix.length(), suffix.length(), suffix) == 0;
}

Eigen::Vector3d RotationVector(const Eigen::Matrix3d& rot)
{
	Eigen::AngleAxisd aa(rot);
	return aa.angle() * aa.axis();
}

double AngleFromRotationMatrix3(const Eigen::Matrix3d& rot)
{
	double c = (rot(0, 0) + rot(1, 1) + rot(2, 2) - 1.0) / 2.0;
	return acos(max(-1.0, min(1.0, c)));
}

struct DetectionState
{
	std::vector<uint32_t> candidates;
	std::vector<std::vector<double>> candidateSpeeds;
	std::vector<double> hmdSpeeds;
	std::vector<Eigen::Matrix3d> prevRot; // [0] = HMD, [i+1] = candidates[i]
	bool havePrev = false;
	double prevTime = 0;

	void Clear()
	{
		candidates.clear();
		candidateSpeeds.clear();
		hmdSpeeds.clear();
		prevRot.clear();
		havePrev = false;
		prevTime = 0;
	}
};

static DetectionState Detection;

static std::string GetDeviceSerial(uint32_t id)
{
	char serial[vr::k_unMaxPropertyStringSize] = {};
	vr::VRSystem()->GetStringTrackedDeviceProperty(id, vr::Prop_SerialNumber_String, serial, vr::k_unMaxPropertyStringSize);
	return std::string(serial);
}

static std::string GetDeviceTrackingSystem(uint32_t id)
{
	char system[vr::k_unMaxPropertyStringSize] = {};
	vr::VRSystem()->GetStringTrackedDeviceProperty(id, vr::Prop_TrackingSystemName_String, system, vr::k_unMaxPropertyStringSize);
	return std::string(system);
}

static std::string GetDeviceModelNumber(uint32_t id)
{
	char model[vr::k_unMaxPropertyStringSize] = {};
	vr::VRSystem()->GetStringTrackedDeviceProperty(id, vr::Prop_ModelNumber_String, model, vr::k_unMaxPropertyStringSize);
	return std::string(model);
}

struct ModelScaleEntry
{
	const char *pattern;
	double scale;
};

static const ModelScaleEntry ModelScales[] = {
	{ "tundra tracker",           0.9969 	},  	// Tundra Tracker
	{ "vive tracker 3.0 mv",      1.0034 	},  	// HTC Vive Tracker 3.0
	{ "vive tracker mv",          1.00585 	}, 		// HTC Vive Tracker 1.0 / 2018
};

static double GetLighthouseModelScale(uint32_t id)
{
	if (id == vr::k_unTrackedDeviceIndexInvalid)
		return 0.0;

	std::string model = GetDeviceModelNumber(id);
	std::transform(model.begin(), model.end(), model.begin(),
		[](unsigned char c) { return (char)std::tolower(c); });

	for (auto &entry : ModelScales)
	{
		if (model.find(entry.pattern) != std::string::npos)
			return entry.scale;
	}

	return 0.0;
}

static double AngularSpeedBetween(const Eigen::Matrix3d &cur, const Eigen::Matrix3d &prev, double dt)
{
	Eigen::Matrix3d delta = cur * prev.transpose();
	double c = (delta(0,0) + delta(1,1) + delta(2,2) - 1.0) / 2.0;
	if (c > 1.0) c = 1.0;
	if (c < -1.0) c = -1.0;
	return acos(c) / dt;
}

static double PearsonCorrelation(const std::vector<double> &a, const std::vector<double> &b)
{
	if (a.size() != b.size() || a.empty())
		return 0.0;

	double meanA = 0, meanB = 0;
	for (size_t i = 0; i < a.size(); i++) { meanA += a[i]; meanB += b[i]; }
	meanA /= a.size();
	meanB /= b.size();

	double cov = 0, varA = 0, varB = 0;
	for (size_t i = 0; i < a.size(); i++)
	{
		double da = a[i] - meanA, db = b[i] - meanB;
		cov += da * db;
		varA += da * da;
		varB += db * db;
	}

	if (varA < 1e-9 || varB < 1e-9)
		return 0.0;

	return cov / std::sqrt(varA * varB);
}

DSample DeltaRotationSamples(Sample s1, Sample s2)
{
	// Difference in rotation between samples.
	auto dref = s1.ref.rot * s2.ref.rot.transpose();
	auto dtarget = s1.target.rot * s2.target.rot.transpose();

	// When stuck together, the two tracked objects rotate as a pair,
	// therefore their axes of rotation must be equal between any given pair of samples.
	DSample ds;
	ds.ref = RotationVector(dref);
	ds.target = RotationVector(dtarget);

	// Reject samples that were too close to each other.
	auto refA = AngleFromRotationMatrix3(dref);
	auto targetA = AngleFromRotationMatrix3(dtarget);
	ds.valid = refA > 0.4 && targetA > 0.4 && ds.ref.norm() > 0.01 && ds.target.norm() > 0.01;

	return ds;
}

Eigen::Vector3d CalibrateRotation(const std::vector<Sample>& samples)
{
	std::vector<DSample> deltas;

	for (size_t i = 0; i < samples.size(); i++)
	{
		for (size_t j = 0; j < i; j++)
		{
			auto delta = DeltaRotationSamples(samples[i], samples[j]);
			if (delta.valid)
				deltas.push_back(delta);
		}
	}
	char buf[256];
	snprintf(buf, sizeof buf, "Got %zd samples with %zd delta samples\n", samples.size(), deltas.size());
	CalCtx.Log(buf);
	Eigen::MatrixXd refPoints(deltas.size(), 3), targetPoints(deltas.size(), 3);

	for (size_t i = 0; i < deltas.size(); i++)
	{
		refPoints.row(i) = deltas[i].ref;
		targetPoints.row(i) = deltas[i].target;
	}

	auto crossCV = refPoints.transpose() * targetPoints;

	Eigen::BDCSVD<Eigen::MatrixXd> bdcsvd;
	auto svd = bdcsvd.compute(crossCV, Eigen::ComputeThinU | Eigen::ComputeThinV);

	Eigen::Matrix3d i = Eigen::Matrix3d::Identity();
	if ((svd.matrixU() * svd.matrixV().transpose()).determinant() < 0)
	{
		i(2, 2) = -1;
	}

	Eigen::Matrix3d rot = svd.matrixV() * i * svd.matrixU().transpose();
	rot.transposeInPlace();

	Eigen::Vector3d euler = rot.eulerAngles(2, 1, 0) * 180.0 / EIGEN_PI;

	snprintf(buf, sizeof buf, "Calibrated rotation: yaw=%.2f pitch=%.2f roll=%.2f\n", euler[1], euler[2], euler[0]);
	CalCtx.Log(buf);
	return euler;
}

static const double ScaleSpreadThreshold = 0.5;
static const double MinCalibratedScale = 0.97;
static const double MaxCalibratedScale = 1.03;

static Eigen::Vector3d SolveTranslation(const std::vector<Sample>& samples, const Eigen::Matrix3d& rotation, double scale)
{
	std::vector<std::pair<Eigen::Vector3d, Eigen::Matrix3d>> deltas;

	for (size_t i = 0; i < samples.size(); i++)
	{
		Sample s_i = samples[i];
		s_i.target.rot = rotation * s_i.target.rot;
		s_i.target.trans = scale * (rotation * s_i.target.trans);

		for (size_t j = 0; j < i; j++)
		{
			Sample s_j = samples[j];
			s_j.target.rot = rotation * s_j.target.rot;
			s_j.target.trans = scale * (rotation * s_j.target.trans);

			auto QAi = s_i.ref.rot.transpose();
			auto QAj = s_j.ref.rot.transpose();
			auto dQA = QAj - QAi;
			auto CA = QAj * (s_j.ref.trans - s_j.target.trans) - QAi * (s_i.ref.trans - s_i.target.trans);
			deltas.push_back(std::make_pair(CA, dQA));

			auto QBi = s_i.target.rot.transpose();
			auto QBj = s_j.target.rot.transpose();
			auto dQB = QBj - QBi;
			auto CB = QBj * (s_j.ref.trans - s_j.target.trans) - QBi * (s_i.ref.trans - s_i.target.trans);
			deltas.push_back(std::make_pair(CB, dQB));
		}
	}

	Eigen::VectorXd constants(deltas.size() * 3);
	Eigen::MatrixXd coefficients(deltas.size() * 3, 3);

	for (size_t i = 0; i < deltas.size(); i++)
	{
		for (int axis = 0; axis < 3; axis++)
		{
			constants(i * 3 + axis) = deltas[i].first(axis);
			coefficients.row(i * 3 + axis) = deltas[i].second.row(axis);
		}
	}

	return coefficients.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(constants);
}

Eigen::Vector3d CalibrateTranslation(const std::vector<Sample>& samples, const Eigen::Matrix3d& rotation, double scale)
{
	Eigen::Vector3d trans = SolveTranslation(samples, rotation, scale);
	auto transcm = trans * 100.0;

	char buf[256];
	snprintf(buf, sizeof buf, "Calibrated translation x=%.2f y=%.2f z=%.2f\n", transcm[0], transcm[1], transcm[2]);
	CalCtx.Log(buf);
	return transcm;
}

static Eigen::Matrix3d ExpRotation(const Eigen::Vector3d& v)
{
	double a = v.norm();
	if (a < 1e-12)
		return Eigen::Matrix3d::Identity();
	return Eigen::AngleAxisd(a, v / a).toRotationMatrix();
}

static void TargetRates(const std::vector<Sample>& samples, std::vector<Eigen::Vector3d>& omega, std::vector<Eigen::Vector3d>& velocity)
{
	omega.assign(samples.size(), Eigen::Vector3d::Zero());
	velocity.assign(samples.size(), Eigen::Vector3d::Zero());
	for (size_t i = 0; i < samples.size(); i++)
	{
		size_t a = i > 0 ? i - 1 : i, b = i + 1 < samples.size() ? i + 1 : i;
		double dt = samples[b].time - samples[a].time;
		if (dt <= 1e-4 || dt > 0.3)
			continue;
		omega[i] = RotationVector(samples[b].target.rot * samples[a].target.rot.transpose()) / dt;
		velocity[i] = (samples[b].target.trans - samples[a].target.trans) / dt;
	}
}

static std::vector<Sample> ShiftTargets(const std::vector<Sample>& samples, const std::vector<Eigen::Vector3d>& omega, const std::vector<Eigen::Vector3d>& velocity, double tauRot, double tauPos)
{
	std::vector<Sample> out = samples;
	for (size_t i = 0; i < out.size(); i++)
	{
		out[i].target.rot = ExpRotation(omega[i] * tauRot) * out[i].target.rot;
		out[i].target.trans += velocity[i] * tauPos;
	}
	return out;
}

static bool RotationFit(const std::vector<Sample>& samples, Eigen::Matrix3d& rot, double& residual)
{
	std::vector<DSample> deltas;
	for (size_t i = 0; i < samples.size(); i++)
		for (size_t j = 0; j < i; j++)
		{
			auto delta = DeltaRotationSamples(samples[i], samples[j]);
			if (delta.valid)
				deltas.push_back(delta);
		}
	if (deltas.size() < 10)
		return false;

	Eigen::Matrix3d cross = Eigen::Matrix3d::Zero();
	for (auto& d : deltas)
		cross += d.ref * d.target.transpose();
	Eigen::JacobiSVD<Eigen::Matrix3d> svd(cross, Eigen::ComputeFullU | Eigen::ComputeFullV);
	Eigen::Matrix3d i = Eigen::Matrix3d::Identity();
	if ((svd.matrixU() * svd.matrixV().transpose()).determinant() < 0)
		i(2, 2) = -1;
	rot = (svd.matrixV() * i * svd.matrixU().transpose()).transpose();

	double sum = 0.0;
	for (auto& d : deltas)
		sum += (d.ref - rot * d.target).squaredNorm();
	residual = sum / deltas.size();
	return true;
}

static double TranslationFitResidual(const std::vector<Sample>& samples, const Eigen::Matrix3d& rotation)
{
	Eigen::Matrix3d AtA = Eigen::Matrix3d::Zero();
	Eigen::Vector3d Atb = Eigen::Vector3d::Zero();
	for (size_t i = 0; i < samples.size(); i++)
	{
		Eigen::Matrix3d QAi = samples[i].ref.rot.transpose();
		Eigen::Matrix3d QBi = (rotation * samples[i].target.rot).transpose();
		Eigen::Vector3d Di = samples[i].ref.trans - rotation * samples[i].target.trans;
		for (size_t j = 0; j < i; j++)
		{
			Eigen::Matrix3d QAj = samples[j].ref.rot.transpose();
			Eigen::Matrix3d QBj = (rotation * samples[j].target.rot).transpose();
			Eigen::Vector3d Dj = samples[j].ref.trans - rotation * samples[j].target.trans;
			Eigen::Matrix3d dQA = QAj - QAi, dQB = QBj - QBi;
			Eigen::Vector3d CA = QAj * Dj - QAi * Di, CB = QBj * Dj - QBi * Di;
			AtA += dQA.transpose() * dQA + dQB.transpose() * dQB;
			Atb += dQA.transpose() * CA + dQB.transpose() * CB;
		}
	}
	Eigen::Vector3d tc = AtA.fullPivLu().solve(Atb);

	Eigen::Vector3d mount = Eigen::Vector3d::Zero();
	for (auto& s : samples)
	{
		Eigen::Matrix3d trackerRot = rotation * s.target.rot;
		mount += trackerRot.transpose() * (s.ref.trans - (rotation * s.target.trans + tc));
	}
	mount /= (double)samples.size();

	double sum = 0.0;
	for (auto& s : samples)
		sum += (rotation * s.target.trans + tc + rotation * s.target.rot * mount - s.ref.trans).squaredNorm();
	return sum / samples.size();
}

static bool GridMinimum(const std::vector<double>& grid, const std::vector<double>& values, double& best)
{
	size_t k = std::min_element(values.begin(), values.end()) - values.begin();
	if (k == 0 || k + 1 >= values.size())
		return false;
	best = grid[k];
	double a = values[k - 1], b = values[k], c = values[k + 1];
	double den = a - 2.0 * b + c;
	if (den > 0.0)
		best += 0.5 * (a - c) / den * (grid[k + 1] - grid[k]);
	return true;
}

static void EstimateTimeOffsets(const std::vector<Sample>& samples, double& tauRot, double& tauPos)
{
	tauRot = 0.0;
	tauPos = 0.0;

	std::vector<Eigen::Vector3d> omegaAll, velocityAll;
	TargetRates(samples, omegaAll, velocityAll);

	size_t stride = samples.size() > 240 ? (samples.size() + 239) / 240 : 1;
	std::vector<Sample> sub;
	std::vector<Eigen::Vector3d> omega, velocity;
	for (size_t i = 0; i < samples.size(); i += stride)
	{
		sub.push_back(samples[i]);
		omega.push_back(omegaAll[i]);
		velocity.push_back(velocityAll[i]);
	}

	std::vector<double> grid, rotResidual;
	for (double tau = -0.05; tau <= 0.15001; tau += 0.004)
	{
		Eigen::Matrix3d rot;
		double r;
		if (!RotationFit(ShiftTargets(sub, omega, velocity, tau, 0.0), rot, r))
			return;
		grid.push_back(tau);
		rotResidual.push_back(r);
	}

	double bestRot;
	if (!GridMinimum(grid, rotResidual, bestRot))
	{
		CalCtx.Log("Time offset between headset and tracker not found, calibrating without it\n");
		return;
	}

	Eigen::Matrix3d rot;
	double rotAtBest = 0.0, rotAtZero = 0.0;
	RotationFit(sub, rot, rotAtZero);
	RotationFit(ShiftTargets(sub, omega, velocity, bestRot, 0.0), rot, rotAtBest);

	std::vector<double> posResidual;
	for (double tau : grid)
		posResidual.push_back(TranslationFitResidual(ShiftTargets(sub, omega, velocity, bestRot, tau), rot));

	double bestPos = 0.0;
	bool havePos = GridMinimum(grid, posResidual, bestPos);

	tauRot = bestRot;
	tauPos = havePos ? bestPos : 0.0;

	char buf[256];
	snprintf(buf, sizeof buf, "Time offset between headset and tracker: rotation %.1f ms, position %.1f ms (rotation fit %.3f -> %.3f deg rms)\n",
		tauRot * 1000.0, tauPos * 1000.0, std::sqrt(rotAtZero) * 180.0 / EIGEN_PI, std::sqrt(rotAtBest) * 180.0 / EIGEN_PI);
	CalCtx.Log(buf);
}

static double EstimateHmdSpaceScale(const std::vector<Sample> &samples, const Eigen::Matrix3d &rotation, double targetModelScale)
{
	Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
	for (auto &sample : samples)
		centroid += rotation * sample.target.trans;
	centroid /= (double)samples.size();

	double spread = 0;
	for (auto &sample : samples)
		spread += (rotation * sample.target.trans - centroid).squaredNorm();
	spread = std::sqrt(spread / (double)samples.size());

	char buf[256];
	if (spread < ScaleSpreadThreshold)
	{
		snprintf(buf, sizeof buf, "Headset scale not measured (movement spread %.2f m, needs %.1f m of walking)\n", spread, ScaleSpreadThreshold);
		CalCtx.Log(buf);
		return 0.0;
	}

	Eigen::MatrixXd coefficients(samples.size() * 3, 7);
	Eigen::VectorXd constants(samples.size() * 3);

	for (size_t i = 0; i < samples.size(); i++)
	{
		Eigen::Vector3d rotatedPos = rotation * samples[i].target.trans;
		Eigen::Matrix3d rotatedRot = rotation * samples[i].target.rot;

		coefficients.block<3, 1>(i * 3, 0) = rotatedPos;
		coefficients.block<3, 3>(i * 3, 1) = Eigen::Matrix3d::Identity();
		coefficients.block<3, 3>(i * 3, 4) = rotatedRot;
		constants.segment<3>(i * 3) = samples[i].ref.trans;
	}

	Eigen::VectorXd result = coefficients.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(constants);
	double fittedScale = result(0);

	if (fittedScale < MinCalibratedScale || fittedScale > MaxCalibratedScale)
	{
		snprintf(buf, sizeof buf, "Fitted headset scale %.5f is outside %.2f..%.2f, not used\n", fittedScale, MinCalibratedScale, MaxCalibratedScale);
		CalCtx.Log(buf);
		return 0.0;
	}

	snprintf(buf, sizeof buf, "Fitted headset space scale relative to lighthouse: %.5f (%+.2f%%) from %.2f m of movement, implied absolute headset scale: %.5f\n",
		fittedScale, (fittedScale - 1.0) * 100.0, spread, fittedScale / targetModelScale);
	CalCtx.Log(buf);
	return fittedScale;
}

static const double AxisVarianceThreshold = 0.0005;

static double SecondAxisVariance(const std::vector<Sample> &samples)
{
	std::vector<Eigen::Vector4d> points;
	points.reserve(samples.size());
	Eigen::Vector4d mean = Eigen::Vector4d::Zero();

	for (auto &sample : samples)
	{
		Eigen::Quaterniond q(sample.target.rot);
		if (q.w() < 0)
			q.coeffs() = -q.coeffs();

		Eigen::Vector4d point(q.w(), q.x(), q.y(), q.z());
		mean += point;
		points.push_back(point);
	}

	if (points.empty())
		return 0.0;

	mean /= (double)points.size();

	Eigen::Matrix4d cov = Eigen::Matrix4d::Zero();
	for (auto &point : points)
	{
		Eigen::Vector4d d = point - mean;
		cov += d * d.transpose();
	}
	cov /= (double)points.size();

	Eigen::SelfAdjointEigenSolver<Eigen::Matrix4d> solver(cov);
	return solver.eigenvalues()(1);
}

static Eigen::Vector3d ComputeRefToTargetOffset(const std::vector<Sample> &samples, const Eigen::Matrix3d &calRot, const Eigen::Vector3d &calTrans, double calScale)
{
	Eigen::Vector3d accum = Eigen::Vector3d::Zero();

	for (auto &sample : samples)
		accum += sample.ref.rot.transpose() * (calScale * (calRot * sample.target.trans) + calTrans - sample.ref.trans);

	return accum / (double)samples.size();
}

static double RetargetingErrorRMS(const std::vector<Sample> &samples, const Eigen::Vector3d &hmdToTargetPos, const Eigen::Matrix3d &calRot, const Eigen::Vector3d &calTrans, double calScale)
{
	double accum = 0;

	for (auto &sample : samples)
		accum += (calScale * (calRot * sample.target.trans) + calTrans - (sample.ref.rot * hmdToTargetPos + sample.ref.trans)).squaredNorm();

	return std::sqrt(accum / (double)samples.size());
}

Sample CollectSample(const CalibrationContext &ctx, const char *&problem)
{
	const vr::TrackedDevicePose_t &reference = ctx.devicePoses[0];
	const vr::TrackedDevicePose_t &target = ctx.devicePoses[ctx.targetID];

	problem = nullptr;
	if (!reference.bPoseIsValid || reference.eTrackingResult != vr::TrackingResult_Running_OK)
		problem = "The headset is not tracking";
	else if (!target.bPoseIsValid || target.eTrackingResult != vr::TrackingResult_Running_OK)
		problem = "The device on your head is not tracking (sensors covered?)";
	if (problem)
		return Sample();

	return Sample(
		Pose(reference.mDeviceToAbsoluteTracking),
		Pose(target.mDeviceToAbsoluteTracking)
	);
}

vr::HmdQuaternion_t VRRotationQuat(Eigen::Vector3d eulerdeg)
{
	auto euler = eulerdeg * EIGEN_PI / 180.0;

	Eigen::Quaterniond rotQuat =
		Eigen::AngleAxisd(euler(0), Eigen::Vector3d::UnitZ()) *
		Eigen::AngleAxisd(euler(1), Eigen::Vector3d::UnitY()) *
		Eigen::AngleAxisd(euler(2), Eigen::Vector3d::UnitX());

	vr::HmdQuaternion_t vrRotQuat;
	vrRotQuat.x = rotQuat.coeffs()[0];
	vrRotQuat.y = rotQuat.coeffs()[1];
	vrRotQuat.z = rotQuat.coeffs()[2];
	vrRotQuat.w = rotQuat.coeffs()[3];
	return vrRotQuat;
}

vr::HmdVector3d_t VRTranslationVec(Eigen::Vector3d transcm)
{
	auto trans = transcm * 0.01;
	vr::HmdVector3d_t vrTrans;
	vrTrans.v[0] = trans[0];
	vrTrans.v[1] = trans[1];
	vrTrans.v[2] = trans[2];
	return vrTrans;
}

void ResetAndDisableOffsets(uint32_t id)
{
	vr::HmdVector3d_t zeroV;
	zeroV.v[0] = zeroV.v[1] = zeroV.v[2] = 0;

	vr::HmdQuaternion_t zeroQ;
	zeroQ.x = 0; zeroQ.y = 0; zeroQ.z = 0; zeroQ.w = 1;

	protocol::Request req(protocol::RequestSetDeviceTransform);
	req.setDeviceTransform = { id, false, zeroV, zeroQ, 1.0 };
	Driver.SendBlocking(req);
}

void SendOneEuroParams()
{
	protocol::Request req(protocol::RequestSetOneEuro);
	req.setOneEuro.headEnabled = CalCtx.headFilterEnabled;
	req.setOneEuro.head = CalCtx.headFilterParams;
	req.setOneEuro.drift = CalCtx.driftFilterParams;
	req.setOneEuro.deviceSmoothing = CalCtx.lighthouseSmoothing;
	req.setOneEuro.latencyCompensation = CalCtx.latencyCompensation;

	try
	{
		Driver.SendBlocking(req);
	}
	catch (const std::runtime_error &e)
	{
		std::cerr << "Failed to send One Euro params: " << e.what() << std::endl;
	}
}

void SendUniverseLock()
{
	protocol::Request req(protocol::RequestSetUniverseLock);
	req.setUniverseLock.enabled = CalCtx.lockBaseStations;
	req.setUniverseLock.calibrating = CalCtx.state != CalibrationState::None;
	req.setUniverseLock.command = CalCtx.lockCommitPending && CalCtx.state == CalibrationState::None ? 1u : 0u;

	try
	{
		Driver.SendBlocking(req);
		if (req.setUniverseLock.command == 1u)
			CalCtx.lockCommitPending = false;
	}
	catch (const std::runtime_error &e)
	{
		std::cerr << "Failed to send base station lock: " << e.what() << std::endl;
	}
}

static Eigen::Matrix3d CalibrationMatrix(const CalibrationContext &ctx)
{
	Eigen::Vector3d e = ctx.calibratedRotation * EIGEN_PI / 180.0;
	return (Eigen::AngleAxisd(e(0), Eigen::Vector3d::UnitZ()) *
		Eigen::AngleAxisd(e(1), Eigen::Vector3d::UnitY()) *
		Eigen::AngleAxisd(e(2), Eigen::Vector3d::UnitX())).toRotationMatrix();
}

void KeepCalibrationSpotAfterRotationEdit(const Eigen::Vector3d &previousRotation)
{
	auto &ctx = CalCtx;
	if (!ctx.calibrationSpotValid)
		return;
	Eigen::Vector3d e = previousRotation * EIGEN_PI / 180.0;
	Eigen::Matrix3d before = (Eigen::AngleAxisd(e(0), Eigen::Vector3d::UnitZ()) *
		Eigen::AngleAxisd(e(1), Eigen::Vector3d::UnitY()) *
		Eigen::AngleAxisd(e(2), Eigen::Vector3d::UnitX())).toRotationMatrix();
	Eigen::Matrix3d after = CalibrationMatrix(ctx);
	ctx.calibratedTranslation += ctx.calibratedScale * ((before - after) * ctx.calibrationSpot) * 100.0;
}

static std::string LighthouseFingerprint(const CalibrationContext &ctx)
{
	uint64_t universe = 0;
	if (ctx.targetID < vr::k_unMaxTrackedDeviceCount)
	{
		vr::ETrackedPropertyError err = vr::TrackedProp_Success;
		universe = vr::VRSystem()->GetUint64TrackedDeviceProperty(ctx.targetID, vr::Prop_CurrentUniverseId_Uint64, &err);
		if (err != vr::TrackedProp_Success)
			universe = 0;
	}

	std::vector<std::pair<std::string, Eigen::Vector3d>> stations;
	for (uint32_t id = 0; id < vr::k_unMaxTrackedDeviceCount; ++id)
	{
		if (vr::VRSystem()->GetTrackedDeviceClass(id) != vr::TrackedDeviceClass_TrackingReference)
			continue;
		if (!ctx.devicePoses[id].bPoseIsValid || GetDeviceTrackingSystem(id) != ctx.targetTrackingSystem)
			continue;
		stations.push_back({ GetDeviceSerial(id), Pose(ctx.devicePoses[id].mDeviceToAbsoluteTracking).trans });
	}
	std::sort(stations.begin(), stations.end(), [](const auto &a, const auto &b) { return a.first < b.first; });

	std::ostringstream out;
	out << universe << "|";
	for (size_t i = 0; i < stations.size(); i++)
		out << (i ? "," : "") << stations[i].first;
	out << "|";
	bool first = true;
	for (size_t i = 0; i < stations.size(); i++)
		for (size_t j = i + 1; j < stations.size(); j++)
		{
			out << (first ? "" : ",") << (long long)std::llround((stations[i].second - stations[j].second).norm() * 1000.0);
			first = false;
		}
	return out.str();
}

static std::vector<std::string> SplitString(const std::string &s, char sep)
{
	std::vector<std::string> parts;
	std::string cur;
	for (char c : s)
	{
		if (c == sep) { parts.push_back(cur); cur.clear(); }
		else cur += c;
	}
	parts.push_back(cur);
	return parts;
}

static bool ParseFingerprint(const std::string &fp, std::string &universe, std::map<std::pair<std::string, std::string>, long long> &distances, std::vector<std::string> &serials)
{
	auto parts = SplitString(fp, '|');
	if (parts.size() != 3)
		return false;
	universe = parts[0];
	serials.clear();
	if (!parts[1].empty())
		serials = SplitString(parts[1], ',');
	std::vector<std::string> d;
	if (!parts[2].empty())
		d = SplitString(parts[2], ',');
	size_t k = 0;
	for (size_t i = 0; i < serials.size(); i++)
		for (size_t j = i + 1; j < serials.size(); j++)
		{
			if (k >= d.size())
				return false;
			distances[{ serials[i], serials[j] }] = std::atoll(d[k++].c_str());
		}
	return true;
}

static bool FingerprintsMatch(const std::string &stored, const std::string &current)
{
	std::string uA, uB;
	std::map<std::pair<std::string, std::string>, long long> dA, dB;
	std::vector<std::string> sA, sB;
	if (!ParseFingerprint(stored, uA, dA, sA) || !ParseFingerprint(current, uB, dB, sB))
		return false;
	if (uA != "0" && uB != "0" && uA != uB)
		return false;
	for (auto &s : sB)
		if (std::find(sA.begin(), sA.end(), s) == sA.end())
			return false;
	for (auto &entry : dB)
	{
		auto it = dA.find(entry.first);
		if (it != dA.end() && std::llabs(it->second - entry.second) > 30)
			return false;
	}
	return true;
}

static bool ComputeTiltSeed(const CalibrationContext &ctx, vr::HmdVector3d_t &seed)
{
	if (ctx.tiltHistory.empty() || !ctx.followSlamHmd || ctx.noHeadTracker || !ctx.validProfile)
		return false;
	if (!ctx.tiltFingerprint.empty() && !FingerprintsMatch(ctx.tiltFingerprint, LighthouseFingerprint(ctx)))
		return false;

	Eigen::Matrix3d C = CalibrationMatrix(ctx);
	const Eigen::Vector3d up(0.0, 1.0, 0.0);
	std::vector<Eigen::Vector3d> tilts;
	for (auto &g : ctx.tiltHistory)
	{
		Eigen::Vector3d u = (C * g).normalized();
		Eigen::Vector3d axis = up.cross(u);
		double s = axis.norm();
		double angle = std::atan2(s, up.dot(u));
		tilts.push_back(s > 1e-12 ? Eigen::Vector3d(axis / s * angle) : Eigen::Vector3d::Zero());
	}

	Eigen::Vector3d mean = Eigen::Vector3d::Zero();
	for (auto &t : tilts)
		mean += t;
	double n = (double)tilts.size();
	mean /= n;
	double spread;
	if (tilts.size() > 1)
	{
		double sum = 0.0;
		for (auto &t : tilts)
			sum += (t - mean).squaredNorm();
		spread = sum / (n - 1.0);
	}
	else
	{
		double prior = 0.3 * EIGEN_PI / 180.0;
		spread = prior * prior;
	}
	double m2 = mean.squaredNorm();
	double w = m2 > 0.0 ? m2 / (m2 + spread / n + spread) : 0.0;
	Eigen::Vector3d result = mean * w;
	seed.v[0] = result.x();
	seed.v[1] = 0.0;
	seed.v[2] = result.z();
	return true;
}

static Eigen::Matrix3d YawRotation(double angle)
{
	return Eigen::AngleAxisd(angle, Eigen::Vector3d::UnitY()).toRotationMatrix();
}

static double YawBetween(const Eigen::Matrix3d &rot, const Eigen::Matrix3d &base)
{
	Eigen::Matrix3d y = rot * base.transpose();
	return std::atan2(y(0, 2) - y(2, 0), y(0, 0) + y(2, 2));
}

static Eigen::Matrix3d ShortestArc(const Eigen::Vector3d &from, const Eigen::Vector3d &to)
{
	Eigen::Vector3d a = from.normalized(), b = to.normalized();
	Eigen::Vector3d axis = a.cross(b);
	double s = axis.norm(), c = a.dot(b);
	if (s < 1e-15)
		return Eigen::Matrix3d::Identity();
	return Eigen::AngleAxisd(std::atan2(s, c), axis / s).toRotationMatrix();
}

static double AngleBetween(const Eigen::Vector3d &a, const Eigen::Vector3d &b)
{
	return std::atan2(a.cross(b).norm(), a.dot(b));
}

static std::vector<DSample> RotationPairs(const std::vector<Sample> &samples)
{
	std::vector<DSample> pairs;
	for (size_t i = 0; i < samples.size(); i++)
		for (size_t j = 0; j < i; j++)
		{
			auto delta = DeltaRotationSamples(samples[i], samples[j]);
			if (delta.valid)
				pairs.push_back(delta);
		}
	return pairs;
}

static Eigen::Matrix3d YawFromRotationAxes(const std::vector<DSample> &pairs, const Eigen::Matrix3d &level)
{
	double a = 0.0, b = 0.0;
	for (auto &p : pairs)
	{
		Eigen::Vector3d v = level * p.target;
		a += p.ref.x() * v.x() + p.ref.z() * v.z();
		b += p.ref.x() * v.z() - p.ref.z() * v.x();
	}
	return YawRotation(std::atan2(b, a)) * level;
}

static Eigen::Matrix3d YawFromPositions(const std::vector<Sample> &samples, const Eigen::Matrix3d &level)
{
	Eigen::Matrix<double, 8, 8> AtA = Eigen::Matrix<double, 8, 8>::Zero();
	Eigen::Matrix<double, 8, 1> Atb = Eigen::Matrix<double, 8, 1>::Zero();
	for (auto &s : samples)
	{
		Eigen::Vector3d v = level * s.target.trans, p = s.ref.trans;
		const Eigen::Matrix3d &q = s.ref.rot;
		Eigen::Matrix<double, 3, 8> A = Eigen::Matrix<double, 3, 8>::Zero();
		Eigen::Vector3d b;
		A(0, 0) = v.x(); A(0, 1) = v.z(); A(0, 2) = 1.0; A.block<1, 3>(0, 5) = -q.row(0); b(0) = p.x();
		A(1, 3) = 1.0; A.block<1, 3>(1, 5) = -q.row(1); b(1) = p.y() - v.y();
		A(2, 0) = v.z(); A(2, 1) = -v.x(); A(2, 4) = 1.0; A.block<1, 3>(2, 5) = -q.row(2); b(2) = p.z();
		AtA += A.transpose() * A;
		Atb += A.transpose() * b;
	}
	Eigen::Matrix<double, 8, 1> x = AtA.fullPivLu().solve(Atb);
	return YawRotation(std::atan2(x(1), x(0))) * level;
}

static Eigen::Matrix3d FuseYaw(const std::vector<DSample> &pairs, const std::vector<Sample> &samples, const Eigen::Matrix3d &fromAxes, const Eigen::Matrix3d &fromPositions, const Eigen::Matrix3d &level, double &axesWeight)
{
	double er = 0.0, sr = 0.0;
	for (auto &p : pairs)
	{
		Eigen::Vector3d v = fromAxes * p.target;
		er += (p.ref - v).squaredNorm();
		sr += v.x() * v.x() + v.z() * v.z();
	}
	double varAxes = er / (std::max)(sr, 1e-12);

	Eigen::Matrix<double, 6, 6> AtA = Eigen::Matrix<double, 6, 6>::Zero();
	Eigen::Matrix<double, 6, 1> Atb = Eigen::Matrix<double, 6, 1>::Zero();
	Eigen::Vector3d center = Eigen::Vector3d::Zero();
	for (auto &s : samples)
	{
		Eigen::Matrix<double, 3, 6> A;
		A.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
		A.block<3, 3>(0, 3) = -s.ref.rot;
		Eigen::Vector3d b = s.ref.trans - fromPositions * s.target.trans;
		AtA += A.transpose() * A;
		Atb += A.transpose() * b;
		center += fromPositions * s.target.trans;
	}
	Eigen::Matrix<double, 6, 1> td = AtA.fullPivLu().solve(Atb);
	center /= (double)samples.size();
	double ep = 0.0, sp = 0.0;
	for (auto &s : samples)
	{
		Eigen::Vector3d v = fromPositions * s.target.trans;
		ep += (s.ref.trans - (v + td.head<3>() - s.ref.rot * td.tail<3>())).squaredNorm();
		Eigen::Vector3d d = v - center;
		sp += d.x() * d.x() + d.z() * d.z();
	}
	double varPositions = ep / (std::max)(sp, 1e-12);

	axesWeight = varAxes + varPositions > 0.0 ? varPositions / (varAxes + varPositions) : 0.5;
	double ya = YawBetween(fromAxes, level), yp = YawBetween(fromPositions, level);
	return YawRotation(ya + (1.0 - axesWeight) * std::remainder(yp - ya, 2.0 * EIGEN_PI)) * level;
}

static bool SavedWorldUp(const CalibrationContext &ctx, Eigen::Vector3d &up, double &sigma)
{
	up = Eigen::Vector3d::UnitY();
	sigma = 0.4 * EIGEN_PI / 180.0;
	if (ctx.tiltHistory.empty())
		return false;
	if (!ctx.tiltFingerprint.empty() && !FingerprintsMatch(ctx.tiltFingerprint, LighthouseFingerprint(ctx)))
		return false;

	Eigen::Vector3d mean = Eigen::Vector3d::Zero();
	for (auto &g : ctx.tiltHistory)
		mean += g.normalized();
	if (!(mean.norm() > 1e-9))
		return false;
	mean.normalize();

	double n = (double)ctx.tiltHistory.size(), spread = 0.0;
	for (auto &g : ctx.tiltHistory)
	{
		double a = AngleBetween(g.normalized(), mean);
		spread += a * a;
	}
	double minSigma = 0.2 * EIGEN_PI / 180.0;
	double var = n > 1.0 ? spread / (n - 1.0) * (1.0 + 1.0 / n) : 0.0;
	sigma = std::sqrt((std::max)(minSigma * minSigma, var));
	up = mean;
	return true;
}

static Eigen::Matrix3d LevelledRotation(const CalibrationContext &ctx, const std::vector<Sample> &samples, const Eigen::Matrix3d &fullRot)
{
	auto pairs = RotationPairs(samples);
	if (pairs.size() < 10)
		return fullRot;

	double er = 0.0, sr = 0.0;
	for (auto &p : pairs)
	{
		er += (p.ref - fullRot * p.target).squaredNorm();
		sr += p.target.squaredNorm();
	}
	double calVar = er / (std::max)(sr, 1e-12);

	Eigen::Vector3d priorUp;
	double priorSigma;
	bool saved = SavedWorldUp(ctx, priorUp, priorSigma);
	double priorVar = priorSigma * priorSigma;
	double calWeight = priorVar / (priorVar + calVar);
	Eigen::Vector3d calUp = fullRot.transpose() * Eigen::Vector3d::UnitY();
	Eigen::Vector3d up = (calWeight * calUp + (1.0 - calWeight) * priorUp).normalized();
	Eigen::Matrix3d level = ShortestArc(up, Eigen::Vector3d::UnitY());

	double axesWeight = 0.5;
	Eigen::Matrix3d rot = FuseYaw(pairs, samples, YawFromRotationAxes(pairs, level), YawFromPositions(samples, level), level, axesWeight);

	const double deg = 180.0 / EIGEN_PI;
	char buf[320];
	snprintf(buf, sizeof buf, "Levelled with %s: measured tilt %.2f deg, reference tilt %.2f deg (+-%.2f), used %.2f deg; yaw %.0f%% from head turns, %.0f%% from head movement\n",
		saved ? "the saved world tilt" : "lighthouse gravity",
		AngleBetween(calUp, Eigen::Vector3d::UnitY()) * deg, AngleBetween(priorUp, Eigen::Vector3d::UnitY()) * deg, priorSigma * deg,
		AngleBetween(up, Eigen::Vector3d::UnitY()) * deg, axesWeight * 100.0, (1.0 - axesWeight) * 100.0);
	CalCtx.Log(buf);
	return rot;
}

static void RecordTilt(CalibrationContext &ctx, const protocol::DriverStatus &st)
{
	if (st.tiltSteps < ctx.tiltStepsRecorded)
	{
		ctx.tiltStepsRecorded = 0;
		ctx.tiltSessionEntry = -1;
	}
	if (!st.slamUpValid || st.tiltSteps <= ctx.tiltStepsRecorded || !ctx.enabled || !ctx.validProfile || !ctx.followSlamHmd || ctx.noHeadTracker)
		return;

	Eigen::Vector3d g(st.slamUpInLighthouse.v[0], st.slamUpInLighthouse.v[1], st.slamUpInLighthouse.v[2]);
	if (!(g.norm() > 0.5))
		return;
	g.normalize();

	std::string fingerprint = LighthouseFingerprint(ctx);
	if (!ctx.tiltFingerprint.empty() && !FingerprintsMatch(ctx.tiltFingerprint, fingerprint))
	{
		ctx.tiltHistory.clear();
		ctx.tiltSessionEntry = -1;
		ctx.Log("Lighthouse setup changed, stored world tilt history cleared\n");
	}

	if (ctx.tiltSessionEntry < 0 || ctx.tiltSessionEntry >= (int)ctx.tiltHistory.size())
	{
		ctx.tiltHistory.push_back(g);
		if (ctx.tiltHistory.size() > 10)
			ctx.tiltHistory.erase(ctx.tiltHistory.begin());
		ctx.tiltSessionEntry = (int)ctx.tiltHistory.size() - 1;
	}
	else
		ctx.tiltHistory[ctx.tiltSessionEntry] = g;

	ctx.tiltFingerprint = fingerprint;
	ctx.tiltStepsRecorded = st.tiltSteps;
	ctx.refinementDirty = true;
}

static uint32_t FindBodyTracker()
{
	uint32_t chest = vr::k_unTrackedDeviceIndexInvalid;
	char buffer[vr::k_unMaxPropertyStringSize];
	for (uint32_t id = 0; id < vr::k_unMaxTrackedDeviceCount; ++id)
	{
		if (vr::VRSystem()->GetTrackedDeviceClass(id) != vr::TrackedDeviceClass_GenericTracker)
			continue;
		vr::ETrackedPropertyError err = vr::TrackedProp_Success;
		vr::VRSystem()->GetStringTrackedDeviceProperty(id, vr::Prop_ControllerType_String, buffer, vr::k_unMaxPropertyStringSize, &err);
		if (err != vr::TrackedProp_Success)
			continue;
		std::string type(buffer);
		if (type == "vive_tracker_waist")
			return id;
		if (type == "vive_tracker_chest" && chest == vr::k_unTrackedDeviceIndexInvalid)
			chest = id;
	}
	return chest;
}

void SendHmdTrackerCommand(uint32_t hmdID, uint32_t trackerID, bool enabled)
{
	protocol::Request req(protocol::RequestSetHmdTracker);
	req.setHmdTracker.hmdID = hmdID;
	req.setHmdTracker.trackerID = trackerID;
	req.setHmdTracker.enabled = enabled;
	req.setHmdTracker.slamFallback = CalCtx.fallbackToSlam;
	req.setHmdTracker.predictionTime = CalCtx.predictionTime;
	req.setHmdTracker.enableAngularVelocity = CalCtx.enableAngularVelocity;
	req.setHmdTracker.offsetRotation = CalCtx.relativeRotation;
	req.setHmdTracker.offsetTranslation = CalCtx.relativeTranslation;
	req.setHmdTracker.calibrationRotation = VRRotationQuat(CalCtx.calibratedRotation);
	req.setHmdTracker.calibrationTranslation = VRTranslationVec(CalCtx.calibratedTranslation);
	req.setHmdTracker.calibrationScale = CalCtx.calibratedScale;
	req.setHmdTracker.hmdScale = CalCtx.hmdScale;
	req.setHmdTracker.followSlamHmd = CalCtx.followSlamHmd;
	// Calibration reads the tracker back through SteamVR.
	req.setHmdTracker.hideHeadTracker = CalCtx.hideHeadTracker && CalCtx.state == CalibrationState::None;
	req.setHmdTracker.tiltSeed = { 0.0, 0.0, 0.0 };
	req.setHmdTracker.tiltSeedValid = enabled && ComputeTiltSeed(CalCtx, req.setHmdTracker.tiltSeed);
	req.setHmdTracker.stayAligned = enabled && CalCtx.stayAligned && CalCtx.followSlamHmd && CalCtx.noHeadTracker && CalCtx.state == CalibrationState::None;
	req.setHmdTracker.stayHipID = req.setHmdTracker.stayAligned ? FindBodyTracker() : vr::k_unTrackedDeviceIndexInvalid;
	req.setHmdTracker.calibrationLatencyValid = CalCtx.calibrationLatencyValid;
	req.setHmdTracker.calibrationLatencyRot = CalCtx.calibrationLatencyRot;
	req.setHmdTracker.calibrationLatencyPos = CalCtx.calibrationLatencyPos;
	Driver.SendBlocking(req);
}

// https://stackoverflow.com/questions/12374087/average-of-multiple-quaternions/27410865
void ComputeRelativeOffset(CalibrationContext &ctx, const std::vector<Sample> &samples, const Eigen::Matrix3d &calRot, const Eigen::Vector3d &calTrans, double calScale)
{
	if (samples.empty())
		return;

	Eigen::Matrix4d quatAccum = Eigen::Matrix4d::Zero();
	Eigen::Vector3d transAccum = Eigen::Vector3d::Zero();

	for (auto &sample : samples)
	{
		Eigen::Matrix3d trackerRot = calRot * sample.target.rot;
		Eigen::Vector3d trackerTrans = calScale * (calRot * sample.target.trans) + calTrans;

		Eigen::Matrix3d offsetRot = trackerRot.transpose() * sample.ref.rot;
		Eigen::Vector3d offsetTrans = trackerRot.transpose() * (sample.ref.trans - trackerTrans);

		Eigen::Quaterniond q(offsetRot);
		Eigen::Vector4d v(q.w(), q.x(), q.y(), q.z());
		quatAccum += v * v.transpose();
		transAccum += offsetTrans;
	}

	Eigen::SelfAdjointEigenSolver<Eigen::Matrix4d> solver(quatAccum);
	Eigen::Vector4d avg = solver.eigenvectors().col(3).normalized();

	Eigen::Quaterniond q(avg(0), avg(1), avg(2), avg(3));
	q.normalize();
	if (q.w() < 0)
		q.coeffs() = -q.coeffs();

	transAccum /= (double)samples.size();

	if (ctx.mountRefined)
	{
		Eigen::Quaterniond stored(ctx.relativeRotation.w, ctx.relativeRotation.x, ctx.relativeRotation.y, ctx.relativeRotation.z);
		Eigen::Vector3d storedT(ctx.relativeTranslation.v[0], ctx.relativeTranslation.v[1], ctx.relativeTranslation.v[2]);
		double angDeg = stored.angularDistance(q) * 180.0 / EIGEN_PI;
		double transM = (storedT - transAccum).norm();
		if (angDeg < 5.0 && transM < 0.03)
		{
			char buf[256];
			snprintf(buf, sizeof buf, "Keeping refined mount offset (fresh measurement differs %.1f mm / %.2f deg)\n", transM * 1000.0, angDeg);
			ctx.Log(buf);
			ctx.validRelativeOffset = true;
			return;
		}
		ctx.mountRefined = false;
		ctx.Log("Refined mount offset discarded, tracker seems to have been remounted\n");
	}

	ctx.relativeRotation.w = q.w();
	ctx.relativeRotation.x = q.x();
	ctx.relativeRotation.y = q.y();
	ctx.relativeRotation.z = q.z();
	ctx.relativeTranslation.v[0] = transAccum.x();
	ctx.relativeTranslation.v[1] = transAccum.y();
	ctx.relativeTranslation.v[2] = transAccum.z();
	ctx.validRelativeOffset = true;
}

static_assert(vr::k_unTrackedDeviceIndex_Hmd == 0, "HMD index expected to be 0");

void ScanAndApplyProfile(CalibrationContext &ctx)
{
	char buffer[vr::k_unMaxPropertyStringSize];
	ctx.enabled = ctx.validProfile;

	if (ctx.enabled)
	{
		ctx.targetID = vr::k_unTrackedDeviceIndexInvalid;
		if (!ctx.trackerSerial.empty())
		{
			for (uint32_t id = 0; id < vr::k_unMaxTrackedDeviceCount; ++id)
			{
				if (vr::VRSystem()->GetTrackedDeviceClass(id) == vr::TrackedDeviceClass_Invalid)
					continue;
				if (GetDeviceSerial(id) == ctx.trackerSerial)
				{
					ctx.targetID = id;
					break;
				}
			}
		}
	}

	bool headTrackerProfile = !ctx.calibratedNoTracker
		&& (ctx.targetID == vr::k_unTrackedDeviceIndexInvalid || vr::VRSystem()->GetTrackedDeviceClass(ctx.targetID) == vr::TrackedDeviceClass_GenericTracker);
	bool overrideActive = ctx.enabled && ctx.validRelativeOffset && ctx.targetID != vr::k_unTrackedDeviceIndexInvalid && !ctx.noHeadTracker && headTrackerProfile;

	// Follow mode: send the HMD command first so the driver is already in follow mode when the
	// head tracker gets its transform. Otherwise transforms first, HMD command last.
	bool noTrackerFollow = ctx.enabled && ctx.validProfile && ctx.followSlamHmd && ctx.noHeadTracker;
	bool hmdCommandSent = false;
	if (overrideActive && ctx.followSlamHmd)
	{
		SendHmdTrackerCommand(vr::k_unTrackedDeviceIndex_Hmd, ctx.targetID, true);
		hmdCommandSent = true;
	}
	else if (noTrackerFollow)
	{
		SendHmdTrackerCommand(vr::k_unTrackedDeviceIndex_Hmd, vr::k_unTrackedDeviceIndexInvalid, true);
		hmdCommandSent = true;
	}

	for (uint32_t id = 0; id < vr::k_unMaxTrackedDeviceCount; ++id)
	{
		auto deviceClass = vr::VRSystem()->GetTrackedDeviceClass(id);
		if (deviceClass == vr::TrackedDeviceClass_Invalid)
			continue;

		// One message per device. Disable-then-enable would let a frame through untransformed.
		bool applyCalibration = false;

		if (ctx.enabled && id != vr::k_unTrackedDeviceIndex_Hmd)
		{
			vr::ETrackedPropertyError err = vr::TrackedProp_Success;
			vr::VRSystem()->GetStringTrackedDeviceProperty(id, vr::Prop_TrackingSystemName_String, buffer, vr::k_unMaxPropertyStringSize, &err);

			if (err == vr::TrackedProp_Success && std::string(buffer) == ctx.targetTrackingSystem)
			{
				// Head tracker stays raw while it drives the headset, in follow mode it's aligned like the rest.
				bool isHeadTracker = headTrackerProfile && deviceClass == vr::TrackedDeviceClass_GenericTracker && id == ctx.targetID;
				applyCalibration = !isHeadTracker || ctx.followSlamHmd;
			}
		}

		if (applyCalibration)
		{
			double known = GetLighthouseModelScale(id);
			double deviceScale = ctx.calibratedScale * (known > 0.0 ? known : ctx.targetModelScale) / ctx.targetModelScale;
			protocol::Request req(protocol::RequestSetDeviceTransform);
			req.setDeviceTransform = {
				id,
				true,
				VRTranslationVec(ctx.calibratedTranslation),
				VRRotationQuat(ctx.calibratedRotation),
				deviceScale
			};
			Driver.SendBlocking(req);
		}
		else
		{
			// Everything else stays raw.
			ResetAndDisableOffsets(id);
		}
	}

	for (uint32_t id = 0; overrideActive && id < vr::k_unMaxTrackedDeviceCount; ++id)
	{
		auto deviceClass = vr::VRSystem()->GetTrackedDeviceClass(id);
		if (deviceClass == vr::TrackedDeviceClass_Invalid)
			continue;

		// Follow mode: the world is SLAM space already, SLAM devices need no sync.
		bool sync = ctx.continuousSync
			&& !ctx.followSlamHmd
			&& id != vr::k_unTrackedDeviceIndex_Hmd
			&& deviceClass != vr::TrackedDeviceClass_TrackingReference;

		if (sync)
		{
			vr::ETrackedPropertyError err = vr::TrackedProp_Success;
			vr::VRSystem()->GetStringTrackedDeviceProperty(id, vr::Prop_TrackingSystemName_String, buffer, vr::k_unMaxPropertyStringSize, &err);
			sync = err == vr::TrackedProp_Success && std::string(buffer) != ctx.targetTrackingSystem;
		}

		protocol::Request req(protocol::RequestSetSlamSync);
		req.setSlamSync = { id, sync };
		Driver.SendBlocking(req);
	}

	if (!hmdCommandSent)
	{
		if (overrideActive)
			SendHmdTrackerCommand(vr::k_unTrackedDeviceIndex_Hmd, ctx.targetID, true);
		else
			SendHmdTrackerCommand(vr::k_unTrackedDeviceIndex_Hmd, vr::k_unTrackedDeviceIndexInvalid, false);
	}

	SendOneEuroParams();
	SendUniverseLock();

	try
	{
		protocol::Request statusReq(protocol::RequestGetStatus);
		protocol::Response statusResp = Driver.SendBlocking(statusReq);
		if (statusResp.type == protocol::ResponseStatus)
		{
			ctx.driverStatus = statusResp.status;
			const auto &st = ctx.driverStatus;
			RecordTilt(ctx, st);
			if (ctx.refinementDirty && ctx.validProfile && ctx.timeLastTick - ctx.timeRefinementSaved > 30.0)
			{
				SaveProfile(ctx);
				ctx.refinementDirty = false;
				ctx.timeRefinementSaved = ctx.timeLastTick;
			}
			if (st.refinementValid && ctx.enabled && ctx.validProfile && st.refinementSolves >= 1)
			{
				auto differs = [](double a, double b) { return std::fabs(a - b) > 1e-12; };
				bool changed = differs(st.offsetRotation.w, ctx.relativeRotation.w) || differs(st.offsetRotation.x, ctx.relativeRotation.x)
					|| differs(st.offsetRotation.y, ctx.relativeRotation.y) || differs(st.offsetRotation.z, ctx.relativeRotation.z)
					|| differs(st.offsetTranslation.v[0], ctx.relativeTranslation.v[0]) || differs(st.offsetTranslation.v[1], ctx.relativeTranslation.v[1])
					|| differs(st.offsetTranslation.v[2], ctx.relativeTranslation.v[2]) || differs(st.hmdScale, ctx.hmdScale);
				if (changed)
				{
					ctx.relativeRotation = st.offsetRotation;
					ctx.relativeTranslation = st.offsetTranslation;
					ctx.hmdScale = st.hmdScale;
					ctx.refinementDirty = true;
				}
				if (st.refinementTranslationSolves >= 1 && !ctx.mountRefined)
				{
					ctx.mountRefined = true;
					ctx.refinementDirty = true;
				}
				if (ctx.refinementDirty && ctx.timeLastTick - ctx.timeRefinementSaved > 30.0)
				{
					SaveProfile(ctx);
					ctx.refinementDirty = false;
					ctx.timeRefinementSaved = ctx.timeLastTick;
				}
			}
		}
	}
	catch (const std::runtime_error &e)
	{
		std::cerr << "Failed to read driver status: " << e.what() << std::endl;
	}

	if (ctx.enabled && ctx.chaperone.valid && ctx.chaperone.autoApply)
	{
		uint32_t quadCount = 0;
		vr::VRChaperoneSetup()->GetLiveCollisionBoundsInfo(nullptr, &quadCount);

		// Heuristic: when SteamVR resets to a blank-ish chaperone, it uses empty geometry,
		// but manual adjustments (e.g. via a play space mover) will not touch geometry.
		if (quadCount != ctx.chaperone.geometry.size())
		{
			ApplyChaperoneBounds();
		}
	}
}

static double lastGoodSampleTime = 0.0;
static double carriedAbsoluteHmdScale = 1.0;

static void BeginSamplingPhase(CalibrationContext &ctx, uint32_t targetID, double time)
{
	std::string hmdSerial = GetDeviceSerial(vr::k_unTrackedDeviceIndex_Hmd);
	double absoluteScale = ctx.targetModelScale > 0.0 ? ctx.hmdScale / ctx.targetModelScale : ctx.hmdScale;
	carriedAbsoluteHmdScale = ctx.validProfile && ctx.hmdSerial == hmdSerial && absoluteScale >= MinCalibratedScale && absoluteScale <= MaxCalibratedScale ? absoluteScale : 1.0;
	lastGoodSampleTime = time;

	ctx.targetID = targetID;
	ctx.targetTrackingSystem = GetDeviceTrackingSystem(targetID);
	ctx.hmdSerial = hmdSerial;
	ctx.trackerSerial = GetDeviceSerial(targetID);

	char buf[256];
	snprintf(buf, sizeof buf, "Using headset tracker: %s (id %d)\n", ctx.trackerSerial.c_str(), targetID);
	ctx.Log(buf);

	ResetAndDisableOffsets(targetID);
	SendHmdTrackerCommand(vr::k_unTrackedDeviceIndex_Hmd, vr::k_unTrackedDeviceIndexInvalid, false);

	ctx.sequenceStart = time;
	ctx.sequenceSteps = ctx.SequenceStepCount();
	ctx.sequenceStep = 0;
	ctx.state = CalibrationState::Sampling;
	ctx.wantedUpdateInterval = 0.0;
	ctx.Log("Starting calibration...\n");
}

static std::vector<Sample> collectedSamples;
static int coplanarRetries = 0;

void StartCalibration()
{
	if (!vr::VRSystem())
		return;

	CalCtx.lastCalibrationOk = false;
	CalCtx.state = CalibrationState::Begin;
	CalCtx.wantedUpdateInterval = 0.0;
	CalCtx.messages.clear();
	Detection.Clear();
	collectedSamples.clear();
	coplanarRetries = 0;
	SendUniverseLock();
}

static void AbortAndRestoreProfile(CalibrationContext &ctx)
{
	if (ctx.targetID != vr::k_unTrackedDeviceIndexInvalid)
		ResetAndDisableOffsets(ctx.targetID);

	LoadProfile(ctx);
	ctx.state = CalibrationState::None;
	collectedSamples.clear();
	coplanarRetries = 0;
}

void CancelCalibration()
{
	auto &ctx = CalCtx;
	if (ctx.state != CalibrationState::Begin && ctx.state != CalibrationState::Detect && ctx.state != CalibrationState::Sampling)
		return;

	ctx.Log("Calibration cancelled by user\n");
	Detection.Clear();
	AbortAndRestoreProfile(ctx);
}

static void UpdateSequenceStep(CalibrationContext &ctx, double time)
{
	if (ctx.state != CalibrationState::Sampling)
	{
		ctx.sequenceStep = 0;
		return;
	}

	int steps = ctx.sequenceSteps > 0 ? ctx.sequenceSteps : ctx.SequenceStepCount();
	int step = (int)((time - ctx.sequenceStart) / CalibrationContext::StepSeconds);
	ctx.sequenceStep = step < 0 ? 0 : (step > steps - 1 ? steps - 1 : step);
}

static void UpdateCalibrationSounds(CalibrationContext &ctx)
{
	static const char *directions[] = { "look_left", "look_center", "look_right", "look_center", "look_up", "look_center", "look_down", "look_center" };
	static CalibrationState lastState = CalibrationState::None;
	static int lastStep = -1;

	const int cycle = (int)(sizeof directions / sizeof directions[0]);
	static_assert(cycle == CalibrationContext::SequenceCycle, "voice cues and wizard steps must line up");

	if (ctx.disableVoiceHelp)
	{
		if (lastState != CalibrationState::None || lastStep != -1)
		{
			sound::Stop();
			lastState = CalibrationState::None;
			lastStep = -1;
		}
		return;
	}

	if (ctx.state == CalibrationState::Sampling)
	{
		if (ctx.sequenceStep != lastStep)
		{
			sound::Stop();
			sound::Play(directions[ctx.sequenceStep % cycle]);
			lastStep = ctx.sequenceStep;
		}
	}
	else
	{
		if (lastState == CalibrationState::Sampling)
		{
			sound::Stop();
			if (ctx.lastCalibrationOk)
				sound::Play("next");
		}
		lastStep = -1;
	}
	lastState = ctx.state;
}

void CalibrationTick(double time)
{
	if (!vr::VRSystem())
		return;

	auto &ctx = CalCtx;
	if ((time - ctx.timeLastTick) < 0.05)
		return;

	ctx.timeLastTick = time;

	static double lastConnectAttempt = -1e9;
	if (!Driver.IsConnected() && time - lastConnectAttempt >= 3.0)
	{
		lastConnectAttempt = time;
		Driver.TryConnect();
	}

	vr::VRSystem()->GetDeviceToAbsoluteTrackingPose(vr::TrackingUniverseRawAndUncalibrated, 0.0f, ctx.devicePoses, vr::k_unMaxTrackedDeviceCount);
	UpdateSequenceStep(ctx, time);
	UpdateCalibrationSounds(ctx);

	if (ctx.state == CalibrationState::None)
	{
		ctx.wantedUpdateInterval = 1.0;

		if ((time - ctx.timeLastScan) >= 1.0)
		{
			ScanAndApplyProfile(ctx);
			ctx.timeLastScan = time;
		}
		return;
	}

	if (ctx.state == CalibrationState::Editing)
	{
		ctx.wantedUpdateInterval = 0.1;

		if ((time - ctx.timeLastScan) >= 0.1)
		{
			ScanAndApplyProfile(ctx);
			ctx.timeLastScan = time;
		}
		return;
	}

	if (ctx.state == CalibrationState::Begin)
	{
		SendHmdTrackerCommand(vr::k_unTrackedDeviceIndex_Hmd, vr::k_unTrackedDeviceIndexInvalid, false);

		if (vr::VRSystem()->GetTrackedDeviceClass(vr::k_unTrackedDeviceIndex_Hmd) != vr::TrackedDeviceClass_HMD ||
			!ctx.devicePoses[vr::k_unTrackedDeviceIndex_Hmd].bPoseIsValid)
		{
			ctx.state = CalibrationState::None;
			CalCtx.Log("No tracking HMD found, aborting calibration!\n");
			return;
		}

		std::string hmdSystem = GetDeviceTrackingSystem(vr::k_unTrackedDeviceIndex_Hmd);

		Detection.Clear();
		for (uint32_t id = 0; id < vr::k_unMaxTrackedDeviceCount; ++id)
		{
			auto deviceClass = vr::VRSystem()->GetTrackedDeviceClass(id);

			bool usable = deviceClass == vr::TrackedDeviceClass_GenericTracker
				|| (ctx.noHeadTracker && deviceClass == vr::TrackedDeviceClass_Controller);
			if (!usable)
				continue;
			if (!ctx.devicePoses[id].bPoseIsValid)
				continue;

			if (!ctx.allowSameSpaceCalibration && GetDeviceTrackingSystem(id) == hmdSystem)
				continue;

			Detection.candidates.push_back(id);
		}

		if (Detection.candidates.empty())
		{
			ctx.state = CalibrationState::None;
			if (ctx.allowSameSpaceCalibration)
			{
				if (ctx.noHeadTracker)
					CalCtx.Log("No tracker or controller detected, aborting! Turn on the device you want to hold against your head.\n");
				else
					CalCtx.Log("No trackers detected, aborting! Turn on the tracker you want to mount on your head.\n");
			}
			else if (ctx.noHeadTracker)
				CalCtx.Log("No tracker or controller from a different tracking system detected, aborting! Turn on the device you want to hold against your head.\n");
			else
				CalCtx.Log("No trackers from a different tracking system detected, aborting!\n");
			return;
		}

		if (Detection.candidates.size() == 1)
		{
			ctx.targetID = Detection.candidates[0];
			BeginSamplingPhase(ctx, Detection.candidates[0], time);
			return;
		}

		Detection.candidateSpeeds.resize(Detection.candidates.size());
		CalCtx.Log("Move your head around to identify the headset tracker...\n");
		ctx.state = CalibrationState::Detect;
		ctx.wantedUpdateInterval = 0.0;
		return;
	}

	if (ctx.state == CalibrationState::Detect)
	{
		if (!ctx.devicePoses[vr::k_unTrackedDeviceIndex_Hmd].bPoseIsValid)
			return;

		Eigen::Matrix3d hmdRot = Pose(ctx.devicePoses[vr::k_unTrackedDeviceIndex_Hmd].mDeviceToAbsoluteTracking).rot;

		std::vector<Eigen::Matrix3d> curRot(Detection.candidates.size());
		for (size_t i = 0; i < Detection.candidates.size(); i++)
			curRot[i] = Pose(ctx.devicePoses[Detection.candidates[i]].mDeviceToAbsoluteTracking).rot;

		double dt = time - Detection.prevTime;
		if (Detection.havePrev && dt > 1e-4)
		{
			Detection.hmdSpeeds.push_back(AngularSpeedBetween(hmdRot, Detection.prevRot[0], dt));
			for (size_t i = 0; i < Detection.candidates.size(); i++)
				Detection.candidateSpeeds[i].push_back(AngularSpeedBetween(curRot[i], Detection.prevRot[i + 1], dt));

			CalCtx.Progress((int) Detection.hmdSpeeds.size(), 40);
		}

		Detection.prevRot.assign(1, hmdRot);
		Detection.prevRot.insert(Detection.prevRot.end(), curRot.begin(), curRot.end());
		Detection.prevTime = time;
		Detection.havePrev = true;

		if ((int) Detection.hmdSpeeds.size() < 40)
			return;

		double hmdPeak = 0;
		for (double s : Detection.hmdSpeeds)
			hmdPeak = max(hmdPeak, s);

		if (hmdPeak < 0.5)
		{
			Detection.Clear();
			ctx.state = CalibrationState::None;
			CalCtx.Log("Didn't detect enough head movement, aborting! Try again and move your head more.\n");
			return;
		}

		double bestCorr = -2, secondCorr = -2;
		int bestIdx = -1;
		for (size_t i = 0; i < Detection.candidates.size(); i++)
		{
			double corr = PearsonCorrelation(Detection.hmdSpeeds, Detection.candidateSpeeds[i]);
			if (corr > bestCorr)
			{
				secondCorr = bestCorr;
				bestCorr = corr;
				bestIdx = (int) i;
			}
			else if (corr > secondCorr)
			{
				secondCorr = corr;
			}
		}

		if (bestIdx == -1 || bestCorr < 0.7 || (bestCorr - secondCorr) < 0.1)
		{
			Detection.Clear();
			ctx.state = CalibrationState::None;
			CalCtx.Log("Couldn't clearly identify the headset tracker, aborting! Make sure only the headset tracker moves with your head, then try again.\n");
			return;
		}

		uint32_t targetID = Detection.candidates[bestIdx];
		Detection.Clear();
		BeginSamplingPhase(ctx, targetID, time);
		return;
	}

	const char *problem = nullptr;
	auto sample = CollectSample(ctx, problem);
	if (!sample.valid)
	{
		if (time - lastGoodSampleTime > 1.0)
		{
			char buf[256];
			snprintf(buf, sizeof buf, "%s for more than a second, aborting calibration! Previous calibration restored.\n", problem ? problem : "Tracking lost");
			CalCtx.Log(buf);
			AbortAndRestoreProfile(ctx);
		}
		return;
	}
	lastGoodSampleTime = time;

	auto &samples = collectedSamples;
	sample.time = time;
	samples.push_back(sample);

	double elapsed = time - ctx.sequenceStart;
	double total = ctx.SequenceSeconds();
	CalCtx.Progress((int)(elapsed * 1000.0), (int)(total * 1000.0));

	if (elapsed >= total)
	{
		CalCtx.Log("\n");

		if (samples.size() < 40)
		{
			CalCtx.Log("Not enough samples were collected, aborting calibration! Previous calibration restored.\n");
			AbortAndRestoreProfile(ctx);
			return;
		}


		double axisVariance = SecondAxisVariance(samples);
		if (axisVariance < AxisVarianceThreshold)
		{
			if (++coplanarRetries >= 10)
			{
				CalCtx.Log("Not enough rotation variety after several attempts, aborting calibration! Previous calibration restored.\n");
				AbortAndRestoreProfile(ctx);
				return;
			}

			char buf[256];
			snprintf(buf, sizeof buf, "Head movement is too uniform (axis variance %.5f), tilt and turn your head in different directions! Collecting more samples...\n", axisVariance);
			CalCtx.Log(buf);
			samples.erase(samples.begin(), samples.begin() + samples.size() / 4);
			return;
		}
		coplanarRetries = 0;

		double tauRot = 0.0, tauPos = 0.0;
		EstimateTimeOffsets(samples, tauRot, tauPos);
		if (tauRot != 0.0 || tauPos != 0.0)
		{
			std::vector<Eigen::Vector3d> omega, velocity;
			TargetRates(samples, omega, velocity);
			samples = ShiftTargets(samples, omega, velocity, tauRot, tauPos);
		}

		ctx.calibratedRotation = CalibrateRotation(samples);

		Eigen::Vector3d eulerRad = ctx.calibratedRotation * EIGEN_PI / 180.0;
		Eigen::Matrix3d calRot =
			(Eigen::AngleAxisd(eulerRad(0), Eigen::Vector3d::UnitZ()) *
			 Eigen::AngleAxisd(eulerRad(1), Eigen::Vector3d::UnitY()) *
			 Eigen::AngleAxisd(eulerRad(2), Eigen::Vector3d::UnitX())).toRotationMatrix();

		double calScale = 1.0;
		ctx.calibratedScale = calScale;
		ctx.targetModelScale = GetLighthouseModelScale(ctx.targetID);
		if (ctx.targetModelScale <= 0.0)
			ctx.targetModelScale = 1.0;

		double fittedHmdScale = EstimateHmdSpaceScale(samples, calRot, ctx.targetModelScale);
		if (fittedHmdScale > 0.0)
			ctx.hmdScale = fittedHmdScale;
		else
		{
			ctx.hmdScale = carriedAbsoluteHmdScale * ctx.targetModelScale;
			char buf[256];
			snprintf(buf, sizeof buf, "Keeping the headset scale %.5f (absolute %.5f)\n", ctx.hmdScale, carriedAbsoluteHmdScale);
			CalCtx.Log(buf);
		}

		for (auto &sample : samples)
			sample.ref.trans /= ctx.hmdScale;

		if (ctx.noHeadTracker)
		{
			calRot = LevelledRotation(ctx, samples, calRot);
			ctx.calibratedRotation = calRot.eulerAngles(2, 1, 0) * 180.0 / EIGEN_PI;
			calRot = CalibrationMatrix(ctx);
		}

		ctx.calibratedTranslation = CalibrateTranslation(samples, calRot, calScale);
		Eigen::Vector3d calTransM = ctx.calibratedTranslation * 0.01;

		Eigen::Vector3d hmdToTarget = ComputeRefToTargetOffset(samples, calRot, calTransM, calScale);
		double rmsError = RetargetingErrorRMS(samples, hmdToTarget, calRot, calTransM, calScale);

		char buf2[256];
		snprintf(buf2, sizeof buf2, "Calibration residual error (RMS): %.1f mm\n", rmsError * 1000.0);
		CalCtx.Log(buf2);

		// TODO: this is an problem for future considering automatic calibration fixing.
		if (rmsError > 0.1)
		{
			CalCtx.Log("Calibration quality is too low, aborting! Previous calibration restored. Try again with a slower calibration speed, moving smoothly.\n");
			AbortAndRestoreProfile(ctx);
			return;
		}

		ComputeRelativeOffset(ctx, samples, calRot, calTransM, calScale);

		ctx.calibrationLatencyValid = tauRot != 0.0;
		ctx.calibrationLatencyRot = tauRot;
		ctx.calibrationLatencyPos = tauPos != 0.0 ? tauPos : tauRot;

		Eigen::Vector3d spot = Eigen::Vector3d::Zero();
		for (auto &sample : samples)
			spot += sample.target.trans;
		ctx.calibrationSpot = spot / (double)samples.size();
		ctx.calibrationSpotValid = true;
		ctx.calibratedNoTracker = ctx.noHeadTracker;

		ctx.validProfile = true;
		ctx.lastCalibrationOk = true;
		SaveProfile(ctx);
		CalCtx.Log("Finished calibration, profile saved\n");

		if (CalCtx.notificationId != 0) {
			vr::VRNotifications()->RemoveNotification(CalCtx.notificationId);
			CalCtx.notificationId = 0;
		}


		ctx.state = CalibrationState::None;
		samples.clear();
		ctx.lockCommitPending = true;
		SendUniverseLock();
	}
}

void LoadChaperoneBounds()
{
	if (!vr::VRChaperoneSetup())
		return;

	vr::VRChaperoneSetup()->RevertWorkingCopy();

	uint32_t quadCount = 0;
	vr::VRChaperoneSetup()->GetLiveCollisionBoundsInfo(nullptr, &quadCount);

	CalCtx.chaperone.geometry.resize(quadCount);
	vr::VRChaperoneSetup()->GetLiveCollisionBoundsInfo(&CalCtx.chaperone.geometry[0], &quadCount);
	vr::VRChaperoneSetup()->GetWorkingStandingZeroPoseToRawTrackingPose(&CalCtx.chaperone.standingCenter);
	vr::VRChaperoneSetup()->GetWorkingPlayAreaSize(&CalCtx.chaperone.playSpaceSize.v[0], &CalCtx.chaperone.playSpaceSize.v[1]);
	CalCtx.chaperone.valid = true;
}

void ApplyChaperoneBounds()
{
	if (!vr::VRChaperoneSetup())
		return;

	vr::VRChaperoneSetup()->RevertWorkingCopy();
	vr::VRChaperoneSetup()->SetWorkingCollisionBoundsInfo(&CalCtx.chaperone.geometry[0], CalCtx.chaperone.geometry.size());
	vr::VRChaperoneSetup()->SetWorkingStandingZeroPoseToRawTrackingPose(&CalCtx.chaperone.standingCenter);
	vr::VRChaperoneSetup()->SetWorkingPlayAreaSize(CalCtx.chaperone.playSpaceSize.v[0], CalCtx.chaperone.playSpaceSize.v[1]);
	vr::VRChaperoneSetup()->CommitWorkingCopy(vr::EChaperoneConfigFile_Live);
}
