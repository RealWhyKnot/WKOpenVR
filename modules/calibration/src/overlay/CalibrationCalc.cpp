#include "CalibrationCalc.h"
#include "Calibration.h"
#include "CalibrationMetrics.h"
#include "Protocol.h"
#include "CalibrationRejectReason.h"
#include "RuntimeHealthSummary.h"
#include "RotationMatrix3.h" // AngleFromRotationMatrix3 / AxisFromRotationMatrix3 (clamped).

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <numeric>
#include <vector>

inline vr::HmdQuaternion_t operator*(const vr::HmdQuaternion_t& lhs, const vr::HmdQuaternion_t& rhs)
{
	return {(lhs.w * rhs.w) - (lhs.x * rhs.x) - (lhs.y * rhs.y) - (lhs.z * rhs.z),
	        (lhs.w * rhs.x) + (lhs.x * rhs.w) + (lhs.y * rhs.z) - (lhs.z * rhs.y),
	        (lhs.w * rhs.y) + (lhs.y * rhs.w) + (lhs.z * rhs.x) - (lhs.x * rhs.z),
	        (lhs.w * rhs.z) + (lhs.z * rhs.w) + (lhs.x * rhs.y) - (lhs.y * rhs.x)};
}

namespace {

inline vr::HmdVector3d_t quaternionRotateVector(const vr::HmdQuaternion_t& quat, const double (&vector)[3])
{
	vr::HmdQuaternion_t vectorQuat = {0.0, vector[0], vector[1], vector[2]};
	vr::HmdQuaternion_t conjugate = {quat.w, -quat.x, -quat.y, -quat.z};
	auto rotatedVectorQuat = quat * vectorQuat * conjugate;
	return {rotatedVectorQuat.x, rotatedVectorQuat.y, rotatedVectorQuat.z};
}

inline Eigen::Matrix3d quaternionRotateMatrix(const vr::HmdQuaternion_t& quat)
{
	return Eigen::Quaterniond(quat.w, quat.x, quat.y, quat.z).toRotationMatrix();
}

struct DSample
{
	bool valid;
	Eigen::Vector3d ref, target;
};

bool StartsWith(const std::string& str, const std::string& prefix)
{
	if (str.length() < prefix.length()) return false;

	return str.compare(0, prefix.length(), prefix) == 0;
}

bool EndsWith(const std::string& str, const std::string& suffix)
{
	if (str.length() < suffix.length()) return false;

	return str.compare(str.length() - suffix.length(), suffix.length(), suffix) == 0;
}

vr::HmdQuaternion_t VRRotationQuat(Eigen::Vector3d eulerdeg)
{
	auto euler = eulerdeg * EIGEN_PI / 180.0;

	Eigen::Quaterniond rotQuat = Eigen::AngleAxisd(euler(0), Eigen::Vector3d::UnitZ()) *
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

DSample DeltaRotationSamples(Sample s1, Sample s2)
{
	// Difference in rotation between samples.
	auto dref = s1.ref.rot * s2.ref.rot.transpose();
	auto dtarget = s1.target.rot * s2.target.rot.transpose();

	// When stuck together, the two tracked objects rotate as a pair,
	// therefore their axes of rotation must be equal between any given pair of samples.
	DSample ds;
	ds.ref = AxisFromRotationMatrix3(dref);
	ds.target = AxisFromRotationMatrix3(dtarget);

	// Reject samples that were too close to each other.
	auto refA = AngleFromRotationMatrix3(dref);
	auto targetA = AngleFromRotationMatrix3(dtarget);
	ds.valid = refA > 0.4 && targetA > 0.4 && ds.ref.norm() > 0.01 && ds.target.norm() > 0.01;

	// Only normalise when the magnitudes pass the gate above; a sub-1cm
	// axis would otherwise be normalised to NaN/Inf entries, and any
	// downstream consumer that forgets to check ds.valid would ingest
	// the garbage.
	if (ds.valid) {
		ds.ref.normalize();
		ds.target.normalize();
	}
	return ds;
}

double PercentileSorted(const std::vector<double>& sorted, double p)
{
	if (sorted.empty()) return std::numeric_limits<double>::infinity();
	if (sorted.size() == 1) return sorted[0];
	const double pos = p * static_cast<double>(sorted.size() - 1);
	const size_t lo = static_cast<size_t>(std::floor(pos));
	const size_t hi = std::min(sorted.size() - 1, lo + 1);
	const double frac = pos - static_cast<double>(lo);
	return sorted[lo] + (sorted[hi] - sorted[lo]) * frac;
}

CalibrationResidualStats ResidualStatsFor(std::vector<double> residuals)
{
	CalibrationResidualStats out;
	out.count = static_cast<int>(residuals.size());
	if (residuals.empty()) return out;

	double sumSq = 0.0;
	for (double r : residuals)
		sumSq += r * r;
	std::sort(residuals.begin(), residuals.end());

	out.medianM = PercentileSorted(residuals, 0.50);
	out.p90M = PercentileSorted(residuals, 0.90);
	out.p95M = PercentileSorted(residuals, 0.95);
	out.maxM = residuals.back();
	out.rmsM = std::sqrt(sumSq / static_cast<double>(residuals.size()));

	std::vector<double> absDev;
	absDev.reserve(residuals.size());
	for (double r : residuals)
		absDev.push_back(std::abs(r - out.medianM));
	std::sort(absDev.begin(), absDev.end());
	out.madSigmaM = 1.4826 * PercentileSorted(absDev, 0.50);

	const double outlierCut = out.medianM + std::max(0.010, 4.0 * out.madSigmaM);
	int outliers = 0;
	for (double r : residuals) {
		if (r > outlierCut) ++outliers;
	}
	out.outlierFraction = static_cast<double>(outliers) / static_cast<double>(residuals.size());
	return out;
}

const char* Bool01(bool value)
{
	return value ? "1" : "0";
}

double SafeRatio(double numerator, double denominator)
{
	if (!std::isfinite(numerator) || !std::isfinite(denominator) || denominator <= 1e-12) {
		return 0.0;
	}
	return std::max(0.0, std::min(1.0, numerator / denominator));
}

template <typename PositionSelector>
double PositionJitterFor(const std::deque<Sample>& samples, PositionSelector selectPosition)
{
	Eigen::Vector3d mean = Eigen::Vector3d::Zero();
	Eigen::Vector3d sumSquares = Eigen::Vector3d::Zero();
	int sampleCount = 0;

	for (const auto& sample : samples) {
		if (!sample.valid) continue;
		const Eigen::Vector3d position = selectPosition(sample);
		++sampleCount;
		if (sampleCount == 1) {
			mean = position;
			continue;
		}

		const Eigen::Vector3d delta = position - mean;
		mean += delta / static_cast<double>(sampleCount);
		const Eigen::Vector3d delta2 = position - mean;
		sumSquares += delta.cwiseProduct(delta2);
	}

	if (sampleCount < 2) return 0.0;
	const Eigen::Vector3d variance = (sumSquares / static_cast<double>(sampleCount - 1)).cwiseMax(0.0);
	return variance.cwiseSqrt().norm();
}
} // namespace

const double CalibrationCalc::AxisVarianceThreshold = 0.001;
void CalibrationCalc::PushSample(const Sample& sample)
{
	if (!sample.ref.trans.allFinite() || !sample.target.trans.allFinite() ||
	    sample.ref.trans.cwiseAbs().maxCoeff() > 5.0 || sample.target.trans.cwiseAbs().maxCoeff() > 5.0) {
		Metrics::WriteLogAnnotation("PushSample_dropped_oversize_or_nonfinite");
		return; // drop the sample
	}
	m_samples.push_back(sample);
	if (sample.valid && sample.timestamp > m_lastSampleTime) {
		m_lastSampleTime = sample.timestamp;
	}
}

void CalibrationCalc::Clear()
{
	m_estimatedTransformation.setIdentity();
	m_isValid = false;
	m_lastComputeUsedRelPose = false;
	m_samples.clear();
	m_rotationFrozen.clear();
	m_axisVariance = 0.0;
	m_refToTargetPose = Eigen::AffineCompact3d::Identity();
	m_relativePosCalibrated = false;
	m_lastPriorRetargetingErrorM = std::numeric_limits<double>::infinity();
	m_lastCandidateRetargetingErrorM = std::numeric_limits<double>::infinity();
}

void CalibrationCalc::SeedEstimatedTransformation(const Eigen::AffineCompact3d& transform, bool annotate)
{
	if (!transform.matrix().allFinite()) {
		if (annotate) {
			Metrics::WriteLogAnnotation("SeedEstimatedTransformation_rejected: nonfinite_transform");
		}
		return;
	}

	m_estimatedTransformation = transform;
	m_isValid = true;
	m_lastComputeUsedRelPose = false;
	m_lastPriorRetargetingErrorM = std::numeric_limits<double>::infinity();
	m_lastCandidateRetargetingErrorM = std::numeric_limits<double>::infinity();

	const Eigen::Quaterniond q(transform.rotation());
	const Eigen::Quaterniond twistY(q.w(), 0.0, q.y(), 0.0);
	const double twistNorm = std::sqrt(twistY.w() * twistY.w() + twistY.y() * twistY.y());
	const double yaw = (twistNorm > 1e-12) ? 2.0 * std::atan2(twistY.y() / twistNorm, twistY.w() / twistNorm) : 0.0;

	if (annotate) {
		char buf[220];
		snprintf(
		    buf, sizeof buf,
		    "SeedEstimatedTransformation_applied: trans_cm=(%.2f,%.2f,%.2f) mag_cm=%.2f yaw_deg=%.3f sample_count=%zu",
		    transform.translation().x() * 100.0, transform.translation().y() * 100.0,
		    transform.translation().z() * 100.0, transform.translation().norm() * 100.0, yaw * 57.29577951308232,
		    m_samples.size());
		Metrics::WriteLogAnnotation(buf);
	}
}

void CalibrationCalc::FreezeRotationPhaseSamples()
{
	// Move the live sample buffer into the frozen-rotation slot so the next
	// CollectSample tick starts a fresh translation-phase buffer. ComputeOneshot
	// later splices these back in for the duration of the solve so the math sees
	// rotation+translation samples as a single unified deque.
	m_rotationFrozen = std::move(m_samples);
	m_samples.clear(); // explicit; std::deque move-from is empty per the standard but be defensive
}

std::vector<bool> CalibrationCalc::DetectOutliers() const
{
	// Use bigger step to get a rough rotation.
	std::vector<DSample> deltas;
	const size_t step = 5;
	for (size_t i = 0; i < m_samples.size(); i += step) {
		for (size_t j = 0; j < i; j += step) {
			auto delta = DeltaRotationSamples(m_samples[i], m_samples[j]);
			if (delta.valid) {
				deltas.push_back(delta);
			}
		}
	}

	// With too little data to make an outlier judgement, accept everything.
	if (deltas.empty()) {
		return std::vector<bool>(m_samples.size(), true);
	}

	// Kabsch algorithm
	Eigen::MatrixXd refPoints(deltas.size(), 3), targetPoints(deltas.size(), 3);
	Eigen::Vector3d refCentroid(0, 0, 0), targetCentroid(0, 0, 0);

	for (size_t i = 0; i < deltas.size(); i++) {
		refPoints.row(i) = deltas[i].ref;
		refCentroid += deltas[i].ref;
		targetPoints.row(i) = deltas[i].target;
		targetCentroid += deltas[i].target;
	}

	refCentroid /= (double)deltas.size();
	targetCentroid /= (double)deltas.size();

	for (size_t i = 0; i < deltas.size(); i++) {
		refPoints.row(i) -= refCentroid;
		targetPoints.row(i) -= targetCentroid;
	}

	auto crossCV = refPoints.transpose() * targetPoints;

	Eigen::BDCSVD<Eigen::MatrixXd> bdcsvd;
	auto svd = bdcsvd.compute(crossCV, Eigen::ComputeThinU | Eigen::ComputeThinV);

	Eigen::Matrix3d i = Eigen::Matrix3d::Identity();
	if ((svd.matrixU() * svd.matrixV().transpose()).determinant() < 0) {
		i(2, 2) = -1;
	}

	Eigen::Matrix3d rot = svd.matrixV() * i * svd.matrixU().transpose();
	rot.transposeInPlace();

	// Optimize an extrinsic from reference to target.
	// Detect the outliers by comparing the extrinsic computed from each pair of rotation to the optimized extrinsic.
	Eigen::MatrixXd coefficients(m_samples.size() * 4, 4);
	Eigen::VectorXd constraints(m_samples.size() * 4);
	std::vector<bool> valids(m_samples.size());
	const double threshold = 0.99;
	for (size_t i = 0; i < m_samples.size(); i++) {
		Eigen::Matrix3d rotExtTmp = (m_samples[i].ref.rot.transpose() * rot * m_samples[i].target.rot);
		Eigen::Quaterniond quatExtTmp(rotExtTmp);
		quatExtTmp.normalize();
		coefficients.block<4, 4>(4 * i, 0) = Eigen::Matrix4d::Identity();
		constraints.block<4, 1>(4 * i, 0) =
		    Eigen::Vector4d(quatExtTmp.w(), quatExtTmp.x(), quatExtTmp.y(), quatExtTmp.z());
	}
	Eigen::Vector4d result = coefficients.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(constraints);
	Eigen::Quaterniond quatExt(result(0), result(1), result(2), result(3));
	quatExt.normalize();

	for (size_t i = 0; i < m_samples.size(); i++) {
		Eigen::Matrix3d rotExtTmp = (m_samples[i].ref.rot.transpose() * rot * m_samples[i].target.rot);
		Eigen::Quaterniond quatExtTmp(rotExtTmp);
		double cosHalfAngle = quatExtTmp.w() * quatExt.w() + quatExtTmp.vec().dot(quatExt.vec());
		valids[i] = std::abs(cosHalfAngle) >= threshold;
	}

	return valids;
}

Eigen::Vector3d CalibrationCalc::CalibrateRotation(const bool ignoreOutliers) const
{
	std::vector<DSample> deltas;
	std::vector<bool> valids = DetectOutliers();

	for (size_t i = 0; i < m_samples.size(); i++) {
		for (size_t j = 0; j < i; j++) {
			if (ignoreOutliers && (!valids[i] || !valids[j])) {
				continue;
			}
			auto delta = DeltaRotationSamples(m_samples[i], m_samples[j]);
			if (delta.valid) {
				deltas.push_back(delta);
			}
		}
	}
	// char buf[256];
	// snprintf(buf, sizeof buf, "Got %zd samples with %zd delta samples\n", m_samples.size(), deltas.size());
	// CalCtx.Log(buf);

	if (deltas.empty()) {
		return Eigen::Vector3d::Zero();
	}

	// Kabsch algorithm, matching the upstream yaw-only solve.
	Eigen::MatrixXd refPoints(deltas.size(), 2), targetPoints(deltas.size(), 2);
	Eigen::Vector2d refCentroid(0, 0), targetCentroid(0, 0);

	for (size_t i = 0; i < deltas.size(); i++) {
		refPoints.row(i) << deltas[i].ref[0], deltas[i].ref[2];
		refCentroid += refPoints.row(i);

		targetPoints.row(i) << deltas[i].target[0], deltas[i].target[2];
		targetCentroid += targetPoints.row(i);
	}

	refCentroid /= (double)deltas.size();
	targetCentroid /= (double)deltas.size();

	for (size_t i = 0; i < deltas.size(); i++) {
		refPoints.row(i) -= refCentroid;
		targetPoints.row(i) -= targetCentroid;
	}

	auto crossCV = refPoints.transpose() * targetPoints;

	Eigen::JacobiSVD<Eigen::MatrixXd> svd(crossCV, Eigen::ComputeThinU | Eigen::ComputeThinV);

	Eigen::Matrix2d i = Eigen::Matrix2d::Identity();
	Eigen::Matrix2d rot = svd.matrixV() * i * svd.matrixU().transpose();

	double yaw = std::atan2(rot(1, 0), rot(0, 0));

	Eigen::Vector3d euler(0.0, yaw * 180.0 / EIGEN_PI, 0.0);

	// snprintf(buf, sizeof buf, "Calibrated rotation: yaw=%.2f pitch=%.2f roll=%.2f\n", euler[1], euler[2], euler[0]);
	// CalCtx.Log(buf);
	return euler;
}

Eigen::Vector3d CalibrationCalc::CalibrateTranslation(const Eigen::Matrix3d& rotation) const
{
	std::vector<std::pair<Eigen::Vector3d, Eigen::Matrix3d>> deltas;

	for (size_t i = 0; i < m_samples.size(); i++) {
		Sample s_i = m_samples[i];
		s_i.target.rot = rotation * s_i.target.rot;
		s_i.target.trans = rotation * s_i.target.trans;

		for (size_t j = 0; j < i; j++) {
			Sample s_j = m_samples[j];
			s_j.target.rot = rotation * s_j.target.rot;
			s_j.target.trans = rotation * s_j.target.trans;

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

	if (deltas.empty()) {
		return Eigen::Vector3d::Zero();
	}

	Eigen::VectorXd constants(deltas.size() * 3);
	Eigen::MatrixXd coefficients(deltas.size() * 3, 3);

	for (size_t i = 0; i < deltas.size(); i++) {
		for (int axis = 0; axis < 3; axis++) {
			constants(i * 3 + axis) = deltas[i].first(axis);
			coefficients.row(i * 3 + axis) = deltas[i].second.row(axis);
		}
	}

	Eigen::Vector3d trans = coefficients.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(constants);
	auto transcm = trans * 100.0;
	(void)transcm;

	// char buf[256];
	// snprintf(buf, sizeof buf, "Calibrated translation x=%.2f y=%.2f z=%.2f\n", transcm[0], transcm[1], transcm[2]);
	// CalCtx.Log(buf);
	return trans;
}

namespace {
Pose ApplyTransform(const Pose& originalPose, const Eigen::AffineCompact3d& transform)
{
	Pose pose(originalPose);
	pose.rot = transform.rotation() * pose.rot;
	pose.trans = transform * pose.trans;
	return pose;
}

Pose ApplyTransform(const Pose& originalPose, const Eigen::Vector3d& vrTrans, const Eigen::Matrix3d& rotMat)
{
	Pose pose(originalPose);
	pose.rot = rotMat * pose.rot;
	pose.trans = vrTrans + (rotMat * pose.trans);
	return pose;
}
} // namespace

Eigen::AffineCompact3d CalibrationCalc::ComputeCalibration(const bool ignoreOutliers) const
{
	Eigen::Vector3d rotation = CalibrateRotation(ignoreOutliers);
	Eigen::Matrix3d rotationMat = quaternionRotateMatrix(VRRotationQuat(rotation));
	Eigen::Vector3d translation = CalibrateTranslation(rotationMat);

	Eigen::AffineCompact3d rot(rotationMat);
	Eigen::Translation3d trans(translation);

	return trans * rot;
}

double CalibrationCalc::RetargetingErrorRMS(const Eigen::Vector3d& hmdToTargetPos,
                                            const Eigen::AffineCompact3d& calibration) const
{
	double errorAccum = 0;
	int sampleCount = 0;

	for (auto& sample : m_samples) {
		if (!sample.valid) continue;

		// Apply transformation
		const auto updatedPose = ApplyTransform(sample.target, calibration);

		const Eigen::Vector3d hmdPoseSpace = sample.ref.rot * hmdToTargetPos + sample.ref.trans;

		// Compute error term
		double error = (updatedPose.trans - hmdPoseSpace).squaredNorm();
		errorAccum += error;
		sampleCount++;
	}

	if (sampleCount == 0) return std::numeric_limits<double>::infinity();
	return sqrt(errorAccum / sampleCount);
}

double CalibrationCalc::ReferenceJitter() const
{
	return PositionJitterFor(m_samples, [](const Sample& sample) { return sample.ref.trans; });
}

double CalibrationCalc::TargetJitter() const
{
	return PositionJitterFor(m_samples, [](const Sample& sample) { return sample.target.trans; });
}

double CalibrationCalc::TranslationDiversity() const
{
	// Per-axis bounding box of the target tracker translation across valid
	// samples. The smallest axis-range relative to a desired total spread
	// is the "weakest link" -- a user who waved on X+Y but never on Z gets
	// a low score regardless of how much they waved on the other two.
	//
	// pairedMotionValid filter: a sample where only the target moved (HMD
	// frozen by passthrough/desktop overlay, etc.) tells us nothing useful
	// about the calibration. Excluding those samples keeps the progress bar
	// honest about how much *usable* data the user has provided.
	if (m_samples.size() < 2) return 0.0;
	constexpr double kInf = std::numeric_limits<double>::infinity();
	Eigen::Vector3d minPos(kInf, kInf, kInf);
	Eigen::Vector3d maxPos(-kInf, -kInf, -kInf);
	int n = 0;
	for (const auto& s : m_samples) {
		if (!s.valid || !s.pairedMotionValid) continue;
		minPos = minPos.cwiseMin(s.target.trans);
		maxPos = maxPos.cwiseMax(s.target.trans);
		++n;
	}
	if (n < 2) return 0.0;
	const Eigen::Vector3d range = maxPos - minPos;
	// 20cm spread per axis is sufficient for the translation LS to be
	// well-conditioned across typical setups, including trackers rigid-
	// mounted to an HMD where pure-translation head movement is limited.
	// Lowered from 0.30 m (2026-05-13): 30cm demanded 21cm per axis before
	// the 70% gate fired, which a head-mounted tracker rarely achieves on
	// the weakest (usually Y or Z) axis in a normal calibration sweep.
	constexpr double kDesiredAxisRange = 0.20;
	const double minAxis = range.minCoeff();
	const double score = minAxis / kDesiredAxisRange;
	return std::min(std::max(score, 0.0), 1.0);
}

Eigen::Vector3d CalibrationCalc::TranslationAxisRangesCm() const
{
	// Same bounding-box scan as TranslationDiversity, but returns the per-axis
	// ranges in centimetres rather than collapsing them to a single score. The
	// UI tooltip uses these to tell the user which axis is the bottleneck when
	// the Translation% bar is stuck below 100. Whichever component is smallest
	// is what's pinning the score (= min component / kDesiredAxisRange = 20 cm).
	if (m_samples.size() < 2) return Eigen::Vector3d::Zero();
	Eigen::Vector3d minPos = Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity());
	Eigen::Vector3d maxPos = -minPos;
	int n = 0;
	for (const auto& s : m_samples) {
		if (!s.valid || !s.pairedMotionValid) continue;
		minPos = minPos.cwiseMin(s.target.trans);
		maxPos = maxPos.cwiseMax(s.target.trans);
		++n;
	}
	if (n < 2) return Eigen::Vector3d::Zero();
	return (maxPos - minPos) * 100.0;
}

double CalibrationCalc::RotationDiversity() const
{
	// Maximum angular distance between any two sampled target rotations.
	// One pair with a wide angular separation is enough to anchor yaw; we
	// scale toward 90 degrees as the "fully covered" point. This is much
	// less stringent than the full SO(3) Kabsch needs for a clean fit, but
	// matches the practical observation that even modest rotation variety
	// constrains the calibration's rotation component well.
	if (m_samples.size() < 2) return 0.0;
	constexpr double kDesiredMaxAngle = EIGEN_PI / 2.0; // 90 deg
	std::vector<Eigen::Quaterniond> rotations;
	rotations.reserve(m_samples.size());
	double maxAngle = 0.0;
	for (const auto& s : m_samples) {
		if (!s.valid || !s.pairedMotionValid) continue;
		rotations.emplace_back(s.target.rot);
	}
	for (size_t i = 0; i < rotations.size(); ++i) {
		for (size_t j = i + 1; j < rotations.size(); ++j) {
			const double a = rotations[i].angularDistance(rotations[j]);
			if (a > maxAngle) maxAngle = a;
		}
	}
	const double score = maxAngle / kDesiredMaxAngle;
	return std::min(std::max(score, 0.0), 1.0);
}

Eigen::Vector3d CalibrationCalc::ComputeRefToTargetOffset(const Eigen::AffineCompact3d& calibration) const
{
	Eigen::Vector3d accum = Eigen::Vector3d::Zero();
	int sampleCount = 0;

	for (auto& sample : m_samples) {
		if (!sample.valid) continue;

		// Apply transformation
		const auto updatedPose = ApplyTransform(sample.target, calibration);

		// Now move the transform from world to HMD space
		const auto hmdOriginPos = updatedPose.trans - sample.ref.trans;
		const auto hmdSpace = sample.ref.rot.inverse() * hmdOriginPos;

		accum += hmdSpace;
		sampleCount++;
	}

	if (sampleCount == 0) return Eigen::Vector3d::Zero();
	accum /= sampleCount;

	return accum;
}

Eigen::Vector4d CalibrationCalc::ComputeAxisVariance(const Eigen::AffineCompact3d& calibration) const
{
	// We want to determine if the user rotated in enough axis to find a unique solution.
	// It's sufficient to rotate in two axis - this is because once we constrain the mapping
	// of those two orthogonal basis vectors, the third is determined by the cross product of
	// those two basis vectors. So, the question we then have to answer is - after accounting for
	// translational movement of the HMD itself, are we too close to having only moved on a plane?

	// To determine this, we perform primary component analysis on the rotation quaternions themselves.
	// Since an angle axis quaternion is defined as the sum of Qidentity*cos(angle/2) + Qaxis*sin(angle/2),
	// we expect that rotations around a single axis will have two primary components: One corresponding
	// to the identity component, and one to the axis component. Thus, we check the variance (eigenvalue) of
	// the third primary component to see if we've moved in two axis.
	std::ostringstream dbgStream;

	std::vector<Eigen::Vector4d> points;

	Eigen::Vector4d mean = Eigen::Vector4d::Zero();

	for (auto& sample : m_samples) {
		if (!sample.valid) continue;

		auto q = Eigen::Quaterniond(sample.target.rot);
		auto point = Eigen::Vector4d(q.w(), q.x(), q.y(), q.z());
		mean += point;

		points.push_back(point);
	}
	if (points.empty()) return Eigen::Vector4d::Zero();
	mean /= (double)points.size();

	// Compute covariance matrix
	Eigen::Matrix4d covMatrix = Eigen::Matrix4d::Zero();

	for (auto& point : points) {
		for (int i = 0; i < 4; i++) {
			for (int j = 0; j < 4; j++) {
				covMatrix(i, j) += (point(i) - mean(i)) * (point(j) - mean(j));
			}
		}
	}
	covMatrix /= (double)points.size();

	Eigen::SelfAdjointEigenSolver<Eigen::Matrix4d> solver;
	solver.compute(covMatrix);

	return solver.eigenvalues();
}

[[nodiscard]] bool CalibrationCalc::ValidateCalibration(const Eigen::AffineCompact3d& calibration, double* error,
                                                        Eigen::Vector3d* posOffsetV)
{
	bool ok = true;

	const auto posOffset = ComputeRefToTargetOffset(calibration);

	if (posOffsetV) *posOffsetV = posOffset;

	// char buf[256];
	// snprintf(buf, sizeof buf, "HMD to target offset: (%.2f, %.2f, %.2f)\n", posOffset(0), posOffset(1),
	// posOffset(2)); CalCtx.Log(buf);

	double rmsError = RetargetingErrorRMS(posOffset, calibration);
	// snprintf(buf, sizeof buf, "Position error (RMS): %.3f\n", rmsError);
	// CalCtx.Log(buf);

	if (rmsError > 0.1) {
		ok = false;
	}

	if (error) *error = rmsError;

	return ok;
}

CalibrationQualityVerdict EvaluateCalibrationQualityVerdict(const CalibrationQualityReport& report)
{
	if (report.validSampleCount <= 0) {
		return {false, "no_valid_samples"};
	}
	if (!report.legacyRmsPass) {
		return {false, "legacy_rms"};
	}
	if (!report.geometryPass) {
		return {false, "geometry"};
	}
	if (!report.robustResidualPass) {
		return {false, "robust_residual"};
	}
	if (!report.holdoutPass) {
		return {false, "holdout"};
	}
	return {true, "pass"};
}

CalibrationQualityReport CalibrationCalc::EvaluateCalibrationQuality(const Eigen::AffineCompact3d& calibration,
                                                                     bool includeHoldout, bool ignoreOutliers) const
{
	CalibrationQualityReport report;
	report.sampleCount = m_samples.size();
	report.posOffsetM = ComputeRefToTargetOffset(calibration);

	std::vector<const Sample*> validSamples;
	validSamples.reserve(m_samples.size());
	Eigen::Vector3d refMin = Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity());
	Eigen::Vector3d refMax = -refMin;
	Eigen::Vector3d targetMin = refMin;
	Eigen::Vector3d targetMax = -targetMin;
	std::vector<Eigen::Quaterniond> targetRotations;

	std::vector<double> residuals;
	residuals.reserve(m_samples.size());
	for (const auto& sample : m_samples) {
		if (!sample.valid) continue;
		validSamples.push_back(&sample);
		++report.validSampleCount;
		if (sample.pairedMotionValid) ++report.pairedSampleCount;
		if (sample.trackingPoseStale) ++report.trackingStaleSampleCount;
		if (sample.trackingPoseJump) ++report.trackingJumpSampleCount;
		report.maxPoseAgeMs = std::max(report.maxPoseAgeMs, std::max(sample.refPoseAgeMs, sample.targetPoseAgeMs));
		report.maxPoseGapMs = std::max(report.maxPoseGapMs, std::max(sample.refPoseGapMs, sample.targetPoseGapMs));
		refMin = refMin.cwiseMin(sample.ref.trans);
		refMax = refMax.cwiseMax(sample.ref.trans);
		targetMin = targetMin.cwiseMin(sample.target.trans);
		targetMax = targetMax.cwiseMax(sample.target.trans);
		targetRotations.emplace_back(sample.target.rot);

		const auto updatedPose = ApplyTransform(sample.target, calibration);
		const Eigen::Vector3d hmdPoseSpace = sample.ref.rot * report.posOffsetM + sample.ref.trans;
		residuals.push_back((updatedPose.trans - hmdPoseSpace).norm());
	}

	report.residuals = ResidualStatsFor(std::move(residuals));
	report.legacyRmsPass = report.residuals.rmsM <= 0.100;

	if (report.validSampleCount >= 2) {
		report.refRangeM = refMax - refMin;
		report.targetRangeM = targetMax - targetMin;
		report.refSpanM = report.refRangeM.norm();
		report.targetSpanM = report.targetRangeM.norm();
	}

	for (size_t i = 0; i < targetRotations.size(); ++i) {
		for (size_t j = i + 1; j < targetRotations.size(); ++j) {
			const double angleDeg = targetRotations[i].angularDistance(targetRotations[j]) * (180.0 / EIGEN_PI);
			if (angleDeg > report.rotationSpanDeg) {
				report.rotationSpanDeg = angleDeg;
			}
		}
	}

	std::vector<DSample> rotationDeltas;
	for (size_t i = 0; i < m_samples.size(); ++i) {
		if (!m_samples[i].valid || !m_samples[i].pairedMotionValid) continue;
		for (size_t j = 0; j < i; ++j) {
			if (!m_samples[j].valid || !m_samples[j].pairedMotionValid) continue;
			const auto delta = DeltaRotationSamples(m_samples[i], m_samples[j]);
			if (delta.valid) rotationDeltas.push_back(delta);
		}
	}
	report.validRotationPairCount = static_cast<int>(rotationDeltas.size());
	if (rotationDeltas.size() >= 2) {
		Eigen::MatrixXd refPoints(rotationDeltas.size(), 2);
		Eigen::MatrixXd targetPoints(rotationDeltas.size(), 2);
		Eigen::Vector2d refCentroid = Eigen::Vector2d::Zero();
		Eigen::Vector2d targetCentroid = Eigen::Vector2d::Zero();
		for (size_t i = 0; i < rotationDeltas.size(); ++i) {
			refPoints.row(i) << rotationDeltas[i].ref.x(), rotationDeltas[i].ref.z();
			targetPoints.row(i) << rotationDeltas[i].target.x(), rotationDeltas[i].target.z();
			refCentroid += refPoints.row(i);
			targetCentroid += targetPoints.row(i);
		}
		refCentroid /= static_cast<double>(rotationDeltas.size());
		targetCentroid /= static_cast<double>(rotationDeltas.size());
		for (size_t i = 0; i < rotationDeltas.size(); ++i) {
			refPoints.row(i) -= refCentroid;
			targetPoints.row(i) -= targetCentroid;
		}
		const Eigen::Matrix2d cross = refPoints.transpose() * targetPoints;
		const Eigen::JacobiSVD<Eigen::Matrix2d> svd(cross);
		const auto values = svd.singularValues();
		report.rotationConditionRatio = SafeRatio(values.minCoeff(), values.maxCoeff());
	}

	std::vector<std::pair<Eigen::Vector3d, Eigen::Matrix3d>> translationDeltas;
	const Eigen::Matrix3d rotation = calibration.rotation();
	for (size_t i = 0; i < m_samples.size(); ++i) {
		if (!m_samples[i].valid || !m_samples[i].pairedMotionValid) continue;
		Sample s_i = m_samples[i];
		s_i.target.rot = rotation * s_i.target.rot;
		s_i.target.trans = rotation * s_i.target.trans;
		for (size_t j = 0; j < i; ++j) {
			if (!m_samples[j].valid || !m_samples[j].pairedMotionValid) continue;
			Sample s_j = m_samples[j];
			s_j.target.rot = rotation * s_j.target.rot;
			s_j.target.trans = rotation * s_j.target.trans;

			const auto QAi = s_i.ref.rot.transpose();
			const auto QAj = s_j.ref.rot.transpose();
			translationDeltas.push_back(
			    {QAj * (s_j.ref.trans - s_j.target.trans) - QAi * (s_i.ref.trans - s_i.target.trans), QAj - QAi});

			const auto QBi = s_i.target.rot.transpose();
			const auto QBj = s_j.target.rot.transpose();
			translationDeltas.push_back(
			    {QBj * (s_j.ref.trans - s_j.target.trans) - QBi * (s_i.ref.trans - s_i.target.trans), QBj - QBi});
		}
	}
	if (!translationDeltas.empty()) {
		Eigen::MatrixXd coefficients(translationDeltas.size() * 3, 3);
		for (size_t i = 0; i < translationDeltas.size(); ++i) {
			for (int axis = 0; axis < 3; ++axis) {
				coefficients.row(i * 3 + axis) = translationDeltas[i].second.row(axis);
			}
		}
		const Eigen::BDCSVD<Eigen::MatrixXd> svd(coefficients);
		const auto values = svd.singularValues();
		const double maxSv = values.size() ? values.maxCoeff() : 0.0;
		const double rankCut = std::max(1e-9, maxSv * 1e-6);
		int rank = 0;
		double minNonZero = std::numeric_limits<double>::infinity();
		for (int i = 0; i < values.size(); ++i) {
			if (values[i] > rankCut) {
				++rank;
				if (values[i] < minNonZero) minNonZero = values[i];
			}
		}
		report.translationRank = rank;
		report.translationConditionRatio = SafeRatio(minNonZero, maxSv);
	}

	if (includeHoldout && validSamples.size() >= 24) {
		std::vector<double> heldoutResiduals;
		const size_t kFolds = 4;
		for (size_t fold = 0; fold < kFolds; ++fold) {
			const size_t begin = (validSamples.size() * fold) / kFolds;
			const size_t end = (validSamples.size() * (fold + 1)) / kFolds;
			if (begin >= end || validSamples.size() - (end - begin) < 8) continue;

			CalibrationCalc train;
			train.enableStaticRecalibration = enableStaticRecalibration;
			for (size_t i = 0; i < validSamples.size(); ++i) {
				if (i >= begin && i < end) continue;
				train.PushSample(*validSamples[i]);
			}
			const Eigen::AffineCompact3d foldCalibration = train.ComputeCalibration(ignoreOutliers);
			const Eigen::Vector3d foldOffset = train.ComputeRefToTargetOffset(foldCalibration);
			for (size_t i = begin; i < end; ++i) {
				const Sample& sample = *validSamples[i];
				const auto updatedPose = ApplyTransform(sample.target, foldCalibration);
				const Eigen::Vector3d hmdPoseSpace = sample.ref.rot * foldOffset + sample.ref.trans;
				heldoutResiduals.push_back((updatedPose.trans - hmdPoseSpace).norm());
			}
		}
		report.holdoutResiduals = ResidualStatsFor(std::move(heldoutResiduals));
	}

	const double residualNoise = std::isfinite(report.residuals.madSigmaM) ? report.residuals.madSigmaM : 0.0;
	const double motionSpanTerm =
	    report.targetSpanM > 0.0 ? (0.04 * report.targetSpanM) / std::sqrt(std::max(1, report.validSampleCount)) : 0.0;
	const double noiseTerm = 5.0 * residualNoise;
	report.dynamicLimitM = std::min(0.100, std::max(0.025, std::max(noiseTerm, motionSpanTerm)));

	report.geometryPass = report.validSampleCount >= 24 && report.validRotationPairCount >= 10 &&
	                      report.rotationSpanDeg >= 20.0 && report.targetSpanM >= 0.05 && report.translationRank >= 2 &&
	                      report.translationConditionRatio >= 1e-4;

	report.robustResidualPass = report.residuals.count > 0 && report.residuals.p95M <= report.dynamicLimitM &&
	                            report.residuals.outlierFraction <= 0.20;

	report.holdoutPass = report.holdoutResiduals.count == 0 || (report.holdoutResiduals.p90M <= report.dynamicLimitM &&
	                                                            report.holdoutResiduals.rmsM <= 0.100);

	report.trackingHealthPass = report.trackingStaleSampleCount == 0 && report.trackingJumpSampleCount == 0;

	report.shadowDynamicPass = report.geometryPass && report.robustResidualPass && report.holdoutPass &&
	                           report.legacyRmsPass && report.trackingHealthPass;

	return report;
}

void CalibrationCalc::LogCalibrationQualitySnapshot(const char* label, const Eigen::AffineCompact3d& calibration,
                                                    bool includeHoldout, bool ignoreOutliers) const
{
	const CalibrationQualityReport q = EvaluateCalibrationQuality(calibration, includeHoldout, ignoreOutliers);
	const Eigen::Quaterniond rot(calibration.rotation());
	const Eigen::Vector3d euler = calibration.rotation().eulerAngles(2, 1, 0) * (180.0 / EIGEN_PI);

	char line1[1100];
	snprintf(line1, sizeof line1,
	         "[cal-quality][%s] samples=%zu valid=%d paired=%d rot_pairs=%d"
	         " ref_range_cm=(%.2f,%.2f,%.2f) target_range_cm=(%.2f,%.2f,%.2f)"
	         " ref_span_cm=%.2f target_span_cm=%.2f rot_span_deg=%.2f"
	         " stale_samples=%d jump_samples=%d max_pose_age_ms=%.1f max_pose_gap_ms=%.1f"
	         " rot_condition=%.9f trans_rank=%d trans_condition=%.9f"
	         " dynamic_limit_mm=%.2f legacy_pass=%s geometry_pass=%s robust_pass=%s holdout_pass=%s tracking_pass=%s "
	         "shadow_pass=%s",
	         label ? label : "candidate", q.sampleCount, q.validSampleCount, q.pairedSampleCount,
	         q.validRotationPairCount, q.refRangeM.x() * 100.0, q.refRangeM.y() * 100.0, q.refRangeM.z() * 100.0,
	         q.targetRangeM.x() * 100.0, q.targetRangeM.y() * 100.0, q.targetRangeM.z() * 100.0, q.refSpanM * 100.0,
	         q.targetSpanM * 100.0, q.rotationSpanDeg, q.trackingStaleSampleCount, q.trackingJumpSampleCount,
	         q.maxPoseAgeMs, q.maxPoseGapMs, q.rotationConditionRatio, q.translationRank, q.translationConditionRatio,
	         q.dynamicLimitM * 1000.0, Bool01(q.legacyRmsPass), Bool01(q.geometryPass), Bool01(q.robustResidualPass),
	         Bool01(q.holdoutPass), Bool01(q.trackingHealthPass), Bool01(q.shadowDynamicPass));
	Metrics::WriteLogAnnotation(line1);

	char line2[900];
	snprintf(line2, sizeof line2,
	         "[cal-quality][%s] transform trans_cm=(%.4f,%.4f,%.4f)"
	         " trans_mag_cm=%.4f quat=(%.9f,%.9f,%.9f,%.9f)"
	         " euler_zyx_deg=(%.4f,%.4f,%.4f)"
	         " offset_cm=(%.4f,%.4f,%.4f)"
	         " residual_mm median=%.3f mad_sigma=%.3f p90=%.3f p95=%.3f max=%.3f rms=%.3f outlier_frac=%.4f"
	         " holdout_mm count=%d median=%.3f p90=%.3f p95=%.3f rms=%.3f",
	         label ? label : "candidate", calibration.translation().x() * 100.0, calibration.translation().y() * 100.0,
	         calibration.translation().z() * 100.0, calibration.translation().norm() * 100.0, rot.w(), rot.x(), rot.y(),
	         rot.z(), euler.x(), euler.y(), euler.z(), q.posOffsetM.x() * 100.0, q.posOffsetM.y() * 100.0,
	         q.posOffsetM.z() * 100.0, q.residuals.medianM * 1000.0, q.residuals.madSigmaM * 1000.0,
	         q.residuals.p90M * 1000.0, q.residuals.p95M * 1000.0, q.residuals.maxM * 1000.0, q.residuals.rmsM * 1000.0,
	         q.residuals.outlierFraction, q.holdoutResiduals.count, q.holdoutResiduals.medianM * 1000.0,
	         q.holdoutResiduals.p90M * 1000.0, q.holdoutResiduals.p95M * 1000.0, q.holdoutResiduals.rmsM * 1000.0);
	Metrics::WriteLogAnnotation(line2);

	const CalibrationQualityVerdict verdict = EvaluateCalibrationQualityVerdict(q);
	char line3[360];
	snprintf(line3, sizeof line3,
	         "[cal-quality-verdict][%s] would_accept=%s reason=%s"
	         " legacy_pass=%s geometry_pass=%s robust_pass=%s holdout_pass=%s",
	         label ? label : "candidate", Bool01(verdict.wouldAccept), verdict.reason, Bool01(q.legacyRmsPass),
	         Bool01(q.geometryPass), Bool01(q.robustResidualPass), Bool01(q.holdoutPass));
	Metrics::WriteLogAnnotation(line3);

	openvr_pair::common::RuntimeCalibrationHealthSample runtimeHealth{};
	runtimeHealth.valid = true;
	runtimeHealth.sampleCount = static_cast<int>(q.sampleCount);
	runtimeHealth.validSampleCount = q.validSampleCount;
	runtimeHealth.pairedSampleCount = q.pairedSampleCount;
	runtimeHealth.trackingHealthPass = q.trackingHealthPass;
	runtimeHealth.shadowDynamicPass = q.shadowDynamicPass;
	runtimeHealth.residualRmsMm = q.residuals.rmsM * 1000.0;
	runtimeHealth.residualP95Mm = q.residuals.p95M * 1000.0;
	runtimeHealth.holdoutRmsMm = q.holdoutResiduals.rmsM * 1000.0;
	openvr_pair::common::RecordRuntimeCalibrationHealth(runtimeHealth);
}

// Given:
//   R - the reference pose (in reference world space)
//   T - the target pose (in target world space)
//   C - the true calibration (target world -> reference world)
// We assume that there is some "static target pose" S s.t.:
// R * S = C * T (we'll call this the static target pose)
// To compute S:
// S = R^-1 * C * T
// To compute C:
// R * S * T^-1 = C

namespace {
class PoseAverager
{
private:
	Eigen::Matrix<double, 4, Eigen::Dynamic> quatAvg;
	Eigen::Vector3d accum = Eigen::Vector3d::Zero();
	int i = 0;

public:
	PoseAverager(size_t n_samples) { quatAvg.resize(4, n_samples); }

	template <typename P> void Push(const P& pose)
	{
		const Eigen::Quaterniond rot(pose.rotation());
		quatAvg.col(i++) = Eigen::Vector4d(rot.w(), rot.x(), rot.y(), rot.z());
		accum += pose.translation();
	}

	Eigen::AffineCompact3d Average()
	{
		// https://stackoverflow.com/a/27410865/36723
		auto quatT = quatAvg.transpose();
		Eigen::Matrix4d quatMul = quatAvg * quatT;
		Eigen::SelfAdjointEigenSolver<Eigen::Matrix4d> solver;
		solver.compute(quatMul);

		Eigen::Vector4d quatAvgV = solver.eigenvectors().col(3).real().normalized();
		Eigen::Quaterniond avgQ(quatAvgV(0), quatAvgV(1), quatAvgV(2), quatAvgV(3));
		avgQ.normalize();

		Eigen::AffineCompact3d pose(avgQ);
		pose.pretranslate(accum * (1.0 / i));

		return pose;
	}

	template <typename XS, typename F>
	static Eigen::AffineCompact3d AverageFor(const XS& samples, const F& poseProvider)
	{
		int sampleCount = 0;

		for (auto& sample : samples) {
			if (!sample.valid) continue;

			sampleCount++;
		}

		PoseAverager accum(sampleCount);

		for (auto& sample : samples) {
			if (!sample.valid) continue;
			auto pose = poseProvider(sample);
			accum.Push(pose);
		}

		return accum.Average();
	}
};
} // namespace

// S = R^-1 * C * T
Eigen::AffineCompact3d CalibrationCalc::EstimateRefToTargetPose(const Eigen::AffineCompact3d& calibration) const
{
	auto avg = PoseAverager::AverageFor(m_samples, [&](const auto& sample) {
		return Eigen::Affine3d(sample.ref.ToAffine().inverse() * calibration * sample.target.ToAffine());
	});

	return avg;
}

// S = R^-1 * C * T
// R * S * T^-1 = C

// R * (R^-1 * C * T) * T^-1 = C

/*
 * This calibration routine attempts to use the estimated refToTargetPose to derive the
 * playspace calibration based on the relative position of reference and target device.
 * This computation can be performed even when the devices are not moving.
 */
bool CalibrationCalc::CalibrateByRelPose(Eigen::AffineCompact3d& out) const
{
	// R * S * T^-1 = C
	out = PoseAverager::AverageFor(m_samples, [&](const auto& sample) {
		return Eigen::AffineCompact3d(sample.ref.ToAffine() * m_refToTargetPose * sample.target.ToAffine().inverse());
	});

	return true;
}

namespace {
// RAII helper that temporarily prepends frozen rotation-phase samples onto
// a live sample buffer for the duration of a one-shot solve. ComputeOneshot
// (and every helper it calls — DetectOutliers, CalibrateRotation,
// ComputeAxisVariance, ValidateCalibration, ComputeRefToTargetOffset,
// ComputeInstantOffset) iterates m_samples directly. Rather than threading
// a "use union" parameter through every call site, we splice the frozen
// samples in at construction and pop them off in the destructor. Result:
// the math sees rotation+translation samples as one deque without any
// internal helper having to know about the phase split. ComputeOneshot's
// metric pushes (samplesInBuffer, posOffset_lastSample, etc.) also reflect
// the unified buffer, which is what we want for triage — the math ran on
// the union, so the diagnostic numbers should describe the union.
//
// Frozen samples go at the FRONT so m_samples.back() (used by
// ComputeInstantOffset) still references the most recent live sample.
class RotationFreezeSplice
{
	std::deque<Sample>& m_live;
	size_t m_count;

public:
	RotationFreezeSplice(std::deque<Sample>& live, const std::deque<Sample>& frozen)
	    : m_live(live), m_count(frozen.size())
	{
		if (m_count) m_live.insert(m_live.begin(), frozen.begin(), frozen.end());
	}
	~RotationFreezeSplice()
	{
		if (m_count) m_live.erase(m_live.begin(), m_live.begin() + m_count);
	}
	RotationFreezeSplice(const RotationFreezeSplice&) = delete;
	RotationFreezeSplice& operator=(const RotationFreezeSplice&) = delete;
};
} // namespace

bool CalibrationCalc::ComputeOneshot(const bool ignoreOutliers)
{
	RotationFreezeSplice splice(m_samples, m_rotationFrozen);
	auto calibration = ComputeCalibration(ignoreOutliers);
	LogCalibrationQualitySnapshot("oneshot_legacy_candidate", calibration, true, ignoreOutliers);

	bool valid = ValidateCalibration(calibration);

	if (valid) {
		m_estimatedTransformation = calibration;
		m_isValid = true;
		return true;
	}
	else {
		CalCtx.Log("Not updating: Low-quality calibration result\n");
		return false;
	}
}

void CalibrationCalc::ComputeInstantOffset()
{
	const auto& latestSample = m_samples.back();

	const auto updatedPose = ApplyTransform(latestSample.target, m_estimatedTransformation);

	const auto hmdOriginPos = updatedPose.trans - latestSample.ref.trans;
	const auto hmdSpace = latestSample.ref.rot.inverse() * hmdOriginPos;

	Metrics::posOffset_lastSample.Push(hmdSpace * 1000);
}

bool CalibrationCalc::ComputeIncremental(bool& lerp, double threshold, double relPoseMaxError,
                                         const bool ignoreOutliers)
{
	Metrics::RecordTimestamp();
	m_lastComputeUsedRelPose = false;
	static double s_lastHoldoutQualityLogAt = -1e9;
	const bool includeHoldoutThisPass = Metrics::CurrentTime - s_lastHoldoutQualityLogAt >= 5.0;
	if (includeHoldoutThisPass) {
		s_lastHoldoutQualityLogAt = Metrics::CurrentTime;
	}

	if (lockRelativePosition) {
		Eigen::AffineCompact3d byRelPose;
		double relPoseError = INFINITY;
		Eigen::Vector3d relPosOffset;
		if (CalibrateByRelPose(byRelPose) && ValidateCalibration(byRelPose, &relPoseError, &relPosOffset)) {
			if (includeHoldoutThisPass) {
				LogCalibrationQualitySnapshot("relpose_locked_candidate", byRelPose, true, ignoreOutliers);
			}

			Metrics::posOffset_byRelPose.Push(relPosOffset * 1000);
			Metrics::error_byRelPose.Push(relPoseError * 1000);

			m_isValid = true;
			m_lastComputeUsedRelPose = true;
			m_estimatedTransformation = byRelPose;
			m_lastCandidateRetargetingErrorM = relPoseError;
			return true;
		}
	}

	double priorCalibrationError = INFINITY;
	Eigen::Vector3d priorPosOffset;
	if (m_isValid && ValidateCalibration(m_estimatedTransformation, &priorCalibrationError, &priorPosOffset)) {
		if (includeHoldoutThisPass) {
			LogCalibrationQualitySnapshot("legacy_current", m_estimatedTransformation, true, ignoreOutliers);
		}
		Metrics::posOffset_currentCal.Push(priorPosOffset * 1000);
		Metrics::error_currentCal.Push(priorCalibrationError * 1000);
		m_lastPriorRetargetingErrorM = priorCalibrationError;
	}

	double newError = INFINITY;
	bool newCalibrationValid = false;
	Eigen::AffineCompact3d byRelPose;
	Eigen::AffineCompact3d calibration;
	bool usingRelPose = false;
	double relPoseError = INFINITY;

	if (enableStaticRecalibration && CalibrateByRelPose(byRelPose)) {
		if (includeHoldoutThisPass) {
			LogCalibrationQualitySnapshot("relpose_candidate", byRelPose, true, ignoreOutliers);
		}
		Eigen::Vector3d relPosOffset;
		if (ValidateCalibration(byRelPose, &relPoseError, &relPosOffset)) {
			Metrics::posOffset_byRelPose.Push(relPosOffset * 1000);
			Metrics::error_byRelPose.Push(relPoseError * 1000);

			if (relPoseError < 0.010 || m_relativePosCalibrated && relPoseError < 0.025) {
				if (relPoseError * threshold >= priorCalibrationError) {
					return false;
				}

				if (relPoseError > relPoseMaxError) {
					return false;
				}

				newCalibrationValid = true;
				usingRelPose = true;
				newError = relPoseError;
				calibration = byRelPose;
			}
		}
	}

	double newVariance = 0;
	bool shouldRapidCorrect = true;
	if (!newCalibrationValid) {
		calibration = ComputeCalibration(ignoreOutliers);
		if (includeHoldoutThisPass) {
			LogCalibrationQualitySnapshot("legacy_full_candidate", calibration, true, ignoreOutliers);
		}

		newVariance = ComputeAxisVariance(calibration)(1);
		Metrics::axisIndependence.Push(newVariance);

		if (newVariance < AxisVarianceThreshold && newVariance < m_axisVariance) {
			newCalibrationValid = false;
			shouldRapidCorrect = false;
		}
		else {
			newCalibrationValid = ValidateCalibration(calibration, &newError, &m_posOffset);
			Metrics::posOffset_rawComputed.Push(m_posOffset * 1000);
		}

		if (m_isValid) {
			if (priorCalibrationError < newError * threshold) {
				newCalibrationValid = false;
				shouldRapidCorrect = false;
			}
		}

		Metrics::error_rawComputed.Push(newError * 1000);

		ComputeInstantOffset();
	}

	if (!newCalibrationValid && shouldRapidCorrect) {
		double existingPoseErrorUsingRelPosition =
		    RetargetingErrorRMS(m_refToTargetPose.translation(), m_estimatedTransformation);
		Metrics::error_currentCalRelPose.Push(existingPoseErrorUsingRelPosition * 1000);
		if (relPoseError * threshold < existingPoseErrorUsingRelPosition ||
		    newCalibrationValid && relPoseError < newError) {
			newCalibrationValid = true;
			usingRelPose = true;
			newError = relPoseError;
			calibration = byRelPose;
		}
	}

	if (newCalibrationValid) {
		lerp = m_isValid;
		m_relativePosCalibrated = m_relativePosCalibrated || newError < 0.005;
		if (!m_isValid) {
			CalCtx.Log("Applying initial transformation...");
		}
		else if (m_relativePosCalibrated) {
			CalCtx.Log("Applying updated transformation...");
		}
		else {
			CalCtx.Log("Applying temporary transformation...");
		}

		m_isValid = true;
		m_lastComputeUsedRelPose = usingRelPose;
		m_estimatedTransformation = calibration;
		m_axisVariance = newVariance;

		if (!usingRelPose) {
			m_refToTargetPose = EstimateRefToTargetPose(m_estimatedTransformation);
		}

		Metrics::calibrationApplied.Push(!usingRelPose);
		m_lastCandidateRetargetingErrorM = newError;
		return true;
	}
	else {
		return false;
	}
}
