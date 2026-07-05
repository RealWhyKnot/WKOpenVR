// Inter-stack time-offset estimation from a full-rate phantom_replay_v1
// recording. The HMD (target universe) and the head-mounted witness tracker
// (reference universe) ride the same rigid body, so their angular-speed
// profiles are the same signal up to a time offset between the two tracking
// stacks. Angular speed is frame-invariant under fixed rigid transforms,
// which makes it comparable across the two universes without knowing the
// calibration. The offset shows up as the lag of the peak of the normalized
// cross-correlation; the practical payoff is measured directly as the
// relative-translation MAD (the auto-lock wobble metric) paired at lag zero
// versus paired at the recovered lag.

#include <gtest/gtest.h>

#include "AutoLockHysteresis.h"

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <deque>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

namespace {

struct PoseSample
{
	double tSec = 0.0;
	Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
	Eigen::Vector3d p = Eigen::Vector3d::Zero();
};

struct SpeedSample
{
	double tSec = 0.0;
	double radps = 0.0;
};

// Angular speed by finite-differencing consecutive orientation samples.
// Gaps (dropouts, retention seams) are skipped rather than bridged.
std::vector<SpeedSample> AngularSpeeds(const std::vector<PoseSample>& poses)
{
	std::vector<SpeedSample> out;
	out.reserve(poses.size());
	for (size_t i = 1; i < poses.size(); ++i) {
		const double dt = poses[i].tSec - poses[i - 1].tSec;
		if (dt <= 1e-4 || dt > 0.5) continue;
		SpeedSample s;
		s.tSec = 0.5 * (poses[i].tSec + poses[i - 1].tSec);
		s.radps = poses[i - 1].q.angularDistance(poses[i].q) / dt;
		out.push_back(s);
	}
	return out;
}

// Linear interpolation of a speed series onto a uniform grid. Grid cells
// outside the series (or across >0.5 s gaps) become NaN and are excluded
// from the correlation.
std::vector<double> ResampleToGrid(const std::vector<SpeedSample>& s, double t0, double dt, size_t n)
{
	std::vector<double> out(n, std::numeric_limits<double>::quiet_NaN());
	if (s.size() < 2) return out;
	size_t j = 0;
	for (size_t i = 0; i < n; ++i) {
		const double t = t0 + dt * static_cast<double>(i);
		while (j + 1 < s.size() && s[j + 1].tSec < t)
			++j;
		if (j + 1 >= s.size()) break;
		const auto& a = s[j];
		const auto& b = s[j + 1];
		if (t < a.tSec || b.tSec - a.tSec > 0.5) continue;
		const double f = (t - a.tSec) / (b.tSec - a.tSec);
		out[i] = a.radps + f * (b.radps - a.radps);
	}
	return out;
}

// Pearson correlation of gridA[i] vs gridB[i + lag] over cells finite in both.
double CorrelationAtLag(const std::vector<double>& a, const std::vector<double>& b, int lag)
{
	double sa = 0, sb = 0, saa = 0, sbb = 0, sab = 0;
	size_t n = 0;
	const int len = static_cast<int>(a.size());
	for (int i = 0; i < len; ++i) {
		const int k = i + lag;
		if (k < 0 || k >= len) continue;
		const double x = a[static_cast<size_t>(i)];
		const double y = b[static_cast<size_t>(k)];
		if (!std::isfinite(x) || !std::isfinite(y)) continue;
		sa += x;
		sb += y;
		saa += x * x;
		sbb += y * y;
		sab += x * y;
		++n;
	}
	if (n < 100) return 0.0;
	const double dn = static_cast<double>(n);
	const double cov = sab - sa * sb / dn;
	const double va = saa - sa * sa / dn;
	const double vb = sbb - sb * sb / dn;
	if (va <= 0.0 || vb <= 0.0) return 0.0;
	return cov / std::sqrt(va * vb);
}

struct XcorrEstimate
{
	double peakLagMs = 0.0; // positive: stream B lags stream A
	double peakCorr = 0.0;
	double rateA = 0.0;
	double rateB = 0.0;
	double overlapSec = 0.0; // usable shared-motion window
	double madLag0Mm = 0.0;
	double madShiftedMm = 0.0;
	std::vector<double> windowLagsMs;
};

constexpr double kGridDtSec = 0.001;
constexpr int kMaxLagMs = 200;

// Peak lag of the normalized cross-correlation between the two streams'
// angular-speed profiles, with sub-millisecond parabolic refinement.
// Convention: positive lag means B's signal arrives `lag` later than A's,
// i.e. B at time t matches A at time (t - lag).
double PeakLagMs(const std::vector<double>& gridA, const std::vector<double>& gridB, double* corrOut)
{
	int bestLag = 0;
	double bestCorr = -2.0;
	std::vector<double> corr(2 * kMaxLagMs + 1, 0.0);
	for (int lag = -kMaxLagMs; lag <= kMaxLagMs; ++lag) {
		const double c = CorrelationAtLag(gridA, gridB, lag);
		const int idx = lag + kMaxLagMs;
		corr[static_cast<size_t>(idx)] = c;
		if (c > bestCorr) {
			bestCorr = c;
			bestLag = lag;
		}
	}
	double refined = static_cast<double>(bestLag);
	if (bestLag > -kMaxLagMs && bestLag < kMaxLagMs) {
		const int midIdx = bestLag + kMaxLagMs; // in (0, 2*kMaxLagMs) here
		const size_t mid = static_cast<size_t>(midIdx);
		const double c0 = corr[mid - 1];
		const double c1 = corr[mid];
		const double c2 = corr[mid + 1];
		const double denom = c0 - 2.0 * c1 + c2;
		if (std::abs(denom) > 1e-12) refined += 0.5 * (c0 - c2) / denom;
	}
	if (corrOut) *corrOut = bestCorr;
	return refined;
}

PoseSample InterpolatePose(const PoseSample& a, const PoseSample& b, double t)
{
	const double f = (t - a.tSec) / (b.tSec - a.tSec);
	PoseSample out;
	out.tSec = t;
	out.p = a.p + f * (b.p - a.p);
	out.q = a.q.slerp(f, b.q);
	return out;
}

// Interpolated pose of `poses` at time t; false near gaps or outside range.
bool PoseAt(const std::vector<PoseSample>& poses, double t, size_t& cursor, PoseSample& out)
{
	while (cursor + 1 < poses.size() && poses[cursor + 1].tSec < t)
		++cursor;
	if (cursor + 1 >= poses.size()) return false;
	const auto& a = poses[cursor];
	const auto& b = poses[cursor + 1];
	if (t < a.tSec || b.tSec - a.tSec > 0.5) return false;
	out = InterpolatePose(a, b, t);
	return true;
}

// The wobble benchmark: relative pose A^-1 * B paired at B's sample times,
// with A read at (t - lagMs). Sliding 20-sample windows mirror the live
// auto-lock detector's history; the reported figure is the median window MAD.
double RelPoseMadMm(const std::vector<PoseSample>& a, const std::vector<PoseSample>& b, double lagMs)
{
	std::deque<Eigen::AffineCompact3d> window;
	std::vector<double> mads;
	size_t cursor = 0;
	for (const auto& bs : b) {
		PoseSample as;
		if (!PoseAt(a, bs.tSec - lagMs / 1000.0, cursor, as)) continue;
		Eigen::AffineCompact3d ta(as.q);
		ta.translation() = as.p;
		Eigen::AffineCompact3d tb(bs.q);
		tb.translation() = bs.p;
		window.push_back(Eigen::AffineCompact3d(ta.inverse() * tb));
		if (window.size() > 20) window.pop_front();
		if (window.size() == 20) mads.push_back(spacecal::autolock::RobustTranslDeviation(window));
	}
	if (mads.empty()) return 0.0;
	const size_t mid = mads.size() / 2;
	std::nth_element(mads.begin(), mads.begin() + mid, mads.end());
	return mads[mid] * 1000.0;
}

XcorrEstimate EstimateTimeOffset(const std::vector<PoseSample>& a, const std::vector<PoseSample>& b)
{
	XcorrEstimate est;
	const auto speedA = AngularSpeeds(a);
	const auto speedB = AngularSpeeds(b);
	if (speedA.size() >= 2) est.rateA = static_cast<double>(speedA.size()) / (speedA.back().tSec - speedA.front().tSec);
	if (speedB.size() >= 2) est.rateB = static_cast<double>(speedB.size()) / (speedB.back().tSec - speedB.front().tSec);
	if (speedA.size() < 50 || speedB.size() < 50) return est;

	const double t0 = std::max(speedA.front().tSec, speedB.front().tSec);
	const double t1 = std::min(speedA.back().tSec, speedB.back().tSec);
	est.overlapSec = t1 - t0;
	// A few seconds of shared motion is enough for a preliminary estimate
	// (hidden-tracker captures only overlap briefly); peak_corr is the
	// confidence signal, not the window length.
	if (est.overlapSec < 5.0) return est;
	const size_t n = static_cast<size_t>(est.overlapSec / kGridDtSec);
	const auto gridA = ResampleToGrid(speedA, t0, kGridDtSec, n);
	const auto gridB = ResampleToGrid(speedB, t0, kGridDtSec, n);

	est.peakLagMs = PeakLagMs(gridA, gridB, &est.peakCorr);

	// Per-window stability: same estimator over 300 s slices.
	constexpr size_t kWindowCells = static_cast<size_t>(300.0 / kGridDtSec);
	for (size_t start = 0; start + kWindowCells <= n; start += kWindowCells) {
		std::vector<double> wa(gridA.begin() + start, gridA.begin() + start + kWindowCells);
		std::vector<double> wb(gridB.begin() + start, gridB.begin() + start + kWindowCells);
		double c = 0.0;
		const double lag = PeakLagMs(wa, wb, &c);
		if (c > 0.2) est.windowLagsMs.push_back(lag);
	}

	est.madLag0Mm = RelPoseMadMm(a, b, 0.0);
	est.madShiftedMm = RelPoseMadMm(a, b, est.peakLagMs);
	return est;
}

// --- phantom_replay_v1 parsing -------------------------------------------

// Columns: time_ms,device_id,serial,class,controller_role,body_role,
// dropout_enabled,pose_valid,connected,result,x,y,z,qw,qx,qy,qz,vx,vy,vz
bool ParsePhantomRow(const std::string& line, const std::string& serial, PoseSample& out)
{
	if (line.empty() || line[0] == '#' || line[0] == 't') return false;
	const char* s = line.c_str();
	double fields[7];
	double tMs = 0.0;
	int col = 0;
	const char* fieldStart = s;
	bool serialMatch = false;
	bool valid = false;
	for (const char* c = s;; ++c) {
		if (*c != ',' && *c != '\0') continue;
		const size_t len = static_cast<size_t>(c - fieldStart);
		switch (col) {
			case 0:
				tMs = std::strtod(fieldStart, nullptr);
				break;
			case 2:
				serialMatch = serial.compare(0, serial.size(), fieldStart, len) == 0;
				if (!serialMatch) return false;
				break;
			case 7:
				valid = len == 1 && fieldStart[0] == '1';
				if (!valid) return false;
				break;
			case 10:
			case 11:
			case 12:
			case 13:
			case 14:
			case 15:
			case 16:
				fields[col - 10] = std::strtod(fieldStart, nullptr);
				break;
			default:
				break;
		}
		if (*c == '\0') break;
		fieldStart = c + 1;
		++col;
	}
	if (col < 16) return false;
	out.tSec = tMs / 1000.0;
	out.p = Eigen::Vector3d(fields[0], fields[1], fields[2]);
	out.q = Eigen::Quaterniond(fields[3], fields[4], fields[5], fields[6]);
	if (std::abs(out.q.norm() - 1.0) > 0.01) return false;
	out.q.normalize();
	return true;
}

std::string EnvOr(const char* name, const char* fallback)
{
	const char* raw = std::getenv(name);
	return (raw && *raw) ? raw : fallback;
}

} // namespace

// Tool self-check: two pose streams of the same rigid trajectory, observed
// through different fixed rigid transforms, at different sample rates, with
// a known 30 ms time shift and a small mounting offset. The estimator must
// recover the shift (sign included: the shifted stream is B, so peak lag is
// positive) and pairing at the recovered lag must beat pairing at lag zero.
TEST(TimeOffsetXcorrTest, RecoversInjectedShiftOnSyntheticStreams)
{
	constexpr double kShiftSec = 0.030;
	const Eigen::Quaterniond mountRot(Eigen::AngleAxisd(0.4, Eigen::Vector3d(0.2, 1.0, 0.1).normalized()));
	const Eigen::Vector3d mountOffset(0.0, 0.10, -0.08);

	auto headPose = [&](double t) {
		const double yaw = 0.8 * std::sin(2.0 * EIGEN_PI * 0.5 * t) + 0.5 * std::sin(2.0 * EIGEN_PI * 1.3 * t + 1.0);
		const double pitch = 0.4 * std::sin(2.0 * EIGEN_PI * 0.9 * t + 0.7);
		PoseSample s;
		s.q = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitX());
		s.p = Eigen::Vector3d(0.3 * std::sin(2.0 * EIGEN_PI * 0.3 * t), 1.6, 0.2 * std::cos(2.0 * EIGEN_PI * 0.4 * t));
		return s;
	};

	std::vector<PoseSample> hmd;
	for (int i = 0; i < 120 * 90; ++i) {
		const double t = static_cast<double>(i) / 90.0;
		PoseSample s = headPose(t);
		s.tSec = t;
		hmd.push_back(s);
	}
	std::vector<PoseSample> witness;
	for (int i = 0; i < 120 * 30; ++i) {
		// The witness reports the same physical motion kShiftSec late, seen
		// through a fixed mount transform (different universe + mounting).
		const double t = static_cast<double>(i) / 30.0;
		PoseSample head = headPose(t - kShiftSec);
		PoseSample s;
		s.tSec = t;
		s.q = head.q * mountRot;
		s.p = head.p + head.q * mountOffset;
		witness.push_back(s);
	}

	const XcorrEstimate est = EstimateTimeOffset(hmd, witness);
	EXPECT_GT(est.peakCorr, 0.9);
	EXPECT_NEAR(est.peakLagMs, kShiftSec * 1000.0, 2.0);
	EXPECT_LT(est.madShiftedMm, est.madLag0Mm);
	EXPECT_LT(est.madShiftedMm, 1.0) << "perfect synthetic rigid pair must be near-zero MAD at the true lag";
}

// Env-driven entry point over a recorded phantom_replay_v1 capture. Prints
// one [dt-xcorr] line; skipped unless WKOPENVR_XCORR_PHANTOM points at a CSV.
TEST(TimeOffsetXcorrTest, EstimateFromPhantomRecordingWhenRequested)
{
	const std::string path = EnvOr("WKOPENVR_XCORR_PHANTOM", "");
	if (path.empty()) {
		GTEST_SKIP() << "Set WKOPENVR_XCORR_PHANTOM=<phantom_replay csv> to estimate the inter-stack time offset.";
	}
	const std::string hmdSerial = EnvOr("WKOPENVR_XCORR_HMD_SERIAL", "1PASH5D1P17365");
	const std::string witnessSerial = EnvOr("WKOPENVR_XCORR_WITNESS_SERIAL", "LHR-10268F5C");

	std::ifstream in(path);
	ASSERT_TRUE(in) << path;
	std::vector<PoseSample> hmd, witness;
	std::string line;
	while (std::getline(in, line)) {
		PoseSample s;
		if (ParsePhantomRow(line, hmdSerial, s))
			hmd.push_back(s);
		else if (ParsePhantomRow(line, witnessSerial, s))
			witness.push_back(s);
	}
	ASSERT_GT(hmd.size(), 1000u) << "HMD serial " << hmdSerial << " not found at usable rate";
	ASSERT_GT(witness.size(), 100u) << "witness serial " << witnessSerial << " not found at usable rate";

	const XcorrEstimate est = EstimateTimeOffset(hmd, witness);
	std::ostringstream windows;
	for (size_t i = 0; i < est.windowLagsMs.size(); ++i)
		windows << (i ? ";" : "") << est.windowLagsMs[i];
	std::cout << "[dt-xcorr] " << path << " hmd=" << hmdSerial << " witness=" << witnessSerial
	          << " peak_lag_ms=" << est.peakLagMs << " peak_corr=" << est.peakCorr << " window_lags_ms=["
	          << windows.str() << "]"
	          << " mad_lag0_mm=" << est.madLag0Mm << " mad_shifted_mm=" << est.madShiftedMm
	          << " hmd_rate_hz=" << est.rateA << " witness_rate_hz=" << est.rateB << " overlap_s=" << est.overlapSec
	          << "\n";
	EXPECT_GT(est.peakCorr, 0.0) << "no usable shared-motion overlap (need >= 5 s of both streams valid; overlap was "
	                             << est.overlapSec << " s at rates " << est.rateA << "/" << est.rateB << " Hz)";
}
