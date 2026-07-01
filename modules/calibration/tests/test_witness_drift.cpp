// Unit tests for the offline witness-drift oracle (WitnessDriftReplay.h).
//
// The correction MATH is tested directly against RunCorrectionModel with injected
// drift vectors (deterministic, no calibration solve). The end-to-end
// ComputeWitnessDrift path -- which reconstructs the calibration with
// CalibrationCalc -- is validated on real recordings; here it is only smoke-
// tested (calibrates + runs), since fitting a synthetic the solver reproduces to
// sub-mm precision is brittle and orthogonal to the correction logic.

#include "WitnessDriftReplay.h"

#include <gtest/gtest.h>

#include <Eigen/Geometry>
#include <cmath>
#include <vector>

using spacecal::replay::ComputeWitnessDrift;
using spacecal::replay::CorrectionModelResult;
using spacecal::replay::LoadedRecording;
using spacecal::replay::ReplayRow;
using spacecal::replay::RunCorrectionModel;
using spacecal::replay::WitnessDriftOptions;

namespace {

std::vector<double> Dts(std::size_t n, double dt)
{
	return std::vector<double>(n, dt);
}

std::vector<Eigen::Vector3d> RampX(int n, double endM)
{
	std::vector<Eigen::Vector3d> v;
	v.reserve(n);
	for (int i = 0; i < n; ++i)
		v.emplace_back(endM * (static_cast<double>(i) / (n - 1)), 0.0, 0.0);
	return v;
}

std::vector<Eigen::Vector3d> ConstX(int n, double m)
{
	return std::vector<Eigen::Vector3d>(n, Eigen::Vector3d(m, 0.0, 0.0));
}

} // namespace

// ---- Pure correction-model math (deterministic) ---------------------------

TEST(CorrectionModelTest, RampDriftIsLargelyRemoved)
{
	const auto drift = RampX(300, 0.045); // 0 -> 45 mm ramp, under the 30 cm cap
	WitnessDriftOptions opts;             // shipped defaults: 3 mm/s slew, 3 mm dead-band
	const CorrectionModelResult r = RunCorrectionModel(drift, Dts(drift.size(), 1.0 / 30.0), opts);

	EXPECT_GT(r.uncorrectedRmsMm, 20.0);
	EXPECT_LT(r.correctedRmsMm, r.uncorrectedRmsMm);
	EXPECT_GT(r.reductionPct, 20.0) << "a slow sub-30 cm drift should be largely removed";
}

TEST(CorrectionModelTest, CorrectionOffIsIdentity)
{
	const auto drift = RampX(300, 0.045);
	WitnessDriftOptions opts;
	opts.applyContinuousCorrection = false;
	const CorrectionModelResult r = RunCorrectionModel(drift, Dts(drift.size(), 1.0 / 30.0), opts);

	EXPECT_NEAR(r.correctedRmsMm, r.uncorrectedRmsMm, 1e-9);
	EXPECT_NEAR(r.subCapCorrectedRmsMm, r.subCapUncorrectedRmsMm, 1e-9);
}

TEST(CorrectionModelTest, BeyondThirtyCentimetreCapNotChased)
{
	const auto drift = ConstX(200, 0.5); // 0.5 m: a relocalization, recovery's job
	WitnessDriftOptions opts;
	const CorrectionModelResult r = RunCorrectionModel(drift, Dts(drift.size(), 1.0 / 30.0), opts);

	EXPECT_GT(r.uncorrectedPeakMm, 400.0);
	EXPECT_NEAR(r.correctedPeakMm, r.uncorrectedPeakMm, 1e-6) << "above the cap the slow loop must not chase";
	EXPECT_EQ(r.subCapSamples, 0) << "all samples are above the 30 cm cap";
}

TEST(CorrectionModelTest, JitterInsideDeadbandNotChased)
{
	std::vector<Eigen::Vector3d> drift;
	for (int i = 0; i < 200; ++i)
		drift.emplace_back(0.002 * std::sin(i * 0.5), 0.0, 0.0); // <= 2 mm < 3 mm band
	WitnessDriftOptions opts;
	const CorrectionModelResult r = RunCorrectionModel(drift, Dts(drift.size(), 1.0 / 30.0), opts);

	EXPECT_NEAR(r.correctedRmsMm, r.uncorrectedRmsMm, 1e-9) << "jitter under the dead-band is left alone";
}

TEST(CorrectionModelTest, FasterSlewTracksBetter)
{
	const auto drift = RampX(400, 0.040);
	WitnessDriftOptions slow;
	slow.correctionSlewMps = 0.002;
	WitnessDriftOptions fast;
	fast.correctionSlewMps = 0.008;

	const CorrectionModelResult rSlow = RunCorrectionModel(drift, Dts(drift.size(), 1.0 / 30.0), slow);
	const CorrectionModelResult rFast = RunCorrectionModel(drift, Dts(drift.size(), 1.0 / 30.0), fast);

	EXPECT_LT(rFast.correctedRmsMm, rSlow.correctedRmsMm);
}

TEST(CorrectionModelTest, LargerDeadbandLeavesMoreResidual)
{
	const auto drift = RampX(300, 0.020); // small drift where the dead-band matters
	WitnessDriftOptions tight;
	tight.correctionDeadbandM = 0.001;
	WitnessDriftOptions wide;
	wide.correctionDeadbandM = 0.010;

	const CorrectionModelResult rTight = RunCorrectionModel(drift, Dts(drift.size(), 1.0 / 30.0), tight);
	const CorrectionModelResult rWide = RunCorrectionModel(drift, Dts(drift.size(), 1.0 / 30.0), wide);

	EXPECT_LT(rTight.correctedRmsMm, rWide.correctedRmsMm);
}

// ---- End-to-end smoke on a synthetic recording ----------------------------

namespace {

Pose MakePose(const Eigen::Affine3d& a)
{
	Pose p;
	p.rot = a.linear();
	p.trans = a.translation();
	return p;
}

Eigen::Affine3d HeadRich(int i)
{
	const double fi = static_cast<double>(i);
	Eigen::Affine3d h = Eigen::Affine3d::Identity();
	h.linear() = (Eigen::AngleAxisd(0.5 * std::sin(fi * 0.30), Eigen::Vector3d::UnitX()) *
	              Eigen::AngleAxisd(0.5 * std::sin(fi * 0.23 + 1.0), Eigen::Vector3d::UnitY()) *
	              Eigen::AngleAxisd(0.5 * std::sin(fi * 0.19 + 2.0), Eigen::Vector3d::UnitZ()))
	                 .toRotationMatrix();
	h.translation() = Eigen::Vector3d(0.3 * std::sin(fi * 0.27), 0.25 * std::cos(fi * 0.22), 0.2 * std::sin(fi * 0.15));
	return h;
}

ReplayRow MakeRow(double ts, const Eigen::Affine3d& hmd, const Eigen::Affine3d& tracker, bool reloc = false)
{
	ReplayRow r;
	r.timestamp = ts;
	r.ref = MakePose(hmd); // ref == hmd, target == head_tracker, as in the recordings
	r.target = MakePose(tracker);
	r.hmd = MakePose(hmd);
	r.headTracker = MakePose(tracker);
	r.hasHmdPose = true;
	r.headTrackerValid = true;
	r.relocDetected = reloc;
	return r;
}

LoadedRecording SyntheticRecording(int n, int relocCount)
{
	LoadedRecording rec;
	rec.formatVersion = 5;
	rec.hasLockedSnapColumns = true;
	Eigen::Affine3d U = Eigen::Affine3d::Identity();
	U.linear() = Eigen::AngleAxisd(0.4, Eigen::Vector3d::UnitY()).toRotationMatrix();
	U.translation() = Eigen::Vector3d(0.1, 0.15, -0.1);
	const Eigen::Affine3d Uinv = U.inverse();
	double ts = 0.0;
	for (int i = 0; i < n; ++i, ts += 1.0 / 30.0) {
		const Eigen::Affine3d H = HeadRich(i);
		rec.rows.push_back(MakeRow(ts, H, Uinv * H, /*reloc=*/i < relocCount && i > 0 && (i % 50) == 0));
	}
	return rec;
}

} // namespace

TEST(WitnessDriftReplayTest, EndToEndCalibratesAndRuns)
{
	const auto rec = SyntheticRecording(600, 0);
	const auto s = ComputeWitnessDrift(rec, WitnessDriftOptions{});

	EXPECT_TRUE(s.calibrated) << "note=" << s.note;
	EXPECT_STREQ(s.note, "ok");
	EXPECT_GT(s.driftSamples, 400);
}

TEST(WitnessDriftReplayTest, CountsRelocRows)
{
	const auto rec = SyntheticRecording(600, 600); // reloc every 50th row
	const auto s = ComputeWitnessDrift(rec, WitnessDriftOptions{});

	EXPECT_TRUE(s.calibrated) << "note=" << s.note;
	EXPECT_GE(s.relocTotal, 5);
	EXPECT_EQ(s.relocMeasured, s.relocTotal) << "all synthetic reloc rows carry valid witness poses";
}

TEST(WitnessDriftReplayTest, NoWitnessColumnsIsInert)
{
	LoadedRecording rec;
	rec.formatVersion = 3;
	rec.hasLockedSnapColumns = false;
	const auto s = ComputeWitnessDrift(rec, WitnessDriftOptions{});

	EXPECT_FALSE(s.calibrated);
	EXPECT_STREQ(s.note, "no_witness_columns");
}
