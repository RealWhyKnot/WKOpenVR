#pragma once

// Offline witness-drift oracle.
//
// RunReplay (MotionRecording.cpp) replays the paired-motion solver and is
// MAD-invariant to the recovery/output layer, so it cannot measure the
// continuous sub-30 cm witness correction. This pass fills that gap on a
// recording's raw HMD + witness (head-mount tracker) poses, so the correction
// toggles can be A/B-measured on real recorded sessions instead of only live.
// RunReplay is left byte-identical so the solver baseline stays a clean
// regression guard.
//
// Geometry (verified against the live path, Calibration.cpp:394-398): the
// calibration reference device is the HMD and the target device is the
// head-mounted witness (ref==hmd, tgt==head_tracker in the recordings). The
// live drift signal is:
//     targetToReference = C                       // the calibration transform
//     trackerReference  = C * trackerTarget       // witness in the HMD frame
//     localOffset       = hmd^-1 * trackerReference
//     savedDeltaM       = |localOffset.trans - headFromTracker^-1.trans|
// localOffset is a rigid HMD-relative offset that is CONSTANT when calibration
// is perfect; it wanders as the SLAM frame drifts. This oracle reconstructs C
// with CalibrationCalc (the same solver RunReplay uses), takes the baseline
// offset over an early window (models T1 offset_auto_calibrate), then measures
// how far localOffset drifts and how much a slew-limited correction closes it.
//
// Models: T1 (auto-commit the offset once the early fit is stable), T2
// (continuous_correction_live -- a slew-limited follower of the drift, capped at
// kMaxCorrectionM so >30 cm jumps hand off to recovery), T3 (reloc rows split by
// drift magnitude: witness-disagreement vs real motion).

#include "CalibrationCalc.h"
#include "ContinuousCorrection.h"
#include "MotionRecording.h"

#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

namespace spacecal::replay {

struct WitnessDriftOptions
{
	bool applyContinuousCorrection = true; // T2 on/off
	// Correction shape. Defaults mirror the shipped ContinuousCorrection
	// constants; the oracle sweeps them to find the drift-minimising setting on
	// real recordings before any constant changes in the live path.
	double correctionSlewMps = spacecal::cont_correction::kCorrectionSlewMps;
	double correctionDeadbandM = spacecal::cont_correction::kDeadbandFloorM;
	double correctionMaxM = spacecal::cont_correction::kMaxCorrectionM;
	// Rows used to fit the calibration + baseline offset ("calibrate then watch
	// drift"). 0 = use the whole recording.
	std::size_t baselineRows = 3000;
	// Drift (mm) above which a relocalization row counts as the witness
	// disagreeing with SLAM (correctable) rather than real head motion.
	double relocDriftFlipMm = 30.0;
};

struct WitnessDriftSummary
{
	bool calibrated = false; // calibration + baseline offset established
	double baselineOffsetMm = 0.0;
	int baselineSamples = 0;
	int driftSamples = 0;

	double uncorrectedRmsMm = 0.0;
	double uncorrectedP50Mm = 0.0;
	double uncorrectedP95Mm = 0.0;
	double uncorrectedPeakMm = 0.0;
	double correctedRmsMm = 0.0;
	double correctedP50Mm = 0.0;
	double correctedP95Mm = 0.0;
	double correctedPeakMm = 0.0;
	double reductionPct = 0.0; // RMS reduction over the whole session

	// The continuous-correction regime: samples whose uncorrected drift is below
	// the 30 cm cap (above it is a relocalization the recovery path owns). This
	// isolates T2's real effect from the tail of big excursions that swamp RMS.
	int subCapSamples = 0;
	double subCapUncorrectedRmsMm = 0.0;
	double subCapCorrectedRmsMm = 0.0;
	double subCapReductionPct = 0.0;

	int relocTotal = 0;
	int relocMeasured = 0;
	int relocFlipLike = 0;
	double relocMeanDriftMm = 0.0;

	const char* note = "ok";
};

// Parameterised form of cont_correction::CorrectionStepM so the oracle can sweep
// slew/dead-band/cap. With the default option values this reduces exactly to
// CorrectionStepM(errorM, 0, dt).
inline double CorrectionStepParam(double errorM, double deadbandM, double slewMps, double maxM, double dt)
{
	if (errorM <= deadbandM) return 0.0;
	if (errorM > maxM) return 0.0;
	const double budget = slewMps * (dt > 0.0 ? dt : 0.0);
	const double remaining = errorM - deadbandM;
	return remaining < budget ? remaining : budget;
}

// Result of the pure slew-limited correction model over a drift sequence.
struct CorrectionModelResult
{
	double uncorrectedRmsMm = 0.0, uncorrectedP50Mm = 0.0, uncorrectedP95Mm = 0.0, uncorrectedPeakMm = 0.0;
	double correctedRmsMm = 0.0, correctedP50Mm = 0.0, correctedP95Mm = 0.0, correctedPeakMm = 0.0;
	double reductionPct = 0.0;
	int subCapSamples = 0;
	double subCapUncorrectedRmsMm = 0.0, subCapCorrectedRmsMm = 0.0, subCapReductionPct = 0.0;
};

namespace detail {

inline double Percentile(std::vector<double> v, double p)
{
	if (v.empty()) return 0.0;
	std::sort(v.begin(), v.end());
	const double pos = p * static_cast<double>(v.size() - 1);
	const std::size_t lo = static_cast<std::size_t>(std::floor(pos));
	const std::size_t hi = std::min<std::size_t>(lo + 1, v.size() - 1);
	return v[lo] + (v[hi] - v[lo]) * (pos - static_cast<double>(lo));
}

inline double Rms(const std::vector<double>& v)
{
	if (v.empty()) return 0.0;
	double s = 0.0;
	for (double x : v)
		s += x * x;
	return std::sqrt(s / static_cast<double>(v.size()));
}

inline Eigen::Affine3d ToAffine3d(const Pose& p)
{
	Eigen::Affine3d a = Eigen::Affine3d::Identity();
	a.linear() = p.rot;
	a.translation() = p.trans;
	return a;
}

// localOffset = hmd^-1 * (C * tracker), translation only (the drift lives in
// translation; the correction is translational). C maps target(witness)->
// reference(HMD).
inline Eigen::Vector3d LocalOffsetTrans(const Eigen::Affine3d& C, const Pose& hmd, const Pose& tracker)
{
	const Eigen::Affine3d hmdA = ToAffine3d(hmd);
	const Eigen::Affine3d trkA = ToAffine3d(tracker);
	return (hmdA.inverse() * (C * trkA)).translation();
}

} // namespace detail

// Pure slew-limited correction model over a drift-vector sequence. Kept free of
// CalibrationCalc so the correction math is unit-testable with injected drift
// (the calibration solve is validated separately on real recordings). Each drift
// vector is the witness-vs-calibration offset from its calibrated baseline; the
// follower tracks it under the slew/dead-band/cap limits, and the corrected
// residual is drift - follower.
inline CorrectionModelResult RunCorrectionModel(const std::vector<Eigen::Vector3d>& driftVecs,
                                                const std::vector<double>& dts, const WitnessDriftOptions& opts)
{
	CorrectionModelResult r;
	std::vector<double> unc;
	std::vector<double> cor;
	unc.reserve(driftVecs.size());
	cor.reserve(driftVecs.size());
	Eigen::Vector3d follower = Eigen::Vector3d::Zero();
	for (std::size_t i = 0; i < driftVecs.size(); ++i) {
		const Eigen::Vector3d& driftVec = driftVecs[i];
		const double dt = i < dts.size() ? dts[i] : (1.0 / 90.0);
		unc.push_back(driftVec.norm() * 1000.0);
		if (opts.applyContinuousCorrection) {
			const Eigen::Vector3d toTarget = driftVec - follower;
			const double mag = toTarget.norm();
			const double step =
			    CorrectionStepParam(mag, opts.correctionDeadbandM, opts.correctionSlewMps, opts.correctionMaxM, dt);
			if (step > 0.0 && mag > 1e-9) follower += step * (toTarget / mag);
			cor.push_back((driftVec - follower).norm() * 1000.0);
		}
		else {
			cor.push_back(driftVec.norm() * 1000.0);
		}
	}

	r.uncorrectedRmsMm = detail::Rms(unc);
	r.uncorrectedP50Mm = detail::Percentile(unc, 0.50);
	r.uncorrectedP95Mm = detail::Percentile(unc, 0.95);
	r.uncorrectedPeakMm = unc.empty() ? 0.0 : *std::max_element(unc.begin(), unc.end());
	r.correctedRmsMm = detail::Rms(cor);
	r.correctedP50Mm = detail::Percentile(cor, 0.50);
	r.correctedP95Mm = detail::Percentile(cor, 0.95);
	r.correctedPeakMm = cor.empty() ? 0.0 : *std::max_element(cor.begin(), cor.end());
	r.reductionPct =
	    r.uncorrectedRmsMm > 1e-9 ? (100.0 * (r.uncorrectedRmsMm - r.correctedRmsMm) / r.uncorrectedRmsMm) : 0.0;

	const double capMm = opts.correctionMaxM * 1000.0;
	std::vector<double> su;
	std::vector<double> sc;
	su.reserve(unc.size());
	sc.reserve(unc.size());
	for (std::size_t i = 0; i < unc.size(); ++i) {
		if (unc[i] < capMm) {
			su.push_back(unc[i]);
			sc.push_back(cor[i]);
		}
	}
	r.subCapSamples = static_cast<int>(su.size());
	r.subCapUncorrectedRmsMm = detail::Rms(su);
	r.subCapCorrectedRmsMm = detail::Rms(sc);
	r.subCapReductionPct =
	    r.subCapUncorrectedRmsMm > 1e-9
	        ? (100.0 * (r.subCapUncorrectedRmsMm - r.subCapCorrectedRmsMm) / r.subCapUncorrectedRmsMm)
	        : 0.0;
	return r;
}

inline WitnessDriftSummary ComputeWitnessDrift(const LoadedRecording& rec, const WitnessDriftOptions& opts)
{
	WitnessDriftSummary out;
	if (!rec.hasLockedSnapColumns) {
		out.note = "no_witness_columns";
		return out;
	}

	// --- Fit the calibration transform C from the ref/tgt pairs (the same solve
	// RunReplay uses). Bound to an early window so C represents the calibration
	// at session start; drift then accumulates against it. ---------------------
	CalibrationCalc calc;
	calc.enableStaticRecalibration = false;
	calc.lockRelativePosition = false;
	const std::size_t baselineLimit = opts.baselineRows == 0 ? rec.rows.size() : opts.baselineRows;
	std::size_t pushed = 0;
	for (std::size_t i = 0; i < rec.rows.size() && pushed < baselineLimit; ++i) {
		const auto& r = rec.rows[i];
		if (!(r.hasHmdPose && r.headTrackerValid)) continue;
		Sample s = r.hasSampleDiagnostics ? r.sample : Sample(r.ref, r.target, r.timestamp);
		if (!s.valid) continue;
		calc.PushSample(s);
		++pushed;
	}
	if (pushed < 10 || !calc.ComputeOneshot(true) || !calc.isValid()) {
		out.note = "calibration_solve_failed";
		return out;
	}
	Eigen::Affine3d C = Eigen::Affine3d::Identity();
	{
		const Eigen::AffineCompact3d cal = calc.Transformation();
		C.linear() = cal.linear();
		C.translation() = cal.translation();
	}

	// Determine C's orientation (target->reference vs its inverse) by whichever
	// makes localOffset most stable over the baseline window, then take the
	// baseline offset as the calibrated constant (models T1 auto-calibrate).
	auto baselineStats = [&](const Eigen::Affine3d& cand, Eigen::Vector3d& meanOut) -> double {
		Eigen::Vector3d sum = Eigen::Vector3d::Zero();
		int n = 0;
		for (std::size_t i = 0; i < rec.rows.size() && static_cast<std::size_t>(n) < baselineLimit; ++i) {
			const auto& r = rec.rows[i];
			if (!(r.hasHmdPose && r.headTrackerValid)) continue;
			sum += detail::LocalOffsetTrans(cand, r.hmd, r.headTracker);
			++n;
		}
		if (n == 0) {
			meanOut = Eigen::Vector3d::Zero();
			return std::numeric_limits<double>::infinity();
		}
		meanOut = sum / static_cast<double>(n);
		double var = 0.0;
		int m = 0;
		for (std::size_t i = 0; i < rec.rows.size() && static_cast<std::size_t>(m) < baselineLimit; ++i) {
			const auto& r = rec.rows[i];
			if (!(r.hasHmdPose && r.headTrackerValid)) continue;
			var += (detail::LocalOffsetTrans(cand, r.hmd, r.headTracker) - meanOut).squaredNorm();
			++m;
		}
		return var / static_cast<double>(m);
	};
	Eigen::Vector3d meanFwd, meanInv;
	const double varFwd = baselineStats(C, meanFwd);
	const Eigen::Affine3d Cinv = C.inverse();
	const double varInv = baselineStats(Cinv, meanInv);
	const Eigen::Affine3d Cuse = varInv < varFwd ? Cinv : C;
	const Eigen::Vector3d refOffset = varInv < varFwd ? meanInv : meanFwd;
	out.baselineSamples = static_cast<int>(pushed);
	out.baselineOffsetMm = std::sqrt(std::min(varFwd, varInv)) * 1000.0;

	// --- Build the drift-vector sequence, then run the pure correction model. --
	std::vector<Eigen::Vector3d> driftVecs;
	std::vector<double> dts;
	driftVecs.reserve(rec.rows.size());
	dts.reserve(rec.rows.size());
	double prevTs = 0.0;
	bool havePrev = false;
	for (std::size_t i = 0; i < rec.rows.size(); ++i) {
		const auto& r = rec.rows[i];
		if (!(r.hasHmdPose && r.headTrackerValid)) {
			havePrev = false;
			continue;
		}
		driftVecs.push_back(detail::LocalOffsetTrans(Cuse, r.hmd, r.headTracker) - refOffset);
		dts.push_back((havePrev && r.timestamp > prevTs) ? (r.timestamp - prevTs) : (1.0 / 90.0));
		prevTs = r.timestamp;
		havePrev = true;
	}

	const CorrectionModelResult m = RunCorrectionModel(driftVecs, dts, opts);
	out.calibrated = true;
	out.driftSamples = static_cast<int>(driftVecs.size());
	out.uncorrectedRmsMm = m.uncorrectedRmsMm;
	out.uncorrectedP50Mm = m.uncorrectedP50Mm;
	out.uncorrectedP95Mm = m.uncorrectedP95Mm;
	out.uncorrectedPeakMm = m.uncorrectedPeakMm;
	out.correctedRmsMm = m.correctedRmsMm;
	out.correctedP50Mm = m.correctedP50Mm;
	out.correctedP95Mm = m.correctedP95Mm;
	out.correctedPeakMm = m.correctedPeakMm;
	out.reductionPct = m.reductionPct;
	out.subCapSamples = m.subCapSamples;
	out.subCapUncorrectedRmsMm = m.subCapUncorrectedRmsMm;
	out.subCapCorrectedRmsMm = m.subCapCorrectedRmsMm;
	out.subCapReductionPct = m.subCapReductionPct;

	// --- T3: relocalization rows -- drift magnitude splits flip/drift from motion.
	double relocSum = 0.0;
	for (std::size_t i = 0; i < rec.rows.size(); ++i) {
		const auto& r = rec.rows[i];
		if (!r.relocDetected) continue;
		++out.relocTotal;
		if (!(r.hasHmdPose && r.headTrackerValid)) continue;
		const double driftMm = (detail::LocalOffsetTrans(Cuse, r.hmd, r.headTracker) - refOffset).norm() * 1000.0;
		++out.relocMeasured;
		relocSum += driftMm;
		if (driftMm > opts.relocDriftFlipMm) ++out.relocFlipLike;
	}
	out.relocMeanDriftMm = out.relocMeasured > 0 ? (relocSum / out.relocMeasured) : 0.0;

	return out;
}

} // namespace spacecal::replay
