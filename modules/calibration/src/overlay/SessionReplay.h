#pragma once

// Offline session-layer replay over a loaded spacecal recording.
//
// RunReplay (MotionRecording.h) re-runs the SOLVER closed-loop; this header
// re-runs the session around it: profile seeding, the bounded sample window,
// and the applied-transform trajectory, producing the wander/tilt metrics the
// session gate pins.

#include "GravityAlignment.h"
#include "MotionRecording.h"

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <string>

namespace spacecal::replay {

struct SessionReplayOptions
{
	// Solver shape (mirrors ReplayOptions).
	std::size_t maxContinuousSamples = 200;
	double threshold = 1.5;
	double maxRelError = 0.005;
	bool ignoreOutliers = true;
	bool lockRelativePosition = true;
	ReplaySeedMode seedMode = ReplaySeedMode::Recorded;
	Eigen::Vector3d seedTransCm = Eigen::Vector3d::Zero();
	Eigen::Vector3d seedRotDeg = Eigen::Vector3d::Zero();
};

struct SessionReplayResult
{
	bool succeeded = false;
	std::string error;
	int rowsProcessed = 0;
	int accepts = 0;
	// Applied-transform trajectory (net movement of the world).
	double totalAppliedPathCm = 0.0;
	double peakAppliedStepCm = 0.0;
	// The subset of the applied path that did NOT arrive through a classified
	// event (session-first candidate): solver wander the user experiences as
	// the world sliding.
	double unclassifiedPathCm = 0.0;
	double maxUnclassifiedStepCm = 0.0;
	double wanderPer10MinCm = 0.0;
	// Rotation churn of the applied transform (geodesic angle per step).
	// Unclassified rotation is what the user feels as the world leaning;
	// the translation wander metric is blind to it.
	double unclassifiedRotPathDeg = 0.0;
	double maxUnclassifiedRotStepDeg = 0.0;
	double rotWanderPer10MinDeg = 0.0;
	// Tilt (deviation of the applied rotation from pure yaw, GravityAlignment.h
	// TiltAngleDeg) -- the "trackers lean sideways" signal the rot-wander
	// metrics can't separate from yaw churn.
	double maxAppliedTiltDeg = 0.0;
	double finalAppliedTiltDeg = 0.0;
	Eigen::Vector3d netAppliedDriftCm = Eigen::Vector3d::Zero();
	bool seedApplied = false;
};

inline SessionReplayResult RunSessionReplay(const LoadedRecording& rec, const SessionReplayOptions& opts)
{
	SessionReplayResult res;
	if (!rec.error.empty()) {
		res.error = rec.error;
		return res;
	}
	if (rec.rows.empty()) {
		res.error = "Recording has no replayable rows.";
		return res;
	}
	if (!rec.hasLockedSnapColumns) {
		res.error = "session_replay_requires_v4";
		return res;
	}

	CalibrationCalc calc;
	calc.enableStaticRecalibration = false;
	calc.lockRelativePosition = opts.lockRelativePosition;

	// Applied transform: what the driver would render. Solver accepts write it
	// absolutely -- the same single variable the live tick shares.
	Eigen::AffineCompact3d applied = Eigen::AffineCompact3d::Identity();
	bool hasApplied = false;
	Eigen::AffineCompact3d seedTransform = Eigen::AffineCompact3d::Identity();
	{
		bool seed = false;
		if (opts.seedMode == ReplaySeedMode::Recorded && rec.seedProfile.valid) {
			seedTransform = ReplayProfileTransform(rec.seedProfile.rotDeg, rec.seedProfile.transCm);
			seed = true;
		}
		else if (opts.seedMode == ReplaySeedMode::Explicit) {
			seedTransform = ReplayProfileTransform(opts.seedRotDeg, opts.seedTransCm);
			seed = true;
		}
		if (seed) {
			calc.SeedEstimatedTransformation(seedTransform, /*annotate=*/false);
			applied = seedTransform;
			hasApplied = true;
			res.seedApplied = true;
		}
	}
	Eigen::Vector3d firstAppliedCm = applied.translation() * 100.0;
	Eigen::Vector3d prevAppliedCm = firstAppliedCm;
	Eigen::Matrix3d prevAppliedRot = applied.linear();

	const std::size_t window = opts.maxContinuousSamples > 0 ? opts.maxContinuousSamples : 200;
	const std::size_t drop = std::max<std::size_t>(1, window / 10);

	// The first accepted candidate after session start lands through the
	// motion gate live -- its step is classified, not wander.
	int classifiedAcceptBudget = 1;
	auto stepApplied = [&](const Eigen::AffineCompact3d& newApplied, bool classified) {
		const Eigen::Vector3d newCm = newApplied.translation() * 100.0;
		if (hasApplied) {
			const double stepCm = (newCm - prevAppliedCm).norm();
			res.totalAppliedPathCm += stepCm;
			res.peakAppliedStepCm = std::max(res.peakAppliedStepCm, stepCm);
			const double rotStepDeg =
			    Eigen::AngleAxisd(prevAppliedRot.transpose() * newApplied.linear()).angle() * (180.0 / EIGEN_PI);
			if (!classified) {
				res.unclassifiedPathCm += stepCm;
				res.maxUnclassifiedStepCm = std::max(res.maxUnclassifiedStepCm, stepCm);
				res.unclassifiedRotPathDeg += rotStepDeg;
				res.maxUnclassifiedRotStepDeg = std::max(res.maxUnclassifiedRotStepDeg, rotStepDeg);
			}
		}
		const double tiltDeg = spacecal::gravity::TiltAngleDeg(Eigen::Quaterniond(newApplied.rotation()));
		res.maxAppliedTiltDeg = std::max(res.maxAppliedTiltDeg, tiltDeg);
		res.finalAppliedTiltDeg = tiltDeg;
		prevAppliedCm = newCm;
		prevAppliedRot = newApplied.linear();
		if (!hasApplied) {
			firstAppliedCm = newCm;
			hasApplied = true;
		}
	};

	for (const auto& row : rec.rows) {
		++res.rowsProcessed;

		const bool sampleAccepted =
		    !row.hasSampleDiagnostics || (row.sampleObserved && row.sampleAccepted && row.sample.valid);
		if (!sampleAccepted) continue;

		Sample s = row.hasSampleDiagnostics ? row.sample : Sample(row.ref, row.target, row.timestamp);
		calc.PushSample(s);
		while (calc.SampleCount() > window)
			calc.ShiftSample();
		if (calc.SampleCount() >= window) {
			bool lerp = false;
			if (calc.ComputeIncremental(lerp, opts.threshold, opts.maxRelError, opts.ignoreOutliers)) {
				++res.accepts;
				applied = calc.Transformation();
				const bool classifiedStep = classifiedAcceptBudget > 0;
				if (classifiedAcceptBudget > 0) --classifiedAcceptBudget;
				stepApplied(applied, classifiedStep);
			}
			for (std::size_t i = 0; i < drop; ++i)
				calc.ShiftSample();
		}
	}

	if (hasApplied) res.netAppliedDriftCm = prevAppliedCm - firstAppliedCm;
	const double sessionSec = rec.rows.back().timestamp - rec.rows.front().timestamp;
	if (sessionSec > 1.0) {
		res.wanderPer10MinCm = res.unclassifiedPathCm * 600.0 / sessionSec;
		res.rotWanderPer10MinDeg = res.unclassifiedRotPathDeg * 600.0 / sessionSec;
	}
	res.succeeded = true;
	return res;
}

} // namespace spacecal::replay
