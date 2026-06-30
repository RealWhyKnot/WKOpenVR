#pragma once

#include "PassiveRoleInference.h"
#include "RoleCatalog.h"

#include <vector>

namespace phantom {

// One-shot "snap" role assignment.
//
// Where the passive accumulator needs seconds of motion to grow confident, a
// snap maps every tracker from a single static read of where the trackers sit
// relative to the head: feet near the floor, waist mid-torso, chest high, and
// left/right off the head's horizontal right axis. Vertical-bob is meaningless
// for a still pose, so the snap ignores that channel (weight 0) and leans purely
// on the geometry, then forces the sample term to full so a clean stand clears
// the apply bar instantly. The math is the same RoleCost/InferRoles the passive
// path uses; only the feature source differs.

enum class SnapStatus : uint8_t
{
	Ok = 0,            // at least one tracker assigned a role
	HeadTilted = 1,    // horizontal head axis unusable (look forward and retry)
	NoTrackers = 2,    // no trackers present to assign
	HmdNotReady = 3,   // implausible head height above floor
	LowConfidence = 4, // trackers present but none crossed the confidence bar
	NotCalibrated = 5  // trackers sit nowhere near the body (not in the HMD's space)
};

struct SnapTrackerInput
{
	uint32_t id = 0; // caller's slot index / device id, echoed back in the result
	double pos[3] = {0.0, 0.0, 0.0};
};

struct SnapAssignment
{
	uint32_t id = 0;
	BodyRole role = BodyRole::None;
	float confidence = 0.0f;
};

struct SnapResult
{
	SnapStatus status = SnapStatus::NoTrackers;
	bool ok = false;
	std::vector<SnapAssignment> assignments; // one per input tracker, input order
	uint32_t assigned_count = 0;
	double measured_height_m = 0.0; // HMD height above the floor
	double floor_y_m = 0.0;
};

// axes_valid + right_xz/fwd_xz are the caller's horizontal projection of the HMD
// orientation (e.g. HmdHorizontalAxes); false means the head is tilted too far to
// trust left/right. floor_y is the learned/standing floor height (0 if unknown).
inline SnapResult SnapCalibrate(const double hmd_pos[3], bool axes_valid, const double right_xz[2],
                                const double fwd_xz[2], double floor_y, const std::vector<SnapTrackerInput>& trackers,
                                InferenceParams params = {})
{
	SnapResult out;
	out.floor_y_m = floor_y;
	out.measured_height_m = hmd_pos[1] - floor_y;
	out.assignments.resize(trackers.size());
	for (size_t i = 0; i < trackers.size(); ++i) {
		out.assignments[i].id = trackers[i].id;
	}

	if (trackers.empty()) {
		out.status = SnapStatus::NoTrackers;
		return out;
	}
	if (!axes_valid) {
		out.status = SnapStatus::HeadTilted;
		return out;
	}
	const double head_h = hmd_pos[1] - floor_y;
	if (head_h < 0.5) {
		out.status = SnapStatus::HmdNotReady;
		return out;
	}

	// A static read carries no vertical-motion signal; score on geometry alone.
	params.weight_vert_motion = 0.0;

	std::vector<TrackerMotionFeatures> feats;
	feats.reserve(trackers.size());
	for (const auto& t : trackers) {
		const double dx = t.pos[0] - hmd_pos[0];
		const double dz = t.pos[2] - hmd_pos[2];
		TrackerMotionFeatures f;
		f.height_ratio = (t.pos[1] - floor_y) / head_h;
		f.lateral_norm = ((dx * right_xz[0]) + (dz * right_xz[1])) / head_h;
		f.forward_norm = ((dx * fwd_xz[0]) + (dz * fwd_xz[1])) / head_h;
		f.vert_motion_norm = 0.0;
		// Force the sample term to full so one clean read is trusted outright.
		f.sample_count = params.full_confidence_samples == 0 ? 1u : params.full_confidence_samples;
		f.has_data = true;
		feats.push_back(f);
	}

	// Sanity gate: a worn body tracker sits within roughly arm's reach of the
	// head, between the floor and just above it. If every tracker is far off
	// (deep negative height or metres away horizontally) the devices are not in
	// the headset's tracking space -- almost always an un-run Space Calibrator on
	// a mixed setup (e.g. Quest HMD + lighthouse trackers). Say so instead of
	// silently failing to assign.
	// A worn tracker sits within ~0.6 head-heights of the head horizontally
	// (waist/chest directly below, feet/knees just ahead, elbows out a little) and
	// between the floor and head height. Cross-space devices land metres away
	// (hdist ~0.85+) and/or well below the floor.
	int plausible = 0;
	for (const auto& f : feats) {
		const double hdist = std::sqrt((f.lateral_norm * f.lateral_norm) + (f.forward_norm * f.forward_norm));
		if (f.height_ratio >= -0.15 && f.height_ratio <= 1.3 && hdist <= 0.6) {
			++plausible;
		}
	}
	if (plausible == 0) {
		out.status = SnapStatus::NotCalibrated;
		return out;
	}

	static const std::vector<BodyRole> kCandidates = {BodyRole::Waist,     BodyRole::Chest,     BodyRole::LeftFoot,
	                                                  BodyRole::RightFoot, BodyRole::LeftKnee,  BodyRole::RightKnee,
	                                                  BodyRole::LeftElbow, BodyRole::RightElbow};

	const auto result = InferRoles(feats, kCandidates, params);
	for (size_t i = 0; i < result.size() && i < out.assignments.size(); ++i) {
		out.assignments[i].role = result[i].role;
		out.assignments[i].confidence = result[i].confidence;
		if (result[i].role != BodyRole::None) {
			++out.assigned_count;
		}
	}

	out.ok = out.assigned_count > 0;
	out.status = out.ok ? SnapStatus::Ok : SnapStatus::LowConfidence;
	return out;
}

// Stable UI/log text for a snap outcome.
inline const char* SnapStatusMessage(SnapStatus s)
{
	switch (s) {
		case SnapStatus::Ok:
			return "Trackers mapped.";
		case SnapStatus::HeadTilted:
			return "Look forward and stand level, then snap again.";
		case SnapStatus::NoTrackers:
			return "No trackers detected to map.";
		case SnapStatus::HmdNotReady:
			return "Headset height looks off; stand up and snap again.";
		case SnapStatus::LowConfidence:
			return "Couldn't place trackers confidently; spread out and retry.";
		case SnapStatus::NotCalibrated:
			return "Trackers aren't aligned to your headset; run Space Calibrator first.";
	}
	return "";
}

} // namespace phantom
