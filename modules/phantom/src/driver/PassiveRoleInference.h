#pragma once

#include "RoleCatalog.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

namespace phantom {

// Passive body-role inference.
//
// Watches trackers move relative to the headset and figures out which physical
// tracker is sitting on which body point. The signal is mostly
// geometric: how high a tracker rides as a fraction of head height (feet near
// the floor, waist around mid-torso, chest high), which side of the head it
// stays on (left vs right), and how much it bobs vertically while walking
// (feet swing, the waist barely moves). Given the set of roles a tracker layout
// can have, it scores every tracker against every role's kinematic prior and
// solves a one-to-one assignment, reporting a confidence per assignment so the
// caller can require corroboration before trusting it.
//
// Everything here is plain math on doubles so the driver and the unit tests
// share the exact same code path; no OpenVR types leak in.

// Running per-tracker feature accumulator. The driver feeds it the HMD pose and
// the tracker pose each frame (already flattened to horizontal head axes so the
// left/right sign is unambiguous); Compute() folds the running sums into a
// normalized feature vector.
struct TrackerMotionFeatures
{
	double height_ratio = 0.0;     // tracker height / head height (floor at y=0)
	double lateral_norm = 0.0;     // signed offset along head-right, / head height (+ = right)
	double forward_norm = 0.0;     // signed offset along head-forward, / head height (+ = ahead)
	double vert_motion_norm = 0.0; // vertical bob std-dev / head height
	uint32_t sample_count = 0;
	bool has_data = false;
};

class RoleInferenceAccumulator
{
public:
	// hmd_pos / tracker_pos are world positions {x,y,z}; hmd_right / hmd_fwd are
	// the HMD's horizontal right and forward unit vectors {x,z} so the caller
	// owns the (sign-sensitive) heading math once. Only valid, tracked samples
	// should be passed in. The floor_y overload normalizes heights against the
	// floor the body-prior estimator has learned (floor at y=0 is the default),
	// so the priors hold regardless of the runtime's standing-zero.
	void AddSample(const double hmd_pos[3], const double hmd_right[2], const double hmd_fwd[2],
	               const double tracker_pos[3])
	{
		AddSample(hmd_pos, hmd_right, hmd_fwd, tracker_pos, 0.0);
	}

	void AddSample(const double hmd_pos[3], const double hmd_right[2], const double hmd_fwd[2],
	               const double tracker_pos[3], double floor_y)
	{
		const double head_h = hmd_pos[1] - floor_y;
		if (head_h < kMinHeadHeight) {
			return; // implausible head height (uninitialised / seated-on-floor); skip.
		}

		const double dx = tracker_pos[0] - hmd_pos[0];
		const double dz = tracker_pos[2] - hmd_pos[2];
		const double lateral = (dx * hmd_right[0]) + (dz * hmd_right[1]);
		const double forward = (dx * hmd_fwd[0]) + (dz * hmd_fwd[1]);
		const double tracker_h = tracker_pos[1] - floor_y;

		sum_height_ratio_ += tracker_h / head_h;
		sum_lateral_ += lateral;
		sum_forward_ += forward;
		sum_head_h_ += head_h;
		sum_tracker_h_ += tracker_h;
		sum_tracker_h_sq_ += tracker_h * tracker_h;
		++count_;
	}

	uint32_t SampleCount() const { return count_; }

	// Decay the running sums toward zero so older samples fade out and the
	// features track the user's current tracker layout. Applied periodically by the
	// driver (e.g. once per inference pass). factor in (0,1]; 1 = no decay.
	void Decay(double factor)
	{
		if (factor >= 1.0 || count_ == 0) {
			return;
		}

		factor = std::clamp(factor, 0.0, 1.0);
		sum_height_ratio_ *= factor;
		sum_lateral_ *= factor;
		sum_forward_ *= factor;
		sum_head_h_ *= factor;
		sum_tracker_h_ *= factor;
		sum_tracker_h_sq_ *= factor;
		count_ = static_cast<uint32_t>(std::llround(count_ * factor));
	}

	void Reset() { *this = RoleInferenceAccumulator{}; }

	TrackerMotionFeatures Compute() const
	{
		TrackerMotionFeatures f;
		if (count_ == 0) {
			return f;
		}

		const double n = static_cast<double>(count_);
		const double head_h = sum_head_h_ / n;
		const double inv_head = head_h > kMinHeadHeight ? 1.0 / head_h : 0.0;
		const double mean_tracker_h = sum_tracker_h_ / n;
		const double var_y = std::max(0.0, (sum_tracker_h_sq_ / n) - (mean_tracker_h * mean_tracker_h));

		f.height_ratio = sum_height_ratio_ / n;
		f.lateral_norm = (sum_lateral_ / n) * inv_head;
		f.forward_norm = (sum_forward_ / n) * inv_head;
		f.vert_motion_norm = std::sqrt(var_y) * inv_head;
		f.sample_count = count_;
		f.has_data = true;
		return f;
	}

private:
	static constexpr double kMinHeadHeight = 0.5; // metres

	double sum_height_ratio_ = 0.0;
	double sum_lateral_ = 0.0;
	double sum_forward_ = 0.0;
	double sum_head_h_ = 0.0;
	double sum_tracker_h_ = 0.0;
	double sum_tracker_h_sq_ = 0.0;
	uint32_t count_ = 0;
};

// Kinematic prior for one role, expressed in the same normalized units the
// accumulator produces. Values are fractions of head height with the floor at
// y=0 and +lateral pointing to the user's right.
struct RolePrior
{
	BodyRole role;
	double height_ratio;
	double lateral_norm;
	double vert_motion_norm;
};

// The roles a real-world extra tracker can sit on (matches the set with a
// SteamVR controller type in RoleCatalog). Shoulders stay solver-internal.
inline const std::array<RolePrior, 8>& DefaultRolePriors()
{
	static const std::array<RolePrior, 8> kPriors = {{
	    {BodyRole::Waist, 0.53, 0.00, 0.10},
	    {BodyRole::Chest, 0.74, 0.00, 0.10},
	    {BodyRole::LeftFoot, 0.06, -0.12, 0.45},
	    {BodyRole::RightFoot, 0.06, 0.12, 0.45},
	    {BodyRole::LeftKnee, 0.28, -0.10, 0.30},
	    {BodyRole::RightKnee, 0.28, 0.10, 0.30},
	    {BodyRole::LeftElbow, 0.63, -0.18, 0.25},
	    {BodyRole::RightElbow, 0.63, 0.18, 0.25},
	}};
	return kPriors;
}

struct InferenceParams
{
	double weight_height = 3.0;
	double weight_lateral = 2.0;
	double weight_vert_motion = 0.5;

	// Cost -> confidence falloff and the ambiguity-margin scale. Smaller scales
	// make the system more demanding before it reports high confidence.
	double cost_scale = 0.10;
	double margin_scale = 0.06;

	// Samples needed before an assignment can reach full confidence.
	uint32_t full_confidence_samples = 600; // ~ a few seconds at tracker rate

	// Samples after which a static (no-motion) first guess is already trusted:
	// at this count the sample term reaches static_sample_floor so a clean
	// standing layout clears the apply bar in a fraction of a second instead of
	// waiting out the full motion ramp. A one-shot snap forces sample_count to
	// full_confidence_samples so it lands at 1.0 immediately.
	uint32_t min_static_samples = 24; // ~0.25 s at tracker rate
	double static_sample_floor = 0.85;

	// Minimum confidence below which an assignment is reported as None.
	float min_confidence = 0.35f;
};

struct RoleAssignment
{
	int tracker_index = -1; // index into the input features vector
	BodyRole role = BodyRole::None;
	float confidence = 0.0f;
	double cost = 0.0;
};

inline double RoleCost(const TrackerMotionFeatures& f, const RolePrior& p, const InferenceParams& params)
{
	const double dh = f.height_ratio - p.height_ratio;
	const double dl = f.lateral_norm - p.lateral_norm;
	const double dv = f.vert_motion_norm - p.vert_motion_norm;
	return (params.weight_height * dh * dh) + (params.weight_lateral * dl * dl) + (params.weight_vert_motion * dv * dv);
}

// Confidence in {0,1} for assigning a tracker to its best role, given the cost
// of that role, the cost of its next-best alternative (ambiguity margin) and
// how many samples backed the features.
inline float AssignmentConfidence(double best_cost, double second_cost, uint32_t sample_count,
                                  const InferenceParams& params)
{
	const double cost_factor = std::exp(-best_cost / std::max(1e-6, params.cost_scale));
	const double margin = std::max(0.0, second_cost - best_cost);
	const double margin_factor = 1.0 - std::exp(-margin / std::max(1e-6, params.margin_scale));

	// Sample term: ramp 0 -> static_sample_floor over the first min_static_samples
	// (so a clean static read is trusted quickly), then floor -> 1.0 over the rest
	// of the way to full_confidence_samples as sustained motion corroborates.
	double sample_factor;
	if (params.full_confidence_samples == 0) {
		sample_factor = 1.0;
	}
	else {
		const double n = static_cast<double>(sample_count);
		const double full = static_cast<double>(params.full_confidence_samples);
		const double min_static = static_cast<double>(params.min_static_samples == 0 ? 1u : params.min_static_samples);
		const double floor = std::clamp(params.static_sample_floor, 0.0, 1.0);
		const double base = std::min(1.0, n / min_static) * floor;
		const double extra = (n > min_static && full > min_static)
		                         ? (1.0 - floor) * std::min(1.0, (n - min_static) / (full - min_static))
		                         : 0.0;
		sample_factor = base + extra;
	}
	const double conf = cost_factor * margin_factor * sample_factor;
	return static_cast<float>(std::clamp(conf, 0.0, 1.0));
}

// Assign each input tracker to at most one candidate role. Greedy lowest-cost
// matching: repeatedly take the cheapest unused (tracker, role) pair until one
// side is exhausted. Trackers left over, or whose confidence falls below the
// threshold, come back with role None. The returned vector has one entry per
// input tracker, in input order.
inline std::vector<RoleAssignment> InferRoles(const std::vector<TrackerMotionFeatures>& trackers,
                                              const std::vector<BodyRole>& candidate_roles,
                                              const InferenceParams& params = {})
{
	const auto& priors = DefaultRolePriors();

	std::vector<RoleAssignment> result(trackers.size());
	for (size_t i = 0; i < trackers.size(); ++i) {
		result[i].tracker_index = static_cast<int>(i);
	}

	// Gather the priors that match the caller's candidate set.
	std::vector<RolePrior> active;
	active.reserve(candidate_roles.size());
	for (BodyRole role : candidate_roles) {
		for (const RolePrior& p : priors) {
			if (p.role == role) {
				active.push_back(p);
				break;
			}
		}
	}

	if (active.empty() || trackers.empty()) {
		return result;
	}

	struct Pair
	{
		int tracker;
		int role; // index into active
		double cost;
	};

	std::vector<Pair> pairs;
	pairs.reserve(trackers.size() * active.size());
	for (size_t t = 0; t < trackers.size(); ++t) {
		if (!trackers[t].has_data) {
			continue;
		}
		for (size_t r = 0; r < active.size(); ++r) {
			pairs.push_back({static_cast<int>(t), static_cast<int>(r), RoleCost(trackers[t], active[r], params)});
		}
	}

	std::sort(pairs.begin(), pairs.end(), [](const Pair& a, const Pair& b) { return a.cost < b.cost; });

	std::vector<bool> tracker_used(trackers.size(), false);
	std::vector<bool> role_used(active.size(), false);
	for (const Pair& pr : pairs) {
		if (tracker_used[pr.tracker] || role_used[pr.role]) {
			continue;
		}

		// Second-best cost for this tracker among all other active roles, for
		// the ambiguity margin. Include roles already claimed by earlier
		// trackers so a forced leftover assignment does not become falsely
		// high-confidence just because no alternatives remain available.
		double second_cost = std::numeric_limits<double>::infinity();
		for (size_t r = 0; r < active.size(); ++r) {
			if (static_cast<int>(r) == pr.role) {
				continue;
			}
			second_cost = std::min(second_cost, RoleCost(trackers[pr.tracker], active[r], params));
		}
		if (!std::isfinite(second_cost)) {
			second_cost = pr.cost + params.margin_scale * 3.0; // last role left: treat as clearly chosen
		}

		const float confidence = AssignmentConfidence(pr.cost, second_cost, trackers[pr.tracker].sample_count, params);

		tracker_used[pr.tracker] = true;
		role_used[pr.role] = true;

		if (confidence >= params.min_confidence) {
			result[pr.tracker].role = active[pr.role].role;
			result[pr.tracker].confidence = confidence;
			result[pr.tracker].cost = pr.cost;
		}
	}

	return result;
}

// Confidence that one tracker's features belong to a specific role, scored the
// same way InferRoles scores its picks (cost of the queried role vs the best
// alternative in the candidate set). Lets a caller ask "does the role this
// tracker currently holds still fit?" without re-running the full assignment.
inline float RoleFitConfidence(const TrackerMotionFeatures& f, BodyRole role,
                               const std::vector<BodyRole>& candidate_roles, const InferenceParams& params = {})
{
	if (!f.has_data || role == BodyRole::None) {
		return 0.0f;
	}
	const auto& priors = DefaultRolePriors();
	double role_cost = std::numeric_limits<double>::infinity();
	double second = std::numeric_limits<double>::infinity();
	bool found = false;
	for (BodyRole c : candidate_roles) {
		const RolePrior* p = nullptr;
		for (const RolePrior& pr : priors) {
			if (pr.role == c) {
				p = &pr;
				break;
			}
		}
		if (!p) {
			continue;
		}
		const double cost = RoleCost(f, *p, params);
		if (c == role) {
			role_cost = cost;
			found = true;
		}
		else {
			second = std::min(second, cost);
		}
	}
	if (!found) {
		return 0.0f;
	}
	if (!std::isfinite(second)) {
		second = role_cost + params.margin_scale * 3.0;
	}
	return AssignmentConfidence(role_cost, second, f.sample_count, params);
}

} // namespace phantom
