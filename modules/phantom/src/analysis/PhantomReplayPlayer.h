#pragma once

#include "PassiveRoleInference.h"
#include "RoleArbiter.h"
#include "RoleCatalog.h"
#include "SnapCalibrate.h"

#include <cmath>
#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

namespace phantom {

// Offline replay of a phantom_replay_v1 capture (the CSV PhantomReplayRecorder
// writes when WKOPENVR_PHANTOM_REPLAY_RECORD=1 or debug logging is on). The CSV
// carries every frame of the HMD + controllers + trackers with pose/velocity
// plus a ground-truth body_role column, which lets us score auto-detection on a
// real session without VR in the loop: replay the samples through the same
// PassiveRoleInference + SnapCalibrate the driver runs and compare the predicted
// roles to the recorded truth. Pure (no OpenVR / Win32) so it builds in tests and
// the sidecar CLI alike.

struct ReplaySample
{
	double time_ms = 0.0;
	uint32_t device_id = 0;
	std::string serial;
	std::string device_class; // "hmd" / "controller" / "tracker"
	BodyRole ground_truth_role = BodyRole::None;
	bool pose_valid = false;
	double pos[3] = {0.0, 0.0, 0.0};
	double quat[4] = {1.0, 0.0, 0.0, 0.0}; // wxyz
};

struct ParsedReplay
{
	std::vector<ReplaySample> samples;
	bool ok = false;
	std::string error;
};

// Horizontal right/forward unit vectors {x,z} from an HMD quaternion (wxyz).
// Mirrors the driver's HmdHorizontalAxes; false when the head is too vertical.
inline bool HorizontalAxesFromQuat(const double q[4], double right_xz[2], double fwd_xz[2])
{
	const double w = q[0], x = q[1], y = q[2], z = q[3];
	double rx = 1.0 - 2.0 * (y * y + z * z);
	double rz = 2.0 * (x * z - w * y);
	double fx = -2.0 * (x * z + w * y);
	double fz = -(1.0 - 2.0 * (x * x + y * y));
	const double rlen = std::sqrt(rx * rx + rz * rz);
	const double flen = std::sqrt(fx * fx + fz * fz);
	if (rlen < 0.1 || flen < 0.1) return false;
	right_xz[0] = rx / rlen;
	right_xz[1] = rz / rlen;
	fwd_xz[0] = fx / flen;
	fwd_xz[1] = fz / flen;
	return true;
}

namespace detail {

inline std::vector<std::string> SplitCsv(const std::string& line)
{
	std::vector<std::string> out;
	std::string cur;
	for (char ch : line) {
		if (ch == ',') {
			out.push_back(cur);
			cur.clear();
		}
		else if (ch != '\r') {
			cur.push_back(ch);
		}
	}
	out.push_back(cur);
	return out;
}

inline double ToD(const std::string& s)
{
	try {
		return std::stod(s);
	}
	catch (...) {
		return 0.0;
	}
}

} // namespace detail

// Parse phantom_replay_v1 lines into samples. Comment (#) and header rows are
// skipped; malformed rows are skipped rather than failing the whole parse.
inline ParsedReplay ParseReplay(const std::vector<std::string>& lines)
{
	ParsedReplay out;
	for (const std::string& raw : lines) {
		if (raw.empty() || raw[0] == '#') continue;
		if (raw.rfind("time_ms", 0) == 0) continue; // header row
		const auto f = detail::SplitCsv(raw);
		if (f.size() < 20) continue;
		ReplaySample s;
		s.time_ms = detail::ToD(f[0]);
		s.device_id = static_cast<uint32_t>(detail::ToD(f[1]));
		s.serial = f[2];
		s.device_class = f[3];
		s.ground_truth_role = BodyRoleFromKey(f[5].c_str());
		s.pose_valid = detail::ToD(f[7]) != 0.0;
		s.pos[0] = detail::ToD(f[10]);
		s.pos[1] = detail::ToD(f[11]);
		s.pos[2] = detail::ToD(f[12]);
		s.quat[0] = detail::ToD(f[13]);
		s.quat[1] = detail::ToD(f[14]);
		s.quat[2] = detail::ToD(f[15]);
		s.quat[3] = detail::ToD(f[16]);
		out.samples.push_back(std::move(s));
	}
	out.ok = !out.samples.empty();
	if (!out.ok) out.error = "no samples parsed";
	return out;
}

// The real-world extra-tracker role set passive inference + snap arbitrate over.
// A local copy keeps this analysis header free of driver-module internals.
inline const std::vector<BodyRole>& InferenceCandidateRolesForScore()
{
	static const std::vector<BodyRole> kRoles = {BodyRole::Waist,     BodyRole::Chest,     BodyRole::LeftFoot,
	                                             BodyRole::RightFoot, BodyRole::LeftKnee,  BodyRole::RightKnee,
	                                             BodyRole::LeftElbow, BodyRole::RightElbow};
	return kRoles;
}

struct ReplayTrackerScore
{
	std::string serial;
	BodyRole ground_truth = BodyRole::None;
	BodyRole passive_predicted = BodyRole::None;
	float passive_confidence = 0.0f;
	double passive_detect_ms = -1.0; // first time passive matched truth (-1 = never)
	BodyRole snap_predicted = BodyRole::None;
	float snap_confidence = 0.0f;
};

struct ReplayScore
{
	std::vector<ReplayTrackerScore> trackers;
	uint32_t total = 0;
	uint32_t passive_correct = 0;
	uint32_t snap_correct = 0;
	double duration_ms = 0.0;
	SnapStatus snap_status = SnapStatus::NoTrackers;
};

// Replay the samples through passive inference (accumulating ~every 100 ms of
// capture time) and a final snap, scoring both against the ground-truth roles.
// floor_y normalizes heights; capture space usually has the floor near 0.
inline ReplayScore ScoreReplay(const std::vector<ReplaySample>& samples, double floor_y = 0.0,
                               InferenceParams params = {})
{
	ReplayScore score;

	std::vector<std::string> order;                // tracker serials in first-seen order
	std::unordered_map<std::string, size_t> index; // serial -> slot
	std::vector<RoleInferenceAccumulator> accums;
	std::vector<BodyRole> truth;
	std::vector<double> last_pos_x, last_pos_y, last_pos_z;
	std::vector<BodyRole> passive_pred;
	std::vector<float> passive_conf;
	std::vector<double> detect_ms;

	auto slot_for = [&](const ReplaySample& s) -> size_t {
		auto it = index.find(s.serial);
		if (it != index.end()) return it->second;
		const size_t i = order.size();
		index[s.serial] = i;
		order.push_back(s.serial);
		accums.emplace_back();
		truth.push_back(s.ground_truth_role);
		last_pos_x.push_back(0.0);
		last_pos_y.push_back(0.0);
		last_pos_z.push_back(0.0);
		passive_pred.push_back(BodyRole::None);
		passive_conf.push_back(0.0f);
		detect_ms.push_back(-1.0);
		return i;
	};

	double hmd_pos[3] = {0.0, 0.0, 0.0};
	double hmd_quat[4] = {1.0, 0.0, 0.0, 0.0};
	bool hmd_valid = false;
	double last_infer_ms = -1e9;
	double first_ms = 0.0, last_ms = 0.0;
	bool any = false;

	const auto& candidates = InferenceCandidateRolesForScore();

	auto run_passive = [&](double now_ms) {
		std::vector<TrackerMotionFeatures> feats;
		std::vector<size_t> feat_slot;
		for (size_t i = 0; i < accums.size(); ++i) {
			const auto fcomp = accums[i].Compute();
			if (!fcomp.has_data) continue;
			feats.push_back(fcomp);
			feat_slot.push_back(i);
		}
		if (feats.empty()) return;
		const auto result = InferRoles(feats, candidates, params);
		for (size_t k = 0; k < result.size(); ++k) {
			const size_t i = feat_slot[k];
			passive_pred[i] = result[k].role;
			passive_conf[i] = result[k].confidence;
			if (detect_ms[i] < 0.0 && result[k].role != BodyRole::None && result[k].role == truth[i]) {
				detect_ms[i] = now_ms;
			}
		}
	};

	for (const ReplaySample& s : samples) {
		if (!any) {
			first_ms = s.time_ms;
			any = true;
		}
		last_ms = s.time_ms;

		if (s.device_class == "hmd") {
			if (s.pose_valid) {
				hmd_pos[0] = s.pos[0];
				hmd_pos[1] = s.pos[1];
				hmd_pos[2] = s.pos[2];
				hmd_quat[0] = s.quat[0];
				hmd_quat[1] = s.quat[1];
				hmd_quat[2] = s.quat[2];
				hmd_quat[3] = s.quat[3];
				hmd_valid = true;
			}
			continue;
		}
		if (s.device_class != "tracker") continue;

		const size_t i = slot_for(s);
		if (truth[i] == BodyRole::None) truth[i] = s.ground_truth_role;
		last_pos_x[i] = s.pos[0];
		last_pos_y[i] = s.pos[1];
		last_pos_z[i] = s.pos[2];

		if (hmd_valid && s.pose_valid) {
			double right[2], fwd[2];
			if (HorizontalAxesFromQuat(hmd_quat, right, fwd)) {
				accums[i].AddSample(hmd_pos, right, fwd, s.pos, floor_y);
			}
		}

		if (s.time_ms - last_infer_ms >= 100.0) {
			last_infer_ms = s.time_ms;
			run_passive(s.time_ms);
		}
	}
	run_passive(last_ms);

	// Final snap from the last-seen tracker positions + last HMD pose.
	double right[2], fwd[2];
	const bool axes_valid = hmd_valid && HorizontalAxesFromQuat(hmd_quat, right, fwd);
	std::vector<SnapTrackerInput> snap_in;
	for (size_t i = 0; i < order.size(); ++i) {
		SnapTrackerInput t;
		t.id = static_cast<uint32_t>(i);
		t.pos[0] = last_pos_x[i];
		t.pos[1] = last_pos_y[i];
		t.pos[2] = last_pos_z[i];
		snap_in.push_back(t);
	}
	const SnapResult snap = SnapCalibrate(hmd_pos, axes_valid, right, fwd, floor_y, snap_in, params);
	score.snap_status = snap.status;

	std::vector<BodyRole> snap_pred(order.size(), BodyRole::None);
	std::vector<float> snap_c(order.size(), 0.0f);
	for (const auto& a : snap.assignments) {
		if (a.id < snap_pred.size()) {
			snap_pred[a.id] = a.role;
			snap_c[a.id] = a.confidence;
		}
	}

	for (size_t i = 0; i < order.size(); ++i) {
		ReplayTrackerScore t;
		t.serial = order[i];
		t.ground_truth = truth[i];
		t.passive_predicted = passive_pred[i];
		t.passive_confidence = passive_conf[i];
		t.passive_detect_ms = detect_ms[i];
		t.snap_predicted = snap_pred[i];
		t.snap_confidence = snap_c[i];
		score.trackers.push_back(t);
		++score.total;
		if (t.ground_truth != BodyRole::None && t.passive_predicted == t.ground_truth) ++score.passive_correct;
		if (t.ground_truth != BodyRole::None && t.snap_predicted == t.ground_truth) ++score.snap_correct;
	}
	score.duration_ms = any ? (last_ms - first_ms) : 0.0;
	return score;
}

} // namespace phantom
