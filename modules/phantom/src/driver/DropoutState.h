#pragma once

#include "PhantomTypes.h"
#include "PoseHistory.h"

#include <openvr_driver.h>

#include <cstdint>

namespace phantom {

// User-tunable timing knobs for the dropout ladder. Defaults come from
// BlendCurves.h. The driver receives values from the overlay via
// RequestSetPhantomConfig and falls back to defaults on first launch.
struct LadderTimings
{
	uint32_t dropout_silence_ms;
	uint32_t blend_out_ms;
	uint32_t blend_in_ms;
	uint32_t reckon_hold_ms;
	uint32_t synth_hold_ms;
	uint32_t lost_hold_ms;

	static LadderTimings Defaults();
};

// Per-device dropout state machine + degradation ladder. The two concepts
// are tightly coupled (state transitions are driven by elapsed time, and
// the ladder IS the time-based escalation), so they live together rather
// than in two thin classes that share the same data.
//
// Usage from the hook:
//   OnRealPoseObserved(qpc_ns, pose)  -- call from the inbound side of the
//                                         hook with every real pose the
//                                         driver sees on this device.
//   Tick(qpc_ns, qpc_freq)            -- call before deciding whether to
//                                         override the outgoing pose;
//                                         advances the state machine based
//                                         on elapsed silence.
//   state()                           -- current TrackerState.
//   should_publish()                  -- false in LOST: caller skips the
//                                         downstream TrackedDevicePoseUpdated.
//   tracking_result_override()        -- ETrackingResult to stamp on the
//                                         published pose; OUT_OF_RANGE for
//                                         the OUT_OF_RANGE state, Running_OK
//                                         otherwise.
//   blend_alpha()                     -- 0..1 fraction for the BlendController
//                                         during BLEND_OUT / BLEND_IN; meaningless
//                                         otherwise.
class DropoutState
{
public:
	DropoutState();

	void SetTimings(const LadderTimings& t) { timings_ = t; }
	const LadderTimings& timings() const { return timings_; }

	// Phase 1.5: tells the ladder whether an IK fallback is available for
	// this device's role. When true, the SYNTH_RECKON -> SYNTH_IK
	// transition fires at kReckonHoldMs; otherwise the ladder stays in
	// SYNTH_RECKON (damped dead reckoning) until OUT_OF_RANGE / LOST.
	// The bool is consulted on each Tick(); the caller updates it whenever
	// role evidence or a compatible legacy offset is available.
	void SetIkAvailable(bool v) { ik_available_ = v; }
	bool ik_available() const { return ik_available_; }

	// Record a real (driver-observed) pose. Updates the state machine and
	// resets silence trackers. The pose is also pushed into `history` by
	// the caller before this call.
	void OnRealPoseObserved(int64_t qpc_ns, const PoseHistory& history, const vr::DriverPose_t& observed);

	// Advance the time-based state. Call once per outgoing pose before
	// emitting (or skipping) it. `qpc_freq` is the QPC counter frequency.
	void Tick(int64_t qpc_ns, int64_t qpc_freq);

	TrackerState state() const { return state_; }
	bool should_publish() const { return state_ != TrackerState::LOST; }
	vr::ETrackingResult tracking_result_override() const;

	// 0..1 blend fraction. Caller uses it as the target-pose weight in
	// BlendController::Lerp. 0 = pure real, 1 = pure synth (in BLEND_OUT) or
	// pure real (in BLEND_IN). Meaningless outside blend states.
	double blend_alpha(int64_t qpc_ns, int64_t qpc_freq) const;

	// The "anchor" pose recorded at the moment the dropout was detected.
	// BLEND_OUT uses this as the "real" pose to fade from. Updated only on
	// REAL -> BLEND_OUT transition.
	const vr::DriverPose_t& dropout_anchor() const { return dropout_anchor_; }

	// Cumulative session metrics for overlay readout.
	uint32_t dropout_count() const { return dropout_count_; }
	uint32_t longest_dropout_ms() const { return longest_dropout_ms_; }
	uint32_t dropout_age_ms(int64_t qpc_ns, int64_t qpc_freq) const;

private:
	LadderTimings timings_;
	TrackerState state_ = TrackerState::REAL;
	bool ik_available_ = false;

	// QPC of the most recent observed real pose; -1 = never.
	int64_t last_real_qpc_ns_ = -1;

	// QPC at which the current dropout-ladder cycle started (the REAL ->
	// BLEND_OUT transition). -1 when state_ == REAL.
	int64_t dropout_start_qpc_ns_ = -1;

	// QPC at which BLEND_IN entered (real signal returned and the fade-back
	// is in progress). -1 when not blending in.
	int64_t blend_in_start_qpc_ns_ = -1;

	// Snapshot of the last real pose at the moment dropout was detected, for
	// the BLEND_OUT lerp source.
	vr::DriverPose_t dropout_anchor_{};
	vr::DriverPose_t last_real_pose_{};

	uint32_t dropout_count_ = 0;
	uint32_t longest_dropout_ms_ = 0;
};

} // namespace phantom
