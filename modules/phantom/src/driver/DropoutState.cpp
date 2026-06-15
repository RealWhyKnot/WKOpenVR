#include "DropoutState.h"

#include "BlendCurves.h"

#include <algorithm>

namespace phantom {

LadderTimings LadderTimings::Defaults()
{
	return LadderTimings{
	    /*dropout_silence_ms=*/DefaultTimings::kDropoutSilenceMs,
	    /*blend_out_ms=*/DefaultTimings::kBlendOutMs,
	    /*blend_in_ms=*/DefaultTimings::kBlendInMs,
	    /*reckon_hold_ms=*/DefaultTimings::kReckonHoldMs,
	    /*synth_hold_ms=*/DefaultTimings::kSynthHoldMs,
	    /*lost_hold_ms=*/DefaultTimings::kLostHoldMs,
	};
}

DropoutState::DropoutState() : timings_(LadderTimings::Defaults()) {}

namespace {

inline int64_t QpcMs(int64_t dt_ns, int64_t qpc_freq)
{
	if (qpc_freq <= 0 || dt_ns <= 0) return 0;
	return (dt_ns * 1000) / qpc_freq;
}

} // namespace

void DropoutState::OnRealPoseObserved(int64_t qpc_ns, const PoseHistory& /*history*/, const vr::DriverPose_t& observed)
{
	if (observed.result == vr::TrackingResult_Running_OK && observed.poseIsValid && observed.deviceIsConnected) {
		// A pose flagged Running_OK is a "real" observation. Other tracking
		// results (Running_OutOfRange, Calibrating_*) must not reset this
		// timestamp; some drivers keep emitting bad poses at frame rate during
		// dropout, and treating those as fresh real samples pins the ladder in
		// REAL forever.
		last_real_qpc_ns_ = qpc_ns;
		last_real_pose_ = observed;

		// Healthy real pose. If we were recovering, finish the blend back to
		// REAL once the BLEND_IN window has elapsed (Tick handles the time
		// transition; here we just remember that a fresh real pose arrived).
		if (state_ == TrackerState::OUT_OF_RANGE || state_ == TrackerState::LOST ||
		    state_ == TrackerState::SYNTH_RECKON || state_ == TrackerState::SYNTH_IK ||
		    state_ == TrackerState::SYNTH_ML) {
			state_ = TrackerState::BLEND_IN;
			blend_in_start_qpc_ns_ = qpc_ns;
		}
		// If currently BLEND_OUT (we never finished entering synth before the
		// real signal returned), short-circuit back to REAL by sliding into
		// BLEND_IN as well; the blend-back window will absorb any partial
		// BLEND_OUT progress.
		else if (state_ == TrackerState::BLEND_OUT) {
			state_ = TrackerState::BLEND_IN;
			blend_in_start_qpc_ns_ = qpc_ns;
		}
	}
}

void DropoutState::Tick(int64_t qpc_ns, int64_t qpc_freq)
{
	// Silence detection: if it has been > kDropoutSilenceMs since the last
	// real observation and we are still in REAL, kick into BLEND_OUT.
	const int64_t silence_ms = (last_real_qpc_ns_ < 0) ? 0 : QpcMs(qpc_ns - last_real_qpc_ns_, qpc_freq);

	if (state_ == TrackerState::REAL && last_real_qpc_ns_ >= 0 && silence_ms >= timings_.dropout_silence_ms) {
		state_ = TrackerState::BLEND_OUT;
		// Anchor the dropout to when the real signal actually vanished, not
		// when we noticed it. Otherwise a single late Tick after seconds of
		// silence escalates only by the Tick-delta and stays at BLEND_OUT,
		// leaving the published ETrackingResult at Running_OK long past
		// synth_hold_ms -- the exact frozen-tracker-wedge regression the
		// FrozenTrackerInvariant test exists to prevent.
		dropout_start_qpc_ns_ = last_real_qpc_ns_;
		dropout_anchor_ = last_real_pose_;
		++dropout_count_;
	}

	// Time-based escalation through the ladder. All times are measured from
	// dropout_start_qpc_ns_ (the moment the silence threshold tripped).
	if (dropout_start_qpc_ns_ >= 0 && state_ != TrackerState::REAL && state_ != TrackerState::BLEND_IN) {
		const int64_t age_ms = QpcMs(qpc_ns - dropout_start_qpc_ns_, qpc_freq);

		// BLEND_OUT -> SYNTH_RECKON after the blend window completes.
		if (state_ == TrackerState::BLEND_OUT && age_ms >= timings_.blend_out_ms) {
			state_ = TrackerState::SYNTH_RECKON;
		}

		// SYNTH_RECKON: the dead-reckoner is the source until either
		// role evidence hands off to SYNTH_IK at kReckonHoldMs or the
		// wall clock pushes through to OUT_OF_RANGE / LOST. When IK is
		// not available the ladder stays in SYNTH_RECKON and the damping
		// in DeadReckoner carries the avatar to rest.
		if (state_ == TrackerState::SYNTH_RECKON && ik_available_ && age_ms >= timings_.reckon_hold_ms) {
			state_ = TrackerState::SYNTH_IK;
		}

		// Past synth_hold_ms: flip to OUT_OF_RANGE so VRChat-style
		// consumers drop the tracker from their IK chain.
		if (age_ms >= timings_.synth_hold_ms && state_ != TrackerState::OUT_OF_RANGE && state_ != TrackerState::LOST) {
			state_ = TrackerState::OUT_OF_RANGE;
		}

		// Past lost_hold_ms: stop publishing on this device entirely.
		if (age_ms >= timings_.lost_hold_ms && state_ != TrackerState::LOST) {
			state_ = TrackerState::LOST;
			longest_dropout_ms_ = std::max<uint32_t>(longest_dropout_ms_, static_cast<uint32_t>(age_ms));
		}
	}

	// BLEND_IN: once the blend-back window elapses, return to REAL and
	// record the dropout's total duration.
	if (state_ == TrackerState::BLEND_IN && blend_in_start_qpc_ns_ >= 0) {
		const int64_t blend_age_ms = QpcMs(qpc_ns - blend_in_start_qpc_ns_, qpc_freq);
		if (blend_age_ms >= timings_.blend_in_ms) {
			if (dropout_start_qpc_ns_ >= 0) {
				const int64_t total_dropout_ms = QpcMs(blend_in_start_qpc_ns_ - dropout_start_qpc_ns_, qpc_freq);
				longest_dropout_ms_ = std::max<uint32_t>(longest_dropout_ms_,
				                                         static_cast<uint32_t>(std::max<int64_t>(0, total_dropout_ms)));
			}
			state_ = TrackerState::REAL;
			dropout_start_qpc_ns_ = -1;
			blend_in_start_qpc_ns_ = -1;
		}
	}
}

vr::ETrackingResult DropoutState::tracking_result_override() const
{
	if (state_ == TrackerState::OUT_OF_RANGE) {
		return vr::TrackingResult_Running_OutOfRange;
	}
	return vr::TrackingResult_Running_OK;
}

double DropoutState::blend_alpha(int64_t qpc_ns, int64_t qpc_freq) const
{
	if (state_ == TrackerState::BLEND_OUT && dropout_start_qpc_ns_ >= 0) {
		const int64_t age = QpcMs(qpc_ns - dropout_start_qpc_ns_, qpc_freq);
		return std::clamp(static_cast<double>(age) / static_cast<double>(std::max<uint32_t>(1, timings_.blend_out_ms)),
		                  0.0, 1.0);
	}
	if (state_ == TrackerState::BLEND_IN && blend_in_start_qpc_ns_ >= 0) {
		const int64_t age = QpcMs(qpc_ns - blend_in_start_qpc_ns_, qpc_freq);
		return std::clamp(static_cast<double>(age) / static_cast<double>(std::max<uint32_t>(1, timings_.blend_in_ms)),
		                  0.0, 1.0);
	}
	return 0.0;
}

uint32_t DropoutState::dropout_age_ms(int64_t qpc_ns, int64_t qpc_freq) const
{
	if (state_ == TrackerState::REAL || dropout_start_qpc_ns_ < 0) return 0;
	return static_cast<uint32_t>(std::max<int64_t>(0, QpcMs(qpc_ns - dropout_start_qpc_ns_, qpc_freq)));
}

} // namespace phantom
