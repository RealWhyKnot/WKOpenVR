#pragma once

// Pure helper for driver-level HMD pose synthesis from a head-mounted tracker.
//
// Extracted from ServerTrackedDeviceProvider::HandleDevicePoseUpdated so unit
// tests can exercise the compose/gate logic without a live OpenVR runtime.
// All functions are pure: no static state, no global CalCtx access, no vr::*
// calls beyond struct field access.
// The driver calls these helpers in all build channels; tests include the
// same header directly so the compose and blend gates stay covered.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>

#include <openvr_driver.h>

#include "HeadMountDriverSynthConfig.h"

namespace driver_synth {

// Parameters captured from the cached HeadMountDriverState. Plain POD so the
// caller can copy them out of the mutex-guarded state in one memcpy and pass
// a stack copy here.
struct SynthState
{
	int mode;         // HeadMountMode integer; 3 = DriverSynth
	int32_t deviceId; // -1 means unresolved
	double headFromTrackerTrans[3];
	double headFromTrackerRot[4]; // xyzw
	bool offsetCalibrated;
};

// A snapshot of a tracker pose and when it was captured. capturedForDeviceId
// records which deviceId the snapshot was filed under so the synth path can
// reject snapshots written for a previously-selected tracker after the user
// switches which device drives the head-mount.
struct TrackerSnapshot
{
	vr::DriverPose_t pose;
	std::chrono::steady_clock::time_point capturedAt;
	int32_t capturedForDeviceId = -1;
	bool valid = false; // false until at least one pose is stored
};

// Freshness limit. Any tracker snapshot older than this is treated as stale.
constexpr int64_t kStaleLimitMs = wkopenvr::headmount::kDriverSynthStaleLimitMsDefault;

constexpr int64_t kGraceHoldMs = wkopenvr::headmount::kDriverSynthGraceHoldMsDefault;
constexpr int64_t kBlendToFallbackMs = wkopenvr::headmount::kDriverSynthBlendToFallbackMsDefault;
constexpr int64_t kStableBeforeSynthMs = wkopenvr::headmount::kDriverSynthStableBeforeSynthMsDefault;
constexpr int64_t kBlendToSynthMs = wkopenvr::headmount::kDriverSynthBlendToSynthMsDefault;

// Returns true when the snapshot is present, valid, and not older than
// kStaleLimitMs relative to `now`.
inline bool IsTrackerFresh(const TrackerSnapshot& snap, std::chrono::steady_clock::time_point now,
                           int64_t staleLimitMs = kStaleLimitMs)
{
	if (!snap.valid) return false;
	if (!snap.pose.poseIsValid) return false;
	using ms = std::chrono::milliseconds;
	return std::chrono::duration_cast<ms>(now - snap.capturedAt).count() <= staleLimitMs;
}

inline int64_t SnapshotAgeMs(const TrackerSnapshot& snap, std::chrono::steady_clock::time_point now)
{
	if (!snap.valid) return -1;
	using ms = std::chrono::milliseconds;
	return std::chrono::duration_cast<ms>(now - snap.capturedAt).count();
}

// Compose a synthesized HMD pose from the tracker pose and state.
//
// Returns true and writes into `out` when all preconditions hold:
//   - state.mode == 3 (DriverSynth)
//   - state.deviceId >= 0 (tracker resolved)
//   - state.offsetCalibrated (offset has been through the solver)
//   - trackerSnap.valid && trackerSnap.pose.poseIsValid
//   - tracker snapshot not stale
//
// Returns false (pass through) on any failure.
inline bool Compose(const SynthState& state, const TrackerSnapshot& trackerSnap,
                    std::chrono::steady_clock::time_point now, vr::DriverPose_t& out,
                    const wkopenvr::headmount::DriverSynthTimingConfig& config = {})
{
	// Mode and resolver gate.
	if (state.mode != 3 /*DriverSynth*/) return false;
	if (state.deviceId < 0) return false;
	if (!state.offsetCalibrated) return false;

	// Tracker freshness.
	const auto timing = wkopenvr::headmount::ClampDriverSynthTimingConfig(config);
	if (!IsTrackerFresh(trackerSnap, now, timing.staleLimitMs)) return false;

	// Reject snapshots captured for a different device. If the user switched
	// which tracker drives the head-mount between snapshot write and synth
	// read, the cached pose belongs to the previous device and cannot be
	// safely composed against the current state's offset.
	if (trackerSnap.capturedForDeviceId != state.deviceId) return false;

	const vr::DriverPose_t& trackerPose = trackerSnap.pose;

	// Build the synthesized pose. Start from the tracker pose to inherit
	// position, rotation, velocities, poseTimeOffset, validity fields, and the
	// calibrated worldFromDriver transform supplied by the driver call site.
	out = trackerPose;

	// Override the driver-from-head transform with our configured offset.
	// This is the OpenVR-native way to express "tracker origin -> IPD midpoint"
	// per the Driver API Documentation: qDriverFromHeadRotation and
	// vecDriverFromHeadTranslation encode how to get from the driver's local
	// coordinate frame (the tracker) to the "head" (IPD midpoint) pose that
	// SteamVR renders from.
	out.qDriverFromHeadRotation.x = state.headFromTrackerRot[0];
	out.qDriverFromHeadRotation.y = state.headFromTrackerRot[1];
	out.qDriverFromHeadRotation.z = state.headFromTrackerRot[2];
	out.qDriverFromHeadRotation.w = state.headFromTrackerRot[3];
	out.vecDriverFromHeadTranslation[0] = state.headFromTrackerTrans[0];
	out.vecDriverFromHeadTranslation[1] = state.headFromTrackerTrans[1];
	out.vecDriverFromHeadTranslation[2] = state.headFromTrackerTrans[2];

	return true;
}

enum class SourceBlendPhase
{
	Uninitialized,
	SynthStable,
	GraceHold,
	BlendingToFallback,
	FallbackStable,
	WaitingForStableSynth,
	BlendingToSynth
};

inline const char* PhaseName(SourceBlendPhase phase)
{
	switch (phase) {
		case SourceBlendPhase::Uninitialized:
			return "uninitialized";
		case SourceBlendPhase::SynthStable:
			return "synth_stable";
		case SourceBlendPhase::GraceHold:
			return "grace_hold";
		case SourceBlendPhase::BlendingToFallback:
			return "blend_to_fallback";
		case SourceBlendPhase::FallbackStable:
			return "fallback_stable";
		case SourceBlendPhase::WaitingForStableSynth:
			return "waiting_for_stable_synth";
		case SourceBlendPhase::BlendingToSynth:
			return "blend_to_synth";
	}
	return "unknown";
}

using SourceBlendConfig = wkopenvr::headmount::DriverSynthTimingConfig;

struct SourceBlendState
{
	SourceBlendPhase phase = SourceBlendPhase::Uninitialized;
	vr::DriverPose_t currentPose{};
	vr::DriverPose_t lastGoodSynthPose{};
	vr::DriverPose_t blendStartPose{};
	std::chrono::steady_clock::time_point phaseStartedAt{};
	std::chrono::steady_clock::time_point stableSynthSince{};
	bool hasCurrentPose = false;
	bool hasLastGoodSynthPose = false;
	bool hasStableSynthSince = false;
};

struct SourceBlendResult
{
	SourceBlendPhase previousPhase = SourceBlendPhase::Uninitialized;
	SourceBlendPhase phase = SourceBlendPhase::Uninitialized;
	double alpha = 0.0;
	bool phaseChanged = false;
};

inline double Clamp01(double x)
{
	return std::clamp(x, 0.0, 1.0);
}

inline double Smoothstep(double x)
{
	x = Clamp01(x);
	return x * x * (3.0 - 2.0 * x);
}

inline double DurationAlpha(std::chrono::steady_clock::time_point start, std::chrono::steady_clock::time_point now,
                            int64_t durationMs)
{
	if (durationMs <= 0) return 1.0;
	using msd = std::chrono::duration<double, std::milli>;
	const double elapsedMs = std::chrono::duration_cast<msd>(now - start).count();
	return Clamp01(elapsedMs / static_cast<double>(durationMs));
}

inline double QuatDot(const vr::HmdQuaternion_t& a, const vr::HmdQuaternion_t& b)
{
	return a.w * b.w + a.x * b.x + a.y * b.y + a.z * b.z;
}

inline void NormalizeQuat(vr::HmdQuaternion_t& q)
{
	const double n = std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
	if (n > 1e-12) {
		q.w /= n;
		q.x /= n;
		q.y /= n;
		q.z /= n;
	}
	else {
		q.w = 1.0;
		q.x = q.y = q.z = 0.0;
	}
}

inline void SlerpQuat(const vr::HmdQuaternion_t& a, const vr::HmdQuaternion_t& b, double alpha,
                      vr::HmdQuaternion_t& out)
{
	alpha = Clamp01(alpha);
	double dot = QuatDot(a, b);
	vr::HmdQuaternion_t bAdj = b;
	if (dot < 0.0) {
		bAdj.w = -b.w;
		bAdj.x = -b.x;
		bAdj.y = -b.y;
		bAdj.z = -b.z;
		dot = -dot;
	}
	dot = std::clamp(dot, -1.0, 1.0);
	if (dot > 0.9995) {
		out.w = a.w + alpha * (bAdj.w - a.w);
		out.x = a.x + alpha * (bAdj.x - a.x);
		out.y = a.y + alpha * (bAdj.y - a.y);
		out.z = a.z + alpha * (bAdj.z - a.z);
	}
	else {
		const double theta = std::acos(dot);
		const double sinT = std::sin(theta);
		const double wa = std::sin((1.0 - alpha) * theta) / sinT;
		const double wb = std::sin(alpha * theta) / sinT;
		out.w = wa * a.w + wb * bAdj.w;
		out.x = wa * a.x + wb * bAdj.x;
		out.y = wa * a.y + wb * bAdj.y;
		out.z = wa * a.z + wb * bAdj.z;
	}
	NormalizeQuat(out);
}

inline void BlendPose(const vr::DriverPose_t& a, const vr::DriverPose_t& b, double alpha, vr::DriverPose_t& out)
{
	const double s = Smoothstep(alpha);
	out = (s >= 0.5) ? b : a;
	for (int i = 0; i < 3; ++i) {
		out.vecPosition[i] = a.vecPosition[i] * (1.0 - s) + b.vecPosition[i] * s;
		out.vecVelocity[i] = a.vecVelocity[i] * (1.0 - s) + b.vecVelocity[i] * s;
		out.vecAcceleration[i] = a.vecAcceleration[i] * (1.0 - s) + b.vecAcceleration[i] * s;
		out.vecAngularVelocity[i] = a.vecAngularVelocity[i] * (1.0 - s) + b.vecAngularVelocity[i] * s;
		out.vecAngularAcceleration[i] = a.vecAngularAcceleration[i] * (1.0 - s) + b.vecAngularAcceleration[i] * s;
		out.vecWorldFromDriverTranslation[i] =
		    a.vecWorldFromDriverTranslation[i] * (1.0 - s) + b.vecWorldFromDriverTranslation[i] * s;
		out.vecDriverFromHeadTranslation[i] =
		    a.vecDriverFromHeadTranslation[i] * (1.0 - s) + b.vecDriverFromHeadTranslation[i] * s;
	}
	SlerpQuat(a.qRotation, b.qRotation, s, out.qRotation);
	SlerpQuat(a.qWorldFromDriverRotation, b.qWorldFromDriverRotation, s, out.qWorldFromDriverRotation);
	SlerpQuat(a.qDriverFromHeadRotation, b.qDriverFromHeadRotation, s, out.qDriverFromHeadRotation);
	out.poseTimeOffset = a.poseTimeOffset * (1.0 - s) + b.poseTimeOffset * s;
	out.poseIsValid = a.poseIsValid || b.poseIsValid;
	out.deviceIsConnected = a.deviceIsConnected || b.deviceIsConnected;
}

inline void SetPhase(SourceBlendState& state, SourceBlendPhase phase, std::chrono::steady_clock::time_point now)
{
	if (state.phase != phase) {
		state.phase = phase;
		state.phaseStartedAt = now;
	}
}

inline SourceBlendResult StepSourceBlend(SourceBlendState& state, const vr::DriverPose_t& fallbackPose,
                                         const vr::DriverPose_t* synthPose, bool synthAvailable,
                                         std::chrono::steady_clock::time_point now, vr::DriverPose_t& out,
                                         const SourceBlendConfig& config = {})
{
	const SourceBlendConfig timing = wkopenvr::headmount::ClampDriverSynthTimingConfig(config);
	const SourceBlendPhase previous = state.phase;
	auto finish = [&](SourceBlendPhase phase, double alpha) {
		state.currentPose = out;
		state.hasCurrentPose = true;
		return SourceBlendResult{previous, phase, alpha, previous != phase};
	};

	if (!state.hasCurrentPose) {
		if (synthAvailable && synthPose) {
			out = *synthPose;
			state.lastGoodSynthPose = *synthPose;
			state.hasLastGoodSynthPose = true;
			state.hasStableSynthSince = true;
			state.stableSynthSince = now;
			SetPhase(state, SourceBlendPhase::SynthStable, now);
			return finish(state.phase, 1.0);
		}
		out = fallbackPose;
		state.hasStableSynthSince = false;
		SetPhase(state, SourceBlendPhase::FallbackStable, now);
		return finish(state.phase, 0.0);
	}

	if (synthAvailable && synthPose) {
		state.lastGoodSynthPose = *synthPose;
		state.hasLastGoodSynthPose = true;

		if (state.phase == SourceBlendPhase::SynthStable || state.phase == SourceBlendPhase::GraceHold) {
			out = *synthPose;
			state.hasStableSynthSince = true;
			state.stableSynthSince = now;
			SetPhase(state, SourceBlendPhase::SynthStable, now);
			return finish(state.phase, 1.0);
		}

		if (!state.hasStableSynthSince) {
			state.hasStableSynthSince = true;
			state.stableSynthSince = now;
		}

		const double stableAlpha = DurationAlpha(state.stableSynthSince, now, timing.stableBeforeSynthMs);
		if (stableAlpha < 1.0 && state.phase != SourceBlendPhase::BlendingToSynth) {
			if (state.phase == SourceBlendPhase::BlendingToFallback) {
				const double fallbackAlpha = DurationAlpha(state.phaseStartedAt, now, timing.blendToFallbackMs);
				if (fallbackAlpha >= 1.0) {
					out = fallbackPose;
					SetPhase(state, SourceBlendPhase::FallbackStable, now);
					return finish(state.phase, 1.0);
				}
				BlendPose(state.blendStartPose, fallbackPose, fallbackAlpha, out);
				return finish(state.phase, fallbackAlpha);
			}
			out = fallbackPose;
			SetPhase(state, SourceBlendPhase::WaitingForStableSynth, now);
			return finish(state.phase, stableAlpha);
		}

		if (state.phase != SourceBlendPhase::BlendingToSynth) {
			state.blendStartPose = state.hasCurrentPose ? state.currentPose : fallbackPose;
			SetPhase(state, SourceBlendPhase::BlendingToSynth, now);
		}

		const double alpha = DurationAlpha(state.phaseStartedAt, now, timing.blendToSynthMs);
		if (alpha >= 1.0) {
			out = *synthPose;
			SetPhase(state, SourceBlendPhase::SynthStable, now);
			return finish(state.phase, 1.0);
		}
		BlendPose(state.blendStartPose, *synthPose, alpha, out);
		return finish(state.phase, alpha);
	}

	state.hasStableSynthSince = false;

	if (state.phase == SourceBlendPhase::SynthStable) {
		state.blendStartPose = state.hasLastGoodSynthPose ? state.lastGoodSynthPose : state.currentPose;
		SetPhase(state, SourceBlendPhase::GraceHold, now);
	}

	if (state.phase == SourceBlendPhase::GraceHold) {
		const double alpha = DurationAlpha(state.phaseStartedAt, now, timing.graceHoldMs);
		if (alpha < 1.0) {
			out = state.blendStartPose;
			return finish(state.phase, alpha);
		}
		SetPhase(state, SourceBlendPhase::BlendingToFallback, now);
	}
	else if (state.phase == SourceBlendPhase::BlendingToSynth ||
	         state.phase == SourceBlendPhase::WaitingForStableSynth) {
		state.blendStartPose = state.currentPose;
		SetPhase(state, SourceBlendPhase::BlendingToFallback, now);
	}

	if (state.phase == SourceBlendPhase::BlendingToFallback) {
		const double alpha = DurationAlpha(state.phaseStartedAt, now, timing.blendToFallbackMs);
		if (alpha >= 1.0) {
			out = fallbackPose;
			SetPhase(state, SourceBlendPhase::FallbackStable, now);
			return finish(state.phase, 1.0);
		}
		BlendPose(state.blendStartPose, fallbackPose, alpha, out);
		return finish(state.phase, alpha);
	}

	out = fallbackPose;
	SetPhase(state, SourceBlendPhase::FallbackStable, now);
	return finish(state.phase, 1.0);
}

} // namespace driver_synth
