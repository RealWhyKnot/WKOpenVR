#pragma once

#include "PredictionSmoothingMath.h"

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace prediction::smart_shadow {

constexpr double kPi = 3.1415926535897932384626433832795;

inline double Clamp(double value, double lo, double hi)
{
	return std::min(hi, std::max(lo, value));
}

inline double Saturate(double value)
{
	return Clamp(value, 0.0, 1.0);
}

inline double Lerp(double a, double b, double t)
{
	return a + (b - a) * Saturate(t);
}

inline double SmoothStep(double edge0, double edge1, double x)
{
	if (!(edge1 > edge0)) return x >= edge1 ? 1.0 : 0.0;
	const double t = Saturate((x - edge0) / (edge1 - edge0));
	return t * t * (3.0 - 2.0 * t);
}

inline double AlphaFromCutoffHz(double cutoffHz, double dtSeconds)
{
	cutoffHz = Clamp(cutoffHz, 0.001, 120.0);
	dtSeconds = Clamp(dtSeconds, 0.001, 0.050);
	const double tau = 1.0 / (2.0 * kPi * cutoffHz);
	return dtSeconds / (dtSeconds + tau);
}

inline double AlphaFromTau(double tauSeconds, double dtSeconds)
{
	tauSeconds = std::max(0.0005, tauSeconds);
	dtSeconds = Clamp(dtSeconds, 0.001, 0.050);
	return dtSeconds / (dtSeconds + tauSeconds);
}

inline double UpdateEnvelope(double current, double target, double dtSeconds, double attackTauSeconds,
                             double releaseTauSeconds)
{
	target = Saturate(target);
	const double tau = target > current ? attackTauSeconds : releaseTauSeconds;
	const double alpha = AlphaFromTau(tau, dtSeconds);
	return Saturate(current + alpha * (target - current));
}

inline double ReleasedPredictionFactor(double baseFactor, double release)
{
	baseFactor = Saturate(baseFactor);
	release = Saturate(release);
	return baseFactor + (1.0 - baseFactor) * release;
}

struct Params
{
	double posMinCutoffHz = 12.0;
	double rotMinCutoffHz = 16.0;
	double posBetaHzPerMps = 6.0;
	double rotBetaHzPerRadps = 1.5;
	double derivCutoffHz = 4.0;
	double maxCutoffHz = 90.0;
	double basePredictionFactor = 1.0;

	double linDerivedDeadband = 0.03;
	double linReportedDeadband = 0.06;
	double angDerivedDeadband = 0.08;
	double angReportedDeadband = 0.15;

	double linStillSpeed = 0.03;
	double linMovingSpeed = 0.35;
	double angStillSpeed = 0.10;
	double angMovingSpeed = 1.20;

	double innovationTauSeconds = 0.08;
	double linInnovationDeadband = 0.0;
	double angInnovationDeadband = 0.0;
	double posErrStillM = 0.012;
	double posErrMovingM = 0.060;
	double rotErrStillRad = 0.01308996938995747;  // 0.75 deg
	double rotErrMovingRad = 0.10471975511965977; // 6 deg

	double gateAttackTauSeconds = 0.006;
	double gateReleaseTauSeconds = 0.045;

	double positionJumpM = 0.50;
	double rotationJumpRad = 1.0471975511965976; // 60 deg
	double resetGapSeconds = 0.25;
};

inline Params BuildParams(uint8_t smoothness)
{
	const double s01 = Clamp(static_cast<double>(smoothness) / 100.0, 0.0, 1.0);
	const double inv = 1.0 - s01;

	Params p;
	p.posMinCutoffHz = 0.45 + 16.0 * std::pow(inv, 1.8);
	p.rotMinCutoffHz = 0.75 + 18.0 * std::pow(inv, 1.6);
	p.posBetaHzPerMps = Lerp(6.0, 24.0, s01);
	p.rotBetaHzPerRadps = Lerp(1.5, 4.5, s01);
	p.basePredictionFactor = prediction::SmoothnessToFactor(smoothness);
	return p;
}

// Candidate selector for the dev A/B shadow. The shadow filter runs one of these
// variants against the live one-euro so a session's [smart-shadow] log shows the
// jitter-vs-lag tradeoff of a tuning we might promote.
enum class CandidateKind
{
	Match,
	Strong,
	Responsive
};

inline const char* CandidateKindName(CandidateKind k)
{
	switch (k) {
		case CandidateKind::Strong:
			return "strong";
		case CandidateKind::Responsive:
			return "responsive";
		case CandidateKind::Match:
		default:
			return "match";
	}
}

// Derive a candidate parameter set from the slider value.
//   Match      -- identical to live (sanity baseline; divergence ~0).
//   Strong     -- lower floor cutoffs => more rest smoothing, expect more lag.
//   Responsive -- higher beta + faster release => less motion lag, slightly more
//                 rest jitter.
inline Params BuildCandidateParams(uint8_t smoothness, CandidateKind kind)
{
	Params p = BuildParams(smoothness);
	switch (kind) {
		case CandidateKind::Strong:
			p.posMinCutoffHz *= 0.5;
			p.rotMinCutoffHz *= 0.5;
			break;
		case CandidateKind::Responsive:
			p.posBetaHzPerMps *= 1.6;
			p.rotBetaHzPerRadps *= 1.6;
			p.gateReleaseTauSeconds *= 0.5;
			break;
		case CandidateKind::Match:
		default:
			break;
	}
	return p;
}

// ---------------------------------------------------------------------------
// Shared per-sample filter (one-euro speed-adaptive low-pass).
//
// Originally lived only in the dev "shadow" comparison path. It is now the
// live smoothing filter for tracked devices as well, so the step is factored
// out here as a pure function over plain doubles (position as double[3],
// rotation as a wxyz double[4]) -- no OpenVR / vr:: types -- so it can be unit
// tested and called from both the live pose path and the dev comparison.
//
// The defining property versus a plain EWM: the cutoff frequency rises with
// speed, so the filter smooths hard at rest (low cutoff) yet adds little lag
// in motion (high cutoff). It never freezes -- even at rest the minimum cutoff
// keeps tracking slow drift -- which is the fix for the "sticks at rest then
// snaps when you move" behaviour of the old (1-s/100)^1.8 position EWM.
// ---------------------------------------------------------------------------

inline bool IsFinite3(const double v[3])
{
	return std::isfinite(v[0]) && std::isfinite(v[1]) && std::isfinite(v[2]);
}

inline double Length3(const double v[3])
{
	return std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

inline double Distance3(const double a[3], const double b[3])
{
	const double dx = a[0] - b[0];
	const double dy = a[1] - b[1];
	const double dz = a[2] - b[2];
	return std::sqrt(dx * dx + dy * dy + dz * dz);
}

// Quaternions are stored wxyz: q[0]=w, q[1]=x, q[2]=y, q[3]=z.
inline double QuatDot(const double a[4], const double b[4])
{
	return a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3];
}

// Normalises q into out. Returns false (and writes identity) for a non-finite
// or degenerate input.
inline bool QuatNormalize(const double q[4], double out[4])
{
	if (!(std::isfinite(q[0]) && std::isfinite(q[1]) && std::isfinite(q[2]) && std::isfinite(q[3]))) {
		out[0] = 1.0;
		out[1] = out[2] = out[3] = 0.0;
		return false;
	}
	const double n2 = QuatDot(q, q);
	if (!(n2 > 1e-18) || !std::isfinite(n2)) {
		out[0] = 1.0;
		out[1] = out[2] = out[3] = 0.0;
		return false;
	}
	const double inv = 1.0 / std::sqrt(n2);
	out[0] = q[0] * inv;
	out[1] = q[1] * inv;
	out[2] = q[2] * inv;
	out[3] = q[3] * inv;
	return true;
}

// Flip q (in place via out) to the hemisphere of reference, so lerps take the
// short path.
inline void QuatSameHemisphere(const double reference[4], const double q[4], double out[4])
{
	const double s = QuatDot(reference, q) < 0.0 ? -1.0 : 1.0;
	out[0] = q[0] * s;
	out[1] = q[1] * s;
	out[2] = q[2] * s;
	out[3] = q[3] * s;
}

// Geodesic angle (radians) between two quaternions; 0 if either is degenerate.
inline double QuatAngleRad(const double a[4], const double b[4])
{
	double na[4];
	double nb[4];
	if (!QuatNormalize(a, na) || !QuatNormalize(b, nb)) return 0.0;
	const double d = Clamp(std::abs(QuatDot(na, nb)), 0.0, 1.0);
	return 2.0 * std::acos(d);
}

// Normalised lerp along the short path. Writes the result to out.
inline void QuatNlerpShortest(const double from[4], const double to[4], double alpha, double out[4])
{
	double nf[4];
	double nt[4];
	const bool okF = QuatNormalize(from, nf);
	const bool okT = QuatNormalize(to, nt);
	if (!okF) {
		out[0] = nt[0];
		out[1] = nt[1];
		out[2] = nt[2];
		out[3] = nt[3];
		return;
	}
	if (!okT) {
		out[0] = nf[0];
		out[1] = nf[1];
		out[2] = nf[2];
		out[3] = nf[3];
		return;
	}

	double hemi[4];
	QuatSameHemisphere(nf, nt, hemi);
	alpha = Saturate(alpha);
	double blended[4] = {
	    nf[0] + alpha * (hemi[0] - nf[0]),
	    nf[1] + alpha * (hemi[1] - nf[1]),
	    nf[2] + alpha * (hemi[2] - nf[2]),
	    nf[3] + alpha * (hemi[3] - nf[3]),
	};
	QuatNormalize(blended, out);
}

// Mutable filter state. Carry one instance per tracked device. Seed once, then
// call Step every pose sample. filteredPos/filteredRot are the smoothed output.
struct FilterState
{
	bool initialized = false;
	double prevRawPos[3] = {0.0, 0.0, 0.0};
	double prevRawRot[4] = {1.0, 0.0, 0.0, 0.0};
	double filteredPos[3] = {0.0, 0.0, 0.0};
	double filteredRot[4] = {1.0, 0.0, 0.0, 0.0};
	double linearSpeedHat = 0.0;
	double angularSpeedHat = 0.0;
	double posRelease = 1.0;
	double rotRelease = 1.0;
};

// Per-sample diagnostics (used by the dev comparison log; harmless to ignore).
enum class ReseedReason
{
	None,
	Init,
	Gap,
	Jump
};
struct StepResult
{
	double posAlpha = 1.0;
	double rotAlpha = 1.0;
	double posRelease = 1.0;
	double rotRelease = 1.0;
	double posCutoffHz = 0.0;
	double rotCutoffHz = 0.0;
	bool reseeded = false;
	ReseedReason reseedReason = ReseedReason::None;
};

// Seed (or reseed) the filter so the output equals the raw input this frame.
inline void SeedFilter(FilterState& s, const double rawPos[3], const double rawRot[4])
{
	s.prevRawPos[0] = s.filteredPos[0] = rawPos[0];
	s.prevRawPos[1] = s.filteredPos[1] = rawPos[1];
	s.prevRawPos[2] = s.filteredPos[2] = rawPos[2];
	double n[4];
	QuatNormalize(rawRot, n);
	s.prevRawRot[0] = s.filteredRot[0] = n[0];
	s.prevRawRot[1] = s.filteredRot[1] = n[1];
	s.prevRawRot[2] = s.filteredRot[2] = n[2];
	s.prevRawRot[3] = s.filteredRot[3] = n[3];
	s.linearSpeedHat = 0.0;
	s.angularSpeedHat = 0.0;
	s.posRelease = 1.0;
	s.rotRelease = 1.0;
	s.initialized = true;
}

// Advance the filter by one pose sample. Reported speeds are |vecVelocity| and
// |vecAngularVelocity| from the driver (0 when unavailable). rawDtSeconds is the
// wall-clock gap since the previous sample. On the first call, a long gap, or a
// large jump the filter reseeds and outputs the raw pose for that frame
// (out.reseeded = true). Caller reads s.filteredPos / s.filteredRot afterward.
inline StepResult FilterStep(FilterState& s, const Params& p, const double rawPos[3], const double rawRotIn[4],
                             double reportedLinearSpeed, double reportedAngularSpeed, double rawDtSeconds)
{
	StepResult out;

	double rawRot[4];
	if (!QuatNormalize(rawRotIn, rawRot) || !IsFinite3(rawPos)) {
		// Degenerate input: force a reseed on the next good sample.
		s.initialized = false;
		out.reseeded = true;
		out.reseedReason = ReseedReason::Init;
		return out;
	}

	if (!s.initialized) {
		SeedFilter(s, rawPos, rawRot);
		out.reseeded = true;
		out.reseedReason = ReseedReason::Init;
		out.posRelease = s.posRelease;
		out.rotRelease = s.rotRelease;
		return out;
	}
	if (!(rawDtSeconds > 0.0) || rawDtSeconds > p.resetGapSeconds) {
		SeedFilter(s, rawPos, rawRot);
		out.reseeded = true;
		out.reseedReason = ReseedReason::Gap;
		out.posRelease = s.posRelease;
		out.rotRelease = s.rotRelease;
		return out;
	}

	const double dt = Clamp(rawDtSeconds, 0.001, 0.050);
	QuatSameHemisphere(s.prevRawRot, rawRot, rawRot);

	const double posInnovation = Distance3(rawPos, s.filteredPos);
	const double rotInnovation = QuatAngleRad(rawRot, s.filteredRot);
	if (posInnovation > p.positionJumpM || rotInnovation > p.rotationJumpRad) {
		SeedFilter(s, rawPos, rawRot);
		out.reseeded = true;
		out.reseedReason = ReseedReason::Jump;
		out.posRelease = s.posRelease;
		out.rotRelease = s.rotRelease;
		return out;
	}

	const double rawStepM = Distance3(rawPos, s.prevRawPos);
	const double derivedLinearSpeed = std::min(rawStepM / dt, 15.0);
	const double rawRotStepRad = QuatAngleRad(rawRot, s.prevRawRot);
	const double derivedAngularSpeed = std::min(rawRotStepRad / dt, 80.0);

	const double linInstant = std::max({
	    std::max(0.0, derivedLinearSpeed - p.linDerivedDeadband),
	    std::max(0.0, reportedLinearSpeed - p.linReportedDeadband),
	    std::max(0.0, posInnovation / p.innovationTauSeconds - p.linInnovationDeadband),
	});
	const double angInstant = std::max({
	    std::max(0.0, derivedAngularSpeed - p.angDerivedDeadband),
	    std::max(0.0, reportedAngularSpeed - p.angReportedDeadband),
	    std::max(0.0, rotInnovation / p.innovationTauSeconds - p.angInnovationDeadband),
	});

	const double derivAlpha = AlphaFromCutoffHz(p.derivCutoffHz, dt);
	s.linearSpeedHat += derivAlpha * (linInstant - s.linearSpeedHat);
	s.angularSpeedHat += derivAlpha * (angInstant - s.angularSpeedHat);

	const double posTarget = std::max(SmoothStep(p.linStillSpeed, p.linMovingSpeed, s.linearSpeedHat),
	                                  SmoothStep(p.posErrStillM, p.posErrMovingM, posInnovation));
	const double rotTarget = std::max(SmoothStep(p.angStillSpeed, p.angMovingSpeed, s.angularSpeedHat),
	                                  SmoothStep(p.rotErrStillRad, p.rotErrMovingRad, rotInnovation));

	s.posRelease = UpdateEnvelope(s.posRelease, posTarget, dt, p.gateAttackTauSeconds, p.gateReleaseTauSeconds);
	s.rotRelease = UpdateEnvelope(s.rotRelease, rotTarget, dt, p.gateAttackTauSeconds, p.gateReleaseTauSeconds);

	const double posErrSpeed = std::min(std::max(0.0, posInnovation - p.posErrStillM) / dt, 3.0);
	const double rotErrSpeed = std::min(std::max(0.0, rotInnovation - p.rotErrStillRad) / dt, 20.0);

	const double posSpeedForCutoff = std::max(s.linearSpeedHat, posErrSpeed);
	const double rotSpeedForCutoff = std::max(s.angularSpeedHat, rotErrSpeed);
	double posCutoff = Clamp(p.posMinCutoffHz + p.posBetaHzPerMps * posSpeedForCutoff, p.posMinCutoffHz, p.maxCutoffHz);
	double rotCutoff =
	    Clamp(p.rotMinCutoffHz + p.rotBetaHzPerRadps * rotSpeedForCutoff, p.rotMinCutoffHz, p.maxCutoffHz);
	posCutoff = std::max(posCutoff, Lerp(p.posMinCutoffHz, p.maxCutoffHz, s.posRelease));
	rotCutoff = std::max(rotCutoff, Lerp(p.rotMinCutoffHz, p.maxCutoffHz, s.rotRelease));

	double posAlpha = AlphaFromCutoffHz(posCutoff, dt);
	double rotAlpha = AlphaFromCutoffHz(rotCutoff, dt);
	posAlpha = posAlpha + (1.0 - posAlpha) * s.posRelease;
	rotAlpha = rotAlpha + (1.0 - rotAlpha) * s.rotRelease;

	for (int axis = 0; axis < 3; ++axis) {
		s.filteredPos[axis] += posAlpha * (rawPos[axis] - s.filteredPos[axis]);
	}
	double newRot[4];
	QuatNlerpShortest(s.filteredRot, rawRot, rotAlpha, newRot);
	s.filteredRot[0] = newRot[0];
	s.filteredRot[1] = newRot[1];
	s.filteredRot[2] = newRot[2];
	s.filteredRot[3] = newRot[3];

	s.prevRawPos[0] = rawPos[0];
	s.prevRawPos[1] = rawPos[1];
	s.prevRawPos[2] = rawPos[2];
	s.prevRawRot[0] = rawRot[0];
	s.prevRawRot[1] = rawRot[1];
	s.prevRawRot[2] = rawRot[2];
	s.prevRawRot[3] = rawRot[3];

	out.posAlpha = posAlpha;
	out.rotAlpha = rotAlpha;
	out.posRelease = s.posRelease;
	out.rotRelease = s.rotRelease;
	out.posCutoffHz = posCutoff;
	out.rotCutoffHz = rotCutoff;
	return out;
}

} // namespace prediction::smart_shadow
