#include "ServerTrackedDeviceProvider.h"

#include "Logging.h"
#include "Win32Paths.h"

#include <algorithm>
#include <cmath>
#include <string>

#if WKOPENVR_BUILD_IS_DEV

// Dev-only A/B telemetry. The LIVE pose path applies the one-euro filter
// (ApplySmartSmoothing); this runs a CANDIDATE one-euro alongside it and logs,
// every ~5 s, how the candidate compares to the live filter and to raw. The two
// readouts that matter for deciding a tuning:
//   * rest_jit -- per-frame output step while the device is at rest (jitter).
//                 Lower = smoother / more glued (but never frozen).
//   * move_lag -- output-vs-raw distance while moving (latency). Lower = crisper.
// A candidate that lowers rest_jit but raises move_lag is "smoother but laggier";
// the point is to see both numbers for the same session and pick.

namespace {

namespace ss = prediction::smart_shadow;

constexpr double kLogIntervalSeconds = 5.0;
constexpr double kCandidateCheckSeconds = 5.0;
constexpr double kMaxReasonableLinearSpeed = 15.0;  // m/s
constexpr double kMaxReasonableAngularSpeed = 80.0; // rad/s
// Regime gates on the raw signal (reported IMU velocity preferred), so the
// rest/move split does not depend on either filter's own behaviour.
constexpr double kRestLinearSpeed = 0.05; // m/s -- below this = "at rest"
constexpr double kMoveLinearSpeed = 0.30; // m/s -- above this = "moving"
constexpr double kRad2Deg = 180.0 / ss::kPi;

bool Finite3(const double v[3])
{
	return std::isfinite(v[0]) && std::isfinite(v[1]) && std::isfinite(v[2]);
}

void Copy3(double dst[3], const double src[3])
{
	dst[0] = src[0];
	dst[1] = src[1];
	dst[2] = src[2];
}

void Copy4(double dst[4], const double src[4])
{
	dst[0] = src[0];
	dst[1] = src[1];
	dst[2] = src[2];
	dst[3] = src[3];
}

void QuatToArray(double out[4], const vr::HmdQuaternion_t& q)
{
	out[0] = q.w;
	out[1] = q.x;
	out[2] = q.y;
	out[3] = q.z;
}

double Rms(double sumSq, uint64_t count)
{
	return count > 0 ? std::sqrt(sumSq / static_cast<double>(count)) : 0.0;
}

// Candidate selection by flag-file existence in the WKOpenVR appdata root. Drop
// an empty file named smart_shadow.match / .responsive / .strong to pick the
// variant the shadow A/B-tests; default Strong. Read at the log cadence only,
// never per sample.
ss::CandidateKind ReadCandidateFromFlags()
{
	std::wstring root = openvr_pair::common::WkOpenVrRootPath(false);
	if (root.empty()) return ss::CandidateKind::Strong;
	if (root.back() != L'\\' && root.back() != L'/') root.push_back(L'\\');
	auto present = [&](const wchar_t* name) -> bool {
		const std::wstring path = root + name;
		const DWORD attr = GetFileAttributesW(path.c_str());
		return attr != INVALID_FILE_ATTRIBUTES && !(attr & FILE_ATTRIBUTE_DIRECTORY);
	};
	if (present(L"smart_shadow.match")) return ss::CandidateKind::Match;
	if (present(L"smart_shadow.responsive")) return ss::CandidateKind::Responsive;
	if (present(L"smart_shadow.strong")) return ss::CandidateKind::Strong;
	return ss::CandidateKind::Strong;
}

} // namespace

void ServerTrackedDeviceProvider::UpdateSmartSmoothingShadow(uint32_t openVRID, DeviceTransform& device,
                                                             const vr::DriverPose_t& rawPose,
                                                             const vr::DriverPose_t& livePose) const
{
	auto& state = device.smartShadow;
	const uint8_t smoothness = device.predictionSmoothness;
	if (smoothness == 0) {
		state.filter.initialized = false;
		state.previousOutputsInitialized = false;
		return;
	}
	if (qpcFreq.QuadPart <= 0) return;

	LARGE_INTEGER now{};
	QueryPerformanceCounter(&now);
	if (state.lastLog.QuadPart == 0) state.lastLog = now;

	auto& stats = state.stats;

	auto maybeLogAndReset = [&]() {
		const double elapsed = state.lastLog.QuadPart > 0
		                           ? (now.QuadPart - state.lastLog.QuadPart) / static_cast<double>(qpcFreq.QuadPart)
		                           : kLogIntervalSeconds;
		if (elapsed < kLogIntervalSeconds || stats.samples == 0) return;

		const auto& s = stats;
		const double inv = 1.0 / static_cast<double>(s.samples);
		LOG("[smart-shadow] id=%u smooth=%u cand=%s live_smart=%d samples=%llu rest=%llu move=%llu"
		    " rest_jit_mm raw=%.3f live=%.3f cand=%.3f rest_jit_deg raw=%.4f live=%.4f cand=%.4f"
		    " move_lag_mm live=%.2f cand=%.2f move_lag_max_mm live=%.2f cand=%.2f"
		    " move_lag_deg live=%.3f cand=%.3f"
		    " cand_release_pos=%.3f cand_cutoff_pos=%.2f cand_release_rot=%.3f cand_cutoff_rot=%.2f"
		    " div_mm cand_v_live=%.2f cand_v_live_max=%.2f cand_v_raw=%.2f"
		    " div_deg cand_v_live=%.3f cand_v_raw=%.3f"
		    " reseeds=%llu gap=%llu jump=%llu nonfinite=%llu bad_vel=%llu bad_ang=%llu",
		    openVRID, static_cast<unsigned>(smoothness), ss::CandidateKindName(state.candidate),
		    device.smartEnabled ? 1 : 0, static_cast<unsigned long long>(s.samples),
		    static_cast<unsigned long long>(s.restSamples), static_cast<unsigned long long>(s.moveSamples),
		    Rms(s.sumSqRestRawStepM, s.restSamples) * 1000.0, Rms(s.sumSqRestLiveStepM, s.restSamples) * 1000.0,
		    Rms(s.sumSqRestCandStepM, s.restSamples) * 1000.0, Rms(s.sumSqRestRawRotStepRad, s.restSamples) * kRad2Deg,
		    Rms(s.sumSqRestLiveRotStepRad, s.restSamples) * kRad2Deg,
		    Rms(s.sumSqRestCandRotStepRad, s.restSamples) * kRad2Deg, Rms(s.sumSqMoveLiveLagM, s.moveSamples) * 1000.0,
		    Rms(s.sumSqMoveCandLagM, s.moveSamples) * 1000.0, s.maxMoveLiveLagM * 1000.0, s.maxMoveCandLagM * 1000.0,
		    Rms(s.sumSqMoveLiveLagRad, s.moveSamples) * kRad2Deg, Rms(s.sumSqMoveCandLagRad, s.moveSamples) * kRad2Deg,
		    s.sumPosRelease * inv, s.sumPosCutoffHz * inv, s.sumRotRelease * inv, s.sumRotCutoffHz * inv,
		    Rms(s.sumSqCandLiveErrM, s.samples) * 1000.0, s.maxCandLiveErrM * 1000.0,
		    Rms(s.sumSqCandRawErrM, s.samples) * 1000.0, Rms(s.sumSqCandLiveErrRad, s.samples) * kRad2Deg,
		    Rms(s.sumSqCandRawErrRad, s.samples) * kRad2Deg, static_cast<unsigned long long>(s.reseeds),
		    static_cast<unsigned long long>(s.gapReseeds), static_cast<unsigned long long>(s.jumpReseeds),
		    static_cast<unsigned long long>(s.nonFinitePoseResets),
		    static_cast<unsigned long long>(s.invalidReportedLinear),
		    static_cast<unsigned long long>(s.invalidReportedAngular));

		stats = SmartSmoothingShadowStats{};
		state.lastLog = now;
	};

	// Refresh the candidate selection at the log cadence (not per sample).
	const double sinceCand =
	    state.lastCandidateCheck.QuadPart > 0
	        ? (now.QuadPart - state.lastCandidateCheck.QuadPart) / static_cast<double>(qpcFreq.QuadPart)
	        : kCandidateCheckSeconds;
	if (sinceCand >= kCandidateCheckSeconds) {
		state.candidate = ReadCandidateFromFlags();
		state.lastCandidateCheck = now;
	}
	const ss::Params candParams = ss::BuildCandidateParams(smoothness, state.candidate);

	// Raw pose -> doubles, with a finite guard so FilterStep stays off its
	// degenerate path and the comparison never reads garbage.
	double rawPos[3] = {rawPose.vecPosition[0], rawPose.vecPosition[1], rawPose.vecPosition[2]};
	double rawRot[4];
	QuatToArray(rawRot, rawPose.qRotation);
	double rawRotN[4];
	if (!Finite3(rawPos) || !ss::QuatNormalize(rawRot, rawRotN)) {
		++stats.nonFinitePoseResets;
		state.filter.initialized = false;
		state.previousOutputsInitialized = false;
		state.lastSample = now;
		maybeLogAndReset();
		return;
	}

	const double dt = state.lastSample.QuadPart > 0
	                      ? (now.QuadPart - state.lastSample.QuadPart) / static_cast<double>(qpcFreq.QuadPart)
	                      : -1.0;
	state.lastSample = now;

	double reportedLinear = 0.0;
	const bool repLinValid = Finite3(rawPose.vecVelocity);
	if (repLinValid) {
		reportedLinear = std::min(ss::Length3(rawPose.vecVelocity), kMaxReasonableLinearSpeed);
	}
	else {
		++stats.invalidReportedLinear;
	}
	double reportedAngular = 0.0;
	if (Finite3(rawPose.vecAngularVelocity)) {
		reportedAngular = std::min(ss::Length3(rawPose.vecAngularVelocity), kMaxReasonableAngularSpeed);
	}
	else {
		++stats.invalidReportedAngular;
	}

	const ss::StepResult r =
	    ss::FilterStep(state.filter, candParams, rawPos, rawRotN, reportedLinear, reportedAngular, dt);

	// Live (applied) pose -> doubles.
	double livePos[3] = {livePose.vecPosition[0], livePose.vecPosition[1], livePose.vecPosition[2]};
	double liveRot[4];
	QuatToArray(liveRot, livePose.qRotation);
	double liveRotN[4];
	if (!ss::QuatNormalize(liveRot, liveRotN)) Copy4(liveRotN, rawRotN);

	const double* candPos = state.filter.filteredPos;
	const double* candRot = state.filter.filteredRot;

	if (r.reseeded) {
		++stats.reseeds;
		if (r.reseedReason == ss::ReseedReason::Gap)
			++stats.gapReseeds;
		else if (r.reseedReason == ss::ReseedReason::Jump)
			++stats.jumpReseeds;
		Copy3(state.prevRawPos, rawPos);
		Copy3(state.prevLivePos, livePos);
		Copy3(state.prevCandPos, candPos);
		Copy4(state.prevRawRot, rawRotN);
		Copy4(state.prevLiveRot, liveRotN);
		Copy4(state.prevCandRot, candRot);
		state.previousOutputsInitialized = true;
		maybeLogAndReset();
		return;
	}

	++stats.samples;
	stats.sumPosRelease += r.posRelease;
	stats.maxPosRelease = std::max(stats.maxPosRelease, r.posRelease);
	stats.sumRotRelease += r.rotRelease;
	stats.maxRotRelease = std::max(stats.maxRotRelease, r.rotRelease);
	stats.sumPosCutoffHz += r.posCutoffHz;
	stats.maxPosCutoffHz = std::max(stats.maxPosCutoffHz, r.posCutoffHz);
	stats.sumRotCutoffHz += r.rotCutoffHz;
	stats.maxRotCutoffHz = std::max(stats.maxRotCutoffHz, r.rotCutoffHz);

	// Divergence of the candidate from the live filter and from raw (every sample).
	const double candLiveErr = ss::Distance3(candPos, livePos);
	const double candRawErr = ss::Distance3(candPos, rawPos);
	const double candLiveErrRot = ss::QuatAngleRad(candRot, liveRotN);
	const double candRawErrRot = ss::QuatAngleRad(candRot, rawRotN);
	stats.sumSqCandLiveErrM += candLiveErr * candLiveErr;
	stats.maxCandLiveErrM = std::max(stats.maxCandLiveErrM, candLiveErr);
	stats.sumSqCandRawErrM += candRawErr * candRawErr;
	stats.sumSqCandLiveErrRad += candLiveErrRot * candLiveErrRot;
	stats.sumSqCandRawErrRad += candRawErrRot * candRawErrRot;

	if (state.previousOutputsInitialized) {
		const double rawStep = ss::Distance3(rawPos, state.prevRawPos);
		const double derivedLin = dt > 0.0 ? std::min(rawStep / dt, kMaxReasonableLinearSpeed) : 0.0;
		const double rawSpeed = repLinValid ? reportedLinear : derivedLin;

		if (rawSpeed < kRestLinearSpeed) {
			++stats.restSamples;
			stats.sumSqRestRawStepM += rawStep * rawStep;
			const double liveStep = ss::Distance3(livePos, state.prevLivePos);
			const double candStep = ss::Distance3(candPos, state.prevCandPos);
			stats.sumSqRestLiveStepM += liveStep * liveStep;
			stats.sumSqRestCandStepM += candStep * candStep;
			const double rawRotStep = ss::QuatAngleRad(rawRotN, state.prevRawRot);
			const double liveRotStep = ss::QuatAngleRad(liveRotN, state.prevLiveRot);
			const double candRotStep = ss::QuatAngleRad(candRot, state.prevCandRot);
			stats.sumSqRestRawRotStepRad += rawRotStep * rawRotStep;
			stats.sumSqRestLiveRotStepRad += liveRotStep * liveRotStep;
			stats.sumSqRestCandRotStepRad += candRotStep * candRotStep;
		}
		else if (rawSpeed > kMoveLinearSpeed) {
			++stats.moveSamples;
			const double liveLag = ss::Distance3(livePos, rawPos);
			const double candLag = candRawErr; // candidate output vs raw = candidate lag
			stats.sumSqMoveLiveLagM += liveLag * liveLag;
			stats.maxMoveLiveLagM = std::max(stats.maxMoveLiveLagM, liveLag);
			stats.sumSqMoveCandLagM += candLag * candLag;
			stats.maxMoveCandLagM = std::max(stats.maxMoveCandLagM, candLag);
			const double liveLagRot = ss::QuatAngleRad(liveRotN, rawRotN);
			stats.sumSqMoveLiveLagRad += liveLagRot * liveLagRot;
			stats.sumSqMoveCandLagRad += candRawErrRot * candRawErrRot; // candidate vs raw rot
		}
	}

	Copy3(state.prevRawPos, rawPos);
	Copy3(state.prevLivePos, livePos);
	Copy3(state.prevCandPos, candPos);
	Copy4(state.prevRawRot, rawRotN);
	Copy4(state.prevLiveRot, liveRotN);
	Copy4(state.prevCandRot, candRot);
	state.previousOutputsInitialized = true;

	maybeLogAndReset();
}

#endif // WKOPENVR_BUILD_IS_DEV
