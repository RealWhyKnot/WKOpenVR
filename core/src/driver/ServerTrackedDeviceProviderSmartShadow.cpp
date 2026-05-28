#include "ServerTrackedDeviceProvider.h"

#include "Logging.h"

#include <algorithm>
#include <cmath>

#if WKOPENVR_BUILD_IS_DEV

namespace {

constexpr double kLogIntervalSeconds = 5.0;
constexpr double kMaxReasonableLinearSpeed = 15.0;  // m/s
constexpr double kMaxReasonableAngularSpeed = 80.0; // rad/s

bool Finite(double v)
{
	return std::isfinite(v);
}

bool Finite3(const double v[3])
{
	return Finite(v[0]) && Finite(v[1]) && Finite(v[2]);
}

bool FiniteQuat(const vr::HmdQuaternion_t& q)
{
	return Finite(q.w) && Finite(q.x) && Finite(q.y) && Finite(q.z);
}

double Length3(const double v[3])
{
	return std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

double Distance3(const double a[3], const double b[3])
{
	const double dx = a[0] - b[0];
	const double dy = a[1] - b[1];
	const double dz = a[2] - b[2];
	return std::sqrt(dx * dx + dy * dy + dz * dz);
}

void Copy3(double dst[3], const double src[3])
{
	dst[0] = src[0];
	dst[1] = src[1];
	dst[2] = src[2];
}

double DotQuat(const vr::HmdQuaternion_t& a, const vr::HmdQuaternion_t& b)
{
	return a.w * b.w + a.x * b.x + a.y * b.y + a.z * b.z;
}

vr::HmdQuaternion_t NegateQuat(vr::HmdQuaternion_t q)
{
	q.w = -q.w;
	q.x = -q.x;
	q.y = -q.y;
	q.z = -q.z;
	return q;
}

vr::HmdQuaternion_t NormalizeQuat(vr::HmdQuaternion_t q, bool* ok = nullptr)
{
	if (ok) *ok = false;
	if (!FiniteQuat(q)) return {1.0, 0.0, 0.0, 0.0};

	const double n2 = DotQuat(q, q);
	if (!(n2 > 1e-18) || !std::isfinite(n2)) return {1.0, 0.0, 0.0, 0.0};

	const double inv = 1.0 / std::sqrt(n2);
	q.w *= inv;
	q.x *= inv;
	q.y *= inv;
	q.z *= inv;
	if (ok) *ok = true;
	return q;
}

vr::HmdQuaternion_t SameHemisphere(const vr::HmdQuaternion_t& reference,
								   vr::HmdQuaternion_t q)
{
	return DotQuat(reference, q) < 0.0 ? NegateQuat(q) : q;
}

double QuaternionAngleRad(vr::HmdQuaternion_t a, vr::HmdQuaternion_t b)
{
	bool okA = false;
	bool okB = false;
	a = NormalizeQuat(a, &okA);
	b = NormalizeQuat(b, &okB);
	if (!okA || !okB) return 0.0;

	const double d = prediction::smart_shadow::Clamp(std::abs(DotQuat(a, b)), 0.0, 1.0);
	return 2.0 * std::acos(d);
}

vr::HmdQuaternion_t NlerpShortest(vr::HmdQuaternion_t from,
								  vr::HmdQuaternion_t to,
								  double alpha)
{
	bool okFrom = false;
	bool okTo = false;
	from = NormalizeQuat(from, &okFrom);
	to = NormalizeQuat(to, &okTo);
	if (!okFrom) return to;
	if (!okTo) return from;

	to = SameHemisphere(from, to);
	alpha = prediction::smart_shadow::Saturate(alpha);
	vr::HmdQuaternion_t out{
		from.w + alpha * (to.w - from.w),
		from.x + alpha * (to.x - from.x),
		from.y + alpha * (to.y - from.y),
		from.z + alpha * (to.z - from.z),
	};
	return NormalizeQuat(out);
}

double Rms(double sumSq, uint64_t count)
{
	return count > 0 ? std::sqrt(sumSq / static_cast<double>(count)) : 0.0;
}

} // namespace

void ServerTrackedDeviceProvider::UpdateSmartSmoothingShadow(
	uint32_t openVRID,
	DeviceTransform& device,
	const vr::DriverPose_t& rawPose,
	const vr::DriverPose_t& livePose) const
{
	auto& state = device.smartShadow;
	const uint8_t smoothness = device.predictionSmoothness;
	if (smoothness == 0) {
		state.initialized = false;
		state.previousOutputsInitialized = false;
		return;
	}

	LARGE_INTEGER now{};
	QueryPerformanceCounter(&now);

	if (qpcFreq.QuadPart <= 0) return;

	auto seedShadowState = [&]() {
		Copy3(state.prevRawPos, rawPose.vecPosition);
		Copy3(state.prevLivePos, rawPose.vecPosition);
		Copy3(state.prevShadowPos, rawPose.vecPosition);
		Copy3(state.filteredPos, rawPose.vecPosition);
		state.prevRawRot = NormalizeQuat(rawPose.qRotation);
		state.prevLiveRot = state.prevRawRot;
		state.prevShadowRot = state.prevRawRot;
		state.filteredRot = state.prevRawRot;
		state.linearSpeedHat = 0.0;
		state.angularSpeedHat = 0.0;
		state.posRelease = 1.0;
		state.rotRelease = 1.0;
		state.lastSample = now;
		state.initialized = true;
		state.previousOutputsInitialized = false;
		if (state.lastLog.QuadPart == 0) {
			state.lastLog = now;
		}
	};

	auto maybeLogAndResetStats = [&]() {
		const double elapsed = state.lastLog.QuadPart > 0
			? (now.QuadPart - state.lastLog.QuadPart) / static_cast<double>(qpcFreq.QuadPart)
			: kLogIntervalSeconds;
		if (elapsed < kLogIntervalSeconds || state.stats.samples == 0) return;

		const auto& s = state.stats;
		const double invSamples = 1.0 / static_cast<double>(s.samples);
		LOG("[smart-shadow] id=%u smooth=%u live_smart=%d samples=%llu rest=%llu"
			" release_pos_avg=%.3f release_pos_max=%.3f release_rot_avg=%.3f release_rot_max=%.3f"
			" cutoff_pos_avg=%.2f cutoff_pos_max=%.2f cutoff_rot_avg=%.2f cutoff_rot_max=%.2f"
			" rest_step_mm raw=%.3f live=%.3f shadow=%.3f rest_step_deg raw=%.3f live=%.3f shadow=%.3f"
			" err_mm shadow_raw_rms=%.3f shadow_raw_max=%.3f live_shadow_rms=%.3f live_shadow_max=%.3f"
			" err_deg shadow_raw_rms=%.3f shadow_raw_max=%.3f live_shadow_rms=%.3f live_shadow_max=%.3f"
			" reseeds=%llu gap=%llu pos_jump=%llu rot_jump=%llu nonfinite_pose=%llu bad_vel=%llu bad_ang_vel=%llu",
			openVRID,
			static_cast<unsigned>(smoothness),
			device.smartEnabled ? 1 : 0,
			static_cast<unsigned long long>(s.samples),
			static_cast<unsigned long long>(s.restSamples),
			s.sumPosRelease * invSamples,
			s.maxPosRelease,
			s.sumRotRelease * invSamples,
			s.maxRotRelease,
			s.sumPosCutoffHz * invSamples,
			s.maxPosCutoffHz,
			s.sumRotCutoffHz * invSamples,
			s.maxRotCutoffHz,
			Rms(s.sumSqRestRawStepM, s.restSamples) * 1000.0,
			Rms(s.sumSqRestLiveStepM, s.restSamples) * 1000.0,
			Rms(s.sumSqRestShadowStepM, s.restSamples) * 1000.0,
			Rms(s.sumSqRestRawRotStepRad, s.restSamples) * 180.0 / prediction::smart_shadow::kPi,
			Rms(s.sumSqRestLiveRotStepRad, s.restSamples) * 180.0 / prediction::smart_shadow::kPi,
			Rms(s.sumSqRestShadowRotStepRad, s.restSamples) * 180.0 / prediction::smart_shadow::kPi,
			Rms(s.sumSqShadowRawErrM, s.samples) * 1000.0,
			s.maxShadowRawErrM * 1000.0,
			Rms(s.sumSqLiveShadowErrM, s.samples) * 1000.0,
			s.maxLiveShadowErrM * 1000.0,
			Rms(s.sumSqShadowRawRotErrRad, s.samples) * 180.0 / prediction::smart_shadow::kPi,
			s.maxShadowRawRotErrRad * 180.0 / prediction::smart_shadow::kPi,
			Rms(s.sumSqLiveShadowRotErrRad, s.samples) * 180.0 / prediction::smart_shadow::kPi,
			s.maxLiveShadowRotErrRad * 180.0 / prediction::smart_shadow::kPi,
			static_cast<unsigned long long>(s.reseeds),
			static_cast<unsigned long long>(s.gapReseeds),
			static_cast<unsigned long long>(s.positionJumpReseeds),
			static_cast<unsigned long long>(s.rotationJumpReseeds),
			static_cast<unsigned long long>(s.nonFinitePoseResets),
			static_cast<unsigned long long>(s.invalidReportedLinear),
			static_cast<unsigned long long>(s.invalidReportedAngular));

		state.stats = SmartSmoothingShadowStats{};
		state.lastLog = now;
	};

	bool rawRotOk = false;
	vr::HmdQuaternion_t rawRot = NormalizeQuat(rawPose.qRotation, &rawRotOk);
	if (!Finite3(rawPose.vecPosition) || !rawRotOk) {
		++state.stats.nonFinitePoseResets;
		state.initialized = false;
		state.previousOutputsInitialized = false;
		maybeLogAndResetStats();
		return;
	}

	if (!state.initialized) {
		seedShadowState();
		return;
	}

	double rawDt = (now.QuadPart - state.lastSample.QuadPart) /
		static_cast<double>(qpcFreq.QuadPart);
	if (!(rawDt > 0.0) || rawDt > device.smartShadowParams.resetGapSeconds) {
		++state.stats.reseeds;
		++state.stats.gapReseeds;
		seedShadowState();
		return;
	}

	const double dt = prediction::smart_shadow::Clamp(rawDt, 0.001, 0.050);
	rawRot = SameHemisphere(state.prevRawRot, rawRot);

	const double posInnovation = Distance3(rawPose.vecPosition, state.filteredPos);
	const double rotInnovation = QuaternionAngleRad(rawRot, state.filteredRot);
	if (posInnovation > device.smartShadowParams.positionJumpM) {
		++state.stats.reseeds;
		++state.stats.positionJumpReseeds;
		seedShadowState();
		return;
	}
	if (rotInnovation > device.smartShadowParams.rotationJumpRad) {
		++state.stats.reseeds;
		++state.stats.rotationJumpReseeds;
		seedShadowState();
		return;
	}

	const double rawStepM = Distance3(rawPose.vecPosition, state.prevRawPos);
	const double derivedLinearSpeed = std::min(rawStepM / dt, kMaxReasonableLinearSpeed);
	const double rawRotStepRad = QuaternionAngleRad(rawRot, state.prevRawRot);
	const double derivedAngularSpeed = std::min(rawRotStepRad / dt, kMaxReasonableAngularSpeed);

	double reportedLinearSpeed = 0.0;
	if (Finite3(rawPose.vecVelocity)) {
		reportedLinearSpeed = std::min(Length3(rawPose.vecVelocity), kMaxReasonableLinearSpeed);
	} else {
		++state.stats.invalidReportedLinear;
	}

	double reportedAngularSpeed = 0.0;
	if (Finite3(rawPose.vecAngularVelocity)) {
		reportedAngularSpeed = std::min(Length3(rawPose.vecAngularVelocity), kMaxReasonableAngularSpeed);
	} else {
		++state.stats.invalidReportedAngular;
	}

	const auto& p = device.smartShadowParams;
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

	const double derivAlpha = prediction::smart_shadow::AlphaFromCutoffHz(p.derivCutoffHz, dt);
	state.linearSpeedHat += derivAlpha * (linInstant - state.linearSpeedHat);
	state.angularSpeedHat += derivAlpha * (angInstant - state.angularSpeedHat);

	const double posTarget = std::max(
		prediction::smart_shadow::SmoothStep(p.linStillSpeed, p.linMovingSpeed, state.linearSpeedHat),
		prediction::smart_shadow::SmoothStep(p.posErrStillM, p.posErrMovingM, posInnovation));
	const double rotTarget = std::max(
		prediction::smart_shadow::SmoothStep(p.angStillSpeed, p.angMovingSpeed, state.angularSpeedHat),
		prediction::smart_shadow::SmoothStep(p.rotErrStillRad, p.rotErrMovingRad, rotInnovation));

	state.posRelease = prediction::smart_shadow::UpdateEnvelope(
		state.posRelease, posTarget, dt, p.gateAttackTauSeconds, p.gateReleaseTauSeconds);
	state.rotRelease = prediction::smart_shadow::UpdateEnvelope(
		state.rotRelease, rotTarget, dt, p.gateAttackTauSeconds, p.gateReleaseTauSeconds);

	const double posErrSpeed = std::min(
		std::max(0.0, posInnovation - p.posErrStillM) / dt,
		3.0);
	const double rotErrSpeed = std::min(
		std::max(0.0, rotInnovation - p.rotErrStillRad) / dt,
		20.0);

	const double posSpeedForCutoff = std::max(state.linearSpeedHat, posErrSpeed);
	const double rotSpeedForCutoff = std::max(state.angularSpeedHat, rotErrSpeed);
	double posCutoff = prediction::smart_shadow::Clamp(
		p.posMinCutoffHz + p.posBetaHzPerMps * posSpeedForCutoff,
		p.posMinCutoffHz,
		p.maxCutoffHz);
	double rotCutoff = prediction::smart_shadow::Clamp(
		p.rotMinCutoffHz + p.rotBetaHzPerRadps * rotSpeedForCutoff,
		p.rotMinCutoffHz,
		p.maxCutoffHz);
	posCutoff = std::max(posCutoff, prediction::smart_shadow::Lerp(
		p.posMinCutoffHz, p.maxCutoffHz, state.posRelease));
	rotCutoff = std::max(rotCutoff, prediction::smart_shadow::Lerp(
		p.rotMinCutoffHz, p.maxCutoffHz, state.rotRelease));

	double posAlpha = prediction::smart_shadow::AlphaFromCutoffHz(posCutoff, dt);
	double rotAlpha = prediction::smart_shadow::AlphaFromCutoffHz(rotCutoff, dt);
	posAlpha = posAlpha + (1.0 - posAlpha) * state.posRelease;
	rotAlpha = rotAlpha + (1.0 - rotAlpha) * state.rotRelease;

	for (int axis = 0; axis < 3; ++axis) {
		state.filteredPos[axis] += posAlpha * (rawPose.vecPosition[axis] - state.filteredPos[axis]);
	}
	state.filteredRot = NlerpShortest(state.filteredRot, rawRot, rotAlpha);

	bool liveRotOk = false;
	vr::HmdQuaternion_t liveRot = NormalizeQuat(livePose.qRotation, &liveRotOk);
	if (!liveRotOk) liveRot = rawRot;

	const double shadowRawErrM = Distance3(rawPose.vecPosition, state.filteredPos);
	const double liveShadowErrM = Finite3(livePose.vecPosition)
		? Distance3(livePose.vecPosition, state.filteredPos)
		: 0.0;
	const double shadowRawRotErrRad = QuaternionAngleRad(rawRot, state.filteredRot);
	const double liveShadowRotErrRad = liveRotOk
		? QuaternionAngleRad(liveRot, state.filteredRot)
		: 0.0;

	auto& stats = state.stats;
	++stats.samples;
	stats.sumPosRelease += state.posRelease;
	stats.maxPosRelease = std::max(stats.maxPosRelease, state.posRelease);
	stats.sumRotRelease += state.rotRelease;
	stats.maxRotRelease = std::max(stats.maxRotRelease, state.rotRelease);
	stats.sumPosCutoffHz += posCutoff;
	stats.maxPosCutoffHz = std::max(stats.maxPosCutoffHz, posCutoff);
	stats.sumRotCutoffHz += rotCutoff;
	stats.maxRotCutoffHz = std::max(stats.maxRotCutoffHz, rotCutoff);

	stats.sumSqShadowRawErrM += shadowRawErrM * shadowRawErrM;
	stats.maxShadowRawErrM = std::max(stats.maxShadowRawErrM, shadowRawErrM);
	stats.sumSqLiveShadowErrM += liveShadowErrM * liveShadowErrM;
	stats.maxLiveShadowErrM = std::max(stats.maxLiveShadowErrM, liveShadowErrM);
	stats.sumSqShadowRawRotErrRad += shadowRawRotErrRad * shadowRawRotErrRad;
	stats.maxShadowRawRotErrRad = std::max(stats.maxShadowRawRotErrRad, shadowRawRotErrRad);
	stats.sumSqLiveShadowRotErrRad += liveShadowRotErrRad * liveShadowRotErrRad;
	stats.maxLiveShadowRotErrRad = std::max(stats.maxLiveShadowRotErrRad, liveShadowRotErrRad);

	if (state.previousOutputsInitialized &&
		state.posRelease < 0.20 &&
		state.rotRelease < 0.20) {
		const double liveStepM = Finite3(livePose.vecPosition)
			? Distance3(livePose.vecPosition, state.prevLivePos)
			: 0.0;
		const double shadowStepM = Distance3(state.filteredPos, state.prevShadowPos);
		const double liveRotStepRad = liveRotOk
			? QuaternionAngleRad(liveRot, state.prevLiveRot)
			: 0.0;
		const double shadowRotStepRad = QuaternionAngleRad(state.filteredRot, state.prevShadowRot);

		++stats.restSamples;
		stats.sumSqRestRawStepM += rawStepM * rawStepM;
		stats.sumSqRestLiveStepM += liveStepM * liveStepM;
		stats.sumSqRestShadowStepM += shadowStepM * shadowStepM;
		stats.sumSqRestRawRotStepRad += rawRotStepRad * rawRotStepRad;
		stats.sumSqRestLiveRotStepRad += liveRotStepRad * liveRotStepRad;
		stats.sumSqRestShadowRotStepRad += shadowRotStepRad * shadowRotStepRad;
	}

	Copy3(state.prevRawPos, rawPose.vecPosition);
	Copy3(state.prevShadowPos, state.filteredPos);
	if (Finite3(livePose.vecPosition)) {
		Copy3(state.prevLivePos, livePose.vecPosition);
	}
	state.prevRawRot = rawRot;
	state.prevShadowRot = state.filteredRot;
	state.prevLiveRot = liveRot;
	state.previousOutputsInitialized = true;
	state.lastSample = now;

	maybeLogAndResetStats();
}

#endif // WKOPENVR_BUILD_IS_DEV
