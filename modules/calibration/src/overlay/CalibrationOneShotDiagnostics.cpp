#include "CalibrationOneShotDiagnostics.h"

#include "CalibrationMetrics.h"
#include "CalibrationProgress.h"

#include <cmath>
#include <cstdio>

namespace spacecal::oneshot {
namespace {

const char* PhaseName(CalibrationState state)
{
	switch (state) {
		case CalibrationState::Rotation:
			return "rotation";
		case CalibrationState::Translation:
			return "translation";
		default:
			return "inactive";
	}
}

} // namespace

void MaybeLogReadiness(const CalibrationContext& ctx, int sampleProgress, int sampleTarget, double rotationDiversity,
                       double translationDiversity, double now)
{
	static CalibrationState s_lastPhase = CalibrationState::None;
	static int s_lastReadyBucket = -1;
	static double s_nextGateLogTime = -1e9;

	const bool active = ctx.state == CalibrationState::Rotation || ctx.state == CalibrationState::Translation;
	if (!active) {
		s_lastPhase = CalibrationState::None;
		s_lastReadyBucket = -1;
		s_nextGateLogTime = -1e9;
		return;
	}

	const double ready = spacecal::calibration_progress::OneShotReadyScore(ctx.state, sampleProgress, sampleTarget,
	                                                                       rotationDiversity, translationDiversity);
	const double clampedReady = spacecal::calibration_progress::Clamp01(ready);
	const int readyBucket = (int)std::floor(clampedReady * 10.0);
	const bool sampleFilled = sampleTarget > 0 && sampleProgress >= sampleTarget;
	const bool rotationGatePending = ctx.state == CalibrationState::Rotation && sampleFilled &&
	                                 rotationDiversity < spacecal::calibration_progress::kOneShotRotationReadyDiversity;
	const bool translationGatePending =
	    ctx.state == CalibrationState::Translation && sampleFilled &&
	    translationDiversity < spacecal::calibration_progress::kOneShotTranslationReadyDiversity;
	const bool gatePending = rotationGatePending || translationGatePending;

	const bool phaseChanged = ctx.state != s_lastPhase;
	const bool bucketChanged = readyBucket != s_lastReadyBucket;
	const bool periodicGateLog = gatePending && now >= s_nextGateLogTime;
	if (!phaseChanged && !bucketChanged && !periodicGateLog) {
		return;
	}

	const char* gate = gatePending ? "waiting_motion" : (sampleFilled ? "ready_or_solving" : "collecting_samples");
	char buf[320];
	snprintf(buf, sizeof buf,
	         "[oneshot] readiness phase=%s ready_pct=%d samples=%d/%d rot=%.2f/%.2f tr=%.2f/%.2f gate=%s",
	         PhaseName(ctx.state), (int)(clampedReady * 100.0), sampleProgress, sampleTarget, rotationDiversity,
	         spacecal::calibration_progress::kOneShotRotationReadyDiversity, translationDiversity,
	         spacecal::calibration_progress::kOneShotTranslationReadyDiversity, gate);
	Metrics::WriteLogAnnotation(buf);

	s_lastPhase = ctx.state;
	s_lastReadyBucket = readyBucket;
	if (gatePending) {
		s_nextGateLogTime = now + 2.0;
	}
}

} // namespace spacecal::oneshot
