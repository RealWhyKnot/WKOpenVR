#pragma once

// Sustained quality-rejection breaker for continuous calibration.
//
// The quality verdict ([cal-quality-verdict], CalibrationCalc.cpp) scores
// every periodic solve snapshot, but the accept path does not consult it:
// candidates land as long as the solver's own gates pass. That is correct
// for ordinary noise -- healthy sessions reject 54-69% of verdicts in short
// streaks -- but when the input geometry is structurally bad (target tracker
// disconnected with a head-mount proxy substituting, reference stale), the
// verdict rejects every snapshot for hours while the solver keeps producing
// and applying jittery candidates. A 2026-07-16 session logged 1,523
// consecutive rejections over 2.4 h with ~6.6 device-transform sends per
// second the whole way.
//
// The breaker watches the verdict stream and, after a long enough
// uninterrupted rejection streak, holds the OUTPUT side: no new candidate is
// accepted, persisted, or applied until a verdict passes again. The solver,
// sampling, recovery layer, and the periodic re-apply of the held offset all
// keep running -- the breaker only stops the churn, it never blanks the
// calibration. Release is a single accepted verdict.
//
// Thresholds come from measured sessions, not intuition. Healthy baselines
// (2026-07-15, 2026-07-16 early) show a maximum rejection streak of 71-80
// verdicts; the broken session ran to 1,523. kEngageConsecutiveRejects sits
// roughly 2x above the healthy maximum, and the sustain floor keeps a burst
// of fast verdicts from engaging early. A spurious engage is benign: output
// holds at the last accepted value until the next passing verdict releases
// it -- in a healthy session that is seconds away.

namespace spacecal::quality_breaker {

constexpr int kEngageConsecutiveRejects = 150;
constexpr double kEngageMinSustainSec = 300.0;

struct State
{
	int consecutiveRejects = 0;
	double firstRejectSec = 0.0; // streak start (tick time); 0 = no streak
	bool engaged = false;
};

// Advance the breaker with one verdict observation. Any accepted verdict
// releases and fully resets the streak; a rejection extends it and engages
// once both the count and the sustain floor are met.
constexpr State Next(State s, bool wouldAccept, double nowSec)
{
	if (wouldAccept) {
		return {};
	}
	if (s.consecutiveRejects == 0) {
		s.firstRejectSec = nowSec;
	}
	s.consecutiveRejects += 1;
	if (s.consecutiveRejects >= kEngageConsecutiveRejects && (nowSec - s.firstRejectSec) >= kEngageMinSustainSec) {
		s.engaged = true;
	}
	return s;
}

} // namespace spacecal::quality_breaker
