#pragma once

#include <cstdint>

// Pure diagnostic rollup for the head-mounted "witness" puck. Surfaced in the
// [cal-heartbeat] line so a session reader can see the witness dropping out
// without parsing every relocalization event. Field data (2026-06-30) showed
// the puck valid for only the first ~3 h of a 5 h session, which silently
// disables witness corroboration -- this makes that visible. Kept pure (no
// CalCtx / OpenVR) so the accounting is unit-testable; the caller supplies the
// per-tick validity and the current time.

namespace spacecal::witness_health {

struct WitnessHealth
{
	uint64_t validTicks = 0;         // ticks the bound witness reported a valid pose
	uint64_t totalTicks = 0;         // ticks a witness was bound and evaluated
	double lastValidTime = -1.0;     // time of last valid pose; <0 = never valid
	uint64_t subthresholdRelocs = 0; // relocs blocked ONLY by the magnitude gate
};

// Record one recovery tick. Counts toward totalTicks only when a witness is
// bound (evaluated); stamps lastValidTime when the pose was valid this tick.
inline void TickWitness(WitnessHealth& w, bool witnessBound, bool witnessValid, double now)
{
	if (!witnessBound) return;
	w.totalTicks++;
	if (witnessValid) {
		w.validTicks++;
		w.lastValidTime = now;
	}
}

// Record a relocalization that passed every recovery gate except magnitude (a
// 5-30 cm SLAM jump that translated the headset uncorrected) -- the felt-jump
// rate.
inline void NoteSubthresholdReloc(WitnessHealth& w)
{
	w.subthresholdRelocs++;
}

// Fraction (0-100) of evaluated ticks the witness was valid; 0 when none.
inline double ValidPct(const WitnessHealth& w)
{
	return w.totalTicks ? 100.0 * static_cast<double>(w.validTicks) / static_cast<double>(w.totalTicks) : 0.0;
}

// Seconds since the witness last reported a valid pose; -1 if never valid.
inline double LastValidSec(const WitnessHealth& w, double now)
{
	return w.lastValidTime < 0.0 ? -1.0 : now - w.lastValidTime;
}

} // namespace spacecal::witness_health
