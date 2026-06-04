#pragma once

#include <algorithm>
#include <cmath>

// EWMA-decayed rolling minimum.
//
// Used by InputHealth for trigger rest-min stuck detection: the rest-min
// observed during identified rest periods (trigger value < rest_threshold)
// EWMA-decays toward the lowest seen value. A trigger that does not return
// to zero shows up as a rest-min that stays elevated even as the EWMA pulls
// toward the running minimum.
//
// Why not a plain rolling min over a fixed window:
//   A plain min is sensitive to a single very-low sample (e.g. a momentary
//   stick crossing zero during recoil) that hides hours of subsequent
//   stuck-high behavior. EWMA decay lets that single low get out-weighed by
//   the steady stuck-high signal once enough time passes.
//
// The form used here is asymmetric:
//   if x < state:  state = x                            // adopt new minimum
//   else:           state = (1 - decay) * state + decay * x  // drift toward x
//
// decay = alpha computed from a wall-clock half-life via
//   alpha = 1 - exp(-dt / tau).
// Typical tau for trigger rest detection: 24 hours (slow drift) for the
// "is this stuck up?" estimator; 30 seconds for the "current rest" estimator.
//
// O(1) per update. Caller owns thread safety. The state is a single double
// plus an initialization flag.

namespace inputhealth {

struct EWMARollingMinState
{
	double value = 0.0;
	bool initialized = false;
};

inline void EWMARollingMinUpdate(EWMARollingMinState& s, double x, double decay)
{
	if (decay < 0.0) decay = 0.0;
	if (decay > 1.0) decay = 1.0;
	if (!s.initialized) {
		s.value = x;
		s.initialized = true;
		return;
	}
	if (x < s.value) {
		s.value = x;
	}
	else {
		s.value = (1.0 - decay) * s.value + decay * x;
	}
}

inline void EWMARollingMinReset(EWMARollingMinState& s)
{
	s.value = 0.0;
	s.initialized = false;
}

inline double EWMARollingMinValue(const EWMARollingMinState& s)
{
	return s.initialized ? s.value : 0.0;
}

} // namespace inputhealth
