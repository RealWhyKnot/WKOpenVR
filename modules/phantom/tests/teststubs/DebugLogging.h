#pragma once

// Test-only DebugLogging.h. The real definition lives in core/src/common/
// DebugLogging.cpp, which the unit-test target does not link. Only
// IsDebugLoggingEnabled is referenced by the code under test; stub it to off so
// the per-tick diagnostic path stays quiet during tests.

namespace openvr_pair::common {
inline bool IsDebugLoggingEnabled()
{
	return false;
}
} // namespace openvr_pair::common
