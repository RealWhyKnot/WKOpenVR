#pragma once

// Test-only Logging.h. The real driver logger (core/src/driver/Logging.h) is not
// on the unit-test include path and its backing functions are not linked, so route
// LOG / TRACE to a no-op that still consumes its arguments (keeps them "used" under
// /W4) without performing any I/O.

namespace phantom_test_detail {
inline void NoOpLog(const char* /*fmt*/, ...) {}
} // namespace phantom_test_detail

#ifndef LOG
#define LOG(...) ::phantom_test_detail::NoOpLog(__VA_ARGS__)
#endif

#ifndef TRACE
#define TRACE LOG
#endif
