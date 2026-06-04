// Minimal stubs to satisfy the linker when compiling overlay source files
// (Config.cpp, Profiles.cpp) in isolation for unit tests.
//
// These stubs replace the full overlay logging and OpenVR-runtime-dependent
// symbols so the test binary has no dependency on SteamVR, ImGui, or the
// overlay process.

#include <cstdio>
#include <cstring>
#include <ctime>

// ---------------------------------------------------------------------------
// Logging stubs (used by the LOG() macro in Logging.h).
// ---------------------------------------------------------------------------

FILE* LogFile = nullptr;

void OpenLogFile() {}
bool EnsureLogFileOpen()
{
	return false;
}
void LogFlush() {}

tm TimeForLog()
{
	tm t{};
	return t;
}
