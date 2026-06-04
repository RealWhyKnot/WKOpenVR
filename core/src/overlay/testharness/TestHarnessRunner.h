#pragma once

#include "BuildChannel.h"

#if WKOPENVR_BUILD_IS_DEV

namespace openvr_pair::overlay::testharness {

// Entry point invoked from main() when --test-harness is on the command line.
// Returns a process exit code: 0 on full pass, non-zero on any failure or
// precondition violation (release-channel binary, SteamVR alive, etc.).
int Run(int argc, char** argv);

} // namespace openvr_pair::overlay::testharness

#endif // WKOPENVR_BUILD_IS_DEV
