#pragma once

#include "BuildChannel.h"

#if WKOPENVR_BUILD_IS_DEV

#include <string>

namespace openvr_pair::overlay::testharness {

struct SteamVRConflictResult
{
	bool ok = true;
	std::string error;
};

// Returns ok=true when neither vrserver.exe nor any of our shmem segments
// are alive on this machine, meaning the test harness can safely stage its
// own driver tree and create the per-module pipes/shmem without colliding
// with a real session.
SteamVRConflictResult CheckSteamVRConflicts();

} // namespace openvr_pair::overlay::testharness

#endif // WKOPENVR_BUILD_IS_DEV
