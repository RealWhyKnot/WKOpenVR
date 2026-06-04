#pragma once

namespace inputhealth::ui {

inline bool ShouldShowDriverProblemBanner(bool hasDriverError, bool isDriverWaitError)
{
	return hasDriverError && !isDriverWaitError;
}

inline bool ShouldShowShmemProblemText(bool vrConnected, bool shmemOpen, bool hasShmemError, bool isVersionMismatch)
{
	return !shmemOpen && hasShmemError && (vrConnected || isVersionMismatch);
}

} // namespace inputhealth::ui
