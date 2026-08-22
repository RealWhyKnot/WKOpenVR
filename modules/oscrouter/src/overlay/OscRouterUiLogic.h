#pragma once

namespace oscrouter::ui {

enum class DriverPanelState
{
	WaitingForBackend,
	WaitingForDriver,
	Active,
	Problem,
};

inline DriverPanelState ResolveDriverPanelState(bool backendAvailable, bool statsOpen, bool driverWaitElapsed)
{
	if (!backendAvailable) return DriverPanelState::WaitingForBackend;
	if (statsOpen) return DriverPanelState::Active;
	return driverWaitElapsed ? DriverPanelState::Problem : DriverPanelState::WaitingForDriver;
}

inline bool ShouldAttemptLiveDriverIpc(bool backendAvailable)
{
	return backendAvailable;
}

inline bool ShouldRetryLiveDriverIpc(bool backendAvailable, bool ipcConnected, bool retryDue)
{
	return backendAvailable && !ipcConnected && retryDue;
}

} // namespace oscrouter::ui
