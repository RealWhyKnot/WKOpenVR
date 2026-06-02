#pragma once

namespace oscrouter::ui {

enum class DriverPanelState
{
    WaitingForSteamVr,
    WaitingForDriver,
    Active,
    Problem,
};

inline DriverPanelState ResolveDriverPanelState(
    bool vrConnected,
    bool statsOpen,
    bool driverWaitElapsed)
{
    if (!vrConnected) return DriverPanelState::WaitingForSteamVr;
    if (statsOpen) return DriverPanelState::Active;
    return driverWaitElapsed
        ? DriverPanelState::Problem
        : DriverPanelState::WaitingForDriver;
}

inline bool ShouldAttemptLiveDriverIpc(bool vrConnected)
{
    return vrConnected;
}

inline bool ShouldRetryLiveDriverIpc(bool vrConnected, bool ipcConnected, bool retryDue)
{
    return vrConnected && !ipcConnected && retryDue;
}

} // namespace oscrouter::ui
