#pragma once

namespace phantom::ui {

inline bool ShouldAttemptDriverConnection(bool vrConnected)
{
    return vrConnected;
}

inline bool ShouldShowDriverError(bool vrConnected, bool hasDriverError)
{
    return vrConnected && hasDriverError;
}

} // namespace phantom::ui
