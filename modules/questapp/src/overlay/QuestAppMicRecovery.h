#pragma once

#include "AdbController.h"
#include "QuestAppConfig.h"

namespace wkopenvr::questapp {

OperationResult ResetSelectedAppMicrophonePermission(AdbController& adb, const QuestCompanionSettings& settings);

} // namespace wkopenvr::questapp
