#pragma once

#include "QuestAppConfig.h"

#include <string>

namespace wkopenvr::questapp {

bool ParseCompanionSettingsJson(const std::string& body, QuestCompanionSettings& settings);

} // namespace wkopenvr::questapp
