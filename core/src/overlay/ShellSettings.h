#pragma once

#include <string>

namespace openvr_pair::overlay {

std::string ReadShellSetting(const std::wstring &profileRoot, const char *key, const char *fallback = "");
bool WriteShellSetting(const std::wstring &profileRoot, const char *key, const std::string &value);

} // namespace openvr_pair::overlay
