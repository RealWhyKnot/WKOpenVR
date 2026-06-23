#pragma once

#include <string>

namespace openvr_pair::common {

std::wstring QuotePowerShellLiteral(const std::wstring& value);
std::wstring EncodePowerShellCommand(const std::wstring& script);
std::string EncodePowerShellCommandUtf8(const std::wstring& script);

} // namespace openvr_pair::common
