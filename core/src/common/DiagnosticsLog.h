#pragma once

#include <cstdarg>

namespace openvr_pair::common {

void DiagnosticLog(const char* component, const char* fmt, ...);
void DiagnosticLogV(const char* component, const char* fmt, va_list args);

} // namespace openvr_pair::common
