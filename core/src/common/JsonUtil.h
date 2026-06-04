#pragma once

#include "picojson.h"

#include <string>

namespace openvr_pair::common::json {

void StripUtf8Bom(std::string& body);
bool Parse(picojson::value& out, std::string body, std::string* error = nullptr);
bool ParseObject(picojson::value& out, std::string body, std::string* error = nullptr);

const picojson::value* ValueAt(const picojson::value& value, const char* key);
const picojson::array* ArrayAt(const picojson::value& value, const char* key);

std::string StringAt(const picojson::value& value, const char* key, std::string fallback = {});
bool BoolAt(const picojson::value& value, const char* key, bool fallback = false);
double NumberAt(const picojson::value& value, const char* key, double fallback = 0.0);
int IntAt(const picojson::value& value, const char* key, int fallback = 0);

} // namespace openvr_pair::common::json
