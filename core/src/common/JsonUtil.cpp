#include "JsonUtil.h"

#include <utility>

namespace openvr_pair::common::json {

void StripUtf8Bom(std::string& body)
{
	if (body.size() >= 3 && static_cast<unsigned char>(body[0]) == 0xEF &&
	    static_cast<unsigned char>(body[1]) == 0xBB && static_cast<unsigned char>(body[2]) == 0xBF) {
		body.erase(0, 3);
	}
}

bool Parse(picojson::value& out, std::string body, std::string* error)
{
	StripUtf8Bom(body);
	std::string parseError = picojson::parse(out, body);
	if (error) *error = parseError;
	return parseError.empty();
}

bool ParseObject(picojson::value& out, std::string body, std::string* error)
{
	if (!Parse(out, std::move(body), error)) return false;
	if (!out.is<picojson::object>()) {
		if (error) *error = "expected JSON object";
		return false;
	}
	return true;
}

const picojson::value* ValueAt(const picojson::value& value, const char* key)
{
	if (!value.is<picojson::object>() || !key) return nullptr;
	const auto& object = value.get<picojson::object>();
	auto it = object.find(key);
	return it == object.end() ? nullptr : &it->second;
}

const picojson::array* ArrayAt(const picojson::value& value, const char* key)
{
	const picojson::value* child = ValueAt(value, key);
	if (!child || !child->is<picojson::array>()) return nullptr;
	return &child->get<picojson::array>();
}

std::string StringAt(const picojson::value& value, const char* key, std::string fallback)
{
	const picojson::value* child = ValueAt(value, key);
	if (!child || !child->is<std::string>()) return fallback;
	return child->get<std::string>();
}

bool BoolAt(const picojson::value& value, const char* key, bool fallback)
{
	const picojson::value* child = ValueAt(value, key);
	if (!child || !child->is<bool>()) return fallback;
	return child->get<bool>();
}

double NumberAt(const picojson::value& value, const char* key, double fallback)
{
	const picojson::value* child = ValueAt(value, key);
	if (!child || !child->is<double>()) return fallback;
	return child->get<double>();
}

int IntAt(const picojson::value& value, const char* key, int fallback)
{
	const picojson::value* child = ValueAt(value, key);
	if (!child || !child->is<double>()) return fallback;
	return static_cast<int>(child->get<double>());
}

} // namespace openvr_pair::common::json
