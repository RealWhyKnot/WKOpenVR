#define _CRT_SECURE_NO_DEPRECATE
#include "Profiles.h"

#include "JsonUtil.h"
#include "Logging.h"
#include "Win32Paths.h"
#include "Win32Text.h"
#include "facetracking/ExpressionNames.h"

#include <algorithm>
#include <cstdio>
#include <ctime>
#include <fstream>
#include <sstream>
#include <string>
#include <utility>

namespace {

// %LocalAppDataLow%\WKOpenVR\profiles\facetracking.json
std::wstring ProfilePath()
{
	std::wstring dir = openvr_pair::common::WkOpenVrSubdirectoryPath(L"profiles", true);
	return dir.empty() ? std::wstring() : dir + L"\\facetracking.json";
}

bool IsAsciiSpace(char c)
{
	return c == ' ' || c == '\t' || c == '\r' || c == '\n';
}

std::string TrimAsciiCopy(std::string value)
{
	size_t first = 0;
	while (first < value.size() && IsAsciiSpace(value[first]))
		++first;
	size_t last = value.size();
	while (last > first && IsAsciiSpace(value[last - 1]))
		--last;
	if (first > 0 || last < value.size()) value = value.substr(first, last - first);
	return value;
}

int ClampShapeScale(int value)
{
	return std::clamp(value, 0, static_cast<int>(protocol::FACETRACKING_SHAPE_TUNING_MAX_PERCENT));
}

int ExpressionIndexForName(const std::string& name)
{
	for (uint32_t i = 0; i < protocol::FACETRACKING_EXPRESSION_COUNT; ++i) {
		if (name == facetracking::ExpressionName(i)) return static_cast<int>(i);
	}
	return -1;
}

void DecodeAvatarShapeTuning(const picojson::object& obj, FacetrackingProfile& p)
{
	auto rootIt = obj.find("avatar_shape_tuning");
	if (rootIt == obj.end() || !rootIt->second.is<picojson::object>()) return;

	const auto& avatarObj = rootIt->second.get<picojson::object>();
	for (const auto& avatarEntry : avatarObj) {
		if (!avatarEntry.second.is<picojson::object>()) continue;

		FaceShapeScaleArray values = DefaultFaceShapeScales();
		bool sawScale = false;
		const auto& shapeObj = avatarEntry.second.get<picojson::object>();
		for (const auto& shapeEntry : shapeObj) {
			if (!shapeEntry.second.is<double>()) continue;
			const int index = ExpressionIndexForName(shapeEntry.first);
			if (index < 0) continue;
			values[static_cast<size_t>(index)] = ClampShapeScale(static_cast<int>(shapeEntry.second.get<double>()));
			sawScale = true;
		}

		if (sawScale && !IsDefaultFaceShapeScales(values)) {
			p.avatar_shape_tuning[NormalizeAvatarShapeTuningKey(avatarEntry.first)] = values;
		}
	}
}

void DecodeAvatarShapeMetadata(const picojson::object& obj, FacetrackingProfile& p)
{
	auto rootIt = obj.find("avatar_shape_metadata");
	if (rootIt == obj.end() || !rootIt->second.is<picojson::object>()) return;

	const auto& avatarObj = rootIt->second.get<picojson::object>();
	for (const auto& avatarEntry : avatarObj) {
		if (!avatarEntry.second.is<picojson::object>()) continue;

		AvatarShapeTuningMetadata metadata;
		metadata.custom_name = TrimAsciiCopy(openvr_pair::common::json::StringAt(avatarEntry.second, "custom_name"));
		metadata.auto_name = TrimAsciiCopy(openvr_pair::common::json::StringAt(avatarEntry.second, "auto_name"));
		metadata.last_used_utc =
		    TrimAsciiCopy(openvr_pair::common::json::StringAt(avatarEntry.second, "last_used_utc"));
		metadata.config_path = TrimAsciiCopy(openvr_pair::common::json::StringAt(avatarEntry.second, "config_path"));

		if (!metadata.custom_name.empty() || !metadata.auto_name.empty() || !metadata.last_used_utc.empty() ||
		    !metadata.config_path.empty()) {
			p.avatar_shape_metadata[NormalizeAvatarShapeTuningKey(avatarEntry.first)] = std::move(metadata);
		}
	}
}

FacetrackingProfile Decode(const picojson::value& v)
{
	FacetrackingProfile p;
	if (!v.is<picojson::object>()) return p;
	const auto& obj = v.get<picojson::object>();

	auto getBool = [&](const char* k, bool& out) {
		auto it = obj.find(k);
		if (it != obj.end() && it->second.is<bool>()) out = it->second.get<bool>();
	};
	auto getInt = [&](const char* k, int& out) {
		auto it = obj.find(k);
		if (it != obj.end() && it->second.is<double>()) out = static_cast<int>(it->second.get<double>());
	};
	auto getStr = [&](const char* k, std::string& out) {
		auto it = obj.find(k);
		if (it != obj.end() && it->second.is<std::string>()) out = it->second.get<std::string>();
	};

	getBool("eyelid_sync_enabled", p.eyelid_sync_enabled);
	getBool("eyelid_sync_preserve_winks", p.eyelid_sync_preserve_winks);
	getInt("eyelid_sync_strength", p.eyelid_sync_strength);
	getInt("eyelid_sync_mode", p.eyelid_sync_mode);
	if (p.eyelid_sync_mode != protocol::FACETRACKING_EYELID_SYNC_MOST_OPEN) {
		p.eyelid_sync_mode = protocol::FACETRACKING_EYELID_SYNC_MOST_CLOSED;
	}
	getBool("vergence_lock_enabled", p.vergence_lock_enabled);
	getInt("vergence_lock_strength", p.vergence_lock_strength);
	getInt("continuous_calib_mode", p.continuous_calib_mode);
	p.continuous_calib_mode = 0;
	getBool("output_osc_enabled", p.output_osc_enabled);
	getInt("gaze_smoothing", p.gaze_smoothing);
	getInt("openness_smoothing", p.openness_smoothing);
	getBool("mouth_close_compensation_enabled", p.mouth_close_compensation_enabled);
	getBool("smile_mouth_open_assist_enabled", p.smile_mouth_open_assist_enabled);
	getInt("smile_mouth_open_strength", p.smile_mouth_open_strength);
	getBool("idle_mouth_auto_close_enabled", p.idle_mouth_auto_close_enabled);
	getBool("eyelid_brow_sync_enabled", p.eyelid_brow_sync_enabled);
	getInt("eyelid_brow_sync_strength", p.eyelid_brow_sync_strength);
	DecodeAvatarShapeTuning(obj, p);
	DecodeAvatarShapeMetadata(obj, p);
	// enabled_module_uuids -- the multi-select list the Modules tab edits.
	// Read the array form first; if missing, fall back to the deprecated
	// single-uuid string field so users upgrading across this change keep
	// their selection.
	auto enabledIt = obj.find("enabled_module_uuids");
	if (enabledIt != obj.end() && enabledIt->second.is<picojson::array>()) {
		for (const auto& el : enabledIt->second.get<picojson::array>()) {
			if (el.is<std::string>()) {
				const std::string& s = el.get<std::string>();
				if (!s.empty()) p.enabled_module_uuids.push_back(s);
			}
		}
	}
	else {
		std::string legacy;
		getStr("active_module_uuid", legacy);
		if (!legacy.empty()) p.enabled_module_uuids.push_back(std::move(legacy));
	}

	getBool("show_raw_values", p.show_raw_values);
	getInt("last_tab_index", p.last_tab_index);

	return p;
}

bool HasLegacyContinuousCalibrationEnabled(const picojson::value& v)
{
	if (!v.is<picojson::object>()) return false;
	const auto& obj = v.get<picojson::object>();
	auto it = obj.find("continuous_calib_mode");
	return it != obj.end() && it->second.is<double>() && static_cast<int>(it->second.get<double>()) != 0;
}

std::string Encode(const FacetrackingProfile& p)
{
	picojson::object obj;
	obj["eyelid_sync_enabled"] = picojson::value(p.eyelid_sync_enabled);
	obj["eyelid_sync_preserve_winks"] = picojson::value(p.eyelid_sync_preserve_winks);
	obj["eyelid_sync_strength"] = picojson::value((double)p.eyelid_sync_strength);
	obj["eyelid_sync_mode"] = picojson::value((double)p.eyelid_sync_mode);
	obj["vergence_lock_enabled"] = picojson::value(p.vergence_lock_enabled);
	obj["vergence_lock_strength"] = picojson::value((double)p.vergence_lock_strength);
	obj["continuous_calib_mode"] = picojson::value(0.0);
	obj["output_osc_enabled"] = picojson::value(p.output_osc_enabled);
	obj["gaze_smoothing"] = picojson::value((double)p.gaze_smoothing);
	obj["openness_smoothing"] = picojson::value((double)p.openness_smoothing);
	obj["mouth_close_compensation_enabled"] = picojson::value(p.mouth_close_compensation_enabled);
	obj["smile_mouth_open_assist_enabled"] = picojson::value(p.smile_mouth_open_assist_enabled);
	obj["smile_mouth_open_strength"] = picojson::value((double)p.smile_mouth_open_strength);
	obj["idle_mouth_auto_close_enabled"] = picojson::value(p.idle_mouth_auto_close_enabled);
	obj["eyelid_brow_sync_enabled"] = picojson::value(p.eyelid_brow_sync_enabled);
	obj["eyelid_brow_sync_strength"] = picojson::value((double)p.eyelid_brow_sync_strength);
	{
		picojson::array arr;
		for (const auto& u : p.enabled_module_uuids)
			arr.push_back(picojson::value(u));
		obj["enabled_module_uuids"] = picojson::value(arr);
	}
	{
		picojson::object tuningRoot;
		for (const auto& entry : p.avatar_shape_tuning) {
			if (IsDefaultFaceShapeScales(entry.second)) continue;

			picojson::object shapeObj;
			for (uint32_t i = 0; i < protocol::FACETRACKING_EXPRESSION_COUNT; ++i) {
				const int value = ClampShapeScale(entry.second[i]);
				if (value == protocol::FACETRACKING_SHAPE_TUNING_DEFAULT_PERCENT) continue;
				shapeObj[facetracking::ExpressionName(i)] = picojson::value(static_cast<double>(value));
			}
			if (!shapeObj.empty()) {
				tuningRoot[NormalizeAvatarShapeTuningKey(entry.first)] = picojson::value(shapeObj);
			}
		}
		if (!tuningRoot.empty()) obj["avatar_shape_tuning"] = picojson::value(tuningRoot);
	}
	{
		picojson::object metadataRoot;
		for (const auto& entry : p.avatar_shape_metadata) {
			picojson::object metadataObj;
			const std::string custom = TrimAsciiCopy(entry.second.custom_name);
			const std::string automatic = TrimAsciiCopy(entry.second.auto_name);
			const std::string lastUsed = TrimAsciiCopy(entry.second.last_used_utc);
			const std::string configPath = TrimAsciiCopy(entry.second.config_path);
			if (!custom.empty()) metadataObj["custom_name"] = picojson::value(custom);
			if (!automatic.empty()) metadataObj["auto_name"] = picojson::value(automatic);
			if (!lastUsed.empty()) metadataObj["last_used_utc"] = picojson::value(lastUsed);
			if (!configPath.empty()) metadataObj["config_path"] = picojson::value(configPath);
			if (!metadataObj.empty()) {
				metadataRoot[NormalizeAvatarShapeTuningKey(entry.first)] = picojson::value(metadataObj);
			}
		}
		if (!metadataRoot.empty()) obj["avatar_shape_metadata"] = picojson::value(metadataRoot);
	}
	obj["show_raw_values"] = picojson::value(p.show_raw_values);
	obj["last_tab_index"] = picojson::value((double)p.last_tab_index);
	return picojson::value(obj).serialize(true);
}

} // namespace

FaceShapeScaleArray DefaultFaceShapeScales()
{
	FaceShapeScaleArray values{};
	values.fill(protocol::FACETRACKING_SHAPE_TUNING_DEFAULT_PERCENT);
	return values;
}

std::string NormalizeAvatarShapeTuningKey(std::string key)
{
	key = TrimAsciiCopy(std::move(key));
	return key.empty() ? std::string(kDefaultAvatarShapeTuningKey) : key;
}

bool IsDefaultFaceShapeScales(const FaceShapeScaleArray& values)
{
	return std::all_of(values.begin(), values.end(),
	                   [](int value) { return value == protocol::FACETRACKING_SHAPE_TUNING_DEFAULT_PERCENT; });
}

FaceShapeScaleArray& ShapeTuningForAvatar(FacetrackingProfile& profile, const std::string& avatarKey)
{
	const std::string key = NormalizeAvatarShapeTuningKey(avatarKey);
	auto it = profile.avatar_shape_tuning.find(key);
	if (it != profile.avatar_shape_tuning.end()) return it->second;
	auto inserted = profile.avatar_shape_tuning.emplace(key, DefaultFaceShapeScales());
	return inserted.first->second;
}

const FaceShapeScaleArray* FindShapeTuningForAvatar(const FacetrackingProfile& profile, const std::string& avatarKey)
{
	const std::string key = NormalizeAvatarShapeTuningKey(avatarKey);
	auto it = profile.avatar_shape_tuning.find(key);
	return it == profile.avatar_shape_tuning.end() ? nullptr : &it->second;
}

void PruneAvatarShapeTuning(FacetrackingProfile& profile, const std::string& avatarKey)
{
	const std::string key = NormalizeAvatarShapeTuningKey(avatarKey);
	auto it = profile.avatar_shape_tuning.find(key);
	if (it != profile.avatar_shape_tuning.end() && IsDefaultFaceShapeScales(it->second)) {
		profile.avatar_shape_tuning.erase(it);
	}
}

AvatarShapeTuningMetadata& MetadataForAvatar(FacetrackingProfile& profile, const std::string& avatarKey)
{
	const std::string key = NormalizeAvatarShapeTuningKey(avatarKey);
	return profile.avatar_shape_metadata[key];
}

const AvatarShapeTuningMetadata* FindMetadataForAvatar(const FacetrackingProfile& profile, const std::string& avatarKey)
{
	const std::string key = NormalizeAvatarShapeTuningKey(avatarKey);
	auto it = profile.avatar_shape_metadata.find(key);
	return it == profile.avatar_shape_metadata.end() ? nullptr : &it->second;
}

std::string AvatarDisplayName(const std::string& avatarKey, const AvatarShapeTuningMetadata* metadata)
{
	if (metadata) {
		std::string custom = TrimAsciiCopy(metadata->custom_name);
		if (!custom.empty()) return custom;
		std::string automatic = TrimAsciiCopy(metadata->auto_name);
		if (!automatic.empty()) return automatic;
	}

	std::string key = NormalizeAvatarShapeTuningKey(avatarKey);
	if (key == kDefaultAvatarShapeTuningKey) return "Default profile";
	if (key.rfind("avtr_", 0) == 0) {
		const size_t firstDash = key.find('-', 5);
		const size_t end = firstDash == std::string::npos ? std::min<size_t>(key.size(), 13) : firstDash;
		return key.substr(0, end);
	}
	if (key.size() > 18) return key.substr(0, 18) + "...";
	return key;
}

std::string AvatarDisplaySourceLabel(const std::string& avatarKey, const AvatarShapeTuningMetadata* metadata)
{
	if (metadata) {
		if (!TrimAsciiCopy(metadata->custom_name).empty()) return "Alias";
		if (!TrimAsciiCopy(metadata->auto_name).empty()) return "OSC";
	}

	const std::string key = NormalizeAvatarShapeTuningKey(avatarKey);
	return key == kDefaultAvatarShapeTuningKey ? "Default" : "ID";
}

int64_t AvatarLastUsedUnixSeconds(const std::string& utc)
{
	int year = 0;
	int month = 0;
	int day = 0;
	int hour = 0;
	int minute = 0;
	int second = 0;
	if (std::sscanf(utc.c_str(), "%d-%d-%dT%d:%d:%d", &year, &month, &day, &hour, &minute, &second) != 6) {
		return 0;
	}
	if (year < 1970 || month < 1 || month > 12 || day < 1 || day > 31 || hour < 0 || hour > 23 || minute < 0 ||
	    minute > 59 || second < 0 || second > 60) {
		return 0;
	}

	std::tm tm{};
	tm.tm_year = year - 1900;
	tm.tm_mon = month - 1;
	tm.tm_mday = day;
	tm.tm_hour = hour;
	tm.tm_min = minute;
	tm.tm_sec = second;
	tm.tm_isdst = 0;
	const __time64_t value = ::_mkgmtime64(&tm);
	return value < 0 ? 0 : static_cast<int64_t>(value);
}

std::string FormatAvatarLastUsedAge(const std::string& utc, int64_t now_unix_seconds)
{
	const int64_t last = AvatarLastUsedUnixSeconds(utc);
	if (last <= 0 || now_unix_seconds <= 0) return "last used unknown";
	int64_t delta = now_unix_seconds - last;
	if (delta <= 5) return "used just now";
	if (delta < 0) delta = 0;

	auto formatUnit = [](int64_t count, const char* singular, const char* plural) {
		char buf[64];
		std::snprintf(buf, sizeof(buf), "used %lld %s ago", static_cast<long long>(count),
		              count == 1 ? singular : plural);
		return std::string(buf);
	};

	if (delta < 60) return formatUnit(delta, "second", "seconds");
	const int64_t minutes = delta / 60;
	if (minutes < 60) return formatUnit(minutes, "minute", "minutes");
	const int64_t hours = minutes / 60;
	if (hours < 24) return formatUnit(hours, "hour", "hours");
	return formatUnit(hours / 24, "day", "days");
}

std::string FormatAvatarLastUsedAge(const std::string& utc)
{
	return FormatAvatarLastUsedAge(utc, static_cast<int64_t>(std::time(nullptr)));
}

bool FacetrackingProfileStore::Load()
{
	std::wstring path = ProfilePath();
	if (path.empty()) return false;

	std::ifstream in(path);
	if (!in.is_open()) return false;

	std::stringstream ss;
	ss << in.rdbuf();
	in.close();

	picojson::value v;
	std::string err;
	if (!openvr_pair::common::json::Parse(v, ss.str(), &err)) {
		FT_LOG_OVL("[profiles] parse error in '%s': %s", openvr_pair::common::WideToUtf8(path).c_str(), err.c_str());
		return false;
	}
	current = Decode(v);
	if (HasLegacyContinuousCalibrationEnabled(v)) {
		FT_LOG_OVL("[profiles] disabled legacy continuous calibration mode");
		Save();
	}
	FT_LOG_OVL("[profiles] loaded facetracking.json");
	return true;
}

bool FacetrackingProfileStore::Save() const
{
	std::wstring path = ProfilePath();
	if (path.empty()) return false;

	std::wstring tmp = path + L".tmp";
	HANDLE hFile = CreateFileW(tmp.c_str(), GENERIC_WRITE, 0, nullptr, CREATE_ALWAYS, FILE_ATTRIBUTE_NORMAL, nullptr);
	if (hFile == INVALID_HANDLE_VALUE) {
		FT_LOG_OVL("[profiles] failed to open tmp for write (err=%lu)", GetLastError());
		return false;
	}

	std::string body = Encode(current);
	DWORD written = 0;
	BOOL ok = WriteFile(hFile, body.data(), (DWORD)body.size(), &written, nullptr);
	if (ok) ok = FlushFileBuffers(hFile);
	CloseHandle(hFile);

	if (!ok || written != (DWORD)body.size()) {
		FT_LOG_OVL("[profiles] write/flush failed for facetracking.json.tmp (err=%lu)", GetLastError());
		DeleteFileW(tmp.c_str());
		return false;
	}

	if (!MoveFileExW(tmp.c_str(), path.c_str(), MOVEFILE_REPLACE_EXISTING | MOVEFILE_WRITE_THROUGH)) {
		FT_LOG_OVL("[profiles] atomic rename failed (err=%lu)", GetLastError());
		DeleteFileW(tmp.c_str());
		return false;
	}

	FT_LOG_OVL("[profiles] saved facetracking.json");
	return true;
}
