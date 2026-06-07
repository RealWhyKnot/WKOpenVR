#define _CRT_SECURE_NO_DEPRECATE
#include "Profiles.h"

#include "JsonUtil.h"
#include "Logging.h"
#include "Win32Paths.h"
#include "Win32Text.h"

#include <cstdio>
#include <fstream>
#include <sstream>
#include <string>

namespace {

// %LocalAppDataLow%\WKOpenVR\profiles\facetracking.json
std::wstring ProfilePath()
{
	std::wstring dir = openvr_pair::common::WkOpenVrSubdirectoryPath(L"profiles", true);
	return dir.empty() ? std::wstring() : dir + L"\\facetracking.json";
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
	getBool("output_osc_enabled", p.output_osc_enabled);
	getInt("gaze_smoothing", p.gaze_smoothing);
	getInt("openness_smoothing", p.openness_smoothing);
	getBool("mouth_close_compensation_enabled", p.mouth_close_compensation_enabled);
	getBool("smile_mouth_open_assist_enabled", p.smile_mouth_open_assist_enabled);
	getInt("smile_mouth_open_strength", p.smile_mouth_open_strength);
	getBool("idle_mouth_auto_close_enabled", p.idle_mouth_auto_close_enabled);
	getBool("eyelid_brow_sync_enabled", p.eyelid_brow_sync_enabled);
	getInt("eyelid_brow_sync_strength", p.eyelid_brow_sync_strength);
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

std::string Encode(const FacetrackingProfile& p)
{
	picojson::object obj;
	obj["eyelid_sync_enabled"] = picojson::value(p.eyelid_sync_enabled);
	obj["eyelid_sync_preserve_winks"] = picojson::value(p.eyelid_sync_preserve_winks);
	obj["eyelid_sync_strength"] = picojson::value((double)p.eyelid_sync_strength);
	obj["eyelid_sync_mode"] = picojson::value((double)p.eyelid_sync_mode);
	obj["vergence_lock_enabled"] = picojson::value(p.vergence_lock_enabled);
	obj["vergence_lock_strength"] = picojson::value((double)p.vergence_lock_strength);
	obj["continuous_calib_mode"] = picojson::value((double)p.continuous_calib_mode);
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
	obj["show_raw_values"] = picojson::value(p.show_raw_values);
	obj["last_tab_index"] = picojson::value((double)p.last_tab_index);
	return picojson::value(obj).serialize(true);
}

} // namespace

bool FacetrackingProfileStore::Load()
{
	std::wstring path = ProfilePath();
	if (path.empty()) return false;

	std::ifstream in(path);
	if (!in.is_open()) return false;

	std::stringstream ss;
	ss << in.rdbuf();
	picojson::value v;
	std::string err;
	if (!openvr_pair::common::json::Parse(v, ss.str(), &err)) {
		FT_LOG_OVL("[profiles] parse error in '%s': %s", openvr_pair::common::WideToUtf8(path).c_str(), err.c_str());
		return false;
	}
	current = Decode(v);
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
