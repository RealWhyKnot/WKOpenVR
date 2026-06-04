#define _CRT_SECURE_NO_DEPRECATE
#include "Profiles.h"

#include "JsonUtil.h"
#include "Logging.h"
#include "Win32Paths.h"
#include "Win32Text.h"

#include "inputhealth/PathPolicy.h"
#include "inputhealth/SerialHash.h"
#include "picojson.h"

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace {

std::wstring ProfilesDir()
{
	return openvr_pair::common::WkOpenVrSubdirectoryPath(L"profiles", true);
}

// Serial strings can contain characters that aren't filename-safe (slashes,
// colons on some vendors). Hash the serial and use the hex hash as the
// filename instead -- collisions are vanishingly unlikely on real hardware
// and the filename is stable across reboots.
std::string FilenameForHash(uint64_t hash)
{
	char buf[32];
	snprintf(buf, sizeof(buf), "%016llx.json", (unsigned long long)hash);
	return buf;
}

DeviceProfile Decode(const picojson::value& v)
{
	DeviceProfile p;
	if (!v.is<picojson::object>()) return p;
	const auto& obj = v.get<picojson::object>();
	auto getStr = [&](const char* k, std::string& out) {
		auto it = obj.find(k);
		if (it != obj.end() && it->second.is<std::string>()) out = it->second.get<std::string>();
	};
	auto getBool = [&](const char* k, bool& out) {
		auto it = obj.find(k);
		if (it != obj.end() && it->second.is<bool>()) out = it->second.get<bool>();
	};
	auto getU64 = [&](const char* k, uint64_t& out) {
		auto it = obj.find(k);
		if (it != obj.end() && it->second.is<std::string>()) {
			out = strtoull(it->second.get<std::string>().c_str(), nullptr, 16);
		}
		else if (it != obj.end() && it->second.is<double>()) {
			double d = it->second.get<double>();
			if (d >= 0.0) out = static_cast<uint64_t>(d);
		}
	};
	auto getU32From = [](const picojson::object& src, const char* k, uint32_t& out) {
		auto it = src.find(k);
		if (it != src.end() && it->second.is<double>()) {
			double d = it->second.get<double>();
			if (d >= 0.0 && d <= 4294967295.0) out = static_cast<uint32_t>(d);
		}
	};
	auto getU64From = [](const picojson::object& src, const char* k, uint64_t& out) {
		auto it = src.find(k);
		if (it != src.end() && it->second.is<double>()) {
			double d = it->second.get<double>();
			if (d >= 0.0) out = static_cast<uint64_t>(d);
		}
	};
	auto getDoubleFrom = [](const picojson::object& src, const char* k, double& out) {
		auto it = src.find(k);
		if (it != src.end() && it->second.is<double>()) out = it->second.get<double>();
	};
	auto getBoolFrom = [](const picojson::object& src, const char* k, bool& out) {
		auto it = src.find(k);
		if (it != src.end() && it->second.is<bool>()) out = it->second.get<bool>();
	};
	auto getStrFrom = [](const picojson::object& src, const char* k, std::string& out) {
		auto it = src.find(k);
		if (it != src.end() && it->second.is<std::string>()) out = it->second.get<std::string>();
	};

	getStr("serial", p.serial);
	getU64("serial_hash_hex", p.serial_hash);
	getStr("display_name", p.display_name);
	getBool("enable_diagnostics_only", p.enable_diagnostics_only);
	getBool("enable_rest_recenter", p.enable_rest_recenter);
	getBool("enable_trigger_remap", p.enable_trigger_remap);
	getBool("corrections_enabled", p.corrections_enabled);
	getBool("rest_recenter_migrated", p.rest_recenter_migrated);

	// One-time migration: rest-recenter's default was flipped on. Profiles saved
	// before the flip persisted the old off default and lack the marker above;
	// force rest-recenter on once for them, then record the marker so a later
	// deliberate opt-out is respected.
	if (!p.rest_recenter_migrated) {
		if (!p.enable_rest_recenter) {
			LOG("[profiles] rest-recenter off->on migration for '%s' (default flip)", p.serial.c_str());
		}
		p.enable_rest_recenter = true;
		p.rest_recenter_migrated = true;
	}

	auto learnedIt = obj.find("learned_paths");
	if (learnedIt != obj.end() && learnedIt->second.is<picojson::array>()) {
		for (const auto& item : learnedIt->second.get<picojson::array>()) {
			if (!item.is<picojson::object>()) continue;
			const auto& lpObj = item.get<picojson::object>();
			LearnedPathRecord r;
			getStrFrom(lpObj, "path", r.path);
			getStrFrom(lpObj, "kind", r.kind);
			getU64From(lpObj, "sample_count", r.sample_count);
			getBoolFrom(lpObj, "ready", r.ready);
			getDoubleFrom(lpObj, "learned_rest_offset", r.learned_rest_offset);
			getDoubleFrom(lpObj, "learned_stddev", r.learned_stddev);
			getDoubleFrom(lpObj, "learned_trigger_min", r.learned_trigger_min);
			getDoubleFrom(lpObj, "learned_trigger_max", r.learned_trigger_max);
			getDoubleFrom(lpObj, "learned_deadzone_radius", r.learned_deadzone_radius);
			getU32From(lpObj, "learned_debounce_us", r.learned_debounce_us);
			getU64From(lpObj, "last_updated_unix", r.last_updated_unix);
			getU32From(lpObj, "drift_shift_resets", r.drift_shift_resets);
			if (!r.path.empty()) p.learned_paths.push_back(std::move(r));
		}
	}

	if (p.serial_hash == 0 && !p.serial.empty()) {
		p.serial_hash = inputhealth::Fnv1a64(p.serial);
	}

	// Drop records whose paths are unsafe for persistent compensation and
	// sanitize legacy force/grip records so they are interpreted only as
	// capped idle-floor offsets.
	{
		size_t before = p.learned_paths.size();
		p.learned_paths.erase(std::remove_if(p.learned_paths.begin(), p.learned_paths.end(),
		                                     [](const LearnedPathRecord& r) {
			                                     const inputhealth::PathFamily family =
			                                         inputhealth::ClassifyPathFamily(r.path);
			                                     return family == inputhealth::PathFamily::Unsupported ||
			                                            inputhealth::IsDiagnosticsOnlyFamily(family);
		                                     }),
		                      p.learned_paths.end());
		size_t after = p.learned_paths.size();
		if (before != after) {
			LOG("[profiles] pruned %zu legacy path record(s) from serial_hash=0x%016llx", before - after,
			    (unsigned long long)p.serial_hash);
		}

		for (auto& record : p.learned_paths) {
			const inputhealth::PathFamily family = inputhealth::ClassifyPathFamily(record.path);
			if (!inputhealth::IsIdleFloorFamily(family)) continue;
			record.kind = "scalar_single";
			record.learned_rest_offset = std::max(0.0, std::min(0.05, record.learned_rest_offset));
			record.learned_trigger_min = 0.0;
			record.learned_trigger_max = 0.0;
			record.learned_deadzone_radius = 0.0;
			record.learned_debounce_us = 0;
		}
	}

	return p;
}

std::string Encode(const DeviceProfile& p)
{
	char hashHex[32];
	snprintf(hashHex, sizeof(hashHex), "%016llx", (unsigned long long)p.serial_hash);

	picojson::object obj;
	obj["serial"] = picojson::value(p.serial);
	obj["serial_hash_hex"] = picojson::value(std::string(hashHex));
	obj["display_name"] = picojson::value(p.display_name);
	obj["enable_diagnostics_only"] = picojson::value(p.enable_diagnostics_only);
	obj["enable_rest_recenter"] = picojson::value(p.enable_rest_recenter);
	obj["enable_trigger_remap"] = picojson::value(p.enable_trigger_remap);
	obj["corrections_enabled"] = picojson::value(p.corrections_enabled);
	obj["rest_recenter_migrated"] = picojson::value(p.rest_recenter_migrated);

	picojson::array learned;
	learned.reserve(p.learned_paths.size());
	for (const auto& r : p.learned_paths) {
		picojson::object item;
		item["path"] = picojson::value(r.path);
		item["kind"] = picojson::value(r.kind);
		item["sample_count"] = picojson::value(static_cast<double>(r.sample_count));
		item["ready"] = picojson::value(r.ready);
		item["learned_rest_offset"] = picojson::value(r.learned_rest_offset);
		item["learned_stddev"] = picojson::value(r.learned_stddev);
		item["learned_trigger_min"] = picojson::value(r.learned_trigger_min);
		item["learned_trigger_max"] = picojson::value(r.learned_trigger_max);
		item["learned_deadzone_radius"] = picojson::value(r.learned_deadzone_radius);
		item["learned_debounce_us"] = picojson::value(static_cast<double>(r.learned_debounce_us));
		item["last_updated_unix"] = picojson::value(static_cast<double>(r.last_updated_unix));
		item["drift_shift_resets"] = picojson::value(static_cast<double>(r.drift_shift_resets));
		learned.push_back(picojson::value(item));
	}
	obj["learned_paths"] = picojson::value(learned);
	return picojson::value(obj).serialize(true);
}

} // namespace

void ProfileStore::LoadAll()
{
	std::wstring dir = ProfilesDir();
	if (dir.empty()) return;

	std::wstring search = dir + L"\\*.json";
	WIN32_FIND_DATAW find_data{};
	HANDLE h = FindFirstFileW(search.c_str(), &find_data);
	if (h == INVALID_HANDLE_VALUE) return;

	do {
		std::wstring path = dir + L"\\" + find_data.cFileName;
		std::ifstream in(path);
		if (!in.is_open()) continue;
		std::stringstream ss;
		ss << in.rdbuf();
		picojson::value v;
		std::string err;
		if (!openvr_pair::common::json::Parse(v, ss.str(), &err)) {
			LOG("[profiles] parse error in '%s': %s", openvr_pair::common::WideToUtf8(path).c_str(), err.c_str());
			continue;
		}
		DeviceProfile p = Decode(v);
		if (p.serial_hash == 0) continue;
		profiles_[p.serial_hash] = std::move(p);
	} while (FindNextFileW(h, &find_data));
	FindClose(h);

	LOG("[profiles] loaded %zu profile(s) from disk (checkpoint)", profiles_.size());
}

bool ProfileStore::Save(const DeviceProfile& profile, const char* reason)
{
	++stats_.attempted_saves;
	if (reason && reason[0] != '\0') {
		stats_.last_save_reason = reason;
	}

	if (profile.serial_hash == 0) {
		++stats_.failed_writes;
		return false;
	}
	std::wstring dir = ProfilesDir();
	if (dir.empty()) {
		++stats_.failed_writes;
		return false;
	}

	// Coalesce by content hash. Save is invoked on every input-state delta
	// per device (~8 saves/sec in observed bursts, ~1100 saves/session) and
	// the vast majority write byte-identical content. Compute an FNV-1a-64
	// hash of the JSON-encoded profile and skip the entire write path if
	// it matches the previous successful save for this device. Keeps a
	// throttled summary log so the suppression rate is auditable.
	std::string preBody = Encode(profile);
	uint64_t hash = 0xcbf29ce484222325ULL;
	for (unsigned char c : preBody) {
		hash ^= c;
		hash *= 0x100000001b3ULL;
	}
	static std::unordered_map<uint64_t, uint64_t> s_lastSavedHashByDevice;
	static std::unordered_map<uint64_t, int> s_skipCountByDevice;
	static std::chrono::steady_clock::time_point s_lastSkipLog{};
	auto it = s_lastSavedHashByDevice.find(profile.serial_hash);
	if (it != s_lastSavedHashByDevice.end() && it->second == hash) {
		++stats_.skipped_unchanged;
		++s_skipCountByDevice[profile.serial_hash];
		const auto nowTp = std::chrono::steady_clock::now();
		if (nowTp - s_lastSkipLog >= std::chrono::seconds(30)) {
			s_lastSkipLog = nowTp;
			int totalSkipped = 0;
			for (const auto& kv : s_skipCountByDevice)
				totalSkipped += kv.second;
			LOG("[profiles] checkpoint coalesced: skipped=%d in last ~30s", totalSkipped);
			s_skipCountByDevice.clear();
		}
		return true;
	}

	std::wstring path = dir + L"\\" + openvr_pair::common::Utf8ToWide(FilenameForHash(profile.serial_hash));
	std::wstring tmpPath = path + L".tmp";

	// Write to a temp file first so a crash mid-write never corrupts the
	// existing profile. MoveFileExW with WRITE_THROUGH makes the rename
	// durable before we return success.
	HANDLE hFile =
	    CreateFileW(tmpPath.c_str(), GENERIC_WRITE, 0, nullptr, CREATE_ALWAYS, FILE_ATTRIBUTE_NORMAL, nullptr);
	if (hFile == INVALID_HANDLE_VALUE) {
		++stats_.failed_writes;
		LOG("[profiles] failed to open tmp '%s' for write (err=%lu)", openvr_pair::common::WideToUtf8(tmpPath).c_str(),
		    GetLastError());
		return false;
	}

	const std::string& body = preBody;
	DWORD written = 0;
	BOOL ok = WriteFile(hFile, body.data(), static_cast<DWORD>(body.size()), &written, nullptr);
	if (ok) ok = FlushFileBuffers(hFile);
	CloseHandle(hFile);

	if (!ok || written != static_cast<DWORD>(body.size())) {
		++stats_.failed_writes;
		LOG("[profiles] write/flush failed for tmp '%s' (err=%lu)", openvr_pair::common::WideToUtf8(tmpPath).c_str(),
		    GetLastError());
		DeleteFileW(tmpPath.c_str());
		return false;
	}

	if (!MoveFileExW(tmpPath.c_str(), path.c_str(), MOVEFILE_REPLACE_EXISTING | MOVEFILE_WRITE_THROUGH)) {
		++stats_.failed_writes;
		LOG("[profiles] atomic rename failed '%s' -> '%s' (err=%lu)", openvr_pair::common::WideToUtf8(tmpPath).c_str(),
		    openvr_pair::common::WideToUtf8(path).c_str(), GetLastError());
		DeleteFileW(tmpPath.c_str());
		return false;
	}

	profiles_[profile.serial_hash] = profile;
	s_lastSavedHashByDevice[profile.serial_hash] = hash;
	++stats_.actual_writes;
	LOG("[profiles] saved profile checkpoint: serial_hash=0x%016llx paths=%zu hash=0x%016llx reason=%s",
	    (unsigned long long)profile.serial_hash, profile.learned_paths.size(), (unsigned long long)hash,
	    stats_.last_save_reason.empty() ? "unspecified" : stats_.last_save_reason.c_str());
	return true;
}

DeviceProfile& ProfileStore::GetOrCreate(uint64_t serial_hash)
{
	auto it = profiles_.find(serial_hash);
	if (it != profiles_.end()) return it->second;
	DeviceProfile p;
	p.serial_hash = serial_hash;
	auto& slot = profiles_[serial_hash];
	slot = std::move(p);
	return slot;
}
