#include "QuestAppConfig.h"

#include "Win32Paths.h"

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#include <bcrypt.h>

#include <algorithm>
#include <cctype>
#include <cstdio>
#include <fstream>
#include <sstream>

namespace wkopenvr::questapp {
namespace {

std::wstring ConfigPath()
{
	const std::wstring dir = openvr_pair::common::WkOpenVrSubdirectoryPath(L"profiles", true);
	if (dir.empty()) return {};
	return dir + L"\\questapp.txt";
}

bool ParseBool(const std::string& value)
{
	return value == "1" || value == "true" || value == "on";
}

void WritePair(std::ostream& out, const char* key, const std::string& value)
{
	out << key << "=" << value << "\n";
}

} // namespace

QuestAppConfig LoadQuestAppConfig()
{
	const std::wstring path = ConfigPath();
	if (path.empty()) return {};
	std::ifstream in(path);
	if (!in) return {};
	return ParseQuestAppConfig(in);
}

void SaveQuestAppConfig(const QuestAppConfig& cfg)
{
	const std::wstring path = ConfigPath();
	if (path.empty()) return;
	std::ofstream out(path, std::ios::trunc);
	if (!out) return;
	WriteQuestAppConfig(cfg, out);
}

QuestAppConfig ParseQuestAppConfig(std::istream& in)
{
	QuestAppConfig cfg;
	std::string line;
	while (std::getline(in, line)) {
		while (!line.empty() && (line.back() == '\r' || line.back() == '\n')) {
			line.pop_back();
		}
		const size_t eq = line.find('=');
		if (eq == std::string::npos) continue;
		const std::string key = line.substr(0, eq);
		const std::string value = line.substr(eq + 1);

		if (key == "pairing_key")
			cfg.pairingKey = value;
		else if (key == "paired_device_serial")
			cfg.pairedDeviceSerial = value;
		else if (key == "companion_host")
			cfg.companionHost = value;
		else if (key == "companion_installed")
			cfg.companionInstalled = ParseBool(value);
	}
	if (!IsValidPairingKey(cfg.pairingKey)) cfg.pairingKey.clear();
	return cfg;
}

void WriteQuestAppConfig(const QuestAppConfig& cfg, std::ostream& out)
{
	WritePair(out, "pairing_key", cfg.pairingKey);
	WritePair(out, "paired_device_serial", cfg.pairedDeviceSerial);
	WritePair(out, "companion_host", cfg.companionHost);
	out << "companion_installed=" << (cfg.companionInstalled ? 1 : 0) << "\n";
}

std::string GeneratePairingKey()
{
	unsigned char bytes[32] = {};
	if (BCryptGenRandom(nullptr, bytes, static_cast<ULONG>(sizeof(bytes)), BCRYPT_USE_SYSTEM_PREFERRED_RNG) != 0) {
		LARGE_INTEGER counter{};
		QueryPerformanceCounter(&counter);
		ULONGLONG seed = GetTickCount64() ^ static_cast<ULONGLONG>(counter.QuadPart);
		for (size_t i = 0; i < sizeof(bytes); ++i) {
			seed = seed * 6364136223846793005ull + 1442695040888963407ull;
			bytes[i] = static_cast<unsigned char>((seed >> 32) & 0xFF);
		}
	}

	static constexpr char kHex[] = "0123456789abcdef";
	std::string out;
	out.reserve(sizeof(bytes) * 2);
	for (unsigned char b : bytes) {
		out.push_back(kHex[(b >> 4) & 0x0F]);
		out.push_back(kHex[b & 0x0F]);
	}
	return out;
}

bool IsValidPairingKey(const std::string& key)
{
	if (key.size() != 64) return false;
	return std::all_of(key.begin(), key.end(),
	                   [](char c) { return std::isxdigit(static_cast<unsigned char>(c)) != 0; });
}

bool CanContactCompanion(const QuestAppConfig& cfg)
{
	return cfg.companionInstalled && IsValidPairingKey(cfg.pairingKey);
}

bool NeedsCompanionReinstall(const QuestAppConfig& cfg)
{
	return cfg.companionInstalled && !IsValidPairingKey(cfg.pairingKey);
}

} // namespace wkopenvr::questapp
