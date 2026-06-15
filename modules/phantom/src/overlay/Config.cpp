#include "Config.h"

#include "Win32Paths.h"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

namespace {

std::wstring ConfigDir()
{
	return openvr_pair::common::WkOpenVrSubdirectoryPath(L"profiles", true);
}

std::wstring ConfigPath()
{
	const std::wstring dir = ConfigDir();
	if (dir.empty()) return {};
	return dir + L"\\phantom.txt";
}

uint32_t ParseMsClamped(const char* val, uint32_t lo, uint32_t hi)
{
	const long n = std::strtol(val, nullptr, 10);
	if (n < (long)lo) return lo;
	if (n > (long)hi) return hi;
	return static_cast<uint32_t>(n);
}

} // namespace

PhantomConfig LoadPhantomConfig()
{
	PhantomConfig cfg;
	const std::wstring path = ConfigPath();
	if (path.empty()) return cfg;

	FILE* f = _wfopen(path.c_str(), L"r");
	if (!f) return cfg;

	char line[512];
	while (fgets(line, sizeof(line), f)) {
		size_t len = std::strlen(line);
		while (len > 0 && (line[len - 1] == '\n' || line[len - 1] == '\r')) {
			line[--len] = '\0';
		}
		char* eq = std::strchr(line, '=');
		if (!eq) continue;
		*eq = '\0';
		const char* key = line;
		const char* val = eq + 1;

		if (std::strcmp(key, "master_enabled") == 0) {
			cfg.master_enabled = (std::atoi(val) != 0);
		}
		else if (std::strcmp(key, "auto_accept_roles") == 0) {
			cfg.auto_accept_roles = (std::atoi(val) != 0);
		}
		else if (std::strcmp(key, "solver.virtual_min_confidence") == 0) {
			cfg.solver.virtual_min_confidence = std::strtod(val, nullptr);
		}
		else if (std::strncmp(key, "solver.", 7) == 0) {
			// Legacy body-prior fields are now estimated in the driver.
			continue;
		}
		else if (std::strcmp(key, "blend_out_ms") == 0) {
			cfg.blend_out_ms = ParseMsClamped(val, 0, 1000);
		}
		else if (std::strcmp(key, "blend_in_ms") == 0) {
			cfg.blend_in_ms = ParseMsClamped(val, 0, 2000);
		}
		else if (std::strcmp(key, "reckon_hold_ms") == 0) {
			cfg.reckon_hold_ms = ParseMsClamped(val, 0, 1000);
		}
		else if (std::strcmp(key, "synth_hold_ms") == 0) {
			cfg.synth_hold_ms = ParseMsClamped(val, 0, 10000);
		}
		else if (std::strcmp(key, "lost_hold_ms") == 0) {
			cfg.lost_hold_ms = ParseMsClamped(val, 0, 60000);
		}
		else if (std::strncmp(key, "dropout_enabled.", 16) == 0) {
			const std::string serial(key + 16);
			cfg.dropout_enabled[serial] = (std::atoi(val) != 0);
		}
		else if (std::strncmp(key, "device_role_manual.", 19) == 0) {
			const std::string serial(key + 19);
			cfg.role_manual[serial] = (std::atoi(val) != 0);
		}
		else if (std::strncmp(key, "device_role.", 12) == 0) {
			const std::string serial(key + 12);
			const phantom::BodyRole r = phantom::BodyRoleFromKey(val);
			if (r != phantom::BodyRole::None) cfg.device_role[serial] = r;
		}
		else if (std::strncmp(key, "virtual_enabled.", 16) == 0) {
			const phantom::BodyRole r = phantom::BodyRoleFromKey(key + 16);
			if (r != phantom::BodyRole::None) {
				cfg.virtual_enabled[r] = (std::atoi(val) != 0);
			}
		}
		else if (std::strncmp(key, "role_offset.", 12) == 0) {
			// Legacy rigid role offsets are no longer used.
			continue;
		}
	}
	std::fclose(f);
	return cfg;
}

void SavePhantomConfig(const PhantomConfig& cfg)
{
	const std::wstring path = ConfigPath();
	if (path.empty()) return;

	FILE* f = _wfopen(path.c_str(), L"w");
	if (!f) return;
	std::fprintf(f, "master_enabled=%d\n", cfg.master_enabled ? 1 : 0);
	std::fprintf(f, "auto_accept_roles=%d\n", cfg.auto_accept_roles ? 1 : 0);
	std::fprintf(f, "blend_out_ms=%u\n", (unsigned)cfg.blend_out_ms);
	std::fprintf(f, "blend_in_ms=%u\n", (unsigned)cfg.blend_in_ms);
	std::fprintf(f, "reckon_hold_ms=%u\n", (unsigned)cfg.reckon_hold_ms);
	std::fprintf(f, "synth_hold_ms=%u\n", (unsigned)cfg.synth_hold_ms);
	std::fprintf(f, "lost_hold_ms=%u\n", (unsigned)cfg.lost_hold_ms);
	std::fprintf(f, "solver.virtual_min_confidence=%.6f\n", cfg.solver.virtual_min_confidence);
	for (const auto& kv : cfg.dropout_enabled) {
		std::fprintf(f, "dropout_enabled.%s=%d\n", kv.first.c_str(), kv.second ? 1 : 0);
	}
	for (const auto& kv : cfg.device_role) {
		if (kv.second != phantom::BodyRole::None) {
			std::fprintf(f, "device_role.%s=%s\n", kv.first.c_str(), phantom::BodyRoleToKey(kv.second));
		}
	}
	for (const auto& kv : cfg.role_manual) {
		// Only the manual flag is persisted; an automatic entry is the default
		// for any device_role without a matching device_role_manual line.
		if (kv.second) {
			std::fprintf(f, "device_role_manual.%s=1\n", kv.first.c_str());
		}
	}
	for (const auto& kv : cfg.virtual_enabled) {
		if (kv.second) {
			std::fprintf(f, "virtual_enabled.%s=1\n", phantom::BodyRoleToKey(kv.first));
		}
	}
	std::fclose(f);
}
