#define _CRT_SECURE_NO_DEPRECATE
#include "Config.h"

#include "Win32Paths.h"

#include <cstdio>
#include <cstring>
#include <string>

namespace {

std::wstring ConfigPath()
{
	const std::wstring dir = openvr_pair::common::WkOpenVrSubdirectoryPath(L"profiles", true);
	if (dir.empty()) return {};
	return dir + L"\\inputhealth.txt";
}

void WriteConfigFile(const std::wstring& path, const InputHealthGlobalConfig& cfg)
{
	FILE* f = _wfopen(path.c_str(), L"w");
	if (!f) return;
	fprintf(f, "master_enabled=%d\n", cfg.master_enabled ? 1 : 0);
	fprintf(f, "diagnostics_only=%d\n", cfg.diagnostics_only ? 1 : 0);
	fprintf(f, "enable_rest_recenter=%d\n", cfg.enable_rest_recenter ? 1 : 0);
	fprintf(f, "enable_trigger_remap=%d\n", cfg.enable_trigger_remap ? 1 : 0);
	fprintf(f, "defaults_v2_migrated=%d\n", cfg.defaults_v2_migrated ? 1 : 0);
	fclose(f);
}

} // namespace

InputHealthGlobalConfig LoadInputHealthConfig()
{
	InputHealthGlobalConfig cfg;
	const std::wstring path = ConfigPath();
	if (path.empty()) return cfg;

	FILE* f = _wfopen(path.c_str(), L"r");
	if (!f) return cfg;

	bool hasDefaultsMarker = false;
	char line[128];
	while (fgets(line, sizeof line, f)) {
		size_t len = strlen(line);
		while (len > 0 && (line[len - 1] == '\n' || line[len - 1] == '\r')) {
			line[--len] = '\0';
		}
		char* eq = strchr(line, '=');
		if (!eq) continue;
		*eq = '\0';
		const char* key = line;
		const char* val = eq + 1;

		const int n = atoi(val);
		if (strcmp(key, "master_enabled") == 0) {
			cfg.master_enabled = (n != 0);
		}
		else if (strcmp(key, "diagnostics_only") == 0) {
			cfg.diagnostics_only = (n != 0);
		}
		else if (strcmp(key, "enable_rest_recenter") == 0) {
			cfg.enable_rest_recenter = (n != 0);
		}
		else if (strcmp(key, "enable_trigger_remap") == 0) {
			cfg.enable_trigger_remap = (n != 0);
		}
		else if (strcmp(key, "defaults_v2_migrated") == 0) {
			cfg.defaults_v2_migrated = (n != 0);
			hasDefaultsMarker = true;
		}
	}
	fclose(f);

	if (!hasDefaultsMarker || !cfg.defaults_v2_migrated) {
		cfg.master_enabled = true;
		cfg.diagnostics_only = false;
		cfg.enable_rest_recenter = true;
		cfg.enable_trigger_remap = true;
		cfg.defaults_v2_migrated = true;
		WriteConfigFile(path, cfg);
	}
	return cfg;
}

void SaveInputHealthConfig(const InputHealthGlobalConfig& cfg)
{
	const std::wstring path = ConfigPath();
	if (path.empty()) return;
	WriteConfigFile(path, cfg);
}
