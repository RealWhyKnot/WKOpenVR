#define _CRT_SECURE_NO_DEPRECATE
#include "Config.h"

#include "Win32Paths.h"

#include <cstdio>
#include <cstring>

namespace {

std::wstring ConfigPath()
{
	const std::wstring dir = openvr_pair::common::WkOpenVrSubdirectoryPath(L"profiles", true);
	if (dir.empty()) return {};
	return dir + L"\\inputhealth.txt";
}

} // namespace

InputHealthGlobalConfig LoadInputHealthConfig()
{
	InputHealthGlobalConfig cfg;
	const std::wstring path = ConfigPath();
	if (path.empty()) return cfg;

	FILE* f = _wfopen(path.c_str(), L"r");
	if (!f) return cfg;

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
	}
	fclose(f);
	return cfg;
}

void SaveInputHealthConfig(const InputHealthGlobalConfig& cfg)
{
	const std::wstring path = ConfigPath();
	if (path.empty()) return;

	FILE* f = _wfopen(path.c_str(), L"w");
	if (!f) return;
	fprintf(f, "master_enabled=%d\n", cfg.master_enabled ? 1 : 0);
	fprintf(f, "diagnostics_only=%d\n", cfg.diagnostics_only ? 1 : 0);
	fprintf(f, "enable_rest_recenter=%d\n", cfg.enable_rest_recenter ? 1 : 0);
	fprintf(f, "enable_trigger_remap=%d\n", cfg.enable_trigger_remap ? 1 : 0);
	fclose(f);
}
