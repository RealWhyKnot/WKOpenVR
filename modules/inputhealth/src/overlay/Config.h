#pragma once

// Persisted InputHealth overlay globals. Saved to
// %LocalAppDataLow%\WKOpenVR\profiles\inputhealth.txt as plain key=value
// lines. On first launch (or after a corrupt/missing file) all fields take
// the defaults listed here, which match the InputHealthPlugin constructor.

struct InputHealthGlobalConfig
{
	bool master_enabled = true;
	bool diagnostics_only = false;
	bool enable_rest_recenter = true;
	bool enable_trigger_remap = true;
	bool defaults_v2_migrated = true;
};

// Load from disk. On any read / parse error the on-disk file is ignored and
// a default-constructed InputHealthGlobalConfig is returned.
InputHealthGlobalConfig LoadInputHealthConfig();

// Save to disk. Best-effort: failures (locked file, missing dir) are silently
// swallowed. The driver gets the live value via IPC regardless.
void SaveInputHealthConfig(const InputHealthGlobalConfig& cfg);
