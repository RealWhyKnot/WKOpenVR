#pragma once

// ModuleSources.h -- manage sources.json + per-module source.json sidecars,
// scan the installed-modules directory, and drive async PowerShell sync runs.
//
// File layout (all under %LocalAppDataLow%\WKOpenVR\facetracking\):
//   sources.json                                -- source catalogue
//   modules/<uuid>/<version>/manifest.json      -- installed module manifest
//   modules/<uuid>/<version>/source.json        -- where the module came from

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>

#include <chrono>
#include <functional>
#include <optional>
#include <string>
#include <vector>

namespace facetracking {

// ---- source kinds -------------------------------------------------------

enum class SourceKind
{
	Registry,
	Folder,
	GitHub
};

struct ModuleSource
{
	std::string id; // stable UUID for this source entry
	SourceKind kind = SourceKind::Registry;
	std::string url;        // registry: base URL
	std::string path;       // folder: abs path
	std::string owner_repo; // github: "owner/repo"
	std::string label;      // display label
	bool auto_update = false;
	bool include_prerelease = false;
	std::string added_at;
	std::string last_checked_at;
	std::string last_release_tag;
	std::string last_sync_error; // empty = ok, set on sync failure
};

struct SourcesCatalogue
{
	int schema_version = 1;
	std::vector<ModuleSource> sources;
};

// ---- installed module (disk scan) ---------------------------------------

struct InstalledModule
{
	std::string uuid;
	std::string version;
	std::string name;
	std::string vendor;
	std::string manifest_path;   // full path to manifest.json
	std::string source_id;       // from source.json, empty if absent
	std::string source_kind_str; // "registry" | "folder" | "github" | ""
	bool sha_verified = false;
	std::string release_tag; // github only
};

struct AvailableModule
{
	std::string uuid;
	std::string version;
	std::string name;
	std::string vendor;
	std::string description;
	std::string source_id;
	std::string source_label;
	std::string registry_url;
	std::string payload_url;
	std::string payload_sha256;
	bool prerelease = false;
	std::string release_channel;
	std::string download_url;
	std::string file_hash;
	std::string dll_file_name;
	std::string module_page_url;
};

// ---- async sync result --------------------------------------------------

struct SyncResult
{
	bool ok = false;
	std::string message;
	std::string installed_uuid;
	std::string installed_version;
	std::string source_id;
	std::string action;
	int available_count = -1;
};

// ---- sources.json helpers -----------------------------------------------

// Resolve %LocalAppDataLow%\WKOpenVR\facetracking\ (ensures dirs exist).
std::wstring FtDataDir();

// Generate a random 128-bit hex ID (no dashes, lower case).
std::string GenerateSourceId();

// Current ISO-8601 timestamp (UTC, second precision).
std::string NowIso8601();

// Read sources.json; returns default-seeded catalogue if absent.
SourcesCatalogue LoadSourcesCatalogue();

// Write sources.json atomically.
bool SaveSourcesCatalogue(const SourcesCatalogue& cat);

// Ensure sources.json exists and has at least the built-in registry entry.
// Returns the (possibly newly created) catalogue.
SourcesCatalogue EnsureSourcesCatalogue();

// Look up a source label for display ("Legacy registry", "Folder: foo", ...).
std::string SourceLabel(const SourcesCatalogue& cat, const std::string& source_id);

// ---- disk-scan ----------------------------------------------------------

// Walk the modules dir and return all installed modules.  Does NOT require
// the host to be running; reads manifest.json + optional source.json.
std::vector<InstalledModule> ScanInstalledModules();

// Read cached registry module lists written by face-module-sync.ps1.
std::vector<AvailableModule> LoadAvailableModules(const std::string& source_id = {});

// ---- async sync runner --------------------------------------------------

class ModuleSyncRunner
{
public:
	ModuleSyncRunner();
	~ModuleSyncRunner();

	// Enqueue a sync operation.  Operations are serialised; later calls while
	// one is running are queued (max depth 8 -- extras are silently dropped).
	void StartAdd(SourceKind kind, const std::string& source_data_json, const std::string& source_id);
	void StartUpdate(const std::string& source_id, const std::string& source_data_json);
	void StartInstall(const std::string& source_id, const std::string& source_data_json);
	void StartRemove(const std::string& source_id);

	// Call each overlay frame.  Returns a finished SyncResult when one
	// becomes available; std::nullopt otherwise.
	std::optional<SyncResult> Poll();

	bool IsRunning() const;

	// Path to face-module-sync.ps1 (located next to the exe).
	static std::wstring ScriptPath();

private:
	struct PendingOp
	{
		std::string action; // "add" | "update" | "install" | "remove"
		std::string kind;   // "registry" | "folder" | "github" | ""
		std::string source_data;
		std::string source_id;
	};

	void LaunchNext();

	std::vector<PendingOp> queue_;
	PendingOp active_op_;
	std::optional<SyncResult> immediate_result_;
	HANDLE proc_ = INVALID_HANDLE_VALUE;
	std::wstring result_path_;
	std::chrono::steady_clock::time_point launch_time_{};
};

} // namespace facetracking
