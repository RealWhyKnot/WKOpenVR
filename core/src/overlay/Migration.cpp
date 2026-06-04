#include "Migration.h"

#include "JsonUtil.h"
#include "Win32Paths.h"

#define WIN32_LEAN_AND_MEAN
#include <windows.h>

#include <cstdio>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>

namespace openvr_pair::overlay {

namespace {

// Step 1: copy %LocalAppDataLow%\OpenVR-Pair\ -> %LocalAppDataLow%\WKOpenVR\
// if the new dir does not exist.
static void MigrateAppData()
{
	std::wstring root = openvr_pair::common::LocalAppDataLowPath();
	if (root.empty()) {
		fprintf(stderr, "[Migration] Could not resolve %%LocalAppDataLow%%; skipping AppData migration\n");
		return;
	}

	namespace fs = std::filesystem;
	fs::path newDir = root + L"\\WKOpenVR";
	fs::path oldDir = root + L"\\OpenVR-Pair";

	// Short-circuit: new dir already exists (migration already done, or fresh install).
	if (fs::exists(newDir)) return;
	// Nothing to migrate if old dir also absent (first install).
	if (!fs::exists(oldDir)) return;

	fprintf(stderr, "[Migration] Migrating AppData from OpenVR-Pair to WKOpenVR...\n");

	std::error_code ec;
	fs::copy(oldDir, newDir, fs::copy_options::recursive | fs::copy_options::skip_existing, ec);

	if (ec) {
		fprintf(stderr, "[Migration] AppData copy failed: %s\n", ec.message().c_str());
		return;
	}

	// Count migrated files + total bytes for the log line.
	uintmax_t fileCount = 0;
	uintmax_t totalBytes = 0;
	for (const auto& entry : fs::recursive_directory_iterator(newDir, ec)) {
		if (!ec && entry.is_regular_file()) {
			++fileCount;
			std::error_code szEc;
			uintmax_t sz = fs::file_size(entry.path(), szEc);
			if (!szEc) totalBytes += sz;
		}
	}

	fprintf(stderr,
	        "[Migration] Migrated AppData from OpenVR-Pair to WKOpenVR "
	        "(%llu files, %llu bytes)\n",
	        (unsigned long long)fileCount, (unsigned long long)totalBytes);
}

// Step 2: copy HKCU\Software\OpenVR-WKSpaceCalibrator ->
//         HKCU\Software\WKOpenVR-SpaceCalibrator if the new key does not exist.
static void MigrateScRegistryKey()
{
	const wchar_t* kOldKey = L"Software\\OpenVR-WKSpaceCalibrator";
	const wchar_t* kNewKey = L"Software\\WKOpenVR-SpaceCalibrator";

	// Check if new key already exists -- if so, nothing to do.
	HKEY hNew = nullptr;
	LSTATUS st = RegOpenKeyExW(HKEY_CURRENT_USER, kNewKey, 0, KEY_READ, &hNew);
	if (st == ERROR_SUCCESS) {
		RegCloseKey(hNew);
		return; // already migrated
	}

	// Check if old key exists -- if not, nothing to migrate.
	HKEY hOld = nullptr;
	st = RegOpenKeyExW(HKEY_CURRENT_USER, kOldKey, 0, KEY_READ, &hOld);
	if (st != ERROR_SUCCESS) {
		return; // old key absent, first install
	}
	RegCloseKey(hOld);

	fprintf(stderr, "[Migration] Copying registry key HKCU\\%ls -> HKCU\\%ls\n", kOldKey, kNewKey);

	// Create the destination key then copy the tree.
	HKEY hDst = nullptr;
	DWORD disp = 0;
	st = RegCreateKeyExW(HKEY_CURRENT_USER, kNewKey, 0, nullptr, REG_OPTION_NON_VOLATILE, KEY_ALL_ACCESS, nullptr,
	                     &hDst, &disp);
	if (st != ERROR_SUCCESS) {
		fprintf(stderr, "[Migration] RegCreateKeyExW failed: %ld\n", (long)st);
		return;
	}

	// Open old key with full read access for the copy.
	HKEY hSrc = nullptr;
	st = RegOpenKeyExW(HKEY_CURRENT_USER, kOldKey, 0, KEY_READ, &hSrc);
	if (st != ERROR_SUCCESS) {
		RegCloseKey(hDst);
		fprintf(stderr, "[Migration] Could not open old registry key for copy: %ld\n", (long)st);
		return;
	}

	st = RegCopyTreeW(hSrc, nullptr, hDst);
	RegCloseKey(hSrc);
	RegCloseKey(hDst);

	if (st == ERROR_SUCCESS) {
		fprintf(stderr,
		        "[Migration] SC registry key copied from OpenVR-WKSpaceCalibrator to WKOpenVR-SpaceCalibrator\n");
	}
	else {
		fprintf(stderr, "[Migration] RegCopyTreeW failed: %ld (new key created but may be empty)\n", (long)st);
	}
}

// Step 3: if the facetracking profile has a non-default osc_port, copy it
// into the oscrouter profile as send_port.  Idempotent: once the sentinel
// key "osc_migrated_to_router" is written into facetracking.json the step
// is skipped entirely on every subsequent launch.
//
// The router driver hard-codes 127.0.0.1:9000 as the send endpoint; the
// port written here is the value users saw before PR #3 (stored under
// "osc_port" in facetracking.json).  If the router profile already has a
// non-default send_port the user has already configured it -- skip.
//
// Note: writing oscrouter.json here only persists the preference.  The
// router overlay reads this file on startup and displays it; a future
// RequestOscRouterSetConfig IPC call will push the value into the running
// driver (deferred, no protocol bump needed here).
static void MigrateFtOscPort()
{
	std::wstring profileDir = openvr_pair::common::WkOpenVrSubdirectoryPath(L"profiles", false);
	if (profileDir.empty()) return;

	std::wstring ftPath = profileDir + L"\\facetracking.json";
	std::wstring orPath = profileDir + L"\\oscrouter.json";

	// 1. Check idempotency sentinel in facetracking.json.
	{
		std::ifstream ftIn(ftPath);
		if (!ftIn) return; // no facetracking.json -- nothing to migrate
		std::stringstream ss;
		ss << ftIn.rdbuf();
		picojson::value root;
		if (!openvr_pair::common::json::ParseObject(root, ss.str())) return;
		if (openvr_pair::common::json::BoolAt(root, "osc_migrated_to_router", false)) {
			// Already migrated on a prior launch.
			return;
		}

		// 2. Read the old osc_port value; if it is default (9000) or absent, skip.
		int oldPort = openvr_pair::common::json::IntAt(root, "osc_port", 9000);
		if (oldPort == 9000 || oldPort <= 0 || oldPort > 65535) {
			// Nothing non-trivial to carry over; mark as done and return.
			// Fall through to write the sentinel even for the default case so
			// the check above short-circuits on the next launch.
		}
		else {
			// 3. Check the router profile -- if it already has a non-default port, skip.
			{
				std::ifstream orIn(orPath);
				if (orIn) {
					std::stringstream orSs;
					orSs << orIn.rdbuf();
					picojson::value orRoot;
					if (openvr_pair::common::json::ParseObject(orRoot, orSs.str())) {
						int existingPort = openvr_pair::common::json::IntAt(orRoot, "send_port", 9000);
						if (existingPort != 9000 && existingPort > 0 && existingPort <= 65535) {
							// User already configured the router port; don't clobber.
							fprintf(stderr,
							        "[Migration] oscrouter.json already has send_port=%d; skipping FT port migration\n",
							        existingPort);
							// Still write the sentinel so we don't re-evaluate next launch.
							goto write_sentinel;
						}
					}
				}
			}

			// 4. Write the FT port into the router profile.
			{
				picojson::object orObj;
				// Preserve any existing keys if the file exists.
				{
					std::ifstream orIn(orPath);
					if (orIn) {
						std::stringstream orSs;
						orSs << orIn.rdbuf();
						picojson::value orRoot;
						if (openvr_pair::common::json::ParseObject(orRoot, orSs.str())) {
							orObj = orRoot.get<picojson::object>();
						}
					}
				}
				orObj["send_port"] = picojson::value(static_cast<double>(oldPort));
				std::string body = picojson::value(orObj).serialize(true);

				std::wstring tmpPath = orPath + L".tmp";
				HANDLE h = CreateFileW(tmpPath.c_str(), GENERIC_WRITE, 0, nullptr, CREATE_ALWAYS, FILE_ATTRIBUTE_NORMAL,
				                       nullptr);
				if (h == INVALID_HANDLE_VALUE) {
					fprintf(stderr, "[Migration] Failed to write oscrouter.json.tmp\n");
				}
				else {
					DWORD written = 0;
					WriteFile(h, body.data(), static_cast<DWORD>(body.size()), &written, nullptr);
					CloseHandle(h);
					if (written == static_cast<DWORD>(body.size())) {
						MoveFileExW(tmpPath.c_str(), orPath.c_str(), MOVEFILE_REPLACE_EXISTING);
						fprintf(stderr, "[Migration] Migrated FT osc_port=%d into oscrouter.json send_port\n", oldPort);
					}
					else {
						DeleteFileW(tmpPath.c_str());
						fprintf(stderr, "[Migration] Partial write to oscrouter.json.tmp; not committed\n");
					}
				}
			}
		}
	}

write_sentinel:
	// 5. Write the idempotency sentinel into facetracking.json.
	//    We re-read and re-encode to avoid clobbering other keys.
	{
		std::ifstream ftIn2(ftPath);
		if (!ftIn2) return;
		std::stringstream ss2;
		ss2 << ftIn2.rdbuf();
		picojson::value root2;
		if (!openvr_pair::common::json::ParseObject(root2, ss2.str())) return;

		picojson::object ftObj = root2.get<picojson::object>();
		ftObj["osc_migrated_to_router"] = picojson::value(true);
		std::string body = picojson::value(ftObj).serialize(true);

		std::wstring tmpPath = ftPath + L".tmp";
		HANDLE h =
		    CreateFileW(tmpPath.c_str(), GENERIC_WRITE, 0, nullptr, CREATE_ALWAYS, FILE_ATTRIBUTE_NORMAL, nullptr);
		if (h == INVALID_HANDLE_VALUE) {
			fprintf(stderr, "[Migration] Failed to write facetracking.json.tmp for sentinel\n");
			return;
		}
		DWORD written = 0;
		WriteFile(h, body.data(), static_cast<DWORD>(body.size()), &written, nullptr);
		CloseHandle(h);
		if (written == static_cast<DWORD>(body.size())) {
			MoveFileExW(tmpPath.c_str(), ftPath.c_str(), MOVEFILE_REPLACE_EXISTING);
			fprintf(stderr, "[Migration] FT OSC port migration sentinel written\n");
		}
		else {
			DeleteFileW(tmpPath.c_str());
		}
	}
}

// Step 4: rename %LocalAppDataLow%\WKOpenVR\translator\ ->
//         %LocalAppDataLow%\WKOpenVR\captions\ on upgrade. The translator
// subtree holds downloaded whisper.cpp model packs (often 300 MB+) that we
// must not force the user to re-download. fs::rename is same-filesystem,
// instant, and preserves host_status.json plus any in-flight pack-extract
// state. Idempotent: short-circuits when the new dir already exists.
static void MigrateTranslatorToCaptions()
{
	std::wstring root = openvr_pair::common::LocalAppDataLowPath();
	if (root.empty()) return;

	namespace fs = std::filesystem;
	fs::path oldDir = fs::path(root) / L"WKOpenVR" / L"translator";
	fs::path newDir = fs::path(root) / L"WKOpenVR" / L"captions";

	if (!fs::exists(oldDir)) return; // nothing to migrate (fresh install)
	if (fs::exists(newDir)) return;  // already migrated

	fprintf(stderr, "[Migration] Renaming %%LocalAppDataLow%%\\WKOpenVR\\translator -> "
	                "%%LocalAppDataLow%%\\WKOpenVR\\captions\n");

	std::error_code ec;
	fs::rename(oldDir, newDir, ec);
	if (ec) {
		fprintf(stderr, "[Migration] Translator->Captions rename failed: %s\n", ec.message().c_str());
		return;
	}

	fprintf(stderr, "[Migration] Captions data migrated in place\n");
}

} // namespace

void RunFirstLaunchMigration()
{
	MigrateAppData();
	MigrateScRegistryKey();
	MigrateFtOscPort();
	MigrateTranslatorToCaptions();
}

} // namespace openvr_pair::overlay
