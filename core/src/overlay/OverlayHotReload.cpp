#include "OverlayHotReload.h"

#include "BuildChannel.h"
#include "DiagnosticsLog.h"

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#include <shellapi.h>

namespace openvr_pair::overlay {

#if WKOPENVR_BUILD_IS_DEV

namespace {

std::wstring SelfExePath()
{
	wchar_t buf[MAX_PATH];
	DWORD len = GetModuleFileNameW(nullptr, buf, MAX_PATH);
	if (len == 0 || len >= MAX_PATH) return {};
	return std::wstring(buf, len);
}

bool FileExists(const std::wstring& path)
{
	if (path.empty()) return false;
	DWORD attr = GetFileAttributesW(path.c_str());
	return attr != INVALID_FILE_ATTRIBUTES && !(attr & FILE_ATTRIBUTE_DIRECTORY);
}

} // namespace

void CleanupStaleOverlayBackup()
{
	const OverlayReloadPaths p = DeriveOverlayReloadPaths(SelfExePath());
	if (p.backup.empty() || !FileExists(p.backup)) return;
	if (DeleteFileW(p.backup.c_str())) {
		openvr_pair::common::DiagnosticLog("overlay", "hot-reload: removed stale backup exe");
	}
	// If it's still locked (rare: prior instance not fully exited), leave it; the
	// next swap replaces it via MOVEFILE_REPLACE_EXISTING.
}

bool MaybeRelaunchStagedOverlay()
{
	// Throttle to ~1 Hz; called every frame.
	static uint64_t lastCheckMs = 0;
	const uint64_t nowMs = GetTickCount64();
	if (lastCheckMs != 0 && nowMs - lastCheckMs < 1000) return false;
	lastCheckMs = nowMs;

	const OverlayReloadPaths p = DeriveOverlayReloadPaths(SelfExePath());
	if (p.staged.empty() || !FileExists(p.staged)) return false;

	openvr_pair::common::DiagnosticLog("overlay", "hot-reload: staged build detected; swapping in");

	// Best-effort clear of a prior backup so the rename below has a clean slot.
	if (FileExists(p.backup)) DeleteFileW(p.backup.c_str());

	// Rename the running exe aside (allowed while running), then move the staged
	// build into the canonical name. If the first rename fails there is nothing
	// to roll back; if the second fails, restore the original so we don't leave a
	// missing canonical exe behind.
	if (!MoveFileExW(p.canonical.c_str(), p.backup.c_str(), MOVEFILE_REPLACE_EXISTING)) {
		openvr_pair::common::DiagnosticLog("overlay", "hot-reload: rename-aside failed err=%lu; aborting swap",
		                                   GetLastError());
		return false;
	}
	if (!MoveFileExW(p.staged.c_str(), p.canonical.c_str(), MOVEFILE_REPLACE_EXISTING)) {
		const DWORD err = GetLastError();
		MoveFileExW(p.backup.c_str(), p.canonical.c_str(), MOVEFILE_REPLACE_EXISTING); // roll back
		openvr_pair::common::DiagnosticLog("overlay", "hot-reload: move-into-place failed err=%lu; rolled back", err);
		return false;
	}

	// Launch the new exe via the shell so it does not inherit this process's
	// handles. It is parented to this overlay (not the deploy script), preserving
	// the original session lineage; when we exit next it simply becomes orphaned.
	SHELLEXECUTEINFOW sei{};
	sei.cbSize = sizeof(sei);
	sei.fMask = SEE_MASK_NOASYNC | SEE_MASK_FLAG_NO_UI;
	sei.lpVerb = L"open";
	sei.lpFile = p.canonical.c_str();
	sei.nShow = SW_SHOWNORMAL;
	if (!ShellExecuteExW(&sei)) {
		openvr_pair::common::DiagnosticLog("overlay", "hot-reload: relaunch failed err=%lu; staying up on new binary",
		                                   GetLastError());
		return false; // swap already applied; keep this instance alive
	}

	openvr_pair::common::DiagnosticLog("overlay", "hot-reload: new overlay launched; exiting");
	return true;
}

#else // release: no self-relaunch machinery compiled in

void CleanupStaleOverlayBackup() {}
bool MaybeRelaunchStagedOverlay()
{
	return false;
}

#endif

} // namespace openvr_pair::overlay
