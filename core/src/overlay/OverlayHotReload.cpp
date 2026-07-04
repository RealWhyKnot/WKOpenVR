#include "OverlayHotReload.h"

#include "BuildChannel.h"
#include "DiagnosticsLog.h"

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>

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

bool FileLooksLikeExecutable(const std::wstring& path)
{
	FILE* f = _wfopen(path.c_str(), L"rb");
	if (!f) return false;
	unsigned char header[2] = {};
	const bool ok = fread(header, 1, sizeof(header), f) == sizeof(header) && LooksLikePeImage(header, sizeof(header));
	fclose(f);
	return ok;
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

	// Reject a staged file that is not a PE image before touching anything;
	// see LooksLikePeImage for why such a file must never reach the launcher.
	if (!FileLooksLikeExecutable(p.staged)) {
		openvr_pair::common::DiagnosticLog("overlay", "hot-reload: staged file is not a valid executable; removing it");
		DeleteFileW(p.staged.c_str());
		return false;
	}

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

	// The relaunch happens in LaunchSwappedOverlayWithRecovery() after the
	// caller finishes its normal shutdown, so the new instance starts against a
	// released VR session instead of racing this one's teardown.
	openvr_pair::common::DiagnosticLog("overlay", "hot-reload: swap complete; shutting down before relaunch");
	return true;
}

namespace {

// Launch an overlay exe without handle inheritance. It is parented to this
// overlay (not the deploy script), preserving the original session lineage;
// once we exit it simply becomes orphaned. CreateProcessW rather than
// ShellExecuteEx: the shell path can block waiting on a message pump this
// thread no longer runs (it wedged on a non-PE file), while CreateProcessW
// fails fast with a plain error code. Returns the process handle (caller
// closes it); sets `launched` false when the launch failed.
HANDLE LaunchOverlayExe(const std::wstring& path, bool& launched)
{
	STARTUPINFOW si{};
	si.cb = sizeof(si);
	PROCESS_INFORMATION pi{};
	std::wstring cmd = L"\"" + path + L"\""; // CreateProcessW may modify this buffer
	launched = CreateProcessW(path.c_str(), cmd.data(), nullptr, nullptr, FALSE, CREATE_UNICODE_ENVIRONMENT, nullptr,
	                          nullptr, &si, &pi) != FALSE;
	if (!launched) return nullptr;
	CloseHandle(pi.hThread);
	return pi.hProcess;
}

} // namespace

void LaunchSwappedOverlayWithRecovery()
{
	// This process now runs from the renamed WKOpenVR.old.exe; the path
	// derivation only uses the directory, so the fixed names still resolve.
	const OverlayReloadPaths p = DeriveOverlayReloadPaths(SelfExePath());
	if (p.canonical.empty()) return;

	// Logged before the attempt so a wedged or crashed launch is attributable.
	openvr_pair::common::DiagnosticLog("overlay", "hot-reload: launching swapped build");
	bool launched = false;
	HANDLE hNew = LaunchOverlayExe(p.canonical, launched);

	// Give the new build a short grace window: a healthy overlay keeps running;
	// an exe that dies immediately (bad build, missing dependency) is caught
	// here instead of leaving the user with no overlay at all.
	bool aliveAfterGrace = launched;
	DWORD exitCode = 0;
	if (hNew) {
		constexpr DWORD kRelaunchGraceMs = 3000;
		aliveAfterGrace = WaitForSingleObject(hNew, kRelaunchGraceMs) == WAIT_TIMEOUT;
		if (!aliveAfterGrace) GetExitCodeProcess(hNew, &exitCode);
		CloseHandle(hNew);
	}

	if (!RelaunchNeedsRollback(launched, aliveAfterGrace)) {
		openvr_pair::common::DiagnosticLog("overlay", "hot-reload: new overlay running; exiting");
		return;
	}

	if (!launched) {
		openvr_pair::common::DiagnosticLog(
		    "overlay", "hot-reload: relaunch failed err=%lu; rolling back to previous build", GetLastError());
	}
	else {
		openvr_pair::common::DiagnosticLog(
		    "overlay", "hot-reload: new overlay exited immediately code=0x%08lx; rolling back to previous build",
		    static_cast<unsigned long>(exitCode));
	}

	// Restore the backup over the bad build (the failed process has exited, so
	// the canonical slot is free; renaming our own running image is allowed).
	if (!MoveFileExW(p.backup.c_str(), p.canonical.c_str(), MOVEFILE_REPLACE_EXISTING)) {
		openvr_pair::common::DiagnosticLog("overlay", "hot-reload: rollback restore failed err=%lu; relaunch manually",
		                                   GetLastError());
		return;
	}
	bool relaunched = false;
	if (HANDLE h = LaunchOverlayExe(p.canonical, relaunched)) CloseHandle(h);
	openvr_pair::common::DiagnosticLog("overlay", "hot-reload: previous build restored; relaunch %s",
	                                   relaunched ? "ok" : "failed -- launch WKOpenVR.exe manually");
}

#else // release: no self-relaunch machinery compiled in

void CleanupStaleOverlayBackup() {}
bool MaybeRelaunchStagedOverlay()
{
	return false;
}
void LaunchSwappedOverlayWithRecovery() {}

#endif

} // namespace openvr_pair::overlay
