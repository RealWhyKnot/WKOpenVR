#pragma once

#include <string>

// Dev-only overlay self-relaunch. The overlay (WKOpenVR.exe) is a separate
// process from vrserver, so a rebuilt overlay can take over WITHOUT restarting
// SteamVR -- you stay in VR. reload.ps1 -Overlay stages the freshly built exe
// next to the running one as WKOpenVR.new.exe (Windows won't overwrite a running
// image). The running overlay notices it, renames itself aside, moves the staged
// build into place, launches it, and exits.
//
// The relaunch is done BY the overlay (not by the deploy script) so the new
// process never inherits the build script's process ancestry -- see the project
// note that a script-parented overlay can disturb tracking.
namespace openvr_pair::overlay {

struct OverlayReloadPaths
{
	std::wstring canonical; // <dir>\WKOpenVR.exe   -- the deployed name
	std::wstring staged;    // <dir>\WKOpenVR.new.exe -- freshly built, staged
	std::wstring backup;    // <dir>\WKOpenVR.old.exe -- running exe renamed aside
};

// Pure: derive the staged/backup sibling paths from the running exe's full path.
// Uses the exe's own directory; the filenames are fixed. Returns empty fields
// when selfExePath has no directory separator. Header-inline so it is unit-
// testable without linking the Win32 action code below.
inline OverlayReloadPaths DeriveOverlayReloadPaths(const std::wstring& selfExePath)
{
	OverlayReloadPaths p;
	const size_t slash = selfExePath.find_last_of(L"\\/");
	if (slash == std::wstring::npos) return p;
	const std::wstring dir = selfExePath.substr(0, slash);
	p.canonical = dir + L"\\WKOpenVR.exe";
	p.staged = dir + L"\\WKOpenVR.new.exe";
	p.backup = dir + L"\\WKOpenVR.old.exe";
	return p;
}

// Dev-only. Delete a leftover WKOpenVR.old.exe from a prior self-swap (now
// unlocked because that process exited). No-op on release or when absent. Call
// once at startup.
void CleanupStaleOverlayBackup();

// Dev-only. If a staged WKOpenVR.new.exe exists next to us, swap it in and launch
// it, returning true to tell the caller to exit its main loop. Throttled
// internally so it is cheap to call every frame. Always returns false on release
// builds or when there is nothing staged.
bool MaybeRelaunchStagedOverlay();

} // namespace openvr_pair::overlay
