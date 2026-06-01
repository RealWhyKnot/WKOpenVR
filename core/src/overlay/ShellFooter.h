#pragma once

namespace openvr_pair::overlay {

enum class ShellFooterConnectionState
{
	Connected,
	WaitingForSteamVR,
	Disconnected,
};

inline ShellFooterConnectionState ResolveShellFooterConnectionState(bool driverConnected, bool vrConnected)
{
	if (driverConnected) return ShellFooterConnectionState::Connected;
	if (!vrConnected) return ShellFooterConnectionState::WaitingForSteamVR;
	return ShellFooterConnectionState::Disconnected;
}

// Status snapshot a feature plugin passes to DrawShellFooter at the bottom of
// its tab body. Mirrors SC's ShowVersionLine layout (status dot + driver
// label + " | WKOpenVR <build>") in a simpler form -- one line, no mode
// pill, no hover hint -- so InputHealth and Smoothing get a consistent
// footer without duplicating SC's calibration-state-aware logic.
struct ShellFooterStatus
{
	// Driver pipe state for the calling plugin's own IPC client. Drives the
	// connected state when true; otherwise vrConnected decides whether the
	// footer says "waiting for SteamVR" or "disconnected".
	bool driverConnected = false;

	// Runtime connection state from the shell. When false, a missing driver
	// pipe is presented as "waiting for SteamVR" instead of a failure.
	bool vrConnected = false;

	// Human-readable name for the driver column, e.g. "Smoothing driver" or
	// "InputHealth driver". Used as "<driverLabel>: connected" /
	// "<driverLabel>: disconnected". Must not be null.
	const char *driverLabel = "Driver";

	// Per-build version stamp shown after "WKOpenVR". May be null; the
	// helper falls back to OPENVR_PAIR_VERSION_STRING when absent.
	const char *buildStamp = nullptr;
};

// Draws a single-row footer pinned to the bottom of the current child /
// window. Caller must already be inside the layout area it wants the footer
// in -- the helper does not push a child of its own.
void DrawShellFooter(const ShellFooterStatus &status);

} // namespace openvr_pair::overlay
