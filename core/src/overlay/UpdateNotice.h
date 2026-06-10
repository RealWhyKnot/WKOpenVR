#pragma once

#include <cstdint>
#include <string>
#include <string_view>
#include <vector>

// Shared "is there a newer release on GitHub" probe and installer queue.
//
// StartUpdateCheck() spawns a one-shot worker that hits
// the canonical module release endpoint, parses tag_name + html_url, and
// compares the remote stamp to the local OPENVR_PAIR_VERSION_STRING. The
// UI polls GetUpdateNoticeState() each frame and renders an update prompt
// when state.available is true.
//
// Replaces the calibration-tab UpdateChecker / Updater pair that pre-dated
// the monorepo migration. The shared queue downloads the matching module
// installer, verifies GitHub's SHA-256 digest and the release-note SHA-256
// line, then starts that NSIS installer after SteamVR has exited.

namespace openvr_pair::overlay {

enum class UpdateInstallPhase
{
	Idle,
	Downloading,
	Ready,
	Launching,
	Failed,
};

struct UpdateInstallState
{
	UpdateInstallPhase phase = UpdateInstallPhase::Idle;
	bool queuedForSteamVrExit = false;
	bool canQueue = false;
	int64_t bytesDownloaded = 0;
	int64_t totalBytes = 0;
	std::string targetTag;
	std::string targetVersion;
	std::string selectedModule;
	std::string assetName;
	std::string installerPath;
	std::string expectedSha256;
	std::string statusMessage;
	std::string errorMessage;
};

struct UpdateNoticeState
{
	bool available = false;     // True when remote stamp > local stamp.
	bool checkComplete = false; // False until the worker writes a result.
	std::string latestTag;      // e.g. "v2026.5.13.1"
	std::string latestVersion;  // tag with leading "v" stripped
	std::string releaseUrl;     // human-readable release page (html_url)
	std::string errorMessage;   // non-empty on failure
	UpdateInstallState install;
};

// Kick the single process-wide check. Safe to call multiple times; the
// worker thread runs once and subsequent calls are no-ops until the prior
// worker finishes. main.cpp invokes this at shell startup.
void StartUpdateCheck();

// Read the latest snapshot. ShellFooter polls this every frame to decide
// whether to render the indicator. Returns a default-constructed
// (checkComplete=false) state until the worker writes a result.
UpdateNoticeState GetUpdateNoticeState();

// Queue an installer for the next SteamVR shutdown. installedFlags should
// contain the enabled module flags visible to the shell; the queue selects
// the matching module repo so an update does not enable a new public module.
// If no public module flag is present, it falls back to Space Calibrator.
bool QueueUpdateForSteamVrClose(const std::vector<std::string_view>& installedFlags, std::string* error = nullptr);

// Clears a queued update and removes the persisted queue marker. A partially
// downloaded installer is left in the updater cache and overwritten on retry.
void CancelQueuedUpdate();

// Called during shutdown after the OpenVR session has been closed. Starts a
// hidden helper that waits for this process and SteamVR processes to exit,
// then launches the verified NSIS installer with elevation.
bool LaunchQueuedUpdateAfterProcessExit(uint32_t currentProcessId, std::string* error = nullptr);

} // namespace openvr_pair::overlay
