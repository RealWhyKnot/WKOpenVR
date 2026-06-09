#pragma once

#include <string>

// Shared "is there a newer release on GitHub" probe.
//
// StartUpdateCheck() spawns a one-shot worker that hits
// the canonical module release endpoint, parses tag_name + html_url, and
// compares the remote stamp to the local OPENVR_PAIR_VERSION_STRING. The
// UI polls GetUpdateNoticeState() each frame; the shared ShellFooter renders
// an indicator when state.available is true.
//
// Replaces the calibration-tab UpdateChecker / Updater pair that pre-dated
// the monorepo migration -- that one was scoped to one tab, pointed at
// the old calibration repo, and shipped a heavy download / install banner.
// The shared case is just a "you should grab a new build" notice; the user
// clicks through to a module release page and runs the installer themselves.

namespace openvr_pair::overlay {

struct UpdateNoticeState
{
	bool available = false;     // True when remote stamp > local stamp.
	bool checkComplete = false; // False until the worker writes a result.
	std::string latestTag;      // e.g. "v2026.5.13.1"
	std::string latestVersion;  // tag with leading "v" stripped
	std::string releaseUrl;     // human-readable release page (html_url)
	std::string errorMessage;   // non-empty on failure
};

// Kick the single process-wide check. Safe to call multiple times; the
// worker thread runs once and subsequent calls are no-ops until the prior
// worker finishes. main.cpp invokes this at shell startup.
void StartUpdateCheck();

// Read the latest snapshot. ShellFooter polls this every frame to decide
// whether to render the indicator. Returns a default-constructed
// (checkComplete=false) state until the worker writes a result.
UpdateNoticeState GetUpdateNoticeState();

} // namespace openvr_pair::overlay
