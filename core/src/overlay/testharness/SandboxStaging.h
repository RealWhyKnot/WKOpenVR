#pragma once

#include "BuildChannel.h"

#if WKOPENVR_BUILD_IS_DEV

#include <filesystem>
#include <string>
#include <vector>

namespace openvr_pair::overlay::testharness {

struct SandboxLayout
{
	std::filesystem::path root;             // %TEMP%\WKOpenVR-TestHarness-<pid>
	std::filesystem::path driver_root;      // root\drivers\01wkopenvr
	std::filesystem::path driver_dll;       // driver_root\bin\win64\driver_wkopenvr.dll
	std::filesystem::path driver_resources; // driver_root\resources
};

struct StageOptions
{
	std::vector<std::string> features; // slugs to enable via enable_<slug>.flag
	bool include_facetracking_host = false;
	bool include_captions_host = false;
	bool include_phantom_sidecar = false;
};

class SandboxStaging
{
public:
	SandboxStaging() = default;
	~SandboxStaging();

	SandboxStaging(const SandboxStaging&) = delete;
	SandboxStaging& operator=(const SandboxStaging&) = delete;

	// Build the sandbox tree. Throws std::runtime_error on any I/O failure.
	// On success the SandboxLayout returned is also stored internally and
	// destroyed (recursively deleted) when keep_on_destruct() is left false.
	SandboxLayout Stage(const StageOptions& options);

	// Disable destructor cleanup so the directory persists for post-mortem
	// inspection. The orchestrator sets this when --keep-sandbox is on.
	void keep_on_destruct() noexcept { keep_ = true; }

	const SandboxLayout& layout() const noexcept { return layout_; }

private:
	SandboxLayout layout_;
	bool keep_ = false;
	bool staged_ = false;
};

// Locate the build's driver tree (build/driver_wkopenvr/) by walking up from
// the running exe path. Returns the resolved path or empty path on failure.
std::filesystem::path LocateBuildDriverTree();

} // namespace openvr_pair::overlay::testharness

#endif // WKOPENVR_BUILD_IS_DEV
