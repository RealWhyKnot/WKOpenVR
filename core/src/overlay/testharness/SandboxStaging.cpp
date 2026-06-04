#include "SandboxStaging.h"

#if WKOPENVR_BUILD_IS_DEV

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>

#include <cstdio>
#include <fstream>
#include <stdexcept>
#include <string>
#include <system_error>

namespace fs = std::filesystem;

namespace openvr_pair::overlay::testharness {

namespace {

fs::path ExeDir()
{
	wchar_t buf[MAX_PATH * 4]{};
	const DWORD n = ::GetModuleFileNameW(nullptr, buf, (DWORD)(sizeof(buf) / sizeof(buf[0])));
	if (n == 0) {
		throw std::runtime_error("GetModuleFileNameW failed");
	}
	fs::path p(buf);
	return p.parent_path();
}

fs::path TempBase()
{
	wchar_t buf[MAX_PATH + 1]{};
	const DWORD n = ::GetTempPathW((DWORD)(sizeof(buf) / sizeof(buf[0])), buf);
	if (n == 0 || n > MAX_PATH) {
		throw std::runtime_error("GetTempPathW failed");
	}
	return fs::path(buf);
}

void CopyDirRecursive(const fs::path& src, const fs::path& dst)
{
	std::error_code ec;
	fs::create_directories(dst, ec);
	if (ec) {
		throw std::runtime_error("create_directories failed for " + dst.string() + ": " + ec.message());
	}
	fs::copy(src, dst,
	         fs::copy_options::recursive | fs::copy_options::overwrite_existing | fs::copy_options::skip_symlinks, ec);
	if (ec) {
		throw std::runtime_error("copy_directory failed " + src.string() + " -> " + dst.string() + ": " + ec.message());
	}
}

void TouchEmptyFile(const fs::path& p)
{
	std::error_code ec;
	fs::create_directories(p.parent_path(), ec);
	std::ofstream out(p, std::ios::binary | std::ios::trunc);
	if (!out) {
		throw std::runtime_error("failed to write " + p.string());
	}
}

} // namespace

fs::path LocateBuildDriverTree()
{
	const fs::path exeDir = ExeDir();
	// WKOpenVR.exe lands at build/artifacts/Release/ during dev builds.
	// Driver tree is at build/driver_wkopenvr/ -- siblings two levels up.
	const std::vector<fs::path> candidates{
	    exeDir / ".." / ".." / "driver_wkopenvr",
	    exeDir / ".." / "driver_wkopenvr",
	    exeDir / "driver_wkopenvr",
	};
	for (const auto& c : candidates) {
		std::error_code ec;
		const fs::path dll = c / "bin" / "win64" / "driver_wkopenvr.dll";
		if (fs::exists(dll, ec)) {
			return fs::weakly_canonical(c, ec);
		}
	}
	return {};
}

SandboxStaging::~SandboxStaging()
{
	if (!staged_ || keep_) return;
	std::error_code ec;
	fs::remove_all(layout_.root, ec);
	if (ec) {
		// Best-effort. The orchestrator logs the leak path.
		std::fprintf(stderr, "[testharness] sandbox cleanup left %s behind: %s\n", layout_.root.string().c_str(),
		             ec.message().c_str());
	}
}

SandboxLayout SandboxStaging::Stage(const StageOptions& options)
{
	if (staged_) {
		throw std::runtime_error("SandboxStaging::Stage called twice on the same instance");
	}

	const fs::path buildDriver = LocateBuildDriverTree();
	if (buildDriver.empty()) {
		throw std::runtime_error("failed to locate build/driver_wkopenvr/ relative to the umbrella exe; "
		                         "build the project first");
	}

	const auto pid = ::GetCurrentProcessId();
	wchar_t pidBuf[32]{};
	::swprintf_s(pidBuf, L"WKOpenVR-TestHarness-%lu", (unsigned long)pid);

	layout_.root = TempBase() / pidBuf;
	layout_.driver_root = layout_.root / L"drivers" / L"01wkopenvr";
	layout_.driver_dll = layout_.driver_root / L"bin" / L"win64" / L"driver_wkopenvr.dll";
	layout_.driver_resources = layout_.driver_root / L"resources";

	std::error_code ec;
	fs::remove_all(layout_.root, ec); // tolerate stale leftover from a prior aborted run
	fs::create_directories(layout_.driver_root, ec);
	if (ec) {
		throw std::runtime_error("create_directories on " + layout_.driver_root.string() + " failed: " + ec.message());
	}

	CopyDirRecursive(buildDriver, layout_.driver_root);

	// Sanity check: the DLL must exist after the copy.
	if (!fs::exists(layout_.driver_dll, ec)) {
		throw std::runtime_error("post-stage driver DLL missing at " + layout_.driver_dll.string());
	}

	// Touch enable_<slug>.flag for each feature requested by the runner.
	for (const auto& slug : options.features) {
		const fs::path flag = layout_.driver_resources / ("enable_" + slug + ".flag");
		TouchEmptyFile(flag);
	}

	// Optional sidecar tree existence checks. These directories should already
	// have been copied by CopyDirRecursive if the build produced them; we just
	// log when they are missing so the scenarios can skip cleanly.
	auto warnIfMissing = [&](const wchar_t* label, const fs::path& p) {
		if (!fs::exists(p, ec)) {
			std::fprintf(stderr, "[testharness] sandbox missing sidecar tree %ls (%s)\n", label, p.string().c_str());
		}
	};
	if (options.include_facetracking_host) {
		warnIfMissing(L"facetracking host", layout_.driver_resources / L"facetracking" / L"host");
	}
	if (options.include_captions_host) {
		warnIfMissing(L"captions host", layout_.driver_resources / L"captions" / L"host");
	}
	if (options.include_phantom_sidecar) {
		warnIfMissing(L"phantom sidecar", layout_.driver_resources / L"phantom" / L"host");
	}

	staged_ = true;
	return layout_;
}

} // namespace openvr_pair::overlay::testharness

#endif // WKOPENVR_BUILD_IS_DEV
