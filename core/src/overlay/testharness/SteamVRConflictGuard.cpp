#include "SteamVRConflictGuard.h"

#if WKOPENVR_BUILD_IS_DEV

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#include <tlhelp32.h>

#include <cstdio>
#include <cwchar>
#include <string>
#include <string_view>
#include <vector>

namespace openvr_pair::overlay::testharness {

namespace {

bool IEqualsAscii(std::wstring_view a, std::wstring_view b) noexcept {
	if (a.size() != b.size()) return false;
	for (size_t i = 0; i < a.size(); ++i) {
		const auto lhs = (wchar_t)::towlower(a[i]);
		const auto rhs = (wchar_t)::towlower(b[i]);
		if (lhs != rhs) return false;
	}
	return true;
}

struct VrServerHit {
	DWORD pid = 0;
};

bool FindVrServer(VrServerHit &out) {
	HANDLE snap = ::CreateToolhelp32Snapshot(TH32CS_SNAPPROCESS, 0);
	if (snap == INVALID_HANDLE_VALUE) return false;
	PROCESSENTRY32W pe{};
	pe.dwSize = sizeof(pe);
	bool found = false;
	if (::Process32FirstW(snap, &pe)) {
		do {
			if (IEqualsAscii(pe.szExeFile, L"vrserver.exe")) {
				out.pid = pe.th32ProcessID;
				found = true;
				break;
			}
		} while (::Process32NextW(snap, &pe));
	}
	::CloseHandle(snap);
	return found;
}

// Returns the first shmem segment name we manage to open, indicating that
// some process is already publishing into it. Empty string means clean.
std::string DetectLiveShmem() {
	static const char *kSegments[] = {
		"WKOpenVRPoseMemoryV2",
		"WKOpenVRInputHealthMemoryV1",
		"WKOpenVRFaceTrackingFrameRingV2",
		"WKOpenVROscRouterStatsV1",
		"WKOpenVRPhantomStateV2",
	};
	for (const char *name : kSegments) {
		HANDLE h = ::OpenFileMappingA(FILE_MAP_READ, FALSE, name);
		if (h != nullptr) {
			::CloseHandle(h);
			return name;
		}
	}
	return {};
}

} // namespace

SteamVRConflictResult CheckSteamVRConflicts() {
	SteamVRConflictResult result;

	VrServerHit hit{};
	if (FindVrServer(hit)) {
		char buf[256];
		std::snprintf(buf, sizeof(buf),
			"refusing to run --test-harness: SteamVR (vrserver.exe pid=%lu) is alive; close SteamVR first",
			(unsigned long)hit.pid);
		result.ok = false;
		result.error = buf;
		return result;
	}

	const std::string live = DetectLiveShmem();
	if (!live.empty()) {
		result.ok = false;
		result.error = "refusing to run --test-harness: shmem segment '" + live +
			"' is already mapped; close any running WKOpenVR driver instance and retry";
		return result;
	}

	return result;
}

} // namespace openvr_pair::overlay::testharness

#endif // WKOPENVR_BUILD_IS_DEV
