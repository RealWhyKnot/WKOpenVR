#include "DebugLogging.h"

#include "BuildChannel.h"
#include "Win32Paths.h"

#include <chrono>
#include <mutex>

namespace openvr_pair::common {
namespace {

constexpr wchar_t kDebugFlagFileName[] = L"debug_logging.enabled";
constexpr auto kRefreshInterval = std::chrono::seconds(1);

std::mutex g_stateMutex;
bool g_cacheInitialized = false;
bool g_cachedEnabled = false;
std::chrono::steady_clock::time_point g_lastRefresh{};

bool EqualAsciiIgnoreCase(std::string_view left, std::string_view right)
{
	if (left.size() != right.size()) return false;
	for (size_t i = 0; i < left.size(); ++i) {
		char a = left[i];
		char b = right[i];
		if (a >= 'A' && a <= 'Z') a = static_cast<char>(a - 'A' + 'a');
		if (b >= 'A' && b <= 'Z') b = static_cast<char>(b - 'A' + 'a');
		if (a != b) return false;
	}
	return true;
}

bool DebugFlagExists()
{
	std::wstring path = DebugLoggingFlagPath(false);
	if (path.empty()) return false;
	DWORD attr = GetFileAttributesW(path.c_str());
	return attr != INVALID_FILE_ATTRIBUTES && !(attr & FILE_ATTRIBUTE_DIRECTORY);
}

void UpdateCache(bool enabled)
{
	g_cachedEnabled = enabled;
	g_cacheInitialized = true;
	g_lastRefresh = std::chrono::steady_clock::now();
}

} // namespace

bool BuildChannelForcesDebugLogging(std::string_view channel)
{
	return EqualAsciiIgnoreCase(channel, "dev");
}

bool IsDebugLoggingForcedOn()
{
	return BuildChannelForcesDebugLogging(WKOPENVR_BUILD_CHANNEL);
}

std::wstring DebugLoggingFlagPath(bool createRoot)
{
	std::wstring root = WkOpenVrRootPath(createRoot);
	if (root.empty()) return {};
	if (root.back() != L'\\' && root.back() != L'/') {
		root.push_back(L'\\');
	}
	root += kDebugFlagFileName;
	return root;
}

bool IsDebugLoggingEnabled()
{
	if (IsDebugLoggingForcedOn()) return true;

	std::lock_guard<std::mutex> lock(g_stateMutex);
	const auto now = std::chrono::steady_clock::now();
	if (!g_cacheInitialized || now - g_lastRefresh >= kRefreshInterval) {
		UpdateCache(DebugFlagExists());
	}
	return g_cachedEnabled;
}

bool SetDebugLoggingEnabled(bool enabled)
{
	if (IsDebugLoggingForcedOn()) {
		std::lock_guard<std::mutex> lock(g_stateMutex);
		UpdateCache(true);
		return true;
	}

	std::wstring path = DebugLoggingFlagPath(enabled);
	if (path.empty()) return false;

	bool ok = false;
	if (enabled) {
		HANDLE file = CreateFileW(path.c_str(), GENERIC_WRITE, FILE_SHARE_READ | FILE_SHARE_WRITE | FILE_SHARE_DELETE,
		                          nullptr, CREATE_ALWAYS, FILE_ATTRIBUTE_NORMAL, nullptr);
		if (file != INVALID_HANDLE_VALUE) {
			const char body[] = "1\n";
			DWORD written = 0;
			ok = WriteFile(file, body, static_cast<DWORD>(sizeof(body) - 1), &written, nullptr) != FALSE;
			CloseHandle(file);
		}
	}
	else {
		if (DeleteFileW(path.c_str()) != FALSE) {
			ok = true;
		}
		else {
			const DWORD err = GetLastError();
			ok = err == ERROR_FILE_NOT_FOUND || err == ERROR_PATH_NOT_FOUND;
		}
	}

	if (ok) {
		std::lock_guard<std::mutex> lock(g_stateMutex);
		UpdateCache(enabled);
	}
	return ok;
}

} // namespace openvr_pair::common
