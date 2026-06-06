#include "Win32Paths.h"

#include <algorithm>
#include <cwchar>
#include <cwctype>
#include <filesystem>
#include <vector>

#include <objbase.h>
#include <shlobj.h>

namespace openvr_pair::common {
namespace {

std::wstring JoinPath(std::wstring_view left, std::wstring_view right)
{
	if (left.empty()) return std::wstring(right.data(), right.size());
	if (right.empty()) return std::wstring(left.data(), left.size());

	std::wstring out(left.data(), left.size());
	if (out.back() != L'\\' && out.back() != L'/') {
		out.push_back(L'\\');
	}
	out.append(right);
	return out;
}

std::wstring EnvVar(const wchar_t* name)
{
	const DWORD needed = GetEnvironmentVariableW(name, nullptr, 0);
	if (needed == 0) return {};

	std::wstring value(needed, L'\0');
	const DWORD written = GetEnvironmentVariableW(name, value.data(), needed);
	if (written == 0 || written >= needed) return {};
	value.resize(written);
	return value;
}

bool EndsWithLocalComponent(std::wstring_view path)
{
	constexpr std::wstring_view suffix1 = L"\\Local";
	if (path.size() < suffix1.size()) return false;

	auto tail = path.substr(path.size() - suffix1.size());
	std::wstring lower(tail.data(), tail.size());
	std::transform(lower.begin(), lower.end(), lower.begin(),
	               [](wchar_t c) { return static_cast<wchar_t>(towlower(c)); });
	return lower == L"\\local" || lower == L"/local";
}

std::wstring LocalAppDataLowFromLocalAppData(const std::wstring& local)
{
	if (local.empty()) return {};
	if (EndsWithLocalComponent(local)) {
		std::wstring out = local.substr(0, local.size() - 5);
		out += L"LocalLow";
		return out;
	}

	std::filesystem::path p(local);
	if (p.has_parent_path()) {
		return (p.parent_path() / L"LocalLow").wstring();
	}
	return {};
}

std::vector<std::wstring> LocalAppDataLowCandidates()
{
	std::vector<std::wstring> candidates;

	std::wstring testOverride = EnvVar(L"WKOPENVR_LOCALAPPDATA_OVERRIDE");
	if (!testOverride.empty()) candidates.push_back(std::move(testOverride));

	PWSTR raw = nullptr;
	if (S_OK == SHGetKnownFolderPath(FOLDERID_LocalAppDataLow, 0, nullptr, &raw) && raw) {
		candidates.emplace_back(raw);
	}
	if (raw) CoTaskMemFree(raw);

	std::wstring fromLocal = LocalAppDataLowFromLocalAppData(EnvVar(L"LOCALAPPDATA"));
	if (!fromLocal.empty()) candidates.push_back(std::move(fromLocal));

	std::wstring userProfile = EnvVar(L"USERPROFILE");
	if (!userProfile.empty()) {
		candidates.push_back(JoinPath(userProfile, L"AppData\\LocalLow"));
	}

	std::vector<std::wstring> unique;
	for (auto& candidate : candidates) {
		if (candidate.empty()) continue;
		const bool exists = std::any_of(unique.begin(), unique.end(), [&](const std::wstring& prior) {
			return _wcsicmp(prior.c_str(), candidate.c_str()) == 0;
		});
		if (!exists) unique.push_back(std::move(candidate));
	}
	return unique;
}

std::wstring TempWkOpenVrRoot()
{
	wchar_t buf[MAX_PATH] = {};
	const DWORD len = GetTempPathW(MAX_PATH, buf);
	if (len == 0 || len >= MAX_PATH) return {};
	return JoinPath(buf, L"WKOpenVR");
}

} // namespace

std::wstring LocalAppDataLowPath()
{
	auto candidates = LocalAppDataLowCandidates();
	return candidates.empty() ? std::wstring{} : candidates.front();
}

bool EnsureDirectory(const std::wstring& path)
{
	if (path.empty()) return false;
	std::error_code ec;
	return std::filesystem::create_directories(path, ec) || std::filesystem::is_directory(path, ec);
}

std::wstring WkOpenVrRootPath(bool create)
{
	for (const auto& candidate : LocalAppDataLowCandidates()) {
		std::wstring root = JoinPath(candidate, L"WKOpenVR");
		if (!create || EnsureDirectory(root)) return root;
	}

	if (create) {
		std::wstring fallback = TempWkOpenVrRoot();
		if (!fallback.empty() && EnsureDirectory(fallback)) return fallback;
	}
	return {};
}

std::wstring WkOpenVrSubdirectoryPath(std::wstring_view relative, bool create)
{
	std::wstring root = WkOpenVrRootPath(create);
	if (root.empty()) return {};

	std::wstring path = JoinPath(root, relative);
	if (create && !EnsureDirectory(path)) return {};
	return path;
}

std::wstring WkOpenVrLogsPath(bool create)
{
	return WkOpenVrSubdirectoryPath(L"Logs", create);
}

int64_t FileLastWriteTime(const std::wstring& path)
{
	WIN32_FILE_ATTRIBUTE_DATA data{};
	if (!GetFileAttributesExW(path.c_str(), GetFileExInfoStandard, &data)) {
		return 0;
	}

	ULARGE_INTEGER stamp{};
	stamp.LowPart = data.ftLastWriteTime.dwLowDateTime;
	stamp.HighPart = data.ftLastWriteTime.dwHighDateTime;
	return static_cast<int64_t>(stamp.QuadPart);
}

} // namespace openvr_pair::common
