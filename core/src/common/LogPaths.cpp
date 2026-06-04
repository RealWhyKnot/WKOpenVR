#include "LogPaths.h"

#include "Win32Paths.h"

#include <cstdint>
#include <vector>

namespace openvr_pair::common {
namespace {

uint64_t FileTimeToU64(const FILETIME& ft)
{
	ULARGE_INTEGER q{};
	q.LowPart = ft.dwLowDateTime;
	q.HighPart = ft.dwHighDateTime;
	return q.QuadPart;
}

} // namespace

std::wstring TimestampedLogFileName(std::wstring_view prefix, const SYSTEMTIME& utc)
{
	const int dateLen = GetDateFormatEx(LOCALE_NAME_INVARIANT, 0, &utc, L"yyyy-MM-dd", nullptr, 0, nullptr);
	const int timeLen = GetTimeFormatEx(LOCALE_NAME_INVARIANT, 0, &utc, L"HH-mm-ss", nullptr, 0);
	if (dateLen <= 0 || timeLen <= 0) return {};

	std::vector<wchar_t> date(static_cast<size_t>(dateLen));
	std::vector<wchar_t> time(static_cast<size_t>(timeLen));
	if (!GetDateFormatEx(LOCALE_NAME_INVARIANT, 0, &utc, L"yyyy-MM-dd", date.data(), dateLen, nullptr)) {
		return {};
	}
	if (!GetTimeFormatEx(LOCALE_NAME_INVARIANT, 0, &utc, L"HH-mm-ss", time.data(), timeLen)) {
		return {};
	}

	std::wstring name(prefix.data(), prefix.size());
	name += L".";
	name += date.data();
	name += L"T";
	name += time.data();
	name += L".txt";
	return name;
}

void DeleteOldLogFiles(const std::wstring& directory, std::wstring_view prefix, std::chrono::hours maxAge)
{
	if (directory.empty() || prefix.empty() || maxAge.count() <= 0) return;

	SYSTEMTIME nowSystem{};
	FILETIME nowFile{};
	GetSystemTime(&nowSystem);
	if (!SystemTimeToFileTime(&nowSystem, &nowFile)) return;

	const uint64_t now = FileTimeToU64(nowFile);
	const uint64_t age100ns = static_cast<uint64_t>(maxAge.count()) * 3600ULL * 10ULL * 1000ULL * 1000ULL;
	const uint64_t cutoff = (now > age100ns) ? now - age100ns : 0;

	std::wstring search = directory;
	if (!search.empty() && search.back() != L'\\' && search.back() != L'/') {
		search.push_back(L'\\');
	}
	search.append(prefix.data(), prefix.size());
	search += L".*.txt";

	WIN32_FIND_DATAW findData{};
	HANDLE handle = FindFirstFileW(search.c_str(), &findData);
	if (handle == INVALID_HANDLE_VALUE) return;

	do {
		const uint64_t fileTime = FileTimeToU64(findData.ftLastWriteTime);
		if (fileTime < cutoff) {
			std::wstring path = directory;
			if (!path.empty() && path.back() != L'\\' && path.back() != L'/') {
				path.push_back(L'\\');
			}
			path += findData.cFileName;
			DeleteFileW(path.c_str());
		}
	} while (FindNextFileW(handle, &findData));

	FindClose(handle);
}

std::wstring TimestampedLogPath(std::wstring_view prefix)
{
	std::wstring directory = WkOpenVrLogsPath(true);
	if (directory.empty()) return {};

	DeleteOldLogFiles(directory, prefix);

	SYSTEMTIME now{};
	GetSystemTime(&now);
	std::wstring name = TimestampedLogFileName(prefix, now);
	if (name.empty()) return {};

	if (!directory.empty() && directory.back() != L'\\' && directory.back() != L'/') {
		directory.push_back(L'\\');
	}
	directory += name;
	return directory;
}

} // namespace openvr_pair::common
