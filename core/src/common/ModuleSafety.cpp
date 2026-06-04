#include "ModuleSafety.h"

#include "Win32Paths.h"

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>

#include <string>

namespace openvr_pair::common::module_safety {
namespace {

constexpr unsigned kActiveOnlyAutoDisableThreshold = 3;

struct CrashCounters
{
	unsigned active_unclean = 0;
	unsigned suspect_unclean = 0;
};

std::wstring WidenAscii(std::string_view value)
{
	std::wstring out;
	out.reserve(value.size());
	for (char ch : value) {
		out.push_back(static_cast<wchar_t>(static_cast<unsigned char>(ch)));
	}
	return out;
}

std::wstring JoinPath(const std::wstring &left, std::wstring_view right)
{
	if (left.empty()) return std::wstring(right.data(), right.size());
	std::wstring out = left;
	if (out.back() != L'\\' && out.back() != L'/') out.push_back(L'\\');
	out.append(right);
	return out;
}

bool FileExists(const std::wstring &path)
{
	if (path.empty()) return false;
	DWORD attr = GetFileAttributesW(path.c_str());
	return attr != INVALID_FILE_ATTRIBUTES && !(attr & FILE_ATTRIBUTE_DIRECTORY);
}

bool DeleteFileIfPresent(const std::wstring &path)
{
	if (path.empty()) return false;
	if (DeleteFileW(path.c_str()) != FALSE) return true;
	const DWORD err = GetLastError();
	return err == ERROR_FILE_NOT_FOUND || err == ERROR_PATH_NOT_FOUND;
}

bool WriteTextFile(const std::wstring &path, std::string_view body)
{
	if (path.empty()) return false;
	HANDLE file = CreateFileW(
		path.c_str(),
		GENERIC_WRITE,
		FILE_SHARE_READ | FILE_SHARE_WRITE | FILE_SHARE_DELETE,
		nullptr,
		CREATE_ALWAYS,
		FILE_ATTRIBUTE_NORMAL,
		nullptr);
	if (file == INVALID_HANDLE_VALUE) return false;

	DWORD written = 0;
	const BOOL ok = WriteFile(
		file,
		body.data(),
		static_cast<DWORD>(body.size()),
		&written,
		nullptr);
	CloseHandle(file);
	return ok != FALSE && written == body.size();
}

std::string ReadTextFile(const std::wstring &path)
{
	if (path.empty()) return {};
	HANDLE file = CreateFileW(
		path.c_str(),
		GENERIC_READ,
		FILE_SHARE_READ | FILE_SHARE_WRITE | FILE_SHARE_DELETE,
		nullptr,
		OPEN_EXISTING,
		FILE_ATTRIBUTE_NORMAL,
		nullptr);
	if (file == INVALID_HANDLE_VALUE) return {};

	LARGE_INTEGER size{};
	if (!GetFileSizeEx(file, &size) || size.QuadPart <= 0 || size.QuadPart > 65536) {
		CloseHandle(file);
		return {};
	}

	std::string body(static_cast<size_t>(size.QuadPart), '\0');
	DWORD read = 0;
	const BOOL ok = ReadFile(
		file,
		body.data(),
		static_cast<DWORD>(body.size()),
		&read,
		nullptr);
	CloseHandle(file);
	if (!ok) return {};
	body.resize(read);
	return body;
}

std::string MarkerBody(const ModuleSpec &spec, std::string_view reason)
{
	std::string body;
	body.reserve(180 + reason.size());
	body += "module=";
	body += spec.slug;
	body += "\nflag=";
	body += spec.flag_file;
	body += "\nreason=";
	body.append(reason.data(), reason.size());
	body += "\n";
	return body;
}

bool WriteMarker(const std::wstring &path, const ModuleSpec &spec, std::string_view reason)
{
	return WriteTextFile(path, MarkerBody(spec, reason));
}

std::string ReadField(std::string_view body, std::string_view key)
{
	const size_t pos = body.find(key);
	if (pos == std::string_view::npos) return {};
	size_t begin = pos + key.size();
	size_t end = body.find('\n', begin);
	if (end == std::string_view::npos) end = body.size();
	return std::string(body.substr(begin, end - begin));
}

unsigned ReadUnsignedField(std::string_view body, std::string_view key)
{
	const std::string value = ReadField(body, key);
	unsigned out = 0;
	for (char ch : value) {
		if (ch < '0' || ch > '9') break;
		out = out * 10u + static_cast<unsigned>(ch - '0');
	}
	return out;
}

CrashCounters ReadCounters(const ModuleSpec &spec)
{
	const std::string body = ReadTextFile(CounterPath(spec, false));
	CrashCounters counters;
	counters.active_unclean = ReadUnsignedField(body, "active_unclean=");
	counters.suspect_unclean = ReadUnsignedField(body, "suspect_unclean=");
	return counters;
}

bool WriteCounters(const ModuleSpec &spec, const CrashCounters &counters)
{
	std::string body;
	body.reserve(96);
	body += "module=";
	body += spec.slug;
	body += "\nactive_unclean=";
	body += std::to_string(counters.active_unclean);
	body += "\nsuspect_unclean=";
	body += std::to_string(counters.suspect_unclean);
	body += "\n";
	return WriteTextFile(CounterPath(spec, true), body);
}

std::wstring EnvironmentRootOverride()
{
	DWORD len = GetEnvironmentVariableW(L"WKOPENVR_MODULE_SAFETY_ROOT", nullptr, 0);
	if (len == 0) return {};
	std::wstring value(len, L'\0');
	DWORD written = GetEnvironmentVariableW(L"WKOPENVR_MODULE_SAFETY_ROOT", value.data(), len);
	if (written == 0 || written >= len) return {};
	value.resize(written);
	return value;
}

std::wstring StatePath(const ModuleSpec &spec, const wchar_t *prefix, const wchar_t *suffix, bool create)
{
	std::wstring root = RootPath(create);
	if (root.empty()) return {};
	std::wstring file = prefix;
	file += WidenAscii(spec.slug);
	file += suffix;
	return JoinPath(root, file);
}

void ClearRuntimeMarkers(const ModuleSpec &spec)
{
	DeleteFileIfPresent(ActiveMarkerPath(spec, false));
	DeleteFileIfPresent(SuspectMarkerPath(spec, false));
}

} // namespace

const ModuleSpec *Specs(size_t *count)
{
	return openvr_pair::common::modules::DriverSafetyModules(count);
}

const ModuleSpec *FindById(openvr_pair::common::modules::ModuleId id)
{
	const ModuleSpec *spec = openvr_pair::common::modules::FindById(id);
	return spec && spec->participates_in_driver_safety ? spec : nullptr;
}

const ModuleSpec *FindBySlug(std::string_view slug)
{
	const ModuleSpec *spec = openvr_pair::common::modules::FindBySlug(slug);
	return spec && spec->participates_in_driver_safety ? spec : nullptr;
}

const ModuleSpec *FindByFlagFileName(std::string_view flagFileName)
{
	const ModuleSpec *spec = openvr_pair::common::modules::FindByFlagFileName(flagFileName);
	return spec && spec->participates_in_driver_safety ? spec : nullptr;
}

std::wstring RootPath(bool create)
{
	std::wstring overrideRoot = EnvironmentRootOverride();
	if (!overrideRoot.empty()) {
		if (create) {
			CreateDirectoryW(overrideRoot.c_str(), nullptr);
		}
		return overrideRoot;
	}
	return WkOpenVrSubdirectoryPath(L"module_safety", create);
}

std::wstring ActiveMarkerPath(const ModuleSpec &spec, bool create)
{
	return StatePath(spec, L"active_", L".flag", create);
}

std::wstring SuspectMarkerPath(const ModuleSpec &spec, bool create)
{
	return StatePath(spec, L"suspect_", L".flag", create);
}

std::wstring CleanMarkerPath(const ModuleSpec &spec, bool create)
{
	return StatePath(spec, L"clean_", L".flag", create);
}

std::wstring CounterPath(const ModuleSpec &spec, bool create)
{
	return StatePath(spec, L"counter_", L".state", create);
}

std::wstring AutoDisabledMarkerPath(const ModuleSpec &spec, bool create)
{
	return StatePath(spec, L"auto_disabled_", L".flag", create);
}

bool HasActiveMarker(const ModuleSpec &spec)
{
	return FileExists(ActiveMarkerPath(spec, false));
}

bool HasSuspectMarker(const ModuleSpec &spec)
{
	return FileExists(SuspectMarkerPath(spec, false));
}

bool HasCleanMarker(const ModuleSpec &spec)
{
	return FileExists(CleanMarkerPath(spec, false));
}

bool HasAutoDisabledMarker(const ModuleSpec &spec)
{
	return FileExists(AutoDisabledMarkerPath(spec, false));
}

std::string AutoDisabledReason(const ModuleSpec &spec)
{
	return ReadField(ReadTextFile(AutoDisabledMarkerPath(spec, false)), "reason=");
}

bool MarkActive(const ModuleSpec &spec)
{
	DeleteFileIfPresent(CleanMarkerPath(spec, false));
	return WriteMarker(ActiveMarkerPath(spec, true), spec, "active");
}

bool MarkSuspect(const ModuleSpec &spec, std::string_view reason)
{
	return WriteMarker(SuspectMarkerPath(spec, true), spec, reason);
}

bool ClearSuspect(const ModuleSpec &spec)
{
	return DeleteFileIfPresent(SuspectMarkerPath(spec, false));
}

bool MarkClean(const ModuleSpec &spec)
{
	const bool activeCleared = DeleteFileIfPresent(ActiveMarkerPath(spec, false));
	const bool suspectCleared = DeleteFileIfPresent(SuspectMarkerPath(spec, false));
	DeleteFileIfPresent(CounterPath(spec, false));
	return WriteMarker(CleanMarkerPath(spec, true), spec, "clean_shutdown")
		&& activeCleared
		&& suspectCleared;
}

bool MarkFault(const ModuleSpec &spec, std::string_view reason)
{
	const bool wrote = WriteMarker(AutoDisabledMarkerPath(spec, true), spec, reason);
	ClearRuntimeMarkers(spec);
	DeleteFileIfPresent(CleanMarkerPath(spec, false));
	return wrote;
}

LaunchAssessment AssessLaunch(const ModuleSpec &spec, bool allowActiveOnlyAutoDisable)
{
	LaunchAssessment result;
	result.had_stale_active = HasActiveMarker(spec);
	result.had_stale_suspect = HasSuspectMarker(spec);
	if (!result.had_stale_active && !result.had_stale_suspect) {
		return result;
	}

	CrashCounters counters = ReadCounters(spec);
	if (result.had_stale_suspect) {
		++counters.suspect_unclean;
		result.reason = "unclean_exit_during_module_operation";
	} else {
		++counters.active_unclean;
		if (allowActiveOnlyAutoDisable &&
			counters.active_unclean >= kActiveOnlyAutoDisableThreshold) {
			result.reason = "repeated_unclean_driver_exit";
		}
	}
	WriteCounters(spec, counters);
	result.active_unclean_count = counters.active_unclean;
	result.suspect_unclean_count = counters.suspect_unclean;

	if (!result.reason.empty()) {
		result.auto_disabled = MarkFault(spec, result.reason);
	} else {
		ClearRuntimeMarkers(spec);
	}
	return result;
}

bool ConvertStaleActiveToAutoDisabled(const ModuleSpec &spec)
{
	return AssessLaunch(spec, true).auto_disabled;
}

bool ClearAutoDisabled(const ModuleSpec &spec)
{
	const bool disabledCleared = DeleteFileIfPresent(AutoDisabledMarkerPath(spec, false));
	const bool activeCleared = DeleteFileIfPresent(ActiveMarkerPath(spec, false));
	const bool suspectCleared = DeleteFileIfPresent(SuspectMarkerPath(spec, false));
	DeleteFileIfPresent(CounterPath(spec, false));
	return disabledCleared && activeCleared && suspectCleared;
}

bool ClearAutoDisabledForFlag(std::string_view flagFileName)
{
	const ModuleSpec *spec = FindByFlagFileName(flagFileName);
	return spec ? ClearAutoDisabled(*spec) : false;
}

} // namespace openvr_pair::common::module_safety
