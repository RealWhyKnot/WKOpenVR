#include "RecordingEnvelope.h"

#include "BuildChannel.h"
#include "DiagnosticsLog.h"
#include "FileLog.h"
#include "Win32Paths.h"

#include <fcntl.h>
#include <io.h>

#include <algorithm>
#include <cerrno>
#include <sstream>

namespace openvr_pair::common::recording {
namespace {

std::wstring ResolveDirectory(std::wstring_view dirOverride, bool create)
{
	if (!dirOverride.empty()) {
		return std::wstring(dirOverride.data(), dirOverride.size());
	}
	return WkOpenVrLogsPath(create);
}

std::wstring WithTrailingSlash(std::wstring directory)
{
	if (!directory.empty() && directory.back() != L'\\' && directory.back() != L'/') {
		directory.push_back(L'\\');
	}
	return directory;
}

} // namespace

std::string ComposeBanner(std::string_view schemaBanner, const std::vector<HeaderKV>& extras)
{
	std::string out;
	out += "# ";
	out.append(schemaBanner.data(), schemaBanner.size());
	out += "\n";
	out += "# build_stamp=" WKOPENVR_BUILD_STAMP "\n";
	out += "# build_channel=" WKOPENVR_BUILD_CHANNEL "\n";
	for (const auto& kv : extras) {
		out += "# ";
		out += kv.key;
		out += "=";
		out += kv.value;
		out += "\n";
	}
	return out;
}

std::string FormatAnnotation(double sessionSec, std::string_view event)
{
	char stamp[64] = {};
	snprintf(stamp, sizeof(stamp), "# [%.3f] ", sessionSec);
	std::string out = stamp;
	out.append(event.data(), event.size());
	out += "\n";
	return out;
}

std::vector<RecordingFileEntry> ListRecordings(std::wstring_view prefix, std::wstring_view extension,
                                               std::wstring_view dirOverride)
{
	std::vector<RecordingFileEntry> out;
	if (prefix.empty() || extension.empty()) return out;

	const std::wstring dir = ResolveDirectory(dirOverride, false);
	if (dir.empty()) return out;

	std::wstring pattern = WithTrailingSlash(dir);
	pattern.append(prefix.data(), prefix.size());
	pattern += L".*.";
	pattern.append(extension.data(), extension.size());

	WIN32_FIND_DATAW find{};
	HANDLE h = FindFirstFileW(pattern.c_str(), &find);
	if (h == INVALID_HANDLE_VALUE) return out;

	do {
		if (find.dwFileAttributes & FILE_ATTRIBUTE_DIRECTORY) continue;

		RecordingFileEntry entry;
		const int n = WideCharToMultiByte(CP_UTF8, 0, find.cFileName, -1, nullptr, 0, nullptr, nullptr);
		if (n > 1) {
			entry.nameUtf8.resize(static_cast<size_t>(n) - 1);
			WideCharToMultiByte(CP_UTF8, 0, find.cFileName, -1, entry.nameUtf8.data(), n, nullptr, nullptr);
		}
		entry.fullPath = WithTrailingSlash(dir) + find.cFileName;
		entry.sizeBytes = (static_cast<uint64_t>(find.nFileSizeHigh) << 32) | static_cast<uint64_t>(find.nFileSizeLow);
		entry.mtimeFileTime = (static_cast<uint64_t>(find.ftLastWriteTime.dwHighDateTime) << 32) |
		                      static_cast<uint64_t>(find.ftLastWriteTime.dwLowDateTime);
		out.push_back(std::move(entry));
	} while (FindNextFileW(h, &find));
	FindClose(h);

	std::sort(out.begin(), out.end(), [](const RecordingFileEntry& a, const RecordingFileEntry& b) {
		if (a.mtimeFileTime == b.mtimeFileTime) return a.nameUtf8 > b.nameUtf8;
		return a.mtimeFileTime > b.mtimeFileTime;
	});
	return out;
}

PruneResult PruneRecordings(std::wstring_view prefix, std::wstring_view extension, const RetentionPolicy& policy,
                            std::wstring_view dirOverride)
{
	PruneResult result;
	const std::vector<RecordingFileEntry> entries = ListRecordings(prefix, extension, dirOverride);
	result.totalFiles = entries.size();
	for (const auto& entry : entries) {
		result.totalBytes = detail::SaturatingAdd(result.totalBytes, entry.sizeBytes);
	}

	const RetentionPlan plan = PlanRetention(entries, policy);
	for (std::size_t index : plan.deleteIndexes) {
		if (index >= entries.size()) continue;
		const auto& entry = entries[index];
		if (DeleteFileW(entry.fullPath.c_str())) {
			++result.deletedFiles;
			result.freedBytes = detail::SaturatingAdd(result.freedBytes, entry.sizeBytes);
		}
		else {
			++result.failedDeletes;
		}
	}

	result.keptFiles = result.totalFiles - result.deletedFiles;
	result.keptBytes = (result.totalBytes >= result.freedBytes) ? (result.totalBytes - result.freedBytes) : 0;
	return result;
}

RecordingEnvelope::~RecordingEnvelope()
{
	Close();
}

bool RecordingEnvelope::Open(const EnvelopeOptions& options)
{
#if WKOPENVR_BUILD_IS_DEV
	if (file_) return false;
	if (options.prefix.empty() || options.extension.empty() || options.schemaBanner.empty()) return false;

	const std::wstring dir = ResolveDirectory(options.directoryOverride, true);
	if (dir.empty()) {
		DiagnosticLog("recording", "envelope_open_failed prefix='%ls' reason=no_directory", options.prefix.c_str());
		return false;
	}

	openPrune_ = PruneResult{};
	if (options.pruneAtOpen) {
		openPrune_ = PruneRecordings(options.prefix, options.extension, options.retention, options.directoryOverride);
	}

	SYSTEMTIME now{};
	GetSystemTime(&now);
	const std::wstring baseName = TimestampedRecordingName(options.prefix, options.extension, now);
	if (baseName.empty()) return false;

	const std::wstring dirSlash = WithTrailingSlash(dir);
	const DWORD pid = GetCurrentProcessId();
	FILE* file = nullptr;
	std::wstring path;
	for (int attempt = 0; attempt < 100; ++attempt) {
		std::wstring candidate = dirSlash + InsertCollisionSuffix(baseName, options.extension, pid, attempt);
		HANDLE handle = CreateFileW(candidate.c_str(), GENERIC_WRITE, FILE_SHARE_READ, nullptr, CREATE_NEW,
		                            FILE_ATTRIBUTE_NORMAL | FILE_FLAG_WRITE_THROUGH, nullptr);
		if (handle == INVALID_HANDLE_VALUE) {
			const DWORD err = GetLastError();
			if (err == ERROR_FILE_EXISTS || err == ERROR_ALREADY_EXISTS) {
				continue;
			}
			DiagnosticLog("recording", "envelope_create_failed path='%ls' err=%lu", candidate.c_str(), err);
			return false;
		}

		const int fd = _open_osfhandle(reinterpret_cast<intptr_t>(handle), _O_WRONLY | _O_BINARY);
		if (fd < 0) {
			CloseHandle(handle);
			DiagnosticLog("recording", "envelope_fd_failed path='%ls' errno=%d", candidate.c_str(), errno);
			return false;
		}

		file = _fdopen(fd, "wb");
		if (!file) {
			_close(fd);
			DiagnosticLog("recording", "envelope_fdopen_failed path='%ls' errno=%d", candidate.c_str(), errno);
			return false;
		}

		path = std::move(candidate);
		break;
	}
	if (!file) {
		DiagnosticLog("recording", "envelope_create_failed prefix='%ls' reason=name_collision_exhausted",
		              options.prefix.c_str());
		return false;
	}

	file_ = file;
	path_ = std::move(path);
	maxFileBytes_ = options.maxFileBytes;
	bytesWritten_ = 0;
	capped_ = false;
	writeFailed_ = false;
	bytesSinceFlush_ = 0;
	openTime_ = std::chrono::steady_clock::now();
	lastFlushTime_ = openTime_;
	SetLowLatencyLogMode(file_);

	std::ostringstream header;
	header << ComposeBanner(options.schemaBanner, options.headerKVs);
	header << "# dev_auto_recording=enabled"
	       << " retention_files=" << options.retention.maxFiles
	       << " retention_bytes=" << options.retention.maxTotalBytes << " total_files=" << openPrune_.totalFiles
	       << " total_bytes=" << openPrune_.totalBytes << " deleted_files=" << openPrune_.deletedFiles
	       << " freed_bytes=" << openPrune_.freedBytes << " kept_files=" << openPrune_.keptFiles
	       << " kept_bytes=" << openPrune_.keptBytes << " failed_deletes=" << openPrune_.failedDeletes << "\n";
	if (!WriteText(header.str())) {
		Close();
		return false;
	}

	DiagnosticLog("recording", "envelope_opened path='%ls' deleted_files=%llu freed_bytes=%llu", path_.c_str(),
	              static_cast<unsigned long long>(openPrune_.deletedFiles),
	              static_cast<unsigned long long>(openPrune_.freedBytes));
	return true;
#else
	(void)options;
	return false;
#endif
}

bool RecordingEnvelope::WriteText(std::string_view text)
{
	if (!file_) return false;
	if (!text.empty()) {
		const size_t written = fwrite(text.data(), 1, text.size(), file_);
		if (written != text.size()) {
			if (!writeFailed_) {
				DiagnosticLog("recording", "envelope_write_failed path='%ls' errno=%d", path_.c_str(), errno);
				writeFailed_ = true;
			}
			return false;
		}
		writeFailed_ = false;
		bytesWritten_ += written;
		bytesSinceFlush_ += static_cast<long long>(written);
	}
	MaybeFlush();
	return true;
}

void RecordingEnvelope::MaybeFlush()
{
	const long long msSinceFlush =
	    std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - lastFlushTime_)
	        .count();
	if (ShouldFlushLog(bytesSinceFlush_, msSinceFlush)) {
		Flush();
	}
}

bool RecordingEnvelope::WriteRow(std::string_view rowNoNewline)
{
	if (!file_ || capped_) return false;
	if (maxFileBytes_ > 0 && bytesWritten_ >= maxFileBytes_) {
		capped_ = true;
		WriteText(FormatAnnotation(SecondsSinceOpen(), "recording_stopped: byte_cap"));
		Flush();
		DiagnosticLog("recording", "envelope_byte_cap path='%ls' bytes=%llu", path_.c_str(),
		              static_cast<unsigned long long>(bytesWritten_));
		return false;
	}
	std::string row(rowNoNewline);
	row += "\n";
	return WriteText(row);
}

bool RecordingEnvelope::WriteAnnotation(std::string_view event)
{
	return WriteAnnotation(SecondsSinceOpen(), event);
}

bool RecordingEnvelope::WriteAnnotation(double sessionSec, std::string_view event)
{
	if (!file_) return false;
	return WriteText(FormatAnnotation(sessionSec, event));
}

double RecordingEnvelope::SecondsSinceOpen() const
{
	return std::chrono::duration<double>(std::chrono::steady_clock::now() - openTime_).count();
}

bool RecordingEnvelope::Flush()
{
	if (!file_) return false;
	if (FlushLogFileToDisk(file_)) {
		bytesSinceFlush_ = 0;
		lastFlushTime_ = std::chrono::steady_clock::now();
		return true;
	}
	return false;
}

void RecordingEnvelope::Close()
{
	if (file_) {
		FlushLogFileToDisk(file_);
		fclose(file_);
	}
	file_ = nullptr;
	path_.clear();
	bytesSinceFlush_ = 0;
}

} // namespace openvr_pair::common::recording
