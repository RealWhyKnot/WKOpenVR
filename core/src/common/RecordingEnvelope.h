#pragma once

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <limits>
#include <string>
#include <string_view>
#include <vector>

// Shared on-disk recording format for dev replay captures. Every module
// recorder writes the same envelope: a "# <schema_name>" banner line, build
// stamp/channel header lines, timestamped file names, count+byte retention
// pruned at open, and a batched device-sync flush. Readers stay per-module;
// the envelope only owns the container so recordings from any module look the
// same in the Logs directory and age out the same way.
//
// Retention enumerates "<prefix>.*.<ext>" non-recursively, so captures copied
// into a subdirectory (Logs\corpus\) are never pruned.

namespace openvr_pair::common::recording {

struct RetentionPolicy
{
	std::size_t maxFiles = 5;
	uint64_t maxTotalBytes = 512ull * 1024ull * 1024ull;
};

struct RecordingFileEntry
{
	std::string nameUtf8;  // file name only, UTF-8 for UI display
	std::wstring fullPath; // absolute path for opening/deleting
	uint64_t sizeBytes = 0;
	uint64_t mtimeFileTime = 0; // FILETIME as uint64; for sorting
};

struct RetentionPlan
{
	std::vector<std::size_t> deleteIndexes;
	std::size_t keptFiles = 0;
	uint64_t keptBytes = 0;
	uint64_t deletedBytes = 0;
};

namespace detail {
inline uint64_t SaturatingAdd(uint64_t a, uint64_t b)
{
	const uint64_t max = std::numeric_limits<uint64_t>::max();
	return (b > max - a) ? max : (a + b);
}
} // namespace detail

// Greedy newest-first keep under both caps. The newest recording survives
// even when it alone exceeds the byte cap: a capture that just finished is
// always worth more than an empty directory.
inline RetentionPlan PlanRetention(const std::vector<RecordingFileEntry>& newestFirst, const RetentionPolicy& policy)
{
	RetentionPlan plan;

	for (std::size_t i = 0; i < newestFirst.size(); ++i) {
		const auto& entry = newestFirst[i];

		bool keep = false;
		if (policy.maxFiles > 0 && plan.keptFiles < policy.maxFiles && policy.maxTotalBytes > 0) {
			bool fitsByteBudget = false;
			if (plan.keptBytes <= policy.maxTotalBytes) {
				const uint64_t remainingBytes = policy.maxTotalBytes - plan.keptBytes;
				fitsByteBudget = entry.sizeBytes <= remainingBytes;
			}
			if (!fitsByteBudget && plan.keptFiles == 0) {
				fitsByteBudget = true;
			}
			keep = fitsByteBudget;
		}

		if (keep) {
			++plan.keptFiles;
			plan.keptBytes = detail::SaturatingAdd(plan.keptBytes, entry.sizeBytes);
		}
		else {
			plan.deleteIndexes.push_back(i);
			plan.deletedBytes = detail::SaturatingAdd(plan.deletedBytes, entry.sizeBytes);
		}
	}

	return plan;
}

// "<prefix>.yyyy-MM-ddTHH-mm-ss.<ext>". Numeric formatting only, so callers
// can pass any SYSTEMTIME (tests use fixed values).
inline std::wstring TimestampedRecordingName(std::wstring_view prefix, std::wstring_view extension,
                                             const SYSTEMTIME& utc)
{
	if (prefix.empty() || extension.empty()) return {};
	wchar_t stamp[32] = {};
	swprintf_s(stamp, L".%04u-%02u-%02uT%02u-%02u-%02u.", utc.wYear, utc.wMonth, utc.wDay, utc.wHour, utc.wMinute,
	           utc.wSecond);
	std::wstring name(prefix.data(), prefix.size());
	name += stamp;
	name.append(extension.data(), extension.size());
	return name;
}

// Collision suffix ".{pid}-{attempt}" inserted before the extension, so a
// second writer in the same second still gets a unique CREATE_NEW name.
inline std::wstring InsertCollisionSuffix(const std::wstring& fileName, std::wstring_view extension, unsigned long pid,
                                          int attempt)
{
	if (attempt == 0) return fileName;

	std::wstring extWithDot = L".";
	extWithDot.append(extension.data(), extension.size());
	const size_t dot = fileName.rfind(extWithDot);
	std::wstring out = dot == std::wstring::npos ? fileName : fileName.substr(0, dot);
	wchar_t suffix[64] = {};
	swprintf_s(suffix, L".%lu-%02d", pid, attempt);
	out += suffix;
	if (dot != std::wstring::npos) {
		out += fileName.substr(dot);
	}
	return out;
}

struct HeaderKV
{
	std::string key;
	std::string value;
};

// Banner: schema name first (readers sniff the format from line one), then
// build identification, then caller extras. Every line is '#'-prefixed so
// CSV parsers that skip comments never see header text as data.
std::string ComposeBanner(std::string_view schemaBanner, const std::vector<HeaderKV>& extras);

// "# [123.456] text\n" -- session-relative seconds, matching the annotation
// shape readers already parse out of calibration captures.
std::string FormatAnnotation(double sessionSec, std::string_view event);

struct PruneResult
{
	std::size_t totalFiles = 0;
	uint64_t totalBytes = 0;
	std::size_t deletedFiles = 0;
	uint64_t freedBytes = 0;
	std::size_t failedDeletes = 0;
	std::size_t keptFiles = 0;
	uint64_t keptBytes = 0;
};

// Newest-first listing of "<prefix>.*.<ext>" in the logs directory (or
// dirOverride for tests). Non-recursive; subdirectories are skipped.
std::vector<RecordingFileEntry> ListRecordings(std::wstring_view prefix, std::wstring_view extension,
                                               std::wstring_view dirOverride = {});

// List + PlanRetention + delete. Safe to call in any build channel and with
// recording disabled: it only removes files matching the recording pattern.
PruneResult PruneRecordings(std::wstring_view prefix, std::wstring_view extension, const RetentionPolicy& policy,
                            std::wstring_view dirOverride = {});

struct EnvelopeOptions
{
	std::wstring prefix;      // e.g. L"phantom_replay"
	std::wstring extension;   // e.g. L"csv" (no leading dot)
	std::string schemaBanner; // e.g. "phantom_replay_v1"
	std::vector<HeaderKV> headerKVs;
	RetentionPolicy retention;
	std::wstring directoryOverride; // tests; empty = WkOpenVrLogsPath(true)
	bool pruneAtOpen = true;
	// In-session stop: retention only runs at open, so a single runaway
	// session could still fill the disk. Past this size the envelope writes
	// one "recording_stopped" annotation and drops further rows.
	uint64_t maxFileBytes = 1024ull * 1024ull * 1024ull;
};

// Dev-only recording file. Open() fails (returns false) outside dev builds;
// List/PruneRecordings above stay available everywhere. Not thread-safe:
// callers serialize access (module recorders already write from one thread
// or under their own lock).
class RecordingEnvelope
{
public:
	RecordingEnvelope() = default;
	~RecordingEnvelope();
	RecordingEnvelope(const RecordingEnvelope&) = delete;
	RecordingEnvelope& operator=(const RecordingEnvelope&) = delete;

	bool Open(const EnvelopeOptions& options);
	bool IsOpen() const { return file_ != nullptr; }

	// Row text without trailing newline. Returns false when closed, capped,
	// or the write failed.
	bool WriteRow(std::string_view rowNoNewline);
	bool WriteAnnotation(std::string_view event); // timestamped from open
	bool WriteAnnotation(double sessionSec, std::string_view event);

	bool Flush(); // forced device sync
	void Close(); // flush + close; also runs from the destructor

	const std::wstring& Path() const { return path_; }
	const PruneResult& OpenPrune() const { return openPrune_; }
	bool ByteCapReached() const { return capped_; }
	uint64_t BytesWritten() const { return bytesWritten_; }
	double SecondsSinceOpen() const;

private:
	bool WriteText(std::string_view text);
	void MaybeFlush();

	FILE* file_ = nullptr;
	std::wstring path_;
	PruneResult openPrune_;
	uint64_t maxFileBytes_ = 0;
	uint64_t bytesWritten_ = 0;
	bool capped_ = false;
	bool writeFailed_ = false;
	long long bytesSinceFlush_ = 0;
	std::chrono::steady_clock::time_point lastFlushTime_{};
	std::chrono::steady_clock::time_point openTime_{};
};

} // namespace openvr_pair::common::recording
