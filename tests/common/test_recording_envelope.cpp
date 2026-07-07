#include "BuildChannel.h"
#include "RecordingEnvelope.h"

#include <gtest/gtest.h>

#include <chrono>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

namespace recording = openvr_pair::common::recording;
namespace fs = std::filesystem;

namespace {

recording::RecordingFileEntry MakeEntry(const char* name, uint64_t sizeBytes)
{
	recording::RecordingFileEntry entry;
	entry.nameUtf8 = name;
	entry.sizeBytes = sizeBytes;
	return entry;
}

fs::path MakeTestDir(const char* tag)
{
	const fs::path dir = fs::temp_directory_path() / (std::string("wkopenvr_envelope_") + tag);
	fs::remove_all(dir);
	fs::create_directories(dir);
	return dir;
}

std::vector<std::string> ReadLines(const fs::path& path)
{
	std::ifstream in(path);
	std::vector<std::string> lines;
	std::string line;
	while (std::getline(in, line)) {
		lines.push_back(line);
	}
	return lines;
}

} // namespace

TEST(RecordingRetentionPlanTest, KeepsNewestUnderBothCaps)
{
	std::vector<recording::RecordingFileEntry> entries{
	    MakeEntry("c.csv", 100),
	    MakeEntry("b.csv", 100),
	    MakeEntry("a.csv", 100),
	};
	recording::RetentionPolicy policy;
	policy.maxFiles = 2;
	policy.maxTotalBytes = 1000;

	const auto plan = recording::PlanRetention(entries, policy);
	ASSERT_EQ(1u, plan.deleteIndexes.size());
	EXPECT_EQ(2u, plan.deleteIndexes[0]); // oldest
	EXPECT_EQ(2u, plan.keptFiles);
	EXPECT_EQ(200u, plan.keptBytes);
	EXPECT_EQ(100u, plan.deletedBytes);
}

TEST(RecordingRetentionPlanTest, ByteCapCutsBeforeFileCap)
{
	std::vector<recording::RecordingFileEntry> entries{
	    MakeEntry("c.csv", 400),
	    MakeEntry("b.csv", 400),
	    MakeEntry("a.csv", 400),
	};
	recording::RetentionPolicy policy;
	policy.maxFiles = 5;
	policy.maxTotalBytes = 900;

	const auto plan = recording::PlanRetention(entries, policy);
	ASSERT_EQ(1u, plan.deleteIndexes.size());
	EXPECT_EQ(2u, plan.deleteIndexes[0]);
	EXPECT_EQ(2u, plan.keptFiles);
	EXPECT_EQ(800u, plan.keptBytes);
}

TEST(RecordingRetentionPlanTest, OversizedNewestIsAlwaysKept)
{
	std::vector<recording::RecordingFileEntry> entries{
	    MakeEntry("big.csv", 5000),
	    MakeEntry("old.csv", 100),
	};
	recording::RetentionPolicy policy;
	policy.maxFiles = 5;
	policy.maxTotalBytes = 1000;

	const auto plan = recording::PlanRetention(entries, policy);
	ASSERT_EQ(1u, plan.deleteIndexes.size());
	EXPECT_EQ(1u, plan.deleteIndexes[0]); // the older file goes, not the newest
	EXPECT_EQ(1u, plan.keptFiles);
}

TEST(RecordingRetentionPlanTest, ZeroPolicyDeletesEverything)
{
	std::vector<recording::RecordingFileEntry> entries{
	    MakeEntry("b.csv", 10),
	    MakeEntry("a.csv", 10),
	};
	const auto planNoFiles = recording::PlanRetention(entries, {0, 1000});
	EXPECT_EQ(2u, planNoFiles.deleteIndexes.size());

	const auto planNoBytes = recording::PlanRetention(entries, {5, 0});
	EXPECT_EQ(2u, planNoBytes.deleteIndexes.size());
}

TEST(RecordingNameTest, FormatsTimestampedName)
{
	SYSTEMTIME utc{};
	utc.wYear = 2026;
	utc.wMonth = 7;
	utc.wDay = 3;
	utc.wHour = 4;
	utc.wMinute = 5;
	utc.wSecond = 6;
	EXPECT_EQ(L"phantom_replay.2026-07-03T04-05-06.csv",
	          recording::TimestampedRecordingName(L"phantom_replay", L"csv", utc));
	EXPECT_TRUE(recording::TimestampedRecordingName(L"", L"csv", utc).empty());
	EXPECT_TRUE(recording::TimestampedRecordingName(L"p", L"", utc).empty());
}

TEST(RecordingNameTest, CollisionSuffixInsertsBeforeExtension)
{
	const std::wstring base = L"phantom_replay.2026-07-03T04-05-06.csv";
	EXPECT_EQ(base, recording::InsertCollisionSuffix(base, L"csv", 4242, 0));
	EXPECT_EQ(L"phantom_replay.2026-07-03T04-05-06.4242-02.csv",
	          recording::InsertCollisionSuffix(base, L"csv", 4242, 2));
}

TEST(RecordingBannerTest, SchemaFirstThenBuildThenExtras)
{
	const std::string banner =
	    recording::ComposeBanner("sample_rec_v1", {{"columns", "time_ms,x"}, {"note", "unit-test"}});
	std::istringstream in(banner);
	std::vector<std::string> lines;
	std::string line;
	while (std::getline(in, line)) {
		lines.push_back(line);
	}

	ASSERT_GE(lines.size(), 5u);
	EXPECT_EQ("# sample_rec_v1", lines[0]);
	EXPECT_EQ(0u, lines[1].rfind("# build_stamp=", 0));
	EXPECT_EQ(0u, lines[2].rfind("# build_channel=", 0));
	EXPECT_EQ("# columns=time_ms,x", lines[3]);
	EXPECT_EQ("# note=unit-test", lines[4]);
	for (const auto& l : lines) {
		ASSERT_FALSE(l.empty());
		EXPECT_EQ('#', l[0]);
	}
}

TEST(RecordingBannerTest, AnnotationFormat)
{
	EXPECT_EQ("# [1.500] budget: rows=3\n", recording::FormatAnnotation(1.5, "budget: rows=3"));
}

// RecordingEnvelope::Open compiles to a stub outside dev-channel builds (no
// auto-recordings in release binaries), so the write-path tests only exist on
// the dev channel; release-channel builds pin the stub behavior instead.
#if WKOPENVR_BUILD_IS_DEV
TEST(RecordingEnvelopeTest, WriteReadRoundTrip)
{
	const fs::path dir = MakeTestDir("roundtrip");

	recording::EnvelopeOptions options;
	options.prefix = L"sample_rec";
	options.extension = L"csv";
	options.schemaBanner = "sample_rec_v1";
	options.headerKVs = {{"columns", "time_ms,x"}};
	options.directoryOverride = dir.wstring();

	recording::RecordingEnvelope envelope;
	ASSERT_TRUE(envelope.Open(options));
	ASSERT_TRUE(envelope.IsOpen());
	EXPECT_TRUE(envelope.WriteRow("time_ms,x"));
	EXPECT_TRUE(envelope.WriteRow("0,1.0"));
	EXPECT_TRUE(envelope.WriteAnnotation(2.0, "marker: k=v"));
	EXPECT_TRUE(envelope.WriteRow("16,1.5"));
	const fs::path written = envelope.Path();
	envelope.Close();
	EXPECT_FALSE(envelope.IsOpen());

	const auto lines = ReadLines(written);
	ASSERT_GE(lines.size(), 9u);
	EXPECT_EQ("# sample_rec_v1", lines[0]);
	EXPECT_EQ("# columns=time_ms,x", lines[3]);
	EXPECT_EQ(0u, lines[4].rfind("# dev_auto_recording=enabled", 0));
	EXPECT_EQ("time_ms,x", lines[5]);
	EXPECT_EQ("0,1.0", lines[6]);
	EXPECT_EQ("# [2.000] marker: k=v", lines[7]);
	EXPECT_EQ("16,1.5", lines[8]);

	fs::remove_all(dir);
}
#endif // WKOPENVR_BUILD_IS_DEV

TEST(RecordingEnvelopeTest, PruneAtOpenSparesSubdirectories)
{
	const fs::path dir = MakeTestDir("prune");
	const fs::path corpus = dir / "corpus";
	fs::create_directories(corpus);

	const auto now = fs::file_time_type::clock::now();
	int age = 10;
	for (const char* name : {"sample_rec.2026-01-01T00-00-01.csv", "sample_rec.2026-01-01T00-00-02.csv",
	                         "sample_rec.2026-01-01T00-00-03.csv"}) {
		const fs::path p = dir / name;
		std::ofstream(p) << std::string(100, 'x');
		fs::last_write_time(p, now - std::chrono::seconds(age--));
	}
	const fs::path pinned = corpus / "sample_rec.2026-01-01T00-00-01.csv";
	std::ofstream(pinned) << std::string(100, 'x');

	recording::RetentionPolicy policy;
	policy.maxFiles = 1;
	policy.maxTotalBytes = 1024;
	const auto result = recording::PruneRecordings(L"sample_rec", L"csv", policy, dir.wstring());

	EXPECT_EQ(3u, result.totalFiles);
	EXPECT_EQ(2u, result.deletedFiles);
	EXPECT_EQ(1u, result.keptFiles);
	EXPECT_TRUE(fs::exists(dir / "sample_rec.2026-01-01T00-00-03.csv")); // newest kept
	EXPECT_FALSE(fs::exists(dir / "sample_rec.2026-01-01T00-00-01.csv"));
	EXPECT_TRUE(fs::exists(pinned)); // subdirectory untouched

	fs::remove_all(dir);
}

#if WKOPENVR_BUILD_IS_DEV
TEST(RecordingEnvelopeTest, ByteCapStopsRowsWithAnnotation)
{
	const fs::path dir = MakeTestDir("bytecap");

	recording::EnvelopeOptions options;
	options.prefix = L"sample_rec";
	options.extension = L"csv";
	options.schemaBanner = "sample_rec_v1";
	options.directoryOverride = dir.wstring();
	options.maxFileBytes = 64; // banner alone exceeds this

	recording::RecordingEnvelope envelope;
	ASSERT_TRUE(envelope.Open(options));
	EXPECT_FALSE(envelope.WriteRow("0,1.0"));
	EXPECT_TRUE(envelope.ByteCapReached());
	EXPECT_FALSE(envelope.WriteRow("16,1.5"));
	const fs::path written = envelope.Path();
	envelope.Close();

	bool sawStop = false;
	bool sawRow = false;
	for (const auto& line : ReadLines(written)) {
		if (line.find("recording_stopped: byte_cap") != std::string::npos) sawStop = true;
		if (line == "0,1.0" || line == "16,1.5") sawRow = true;
	}
	EXPECT_TRUE(sawStop);
	EXPECT_FALSE(sawRow);

	fs::remove_all(dir);
}
#else
TEST(RecordingEnvelopeTest, ReleaseChannelStubsOpen)
{
	const fs::path dir = MakeTestDir("releasestub");

	recording::EnvelopeOptions options;
	options.prefix = L"sample_rec";
	options.extension = L"csv";
	options.schemaBanner = "sample_rec_v1";
	options.directoryOverride = dir.wstring();

	recording::RecordingEnvelope envelope;
	EXPECT_FALSE(envelope.Open(options));
	EXPECT_FALSE(envelope.IsOpen());
	EXPECT_FALSE(envelope.WriteRow("0,1.0"));
	EXPECT_TRUE(fs::is_empty(dir)); // release builds must not create recording files

	fs::remove_all(dir);
}
#endif // WKOPENVR_BUILD_IS_DEV
