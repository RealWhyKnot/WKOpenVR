#include "Win32Paths.h"

#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>

namespace common = openvr_pair::common;
namespace fs = std::filesystem;

namespace {

fs::path MakeTestDir(const char* tag)
{
	const fs::path dir = fs::temp_directory_path() / (std::string("wkopenvr_paths_") + tag);
	fs::remove_all(dir);
	fs::create_directories(dir);
	return dir;
}

std::string ReadAll(const fs::path& path)
{
	std::ifstream in(path, std::ios::binary);
	std::stringstream ss;
	ss << in.rdbuf();
	return ss.str();
}

} // namespace

TEST(WriteFileAtomic, WritesBodyAndLeavesNoTemp)
{
	const fs::path dir = MakeTestDir("write");
	const fs::path file = dir / "a.txt";
	ASSERT_TRUE(common::WriteFileAtomic(file.wstring(), "hello\n"));
	EXPECT_EQ(ReadAll(file), "hello\n");
	EXPECT_FALSE(fs::exists(dir / "a.txt.tmp"));
	fs::remove_all(dir);
}

TEST(WriteFileAtomic, ReplacesExistingAndAcceptsEmptyBody)
{
	const fs::path dir = MakeTestDir("replace");
	const fs::path file = dir / "a.txt";
	ASSERT_TRUE(common::WriteFileAtomic(file.wstring(), "one"));
	ASSERT_TRUE(common::WriteFileAtomic(file.wstring(), "two", true));
	EXPECT_EQ(ReadAll(file), "two");
	ASSERT_TRUE(common::WriteFileAtomic(file.wstring(), ""));
	EXPECT_EQ(ReadAll(file), "");
	EXPECT_FALSE(fs::exists(dir / "a.txt.tmp"));
	fs::remove_all(dir);
}

TEST(WriteFileAtomic, FailsWhenDirectoryMissingAndKeepsLastError)
{
	const fs::path dir = MakeTestDir("missing");
	const fs::path file = dir / "nope" / "a.txt";
	EXPECT_FALSE(common::WriteFileAtomic(file.wstring(), "x"));
	EXPECT_EQ(GetLastError(), static_cast<DWORD>(ERROR_PATH_NOT_FOUND));
	EXPECT_FALSE(fs::exists(file));
	fs::remove_all(dir);
}
