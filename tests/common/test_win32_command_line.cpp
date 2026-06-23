#include "Win32CommandLine.h"

#include <gtest/gtest.h>

namespace {

using openvr_pair::common::QuoteCommandLineArg;

TEST(Win32CommandLine, QuotesArguments)
{
	EXPECT_EQ(QuoteCommandLineArg(L"simple"), L"\"simple\"");
	EXPECT_EQ(QuoteCommandLineArg(L""), L"\"\"");
	EXPECT_EQ(QuoteCommandLineArg(L"C:\\Path With Spaces\\tool.ps1"), L"\"C:\\Path With Spaces\\tool.ps1\"");
	EXPECT_EQ(QuoteCommandLineArg(L"C:\\ends\\with\\slash\\"), L"\"C:\\ends\\with\\slash\\\\\"");
	EXPECT_EQ(QuoteCommandLineArg(L"a \"quoted\" value"), L"\"a \\\"quoted\\\" value\"");
	EXPECT_EQ(QuoteCommandLineArg(L"C:\\path\\\"quoted\""), L"\"C:\\path\\\\\\\"quoted\\\"\"");
}

} // namespace
