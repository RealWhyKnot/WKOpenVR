#include "PowerShellCommand.h"

#include <gtest/gtest.h>

namespace {

using openvr_pair::common::EncodePowerShellCommand;
using openvr_pair::common::EncodePowerShellCommandUtf8;
using openvr_pair::common::QuoteCommandLineArg;
using openvr_pair::common::QuotePowerShellLiteral;

TEST(PowerShellCommand, QuotesPowerShellLiteral)
{
	EXPECT_EQ(QuotePowerShellLiteral(L"C:\\Users\\Name"), L"'C:\\Users\\Name'");
	EXPECT_EQ(QuotePowerShellLiteral(L"can't stop"), L"'can''t stop'");
	EXPECT_EQ(QuotePowerShellLiteral(L""), L"''");
}

TEST(PowerShellCommand, EncodesUtf16LeScript)
{
	EXPECT_EQ(EncodePowerShellCommand(L""), L"");
	EXPECT_EQ(EncodePowerShellCommand(L"exit 0"), L"ZQB4AGkAdAAgADAA");
	EXPECT_EQ(EncodePowerShellCommand(L"$x='a'"), L"JAB4AD0AJwBhACcA");
	EXPECT_EQ(EncodePowerShellCommandUtf8(L"exit 0"), "ZQB4AGkAdAAgADAA");
}

TEST(PowerShellCommand, QuotesWindowsCommandLineArguments)
{
	EXPECT_EQ(QuoteCommandLineArg(L"simple"), L"\"simple\"");
	EXPECT_EQ(QuoteCommandLineArg(L""), L"\"\"");
	EXPECT_EQ(QuoteCommandLineArg(L"C:\\Path With Spaces\\tool.ps1"), L"\"C:\\Path With Spaces\\tool.ps1\"");
	EXPECT_EQ(QuoteCommandLineArg(L"C:\\ends\\with\\slash\\"), L"\"C:\\ends\\with\\slash\\\\\"");
	EXPECT_EQ(QuoteCommandLineArg(L"a \"quoted\" value"), L"\"a \\\"quoted\\\" value\"");
	EXPECT_EQ(QuoteCommandLineArg(L"C:\\path\\\"quoted\""), L"\"C:\\path\\\\\\\"quoted\\\"\"");
}

} // namespace
