#include "PowerShellCommand.h"

#include <gtest/gtest.h>

namespace {

using openvr_pair::common::EncodePowerShellCommand;
using openvr_pair::common::EncodePowerShellCommandUtf8;
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

} // namespace
