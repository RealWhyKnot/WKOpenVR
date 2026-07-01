#include "DevTuning.h"

#include <gtest/gtest.h>

namespace dt = openvr_pair::common::devtuning;

namespace {

// Each test sets a known snapshot first; the store is process-global.
TEST(DevTuning, DefaultsWhenEmpty)
{
	dt::ApplyTextForTest("");
	EXPECT_DOUBLE_EQ(dt::Get("anything", 1.25), 1.25);
	EXPECT_DOUBLE_EQ(dt::Get(nullptr, 7.0), 7.0);
}

TEST(DevTuning, ParsesKeyValuePairs)
{
	dt::ApplyTextForTest("a = 1.5\nb=2\n  c  =  -3.25  \n");
	EXPECT_DOUBLE_EQ(dt::Get("a", 0.0), 1.5);
	EXPECT_DOUBLE_EQ(dt::Get("b", 0.0), 2.0);
	EXPECT_DOUBLE_EQ(dt::Get("c", 0.0), -3.25);
	EXPECT_DOUBLE_EQ(dt::Get("missing", 9.0), 9.0);
}

TEST(DevTuning, IgnoresCommentsAndBlankLines)
{
	dt::ApplyTextForTest("# whole line\n\n; also comment\nkept = 4  # trailing\nc2 = 5 ; trailing2\n");
	EXPECT_DOUBLE_EQ(dt::Get("kept", 0.0), 4.0);
	EXPECT_DOUBLE_EQ(dt::Get("c2", 0.0), 5.0);
	// A commented-out key must not be present.
	dt::ApplyTextForTest("# foo = 1\n");
	EXPECT_DOUBLE_EQ(dt::Get("foo", 42.0), 42.0);
}

TEST(DevTuning, SkipsMalformedValuesKeepingDefault)
{
	dt::ApplyTextForTest("x = notanumber\ny =\n= 3\nz = 6\n");
	EXPECT_DOUBLE_EQ(dt::Get("x", 5.0), 5.0); // unparseable -> default
	EXPECT_DOUBLE_EQ(dt::Get("y", 5.0), 5.0); // empty value -> default
	EXPECT_DOUBLE_EQ(dt::Get("z", 0.0), 6.0); // valid line still parsed
}

TEST(DevTuning, ReapplyReplacesSnapshot)
{
	dt::ApplyTextForTest("k = 1\n");
	EXPECT_DOUBLE_EQ(dt::Get("k", 0.0), 1.0);
	dt::ApplyTextForTest("k = 2\n");
	EXPECT_DOUBLE_EQ(dt::Get("k", 0.0), 2.0);
	dt::ApplyTextForTest(""); // cleared -> back to default
	EXPECT_DOUBLE_EQ(dt::Get("k", 0.0), 0.0);
}

} // namespace
