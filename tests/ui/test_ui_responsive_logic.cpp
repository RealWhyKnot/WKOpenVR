#include <gtest/gtest.h>

#include "UiResponsiveLogic.h"

namespace ui = openvr_pair::overlay::ui;

TEST(ResponsiveColumns, CollapsesAsWidthShrinks)
{
	// Three 200px columns: full width fits all three, half drops to two, a
	// narrow window collapses to one.
	EXPECT_EQ(3, ui::ComputeResponsiveColumnCount(700.0f, 3, 200.0f));
	EXPECT_EQ(2, ui::ComputeResponsiveColumnCount(500.0f, 3, 200.0f));
	EXPECT_EQ(1, ui::ComputeResponsiveColumnCount(150.0f, 3, 200.0f));
}

TEST(ResponsiveColumns, NeverExceedsRequestAndNeverBelowOne)
{
	EXPECT_EQ(3, ui::ComputeResponsiveColumnCount(5000.0f, 3, 200.0f));
	EXPECT_EQ(1, ui::ComputeResponsiveColumnCount(10.0f, 3, 200.0f));
	EXPECT_EQ(1, ui::ComputeResponsiveColumnCount(700.0f, 0, 200.0f));
	EXPECT_EQ(1, ui::ComputeResponsiveColumnCount(700.0f, 1, 200.0f));
}

TEST(ResponsiveColumns, NoWidthSignalKeepsRequest)
{
	// Before the first frame there is no usable width; honor the request.
	EXPECT_EQ(3, ui::ComputeResponsiveColumnCount(0.0f, 3, 200.0f));
	EXPECT_EQ(3, ui::ComputeResponsiveColumnCount(700.0f, 3, 0.0f));
}

TEST(FlowWrap, WrapsOnlyWhenItemCrossesBoundary)
{
	const float rightEdge = 400.0f;
	const float spacing = 8.0f;
	// Item ends at 300, next is 50 wide: 300 + 8 + 50 = 358 <= 400 -> stays.
	EXPECT_FALSE(ui::FlowShouldWrap(300.0f, 50.0f, rightEdge, spacing));
	// Item ends at 370, next is 50 wide: 370 + 8 + 50 = 428 > 400 -> wraps.
	EXPECT_TRUE(ui::FlowShouldWrap(370.0f, 50.0f, rightEdge, spacing));
}
