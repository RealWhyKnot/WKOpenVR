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

TEST(DesktopVisible, NeedsPixelsAndNotMinimized)
{
	// A normal window with a real framebuffer and not iconified shows pixels.
	EXPECT_TRUE(ui::ComputeDesktopVisible(1200, 780, false));
	// Minimized reports a real size on some shells but is not visible.
	EXPECT_FALSE(ui::ComputeDesktopVisible(1200, 780, true));
	// A 0x0 framebuffer (minimized on Windows) is not visible either way.
	EXPECT_FALSE(ui::ComputeDesktopVisible(0, 0, false));
	EXPECT_FALSE(ui::ComputeDesktopVisible(0, 780, false));
	EXPECT_FALSE(ui::ComputeDesktopVisible(1200, 0, false));
}

TEST(ShouldRenderUi, RendersWhenAnySurfaceIsVisible)
{
	// In-VR overlay up: render regardless of the desktop window state.
	EXPECT_TRUE(ui::ShouldRenderUi(true, false));
	EXPECT_TRUE(ui::ShouldRenderUi(true, true));
	// No VR surface but the desktop window is showing: still render.
	EXPECT_TRUE(ui::ShouldRenderUi(false, true));
	// Background case -- dashboard closed and window minimized: skip the build.
	EXPECT_FALSE(ui::ShouldRenderUi(false, false));
}
