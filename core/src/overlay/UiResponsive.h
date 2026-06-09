#pragma once

#include "UiResponsiveLogic.h"

#include <imgui.h>

// Responsive layout primitives. These adapt to the live content width
// (GetContentRegionAvail), so the same call site reads well on the fixed
// 1200x780 VR dashboard and on a narrow, resized desktop window.

namespace openvr_pair::overlay::ui {

// A grid of equal-width columns that collapses toward a single column as the
// window narrows. Construct it, fill the first cell, then call Next() before
// each following cell. Cells wrap to new rows automatically.
//
//   ResponsiveColumnsScope cols("metrics", 3, 220.0f);
//   if (cols) {
//       DrawCard("A", ...); cols.Next();
//       DrawCard("B", ...); cols.Next();
//       DrawCard("C", ...);
//   }
struct ResponsiveColumnsScope
{
	ResponsiveColumnsScope(const char* id, int requestedColumns, float minColumnWidth);
	~ResponsiveColumnsScope();

	int columns() const { return cols_; }
	void Next();
	explicit operator bool() const { return open_; }

	ResponsiveColumnsScope(const ResponsiveColumnsScope&) = delete;
	ResponsiveColumnsScope& operator=(const ResponsiveColumnsScope&) = delete;

private:
	int cols_ = 1;
	int cellIndex_ = 0;
	bool open_ = false;
};

// A horizontal run of similarly-sized items (chips, badges, small buttons)
// that wraps to the next line instead of overflowing. Call Item() immediately
// before drawing each item.
//
//   FlowRowScope flow;
//   for (const auto& tag : tags) { flow.Item(); StatusBadge(tag.c_str(), tone); }
struct FlowRowScope
{
	explicit FlowRowScope(float spacing = -1.0f);
	void Item();

	FlowRowScope(const FlowRowScope&) = delete;
	FlowRowScope& operator=(const FlowRowScope&) = delete;

private:
	float spacing_;
	float lineStartX_ = 0.0f;
	float rightEdge_ = 0.0f;
	bool first_ = true;
};

} // namespace openvr_pair::overlay::ui
