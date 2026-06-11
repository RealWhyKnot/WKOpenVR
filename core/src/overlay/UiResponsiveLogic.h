#pragma once

// Pure layout math for the responsive UI primitives. This header is
// deliberately free of ImGui so it can be unit-tested directly and reused by
// any caller that needs the same decisions without a live ImGui context.

namespace openvr_pair::overlay::ui {

// Choose how many columns actually fit. Starts from the requested count and
// drops it until each column has at least minColumnWidth, never below 1 and
// never above the request. A non-positive width or min returns the request
// unchanged (caller has no width signal yet -- e.g. first frame).
inline int ComputeResponsiveColumnCount(float availWidth, int requestedColumns, float minColumnWidth)
{
	if (requestedColumns < 1) return 1;
	if (availWidth <= 0.0f || minColumnWidth <= 0.0f) return requestedColumns;

	int fit = static_cast<int>(availWidth / minColumnWidth);
	if (fit < 1) fit = 1;
	if (fit > requestedColumns) fit = requestedColumns;
	return fit;
}

// Decide whether the next item must wrap to a new line in a flow row. Given
// the right edge of the previously placed item, the upcoming item's width, the
// line's right boundary, and the inter-item spacing: wrap when the item would
// cross the boundary.
inline bool FlowShouldWrap(float prevItemRightEdge, float nextItemWidth, float rightEdge, float spacing)
{
	return (prevItemRightEdge + spacing + nextItemWidth) > rightEdge;
}

// Is the desktop window actually showing pixels to the user? A minimized window
// reports a 0x0 framebuffer on Windows, but check the iconified flag too so the
// caller never builds a UI frame for a window nobody can see.
inline bool ComputeDesktopVisible(int framebufferWidth, int framebufferHeight, bool iconified)
{
	return framebufferWidth > 0 && framebufferHeight > 0 && !iconified;
}

// The overlay only needs to rebuild and rasterise its ImGui frame when a surface
// can actually be seen: either the in-VR dashboard overlay is up, or the desktop
// window is visible. When both are false the loop runs its background heartbeat
// (VR tick, plugin ticks, perf sampling) and skips the per-frame UI rebuild.
inline bool ShouldRenderUi(bool vrSurfaceVisible, bool desktopVisible)
{
	return vrSurfaceVisible || desktopVisible;
}

} // namespace openvr_pair::overlay::ui
