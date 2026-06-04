#pragma once

class FacetrackingPlugin;

namespace facetracking::ui {
// Draws the FaceTracking contribution inside the umbrella's global Logs tab.
// The umbrella wraps this in a collapsing header named after the plugin so
// this function should emit its content without adding its own heading.
void DrawLogsSection(FacetrackingPlugin& plugin);
} // namespace facetracking::ui
