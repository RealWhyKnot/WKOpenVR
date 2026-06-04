#pragma once

class CaptionsPlugin;

namespace captions::ui {

// Draws the main captions settings tab content.
// Called by CaptionsPlugin::DrawTab() after the tab bar is set up.
void DrawCaptionsTab(CaptionsPlugin& plugin);

} // namespace captions::ui
