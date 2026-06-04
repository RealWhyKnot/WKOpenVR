#pragma once

#include <memory>
#include <vector>

namespace openvr_pair::overlay {

class FeaturePlugin;
struct ShellContext;

void DrawFeatureTab(ShellContext& context, FeaturePlugin& plugin);
void DrawLogsTab(ShellContext& context, std::vector<std::unique_ptr<FeaturePlugin>>& plugins);
void DrawModulesTab(ShellContext& context, std::vector<std::unique_ptr<FeaturePlugin>>& plugins);
void DrawThemesTab(ShellContext& context);
void DrawShellWindow(ShellContext& context, std::vector<std::unique_ptr<FeaturePlugin>>& plugins);

} // namespace openvr_pair::overlay
