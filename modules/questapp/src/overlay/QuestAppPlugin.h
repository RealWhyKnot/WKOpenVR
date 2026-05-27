#pragma once

#include "AdbController.h"
#include "AdbSetupWizard.h"
#include "FeaturePlugin.h"
#include "QuestAppConfig.h"
#include "QuestAppCatalog.h"

#include <string>
#include <vector>

class QuestAppPlugin final : public openvr_pair::overlay::FeaturePlugin
{
public:
    const char* Name() const override { return "Quest App"; }
    const char* FlagFileName() const override { return "enable_questapp.flag"; }
    const char* PipeName() const override { return ""; }

    void OnStart(openvr_pair::overlay::ShellContext& context) override;
    void DrawTab(openvr_pair::overlay::ShellContext& context) override;
    void DrawLogsSection(openvr_pair::overlay::ShellContext& context) override;

private:
    wkopenvr::questapp::QuestAppConfig cfg_;
    wkopenvr::questapp::QuestCompanionSettings companionSettings_;
    AdbController adb_;
    wkopenvr::questapp::AdbSetupWizard wizard_{adb_};
    std::string status_;
    bool statusWarn_ = false;
    bool showAllPackages_ = false;
    bool packagesLoaded_ = false;
    bool companionSettingsLoaded_ = false;
    std::vector<wkopenvr::questapp::QuestLaunchTarget> detectedPackages_;

    void SetStatus(std::string text, bool warn = false);
    void DrawSetup(openvr_pair::overlay::ShellContext& context);
    void DrawBoundaryGuide();
    void DrawCompanion(openvr_pair::overlay::ShellContext& context);
    void DrawLaunchTargetPicker();
    void RefreshPackages();
};
