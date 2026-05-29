#pragma once

#include "AdbController.h"
#include "AdbSetupWizard.h"
#include "BuildChannel.h"
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
#if WKOPENVR_BUILD_IS_DEV
    bool HasDevTools() const override { return true; }
    void DrawDevTools(openvr_pair::overlay::ShellContext& context) override;
#endif

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
#if WKOPENVR_BUILD_IS_DEV
    std::string lastWifiEndpoint_;
    std::string lastDevicesOutput_;
#endif

    void SetStatus(std::string text, bool warn = false);
    void DrawSetup(openvr_pair::overlay::ShellContext& context);
    void DrawBoundaryGuide();
    void DrawCompanion(openvr_pair::overlay::ShellContext& context);
    void DrawLaunchTargetPicker();
    void RefreshPackages();
#if WKOPENVR_BUILD_IS_DEV
    void DrawAdbDevTools();
#endif
};
