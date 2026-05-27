#include "QuestAppPlugin.h"

#include "QuestAppInstaller.h"
#include "ShellContext.h"
#include "UiHelpers.h"

#include <imgui.h>

#include <memory>
#include <utility>

using wkopenvr::questapp::CuratedLaunchTargets;
using wkopenvr::questapp::LaunchTargetDisplayName;
using wkopenvr::questapp::QuestLaunchTarget;

void QuestAppPlugin::OnStart(openvr_pair::overlay::ShellContext&)
{
    cfg_ = wkopenvr::questapp::LoadQuestAppConfig();
}

void QuestAppPlugin::SetStatus(std::string text, bool warn)
{
    status_ = std::move(text);
    statusWarn_ = warn;
}

void QuestAppPlugin::DrawTab(openvr_pair::overlay::ShellContext& context)
{
    openvr_pair::overlay::ui::TabBarScope tabs("questapp_tabs");
    if (tabs) {
        openvr_pair::overlay::ui::DrawTabItem("Setup", [&] { DrawSetup(context); });
        openvr_pair::overlay::ui::DrawTabItem("Boundary", [&] { DrawBoundaryGuide(); });
        openvr_pair::overlay::ui::DrawTabItem("Companion", [&] { DrawCompanion(context); });
    }

    if (!status_.empty()) {
        ImGui::Spacing();
        const auto& pal = openvr_pair::overlay::ui::GetPalette();
        if (statusWarn_) {
            openvr_pair::overlay::ui::DrawBanner(
                "Quest App", status_.c_str(),
                pal.bannerWarnBg, pal.bannerWarnTitle, pal.bannerWarnDetail);
        } else {
            openvr_pair::overlay::ui::DrawInfoBanner("Quest App", status_.c_str());
        }
    }
}

void QuestAppPlugin::DrawSetup(openvr_pair::overlay::ShellContext& context)
{
    const auto& pal = openvr_pair::overlay::ui::GetPalette();
    openvr_pair::overlay::ui::DrawTextWrapped(
        "Quest App uses ADB only for one-time setup: install platform-tools, authorize USB debugging, then install the headset companion.");
    ImGui::Spacing();

    openvr_pair::overlay::ui::DrawSectionHeading("ADB platform-tools");
    const bool adbInstalled = wkopenvr::questapp::PlatformToolsInstalled();
    openvr_pair::overlay::ui::DrawStatusDot(adbInstalled ? pal.dotOk : pal.dotPending);
    ImGui::SameLine();
    ImGui::TextUnformatted(adbInstalled ? "Installed" : "Not installed");
    ImGui::SameLine();
    if (ImGui::Button(adbInstalled ? "Reinstall ADB" : "Install ADB")) {
        SetStatus("Installing platform-tools...");
        const auto result = wkopenvr::questapp::InstallPlatformTools(context);
        adb_.RefreshResolvedBinaryPath();
        wizard_.CheckPlatformTools();
        SetStatus(result.message, !result.ok);
    }

    ImGui::Spacing();
    openvr_pair::overlay::ui::DrawSectionHeading("Enable Developer Mode");
    openvr_pair::overlay::ui::DrawTextWrapped(
        "In the Meta Horizon mobile app: Menu > Devices > your headset > Headset settings > Developer Mode > On.");
    if (ImGui::Button("Developer Mode is on")) {
        const auto result = wizard_.MarkDeveloperModeReady();
        SetStatus(result.detail, result.status != wkopenvr::questapp::StepStatus::Passed);
    }

    ImGui::Spacing();
    openvr_pair::overlay::ui::DrawSectionHeading("Authorize USB debugging");
    openvr_pair::overlay::ui::DrawTextWrapped(
        "Plug in USB, unlock the headset, and accept the USB debugging prompt inside the headset.");
    if (ImGui::Button("Check USB authorization")) {
        const auto result = wizard_.CheckUsbAuthorization();
        SetStatus(result.detail, result.status != wkopenvr::questapp::StepStatus::Passed);
    }
}

void QuestAppPlugin::DrawBoundaryGuide()
{
    openvr_pair::overlay::ui::DrawTextWrapped(
        "Boundary disable is a manual headset setting. WKOpenVR does not use ADB to pause Guardian because that path is temporary and does not survive headset reboot reliably.");
    ImGui::Spacing();
    openvr_pair::overlay::ui::DrawSectionHeading("Headset steps");
    ImGui::BulletText("Open Settings > Developer.");
    ImGui::BulletText("Turn Physical space features off.");
    ImGui::BulletText("Re-enable it when you want Quest Boundary and Home passthrough behavior back.");
    ImGui::Spacing();
    openvr_pair::overlay::ui::DrawTextWrapped(
        "Reported side effects include disabled or limited Quest Home passthrough, double-tap passthrough, space setup, and some mixed-reality features.");
}

void QuestAppPlugin::DrawCompanion(openvr_pair::overlay::ShellContext& context)
{
    const auto& pal = openvr_pair::overlay::ui::GetPalette();
    const bool apkAvailable = wkopenvr::questapp::CompanionApkAvailable(context);
    const bool canControl = wkopenvr::questapp::CanContactCompanion(cfg_);
    const bool needsReinstall = wkopenvr::questapp::NeedsCompanionReinstall(cfg_);

    openvr_pair::overlay::ui::DrawSectionHeading("Headset companion");
    openvr_pair::overlay::ui::DrawStatusDot(canControl ? pal.dotOk : pal.dotPending);
    ImGui::SameLine();
    if (canControl) {
        ImGui::TextUnformatted("Paired");
    } else if (needsReinstall) {
        ImGui::TextUnformatted("Install key missing");
    } else {
        ImGui::TextUnformatted("Not paired");
    }

    if (needsReinstall) {
        ImGui::Spacing();
        openvr_pair::overlay::ui::DrawBanner(
            "Reinstall required",
            "The local install key is missing. The headset app only responds to that key, so WKOpenVR cannot safely control the existing install. Reinstall the companion to create a new key.",
            pal.bannerWarnBg, pal.bannerWarnTitle, pal.bannerWarnDetail);
    }

    ImGui::Spacing();
    if (!apkAvailable) {
        ImGui::TextDisabled("Companion APK is not bundled in this build yet.");
    }
    const char* installLabel = needsReinstall
        ? "Repair companion install"
        : (canControl ? "Reinstall companion" : "Install companion");
    if (ImGui::Button(installLabel)) {
        const auto result = wkopenvr::questapp::InstallCompanionApp(context, adb_, cfg_);
        companionSettingsLoaded_ = false;
        SetStatus(result.message, !result.ok);
    }
    ImGui::SameLine();
    if (ImGui::Button("Uninstall companion")) {
        const auto result = wkopenvr::questapp::UninstallCompanionApp(adb_, cfg_);
        companionSettings_ = {};
        companionSettingsLoaded_ = false;
        SetStatus(result.message, !result.ok);
    }

    ImGui::Spacing();
    DrawLaunchTargetPicker();
}

void QuestAppPlugin::DrawLaunchTargetPicker()
{
    openvr_pair::overlay::ui::DrawSectionHeading("Boot auto-launch");
    if (ImGui::Button("Load settings from headset")) {
        const auto query = wkopenvr::questapp::QueryCompanionSettings(adb_, cfg_);
        if (query.result.ok) {
            companionSettings_ = query.settings;
            companionSettingsLoaded_ = true;
        }
        SetStatus(query.result.message, !query.result.ok);
    }
    ImGui::SameLine();
    ImGui::TextDisabled(companionSettingsLoaded_ ? "Loaded from companion" : "Not loaded");

    ImGui::Spacing();
    bool autoLaunch = companionSettings_.autoLaunchEnabled;
    if (ImGui::Checkbox("Auto-launch selected app when the headset boots", &autoLaunch)) {
        companionSettings_.autoLaunchEnabled = autoLaunch;
    }

    ImGui::Spacing();
    ImGui::TextUnformatted("Curated apps");
    for (const auto& target : CuratedLaunchTargets()) {
        ImGui::PushID(target.packageName.c_str());
        const bool selected =
            companionSettings_.selectedPackage == target.packageName &&
            companionSettings_.selectedActivity == target.activityName;
        if (ImGui::RadioButton(LaunchTargetDisplayName(target).c_str(), selected)) {
            companionSettings_.selectedPackage = target.packageName;
            companionSettings_.selectedActivity = target.activityName;
        }
        ImGui::PopID();
    }

    ImGui::Spacing();
    if (ImGui::Button(packagesLoaded_ ? "Refresh installed apps" : "Show all installed apps")) {
        RefreshPackages();
    }
    if (packagesLoaded_) {
        ImGui::SameLine();
        ImGui::Checkbox("Show package list", &showAllPackages_);
    }
    if (packagesLoaded_ && showAllPackages_) {
        ImGui::Spacing();
        ImGui::BeginChild("questapp_all_packages", ImVec2(0.0f, 220.0f), true);
        for (const auto& target : detectedPackages_) {
            ImGui::PushID(target.packageName.c_str());
            const bool selected =
                companionSettings_.selectedPackage == target.packageName &&
                companionSettings_.selectedActivity == target.activityName;
            if (ImGui::RadioButton(target.packageName.c_str(), selected)) {
                companionSettings_.selectedPackage = target.packageName;
                companionSettings_.selectedActivity = target.activityName;
            }
            ImGui::PopID();
        }
        if (detectedPackages_.empty()) {
            ImGui::TextDisabled("No packages listed. Check USB ADB authorization.");
        }
        ImGui::EndChild();
    }

    ImGui::Spacing();
    if (ImGui::Button("Apply to headset companion")) {
        const auto result = wkopenvr::questapp::SyncCompanionConfig(adb_, cfg_, companionSettings_);
        SetStatus(result.message, !result.ok);
    }
    ImGui::SameLine();
    if (wkopenvr::questapp::NeedsCompanionReinstall(cfg_)) {
        ImGui::TextDisabled("Reinstall required before settings can be sent.");
    } else {
        ImGui::TextDisabled("Requires USB ADB for this setup step.");
    }
}

void QuestAppPlugin::RefreshPackages()
{
    detectedPackages_ = wkopenvr::questapp::QueryInstalledPackages(adb_);
    packagesLoaded_ = true;
    showAllPackages_ = true;
    SetStatus("Package list refreshed.", false);
}

void QuestAppPlugin::DrawLogsSection(openvr_pair::overlay::ShellContext&)
{
    openvr_pair::overlay::ui::DrawTextWrapped(
        "Quest App logs use the shared WKOpenVR log folder. ADB command output redacts device addresses, pairing codes, and the companion install key.");
}

namespace openvr_pair::overlay {

std::unique_ptr<FeaturePlugin> CreateQuestAppPlugin()
{
    return std::make_unique<QuestAppPlugin>();
}

} // namespace openvr_pair::overlay
