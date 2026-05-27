#pragma once

#include "AdbController.h"
#include "QuestAppCatalog.h"
#include "QuestAppConfig.h"
#include "ShellContext.h"

#include <string>
#include <vector>

namespace wkopenvr::questapp {

struct OperationResult {
    bool ok = false;
    std::string message;
};

struct SettingsQueryResult {
    OperationResult result;
    QuestCompanionSettings settings;
};

std::wstring QuestAppDataDir(bool create = true);
std::wstring PlatformToolsDir(bool create = true);
std::wstring CompanionApkPath(const openvr_pair::overlay::ShellContext& context);
std::wstring InstallPlatformToolsScriptPath(const openvr_pair::overlay::ShellContext& context);

bool PlatformToolsInstalled();
bool CompanionApkAvailable(const openvr_pair::overlay::ShellContext& context);

OperationResult InstallPlatformTools(const openvr_pair::overlay::ShellContext& context);
OperationResult InstallCompanionApp(
    const openvr_pair::overlay::ShellContext& context,
    AdbController& adb,
    QuestAppConfig& cfg);
OperationResult SyncCompanionConfig(
    AdbController& adb,
    const QuestAppConfig& cfg,
    const QuestCompanionSettings& settings);
SettingsQueryResult QueryCompanionSettings(AdbController& adb, const QuestAppConfig& cfg);
OperationResult UninstallCompanionApp(AdbController& adb, QuestAppConfig& cfg);

std::vector<QuestLaunchTarget> QueryInstalledPackages(AdbController& adb);

} // namespace wkopenvr::questapp
