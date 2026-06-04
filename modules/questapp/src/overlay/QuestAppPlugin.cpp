#include "QuestAppPlugin.h"

#include "QuestAppInstaller.h"
#include "ShellContext.h"
#include "UiHelpers.h"

#include <imgui.h>

#include <chrono>
#include <memory>
#include <regex>
#include <utility>

using wkopenvr::questapp::CuratedLaunchTargets;
using wkopenvr::questapp::LaunchTargetDisplayName;
using wkopenvr::questapp::QuestLaunchTarget;

#if WKOPENVR_BUILD_IS_DEV
namespace {

std::string TrimAscii(std::string s)
{
	while (!s.empty() && static_cast<unsigned char>(s.back()) <= ' ') {
		s.pop_back();
	}
	size_t start = 0;
	while (start < s.size() && static_cast<unsigned char>(s[start]) <= ' ') {
		++start;
	}
	return s.substr(start);
}

std::string ExtractWifiAdbEndpoint(const std::string& routeOutput, int port)
{
	std::smatch match;
	const std::regex srcPattern(R"(\bsrc\s+(\d{1,3}(?:\.\d{1,3}){3})\b)");
	if (std::regex_search(routeOutput, match, srcPattern) && match.size() >= 2) {
		return match[1].str() + ":" + std::to_string(port);
	}
	return {};
}

std::string PreferredAdbSerialFromDevices(const std::string& devicesOutput)
{
	std::string serial = wkopenvr::questapp::FindAuthorizedUsbQuestSerial(devicesOutput);
	if (!serial.empty()) return serial;
	return wkopenvr::questapp::FindAuthorizedWifiQuestSerial(devicesOutput);
}

} // namespace
#endif

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
			openvr_pair::overlay::ui::DrawBanner("Quest App", status_.c_str(), pal.bannerWarnBg, pal.bannerWarnTitle,
			                                     pal.bannerWarnDetail);
		}
		else {
			openvr_pair::overlay::ui::DrawInfoBanner("Quest App", status_.c_str());
		}
	}
}

void QuestAppPlugin::DrawSetup(openvr_pair::overlay::ShellContext& context)
{
	const auto& pal = openvr_pair::overlay::ui::GetPalette();
	openvr_pair::overlay::ui::DrawTextWrapped("Quest App uses ADB only for one-time setup: install platform-tools, "
	                                          "authorize USB debugging, then install the headset companion.");
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
	    "Boundary disable is a manual headset setting. WKOpenVR does not use ADB to pause Guardian because that path "
	    "is temporary and does not survive headset reboot reliably.");
	ImGui::Spacing();
	openvr_pair::overlay::ui::DrawSectionHeading("Headset steps");
	ImGui::BulletText("Open Settings > Developer.");
	ImGui::BulletText("Turn Physical space features off.");
	ImGui::BulletText("Re-enable it when you want Quest Boundary and Home passthrough behavior back.");
	ImGui::Spacing();
	openvr_pair::overlay::ui::DrawTextWrapped(
	    "Reported side effects include disabled or limited Quest Home passthrough, double-tap passthrough, space "
	    "setup, and some mixed-reality features.");
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
	}
	else if (needsReinstall) {
		ImGui::TextUnformatted("Install key missing");
	}
	else {
		ImGui::TextUnformatted("Not paired");
	}

	if (needsReinstall) {
		ImGui::Spacing();
		openvr_pair::overlay::ui::DrawBanner(
		    "Reinstall required",
		    "The local install key is missing. The headset app only responds to that key, so WKOpenVR cannot safely "
		    "control the existing install. Reinstall the companion to create a new key.",
		    pal.bannerWarnBg, pal.bannerWarnTitle, pal.bannerWarnDetail);
	}

	ImGui::Spacing();
	if (!apkAvailable) {
		ImGui::TextDisabled("Companion APK is not bundled in this build yet.");
	}
	const char* installLabel =
	    needsReinstall ? "Repair companion install" : (canControl ? "Reinstall companion" : "Install companion");
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
		const bool selected = companionSettings_.selectedPackage == target.packageName &&
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
			const bool selected = companionSettings_.selectedPackage == target.packageName &&
			                      companionSettings_.selectedActivity == target.activityName;
			if (ImGui::RadioButton(target.packageName.c_str(), selected)) {
				companionSettings_.selectedPackage = target.packageName;
				companionSettings_.selectedActivity = target.activityName;
			}
			ImGui::PopID();
		}
		if (detectedPackages_.empty()) {
			ImGui::TextDisabled("No packages listed. Check ADB authorization or Wi-Fi ADB connection.");
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
	}
	else if (wkopenvr::questapp::CanContactCompanion(cfg_)) {
		ImGui::TextDisabled("Uses the paired headset companion over Wi-Fi when reachable.");
	}
	else {
		ImGui::TextDisabled("Install the headset companion before settings can be sent.");
	}
}

void QuestAppPlugin::RefreshPackages()
{
	detectedPackages_ = wkopenvr::questapp::QueryInstalledPackages(adb_);
	packagesLoaded_ = true;
	showAllPackages_ = true;
	SetStatus("Package list refreshed.", false);
}

#if WKOPENVR_BUILD_IS_DEV
void QuestAppPlugin::DrawDevTools(openvr_pair::overlay::ShellContext&)
{
	DrawAdbDevTools();
}

void QuestAppPlugin::DrawAdbDevTools()
{
	const auto& pal = openvr_pair::overlay::ui::GetPalette();
	openvr_pair::overlay::ui::DrawInfoBanner(
	    "Developer build only",
	    "ADB controls here are for local headset testing. They are not exposed in release builds.");

	ImGui::Spacing();
	openvr_pair::overlay::ui::DrawSectionHeading("ADB status");
	adb_.RefreshResolvedBinaryPath();
	const bool adbInstalled = adb_.BinaryAvailable();
	openvr_pair::overlay::ui::DrawStatusDot(adbInstalled ? pal.dotOk : pal.dotPending);
	ImGui::SameLine();
	ImGui::TextUnformatted(adbInstalled ? "platform-tools ready" : "Install ADB from Setup first");

	ImGui::Spacing();
	{
		openvr_pair::overlay::ui::DisabledSection disabled(!adbInstalled,
		                                                   "Install ADB platform-tools from the Setup tab first.");
		if (ImGui::Button("List ADB devices")) {
			const auto result = adb_.Run({"devices", "-l"}, std::chrono::seconds(8));
			if (result.timedOut || result.exitCode != 0) {
				lastDevicesOutput_ = TrimAscii(result.err.empty() ? result.out : result.err);
				SetStatus("adb devices failed.", true);
			}
			else {
				lastDevicesOutput_ = TrimAscii(result.out);
				SetStatus("ADB device list refreshed.", false);
			}
		}
		disabled.AttachReasonTooltip();
	}
	ImGui::SameLine();
	{
		openvr_pair::overlay::ui::DisabledSection disabled(!adbInstalled,
		                                                   "Install ADB platform-tools from the Setup tab first.");
		if (ImGui::Button("Open platform-tools terminal")) {
			const bool ok = adb_.OpenToolsTerminal();
			SetStatus(ok ? "Opened platform-tools terminal." : "Could not open platform-tools terminal.", !ok);
		}
		disabled.AttachReasonTooltip();
	}
	ImGui::SameLine();
	{
		openvr_pair::overlay::ui::DisabledSection disabled(!adbInstalled,
		                                                   "Install ADB platform-tools from the Setup tab first.");
		if (ImGui::Button("Open adb shell")) {
			const auto devices = adb_.Run({"devices", "-l"}, std::chrono::seconds(8));
			std::string serial;
			if (!devices.timedOut && devices.exitCode == 0) {
				lastDevicesOutput_ = TrimAscii(devices.out);
				serial = PreferredAdbSerialFromDevices(devices.out);
			}
			const bool ok = adb_.OpenInteractiveShell(serial);
			if (ok && !serial.empty()) {
				SetStatus("Opened adb shell for " + serial + ".", false);
			}
			else {
				SetStatus(ok ? "Opened adb shell terminal." : "Could not open adb shell terminal.", !ok);
			}
		}
		disabled.AttachReasonTooltip();
	}

	if (!lastDevicesOutput_.empty()) {
		ImGui::Spacing();
		ImGui::TextUnformatted("adb devices -l");
		ImGui::BeginChild("questapp_adb_devices_output", ImVec2(0.0f, 92.0f), true);
		ImGui::TextUnformatted(lastDevicesOutput_.c_str());
		ImGui::EndChild();
	}

	ImGui::Spacing();
	openvr_pair::overlay::ui::DrawSectionHeading("Wi-Fi ADB");
	openvr_pair::overlay::ui::DrawTextWrapped("Use this with USB connected and authorized. It asks the headset adb "
	                                          "daemon to listen on TCP port 5555 for this boot.");
	{
		openvr_pair::overlay::ui::DisabledSection disabled(!adbInstalled,
		                                                   "Install ADB platform-tools from the Setup tab first.");
		if (ImGui::Button("Enable Wi-Fi ADB on USB headset")) {
			const int port = 5555;
			const auto devices = adb_.Run({"devices", "-l"}, std::chrono::seconds(8));
			std::string usbSerial;
			if (!devices.timedOut && devices.exitCode == 0) {
				lastDevicesOutput_ = TrimAscii(devices.out);
				usbSerial = wkopenvr::questapp::FindAuthorizedUsbQuestSerial(devices.out);
			}
			if (usbSerial.empty()) {
				SetStatus("No authorized USB Quest was found.", true);
				disabled.AttachReasonTooltip();
				return;
			}
			const auto route = adb_.Run({"-s", usbSerial, "shell", "ip", "route"}, std::chrono::seconds(5));
			std::string endpoint;
			if (!route.timedOut && route.exitCode == 0) {
				endpoint = ExtractWifiAdbEndpoint(route.out, port);
			}
			const bool ok = adb_.EnableWirelessAdb(usbSerial, port);
			if (ok) {
				lastWifiEndpoint_ = endpoint;
				if (!endpoint.empty()) {
					SetStatus("Wi-Fi ADB enabled. Connect with: adb connect " + endpoint, false);
				}
				else {
					SetStatus("Wi-Fi ADB enabled. Use the headset Wi-Fi IP with port 5555.", false);
				}
			}
			else {
				SetStatus("Could not enable Wi-Fi ADB. Check USB authorization.", true);
			}
		}
		disabled.AttachReasonTooltip();
	}
	ImGui::SameLine();
	{
		openvr_pair::overlay::ui::DisabledSection disabled(!adbInstalled,
		                                                   "Install ADB platform-tools from the Setup tab first.");
		if (ImGui::Button("Return ADB to USB")) {
			const auto devices = adb_.Run({"devices", "-l"}, std::chrono::seconds(8));
			std::string serial;
			if (!devices.timedOut && devices.exitCode == 0) {
				lastDevicesOutput_ = TrimAscii(devices.out);
				serial = wkopenvr::questapp::FindAuthorizedWifiQuestSerial(devices.out);
				if (serial.empty()) {
					serial = wkopenvr::questapp::FindAuthorizedUsbQuestSerial(devices.out);
				}
			}
			if (serial.empty()) {
				SetStatus("No authorized Quest was found for adb usb.", true);
			}
			else {
				const bool ok = adb_.DisableWirelessAdb(serial);
				SetStatus(ok ? "ADB switched back to USB mode." : "Could not switch ADB back to USB mode.", !ok);
			}
		}
		disabled.AttachReasonTooltip();
	}

	if (!lastWifiEndpoint_.empty()) {
		ImGui::Spacing();
		const std::string connectCommand = "adb connect " + lastWifiEndpoint_;
		ImGui::TextUnformatted(connectCommand.c_str());
		ImGui::SameLine();
		openvr_pair::overlay::ui::CopyToClipboardButton("copy_wifi_adb_connect", connectCommand.c_str());
	}
}
#endif

void QuestAppPlugin::DrawLogsSection(openvr_pair::overlay::ShellContext&)
{
	openvr_pair::overlay::ui::DrawTextWrapped(
	    "Quest App logs use the shared WKOpenVR log folder. ADB command output redacts device addresses, pairing "
	    "codes, and the companion install key.");
}

namespace openvr_pair::overlay {

std::unique_ptr<FeaturePlugin> CreateQuestAppPlugin()
{
	return std::make_unique<QuestAppPlugin>();
}

} // namespace openvr_pair::overlay
