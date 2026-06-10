#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace openvr_pair::overlay {

struct ShellContext
{
	std::wstring installDir;
	std::wstring profileRoot;
	std::wstring logRoot;
	std::vector<std::wstring> driverResourceDirs;
	std::string status;
	double statusClearAtSeconds = 0.0;
	std::string desktopDefaultModuleFlagFileName;
	bool vrConnected = false;
	bool activeDashboardOverlay = false;
	bool anyDashboardVisible = false;
	uint32_t primaryDashboardDevice = 0xFFFFFFFFu;
	int primaryDashboardHand = 0;
	bool dashboardVisible = false;
	bool dashboardInputSafeOverlayVisible = false;
	bool dashboardInputSafeOverlayToggleRequested = false;
	std::string dashboardInputSafeOverlayStatus;
	std::vector<std::string> moduleTabOrder;

	std::string DesktopDefaultModuleFlagFileName() const;
	bool SetDesktopDefaultModuleFlagFileName(const char* flagFileName);
	std::vector<std::string> ModuleTabOrder() const;
	bool SetModuleTabOrder(const std::vector<std::string>& order);
	std::wstring FlagPath(const char* flagFileName) const;
	bool IsFlagPresent(const char* flagFileName) const;
	bool IsModuleAutoDisabled(const char* flagFileName) const;
	std::string ModuleAutoDisabledReason(const char* flagFileName) const;
	bool SetFlagPresent(const char* flagFileName, bool present);
	bool IsTogglePending(const char* flagFileName) const;
	void TickToggles();
	void TickStatus();
	void ClearStatus();
	void SetStatus(std::string message, double ttlSeconds = -1.0);
};

ShellContext CreateShellContext();
std::string Narrow(const std::wstring& value);

} // namespace openvr_pair::overlay
