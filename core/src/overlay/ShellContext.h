#pragma once

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
	std::string desktopDefaultModuleFlagFileName;
	bool vrConnected = false;
	bool dashboardVisible = false;

	std::string DesktopDefaultModuleFlagFileName() const;
	bool SetDesktopDefaultModuleFlagFileName(const char *flagFileName);
	std::wstring FlagPath(const char *flagFileName) const;
	bool IsFlagPresent(const char *flagFileName) const;
	bool SetFlagPresent(const char *flagFileName, bool present);
	bool IsTogglePending(const char *flagFileName) const;
	void TickToggles();
	void SetStatus(std::string message);
};

ShellContext CreateShellContext();
std::string Narrow(const std::wstring &value);

} // namespace openvr_pair::overlay
