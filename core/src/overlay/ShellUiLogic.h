#pragma once

#include <algorithm>
#include <cctype>
#include <iterator>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace openvr_pair::overlay {

constexpr std::string_view kDefaultLogsPanelPluginFlag = "enable_calibration.flag";
constexpr double kShellStatusDefaultSeconds = 8.0;

struct FeaturePickerSelection
{
	std::string_view flag;
	bool applyDesktopDefault = false;
};

inline bool ShouldSelectDesktopDefaultTab(bool vrConnected, const char* pluginFlagFileName,
                                          const std::string& desktopDefaultFlagFileName,
                                          const std::string& appliedFlagFileName)
{
	if (vrConnected || !pluginFlagFileName || desktopDefaultFlagFileName.empty()) return false;
	return appliedFlagFileName != desktopDefaultFlagFileName && desktopDefaultFlagFileName == pluginFlagFileName;
}

inline bool IsDefaultLogsPanelPlugin(const char* pluginFlagFileName)
{
	return pluginFlagFileName && std::string_view(pluginFlagFileName) == kDefaultLogsPanelPluginFlag;
}

inline bool ContainsFeatureFlag(const std::vector<std::string_view>& installedFlags, std::string_view flag)
{
	for (std::string_view installed : installedFlags) {
		if (installed == flag) return true;
	}
	return false;
}

inline bool ShouldShowFeatureContentTab(bool installed, bool autoDisabled, bool dependencyBlocked)
{
	return installed && !autoDisabled && !dependencyBlocked;
}

inline std::string TrimShellToken(std::string_view value)
{
	size_t begin = 0;
	while (begin < value.size() &&
	       (value[begin] == ' ' || value[begin] == '\t' || value[begin] == '\r' || value[begin] == '\n')) {
		++begin;
	}
	size_t end = value.size();
	while (end > begin &&
	       (value[end - 1] == ' ' || value[end - 1] == '\t' || value[end - 1] == '\r' || value[end - 1] == '\n')) {
		--end;
	}
	return std::string(value.substr(begin, end - begin));
}

inline bool IsValidModuleFlagForShellOrder(std::string_view value)
{
	constexpr std::string_view prefix = "enable_";
	constexpr std::string_view suffix = ".flag";
	if (value.size() <= prefix.size() + suffix.size()) return false;
	if (value.substr(0, prefix.size()) != prefix) return false;
	if (value.substr(value.size() - suffix.size()) != suffix) return false;
	for (const char ch : value) {
		const unsigned char c = static_cast<unsigned char>(ch);
		if (std::isalnum(c) || ch == '_' || ch == '-' || ch == '.') continue;
		return false;
	}
	return value.find('\\') == std::string_view::npos && value.find('/') == std::string_view::npos;
}

inline bool ContainsString(const std::vector<std::string>& values, std::string_view value)
{
	for (const std::string& entry : values) {
		if (entry == value) return true;
	}
	return false;
}

inline bool ContainsStringView(const std::vector<std::string_view>& values, std::string_view value)
{
	for (std::string_view entry : values) {
		if (entry == value) return true;
	}
	return false;
}

inline std::vector<std::string> ParseModuleTabOrderSetting(std::string_view setting)
{
	std::vector<std::string> order;
	size_t begin = 0;
	while (begin <= setting.size()) {
		size_t end = setting.find(';', begin);
		if (end == std::string_view::npos) end = setting.size();
		std::string token = TrimShellToken(setting.substr(begin, end - begin));
		if (IsValidModuleFlagForShellOrder(token) && !ContainsString(order, token)) {
			order.push_back(std::move(token));
		}
		if (end == setting.size()) break;
		begin = end + 1;
	}
	return order;
}

inline std::string SerializeModuleTabOrderSetting(const std::vector<std::string>& order)
{
	std::string serialized;
	std::vector<std::string> emitted;
	for (const std::string& flag : order) {
		if (!IsValidModuleFlagForShellOrder(flag) || ContainsString(emitted, flag)) continue;
		if (!serialized.empty()) serialized += ';';
		serialized += flag;
		emitted.push_back(flag);
	}
	return serialized;
}

inline std::vector<std::string> ResolveModuleTabOrder(const std::vector<std::string>& preferredOrder,
                                                      const std::vector<std::string_view>& availableFlags)
{
	std::vector<std::string> order;
	for (const std::string& flag : preferredOrder) {
		if (!IsValidModuleFlagForShellOrder(flag)) continue;
		if (!ContainsStringView(availableFlags, flag)) continue;
		if (!ContainsString(order, flag)) order.push_back(flag);
	}
	for (std::string_view flag : availableFlags) {
		if (!IsValidModuleFlagForShellOrder(flag)) continue;
		if (!ContainsString(order, flag)) order.emplace_back(flag);
	}
	return order;
}

inline bool MoveModuleTabOrder(std::vector<std::string>& order, std::string_view flag, int offset)
{
	if (offset == 0) return false;
	const auto it = std::find(order.begin(), order.end(), flag);
	if (it == order.end()) return false;
	const int index = static_cast<int>(std::distance(order.begin(), it));
	const int target = index + offset;
	if (target < 0 || target >= static_cast<int>(order.size())) return false;
	std::swap(order[static_cast<size_t>(index)], order[static_cast<size_t>(target)]);
	return true;
}

inline FeaturePickerSelection ResolveFeaturePickerSelection(bool vrConnected, std::string_view currentFlag,
                                                            std::string_view desktopDefaultFlag,
                                                            std::string_view appliedDesktopDefaultFlag,
                                                            const std::vector<std::string_view>& installedFlags)
{
	if (installedFlags.empty()) return {};

	if (!vrConnected && !desktopDefaultFlag.empty() && appliedDesktopDefaultFlag != desktopDefaultFlag &&
	    ContainsFeatureFlag(installedFlags, desktopDefaultFlag)) {
		return {desktopDefaultFlag, true};
	}

	if (!currentFlag.empty() && ContainsFeatureFlag(installedFlags, currentFlag)) {
		return {currentFlag, false};
	}

	return {installedFlags.front(), false};
}

inline bool ShouldClearTransientStatus(double nowSeconds, double clearAtSeconds)
{
	return clearAtSeconds > 0.0 && nowSeconds >= clearAtSeconds;
}

} // namespace openvr_pair::overlay
