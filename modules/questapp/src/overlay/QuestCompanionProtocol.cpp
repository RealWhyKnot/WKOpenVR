#include "QuestCompanionProtocol.h"

#include <picojson.h>

namespace wkopenvr::questapp {

bool ParseCompanionSettingsJson(const std::string& body, QuestCompanionSettings& settings)
{
	picojson::value parsed;
	const std::string err = picojson::parse(parsed, body);
	if (!err.empty() || !parsed.is<picojson::object>()) return false;

	QuestCompanionSettings parsedSettings;
	const auto& obj = parsed.get<picojson::object>();
	auto remoteVersionIt = obj.find("remoteProgramVersion");
	if (remoteVersionIt != obj.end() && remoteVersionIt->second.is<double>()) {
		parsedSettings.remoteProgramVersion = static_cast<int>(remoteVersionIt->second.get<double>());
	}
	auto remoteFeaturesIt = obj.find("remoteFeatures");
	if (remoteFeaturesIt != obj.end() && remoteFeaturesIt->second.is<picojson::array>()) {
		for (const auto& feature : remoteFeaturesIt->second.get<picojson::array>()) {
			if (feature.is<std::string>() && feature.get<std::string>() == "settingsReport") {
				parsedSettings.remoteReportsSettings = true;
			}
		}
	}
	auto autoIt = obj.find("autoLaunchEnabled");
	if (autoIt != obj.end() && autoIt->second.is<bool>()) {
		parsedSettings.autoLaunchEnabled = autoIt->second.get<bool>();
	}
	auto pkgIt = obj.find("selectedPackage");
	if (pkgIt != obj.end() && pkgIt->second.is<std::string>()) {
		parsedSettings.selectedPackage = pkgIt->second.get<std::string>();
	}
	auto activityIt = obj.find("selectedActivity");
	if (activityIt != obj.end() && activityIt->second.is<std::string>()) {
		parsedSettings.selectedActivity = activityIt->second.get<std::string>();
	}

	settings = parsedSettings;
	return true;
}

} // namespace wkopenvr::questapp
