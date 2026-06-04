#pragma once

#include <iosfwd>
#include <string>

namespace wkopenvr::questapp {

struct QuestAppConfig
{
	std::string pairingKey;
	std::string pairedDeviceSerial;
	std::string companionHost;
	bool companionInstalled = false;
};

struct QuestCompanionSettings
{
	bool autoLaunchEnabled = false;
	std::string selectedPackage;
	std::string selectedActivity;
};

QuestAppConfig LoadQuestAppConfig();
void SaveQuestAppConfig(const QuestAppConfig& cfg);

QuestAppConfig ParseQuestAppConfig(std::istream& in);
void WriteQuestAppConfig(const QuestAppConfig& cfg, std::ostream& out);

std::string GeneratePairingKey();
bool IsValidPairingKey(const std::string& key);
bool CanContactCompanion(const QuestAppConfig& cfg);
bool NeedsCompanionReinstall(const QuestAppConfig& cfg);

} // namespace wkopenvr::questapp
