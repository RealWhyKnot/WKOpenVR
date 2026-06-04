#include "QuestAppConfig.h"

#include <gtest/gtest.h>

#include <sstream>

TEST(QuestAppConfig, GeneratedPairingKeyIsValid)
{
	const std::string a = wkopenvr::questapp::GeneratePairingKey();
	const std::string b = wkopenvr::questapp::GeneratePairingKey();

	EXPECT_TRUE(wkopenvr::questapp::IsValidPairingKey(a));
	EXPECT_TRUE(wkopenvr::questapp::IsValidPairingKey(b));
	EXPECT_NE(a, b);
}

TEST(QuestAppConfig, RoundTripsInstallStateOnly)
{
	wkopenvr::questapp::QuestAppConfig src;
	src.pairingKey = "0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef";
	src.pairedDeviceSerial = "230YC01DBK003Q";
	src.companionHost = "192.168.50.93";
	src.companionInstalled = true;

	std::stringstream stream;
	wkopenvr::questapp::WriteQuestAppConfig(src, stream);

	wkopenvr::questapp::QuestAppConfig dst = wkopenvr::questapp::ParseQuestAppConfig(stream);

	EXPECT_EQ(dst.pairingKey, src.pairingKey);
	EXPECT_EQ(dst.pairedDeviceSerial, src.pairedDeviceSerial);
	EXPECT_EQ(dst.companionHost, src.companionHost);
	EXPECT_TRUE(dst.companionInstalled);
	EXPECT_TRUE(wkopenvr::questapp::CanContactCompanion(dst));
	EXPECT_FALSE(wkopenvr::questapp::NeedsCompanionReinstall(dst));
	EXPECT_EQ(stream.str().find("selected_package"), std::string::npos);
	EXPECT_EQ(stream.str().find("auto_launch_enabled"), std::string::npos);
}

TEST(QuestAppConfig, MissingKeyRequiresReinstallAfterInstall)
{
	wkopenvr::questapp::QuestAppConfig cfg;
	cfg.companionInstalled = true;

	EXPECT_FALSE(wkopenvr::questapp::CanContactCompanion(cfg));
	EXPECT_TRUE(wkopenvr::questapp::NeedsCompanionReinstall(cfg));
}

TEST(QuestAppConfig, InvalidKeyIsDroppedOnParse)
{
	std::stringstream stream;
	stream << "pairing_key=short\n";
	stream << "companion_installed=1\n";

	const wkopenvr::questapp::QuestAppConfig cfg = wkopenvr::questapp::ParseQuestAppConfig(stream);

	EXPECT_TRUE(cfg.pairingKey.empty());
	EXPECT_TRUE(cfg.companionInstalled);
	EXPECT_TRUE(wkopenvr::questapp::NeedsCompanionReinstall(cfg));
}
