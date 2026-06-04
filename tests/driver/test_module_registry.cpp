#include "ModuleRegistry.h"
#include "ModuleSafety.h"
#include "ProtocolNames.h"

#include <gtest/gtest.h>

#include <string_view>

namespace module_registry = openvr_pair::common::modules;
namespace module_safety = openvr_pair::common::module_safety;

TEST(ModuleRegistry, ContainsAllUserVisibleModules)
{
	size_t count = 0;
	const module_registry::ModuleInfo* modules = module_registry::All(&count);
	ASSERT_NE(modules, nullptr);
	EXPECT_EQ(count, 8u);

	EXPECT_NE(module_registry::FindById(module_registry::ModuleId::Calibration), nullptr);
	EXPECT_NE(module_registry::FindById(module_registry::ModuleId::Smoothing), nullptr);
	EXPECT_NE(module_registry::FindById(module_registry::ModuleId::InputHealth), nullptr);
	EXPECT_NE(module_registry::FindById(module_registry::ModuleId::FaceTracking), nullptr);
	EXPECT_NE(module_registry::FindById(module_registry::ModuleId::OscRouter), nullptr);
	EXPECT_NE(module_registry::FindById(module_registry::ModuleId::Captions), nullptr);
	EXPECT_NE(module_registry::FindById(module_registry::ModuleId::Phantom), nullptr);
	EXPECT_NE(module_registry::FindById(module_registry::ModuleId::QuestApp), nullptr);
}

TEST(ModuleRegistry, ExposesStableFaceTrackingMetadata)
{
	const auto* module = module_registry::FindById(module_registry::ModuleId::FaceTracking);
	ASSERT_NE(module, nullptr);

	EXPECT_EQ(std::string_view(module->slug), "facetracking");
	EXPECT_EQ(std::string_view(module->flag_file), "enable_facetracking.flag");
	EXPECT_EQ(std::wstring_view(module->flag_file_wide), L"enable_facetracking.flag");
	EXPECT_EQ(std::string_view(module->display_name), "Face Tracking");
	EXPECT_EQ(std::wstring_view(module->shortcut_argument), L"--launch=facetracking");
	EXPECT_EQ(std::string_view(module->pipe_name), OPENVR_PAIRDRIVER_FACETRACKING_PIPE_NAME);
	EXPECT_TRUE(module->requires_osc_router);
	EXPECT_TRUE(module->participates_in_driver_safety);
}

TEST(ModuleRegistry, CaptionsKeepsLegacyTranslatorFlagAlias)
{
	const auto* module = module_registry::FindByAnyFlagFileName("enable_translator.flag");
	ASSERT_NE(module, nullptr);

	EXPECT_EQ(module->id, module_registry::ModuleId::Captions);
	EXPECT_EQ(std::string_view(module->flag_file), "enable_captions.flag");
	EXPECT_EQ(std::string_view(module->legacy_flag_file), "enable_translator.flag");
	EXPECT_EQ(std::wstring_view(module->legacy_flag_file_wide), L"enable_translator.flag");
	EXPECT_TRUE(module->requires_osc_router);
}

TEST(ModuleRegistry, QuestAppIsUserVisibleButNotDriverSafetyManaged)
{
	const auto* module = module_registry::FindById(module_registry::ModuleId::QuestApp);
	ASSERT_NE(module, nullptr);

	EXPECT_EQ(std::string_view(module->flag_file), "enable_questapp.flag");
	EXPECT_EQ(std::string_view(module->pipe_name), "");
	EXPECT_FALSE(module->requires_osc_router);
	EXPECT_FALSE(module->participates_in_driver_safety);
	EXPECT_EQ(module_safety::FindByFlagFileName(module->flag_file), nullptr);
}

TEST(ModuleRegistry, ModuleSafetyDelegatesToDriverSafetyEntries)
{
	size_t count = 0;
	const module_safety::ModuleSpec* safetyModules = module_safety::Specs(&count);
	ASSERT_NE(safetyModules, nullptr);
	EXPECT_EQ(count, 7u);

	const auto* phantom = module_registry::FindById(module_registry::ModuleId::Phantom);
	ASSERT_NE(phantom, nullptr);
	EXPECT_EQ(module_safety::FindByFlagFileName("enable_phantom.flag"), phantom);
}
