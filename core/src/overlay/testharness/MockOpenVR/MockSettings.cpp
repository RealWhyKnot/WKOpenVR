#include "MockSettings.h"

#if WKOPENVR_BUILD_IS_DEV

#include "../HarnessScenario.h"
#include "../MockPoseSource.h"

#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <string>

namespace fs = std::filesystem;

namespace openvr_pair::overlay::testharness {

size_t MockSettings::KeyHash::operator()(const Key& k) const noexcept
{
	return std::hash<std::string>{}(k.section) * 0x9E3779B97F4A7C15ULL ^ std::hash<std::string>{}(k.key);
}
bool MockSettings::KeyEq::operator()(const Key& a, const Key& b) const noexcept
{
	return a.section == b.section && a.key == b.key;
}

MockSettings::MockSettings(MockOpenVRRuntime& owner) : owner_(owner) {}

const char* MockSettings::GetSettingsErrorNameFromEnum(vr::EVRSettingsError /*eError*/)
{
	return "MockSettings::default";
}

void MockSettings::SetBool(const char* pchSection, const char* pchSettingsKey, bool bValue,
                           vr::EVRSettingsError* peError)
{
	if (peError) *peError = vr::VRSettingsError_None;
	std::lock_guard<std::mutex> lock(mu_);
	bool_[Key{pchSection ? pchSection : "", pchSettingsKey ? pchSettingsKey : ""}] = bValue;
}
void MockSettings::SetInt32(const char* pchSection, const char* pchSettingsKey, int32_t nValue,
                            vr::EVRSettingsError* peError)
{
	if (peError) *peError = vr::VRSettingsError_None;
	std::lock_guard<std::mutex> lock(mu_);
	int_[Key{pchSection ? pchSection : "", pchSettingsKey ? pchSettingsKey : ""}] = nValue;
}
void MockSettings::SetFloat(const char* pchSection, const char* pchSettingsKey, float flValue,
                            vr::EVRSettingsError* peError)
{
	if (peError) *peError = vr::VRSettingsError_None;
	std::lock_guard<std::mutex> lock(mu_);
	float_[Key{pchSection ? pchSection : "", pchSettingsKey ? pchSettingsKey : ""}] = flValue;
}
void MockSettings::SetString(const char* pchSection, const char* pchSettingsKey, const char* pchValue,
                             vr::EVRSettingsError* peError)
{
	if (peError) *peError = vr::VRSettingsError_None;
	std::lock_guard<std::mutex> lock(mu_);
	string_[Key{pchSection ? pchSection : "", pchSettingsKey ? pchSettingsKey : ""}] = pchValue ? pchValue : "";
}

bool MockSettings::GetBool(const char* pchSection, const char* pchSettingsKey, vr::EVRSettingsError* peError)
{
	if (peError) *peError = vr::VRSettingsError_None;
	std::lock_guard<std::mutex> lock(mu_);
	auto it = bool_.find(Key{pchSection ? pchSection : "", pchSettingsKey ? pchSettingsKey : ""});
	if (it == bool_.end()) {
		if (peError) *peError = vr::VRSettingsError_UnsetSettingHasNoDefault;
		return false;
	}
	return it->second;
}
int32_t MockSettings::GetInt32(const char* pchSection, const char* pchSettingsKey, vr::EVRSettingsError* peError)
{
	if (peError) *peError = vr::VRSettingsError_None;
	std::lock_guard<std::mutex> lock(mu_);
	auto it = int_.find(Key{pchSection ? pchSection : "", pchSettingsKey ? pchSettingsKey : ""});
	if (it == int_.end()) {
		if (peError) *peError = vr::VRSettingsError_UnsetSettingHasNoDefault;
		return 0;
	}
	return it->second;
}
float MockSettings::GetFloat(const char* pchSection, const char* pchSettingsKey, vr::EVRSettingsError* peError)
{
	if (peError) *peError = vr::VRSettingsError_None;
	std::lock_guard<std::mutex> lock(mu_);
	auto it = float_.find(Key{pchSection ? pchSection : "", pchSettingsKey ? pchSettingsKey : ""});
	if (it == float_.end()) {
		if (peError) *peError = vr::VRSettingsError_UnsetSettingHasNoDefault;
		return 0.0f;
	}
	return it->second;
}
void MockSettings::GetString(const char* pchSection, const char* pchSettingsKey, char* pchValue, uint32_t unValueLen,
                             vr::EVRSettingsError* peError)
{
	if (peError) *peError = vr::VRSettingsError_None;
	if (!pchValue || unValueLen == 0) return;
	pchValue[0] = '\0';
	std::lock_guard<std::mutex> lock(mu_);
	auto it = string_.find(Key{pchSection ? pchSection : "", pchSettingsKey ? pchSettingsKey : ""});
	if (it == string_.end()) {
		if (peError) *peError = vr::VRSettingsError_UnsetSettingHasNoDefault;
		return;
	}
	std::snprintf(pchValue, static_cast<size_t>(unValueLen), "%s", it->second.c_str());
}
void MockSettings::RemoveSection(const char* pchSection, vr::EVRSettingsError* peError)
{
	if (peError) *peError = vr::VRSettingsError_None;
	if (!pchSection) return;
	std::lock_guard<std::mutex> lock(mu_);
	const std::string sect(pchSection);
	for (auto it = bool_.begin(); it != bool_.end();)
		it = it->first.section == sect ? bool_.erase(it) : ++it;
	for (auto it = int_.begin(); it != int_.end();)
		it = it->first.section == sect ? int_.erase(it) : ++it;
	for (auto it = float_.begin(); it != float_.end();)
		it = it->first.section == sect ? float_.erase(it) : ++it;
	for (auto it = string_.begin(); it != string_.end();)
		it = it->first.section == sect ? string_.erase(it) : ++it;
}
void MockSettings::RemoveKeyInSection(const char* pchSection, const char* pchSettingsKey, vr::EVRSettingsError* peError)
{
	if (peError) *peError = vr::VRSettingsError_None;
	if (!pchSection || !pchSettingsKey) return;
	std::lock_guard<std::mutex> lock(mu_);
	Key k{pchSection, pchSettingsKey};
	bool_.erase(k);
	int_.erase(k);
	float_.erase(k);
	string_.erase(k);
}

// -----------------------------------------------------------------------------
// MockDriverLog

MockDriverLog::MockDriverLog(MockOpenVRRuntime& owner) : owner_(owner) {}

void MockDriverLog::Log(const char* pchLogMessage)
{
	if (pchLogMessage) {
		std::fprintf(stdout, "[driverlog] %s", pchLogMessage);
		// Many driver lines lack trailing newline; normalise.
		const size_t n = std::strlen(pchLogMessage);
		if (n == 0 || pchLogMessage[n - 1] != '\n') std::fputc('\n', stdout);
		std::fflush(stdout);
	}

	MockCall c;
	c.kind = MockCallKind::LogMessage;
	c.text = pchLogMessage ? pchLogMessage : "";
	owner_.recorder().Push(std::move(c));
}

// -----------------------------------------------------------------------------
// MockResources

MockResources::MockResources(MockOpenVRRuntime& owner) : owner_(owner) {}

uint32_t MockResources::LoadSharedResource(const char* pchResourceName, char* pchBuffer, uint32_t unBufferLen)
{
	if (!pchResourceName) return 0;
	const fs::path root = owner_.driver_resources();
	const fs::path full = root / pchResourceName;
	std::error_code ec;
	if (!fs::exists(full, ec)) return 0;
	const auto sz = (uint32_t)fs::file_size(full, ec);
	if (!pchBuffer || unBufferLen < sz) return sz;
	std::ifstream in(full, std::ios::binary);
	if (!in) return 0;
	in.read(pchBuffer, sz);
	return sz;
}

uint32_t MockResources::GetResourceFullPath(const char* pchResourceName, const char* pchResourceTypeDirectory,
                                            char* pchPathBuffer, uint32_t unBufferLen)
{
	if (!pchResourceName) return 0;
	const fs::path root = owner_.driver_resources();
	fs::path p = root;
	if (pchResourceTypeDirectory && *pchResourceTypeDirectory) p /= pchResourceTypeDirectory;
	p /= pchResourceName;
	const std::string s = p.string();
	const uint32_t need = (uint32_t)s.size() + 1;
	if (!pchPathBuffer || unBufferLen < need) return need;
	std::memcpy(pchPathBuffer, s.c_str(), need);
	return need;
}

// -----------------------------------------------------------------------------
// MockDriverManager

MockDriverManager::MockDriverManager(MockOpenVRRuntime& owner) : owner_(owner) {}

uint32_t MockDriverManager::GetDriverCount() const
{
	return 1;
}
uint32_t MockDriverManager::GetDriverName(vr::DriverId_t /*nDriver*/, char* pchValue, uint32_t unBufferSize)
{
	static const char* kName = "01wkopenvr";
	const uint32_t need = (uint32_t)std::strlen(kName) + 1;
	if (!pchValue || unBufferSize < need) return need;
	std::memcpy(pchValue, kName, need);
	return need;
}
vr::DriverHandle_t MockDriverManager::GetDriverHandle(const char* /*pchDriverName*/)
{
	return (vr::DriverHandle_t)0x10000001ULL;
}
bool MockDriverManager::IsEnabled(vr::DriverId_t /*nDriver*/) const
{
	return true;
}

} // namespace openvr_pair::overlay::testharness

#endif // WKOPENVR_BUILD_IS_DEV
