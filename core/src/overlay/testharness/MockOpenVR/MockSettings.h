#pragma once

#include "BuildChannel.h"

#if WKOPENVR_BUILD_IS_DEV

#include <openvr_driver.h>

#include <cstdint>
#include <mutex>
#include <string>
#include <unordered_map>

namespace openvr_pair::overlay::testharness {

class MockOpenVRRuntime;

// Implements vr::IVRSettings. Driver-side code does not call this in the
// inventory we surveyed, but the OpenVR macro VR_INIT_SERVER_DRIVER_CONTEXT
// still resolves it. Returns default values for everything.
class MockSettings : public vr::IVRSettings
{
public:
	explicit MockSettings(MockOpenVRRuntime& owner);

	const char* GetSettingsErrorNameFromEnum(vr::EVRSettingsError eError) override;

	void SetBool(const char* pchSection, const char* pchSettingsKey, bool bValue,
	             vr::EVRSettingsError* peError = nullptr) override;
	void SetInt32(const char* pchSection, const char* pchSettingsKey, int32_t nValue,
	              vr::EVRSettingsError* peError = nullptr) override;
	void SetFloat(const char* pchSection, const char* pchSettingsKey, float flValue,
	              vr::EVRSettingsError* peError = nullptr) override;
	void SetString(const char* pchSection, const char* pchSettingsKey, const char* pchValue,
	               vr::EVRSettingsError* peError = nullptr) override;

	bool GetBool(const char* pchSection, const char* pchSettingsKey, vr::EVRSettingsError* peError = nullptr) override;
	int32_t GetInt32(const char* pchSection, const char* pchSettingsKey,
	                 vr::EVRSettingsError* peError = nullptr) override;
	float GetFloat(const char* pchSection, const char* pchSettingsKey,
	               vr::EVRSettingsError* peError = nullptr) override;
	void GetString(const char* pchSection, const char* pchSettingsKey, char* pchValue, uint32_t unValueLen,
	               vr::EVRSettingsError* peError = nullptr) override;

	void RemoveSection(const char* pchSection, vr::EVRSettingsError* peError = nullptr) override;
	void RemoveKeyInSection(const char* pchSection, const char* pchSettingsKey,
	                        vr::EVRSettingsError* peError = nullptr) override;

private:
	struct Key
	{
		std::string section;
		std::string key;
	};
	struct KeyHash
	{
		size_t operator()(const Key& k) const noexcept;
	};
	struct KeyEq
	{
		bool operator()(const Key& a, const Key& b) const noexcept;
	};

	MockOpenVRRuntime& owner_;
	std::mutex mu_;
	std::unordered_map<Key, std::string, KeyHash, KeyEq> string_;
	std::unordered_map<Key, int32_t, KeyHash, KeyEq> int_;
	std::unordered_map<Key, float, KeyHash, KeyEq> float_;
	std::unordered_map<Key, bool, KeyHash, KeyEq> bool_;
};

// Implements vr::IVRDriverLog. Forward log lines to stdout prefixed with
// [driverlog]; the harness logger picks them up.
class MockDriverLog : public vr::IVRDriverLog
{
public:
	explicit MockDriverLog(MockOpenVRRuntime& owner);
	void Log(const char* pchLogMessage) override;

private:
	MockOpenVRRuntime& owner_;
};

// Implements vr::IVRResources. Backs LoadSharedResource by reading from the
// sandbox driver resources tree; GetResourceFullPath returns a sandbox-rooted
// path. The runtime exposes the sandbox path via owner_.driver_resources().
class MockResources : public vr::IVRResources
{
public:
	explicit MockResources(MockOpenVRRuntime& owner);
	uint32_t LoadSharedResource(const char* pchResourceName, char* pchBuffer, uint32_t unBufferLen) override;
	uint32_t GetResourceFullPath(const char* pchResourceName, const char* pchResourceTypeDirectory, char* pchPathBuffer,
	                             uint32_t unBufferLen) override;

private:
	MockOpenVRRuntime& owner_;
};

// Implements vr::IVRDriverManager. The driver only calls GetDriverHandle.
class MockDriverManager : public vr::IVRDriverManager
{
public:
	explicit MockDriverManager(MockOpenVRRuntime& owner);
	uint32_t GetDriverCount() const override;
	uint32_t GetDriverName(vr::DriverId_t nDriver, char* pchValue, uint32_t unBufferSize) override;
	vr::DriverHandle_t GetDriverHandle(const char* pchDriverName) override;
	bool IsEnabled(vr::DriverId_t nDriver) const override;

private:
	MockOpenVRRuntime& owner_;
};

} // namespace openvr_pair::overlay::testharness

#endif // WKOPENVR_BUILD_IS_DEV
