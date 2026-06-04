#include "ManifestRegistration.h"

#include <openvr.h>

#include <cstdio>
#include <cstring>
#include <filesystem>
#include <string>

#ifdef _WIN32
#include <windows.h>
#endif

namespace openvr_pair::overlay {

namespace {

// Umbrella app key. Anything starting with a non-"steam." prefix avoids
// colliding with Steam's appid-derived overlay keys.
constexpr const char* kAppKey = "wk.wkopenvr";

// Pre-rename app key (was wk.openvr-pair). After registering the new key,
// attempt to remove the old registration so SteamVR does not show a stale
// duplicate row in the overlay list.
constexpr const char* kRenamedLegacyAppKey = "wk.openvr-pair";

// Pre-monorepo SC standalone key. Its registration still points at the now-
// deleted SpaceCalibrator.exe so SteamVR keeps a phantom autolaunch entry.
// Removing it after the umbrella registers prevents a duplicate row in the
// SteamVR overlay list.
constexpr const char* kLegacyAppKey = "steam.overlay.3368750";

std::filesystem::path ResolveManifestPath()
{
#ifdef _WIN32
	char exePath[MAX_PATH] = {};
	const DWORD len = GetModuleFileNameA(nullptr, exePath, MAX_PATH);
	if (len == 0 || len == MAX_PATH) return {};
	std::filesystem::path p(exePath);
	p.replace_filename("manifest.vrmanifest");
	return p;
#else
	return {};
#endif
}

} // namespace

void RegisterApplicationManifest(bool allowRuntimeLaunch)
{
	using namespace vr;

	const std::filesystem::path manifestPath = ResolveManifestPath();
	if (manifestPath.empty() || !std::filesystem::exists(manifestPath)) {
		fprintf(stderr, "manifest.vrmanifest missing next to WKOpenVR.exe; skipping SteamVR registration\n");
		return;
	}

	EVRInitError initErr = VRInitError_None;
	const EVRApplicationType appType = allowRuntimeLaunch ? VRApplication_Utility : VRApplication_Background;
	VR_Init(&initErr, appType);
	if (initErr != VRInitError_None) {
		fprintf(stderr, "VR_Init (%s) failed: %s\n", allowRuntimeLaunch ? "utility" : "background",
		        VR_GetVRInitErrorAsEnglishDescription(initErr));
		return;
	}

	if (VRApplications() == nullptr) {
		VR_Shutdown();
		return;
	}

	const std::string manifestStr = manifestPath.string();

	if (!VRApplications()->IsApplicationInstalled(kAppKey)) {
		const EVRApplicationError appErr = VRApplications()->AddApplicationManifest(manifestStr.c_str());
		if (appErr != VRApplicationError_None) {
			fprintf(stderr, "AddApplicationManifest failed: %s\n",
			        VRApplications()->GetApplicationsErrorNameFromEnum(appErr));
		}
		else {
			VRApplications()->SetApplicationAutoLaunch(kAppKey, true);
			fprintf(stderr, "Registered %s with SteamVR (autolaunch on)\n", kAppKey);
		}
	}

	// Remove the pre-rename registration (wk.openvr-pair) if it still exists.
	// The new key (wk.wkopenvr) is registered above; leaving the old one would
	// show a stale duplicate row in the SteamVR overlay list.
	if (VRApplications()->IsApplicationInstalled(kRenamedLegacyAppKey)) {
		VRApplications()->SetApplicationAutoLaunch(kRenamedLegacyAppKey, false);

		char oldBuf[MAX_PATH + 64] = {};
		EVRApplicationError oldErr = VRApplicationError_None;
		VRApplications()->GetApplicationPropertyString(kRenamedLegacyAppKey, VRApplicationProperty_BinaryPath_String,
		                                               oldBuf, sizeof(oldBuf), &oldErr);
		if (oldErr == VRApplicationError_None) {
			char* slash = std::strrchr(oldBuf, '\\');
			if (slash) {
				*(slash + 1) = '\0';
				strcat_s(oldBuf, sizeof(oldBuf), "manifest.vrmanifest");
				EVRApplicationError rmErr = VRApplications()->RemoveApplicationManifest(oldBuf);
				fprintf(stderr, "Removed old wk.openvr-pair manifest: %s\n",
				        VRApplications()->GetApplicationsErrorNameFromEnum(rmErr));
			}
		}
	}

	// Belt-and-suspenders cleanup of the SC standalone registration. The
	// vrappconfig entry persisted after SC.exe was deleted and SteamVR keeps
	// trying to autolaunch the missing binary, which appears to swallow the
	// umbrella's autolaunch too. Disabling autolaunch on the orphan is the
	// minimum that unblocks SteamVR from autolaunching the old key; the
	// manifest-removal step below is the cosmetic follow-up.
	if (VRApplications()->IsApplicationInstalled(kLegacyAppKey)) {
		VRApplications()->SetApplicationAutoLaunch(kLegacyAppKey, false);

		char buf[MAX_PATH + 64] = {};
		EVRApplicationError appErr = VRApplicationError_None;
		VRApplications()->GetApplicationPropertyString(kLegacyAppKey, VRApplicationProperty_BinaryPath_String, buf,
		                                               sizeof(buf), &appErr);
		if (appErr == VRApplicationError_None) {
			char* lastSlash = std::strrchr(buf, '\\');
			if (lastSlash) {
				*(lastSlash + 1) = '\0';
				strcat_s(buf, sizeof(buf), "manifest.vrmanifest");
				VRApplications()->RemoveApplicationManifest(buf);
			}
		}
	}

	VR_Shutdown();
}

void UnregisterApplicationManifest()
{
	using namespace vr;

	const std::filesystem::path manifestPath = ResolveManifestPath();
	if (manifestPath.empty()) return;

	EVRInitError initErr = VRInitError_None;
	VR_Init(&initErr, VRApplication_Utility);
	if (initErr != VRInitError_None) {
		fprintf(stderr, "VR_Init (utility) failed during unregister: %s\n",
		        VR_GetVRInitErrorAsEnglishDescription(initErr));
		return;
	}

	if (VRApplications() == nullptr) {
		VR_Shutdown();
		return;
	}

	if (VRApplications()->IsApplicationInstalled(kAppKey)) {
		const std::string manifestStr = manifestPath.string();
		const EVRApplicationError appErr = VRApplications()->RemoveApplicationManifest(manifestStr.c_str());
		if (appErr != VRApplicationError_None) {
			fprintf(stderr, "RemoveApplicationManifest failed: %s\n",
			        VRApplications()->GetApplicationsErrorNameFromEnum(appErr));
		}
	}

	VR_Shutdown();
}

} // namespace openvr_pair::overlay
