#define _CRT_SECURE_NO_DEPRECATE
#define WIN32_LEAN_AND_MEAN
#include <windows.h>

#include "ActionBindings.h"
#include "Logging.h"

#include <openvr.h>

#include <string>

// ---------------------------------------------------------------------------
// Manifest path resolution
// ---------------------------------------------------------------------------

std::string ActionBindings::ResolveManifestPath()
{
	char buf[MAX_PATH] = {};
	GetModuleFileNameA(nullptr, buf, MAX_PATH);
	std::string path(buf);
	auto sep = path.find_last_of("/\\");
	if (sep != std::string::npos) path = path.substr(0, sep);
	path += "\\actions.json";
	return path;
}

bool ActionBindings::Register(const std::string& manifest_path)
{
	last_error_.clear();
	app_key_.clear();

	if (auto* apps = vr::VRApplications()) {
		char key[vr::k_unMaxApplicationKeyLength] = {};
		vr::EVRApplicationError appErr = apps->GetApplicationKeyByProcessId(GetCurrentProcessId(), key, sizeof(key));
		if (appErr == vr::VRApplicationError_None) {
			app_key_ = key;
			TH_LOG("[actions] process app key: %s", key);
			if (app_key_.rfind("system.generated.", 0) == 0 || app_key_.rfind("steam.", 0) == 0) {
				TH_LOG("[actions] app key is not stable for captions bindings; PTT may use a generated binding set");
			}
		}
		else {
			TH_LOG("[actions] GetApplicationKeyByProcessId failed: %d", (int)appErr);
		}
	}
	else {
		TH_LOG("[actions] VRApplications() not available");
	}

	DWORD attr = GetFileAttributesA(manifest_path.c_str());
	if (attr == INVALID_FILE_ATTRIBUTES || (attr & FILE_ATTRIBUTE_DIRECTORY)) {
		last_error_ = "actions.json missing at " + manifest_path;
		TH_LOG("[actions] manifest missing: path='%s' err=%lu", manifest_path.c_str(), (unsigned long)GetLastError());
		return false;
	}

	auto* input = vr::VRInput();
	if (!input) {
		last_error_ = "VRInput() not available";
		TH_LOG("[actions] %s", last_error_.c_str());
		return false;
	}

	vr::EVRInputError err = input->SetActionManifestPath(manifest_path.c_str());
	if (err != vr::VRInputError_None) {
		last_error_ = "SetActionManifestPath failed: " + std::to_string((int)err);
		TH_LOG("[actions] SetActionManifestPath failed: %d (path='%s')", (int)err, manifest_path.c_str());
		return false;
	}

	err = input->GetActionSetHandle("/actions/translator", &action_set_);
	if (err != vr::VRInputError_None) {
		last_error_ = "GetActionSetHandle failed: " + std::to_string((int)err);
		TH_LOG("[actions] GetActionSetHandle failed: %d", (int)err);
		return false;
	}

	err = input->GetActionHandle("/actions/translator/in/push_to_talk", &ptt_action_);
	if (err != vr::VRInputError_None) {
		last_error_ = "GetActionHandle failed: " + std::to_string((int)err);
		TH_LOG("[actions] GetActionHandle (push_to_talk) failed: %d", (int)err);
		return false;
	}

	registered_ = true;
	last_error_.clear();
	TH_LOG("[actions] manifest registered, action set and handle obtained");
	return true;
}

bool ActionBindings::Poll()
{
	if (!registered_) return false;
	auto* input = vr::VRInput();
	if (!input) return false;

	vr::VRActiveActionSet_t active{};
	active.ulActionSet = action_set_;
	input->UpdateActionState(&active, sizeof(active), 1);

	vr::InputDigitalActionData_t data{};
	vr::EVRInputError err =
	    input->GetDigitalActionData(ptt_action_, &data, sizeof(data), vr::k_ulInvalidInputValueHandle);

	if (err != vr::VRInputError_None) return false;
	last_state_ = data.bActive && data.bState;
	return last_state_;
}

bool ActionBindings::IsActionActive() const
{
	return last_state_;
}
