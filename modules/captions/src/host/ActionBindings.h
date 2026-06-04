#pragma once

#include <openvr.h>

#include <atomic>
#include <string>

// Registers the captions action manifest with IVRInput and polls the
// push-to-talk action each frame.
//
// The caller is responsible for ensuring that vr::VRInput() is valid (i.e.
// OpenVR has been initialised) before calling Register(). Poll() must be
// called on the same thread that called Register().
class ActionBindings
{
public:
	// Returns the path to the bundled actions.json, resolved relative to the
	// executable directory.
	static std::string ResolveManifestPath();

	// Register the action manifest and obtain handle values.
	// Returns false if OpenVR is not initialised or manifest not found.
	bool Register(const std::string& manifest_path);

	// Call once per frame after vr::VRInput()->UpdateActionState().
	// Returns true while push-to-talk is held.
	bool IsActionActive() const;

	// Update action state and sample the digital action. Call at ~90 Hz.
	// Returns true while push-to-talk is held.
	bool Poll();

	bool IsRegistered() const noexcept { return registered_; }
	const std::string& LastError() const noexcept { return last_error_; }
	const std::string& ApplicationKey() const noexcept { return app_key_; }

private:
	vr::VRActionSetHandle_t action_set_ = vr::k_ulInvalidActionSetHandle;
	vr::VRActionHandle_t ptt_action_ = vr::k_ulInvalidActionHandle;
	bool registered_ = false;
	bool last_state_ = false;
	std::string last_error_;
	std::string app_key_;
};
