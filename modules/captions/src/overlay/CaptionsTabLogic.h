#pragma once

#include "HostStatusPoller.h"

#include <cstring>
#include <string>

// Pure presentation logic for the captions tab. Kept free of ImGui so the
// button-gating and "no audio" decisions can be unit-tested without a live
// render frame.
namespace captions::ui {

// The speech pack counts as installed once the host reports it present -- even
// if the VAD runtime is still missing -- so the user can Uninstall a partial
// install and start over.
inline bool SpeechInstalled(const captions::HostStatusSnapshot& snap)
{
	return snap.valid && snap.speech_pack_installed;
}

inline bool SpeechInstallEnabled(const captions::HostStatusSnapshot& snap, bool busy)
{
	return !busy && !SpeechInstalled(snap);
}

inline bool SpeechUninstallEnabled(const captions::HostStatusSnapshot& snap, bool busy)
{
	return !busy && SpeechInstalled(snap);
}

// Whether the translation pack for the currently selected language pair is
// installed. Mirrors the condition under which TranslationPackStatus() reports
// "Installed": a pack is selected, the host reports it installed, and the host's
// active pair is not mid-switch to a different one.
inline bool TranslationInstalled(const captions::HostStatusSnapshot& snap, const std::string& packId)
{
	if (packId.empty() || !snap.valid) return false;

	constexpr const char* kPrefix = "translation-";
	std::string expected = packId;
	if (expected.rfind(kPrefix, 0) == 0) expected = expected.substr(std::strlen(kPrefix));
	if (!expected.empty() && !snap.active_translation_pair.empty() && snap.active_translation_pair != expected) {
		return false; // host is switching pairs -> this pack is not the active one
	}
	return snap.translation_pack_installed;
}

inline bool TranslationInstallEnabled(const captions::HostStatusSnapshot& snap, bool busy, const std::string& packId)
{
	return !busy && !packId.empty() && !TranslationInstalled(snap, packId);
}

inline bool TranslationUninstallEnabled(const captions::HostStatusSnapshot& snap, bool busy, const std::string& packId)
{
	return !busy && !packId.empty() && TranslationInstalled(snap, packId);
}

// The "no audio reaching the host" warning. A counter that has not moved for a
// few seconds means the selected device is delivering nothing at all. A short
// grace period avoids flashing the warning while a slow device spins up.
inline bool ShouldWarnNoAudio(bool hostValid, double secondsSinceFramesAdvanced, double thresholdSec = 3.0)
{
	return hostValid && secondsSinceFramesAdvanced >= thresholdSec;
}

// Some virtual devices keep delivering silent frames forever, so the frame
// counter alone is not enough. Warn only after frames are actively arriving but
// the input meter has stayed below the audible threshold for a longer window.
inline bool ShouldWarnSilentInput(bool hostValid, bool framesArriving, double secondsSinceAudibleInput,
                                  double thresholdSec = 10.0)
{
	return hostValid && framesArriving && secondsSinceAudibleInput >= thresholdSec;
}

} // namespace captions::ui
