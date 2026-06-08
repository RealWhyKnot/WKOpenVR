#pragma once

#include "CaptionsRealtimeFlags.h"
#include "CaptionsSpeechModels.h"

#include <cstdint>
#include <string>

// On-disk persistence for the captions overlay plugin's runtime-mutable
// settings. Stored at %LocalAppDataLow%\WKOpenVR\profiles\captions.txt as a
// minimal key=value file (same pattern as smoothing's Config.cpp).
//
// Persists: mode (PTT vs always-on), source / target language, OSC chatbox
// path, audible notify-sound flag, speech model, realtime behavior flags, and
// the always-on consent sentinel so the user doesn't have to re-confirm every
// restart.
struct CaptionsConfig
{
	int mode = 0; // 0 = PTT, 1 = always-on
	bool always_on_consented = false;
	std::string source_lang = "auto";
	std::string target_lang = "";
	bool chatbox_enabled = false;
	std::string chatbox_address = "/chatbox/input";
	bool notify_sound = false;
	uint8_t realtime_flags = captions::kCaptionsRealtimeDefaultFlags;
	uint8_t speech_model = captions::kCaptionsSpeechModelBalanced;

	// Selected capture endpoint id (IMMDevice::GetId). Empty = system default.
	std::string input_device = "";
};

// Load from disk. Returns a default-constructed struct if the file is
// missing or unparseable -- intentionally silent so a fresh install just
// boots into the defaults rather than logging a fake error.
CaptionsConfig LoadCaptionsConfig();

// Save to disk via write-to-tmp + MoveFileExW(REPLACE_EXISTING). A crash
// mid-write leaves the existing file untouched rather than truncated.
// Silent on failure (the existing file stays valid; next call will retry).
void SaveCaptionsConfig(const CaptionsConfig& cfg);
