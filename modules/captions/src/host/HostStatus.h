#pragma once

#include <string>

// Writes a host_status.json file that the overlay polls to show live state.
// File location: %LocalAppDataLow%\WKOpenVR\captions\host_status.json
// Written atomically via a .tmp + rename. Refreshed at most once per second.
class HostStatus
{
public:
	enum class State : int
	{
		Idle = 0,
		Listening = 1,
		Transcribing = 2,
		Translating = 3,
		Sending = 4,
		Error = 5,
	};

	explicit HostStatus(const std::wstring& status_path = L"");

	void SetState(State s) noexcept;
	void SetMicName(const std::string& name);
	void SetLastTranscript(const std::string& t);
	void SetLastTranslation(const std::string& t);
	void SetLastError(const std::string& e);
	void SetPhase(const std::string& phase);
	void SetPttStatus(bool available, bool registered, const std::string& app_key, const std::string& error);
	void SetSpeechPackInstalled(bool installed) noexcept;
	void SetVadRuntimeAvailable(bool available) noexcept;
	void SetTranslationRuntimeAvailable(bool available) noexcept;
	void SetTranslationPackInstalled(bool installed) noexcept;
	void SetActiveTranslationPair(const std::string& pair);
	void IncrementCaptionsCompleted() noexcept;
	void IncrementPacketsSent() noexcept;

	// Live capture instrumentation surfaced to the overlay so the user can see
	// whether the selected microphone is actually delivering audio.
	void SetAudioLevel(float level) noexcept;
	void SetFramesCaptured(long long frames) noexcept;

	// Write the JSON file to disk if at least 1 s has elapsed since the
	// last write. Call periodically from the main loop.
	void MaybeFlush();

	// Force a flush regardless of the timer (call on shutdown).
	void Flush();

private:
	std::wstring status_path_;
	State state_ = State::Idle;
	std::string mic_name_;
	std::string last_transcript_;
	std::string last_translation_;
	std::string last_error_;
	std::string phase_ = "starting";
	bool ptt_available_ = false;
	bool ptt_registered_ = false;
	std::string ptt_app_key_;
	std::string ptt_error_;
	bool speech_pack_installed_ = false;
	bool vad_runtime_available_ = false;
	bool translation_runtime_available_ = false;
	bool translation_pack_installed_ = false;
	std::string active_translation_pair_;
	long long captions_completed_ = 0;
	long long packets_sent_ = 0;
	float audio_level_ = 0.0f;
	long long frames_captured_ = 0;

	void WritePath(const std::wstring& status_path);
	void DoFlush();

	// Timing.
	long long last_flush_tick_ = 0; // GetTickCount64
};
