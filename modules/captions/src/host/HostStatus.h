#pragma once

#include <cstddef>
#include <cstdint>
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
	void SetInputDeviceDiagnostics(bool explicit_selection, bool audio_input_file_present,
	                               const std::string& effective_name);
	void SetPttStatus(bool available, bool registered, const std::string& app_key, const std::string& error);
	void SetSpeechPackInstalled(bool installed) noexcept;
	void SetVadRuntimeAvailable(bool available) noexcept;
	void SetVadDiagnostics(bool loaded, float last_probability, long long inference_failures,
	                       const std::string& last_error);
	void SetTranslationRuntimeAvailable(bool available) noexcept;
	void SetTranslationPackInstalled(bool installed) noexcept;
	void SetActiveTranslationPair(const std::string& pair);
	void IncrementCaptionsCompleted() noexcept;
	void IncrementPacketsSent() noexcept;

	// Live capture instrumentation surfaced to the overlay so the user can see
	// whether the selected microphone is actually delivering audio.
	void SetAudioLevel(float level) noexcept;
	void SetFramesCaptured(long long frames) noexcept;
	void SetAudioQueueDiagnostics(long long queued_frames, long long queued_audio_ms) noexcept;
	void SetSpeechGateDiagnostics(float last_peak, float last_rms, float ambient_peak, float ambient_rms,
	                              float speech_peak_threshold, float speech_rms_threshold) noexcept;
	void SetSpeechModel(uint8_t model, const std::string& name, const std::string& active_path, bool loaded,
	                    bool fallback);
	void SetLastSegmentDiagnostics(const std::string& reason, long long audio_ms, long long evidence_ms,
	                               long long decode_ms, float max_vad_probability, float max_peak,
	                               float speech_peak_threshold, float max_rms, float speech_rms_threshold);
	void SetPromptContextLength(size_t chars) noexcept;
	void SetTranscriptSuppressionDiagnostics(const std::string& last_reason, long long total, long long non_speech,
	                                         long long no_speech_probability, long long common_hallucination,
	                                         long long common_filler, long long short_weak_audio, long long repetitive,
	                                         long long low_confidence, long long slow_short_decode);
	void SetSegmentRiskDiagnostics(int risk_score, const std::string& risk_reason, float speech_frame_ratio,
	                               float possible_frame_ratio, int prompt_quarantine_segments);

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
	std::string input_device_selection_mode_ = "system-default";
	bool audio_input_file_present_ = false;
	std::string effective_input_device_name_;
	bool ptt_available_ = false;
	bool ptt_registered_ = false;
	std::string ptt_app_key_;
	std::string ptt_error_;
	bool speech_pack_installed_ = false;
	bool vad_runtime_available_ = false;
	bool vad_model_loaded_ = false;
	float vad_last_probability_ = -1.0f;
	long long vad_inference_failures_ = 0;
	std::string vad_last_error_;
	bool translation_runtime_available_ = false;
	bool translation_pack_installed_ = false;
	std::string active_translation_pair_;
	long long captions_completed_ = 0;
	long long packets_sent_ = 0;
	float audio_level_ = 0.0f;
	long long frames_captured_ = 0;
	long long audio_queue_frames_ = 0;
	long long audio_queue_ms_ = 0;
	float gate_last_peak_ = 0.0f;
	float gate_last_rms_ = 0.0f;
	float gate_ambient_peak_ = 0.0f;
	float gate_ambient_rms_ = 0.0f;
	float gate_speech_peak_threshold_ = 0.0f;
	float gate_speech_rms_threshold_ = 0.0f;
	int speech_model_ = 0;
	std::string speech_model_name_;
	std::string active_speech_model_path_;
	bool speech_model_loaded_ = false;
	bool speech_model_fallback_ = false;
	std::string last_segment_reason_;
	long long last_segment_audio_ms_ = 0;
	long long last_segment_evidence_ms_ = 0;
	long long last_transcribe_ms_ = 0;
	float last_segment_vad_probability_ = -1.0f;
	float last_segment_peak_ = 0.0f;
	float last_segment_threshold_ = 0.0f;
	float last_segment_rms_ = 0.0f;
	float last_segment_rms_threshold_ = 0.0f;
	long long prompt_context_chars_ = 0;
	std::string last_suppression_reason_;
	long long suppressed_transcripts_ = 0;
	long long suppressed_non_speech_ = 0;
	long long suppressed_no_speech_probability_ = 0;
	long long suppressed_common_hallucination_ = 0;
	long long suppressed_common_filler_ = 0;
	long long suppressed_short_weak_audio_ = 0;
	long long suppressed_repetitive_ = 0;
	long long suppressed_low_confidence_ = 0;
	long long suppressed_slow_short_decode_ = 0;
	int last_segment_risk_score_ = 0;
	std::string last_segment_risk_reason_;
	float last_segment_speech_frame_ratio_ = 0.0f;
	float last_segment_possible_frame_ratio_ = 0.0f;
	int prompt_context_quarantine_segments_ = 0;

	void WritePath(const std::wstring& status_path);
	void DoFlush();

	// Timing.
	long long last_flush_tick_ = 0; // GetTickCount64
};
