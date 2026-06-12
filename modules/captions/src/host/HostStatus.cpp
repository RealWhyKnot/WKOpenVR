#define _CRT_SECURE_NO_DEPRECATE
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <shlobj.h>
#include <objbase.h>

#include "HostStatus.h"
#include "Logging.h"
#include "Win32Paths.h"

#include <cstdio>
#include <cstring>
#include <sstream>
#include <string>

static std::string EscapeJson(const std::string& s)
{
	std::string out;
	out.reserve(s.size() + 4);
	for (char c : s) {
		switch (c) {
			case '"':
				out += "\\\"";
				break;
			case '\\':
				out += "\\\\";
				break;
			case '\n':
				out += "\\n";
				break;
			case '\r':
				out += "\\r";
				break;
			case '\t':
				out += "\\t";
				break;
			default:
				if (static_cast<unsigned char>(c) < 0x20) {
					char esc[8];
					snprintf(esc, sizeof(esc), "\\u%04x", (unsigned char)c);
					out += esc;
				}
				else {
					out += c;
				}
		}
	}
	return out;
}

HostStatus::HostStatus(const std::wstring& status_path)
{
	WritePath(status_path);
}

void HostStatus::WritePath(const std::wstring& status_path)
{
	if (!status_path.empty()) {
		status_path_ = status_path;
		TH_LOG("[status] path=%ls", status_path_.c_str());
		return;
	}

	std::wstring root = openvr_pair::common::WkOpenVrSubdirectoryPath(L"captions", true);
	if (root.empty()) {
		TH_LOG("[status] failed to resolve captions status directory");
		return;
	}
	status_path_ = root + L"\\host_status.json";
	TH_LOG("[status] path=%ls", status_path_.c_str());
}

void HostStatus::SetState(State s) noexcept
{
	state_ = s;
}
void HostStatus::SetMicName(const std::string& name)
{
	mic_name_ = name;
}
void HostStatus::SetLastTranscript(const std::string& t)
{
	last_transcript_ = t;
}
void HostStatus::SetLastTranslation(const std::string& t)
{
	last_translation_ = t;
}
void HostStatus::SetLastError(const std::string& e)
{
	last_error_ = e;
}
void HostStatus::SetPhase(const std::string& phase)
{
	phase_ = phase;
}
void HostStatus::SetInputDeviceDiagnostics(bool explicit_selection, bool audio_input_file_present,
                                           const std::string& effective_name)
{
	input_device_selection_mode_ = explicit_selection ? "explicit" : "system-default";
	audio_input_file_present_ = audio_input_file_present;
	effective_input_device_name_ = effective_name;
}
void HostStatus::SetPttStatus(bool available, bool registered, const std::string& app_key, const std::string& error)
{
	ptt_available_ = available;
	ptt_registered_ = registered;
	ptt_app_key_ = app_key;
	ptt_error_ = error;
}
void HostStatus::SetSpeechPackInstalled(bool installed) noexcept
{
	speech_pack_installed_ = installed;
}
void HostStatus::SetVadRuntimeAvailable(bool available) noexcept
{
	vad_runtime_available_ = available;
}
void HostStatus::SetVadDiagnostics(bool loaded, float last_probability, long long inference_failures,
                                   const std::string& last_error)
{
	vad_model_loaded_ = loaded;
	vad_last_probability_ = last_probability;
	vad_inference_failures_ = inference_failures;
	vad_last_error_ = last_error;
}
void HostStatus::SetTranslationRuntimeAvailable(bool available) noexcept
{
	translation_runtime_available_ = available;
}
void HostStatus::SetTranslationPackInstalled(bool installed) noexcept
{
	translation_pack_installed_ = installed;
}
void HostStatus::SetActiveTranslationPair(const std::string& pair)
{
	active_translation_pair_ = pair;
}
void HostStatus::IncrementCaptionsCompleted() noexcept
{
	++captions_completed_;
}
void HostStatus::IncrementPacketsSent() noexcept
{
	++packets_sent_;
}
void HostStatus::SetAudioLevel(float level) noexcept
{
	if (level < 0.0f) level = 0.0f;
	if (level > 1.0f) level = 1.0f;
	audio_level_ = level;
}
void HostStatus::SetFramesCaptured(long long frames) noexcept
{
	frames_captured_ = frames;
}
void HostStatus::SetAudioQueueDiagnostics(long long queued_frames, long long queued_audio_ms) noexcept
{
	if (queued_frames < 0) queued_frames = 0;
	if (queued_audio_ms < 0) queued_audio_ms = 0;
	audio_queue_frames_ = queued_frames;
	audio_queue_ms_ = queued_audio_ms;
}
void HostStatus::SetSpeechGateDiagnostics(float last_peak, float last_rms, float ambient_peak, float ambient_rms,
                                          float speech_peak_threshold, float speech_rms_threshold) noexcept
{
	gate_last_peak_ = last_peak;
	gate_last_rms_ = last_rms;
	gate_ambient_peak_ = ambient_peak;
	gate_ambient_rms_ = ambient_rms;
	gate_speech_peak_threshold_ = speech_peak_threshold;
	gate_speech_rms_threshold_ = speech_rms_threshold;
}
void HostStatus::SetSpeechModel(uint8_t model, const std::string& name, const std::string& active_path, bool loaded,
                                bool fallback)
{
	speech_model_ = static_cast<int>(model);
	speech_model_name_ = name;
	active_speech_model_path_ = active_path;
	speech_model_loaded_ = loaded;
	speech_model_fallback_ = fallback;
}
void HostStatus::SetComputeBackend(const std::string& name, bool gpu_active)
{
	compute_backend_ = name.empty() ? std::string("CPU") : name;
	compute_backend_gpu_ = gpu_active;
}
void HostStatus::SetLastSegmentDiagnostics(const std::string& reason, long long audio_ms, long long evidence_ms,
                                           long long decode_ms, float max_vad_probability, float max_peak,
                                           float speech_peak_threshold, float max_rms, float speech_rms_threshold)
{
	last_segment_reason_ = reason;
	last_segment_audio_ms_ = audio_ms;
	last_segment_evidence_ms_ = evidence_ms;
	last_transcribe_ms_ = decode_ms;
	last_segment_vad_probability_ = max_vad_probability;
	last_segment_peak_ = max_peak;
	last_segment_threshold_ = speech_peak_threshold;
	last_segment_rms_ = max_rms;
	last_segment_rms_threshold_ = speech_rms_threshold;
}
void HostStatus::SetPromptContextLength(size_t chars) noexcept
{
	prompt_context_chars_ = static_cast<long long>(chars);
}
void HostStatus::SetTranscriptSuppressionDiagnostics(const std::string& last_reason, long long total,
                                                     long long non_speech, long long no_speech_probability,
                                                     long long common_hallucination, long long common_filler,
                                                     long long short_weak_audio, long long repetitive,
                                                     long long low_confidence, long long slow_short_decode)
{
	last_suppression_reason_ = last_reason;
	suppressed_transcripts_ = total;
	suppressed_non_speech_ = non_speech;
	suppressed_no_speech_probability_ = no_speech_probability;
	suppressed_common_hallucination_ = common_hallucination;
	suppressed_common_filler_ = common_filler;
	suppressed_short_weak_audio_ = short_weak_audio;
	suppressed_repetitive_ = repetitive;
	suppressed_low_confidence_ = low_confidence;
	suppressed_slow_short_decode_ = slow_short_decode;
}
void HostStatus::SetSegmentRiskDiagnostics(int risk_score, const std::string& risk_reason, float speech_frame_ratio,
                                           float possible_frame_ratio, float acoustic_artifact_risk,
                                           float speech_band_ratio, float zero_crossing_rate, float clipping_ratio,
                                           int prompt_quarantine_segments)
{
	if (risk_score < 0) risk_score = 0;
	if (speech_frame_ratio < 0.0f) speech_frame_ratio = 0.0f;
	if (speech_frame_ratio > 1.0f) speech_frame_ratio = 1.0f;
	if (possible_frame_ratio < 0.0f) possible_frame_ratio = 0.0f;
	if (possible_frame_ratio > 1.0f) possible_frame_ratio = 1.0f;
	if (acoustic_artifact_risk < 0.0f) acoustic_artifact_risk = 0.0f;
	if (acoustic_artifact_risk > 1.0f) acoustic_artifact_risk = 1.0f;
	if (speech_band_ratio < 0.0f) speech_band_ratio = 0.0f;
	if (speech_band_ratio > 1.0f) speech_band_ratio = 1.0f;
	if (zero_crossing_rate < 0.0f) zero_crossing_rate = 0.0f;
	if (zero_crossing_rate > 1.0f) zero_crossing_rate = 1.0f;
	if (clipping_ratio < 0.0f) clipping_ratio = 0.0f;
	if (clipping_ratio > 1.0f) clipping_ratio = 1.0f;
	if (prompt_quarantine_segments < 0) prompt_quarantine_segments = 0;
	last_segment_risk_score_ = risk_score;
	last_segment_risk_reason_ = risk_reason;
	last_segment_speech_frame_ratio_ = speech_frame_ratio;
	last_segment_possible_frame_ratio_ = possible_frame_ratio;
	last_segment_acoustic_artifact_risk_ = acoustic_artifact_risk;
	last_segment_speech_band_ratio_ = speech_band_ratio;
	last_segment_zero_crossing_rate_ = zero_crossing_rate;
	last_segment_clipping_ratio_ = clipping_ratio;
	prompt_context_quarantine_segments_ = prompt_quarantine_segments;
}

void HostStatus::MaybeFlush()
{
	const long long now = static_cast<long long>(GetTickCount64());
	if (now - last_flush_tick_ < 1000) return;
	last_flush_tick_ = now;
	DoFlush();
}

void HostStatus::Flush()
{
	last_flush_tick_ = static_cast<long long>(GetTickCount64());
	DoFlush();
}

void HostStatus::DoFlush()
{
	if (status_path_.empty()) return;

	std::ostringstream o;
	o << "{\n";
	o << "  \"schema_version\": 1,\n";
	o << "  \"host_pid\": " << (long long)GetCurrentProcessId() << ",\n";
	o << "  \"state\": " << (int)state_ << ",\n";
	o << "  \"phase\": \"" << EscapeJson(phase_) << "\",\n";
	o << "  \"mic_name\": \"" << EscapeJson(mic_name_) << "\",\n";
	o << "  \"input_device_selection_mode\": \"" << EscapeJson(input_device_selection_mode_) << "\",\n";
	o << "  \"audio_input_file_present\": " << (audio_input_file_present_ ? "true" : "false") << ",\n";
	o << "  \"effective_input_device_name\": \"" << EscapeJson(effective_input_device_name_) << "\",\n";
	o << "  \"last_transcript\": \"" << EscapeJson(last_transcript_) << "\",\n";
	o << "  \"last_translation\": \"" << EscapeJson(last_translation_) << "\",\n";
	o << "  \"last_error\": \"" << EscapeJson(last_error_) << "\",\n";
	o << "  \"ptt_available\": " << (ptt_available_ ? "true" : "false") << ",\n";
	o << "  \"ptt_registered\": " << (ptt_registered_ ? "true" : "false") << ",\n";
	o << "  \"ptt_app_key\": \"" << EscapeJson(ptt_app_key_) << "\",\n";
	o << "  \"ptt_error\": \"" << EscapeJson(ptt_error_) << "\",\n";
	o << "  \"speech_pack_installed\": " << (speech_pack_installed_ ? "true" : "false") << ",\n";
	o << "  \"vad_runtime_available\": " << (vad_runtime_available_ ? "true" : "false") << ",\n";
	o << "  \"translation_runtime_available\": " << (translation_runtime_available_ ? "true" : "false") << ",\n";
	o << "  \"translation_pack_installed\": " << (translation_pack_installed_ ? "true" : "false") << ",\n";
	o << "  \"active_translation_pair\": \"" << EscapeJson(active_translation_pair_) << "\",\n";
	char level_buf[32];
	snprintf(level_buf, sizeof(level_buf), "%.3f", audio_level_);
	char vad_buf[32];
	snprintf(vad_buf, sizeof(vad_buf), "%.3f", last_segment_vad_probability_);
	char peak_buf[32];
	snprintf(peak_buf, sizeof(peak_buf), "%.3f", last_segment_peak_);
	char threshold_buf[32];
	snprintf(threshold_buf, sizeof(threshold_buf), "%.3f", last_segment_threshold_);
	char rms_buf[32];
	snprintf(rms_buf, sizeof(rms_buf), "%.3f", last_segment_rms_);
	char rms_threshold_buf[32];
	snprintf(rms_threshold_buf, sizeof(rms_threshold_buf), "%.3f", last_segment_rms_threshold_);
	char vad_prob_buf[32];
	snprintf(vad_prob_buf, sizeof(vad_prob_buf), "%.3f", vad_last_probability_);
	char gate_last_peak_buf[32];
	snprintf(gate_last_peak_buf, sizeof(gate_last_peak_buf), "%.3f", gate_last_peak_);
	char gate_last_rms_buf[32];
	snprintf(gate_last_rms_buf, sizeof(gate_last_rms_buf), "%.3f", gate_last_rms_);
	char gate_ambient_peak_buf[32];
	snprintf(gate_ambient_peak_buf, sizeof(gate_ambient_peak_buf), "%.3f", gate_ambient_peak_);
	char gate_ambient_rms_buf[32];
	snprintf(gate_ambient_rms_buf, sizeof(gate_ambient_rms_buf), "%.3f", gate_ambient_rms_);
	char gate_speech_peak_threshold_buf[32];
	snprintf(gate_speech_peak_threshold_buf, sizeof(gate_speech_peak_threshold_buf), "%.3f",
	         gate_speech_peak_threshold_);
	char gate_speech_rms_threshold_buf[32];
	snprintf(gate_speech_rms_threshold_buf, sizeof(gate_speech_rms_threshold_buf), "%.3f", gate_speech_rms_threshold_);
	char speech_frame_ratio_buf[32];
	snprintf(speech_frame_ratio_buf, sizeof(speech_frame_ratio_buf), "%.3f", last_segment_speech_frame_ratio_);
	char possible_frame_ratio_buf[32];
	snprintf(possible_frame_ratio_buf, sizeof(possible_frame_ratio_buf), "%.3f", last_segment_possible_frame_ratio_);
	char acoustic_risk_buf[32];
	snprintf(acoustic_risk_buf, sizeof(acoustic_risk_buf), "%.3f", last_segment_acoustic_artifact_risk_);
	char speech_band_ratio_buf[32];
	snprintf(speech_band_ratio_buf, sizeof(speech_band_ratio_buf), "%.3f", last_segment_speech_band_ratio_);
	char zero_crossing_rate_buf[32];
	snprintf(zero_crossing_rate_buf, sizeof(zero_crossing_rate_buf), "%.3f", last_segment_zero_crossing_rate_);
	char clipping_ratio_buf[32];
	snprintf(clipping_ratio_buf, sizeof(clipping_ratio_buf), "%.3f", last_segment_clipping_ratio_);

	o << "  \"captions_completed\": " << captions_completed_ << ",\n";
	o << "  \"packets_sent\": " << packets_sent_ << ",\n";
	o << "  \"audio_level\": " << level_buf << ",\n";
	o << "  \"frames_captured\": " << frames_captured_ << ",\n";
	o << "  \"audio_queue_frames\": " << audio_queue_frames_ << ",\n";
	o << "  \"audio_queue_ms\": " << audio_queue_ms_ << ",\n";
	o << "  \"gate_last_peak\": " << gate_last_peak_buf << ",\n";
	o << "  \"gate_last_rms\": " << gate_last_rms_buf << ",\n";
	o << "  \"gate_ambient_peak\": " << gate_ambient_peak_buf << ",\n";
	o << "  \"gate_ambient_rms\": " << gate_ambient_rms_buf << ",\n";
	o << "  \"gate_speech_peak_threshold\": " << gate_speech_peak_threshold_buf << ",\n";
	o << "  \"gate_speech_rms_threshold\": " << gate_speech_rms_threshold_buf << ",\n";
	o << "  \"speech_model\": " << speech_model_ << ",\n";
	o << "  \"speech_model_name\": \"" << EscapeJson(speech_model_name_) << "\",\n";
	o << "  \"speech_model_loaded\": " << (speech_model_loaded_ ? "true" : "false") << ",\n";
	o << "  \"speech_model_fallback\": " << (speech_model_fallback_ ? "true" : "false") << ",\n";
	o << "  \"active_speech_model_path\": \"" << EscapeJson(active_speech_model_path_) << "\",\n";
	o << "  \"compute_backend\": \"" << EscapeJson(compute_backend_) << "\",\n";
	o << "  \"compute_backend_gpu\": " << (compute_backend_gpu_ ? "true" : "false") << ",\n";
	o << "  \"vad_model_loaded\": " << (vad_model_loaded_ ? "true" : "false") << ",\n";
	o << "  \"vad_last_probability\": " << vad_prob_buf << ",\n";
	o << "  \"vad_inference_failures\": " << vad_inference_failures_ << ",\n";
	o << "  \"vad_last_error\": \"" << EscapeJson(vad_last_error_) << "\",\n";
	o << "  \"last_segment_reason\": \"" << EscapeJson(last_segment_reason_) << "\",\n";
	o << "  \"last_segment_audio_ms\": " << last_segment_audio_ms_ << ",\n";
	o << "  \"last_segment_evidence_ms\": " << last_segment_evidence_ms_ << ",\n";
	o << "  \"last_transcribe_ms\": " << last_transcribe_ms_ << ",\n";
	o << "  \"last_segment_vad_probability\": " << vad_buf << ",\n";
	o << "  \"last_segment_peak\": " << peak_buf << ",\n";
	o << "  \"last_segment_threshold\": " << threshold_buf << ",\n";
	o << "  \"last_segment_rms\": " << rms_buf << ",\n";
	o << "  \"last_segment_rms_threshold\": " << rms_threshold_buf << ",\n";
	o << "  \"prompt_context_chars\": " << prompt_context_chars_ << ",\n";
	o << "  \"last_suppression_reason\": \"" << EscapeJson(last_suppression_reason_) << "\",\n";
	o << "  \"suppressed_transcripts\": " << suppressed_transcripts_ << ",\n";
	o << "  \"suppressed_non_speech\": " << suppressed_non_speech_ << ",\n";
	o << "  \"suppressed_no_speech_probability\": " << suppressed_no_speech_probability_ << ",\n";
	o << "  \"suppressed_common_hallucination\": " << suppressed_common_hallucination_ << ",\n";
	o << "  \"suppressed_common_filler\": " << suppressed_common_filler_ << ",\n";
	o << "  \"suppressed_short_weak_audio\": " << suppressed_short_weak_audio_ << ",\n";
	o << "  \"suppressed_repetitive\": " << suppressed_repetitive_ << ",\n";
	o << "  \"suppressed_low_confidence\": " << suppressed_low_confidence_ << ",\n";
	o << "  \"suppressed_slow_short_decode\": " << suppressed_slow_short_decode_ << ",\n";
	o << "  \"last_segment_risk_score\": " << last_segment_risk_score_ << ",\n";
	o << "  \"last_segment_risk_reason\": \"" << EscapeJson(last_segment_risk_reason_) << "\",\n";
	o << "  \"last_segment_speech_frame_ratio\": " << speech_frame_ratio_buf << ",\n";
	o << "  \"last_segment_possible_frame_ratio\": " << possible_frame_ratio_buf << ",\n";
	o << "  \"last_segment_acoustic_risk\": " << acoustic_risk_buf << ",\n";
	o << "  \"last_segment_speech_band_ratio\": " << speech_band_ratio_buf << ",\n";
	o << "  \"last_segment_zero_crossing_rate\": " << zero_crossing_rate_buf << ",\n";
	o << "  \"last_segment_clipping_ratio\": " << clipping_ratio_buf << ",\n";
	o << "  \"prompt_context_quarantine_segments\": " << prompt_context_quarantine_segments_ << ",\n";
	o << "  \"osc_messages_sent\": " << packets_sent_ << ",\n";
	o << "  \"last_exit_code\": 0,\n";
	o << "  \"last_restart_time\": \"\"\n";
	o << "}\n";
	std::string json = o.str();

	std::wstring tmp = status_path_ + L".tmp";
	HANDLE h = CreateFileW(tmp.c_str(), GENERIC_WRITE, 0, nullptr, CREATE_ALWAYS, FILE_ATTRIBUTE_NORMAL, nullptr);
	if (h == INVALID_HANDLE_VALUE) {
		TH_LOG("[status] CreateFileW failed err=%lu path=%ls", (unsigned long)GetLastError(), tmp.c_str());
		return;
	}

	DWORD written = 0;
	WriteFile(h, json.data(), static_cast<DWORD>(json.size()), &written, nullptr);
	CloseHandle(h);

	if (written == static_cast<DWORD>(json.size())) {
		if (!MoveFileExW(tmp.c_str(), status_path_.c_str(), MOVEFILE_REPLACE_EXISTING)) {
			TH_LOG("[status] MoveFileExW failed err=%lu path=%ls", (unsigned long)GetLastError(), status_path_.c_str());
		}
	}
	else {
		TH_LOG("[status] partial write path=%ls written=%lu expected=%zu", tmp.c_str(), (unsigned long)written,
		       json.size());
		DeleteFileW(tmp.c_str());
	}
}
