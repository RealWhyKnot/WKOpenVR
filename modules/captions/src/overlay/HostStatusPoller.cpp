#define _CRT_SECURE_NO_DEPRECATE
#include "HostStatusPoller.h"

#include "JsonUtil.h"
#include "Win32Paths.h"
#include "Win32Text.h"

#include <chrono>
#include <fstream>
#include <sstream>
#include <string>

namespace captions {

namespace {
constexpr auto kStaleAfter = std::chrono::seconds(10);
constexpr auto kReadInterval = std::chrono::milliseconds(500);

std::wstring ResolveStatusPath()
{
	std::wstring dir = openvr_pair::common::WkOpenVrSubdirectoryPath(L"captions", false);
	return dir.empty() ? std::wstring() : dir + L"\\host_status.json";
}
} // namespace

HostStatusPoller::HostStatusPoller()
{
	ResolvePath();
}

void HostStatusPoller::ResolvePath()
{
	std::wstring wpath = ResolveStatusPath();
	path_utf8_ = openvr_pair::common::WideToUtf8(wpath);
}

void HostStatusPoller::Tick()
{
	auto now = std::chrono::steady_clock::now();
	if (now - last_read_attempt_ < kReadInterval) {
		if (snapshot_.valid) snapshot_.stale = (now - last_successful_read_) > kStaleAfter;
		return;
	}
	last_read_attempt_ = now;

	if (path_utf8_.empty()) return;

	std::wstring wpath = openvr_pair::common::Utf8ToWide(path_utf8_);
	int64_t mtime = openvr_pair::common::FileLastWriteTime(wpath);
	if (mtime == 0) {
		if (snapshot_.valid) snapshot_.stale = (now - last_successful_read_) > kStaleAfter;
		return;
	}

	if (mtime == last_observed_mtime_) {
		snapshot_.stale = (now - last_successful_read_) > kStaleAfter;
		return;
	}

	ReadFile();
	if (snapshot_.valid) {
		last_observed_mtime_ = mtime;
		last_successful_read_ = now;
		snapshot_.stale = false;
	}
}

void HostStatusPoller::ReadFile()
{
	std::ifstream is(openvr_pair::common::Utf8ToWide(path_utf8_));
	if (!is) return;

	std::stringstream ss;
	ss << is.rdbuf();
	std::string body = ss.str();

	picojson::value root;
	if (!openvr_pair::common::json::ParseObject(root, body)) return;

	HostStatusSnapshot s;
	s.valid = true;
	s.host_halted = snapshot_.host_halted;
	s.last_exit_code = snapshot_.last_exit_code;
	s.last_exit_description = snapshot_.last_exit_description;
	s.host_pid = openvr_pair::common::json::IntAt(root, "host_pid");
	s.state = openvr_pair::common::json::IntAt(root, "state");
	s.phase = openvr_pair::common::json::StringAt(root, "phase");
	s.mic_name = openvr_pair::common::json::StringAt(root, "mic_name");
	s.input_device_selection_mode = openvr_pair::common::json::StringAt(root, "input_device_selection_mode");
	s.audio_input_file_present = openvr_pair::common::json::BoolAt(root, "audio_input_file_present");
	s.effective_input_device_name = openvr_pair::common::json::StringAt(root, "effective_input_device_name");
	s.last_transcript = openvr_pair::common::json::StringAt(root, "last_transcript");
	s.last_translation = openvr_pair::common::json::StringAt(root, "last_translation");
	s.last_error = openvr_pair::common::json::StringAt(root, "last_error");
	s.ptt_available = openvr_pair::common::json::BoolAt(root, "ptt_available");
	s.ptt_registered = openvr_pair::common::json::BoolAt(root, "ptt_registered");
	s.ptt_app_key = openvr_pair::common::json::StringAt(root, "ptt_app_key");
	s.ptt_error = openvr_pair::common::json::StringAt(root, "ptt_error");
	s.speech_pack_installed = openvr_pair::common::json::BoolAt(root, "speech_pack_installed");
	s.vad_runtime_available = openvr_pair::common::json::BoolAt(root, "vad_runtime_available");
	s.translation_runtime_available = openvr_pair::common::json::BoolAt(root, "translation_runtime_available");
	s.translation_pack_installed = openvr_pair::common::json::BoolAt(root, "translation_pack_installed");
	s.active_translation_pair = openvr_pair::common::json::StringAt(root, "active_translation_pair");
	s.captions_completed = static_cast<long long>(openvr_pair::common::json::NumberAt(root, "captions_completed"));
	s.packets_sent = static_cast<long long>(openvr_pair::common::json::NumberAt(root, "packets_sent"));
	s.audio_level = static_cast<float>(openvr_pair::common::json::NumberAt(root, "audio_level"));
	s.frames_captured = static_cast<long long>(openvr_pair::common::json::NumberAt(root, "frames_captured"));
	s.audio_queue_frames = static_cast<long long>(openvr_pair::common::json::NumberAt(root, "audio_queue_frames"));
	s.audio_queue_ms = static_cast<long long>(openvr_pair::common::json::NumberAt(root, "audio_queue_ms"));
	s.gate_last_peak = static_cast<float>(openvr_pair::common::json::NumberAt(root, "gate_last_peak"));
	s.gate_last_rms = static_cast<float>(openvr_pair::common::json::NumberAt(root, "gate_last_rms"));
	s.gate_ambient_peak = static_cast<float>(openvr_pair::common::json::NumberAt(root, "gate_ambient_peak"));
	s.gate_ambient_rms = static_cast<float>(openvr_pair::common::json::NumberAt(root, "gate_ambient_rms"));
	s.gate_speech_peak_threshold =
	    static_cast<float>(openvr_pair::common::json::NumberAt(root, "gate_speech_peak_threshold"));
	s.gate_speech_rms_threshold =
	    static_cast<float>(openvr_pair::common::json::NumberAt(root, "gate_speech_rms_threshold"));
	s.speech_model = openvr_pair::common::json::IntAt(root, "speech_model");
	s.speech_model_name = openvr_pair::common::json::StringAt(root, "speech_model_name");
	s.speech_model_loaded = openvr_pair::common::json::BoolAt(root, "speech_model_loaded");
	s.speech_model_fallback = openvr_pair::common::json::BoolAt(root, "speech_model_fallback");
	s.active_speech_model_path = openvr_pair::common::json::StringAt(root, "active_speech_model_path");
	s.vad_model_loaded = openvr_pair::common::json::BoolAt(root, "vad_model_loaded");
	s.vad_last_probability = static_cast<float>(openvr_pair::common::json::NumberAt(root, "vad_last_probability"));
	s.vad_inference_failures =
	    static_cast<long long>(openvr_pair::common::json::NumberAt(root, "vad_inference_failures"));
	s.vad_last_error = openvr_pair::common::json::StringAt(root, "vad_last_error");
	s.last_segment_reason = openvr_pair::common::json::StringAt(root, "last_segment_reason");
	s.last_segment_audio_ms =
	    static_cast<long long>(openvr_pair::common::json::NumberAt(root, "last_segment_audio_ms"));
	s.last_segment_evidence_ms =
	    static_cast<long long>(openvr_pair::common::json::NumberAt(root, "last_segment_evidence_ms"));
	s.last_transcribe_ms = static_cast<long long>(openvr_pair::common::json::NumberAt(root, "last_transcribe_ms"));
	s.last_segment_vad_probability =
	    static_cast<float>(openvr_pair::common::json::NumberAt(root, "last_segment_vad_probability"));
	s.last_segment_peak = static_cast<float>(openvr_pair::common::json::NumberAt(root, "last_segment_peak"));
	s.last_segment_threshold = static_cast<float>(openvr_pair::common::json::NumberAt(root, "last_segment_threshold"));
	s.last_segment_rms = static_cast<float>(openvr_pair::common::json::NumberAt(root, "last_segment_rms"));
	s.last_segment_rms_threshold =
	    static_cast<float>(openvr_pair::common::json::NumberAt(root, "last_segment_rms_threshold"));
	s.prompt_context_chars = static_cast<long long>(openvr_pair::common::json::NumberAt(root, "prompt_context_chars"));
	s.last_suppression_reason = openvr_pair::common::json::StringAt(root, "last_suppression_reason");
	s.suppressed_transcripts =
	    static_cast<long long>(openvr_pair::common::json::NumberAt(root, "suppressed_transcripts"));
	s.suppressed_non_speech =
	    static_cast<long long>(openvr_pair::common::json::NumberAt(root, "suppressed_non_speech"));
	s.suppressed_no_speech_probability =
	    static_cast<long long>(openvr_pair::common::json::NumberAt(root, "suppressed_no_speech_probability"));
	s.suppressed_common_hallucination =
	    static_cast<long long>(openvr_pair::common::json::NumberAt(root, "suppressed_common_hallucination"));
	s.suppressed_common_filler =
	    static_cast<long long>(openvr_pair::common::json::NumberAt(root, "suppressed_common_filler"));
	s.suppressed_short_weak_audio =
	    static_cast<long long>(openvr_pair::common::json::NumberAt(root, "suppressed_short_weak_audio"));
	s.suppressed_repetitive =
	    static_cast<long long>(openvr_pair::common::json::NumberAt(root, "suppressed_repetitive"));
	s.suppressed_low_confidence =
	    static_cast<long long>(openvr_pair::common::json::NumberAt(root, "suppressed_low_confidence"));
	s.suppressed_slow_short_decode =
	    static_cast<long long>(openvr_pair::common::json::NumberAt(root, "suppressed_slow_short_decode"));

	snapshot_ = std::move(s);
}

} // namespace captions
