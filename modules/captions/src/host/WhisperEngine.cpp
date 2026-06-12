#define _CRT_SECURE_NO_DEPRECATE
#include "WhisperEngine.h"
#include "Logging.h"

// whisper.cpp is vendored under lib/whisper.cpp/.
// The CMakeLists links whisper.lib / whisper.dll.
#include <whisper.h>

#if defined(WKOPENVR_CAPTIONS_VULKAN_ENABLED)
#include <ggml-vulkan.h>
#endif

#include <cmath>
#include <cstring>
#include <algorithm>
#include <chrono>
#include <fstream>
#include <sstream>

WhisperEngine::WhisperEngine() = default;

namespace {
long long SamplesToAudioMs(size_t samples)
{
	return static_cast<long long>((samples * 1000ULL) / 16000ULL);
}

long long ElapsedMs(std::chrono::steady_clock::time_point start, std::chrono::steady_clock::time_point end)
{
	return std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
}

long long FileSizeBytes(const std::string& path)
{
	std::ifstream file(path, std::ios::binary | std::ios::ate);
	if (!file) return -1;
	return static_cast<long long>(file.tellg());
}
} // namespace

WhisperEngine::~WhisperEngine()
{
	Unload();
}

bool WhisperEngine::Load(const std::string& model_path, int n_threads, bool use_gpu)
{
	Unload();

	n_threads_ = n_threads;

	whisper_context_params cparams = whisper_context_default_params();
	cparams.use_gpu = use_gpu;

	const auto load_start = std::chrono::steady_clock::now();
	ctx_ = whisper_init_from_file_with_params(model_path.c_str(), cparams);
	const long long load_ms = ElapsedMs(load_start, std::chrono::steady_clock::now());
	if (!ctx_) {
		TH_LOG("[whisper] failed to load model from '%s' load_ms=%lld", model_path.c_str(), load_ms);
		return false;
	}

	// Record the compute backend that ended up active. The caller only passes
	// use_gpu=true after a guarded device probe found at least one GPU, so the
	// Vulkan instance is already initialized here and querying it is safe.
	backend_info_ = WhisperBackendInfo{};
	backend_info_.gpu_requested = use_gpu;
	backend_info_.gpu_active = false;
	backend_info_.device_name = "CPU";
#if defined(WKOPENVR_CAPTIONS_VULKAN_ENABLED)
	if (use_gpu) {
		char vk_desc[256] = {};
		ggml_backend_vk_get_device_description(0, vk_desc, sizeof(vk_desc));
		backend_info_.gpu_active = true;
		backend_info_.device_name = vk_desc[0] ? (std::string("Vulkan: ") + vk_desc) : std::string("Vulkan");
	}
#endif

	const long long size_bytes = FileSizeBytes(model_path);
	const double size_mb = size_bytes >= 0 ? static_cast<double>(size_bytes) / (1024.0 * 1024.0) : -1.0;
	TH_LOG("[whisper] model loaded from '%s' (gpu=%d backend=%s threads=%d load_ms=%lld size_mb=%.1f)",
	       model_path.c_str(), (int)use_gpu, backend_info_.device_name.c_str(), n_threads_, load_ms, size_mb);
	return true;
}

void WhisperEngine::Unload()
{
	if (ctx_) {
		whisper_free(ctx_);
		ctx_ = nullptr;
	}
	backend_info_ = WhisperBackendInfo{};
}

void WhisperEngine::SetLanguage(const std::string& lang)
{
	language_hint_ = lang;
}

void WhisperEngine::SetInitialPrompt(const std::string& prompt)
{
	initial_prompt_ = prompt;
}

std::string WhisperEngine::Transcribe(const std::vector<float>& pcm16k, std::string* detected_lang_out)
{
	const WhisperTranscriptResult result = TranscribeDetailed(pcm16k);
	if (detected_lang_out) *detected_lang_out = result.detected_lang;
	return result.text;
}

WhisperTranscriptResult WhisperEngine::TranscribeDetailed(const std::vector<float>& pcm16k,
                                                          const WhisperDecodeOptions& options)
{
	WhisperTranscriptResult result;
	result.audio_ms = SamplesToAudioMs(pcm16k.size());
	if (!ctx_ || pcm16k.empty()) return result;

	whisper_full_params params = whisper_full_default_params(WHISPER_SAMPLING_GREEDY);
	params.n_threads = n_threads_;
	params.no_context = true; // clean chunk boundary
	params.n_max_text_ctx = 128;
	params.no_timestamps = true;
	params.single_segment = false;
	params.translate = false; // translation handled downstream
	params.initial_prompt = initial_prompt_.empty() ? nullptr : initial_prompt_.c_str();
	params.no_speech_thold = options.use_no_speech_threshold ? options.no_speech_threshold : 1.1f;
	params.suppress_nst = options.suppress_non_speech_tokens;

	if (!language_hint_.empty() && language_hint_ != "auto") {
		params.language = language_hint_.c_str();
		params.detect_language = false;
	}
	else {
		params.language = nullptr;
		params.detect_language = true;
	}

	// Suppress printing to stderr.
	params.print_special = false;
	params.print_progress = false;
	params.print_realtime = false;
	params.print_timestamps = false;

	const auto decode_start = std::chrono::steady_clock::now();
	int rc = whisper_full(ctx_, params, pcm16k.data(), static_cast<int>(pcm16k.size()));
	result.decode_ms = ElapsedMs(decode_start, std::chrono::steady_clock::now());
	if (rc != 0) {
		TH_LOG("[whisper] whisper_full returned %d audio_ms=%lld decode_ms=%lld", rc, result.audio_ms,
		       result.decode_ms);
		return result;
	}

	result.succeeded = true;

	int lang_id = whisper_full_lang_id(ctx_);
	result.detected_lang = (lang_id >= 0) ? std::string(whisper_lang_str(lang_id)) : std::string("?");

	std::ostringstream oss;
	const int nseg = whisper_full_n_segments(ctx_);
	result.segment_count = nseg;
	double sum_log_probability = 0.0;
	for (int i = 0; i < nseg; ++i) {
		const char* txt = whisper_full_get_segment_text(ctx_, i);
		if (txt) oss << txt;

		result.max_no_speech_probability =
		    std::max(result.max_no_speech_probability, whisper_full_get_segment_no_speech_prob(ctx_, i));

		const int ntok = whisper_full_n_tokens(ctx_, i);
		for (int j = 0; j < ntok; ++j) {
			const whisper_token_data token = whisper_full_get_token_data(ctx_, i, j);
			if (std::isfinite(token.plog) && token.p > 0.0f) {
				sum_log_probability += token.plog;
				++result.token_count;
			}
		}
	}

	result.average_token_log_probability =
	    result.token_count > 0 ? static_cast<float>(sum_log_probability / result.token_count) : 0.0f;

	result.text = oss.str();
	// Strip leading whitespace that whisper often prepends.
	size_t start = result.text.find_first_not_of(" \t\r\n");
	if (start != std::string::npos) result.text = result.text.substr(start);

	return result;
}
