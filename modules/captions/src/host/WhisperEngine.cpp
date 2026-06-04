#define _CRT_SECURE_NO_DEPRECATE
#include "WhisperEngine.h"
#include "Logging.h"

// whisper.cpp is vendored under lib/whisper.cpp/.
// The CMakeLists links whisper.lib / whisper.dll.
#include <whisper.h>

#include <cstring>
#include <sstream>

WhisperEngine::WhisperEngine() = default;

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

	ctx_ = whisper_init_from_file_with_params(model_path.c_str(), cparams);
	if (!ctx_) {
		TH_LOG("[whisper] failed to load model from '%s'", model_path.c_str());
		return false;
	}

	TH_LOG("[whisper] model loaded from '%s' (gpu=%d threads=%d)", model_path.c_str(), (int)use_gpu, n_threads_);
	return true;
}

void WhisperEngine::Unload()
{
	if (ctx_) {
		whisper_free(ctx_);
		ctx_ = nullptr;
	}
}

void WhisperEngine::SetLanguage(const std::string& lang)
{
	language_hint_ = lang;
}

std::string WhisperEngine::Transcribe(const std::vector<float>& pcm16k, std::string* detected_lang_out)
{
	if (!ctx_ || pcm16k.empty()) return {};

	whisper_full_params params = whisper_full_default_params(WHISPER_SAMPLING_GREEDY);
	params.n_threads = n_threads_;
	params.no_context = true; // clean chunk boundary
	params.single_segment = false;
	params.translate = false; // translation handled downstream

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

	int rc = whisper_full(ctx_, params, pcm16k.data(), static_cast<int>(pcm16k.size()));
	if (rc != 0) {
		TH_LOG("[whisper] whisper_full returned %d", rc);
		return {};
	}

	if (detected_lang_out) {
		int lang_id = whisper_full_lang_id(ctx_);
		*detected_lang_out = (lang_id >= 0) ? std::string(whisper_lang_str(lang_id)) : std::string("?");
	}

	std::ostringstream oss;
	const int nseg = whisper_full_n_segments(ctx_);
	for (int i = 0; i < nseg; ++i) {
		const char* txt = whisper_full_get_segment_text(ctx_, i);
		if (txt) oss << txt;
	}

	std::string result = oss.str();
	// Strip leading whitespace that whisper often prepends.
	size_t start = result.find_first_not_of(" \t\r\n");
	if (start != std::string::npos) result = result.substr(start);

	return result;
}
