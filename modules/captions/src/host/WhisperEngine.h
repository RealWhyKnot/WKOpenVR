#pragma once

#include <cstddef>
#include <memory>
#include <string>
#include <vector>

// Forward declarations to avoid pulling in whisper.h in every TU.
struct whisper_context;
struct whisper_full_params;

// Wrapper around whisper.cpp.
//
// Loads a ggml model file at Load() time. Transcribe() accepts a buffer of
// 16 kHz mono float32 samples and returns the transcript text. Detected
// language is written to detected_lang_out when non-null.
class WhisperEngine
{
public:
	WhisperEngine();
	~WhisperEngine();

	// Load model from file. Returns false on failure.
	bool Load(const std::string& model_path, int n_threads = 4, bool use_gpu = false);

	void Unload();

	bool IsLoaded() const noexcept { return ctx_ != nullptr; }

	// Set source language. "auto" = whisper auto-detect (adds ~50-100 ms).
	void SetLanguage(const std::string& lang);

	// Set short text context for the next transcription chunk.
	void SetInitialPrompt(const std::string& prompt);

	// Transcribe `frames` of 16 kHz mono float32 PCM.
	// Returns the concatenated segment text. Writes the detected language
	// BCP-47 code to `*detected_lang_out` if non-null.
	std::string Transcribe(const std::vector<float>& pcm16k, std::string* detected_lang_out = nullptr);

private:
	whisper_context* ctx_ = nullptr;
	std::string language_hint_; // "" -> "auto"
	std::string initial_prompt_;
	int n_threads_ = 4;
};
