#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>

// Forward-declare ORT types to avoid pulling in the full header here.
struct OrtApi;
struct OrtEnv;
struct OrtSession;
struct OrtSessionOptions;
struct OrtMemoryInfo;

// Silero VAD ONNX session wrapper.
//
// Feeds 30 ms mono float32 frames (512 samples at 16 kHz) through the Silero
// VAD model and returns a speech probability. The caller decides the
// speech-start and speech-end thresholds.
class SileroVad
{
public:
	SileroVad();
	~SileroVad();

	static bool RuntimeAvailable();

	// Load the ONNX model at `model_path`. Returns false on failure.
	bool Load(const std::string& model_path);

	// Feed one 30 ms frame (exactly 512 float samples). Returns speech
	// probability in [0, 1]. Returns -1 on inference error.
	float Feed(const float* samples, size_t count);

	bool IsLoaded() const noexcept { return session_ != nullptr; }
	uint64_t InferenceFailures() const noexcept { return inference_failures_; }
	const std::string& LastError() const noexcept { return last_error_; }

	// Reset LSTM state (call at speech-end boundary).
	void Reset();

private:
	const OrtApi* ort_api_ = nullptr;
	OrtEnv* env_ = nullptr;
	OrtSession* session_ = nullptr;
	OrtMemoryInfo* mem_info_ = nullptr;

	// Silero VAD v4 stateful tensors.
	// h and c are [2, 1, 64] float; sr is [1] int64 = 16000.
	static constexpr int kStateSize = 2 * 1 * 64;
	float h_[kStateSize] = {};
	float c_[kStateSize] = {};
	uint64_t inference_failures_ = 0;
	std::string last_error_;

	void ResetState();
	void RecordInferenceFailure(const char* stage, const char* message);
};
