#define _CRT_SECURE_NO_DEPRECATE
#include "SileroVad.h"
#include "Logging.h"

#include <algorithm>
#include <cstring>
#include <stdexcept>
#include <string>
#include <vector>

#define WIN32_LEAN_AND_MEAN
#include <windows.h>

// ONNX Runtime is vendored under lib/onnxruntime/. When it is absent the
// CMakeLists builds the host without HAVE_ORT defined; the whole
// implementation collapses to stubs that log a clear "not vendored" error
// at runtime. The point is letting a C++-only contributor build + run the
// driver + overlay without the inference-library dependency drop.
#ifdef HAVE_ORT
#include <onnxruntime_c_api.h>
#endif

#ifdef HAVE_ORT

// Silero VAD v4 input names (from the ONNX model's graph).
static const char* kInputNames[] = {"input", "sr", "h", "c"};
static const char* kOutputNames[] = {"output", "hn", "cn"};

SileroVad::SileroVad()
{
	ort_api_ = OrtGetApiBase()->GetApi(ORT_API_VERSION);
}

SileroVad::~SileroVad()
{
	if (session_) {
		ort_api_->ReleaseSession(session_);
		session_ = nullptr;
	}
	if (mem_info_) {
		ort_api_->ReleaseMemoryInfo(mem_info_);
		mem_info_ = nullptr;
	}
	if (env_) {
		ort_api_->ReleaseEnv(env_);
		env_ = nullptr;
	}
}

bool SileroVad::RuntimeAvailable()
{
	HMODULE h = LoadLibraryW(L"onnxruntime.dll");
	if (!h) return false;
	FreeLibrary(h);
	return true;
}

bool SileroVad::Load(const std::string& model_path)
{
	if (!ort_api_) {
		TH_LOG("[vad] ORT API not available");
		return false;
	}

	OrtStatus* status = nullptr;

	status = ort_api_->CreateEnv(ORT_LOGGING_LEVEL_WARNING, "silero_vad", &env_);
	if (status) {
		TH_LOG("[vad] CreateEnv failed: %s", ort_api_->GetErrorMessage(status));
		ort_api_->ReleaseStatus(status);
		return false;
	}

	OrtSessionOptions* opts = nullptr;
	ort_api_->CreateSessionOptions(&opts);
	ort_api_->SetIntraOpNumThreads(opts, 1);
	ort_api_->SetInterOpNumThreads(opts, 1);
	ort_api_->SetSessionGraphOptimizationLevel(opts, ORT_ENABLE_BASIC);

	// Convert UTF-8 path to wide for CreateSession on Windows.
	int wlen = MultiByteToWideChar(CP_UTF8, 0, model_path.c_str(), -1, nullptr, 0);
	std::wstring wpath(wlen, L'\0');
	MultiByteToWideChar(CP_UTF8, 0, model_path.c_str(), -1, wpath.data(), wlen);

	status = ort_api_->CreateSession(env_, wpath.c_str(), opts, &session_);
	ort_api_->ReleaseSessionOptions(opts);
	if (status) {
		TH_LOG("[vad] CreateSession failed: %s", ort_api_->GetErrorMessage(status));
		ort_api_->ReleaseStatus(status);
		session_ = nullptr;
		return false;
	}

	status = ort_api_->CreateCpuMemoryInfo(OrtArenaAllocator, OrtMemTypeDefault, &mem_info_);
	if (status) {
		TH_LOG("[vad] CreateCpuMemoryInfo failed: %s", ort_api_->GetErrorMessage(status));
		ort_api_->ReleaseStatus(status);
		return false;
	}

	ResetState();
	TH_LOG("[vad] model loaded from '%s'", model_path.c_str());
	return true;
}

void SileroVad::Reset()
{
	ResetState();
}

void SileroVad::ResetState()
{
	memset(h_, 0, sizeof(h_));
	memset(c_, 0, sizeof(c_));
}

void SileroVad::RecordInferenceFailure(const char* stage, const char* message)
{
	++inference_failures_;
	last_error_ = stage ? stage : "unknown";
	if (message && message[0]) {
		last_error_ += ": ";
		last_error_ += message;
	}
	if (inference_failures_ <= 3 || (inference_failures_ % 100) == 0) {
		TH_LOG("[vad] inference failure count=%llu stage=%s error=%s",
		       static_cast<unsigned long long>(inference_failures_), stage ? stage : "unknown",
		       message && message[0] ? message : "(none)");
	}
}

float SileroVad::Feed(const float* samples, size_t count)
{
	if (!session_ || !ort_api_ || !mem_info_) return -1.f;
	if (count != 512) {
		RecordInferenceFailure("input-size", count < 512 ? "short frame" : "long frame");
		return -1.f;
	}

	OrtStatus* status = nullptr;

	// input tensor: [1, 512] float
	{
		int64_t input_shape[] = {1, 512};
		OrtValue* input_tensor = nullptr;
		status = ort_api_->CreateTensorWithDataAsOrtValue(mem_info_, const_cast<float*>(samples), 512 * sizeof(float),
		                                                  input_shape, 2, ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT,
		                                                  &input_tensor);
		if (status) {
			RecordInferenceFailure("input-tensor", ort_api_->GetErrorMessage(status));
			ort_api_->ReleaseStatus(status);
			return -1.f;
		}

		// sr tensor: [1] int64 = 16000
		int64_t sr_val = 16000;
		int64_t sr_shape[] = {1};
		OrtValue* sr_tensor = nullptr;
		status = ort_api_->CreateTensorWithDataAsOrtValue(mem_info_, &sr_val, sizeof(int64_t), sr_shape, 1,
		                                                  ONNX_TENSOR_ELEMENT_DATA_TYPE_INT64, &sr_tensor);
		if (status) {
			RecordInferenceFailure("sample-rate-tensor", ort_api_->GetErrorMessage(status));
			ort_api_->ReleaseStatus(status);
			ort_api_->ReleaseValue(input_tensor);
			return -1.f;
		}

		// h and c tensors: [2, 1, 64] float
		int64_t hc_shape[] = {2, 1, 64};
		OrtValue *h_tensor = nullptr, *c_tensor = nullptr;
		status = ort_api_->CreateTensorWithDataAsOrtValue(mem_info_, h_, kStateSize * sizeof(float), hc_shape, 3,
		                                                  ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT, &h_tensor);
		if (status) {
			RecordInferenceFailure("h-state-tensor", ort_api_->GetErrorMessage(status));
			ort_api_->ReleaseStatus(status);
			ort_api_->ReleaseValue(input_tensor);
			ort_api_->ReleaseValue(sr_tensor);
			return -1.f;
		}
		status = ort_api_->CreateTensorWithDataAsOrtValue(mem_info_, c_, kStateSize * sizeof(float), hc_shape, 3,
		                                                  ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT, &c_tensor);
		if (status) {
			RecordInferenceFailure("c-state-tensor", ort_api_->GetErrorMessage(status));
			ort_api_->ReleaseStatus(status);
			ort_api_->ReleaseValue(input_tensor);
			ort_api_->ReleaseValue(sr_tensor);
			ort_api_->ReleaseValue(h_tensor);
			return -1.f;
		}

		const OrtValue* inputs[] = {input_tensor, sr_tensor, h_tensor, c_tensor};
		OrtValue* outputs[] = {nullptr, nullptr, nullptr};

		status = ort_api_->Run(session_, nullptr, kInputNames, inputs, 4, kOutputNames, 3, outputs);

		float prob = -1.f;
		if (!status) {
			// Extract output probability.
			float* out_data = nullptr;
			if (!outputs[0]) {
				RecordInferenceFailure("output", "missing probability tensor");
			}
			else {
				ort_api_->GetTensorMutableData(outputs[0], reinterpret_cast<void**>(&out_data));
			}
			if (out_data) prob = out_data[0];

			// Update LSTM state from hn/cn.
			float *hn_data = nullptr, *cn_data = nullptr;
			if (outputs[1]) ort_api_->GetTensorMutableData(outputs[1], reinterpret_cast<void**>(&hn_data));
			if (outputs[2]) ort_api_->GetTensorMutableData(outputs[2], reinterpret_cast<void**>(&cn_data));
			if (hn_data) memcpy(h_, hn_data, kStateSize * sizeof(float));
			if (cn_data) memcpy(c_, cn_data, kStateSize * sizeof(float));
		}
		else {
			RecordInferenceFailure("run", ort_api_->GetErrorMessage(status));
			ort_api_->ReleaseStatus(status);
		}

		for (auto* v : outputs)
			if (v) ort_api_->ReleaseValue(v);
		ort_api_->ReleaseValue(input_tensor);
		ort_api_->ReleaseValue(sr_tensor);
		ort_api_->ReleaseValue(h_tensor);
		ort_api_->ReleaseValue(c_tensor);

		return prob;
	}
}

#else // !HAVE_ORT

// Build-without-ORT stubs. The host links and runs, but speech detection
// never fires. The host-status log line on first Load() call tells anyone
// looking at the log how to get the real implementation.
SileroVad::SileroVad() = default;
SileroVad::~SileroVad() = default;
bool SileroVad::RuntimeAvailable()
{
	return false;
}
bool SileroVad::Load(const std::string&)
{
	static bool logged = false;
	if (!logged) {
		TH_LOG("[vad] ONNX Runtime was not vendored at build time. The "
		       "captions host built in stub mode -- VAD will not fire. "
		       "Drop the prebuilt onnxruntime Windows tree into lib/onnxruntime/ "
		       "(headers + lib + dll) and rebuild to enable speech detection.");
		logged = true;
	}
	return false;
}
void SileroVad::Reset() {}
void SileroVad::ResetState() {}
float SileroVad::Feed(const float*, size_t)
{
	return -1.f;
}

#endif // HAVE_ORT
