#pragma once

#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <mmdeviceapi.h>
#include <audioclient.h>

#include <atomic>
#include <cstdint>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

// WASAPI shared-mode microphone capture.
//
// Captures audio from the default input endpoint in shared mode, converts to
// 16 kHz mono float32, and calls FrameCallback with non-overlapping 30 ms
// chunks (~480 samples each). Thread-safe start/stop; device-change handling
// via IMMNotificationClient restarts the capture session automatically.
class WasapiCapture
{
public:
	// Called on the capture thread with 30 ms of 16 kHz mono PCM (float32).
	using FrameCallback = std::function<void(const float*, size_t)>;

	WasapiCapture();
	~WasapiCapture();

	// Open the default mic and start the capture thread. Returns false on failure.
	// callback is called on the capture thread -- do not block it.
	bool Start(FrameCallback callback);

	// Stop capture and release COM resources.
	void Stop();

	bool IsCapturing() const noexcept;

	// Friendly name of the current capture device, or empty if not open.
	std::string DeviceName() const;

	// Enumerate all active capture endpoint friendly names.
	static std::vector<std::string> EnumerateDevices();

private:
	FrameCallback callback_;
	std::atomic<bool> running_{false};
	std::thread capture_thread_;

	// COM objects owned by the capture thread.
	IMMDeviceEnumerator* enumerator_ = nullptr;
	IAudioClient* audio_client_ = nullptr;
	IAudioCaptureClient* capture_client_ = nullptr;

	mutable std::mutex name_mutex_;
	std::string device_name_;

	// 30 ms PCM accumulator (16 kHz mono float32).
	std::vector<float> accum_;
	static constexpr size_t kFrameSamples = 480; // 30 ms at 16 kHz

	// Per-packet downmix scratch (capture thread only). Reused across packets
	// so a 100 Hz capture loop does not allocate every frame.
	std::vector<float> mono_scratch_;

	void CaptureLoop();
	bool InitCom();
	void ReleaseCom();
	bool OpenDefaultDevice();

	// Resample `in_samples` frames from `in_rate` Hz mono float to 16 kHz mono float.
	// Appends resampled frames to accum_ and flushes 30 ms chunks via callback_.
	void ResampleAndAccumulate(const float* data, size_t frames, uint32_t in_rate);
};
