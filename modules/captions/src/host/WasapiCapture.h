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
// 16 kHz mono float32, and calls FrameCallback with non-overlapping 32 ms
// chunks (512 samples each). Thread-safe start/stop; device-change handling
// via IMMNotificationClient restarts the capture session automatically.
class WasapiCapture
{
public:
	// Called on the capture thread with 32 ms of 16 kHz mono PCM (float32).
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

	// Select the capture endpoint by its WASAPI endpoint id (from
	// IMMDevice::GetId). An empty id means "system default" -- the historical
	// behaviour. Safe to call from any thread; the capture thread re-opens the
	// new device on its next iteration.
	void SetDevice(const std::string& endpointId);

	// Most recent input level on a 0..1 scale (decayed peak). 0 while silent.
	float Level() const noexcept { return peak_level_.load(std::memory_order_relaxed); }

	// Total audio frames received from the device since start. Advances only
	// while the endpoint is actually delivering packets -- this distinguishes a
	// silent-but-live device from one delivering nothing at all.
	uint64_t FramesCaptured() const noexcept { return frames_captured_.load(std::memory_order_relaxed); }

	// Enumerate all active capture endpoint friendly names.
	static std::vector<std::string> EnumerateDevices();

private:
	FrameCallback callback_;
	std::atomic<bool> running_{false};
	std::thread capture_thread_;

	// Desired endpoint id (empty = system default) and a dirty flag the capture
	// loop watches so a mid-session device change re-opens cleanly.
	mutable std::mutex device_mutex_;
	std::string desired_device_id_;
	std::atomic<bool> device_dirty_{false};

	// Live instrumentation (written on the capture thread, read elsewhere).
	std::atomic<float> peak_level_{0.0f};
	std::atomic<uint64_t> frames_captured_{0};

	// COM objects owned by the capture thread.
	IMMDeviceEnumerator* enumerator_ = nullptr;
	IAudioClient* audio_client_ = nullptr;
	IAudioCaptureClient* capture_client_ = nullptr;

	mutable std::mutex name_mutex_;
	std::string device_name_;

	// 32 ms PCM accumulator (16 kHz mono float32). Silero VAD v4 expects
	// exactly 512 samples at 16 kHz.
	std::vector<float> accum_;
	static constexpr size_t kFrameSamples = 512; // 32 ms at 16 kHz

	// Per-packet downmix scratch (capture thread only). Reused across packets
	// so a 100 Hz capture loop does not allocate every frame.
	std::vector<float> mono_scratch_;

	void CaptureLoop();
	bool InitCom();
	void ReleaseCom();
	// Open the desired endpoint (or the system default when none is selected).
	// Clears device_dirty_ and falls back to the default if a selected id has
	// gone away.
	bool OpenSelectedDevice();

	// Resample `in_samples` frames from `in_rate` Hz mono float to 16 kHz mono float.
	// Appends resampled frames to accum_ and flushes 32 ms chunks via callback_.
	void ResampleAndAccumulate(const float* data, size_t frames, uint32_t in_rate);
};
