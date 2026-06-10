#define _CRT_SECURE_NO_DEPRECATE
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <mmdeviceapi.h>
#include <audioclient.h>
#include <functiondiscoverykeys_devpkey.h>
#include <Objbase.h>
#include <wrl/client.h>

#include "WasapiCapture.h"
#include "AudioLevel.h"
#include "Logging.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>

using Microsoft::WRL::ComPtr;

WasapiCapture::WasapiCapture() = default;

WasapiCapture::~WasapiCapture()
{
	Stop();
}

bool WasapiCapture::Start(FrameCallback callback)
{
	if (running_.load(std::memory_order_acquire)) return true;
	callback_ = std::move(callback);
	running_.store(true, std::memory_order_release);
	capture_thread_ = std::thread([this] { CaptureLoop(); });
	return true;
}

void WasapiCapture::Stop()
{
	running_.store(false, std::memory_order_release);
	if (capture_thread_.joinable()) capture_thread_.join();
}

bool WasapiCapture::IsCapturing() const noexcept
{
	return running_.load(std::memory_order_acquire);
}

std::string WasapiCapture::DeviceName() const
{
	std::lock_guard<std::mutex> lk(name_mutex_);
	return device_name_;
}

void WasapiCapture::SetDevice(const std::string& endpointId)
{
	{
		std::lock_guard<std::mutex> lk(device_mutex_);
		if (desired_device_id_ == endpointId) return; // no change
		desired_device_id_ = endpointId;
	}
	device_dirty_.store(true, std::memory_order_release);
	TH_LOG("[wasapi] device selection changed; will reopen (id='%s')",
	       endpointId.empty() ? "(system default)" : endpointId.c_str());
}

std::vector<std::string> WasapiCapture::EnumerateDevices()
{
	std::vector<std::string> names;

	CoInitializeEx(nullptr, COINIT_MULTITHREADED);

	ComPtr<IMMDeviceEnumerator> enumerator;
	HRESULT hr = CoCreateInstance(__uuidof(MMDeviceEnumerator), nullptr, CLSCTX_ALL, IID_PPV_ARGS(&enumerator));
	if (FAILED(hr)) {
		CoUninitialize();
		return names;
	}

	ComPtr<IMMDeviceCollection> collection;
	hr = enumerator->EnumAudioEndpoints(eCapture, DEVICE_STATE_ACTIVE, &collection);
	if (FAILED(hr)) {
		CoUninitialize();
		return names;
	}

	UINT count = 0;
	collection->GetCount(&count);
	for (UINT i = 0; i < count; ++i) {
		ComPtr<IMMDevice> device;
		if (FAILED(collection->Item(i, &device))) continue;
		ComPtr<IPropertyStore> props;
		if (FAILED(device->OpenPropertyStore(STGM_READ, &props))) continue;
		PROPVARIANT pv;
		PropVariantInit(&pv);
		if (SUCCEEDED(props->GetValue(PKEY_Device_FriendlyName, &pv)) && pv.vt == VT_LPWSTR && pv.pwszVal) {
			int n = WideCharToMultiByte(CP_UTF8, 0, pv.pwszVal, -1, nullptr, 0, nullptr, nullptr);
			if (n > 0) {
				std::string s(n, '\0');
				WideCharToMultiByte(CP_UTF8, 0, pv.pwszVal, -1, s.data(), n, nullptr, nullptr);
				s.resize(s.size() - 1); // drop NUL
				names.push_back(std::move(s));
			}
		}
		PropVariantClear(&pv);
	}

	CoUninitialize();
	return names;
}

// ---------------------------------------------------------------------------
// Capture loop
// ---------------------------------------------------------------------------

void WasapiCapture::CaptureLoop()
{
	CoInitializeEx(nullptr, COINIT_MULTITHREADED);

	int backoff_ms = 500;
	constexpr int kMaxBackoffMs = 10000;

	while (running_.load(std::memory_order_acquire)) {
		if (!OpenSelectedDevice()) {
			TH_LOG("[wasapi] failed to open capture device; retry in %d ms", backoff_ms);
			Sleep(static_cast<DWORD>(backoff_ms));
			backoff_ms = std::min(backoff_ms * 2, kMaxBackoffMs);
			continue;
		}
		backoff_ms = 500;

		// Retrieve mix format.
		WAVEFORMATEX* mixFmt = nullptr;
		HRESULT hr = audio_client_->GetMixFormat(&mixFmt);
		if (FAILED(hr)) {
			ReleaseCom();
			continue;
		}

		// We'll request 100 ms buffer.
		hr = audio_client_->Initialize(AUDCLNT_SHAREMODE_SHARED, 0, 1000000 /* 100 ms in 100-ns units */, 0, mixFmt,
		                               nullptr);
		if (FAILED(hr)) {
			CoTaskMemFree(mixFmt);
			ReleaseCom();
			continue;
		}

		hr = audio_client_->GetService(IID_PPV_ARGS(&capture_client_));
		if (FAILED(hr)) {
			CoTaskMemFree(mixFmt);
			ReleaseCom();
			continue;
		}

		UINT32 nChannels = mixFmt->nChannels;
		UINT32 nSampleRate = mixFmt->nSamplesPerSec;
		WORD bitsPerSample = mixFmt->wBitsPerSample;
		bool isFloat = (mixFmt->wFormatTag == WAVE_FORMAT_IEEE_FLOAT) ||
		               (mixFmt->wFormatTag == WAVE_FORMAT_EXTENSIBLE &&
		                reinterpret_cast<WAVEFORMATEXTENSIBLE*>(mixFmt)->SubFormat == KSDATAFORMAT_SUBTYPE_IEEE_FLOAT);
		// Read every field we need before freeing mixFmt; dereferencing it after
		// CoTaskMemFree would be a use-after-free.
		CoTaskMemFree(mixFmt);
		mixFmt = nullptr;

		if (!isFloat && bitsPerSample != 16) {
			TH_LOG("[wasapi] unsupported sample format; bits=%u", (unsigned)bitsPerSample);
			ReleaseCom();
			continue;
		}

		hr = audio_client_->Start();
		if (FAILED(hr)) {
			ReleaseCom();
			continue;
		}

		const std::string opened_device_name = DeviceName();
		TH_LOG("[wasapi] capture started device='%s' rate=%u channels=%u format=%s bits=%u", opened_device_name.c_str(),
		       (unsigned)nSampleRate, (unsigned)nChannels, isFloat ? "float" : "pcm", (unsigned)bitsPerSample);

		// Per-tick decay so the level meter falls back toward silence between
		// utterances and reaches 0 when the device stops delivering audio.
		constexpr float kLevelDecay = 0.85f;

		while (running_.load(std::memory_order_acquire)) {
			Sleep(10); // 10 ms polling

			// A device selection change tears down and re-opens cleanly.
			if (device_dirty_.load(std::memory_order_acquire)) break;

			float tickPeak = 0.0f;

			UINT32 packetLen = 0;
			hr = capture_client_->GetNextPacketSize(&packetLen);
			if (FAILED(hr)) break;

			while (packetLen > 0) {
				BYTE* data = nullptr;
				UINT32 frames = 0;
				DWORD flags = 0;
				hr = capture_client_->GetBuffer(&data, &frames, &flags, nullptr, nullptr);
				if (FAILED(hr)) {
					packetLen = 0;
					break;
				}

				if (frames > 0) {
					// Count every delivered frame, even silent ones: a live but
					// muted endpoint advances this counter while a dead endpoint
					// (e.g. an idle virtual cable) does not.
					frames_captured_.fetch_add(frames, std::memory_order_relaxed);

					if (!(flags & AUDCLNT_BUFFERFLAGS_SILENT) && data) {
						// Convert to float mono. mono_scratch_ persists across
						// packets so the 100 Hz capture loop does not allocate.
						mono_scratch_.resize(frames);
						if (isFloat) {
							const float* src = reinterpret_cast<const float*>(data);
							for (UINT32 i = 0; i < frames; ++i) {
								float s = 0.f;
								for (UINT32 ch = 0; ch < nChannels; ++ch)
									s += src[i * nChannels + ch];
								mono_scratch_[i] = s / static_cast<float>(nChannels);
							}
						}
						else {
							// PCM16
							const int16_t* src = reinterpret_cast<const int16_t*>(data);
							for (UINT32 i = 0; i < frames; ++i) {
								float s = 0.f;
								for (UINT32 ch = 0; ch < nChannels; ++ch)
									s += static_cast<float>(src[i * nChannels + ch]) / 32768.f;
								mono_scratch_[i] = s / static_cast<float>(nChannels);
							}
						}
						float pk = captions::ComputeBufferPeak(mono_scratch_.data(), frames);
						if (pk > tickPeak) tickPeak = pk;
						ResampleAndAccumulate(mono_scratch_.data(), frames, nSampleRate);
					}
				}

				capture_client_->ReleaseBuffer(frames);
				hr = capture_client_->GetNextPacketSize(&packetLen);
				if (FAILED(hr)) {
					packetLen = 0;
					break;
				}
			}

			// Rise instantly to the loudest packet this tick; otherwise decay.
			float cur = peak_level_.load(std::memory_order_relaxed);
			peak_level_.store(std::max(tickPeak, cur * kLevelDecay), std::memory_order_relaxed);
		}

		audio_client_->Stop();
		ReleaseCom();
		peak_level_.store(0.0f, std::memory_order_relaxed);
	}

	CoUninitialize();
	TH_LOG("[wasapi] capture thread exiting");
}

bool WasapiCapture::OpenSelectedDevice()
{
	HRESULT hr = CoCreateInstance(__uuidof(MMDeviceEnumerator), nullptr, CLSCTX_ALL, IID_PPV_ARGS(&enumerator_));
	if (FAILED(hr)) return false;

	// Snapshot the desired endpoint id and consume the dirty flag together so a
	// SetDevice() racing this read either lands here or trips the flag again.
	std::string wantId;
	{
		std::lock_guard<std::mutex> lk(device_mutex_);
		wantId = desired_device_id_;
		device_dirty_.store(false, std::memory_order_release);
	}

	ComPtr<IMMDevice> device;
	bool fell_back_to_default = false;
	if (!wantId.empty()) {
		int wn = MultiByteToWideChar(CP_UTF8, 0, wantId.c_str(), -1, nullptr, 0);
		if (wn > 0) {
			std::wstring idW(wn, L'\0');
			MultiByteToWideChar(CP_UTF8, 0, wantId.c_str(), -1, idW.data(), wn);
			idW.resize(idW.size() - 1); // drop NUL
			hr = enumerator_->GetDevice(idW.c_str(), &device);
			if (FAILED(hr) || !device.Get()) {
				TH_LOG("[wasapi] selected device unavailable (id='%s'); falling back to system default",
				       wantId.c_str());
				device.Reset();
				fell_back_to_default = true;
			}
		}
	}

	if (!device.Get()) {
		hr = enumerator_->GetDefaultAudioEndpoint(eCapture, eConsole, &device);
		if (FAILED(hr)) return false;
	}

	// Capture friendly name for the status poller.
	{
		ComPtr<IPropertyStore> props;
		if (SUCCEEDED(device->OpenPropertyStore(STGM_READ, &props))) {
			PROPVARIANT pv;
			PropVariantInit(&pv);
			if (SUCCEEDED(props->GetValue(PKEY_Device_FriendlyName, &pv)) && pv.vt == VT_LPWSTR && pv.pwszVal) {
				int n = WideCharToMultiByte(CP_UTF8, 0, pv.pwszVal, -1, nullptr, 0, nullptr, nullptr);
				if (n > 0) {
					std::string s(n, '\0');
					WideCharToMultiByte(CP_UTF8, 0, pv.pwszVal, -1, s.data(), n, nullptr, nullptr);
					s.resize(s.size() - 1);
					std::lock_guard<std::mutex> lk(name_mutex_);
					device_name_ = std::move(s);
				}
			}
			PropVariantClear(&pv);
		}
	}
	TH_LOG("[wasapi] opened capture endpoint requested_id='%s' fallback=%d name='%s'",
	       wantId.empty() ? "(system default)" : wantId.c_str(), fell_back_to_default ? 1 : 0, DeviceName().c_str());

	hr = device->Activate(__uuidof(IAudioClient), CLSCTX_ALL, nullptr, reinterpret_cast<void**>(&audio_client_));
	return SUCCEEDED(hr);
}

void WasapiCapture::ReleaseCom()
{
	if (capture_client_) {
		capture_client_->Release();
		capture_client_ = nullptr;
	}
	if (audio_client_) {
		audio_client_->Release();
		audio_client_ = nullptr;
	}
	if (enumerator_) {
		enumerator_->Release();
		enumerator_ = nullptr;
	}
}

// ---------------------------------------------------------------------------
// Resampling: linear interpolation from in_rate to 16 kHz
// ---------------------------------------------------------------------------

void WasapiCapture::ResampleAndAccumulate(const float* data, size_t frames, uint32_t in_rate)
{
	const uint32_t out_rate = 16000;
	if (in_rate == out_rate) {
		// No resampling needed.
		for (size_t i = 0; i < frames; ++i) {
			accum_.push_back(data[i]);
			if (accum_.size() >= kFrameSamples) {
				if (callback_) callback_(accum_.data(), kFrameSamples);
				accum_.clear();
			}
		}
		return;
	}

	// Linear interpolation.
	const double ratio = static_cast<double>(in_rate) / static_cast<double>(out_rate);
	const size_t out_count = static_cast<size_t>(std::ceil(static_cast<double>(frames) / ratio));

	for (size_t oi = 0; oi < out_count; ++oi) {
		double src_pos = static_cast<double>(oi) * ratio;
		size_t lo = static_cast<size_t>(src_pos);
		size_t hi = lo + 1;
		float frac = static_cast<float>(src_pos - static_cast<double>(lo));
		float sample;
		if (hi < frames)
			sample = data[lo] * (1.f - frac) + data[hi] * frac;
		else
			sample = (lo < frames) ? data[lo] : 0.f;

		accum_.push_back(sample);
		if (accum_.size() >= kFrameSamples) {
			if (callback_) callback_(accum_.data(), kFrameSamples);
			accum_.clear();
		}
	}
}
