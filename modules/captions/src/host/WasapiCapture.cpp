#define _CRT_SECURE_NO_DEPRECATE
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <mmdeviceapi.h>
#include <audioclient.h>
#include <functiondiscoverykeys_devpkey.h>
#include <Objbase.h>
#include <wrl/client.h>

#include "WasapiCapture.h"
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
		if (!OpenDefaultDevice()) {
			TH_LOG("[wasapi] failed to open default device; retry in %d ms", backoff_ms);
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
		bool isFloat = (mixFmt->wFormatTag == WAVE_FORMAT_IEEE_FLOAT) ||
		               (mixFmt->wFormatTag == WAVE_FORMAT_EXTENSIBLE &&
		                reinterpret_cast<WAVEFORMATEXTENSIBLE*>(mixFmt)->SubFormat == KSDATAFORMAT_SUBTYPE_IEEE_FLOAT);
		CoTaskMemFree(mixFmt);

		if (!isFloat && mixFmt->wBitsPerSample != 16) {
			TH_LOG("[wasapi] unsupported sample format; bits=%u", (unsigned)mixFmt->wBitsPerSample);
			ReleaseCom();
			continue;
		}

		hr = audio_client_->Start();
		if (FAILED(hr)) {
			ReleaseCom();
			continue;
		}

		TH_LOG("[wasapi] capture started at %u Hz %u ch", (unsigned)nSampleRate, (unsigned)nChannels);

		while (running_.load(std::memory_order_acquire)) {
			Sleep(10); // 10 ms polling

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

				if (!(flags & AUDCLNT_BUFFERFLAGS_SILENT) && frames > 0 && data) {
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
					ResampleAndAccumulate(mono_scratch_.data(), frames, nSampleRate);
				}

				capture_client_->ReleaseBuffer(frames);
				hr = capture_client_->GetNextPacketSize(&packetLen);
				if (FAILED(hr)) {
					packetLen = 0;
					break;
				}
			}
		}

		audio_client_->Stop();
		ReleaseCom();
	}

	CoUninitialize();
	TH_LOG("[wasapi] capture thread exiting");
}

bool WasapiCapture::OpenDefaultDevice()
{
	HRESULT hr = CoCreateInstance(__uuidof(MMDeviceEnumerator), nullptr, CLSCTX_ALL, IID_PPV_ARGS(&enumerator_));
	if (FAILED(hr)) return false;

	ComPtr<IMMDevice> device;
	hr = enumerator_->GetDefaultAudioEndpoint(eCapture, eConsole, &device);
	if (FAILED(hr)) return false;

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
