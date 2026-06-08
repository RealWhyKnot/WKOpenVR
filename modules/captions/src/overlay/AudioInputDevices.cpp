#include "AudioInputDevices.h"

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>

#include <mmdeviceapi.h>
#include <wrl/client.h>

using Microsoft::WRL::ComPtr;

namespace captions {

namespace {

// PKEY_Device_FriendlyName, defined locally so this TU does not include
// <functiondiscoverykeys_devpkey.h>: the overlay's link unit defines INITGUID,
// which makes that header allocate property-key storage and collide. Value from
// the Windows SDK: fmtid {a45c254e-df1c-4efd-8020-67d146a850e0}, pid 14.
const PROPERTYKEY kPkeyDeviceFriendlyName = {
    {0xa45c254e, 0xdf1c, 0x4efd, {0x80, 0x20, 0x67, 0xd1, 0x46, 0xa8, 0x50, 0xe0}}, 14};

std::string WideToUtf8(const wchar_t* w)
{
	if (!w) return {};
	int n = WideCharToMultiByte(CP_UTF8, 0, w, -1, nullptr, 0, nullptr, nullptr);
	if (n <= 0) return {};
	std::string s(n, '\0');
	WideCharToMultiByte(CP_UTF8, 0, w, -1, s.data(), n, nullptr, nullptr);
	s.resize(s.size() - 1); // drop NUL
	return s;
}

} // namespace

std::vector<AudioInputDevice> EnumerateCaptureDevices()
{
	std::vector<AudioInputDevice> devices;

	// The overlay's render thread already has a COM apartment; CoInitializeEx
	// returns S_FALSE if so (still a success we must balance) or
	// RPC_E_CHANGED_MODE if it is in a different mode (a failure we must NOT
	// balance, but the enumerator still works regardless of apartment).
	HRESULT hrInit = CoInitializeEx(nullptr, COINIT_APARTMENTTHREADED);
	const bool balanceInit = SUCCEEDED(hrInit);

	ComPtr<IMMDeviceEnumerator> enumerator;
	HRESULT hr = CoCreateInstance(__uuidof(MMDeviceEnumerator), nullptr, CLSCTX_ALL, IID_PPV_ARGS(&enumerator));
	if (FAILED(hr)) {
		if (balanceInit) CoUninitialize();
		return devices;
	}

	ComPtr<IMMDeviceCollection> collection;
	hr = enumerator->EnumAudioEndpoints(eCapture, DEVICE_STATE_ACTIVE, &collection);
	if (FAILED(hr)) {
		if (balanceInit) CoUninitialize();
		return devices;
	}

	UINT count = 0;
	collection->GetCount(&count);
	for (UINT i = 0; i < count; ++i) {
		ComPtr<IMMDevice> device;
		if (FAILED(collection->Item(i, &device))) continue;

		AudioInputDevice entry;

		LPWSTR idW = nullptr;
		if (SUCCEEDED(device->GetId(&idW)) && idW) {
			entry.id = WideToUtf8(idW);
			CoTaskMemFree(idW);
		}
		if (entry.id.empty()) continue; // no stable id -> not selectable

		ComPtr<IPropertyStore> props;
		if (SUCCEEDED(device->OpenPropertyStore(STGM_READ, &props))) {
			PROPVARIANT pv;
			PropVariantInit(&pv);
			if (SUCCEEDED(props->GetValue(kPkeyDeviceFriendlyName, &pv)) && pv.vt == VT_LPWSTR) {
				entry.name = WideToUtf8(pv.pwszVal);
			}
			PropVariantClear(&pv);
		}
		if (entry.name.empty()) entry.name = entry.id;

		devices.push_back(std::move(entry));
	}

	if (balanceInit) CoUninitialize();
	return devices;
}

} // namespace captions
