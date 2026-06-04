#include "InProcessDriverLoader.h"

#if WKOPENVR_BUILD_IS_DEV

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>

#include "MockPoseSource.h" // for MockOpenVRRuntime

#include <chrono>
#include <stdexcept>
#include <string>

namespace openvr_pair::overlay::testharness {

namespace {

using HmdDriverFactoryFn = void* (*)(const char*, int*);

std::wstring Widen(const std::string& s)
{
	if (s.empty()) return {};
	const int wlen = ::MultiByteToWideChar(CP_UTF8, 0, s.c_str(), (int)s.size(), nullptr, 0);
	std::wstring w(wlen, L'\0');
	::MultiByteToWideChar(CP_UTF8, 0, s.c_str(), (int)s.size(), w.data(), wlen);
	return w;
}

std::string DescribeWin32Error(DWORD err)
{
	LPSTR buf = nullptr;
	const DWORD n = ::FormatMessageA(FORMAT_MESSAGE_ALLOCATE_BUFFER | FORMAT_MESSAGE_FROM_SYSTEM, nullptr, err,
	                                 MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT), (LPSTR)&buf, 0, nullptr);
	std::string s;
	if (n != 0 && buf) {
		s.assign(buf, n);
		while (!s.empty() && (s.back() == '\r' || s.back() == '\n' || s.back() == ' '))
			s.pop_back();
		::LocalFree(buf);
	}
	else {
		s = "err=" + std::to_string(err);
	}
	return s;
}

} // namespace

InProcessDriverLoader::~InProcessDriverLoader()
{
	Stop();
}

void InProcessDriverLoader::Load(const std::filesystem::path& driver_dll, MockOpenVRRuntime& runtime)
{
	if (hModule_ != nullptr) {
		throw std::runtime_error("InProcessDriverLoader::Load called twice");
	}

	// SetDllDirectory ensures dependent DLLs (MinHook etc.) resolve from the
	// staged bin\win64 directory.
	const std::wstring binDir = driver_dll.parent_path().wstring();
	::SetDllDirectoryW(binDir.c_str());

	HMODULE h = ::LoadLibraryExW(driver_dll.wstring().c_str(), nullptr, LOAD_WITH_ALTERED_SEARCH_PATH);
	if (!h) {
		const DWORD err = ::GetLastError();
		throw std::runtime_error("LoadLibraryExW failed for " + driver_dll.string() + ": " + DescribeWin32Error(err));
	}
	hModule_ = h;

	auto factory = (HmdDriverFactoryFn)::GetProcAddress(h, "HmdDriverFactory");
	if (!factory) {
		const DWORD err = ::GetLastError();
		::FreeLibrary(h);
		hModule_ = nullptr;
		throw std::runtime_error("GetProcAddress(HmdDriverFactory) failed: " + DescribeWin32Error(err));
	}

	int rc = 0;
	void* iface = factory(vr::IServerTrackedDeviceProvider_Version, &rc);
	if (!iface || rc != 0) {
		::FreeLibrary(h);
		hModule_ = nullptr;
		throw std::runtime_error("HmdDriverFactory(IServerTrackedDeviceProvider_004) returned rc=" +
		                         std::to_string(rc));
	}
	provider_ = (vr::IServerTrackedDeviceProvider*)iface;

	const vr::EVRInitError initErr = provider_->Init(&runtime.context());
	if (initErr != vr::VRInitError_None) {
		// Provider keeps an internal pointer to our mock context. Calling
		// Cleanup() before FreeLibrary keeps internal state from leaking.
		provider_->Cleanup();
		provider_ = nullptr;
		::FreeLibrary(h);
		hModule_ = nullptr;
		throw std::runtime_error("ServerTrackedDeviceProvider::Init returned VRInitError=" +
		                         std::to_string((int)initErr));
	}
}

void InProcessDriverLoader::StartFrameTicker()
{
	if (started_ticker_ || !provider_) return;
	started_ticker_ = true;
	stop_requested_.store(false, std::memory_order_relaxed);
	ticker_ = std::thread(&InProcessDriverLoader::FrameLoop, this);
}

void InProcessDriverLoader::Stop()
{
	if (started_ticker_) {
		stop_requested_.store(true, std::memory_order_relaxed);
		if (ticker_.joinable()) ticker_.join();
		started_ticker_ = false;
	}

	if (provider_ && !cleaned_up_) {
		provider_->Cleanup();
		cleaned_up_ = true;
		provider_ = nullptr;
	}

	if (hModule_) {
		::FreeLibrary((HMODULE)hModule_);
		hModule_ = nullptr;
	}
}

void InProcessDriverLoader::FrameLoop()
{
	using namespace std::chrono;
	const auto frame_duration = milliseconds(11); // ~90 Hz
	while (!stop_requested_.load(std::memory_order_relaxed)) {
		if (provider_) provider_->RunFrame();
		std::this_thread::sleep_for(frame_duration);
	}
}

} // namespace openvr_pair::overlay::testharness

#endif // WKOPENVR_BUILD_IS_DEV
