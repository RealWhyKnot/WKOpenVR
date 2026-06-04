#include "SidecarSupervisor.h"

#include "Logging.h"

#include <chrono>
#include <cstring>
#include <vector>

namespace phantom {

namespace {

constexpr DWORD kBackoffSeconds[] = {1, 2, 4, 8, 16, 32, 60};
constexpr size_t kCircuitBreakerAttempts = 5;
constexpr std::chrono::minutes kCircuitBreakerWindow{10};

// Free function so we can take its address (member function pointers cannot
// be reinterpret_cast to LPCWSTR for GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS).
std::wstring ResolveDriverDllPath()
{
	HMODULE hMod = nullptr;
	if (!GetModuleHandleExW(GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS | GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,
	                        reinterpret_cast<LPCWSTR>(&ResolveDriverDllPath), &hMod)) {
		return {};
	}
	wchar_t buf[MAX_PATH];
	DWORD len = GetModuleFileNameW(hMod, buf, MAX_PATH);
	if (len == 0 || len >= MAX_PATH) return {};
	return std::wstring(buf, len);
}

} // namespace

std::wstring SidecarSupervisor::ResolveSidecarPath() const
{
	std::wstring path = ResolveDriverDllPath();
	if (path.empty()) return {};
	// Walk-up THREE segments: filename, "win64", "bin" -> driver root.
	// Matches the corrected face-tracking + translator host walk-up
	// (walk-up-2 was the bug in earlier hosts that surfaced as
	// CreateProcessW err=3; see project_driver_host_path_walk memo).
	for (int i = 0; i < 3; ++i) {
		const size_t slash = path.find_last_of(L"\\/");
		if (slash == std::wstring::npos) return {};
		path.resize(slash);
	}
	path += L"\\resources\\phantom\\host\\WKOpenVRPhantomSidecar.exe";
	return path;
}

bool SidecarSupervisor::Start()
{
	if (running_.load(std::memory_order_acquire)) return true;
	stop_requested_.store(false, std::memory_order_release);
	halted_.store(false, std::memory_order_release);
	running_.store(true, std::memory_order_release);
	worker_ = std::thread(&SidecarSupervisor::SuperviseLoop, this);
	return true;
}

void SidecarSupervisor::Stop()
{
	stop_requested_.store(true, std::memory_order_release);
	if (child_process_) {
		TerminateProcess(child_process_, 0);
	}
	if (worker_.joinable()) worker_.join();
	if (child_process_) {
		CloseHandle(child_process_);
		child_process_ = nullptr;
	}
	running_.store(false, std::memory_order_release);
}

void SidecarSupervisor::SuperviseLoop()
{
	const std::wstring exe = ResolveSidecarPath();
	if (exe.empty()) {
		LOG("[phantom] sidecar: unable to resolve exe path; supervisor exiting");
		running_.store(false, std::memory_order_release);
		return;
	}

	std::vector<std::chrono::steady_clock::time_point> exit_history;
	size_t backoff_idx = 0;

	while (!stop_requested_.load(std::memory_order_acquire)) {
		STARTUPINFOW si{};
		si.cb = sizeof(si);
		PROCESS_INFORMATION pi{};
		// Empty command line; CreateProcessW needs a mutable buffer.
		std::wstring cmdline = L"\"" + exe + L"\"";
		std::vector<wchar_t> cmdbuf(cmdline.begin(), cmdline.end());
		cmdbuf.push_back(L'\0');
		const BOOL ok = CreateProcessW(exe.c_str(), cmdbuf.data(), nullptr, nullptr, FALSE, CREATE_NO_WINDOW, nullptr,
		                               nullptr, &si, &pi);
		if (!ok) {
			LOG("[phantom] sidecar: CreateProcessW err=%u; retrying after backoff", (unsigned)GetLastError());
		}
		else {
			LOG("[phantom] sidecar: spawned pid=%u", (unsigned)pi.dwProcessId);
			child_process_ = pi.hProcess;
			CloseHandle(pi.hThread);
			// Block until exit or stop.
			for (;;) {
				const DWORD wait = WaitForSingleObject(child_process_, 250);
				if (wait == WAIT_OBJECT_0) break;
				if (stop_requested_.load(std::memory_order_acquire)) {
					TerminateProcess(child_process_, 0);
					WaitForSingleObject(child_process_, 1000);
					break;
				}
			}
			DWORD exit_code = 0;
			GetExitCodeProcess(child_process_, &exit_code);
			CloseHandle(child_process_);
			child_process_ = nullptr;
			last_exit_code_.store(exit_code, std::memory_order_release);
			LOG("[phantom] sidecar: exit code=%u", (unsigned)exit_code);
		}

		if (stop_requested_.load(std::memory_order_acquire)) break;

		// Circuit breaker: 5 exits within 10 min -> halt.
		const auto now = std::chrono::steady_clock::now();
		exit_history.push_back(now);
		while (!exit_history.empty() && (now - exit_history.front()) > kCircuitBreakerWindow) {
			exit_history.erase(exit_history.begin());
		}
		if (exit_history.size() >= kCircuitBreakerAttempts) {
			LOG("[phantom] sidecar: circuit breaker tripped after %zu exits "
			    "within %lld min; halting supervisor",
			    exit_history.size(),
			    (long long)std::chrono::duration_cast<std::chrono::minutes>(kCircuitBreakerWindow).count());
			halted_.store(true, std::memory_order_release);
			break;
		}

		const DWORD secs =
		    kBackoffSeconds[backoff_idx < std::size(kBackoffSeconds) - 1 ? backoff_idx++
		                                                                 : std::size(kBackoffSeconds) - 1];
		for (DWORD elapsed = 0; elapsed < secs && !stop_requested_.load(std::memory_order_acquire); ++elapsed) {
			Sleep(1000);
		}
	}
	running_.store(false, std::memory_order_release);
}

} // namespace phantom
