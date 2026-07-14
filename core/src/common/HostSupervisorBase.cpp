#define _CRT_SECURE_NO_DEPRECATE
#include "HostSupervisorBase.h"

#include "BuildChannel.h"
#include "DiagnosticsLog.h"
#include "ModulePerf.h"
#include "Win32CommandLine.h"

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#include <tlhelp32.h>

#include <algorithm>
#include <chrono>
#include <cstdarg>
#include <cstdio>
#include <cwchar>
#include <cstring>
#include <exception>
#include <utility>

namespace {

// Lowercase ASCII compare for image-name match. Win32 process names from
// Toolhelp32 come back in the original case (typically PascalCase exe
// names) -- the supervisor's host_exe_path_ is whatever the resolver
// returned. Compare case-insensitively over the last path component only.
std::string ExtractImageName(const std::string& full_path)
{
	size_t slash = full_path.find_last_of("\\/");
	std::string name = (slash == std::string::npos) ? full_path : full_path.substr(slash + 1);
	for (char& c : name)
		if (c >= 'A' && c <= 'Z') c = static_cast<char>(c - 'A' + 'a');
	return name;
}

std::string LowerAscii(const wchar_t* w)
{
	std::string out;
	for (; *w; ++w) {
		wchar_t c = *w;
		if (c < 128) {
			char ch = static_cast<char>(c);
			if (ch >= 'A' && ch <= 'Z') ch = static_cast<char>(ch - 'A' + 'a');
			out.push_back(ch);
		}
	}
	return out;
}

std::wstring WidenUtf8(const std::string& value)
{
	if (value.empty()) return {};
	int needed = MultiByteToWideChar(CP_UTF8, 0, value.c_str(), static_cast<int>(value.size()), nullptr, 0);
	if (needed <= 0) return {};
	std::wstring out(static_cast<size_t>(needed), L'\0');
	MultiByteToWideChar(CP_UTF8, 0, value.c_str(), static_cast<int>(value.size()), out.data(), needed);
	return out;
}

} // namespace

namespace openvr_pair::common {

HostSupervisorBase::HostSupervisorBase(std::string host_exe_path) : host_exe_path_(std::move(host_exe_path))
{
	job_handle_ = CreateJobObjectW(nullptr, nullptr);
	if (job_handle_) {
		JOBOBJECT_EXTENDED_LIMIT_INFORMATION info{};
		info.BasicLimitInformation.LimitFlags = JOB_OBJECT_LIMIT_KILL_ON_JOB_CLOSE;
		if (!SetInformationJobObject(job_handle_, JobObjectExtendedLimitInformation, &info, sizeof(info))) {
			Log("[host-supervisor] SetInformationJobObject failed err=%lu; "
			    "host will not be auto-killed on driver crash",
			    GetLastError());
			CloseHandle(job_handle_);
			job_handle_ = nullptr;
		}
	}
	else {
		Log("[host-supervisor] CreateJobObjectW failed err=%lu; "
		    "host will not be auto-killed on driver crash",
		    GetLastError());
	}
}

HostSupervisorBase::~HostSupervisorBase()
{
	Stop();
	if (job_handle_) {
		CloseHandle(job_handle_);
		job_handle_ = nullptr;
	}
}

bool HostSupervisorBase::Start()
{
	if (host_exe_path_.empty()) {
		Log("[host] host exe path is empty -- not starting");
		return false;
	}
	{
		std::lock_guard<std::mutex> lk(process_mutex_);
		stop_requested_.store(false, std::memory_order_release);
		halted_ = false;
		consecutive_fast_exits_ = 0;
		attached_to_existing_ = false;
	}
	EnsureOwnerLease();
	HeartbeatOwnerLease();
	bool initial_spawned = Spawn();
	if (!initial_spawned) {
		Log("[host-supervisor] initial spawn failed; monitor will retry");
	}
	// MonitorLoop runs as the thread body; an exception escaping a std::thread
	// entry calls std::terminate and would take vrserver down with it. Contain
	// it at the boundary -- the supervisor stops respawning on an unexpected
	// throw, but the host process (driven by the kill-on-close job object) and
	// vrserver stay alive.
	monitor_thread_ = std::thread([this] {
		try {
			MonitorLoop();
		}
		catch (const std::exception& ex) {
			Log("[host-supervisor] monitor thread terminated by exception: %s", ex.what());
		}
		catch (...) {
			Log("[host-supervisor] monitor thread terminated by an unknown exception");
		}
	});
	return initial_spawned;
}

void HostSupervisorBase::Stop()
{
	stop_requested_.store(true, std::memory_order_release);
	MarkOwnerLeaseShuttingDown();
	RequestGracefulShutdown();
	Kill();
	MarkOwnerLeaseDisabled();
	if (monitor_thread_.joinable()) monitor_thread_.join();
	std::lock_guard<std::mutex> lk(process_mutex_);
	halted_ = false;
	consecutive_fast_exits_ = 0;
}

void HostSupervisorBase::Restart()
{
	Log("[host] Restart() requested");
	{
		std::lock_guard<std::mutex> lk(process_mutex_);
		halted_ = false;
		consecutive_fast_exits_ = 0;
	}
	OnHostExited();
	MarkOwnerLeaseShuttingDown();
	RequestGracefulShutdown();
	Kill();
	if (!stop_requested_.load(std::memory_order_acquire)) {
		HeartbeatOwnerLease();
		Spawn();
	}
}

bool HostSupervisorBase::IsRunning() const
{
	return running_.load(std::memory_order_acquire);
}

bool HostSupervisorBase::IsHalted() const
{
	std::lock_guard<std::mutex> lk(process_mutex_);
	return halted_;
}

uint32_t HostSupervisorBase::LastExitCode() const
{
	std::lock_guard<std::mutex> lk(process_mutex_);
	return last_exit_code_;
}

std::string HostSupervisorBase::LastExitDescription() const
{
	std::lock_guard<std::mutex> lk(process_mutex_);
	return last_exit_description_;
}

void HostSupervisorBase::Log(const char* fmt, ...)
{
	va_list args;
	va_start(args, fmt);
	va_list diagArgs;
	va_copy(diagArgs, args);
	DiagnosticLogV("host-supervisor", fmt, diagArgs);
	va_end(diagArgs);
	LogV(fmt, args);
	va_end(args);
}

void HostSupervisorBase::BuildCommandLine(std::wstring& /*commandLine*/, const std::wstring& /*exe_path*/) const
{
	// Default: nothing appended after argv[0].
}

bool HostSupervisorBase::EnsureOwnerLease()
{
	const auto ownerModule = SidecarOwnerModuleId();
	if (!ownerModule) return false;

	std::lock_guard<std::mutex> lk(owner_lease_mutex_);
	if (!owner_lease_) {
		owner_lease_ = std::make_unique<sidecar_owner::LeaseOwner>();
	}
	if (owner_lease_->IsOpen()) return true;
	if (!owner_lease_->Create(*ownerModule)) {
		Log("[host-supervisor] failed to create sidecar owner lease for module=%s", modules::Slug(*ownerModule));
		return false;
	}
	Log("[host-supervisor] sidecar owner lease created module=%s name=%s nonce=0x%016llX", modules::Slug(*ownerModule),
	    owner_lease_->Name().c_str(), static_cast<unsigned long long>(owner_lease_->Nonce()));
	return true;
}

void HostSupervisorBase::HeartbeatOwnerLease(sidecar_owner::LeaseState state)
{
	if (state == sidecar_owner::LeaseState::Alive && stop_requested_.load(std::memory_order_acquire)) {
		return;
	}
	std::lock_guard<std::mutex> lk(owner_lease_mutex_);
	if (owner_lease_ && owner_lease_->IsOpen()) {
		owner_lease_->Heartbeat(state);
	}
}

void HostSupervisorBase::MarkOwnerLeaseShuttingDown()
{
	std::lock_guard<std::mutex> lk(owner_lease_mutex_);
	if (owner_lease_ && owner_lease_->IsOpen()) {
		owner_lease_->MarkShuttingDown();
	}
}

void HostSupervisorBase::MarkOwnerLeaseDisabled()
{
	std::lock_guard<std::mutex> lk(owner_lease_mutex_);
	if (owner_lease_ && owner_lease_->IsOpen()) {
		owner_lease_->MarkDisabled();
	}
}

void HostSupervisorBase::AppendOwnerLivenessArgs(std::wstring& commandLine) const
{
	std::lock_guard<std::mutex> lk(owner_lease_mutex_);
	if (!owner_lease_ || !owner_lease_->IsOpen()) return;

	wchar_t nonce[32] = {};
	std::swprintf(nonce, sizeof(nonce) / sizeof(nonce[0]), L"%016llX",
	              static_cast<unsigned long long>(owner_lease_->Nonce()));
	commandLine += L" --owner-liveness ";
	commandLine += QuoteCommandLineArg(WidenUtf8(owner_lease_->Name()));
	commandLine += L" --owner-liveness-nonce ";
	commandLine += nonce;
}

bool HostSupervisorBase::CanConnectToHost(int timeout_ms) const
{
	const std::string pipe = ControlPipeName();
	if (!WaitNamedPipeA(pipe.c_str(), static_cast<DWORD>(timeout_ms))) return false;
	HANDLE h = CreateFileA(pipe.c_str(), GENERIC_WRITE, 0, nullptr, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, nullptr);
	if (h == INVALID_HANDLE_VALUE) return false;
	CloseHandle(h);
	return true;
}

bool HostSupervisorBase::IsSingletonMutexHeld() const
{
	std::wstring name = SingletonMutexName();
	if (name.empty()) return false;
	HANDLE h = OpenMutexW(SYNCHRONIZE, FALSE, name.c_str());
	if (!h) return false;
	CloseHandle(h);
	return true;
}

int HostSupervisorBase::CleanupStaleHostIfWedged()
{
	// Only trigger cleanup when the host appears to be running but
	// unresponsive: singleton mutex is held AND the control pipe does not
	// answer within 200 ms. If either condition fails, there is nothing
	// wedged to clean up.
	if (SingletonMutexName().empty()) return 0;
	if (!IsSingletonMutexHeld()) return 0;
	if (CanConnectToHost(200)) return 0;

	const std::string image_lc = ExtractImageName(host_exe_path_);
	if (image_lc.empty()) return 0;

	HANDLE snap = CreateToolhelp32Snapshot(TH32CS_SNAPPROCESS, 0);
	if (snap == INVALID_HANDLE_VALUE) {
		Log("[host-supervisor] CleanupStaleHostIfWedged: CreateToolhelp32Snapshot failed err=%lu", GetLastError());
		return 0;
	}

	PROCESSENTRY32W entry{};
	entry.dwSize = sizeof(entry);

	int killed = 0;
	if (Process32FirstW(snap, &entry)) {
		do {
			const std::string proc_lc = LowerAscii(entry.szExeFile);
			if (proc_lc != image_lc) continue;

			// Don't terminate our own driver host process (vrserver,
			// technically possible but the image name will not match).
			if (entry.th32ProcessID == GetCurrentProcessId()) continue;

			HANDLE proc = OpenProcess(PROCESS_TERMINATE | SYNCHRONIZE, FALSE, entry.th32ProcessID);
			if (!proc) {
				Log("[host-supervisor] CleanupStaleHostIfWedged: OpenProcess pid=%lu failed err=%lu",
				    entry.th32ProcessID, GetLastError());
				continue;
			}

			if (TerminateProcess(proc, 1)) {
				WaitForSingleObject(proc, 1000);
				++killed;
				Log("[host-supervisor] CleanupStaleHostIfWedged: terminated stale pid=%lu image=%s",
				    entry.th32ProcessID, image_lc.c_str());
			}
			else {
				Log("[host-supervisor] CleanupStaleHostIfWedged: TerminateProcess pid=%lu failed err=%lu",
				    entry.th32ProcessID, GetLastError());
			}
			CloseHandle(proc);
		} while (Process32NextW(snap, &entry));
	}
	CloseHandle(snap);

	if (killed > 0) {
		Log("[host-supervisor] CleanupStaleHostIfWedged: cleaned up %d stale host(s) before spawn", killed);
	}
	return killed;
}

bool HostSupervisorBase::SendBytesOverControlPipe(const void* data, size_t len)
{
	const std::string pipe = ControlPipeName();
	HANDLE h = CreateFileA(pipe.c_str(), GENERIC_WRITE, 0, nullptr, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, nullptr);
	if (h == INVALID_HANDLE_VALUE) return false;

	DWORD written = 0;
	BOOL ok = WriteFile(h, data, static_cast<DWORD>(len), &written, nullptr);
	CloseHandle(h);

	return ok && written == static_cast<DWORD>(len);
}

uint64_t HostSupervisorBase::QueryExeWriteTime() const
{
	std::wstring w = WidenUtf8(host_exe_path_);
	if (w.empty()) return 0;
	WIN32_FILE_ATTRIBUTE_DATA a{};
	if (!GetFileAttributesExW(w.c_str(), GetFileExInfoStandard, &a)) return 0;
	return (static_cast<uint64_t>(a.ftLastWriteTime.dwHighDateTime) << 32) | a.ftLastWriteTime.dwLowDateTime;
}

bool HostSupervisorBase::Spawn()
{
	bool already_running = false;
	bool attached = false;
	bool spawned = false;
	bool rejected_unleased_existing = false;

	{
		std::lock_guard<std::mutex> lk(process_mutex_);

		if (process_handle_ != INVALID_HANDLE_VALUE && WaitForSingleObject(process_handle_, 0) == WAIT_TIMEOUT) {
			Log("[host-supervisor] host already tracked; skipping spawn");
			already_running = true;
		}
		else if (CanConnectToHost(200) && (SingletonMutexName().empty() || IsSingletonMutexHeld())) {
			// Attach-to-existing requires BOTH a responsive pipe AND, when
			// the host advertises a singleton mutex, that the mutex is
			// held. The combined check rejects a half-spawned host that
			// owns the mutex but hasn't bound the pipe yet (200 ms window),
			// and also rejects a stale pipe whose owning process died
			// without releasing its named-pipe instance.
			if (SidecarOwnerModuleId()) {
				Log("[host-supervisor] existing host responsive but not owned by this supervisor; requesting shutdown");
				rejected_unleased_existing = true;
			}
			else {
				Log("[host-supervisor] existing host responsive on pipe; "
				    "attaching without spawn");
				attached_to_existing_ = true;
				running_.store(true, std::memory_order_release);
				attached = true;
			}
		}
		else {
			int spawn_attempt = consecutive_fast_exits_;
			Log("[host-supervisor] spawn attempt #%d (consecutive_fast_exits=%d)", spawn_attempt + 1, spawn_attempt);

			int wlen = MultiByteToWideChar(CP_UTF8, 0, host_exe_path_.c_str(), -1, nullptr, 0);
			if (wlen > 0) {
				std::wstring wpath(wlen, L'\0');
				MultiByteToWideChar(CP_UTF8, 0, host_exe_path_.c_str(), -1, wpath.data(), wlen);
				if (!wpath.empty() && wpath.back() == L'\0') wpath.pop_back();

				std::wstring commandLine = L"\"" + wpath + L"\"";
				AppendOwnerLivenessArgs(commandLine);
				BuildCommandLine(commandLine, wpath);

				STARTUPINFOW si{};
				si.cb = sizeof(si);
				PROCESS_INFORMATION pi{};

				if (!CreateProcessW(wpath.c_str(), commandLine.data(), nullptr, nullptr, FALSE, CREATE_NO_WINDOW,
				                    nullptr, nullptr, &si, &pi)) {
					DWORD cpErr = GetLastError();
					char errMsg[256] = {};
					FormatMessageA(FORMAT_MESSAGE_FROM_SYSTEM | FORMAT_MESSAGE_IGNORE_INSERTS, nullptr, cpErr, 0,
					               errMsg, sizeof(errMsg) - 1, nullptr);
					size_t mlen = strlen(errMsg);
					while (mlen > 0 && (errMsg[mlen - 1] == '\r' || errMsg[mlen - 1] == '\n'))
						errMsg[--mlen] = '\0';
					Log("[host-supervisor] CreateProcessW FAILED: "
					    "err=%lu msg=%s path='%s'",
					    cpErr, errMsg, host_exe_path_.c_str());
				}
				else {
					CloseHandle(pi.hThread);
					process_handle_ = pi.hProcess;
					++spawn_generation_;
					attached_to_existing_ = false;
					running_.store(true, std::memory_order_release);

					if (const auto perfId = PerfModuleId()) {
						moduleperf::Registry::Instance().RegisterChildProcess(*perfId, pi.hProcess, "feature-host");
					}

					if (job_handle_) {
						if (!AssignProcessToJobObject(job_handle_, pi.hProcess)) {
							Log("[host-supervisor] AssignProcessToJobObject failed "
							    "err=%lu pid=%lu; host can outlive an abnormal driver exit",
							    GetLastError(), pi.dwProcessId);
						}
					}

					Log("[host-supervisor] CreateProcessW OK: pid=%lu path='%s'", pi.dwProcessId,
					    host_exe_path_.c_str());
					// Baseline for the dev hot-reload watch: the exe bytes this
					// process was launched from. A later on-disk change relative
					// to this value triggers a kill+respawn (dev builds only).
					watched_exe_write_ = QueryExeWriteTime();
					spawned = true;
				}
			}
		}
	} // process_mutex_ released

	if (already_running) return true;
	if (rejected_unleased_existing) {
		RequestGracefulShutdown();
		return false;
	}
	if (!attached && !spawned) return false;

	// Subclass flushes any queued control-pipe message OUTSIDE the process
	// lock so a slow pipe write does not block the supervisor.
	OnHostReady();
	return true;
}

void HostSupervisorBase::Kill()
{
	std::lock_guard<std::mutex> lk(process_mutex_);
	if (process_handle_ == INVALID_HANDLE_VALUE) return;

	if (!attached_to_existing_) {
		if (!TerminateProcess(process_handle_, 0)) {
			const DWORD err = GetLastError();
			// Common case: process already exited between caller and here.
			// Only log when it is something else worth investigating.
			if (err != ERROR_ACCESS_DENIED && err != ERROR_INVALID_HANDLE) {
				Log("[host-supervisor] Kill: TerminateProcess failed err=%lu", err);
			}
		}
		DWORD wait_result = WaitForSingleObject(process_handle_, 5000);
		if (wait_result == WAIT_TIMEOUT) {
			Log("[host-supervisor] Kill: host did not exit within 5s of "
			    "TerminateProcess; retrying");
			TerminateProcess(process_handle_, 1);
			WaitForSingleObject(process_handle_, 1000);
		}
		else if (wait_result == WAIT_FAILED) {
			Log("[host-supervisor] Kill: WaitForSingleObject failed err=%lu", GetLastError());
		}
	}
	if (const auto perfId = PerfModuleId()) {
		moduleperf::Registry::Instance().UnregisterChildProcess(*perfId, GetProcessId(process_handle_));
	}
	CloseHandle(process_handle_);
	process_handle_ = INVALID_HANDLE_VALUE;
	++spawn_generation_;
	attached_to_existing_ = false;
	running_.store(false, std::memory_order_release);
}

void HostSupervisorBase::MonitorLoop()
{
	std::optional<moduleperf::ScopedThreadRegistration> perfRegistration;
	if (const auto perfId = PerfModuleId()) {
		perfRegistration.emplace(*perfId, "host-supervisor");
	}

	int backoff_ms = kBackoffStartMs;
	auto sleep_or_stop = [this](int delay_ms) {
		int remaining_ms = delay_ms;
		while (remaining_ms > 0 && !stop_requested_.load(std::memory_order_acquire)) {
			HeartbeatOwnerLease();
			int chunk_ms = std::min(remaining_ms, 100);
			std::this_thread::sleep_for(std::chrono::milliseconds(chunk_ms));
			remaining_ms -= chunk_ms;
		}
		return stop_requested_.load(std::memory_order_acquire);
	};

	// Time when the currently tracked spawn generation was first observed --
	// the uptime baseline for fast-exit classification. A per-iteration
	// timestamp would misclassify every long-lived host's eventual exit as
	// fast, because the wait below is re-armed each tick.
	uint64_t tracked_generation = ~0ull;
	auto tracked_since = std::chrono::steady_clock::now();

	while (!stop_requested_.load(std::memory_order_acquire)) {
		HeartbeatOwnerLease();
		HANDLE cur_handle = INVALID_HANDLE_VALUE;
		uint64_t cur_generation = 0;
		bool is_halted = false;
		{
			std::lock_guard<std::mutex> lk(process_mutex_);
			cur_handle = process_handle_;
			cur_generation = spawn_generation_;
			is_halted = halted_;
		}
		if (cur_generation != tracked_generation) {
			tracked_generation = cur_generation;
			tracked_since = std::chrono::steady_clock::now();
		}

		if (is_halted) {
#if WKOPENVR_BUILD_IS_DEV
			// Dev-only: a crash-looped halt normally needs a SteamVR restart to
			// clear, but if the exe on disk changed a fixed build was deployed;
			// let it take over (Restart() resets the halt and fast-exit count).
			uint64_t halted_baseline = 0;
			bool halted_attached = false;
			{
				std::lock_guard<std::mutex> lk(process_mutex_);
				halted_baseline = watched_exe_write_;
				halted_attached = attached_to_existing_;
			}
			if (ShouldUnhaltForNewHostExe(halted_attached, halted_baseline, QueryExeWriteTime())) {
				Log("[host-supervisor] host exe changed on disk while halted (dev hot-reload); restarting");
				Restart();
				backoff_ms = kBackoffStartMs;
				continue;
			}
#endif
			if (sleep_or_stop(1000)) break;
			continue;
		}

		if (cur_handle == INVALID_HANDLE_VALUE) {
			bool is_attached = false;
			{
				std::lock_guard<std::mutex> lk(process_mutex_);
				is_attached = attached_to_existing_;
			}
			if (is_attached) {
				if (!CanConnectToHost(0)) {
					Log("[host-supervisor] attached host pipe gone; "
					    "triggering respawn");
					running_.store(false, std::memory_order_release);
					{
						std::lock_guard<std::mutex> lk(process_mutex_);
						attached_to_existing_ = false;
					}
					OnHostExited();
					if (sleep_or_stop(backoff_ms)) break;
					if (!stop_requested_.load(std::memory_order_acquire)) {
						if (Spawn())
							backoff_ms = kBackoffStartMs;
						else
							backoff_ms = std::min(backoff_ms * 2, kBackoffMaxMs);
					}
				}
				else {
					OnHostReady();
					if (sleep_or_stop(500)) break;
				}
			}
			else {
				Log("[host-supervisor] no host process tracked; retrying spawn in %d ms", backoff_ms);
				if (sleep_or_stop(backoff_ms)) break;
				if (!stop_requested_.load(std::memory_order_acquire)) {
					if (Spawn())
						backoff_ms = kBackoffStartMs;
					else
						backoff_ms = std::min(backoff_ms * 2, kBackoffMaxMs);
				}
			}
			continue;
		}

		OnHostReady();

		DWORD wait = WaitForSingleObject(cur_handle, static_cast<DWORD>(backoff_ms > 1000 ? backoff_ms : 1000));

		if (stop_requested_.load(std::memory_order_acquire)) break;

		if (wait == WAIT_OBJECT_0) {
			DWORD code = 0;
			bool generation_matches = false;
			bool handle_was_valid = false;
			long long uptime_ms = 0;
			HostExitAction action;
			{
				std::lock_guard<std::mutex> lk(process_mutex_);
				generation_matches = spawn_generation_ == cur_generation;
				if (generation_matches && process_handle_ != INVALID_HANDLE_VALUE) {
					GetExitCodeProcess(process_handle_, &code);
					if (const auto perfId = PerfModuleId()) {
						moduleperf::Registry::Instance().UnregisterChildProcess(*perfId, GetProcessId(process_handle_));
					}
					CloseHandle(process_handle_);
					process_handle_ = INVALID_HANDLE_VALUE;
					++spawn_generation_;
					handle_was_valid = true;
					uptime_ms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() -
					                                                                  tracked_since)
					                .count();
					// Counter and halt update in the same critical section as
					// the reap so a concurrent Restart() cannot interleave a
					// reset between them.
					action = DecideHostExitAction(true, code, uptime_ms, consecutive_fast_exits_, backoff_ms);
					consecutive_fast_exits_ = action.consecutiveFastExits;
					if (action.halt) halted_ = true;
				}
			}
			if (!generation_matches) {
				// Restart()/Kill() already reaped the process this wait fired
				// for, and process_handle_ may now track a live replacement
				// host. Reaping that one would misread it as exited
				// (GetExitCodeProcess reports STILL_ACTIVE), abandon it
				// mid-startup, and then respawn in a loop against the
				// singleton mutex it still holds.
				Log("[host-supervisor] tracked host changed during wait; ignoring stale exit");
				continue;
			}
			if (!handle_was_valid) {
				// Defensive: the handle cannot go invalid without a generation
				// bump, so this should be unreachable. Reset state and loop.
				running_.store(false, std::memory_order_release);
				continue;
			}
			running_.store(false, std::memory_order_release);
			OnHostExited();

			std::string description = DescribeExitCode(code);
			if (description.empty()) {
				Log("[host-supervisor] host process exited "
				    "code=0x%08lx uptime_ms=%lld",
				    static_cast<unsigned long>(code), uptime_ms);
			}
			else {
				Log("[host-supervisor] host process exited "
				    "code=0x%08lx uptime_ms=%lld -- %s",
				    static_cast<unsigned long>(code), uptime_ms, description.c_str());
			}
			{
				std::lock_guard<std::mutex> lk(process_mutex_);
				last_exit_code_ = code;
				last_exit_description_ = description;
			}

			if (IsCleanSingletonExit(code)) {
				Log("[host-supervisor] clean singleton exit (code=%lu); "
				    "skipping fast-exit counter",
				    static_cast<unsigned long>(code));
			}
			else if (action.consecutiveFastExits > 0) {
				Log("[host-supervisor] fast exit count: %d/%d", action.consecutiveFastExits, kCircuitBreakerThreshold);
			}

			if (action.halt) {
				if (description.empty()) {
					Log("[host-supervisor] CIRCUIT BREAKER: %d consecutive fast "
					    "exits, halting respawn. Last exit code: 0x%08lx",
					    kCircuitBreakerThreshold, static_cast<unsigned long>(code));
				}
				else {
					Log("[host-supervisor] CIRCUIT BREAKER: %d consecutive fast "
					    "exits, halting respawn. Last exit code 0x%08lx -- %s",
					    kCircuitBreakerThreshold, static_cast<unsigned long>(code), description.c_str());
				}
				continue;
			}

			if (action.respawnDelayMs > 0) {
				Log("[host-supervisor] process exited (code=%lu); "
				    "restarting in %d ms",
				    static_cast<unsigned long>(code), action.respawnDelayMs);
				if (sleep_or_stop(action.respawnDelayMs)) break;
			}

			if (stop_requested_.load(std::memory_order_acquire)) break;

			if (Spawn()) {
				backoff_ms = kBackoffStartMs;
			}
			else {
				backoff_ms = std::min(backoff_ms * 2, kBackoffMaxMs);
			}
		}
#if WKOPENVR_BUILD_IS_DEV
		else if (wait == WAIT_TIMEOUT) {
			// Dev-only hot-reload: if the host exe on disk changed since we
			// launched it (a rebuild+redeploy while SteamVR keeps running),
			// kill and respawn so the new binary takes over. The redeploy must
			// rename the running exe aside before writing the new one (Windows
			// won't overwrite a running image); the fresh spawn re-resolves the
			// canonical path and the host reconnects its shmem rings and pipe.
			uint64_t baseline = 0;
			bool attached = false;
			bool haveHandle = false;
			{
				std::lock_guard<std::mutex> lk(process_mutex_);
				baseline = watched_exe_write_;
				attached = attached_to_existing_;
				haveHandle = process_handle_ != INVALID_HANDLE_VALUE;
			}
			if (ShouldHotReloadHost(attached, haveHandle, baseline, QueryExeWriteTime())) {
				Log("[host-supervisor] host exe changed on disk (dev hot-reload); restarting");
				Restart();
				backoff_ms = kBackoffStartMs;
			}
		}
#endif
		// else WAIT_TIMEOUT: process still alive, loop again and let
		// OnHostReady() retry any queued message.
	}

	Log("[host-supervisor] monitor thread exiting");
}

} // namespace openvr_pair::common
