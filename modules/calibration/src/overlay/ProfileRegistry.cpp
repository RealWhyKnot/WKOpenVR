#include "Configuration.h"
#include "CalibrationHeadMountShadow.h" // HeadMountSampleSourceName for the save-anomaly log
#include "CalibrationMetrics.h"
#include "TrackingStyle.h" // TrackingStyleLabel for the profile-loaded log

#include <atomic>
#include <cmath>
#include <condition_variable>
#include <cstdio>
#include <iostream>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>

static void LogRegistryResult(LSTATUS result)
{
	char* message;
	FormatMessageA(FORMAT_MESSAGE_FROM_SYSTEM | FORMAT_MESSAGE_ALLOCATE_BUFFER, 0, result, LANG_USER_DEFAULT,
	               (LPSTR)&message, 0, nullptr);
	std::cerr << "Opening registry key: " << message << '\n';
}

static const char* RegistryKey = "Software\\WKOpenVR-SpaceCalibrator";

// Strip the trailing null terminator that RegGetValueA reports as part of
// the byte count for REG_SZ. Pure for testability — see
// tests/test_configuration.cpp::ReadRegistryKey_StripNullTerminator_*.
//
// The reported `size` may legitimately be 0 if the registry value exists
// but is empty (or in a malformed/tampered state). Without this guard,
// the original code did `str.resize(size - 1)` which underflowed to
// (DWORD)0xFFFFFFFF and the resize threw std::bad_alloc, propagating up
// through LoadProfile and crashing the overlay before ParseProfile could
// even run. Returns 0 on size==0 (caller treats as "no profile, bootstrap
// fresh"); otherwise returns size-1 (drop the null).
size_t StripRegistryNullTerminator(DWORD reportedSize)
{
	return reportedSize > 0 ? (size_t)reportedSize - 1 : 0;
}

// Outcome of the last ReadRegistryKey() call. Distinguishes "key simply
// absent (first run)" from "key present but value read failed (I/O error,
// permissions)". Callers use this to decide whether to surface an error
// banner vs. silently bootstrapping a fresh profile.
enum class RegistryReadOutcome
{
	Absent,     // Key or value not found -- normal first-run condition.
	ReadFailed, // Key found but data read failed -- surface to user.
	Success,    // Data read successfully.
};
static RegistryReadOutcome g_lastRegistryReadOutcome = RegistryReadOutcome::Absent;

static std::string ReadRegistryKey()
{
	DWORD size = 0;
	auto result = RegGetValueA(HKEY_CURRENT_USER_LOCAL_SETTINGS, RegistryKey, "Config", RRF_RT_REG_SZ, 0, 0, &size);
	if (result != ERROR_SUCCESS) {
		// ERROR_FILE_NOT_FOUND / ERROR_PATH_NOT_FOUND: key genuinely absent
		// (first run or profile wiped). Anything else is an I/O / permission
		// failure that should be surfaced to the user.
		if (result == ERROR_FILE_NOT_FOUND || result == ERROR_PATH_NOT_FOUND) {
			g_lastRegistryReadOutcome = RegistryReadOutcome::Absent;
		}
		else {
			g_lastRegistryReadOutcome = RegistryReadOutcome::ReadFailed;
			std::cerr << "Registry size query failed (not a first-run absence): ";
			LogRegistryResult(result);
		}
		return "";
	}

	// Empty / malformed registry value: short-circuit before allocating.
	if (size == 0) {
		g_lastRegistryReadOutcome = RegistryReadOutcome::Absent;
		return "";
	}

	std::string str;
	str.resize(size);

	result = RegGetValueA(HKEY_CURRENT_USER_LOCAL_SETTINGS, RegistryKey, "Config", RRF_RT_REG_SZ, 0, &str[0], &size);
	if (result != ERROR_SUCCESS) {
		// Key was present and queryable but the data read itself failed.
		// This is distinct from "key missing" -- log clearly and flag for
		// the UI so it can show a "failed to read saved calibration" banner
		// rather than silently zeroing out and pretending it's a fresh start.
		g_lastRegistryReadOutcome = RegistryReadOutcome::ReadFailed;
		std::cerr << "Registry value read failed after successful size query: ";
		LogRegistryResult(result);
		return "";
	}

	g_lastRegistryReadOutcome = RegistryReadOutcome::Success;
	// `size` is re-populated by the data-fetch call. Use the helper so
	// the truncation logic stays unit-testable.
	str.resize(StripRegistryNullTerminator(size));
	return str;
}

static bool WriteRegistryKey(const std::string& str)
{
	HKEY hkey;
	auto result =
	    RegCreateKeyExA(HKEY_CURRENT_USER_LOCAL_SETTINGS, RegistryKey, 0, REG_NONE, 0, KEY_ALL_ACCESS, 0, &hkey, 0);
	if (result != ERROR_SUCCESS) {
		LogRegistryResult(result);
		return false;
	}

	DWORD size = static_cast<DWORD>(str.size() + 1);

	result = RegSetValueExA(hkey, "Config", 0, REG_SZ, reinterpret_cast<const BYTE*>(str.c_str()), size);
	const bool ok = (result == ERROR_SUCCESS);
	if (!ok) {
		LogRegistryResult(result);
	}

	RegCloseKey(hkey);
	return ok;
}

namespace {

// Latest-wins off-thread registry writer. Profile serialization (and every
// log annotation) stays on the tick thread; only the Win32 registry write
// moves here, so a slow HKCU flush can never stall a frame. One pending
// slot: registry state is last-writer-wins, so intermediate blobs that were
// superseded before the worker woke carry no information. Stop() drains the
// final pending blob before joining -- the shutdown flush relies on that.
class ProfileRegistryWriter
{
public:
	void Submit(std::string blob)
	{
		{
			std::lock_guard<std::mutex> lock(mutex_);
			if (!running_) {
				running_ = true;
				stop_ = false;
				worker_ = std::thread([this] { Run(); });
			}
			pending_ = std::move(blob);
			hasPending_ = true;
		}
		cv_.notify_one();
	}

	void Stop()
	{
		{
			std::lock_guard<std::mutex> lock(mutex_);
			if (!running_) return;
			stop_ = true;
		}
		cv_.notify_all();
		if (worker_.joinable()) worker_.join();
		std::lock_guard<std::mutex> lock(mutex_);
		running_ = false;
	}

	uint64_t Writes() const { return writes_.load(std::memory_order_relaxed); }
	uint64_t Failures() const { return failures_.load(std::memory_order_relaxed); }

private:
	void Run()
	{
		for (;;) {
			std::string blob;
			bool haveBlob = false;
			{
				std::unique_lock<std::mutex> lock(mutex_);
				cv_.wait(lock, [this] { return stop_ || hasPending_; });
				if (hasPending_) {
					blob = std::move(pending_);
					hasPending_ = false;
					haveBlob = true;
				}
				else if (stop_) {
					return;
				}
			}
			if (haveBlob) {
				if (WriteRegistryKey(blob)) {
					writes_.fetch_add(1, std::memory_order_relaxed);
				}
				else {
					failures_.fetch_add(1, std::memory_order_relaxed);
				}
			}
		}
	}

	std::mutex mutex_;
	std::condition_variable cv_;
	std::string pending_;
	bool hasPending_ = false;
	bool stop_ = false;
	bool running_ = false;
	std::thread worker_;
	std::atomic<uint64_t> writes_{0};
	std::atomic<uint64_t> failures_{0};
};

ProfileRegistryWriter g_profileRegistryWriter;

} // namespace

void StopProfileSaveWorker()
{
	g_profileRegistryWriter.Stop();
}

void LoadProfile(CalibrationContext& ctx)
{
	// @TODO: Rewrite this to migrate configs from the registry to the spacecal directory
	//        I don't know why whoever wrote this thought writing to the registry in the 2020s was a good idea...
	//        NOTE: HKEY_CURRENT_USER_LOCAL_SETTINGS evaluates to	HKCU\Software\Classes\Local Settings
	//              Settings are currently stored at				HKCU\Software\Classes\Local
	//              Settings\Software\WKOpenVR-SpaceCalibrator
	//
	//        Profiles stored at this registry path are now versioned via the top-level
	//        "schema_version" integer (see kProfileSchemaVersion). Legacy registry blobs
	//        written before versioning are treated as version 0 and migrated on read.

	ctx.validProfile = false;

	auto str = ReadRegistryKey();
	if (str == "") {
		if (g_lastRegistryReadOutcome == RegistryReadOutcome::ReadFailed) {
			// Registry key exists but the value could not be read (I/O error,
			// ACL change, corruption). Warn visibly rather than silently
			// zeroing the calibration.
			std::cerr << "LoadProfile: failed to read saved calibration from registry; "
			             "running with empty profile. Check registry permissions at "
			             "HKCU\\Software\\Classes\\Local Settings\\"
			          << RegistryKey << '\n';
		}
		else {
			std::cout << "Profile is empty" << '\n';
		}
		ctx.Clear();
		return;
	}

	try {
		std::stringstream io(str);
		ParseProfile(ctx, io);
		std::cout << "Loaded profile" << '\n';
		// Capture the load event in the spacecal log so anyone reading the
		// session can correlate any post-load behavior change with the
		// load itself (rather than guessing whether the user re-applied a
		// stale profile mid-session).
		// NOTE: ctx.calibratedTranslation is stored in centimetres (see
		// Calibration.cpp:2800 -- "convert to cm units for profile storage").
		// Do NOT multiply by 100; the value is already in cm.
		const double transMagCm = std::sqrt(ctx.calibratedTranslation.x() * ctx.calibratedTranslation.x() +
		                                    ctx.calibratedTranslation.y() * ctx.calibratedTranslation.y() +
		                                    ctx.calibratedTranslation.z() * ctx.calibratedTranslation.z());
		char loadBuf[320];
		snprintf(loadBuf, sizeof loadBuf,
		         "profile_loaded: bytes=%zu valid=%d trans_mag_cm=%.2f euler_deg=(%.2f,%.2f,%.2f)"
		         " ref_sys=%s tgt_sys=%s",
		         str.size(), (int)ctx.validProfile, transMagCm, ctx.calibratedRotation.x(), ctx.calibratedRotation.y(),
		         ctx.calibratedRotation.z(), ctx.referenceTrackingSystem.c_str(), ctx.targetTrackingSystem.c_str());
		Metrics::WriteLogAnnotation(loadBuf);
		// Surface the resolved tracking style so a session reader can tell at a
		// glance whether the headset is raw (Continuous/Manual) or driven by the
		// head-mount tracker (DriverSynth), and whether the raw-HMD fallback is
		// permitted. headMountMode 3 == DriverSynth.
		char styleBuf[256];
		snprintf(styleBuf, sizeof styleBuf,
		         "profile_loaded_tracking_style: style=%d (%s) headMountMode=%d allowRawHmdFallback=%d"
		         " lockMode=%d synthStaleMs=%d",
		         (int)ctx.trackingStyle, TrackingStyleLabel(ctx.trackingStyle), (int)ctx.headMount.mode,
		         (int)ctx.headMount.allowRawHmdFallback, (int)ctx.lockRelativePositionMode,
		         ctx.headMount.driverSynthTiming.staleLimitMs);
		Metrics::WriteLogAnnotation(styleBuf);
	}
	catch (const std::runtime_error& e) {
		std::cerr << "Error loading profile: " << e.what() << '\n';
		char errBuf[256];
		snprintf(errBuf, sizeof errBuf, "profile_load_failed: bytes=%zu reason=%s", str.size(), e.what());
		Metrics::WriteLogAnnotation(errBuf);
	}
}

void SaveProfile(CalibrationContext& ctx)
{
	std::stringstream io;
	WriteProfile(ctx, io);
	const std::string serialized = io.str();

	// Hash-and-skip: SaveProfile is invoked on every cal convergence tick
	// (~3.5 Hz), and most consecutive saves are byte-identical (the cal
	// is converged and steady-state). Live sessions logged 8000+ saves in
	// 37 min with mostly-identical content. Compute a cheap FNV-1a-64
	// hash of the serialized payload and skip both the registry write and
	// the annotation when it matches the last successful save. Cleared by
	// CalCtx.Clear() via the `lastSavedProfileHash = 0` reset there.
	static uint64_t s_lastSavedHash = 0;
	uint64_t hash = 0xcbf29ce484222325ULL;
	for (unsigned char c : serialized) {
		hash ^= c;
		hash *= 0x100000001b3ULL;
	}
	const bool unchanged = (hash != 0 && hash == s_lastSavedHash);
	if (unchanged) {
		// Skip the actual write -- nothing to persist. Suppress the
		// per-tick annotation flood; emit a throttled one-shot so the
		// log shows the cumulative skip rate without per-save noise.
		static int s_skipBurstCount = 0;
		static double s_lastSkipLogTime = -1e9;
		++s_skipBurstCount;
		const double nowSec = Metrics::CurrentTime;
		if ((nowSec - s_lastSkipLogTime) >= 30.0) {
			s_lastSkipLogTime = nowSec;
			char skipBuf[200];
			snprintf(skipBuf, sizeof skipBuf, "[profile-save][skipped] count_since_last_log=%d hash_unchanged=1",
			         s_skipBurstCount);
			Metrics::WriteLogAnnotation(skipBuf);
			s_skipBurstCount = 0;
		}
		return;
	}
	s_lastSavedHash = hash;

	std::cout << "Saving profile to registry" << '\n';
	g_profileRegistryWriter.Submit(serialized);

	// Surface worker write failures on the tick thread; the worker itself
	// never touches the metrics stream.
	static uint64_t s_lastReportedWriteFailures = 0;
	const uint64_t writeFailures = g_profileRegistryWriter.Failures();
	if (writeFailures != s_lastReportedWriteFailures) {
		s_lastReportedWriteFailures = writeFailures;
		Metrics::LogAnnotationf("[profile-save][write-failed] total_failures=%llu", (unsigned long long)writeFailures);
	}

	// Annotate the save event so the log lets a reader correlate writes
	// with the cal-state changes that triggered them. The wedged-state-saved
	// bug from 2026-05-03 was hard to debug because saves were silent; this
	// closes that gap. Includes the magnitude of what we just persisted so
	// a "saved a wedged cal at time T" question can be spot-checked.
	const double transMagCm = std::sqrt(ctx.calibratedTranslation.x() * ctx.calibratedTranslation.x() +
	                                    ctx.calibratedTranslation.y() * ctx.calibratedTranslation.y() +
	                                    ctx.calibratedTranslation.z() * ctx.calibratedTranslation.z());
	char saveBuf[256];
	snprintf(saveBuf, sizeof saveBuf, "profile_saved: bytes=%zu valid=%d trans_mag_cm=%.2f hash=0x%016llx",
	         serialized.size(), (int)ctx.validProfile, transMagCm, (unsigned long long)hash);
	Metrics::WriteLogAnnotation(saveBuf);

	// Anomaly detection: when the saved magnitude differs from the previous
	// saved magnitude by more than the threshold, that's either a real
	// big-change event (legitimate big move OR a relocalization) OR a
	// degenerate post-recovery save where the solver hasn't converged. Log
	// the anomaly so a reader can spot the case where a wedged value got
	// persisted -- the per-session 2026-05-19 trace showed exactly that
	// failure mode (`bytes=2261 trans_mag_cm=30392.05` after a Quest
	// relocalization, ~2 km from the steady-state magnitude). The save
	// itself is not blocked -- the hash-skip path will catch repeats.
	static double s_lastSavedTransMagCm = 0.0;
	constexpr double kProfileSaveDeltaWarnCm = 5.0;
	if (s_lastSavedTransMagCm > 0.0 && std::abs(transMagCm - s_lastSavedTransMagCm) > kProfileSaveDeltaWarnCm) {
		char anomBuf[640];
		snprintf(anomBuf, sizeof anomBuf,
		         "[profile-save][anomaly] trans_mag_cm=%.2f prev_trans_mag_cm=%.2f"
		         " delta_cm=%.2f bytes=%zu valid=%d mode=%d source=%s"
		         " offset_version=%u relPosCal=%d needsFreshRelPose=%d"
		         " head_tracker_serial='%s' target_serial='%s'"
		         " (large_delta_warning)",
		         transMagCm, s_lastSavedTransMagCm, std::abs(transMagCm - s_lastSavedTransMagCm), serialized.size(),
		         (int)ctx.validProfile, (int)ctx.headMount.mode,
		         HeadMountSampleSourceName(ctx.headMountLastSampleSource), (unsigned)ctx.headMountOffsetVersion,
		         (int)ctx.relativePosCalibrated, (int)ctx.headMountNeedsFreshRelativePose,
		         ctx.headMount.trackerSerial.c_str(), ctx.targetStandby.serial.c_str());
		Metrics::WriteLogAnnotation(anomBuf);
	}
	s_lastSavedTransMagCm = transMagCm;
}
