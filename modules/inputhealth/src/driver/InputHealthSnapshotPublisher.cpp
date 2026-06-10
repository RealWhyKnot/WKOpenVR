#include "InputHealthSnapshotPublisher.h"

#include "InputHealthSnapshotStaging.h"
#include "Logging.h"
#include "ModulePerf.h"
#include "Protocol.h"

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <cstring>
#include <exception>
#include <mutex>
#include <thread>
#include <unordered_map>
#include <vector>

namespace inputhealth {

namespace {

constexpr int kPublishHz = 10;
constexpr int kPublishPeriodMs = 1000 / kPublishHz;

// Module-scope state. Init / Shutdown are called from a single thread
// (driver lifecycle), so the booleans don't need atomics; the worker uses
// the condition variable to wake on Shutdown.
protocol::InputHealthSnapshotShmem g_shmem;
std::thread g_workerThread;
std::mutex g_workerMutex;
std::condition_variable g_workerCv;
bool g_workerStop = false;
bool g_running = false;

// handle -> slot index. Owned solely by the worker thread, so no mutex.
std::unordered_map<uint64_t, uint32_t> g_handleToSlot;
uint32_t g_nextFreeSlot = 0;
bool g_loggedFullTable = false;

void PublishOneTick()
{
	thread_local std::vector<StagedSnapshot> staged;
	staged.clear();
	StageSnapshots(staged);

	for (const auto& rec : staged) {
		uint32_t slot;
		auto it = g_handleToSlot.find(rec.handle);
		if (it == g_handleToSlot.end()) {
			if (g_nextFreeSlot >= protocol::INPUTHEALTH_SLOT_COUNT) {
				if (!g_loggedFullTable) {
					g_loggedFullTable = true;
					LOG("[inputhealth-publisher] slot table full at %u entries; further handles will not be published "
					    "this session",
					    protocol::INPUTHEALTH_SLOT_COUNT);
				}
				continue;
			}
			slot = g_nextFreeSlot++;
			g_handleToSlot[rec.handle] = slot;
		}
		else {
			slot = it->second;
		}
		g_shmem.WriteSlot(slot, rec.body);
	}

	g_shmem.BumpPublishTick();
}

void PublishOneTickSafely()
{
	static uint64_t s_publishErrors = 0;
	try {
		PublishOneTick();
	}
	catch (const std::exception& e) {
		++s_publishErrors;
		if (s_publishErrors == 1 || s_publishErrors == 100 || (s_publishErrors % 10000) == 0) {
			LOG("[inputhealth-publisher] skipped publish tick after error '%s' (count=%llu)", e.what(),
			    (unsigned long long)s_publishErrors);
		}
	}
	catch (...) {
		++s_publishErrors;
		if (s_publishErrors == 1 || s_publishErrors == 100 || (s_publishErrors % 10000) == 0) {
			LOG("[inputhealth-publisher] skipped publish tick after non-std exception (count=%llu)",
			    (unsigned long long)s_publishErrors);
		}
	}
}

void WorkerMain()
{
	openvr_pair::common::moduleperf::ScopedThreadRegistration perfRegistration(
	    openvr_pair::common::modules::ModuleId::InputHealth, "snapshot-publisher");
	LOG("[inputhealth-publisher] worker thread started (%d Hz)", kPublishHz);

	while (true) {
		{
			std::unique_lock<std::mutex> lk(g_workerMutex);
			g_workerCv.wait_for(lk, std::chrono::milliseconds(kPublishPeriodMs), [] { return g_workerStop; });
			if (g_workerStop) break;
		}

		PublishOneTickSafely();
	}

	LOG("[inputhealth-publisher] worker thread exiting");
}

} // namespace

void SnapshotPublisherInit()
{
	if (g_running) return;

	if (!g_shmem.Create(OPENVR_PAIRDRIVER_INPUTHEALTH_SHMEM_NAME)) {
		LOG("[inputhealth-publisher] FAILED to create shmem segment '%s' (err=%lu); snapshot publish disabled",
		    OPENVR_PAIRDRIVER_INPUTHEALTH_SHMEM_NAME, GetLastError());
		return;
	}

	g_handleToSlot.clear();
	g_nextFreeSlot = 0;
	g_loggedFullTable = false;

	{
		std::lock_guard<std::mutex> lk(g_workerMutex);
		g_workerStop = false;
	}
	g_workerThread = std::thread(WorkerMain);
	g_running = true;

	LOG("[inputhealth-publisher] online: shmem='%s' slot_count=%u period_ms=%d",
	    OPENVR_PAIRDRIVER_INPUTHEALTH_SHMEM_NAME, (unsigned)protocol::INPUTHEALTH_SLOT_COUNT, kPublishPeriodMs);
}

void SnapshotPublisherShutdown()
{
	if (!g_running) return;

	{
		std::lock_guard<std::mutex> lk(g_workerMutex);
		g_workerStop = true;
	}
	g_workerCv.notify_all();
	if (g_workerThread.joinable()) g_workerThread.join();

	g_shmem.Close();
	g_handleToSlot.clear();
	g_nextFreeSlot = 0;
	g_loggedFullTable = false;
	g_running = false;

	LOG("[inputhealth-publisher] offline");
}

} // namespace inputhealth
