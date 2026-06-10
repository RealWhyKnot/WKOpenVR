#include "SidecarOwnerLease.h"

#include <cstddef>
#include <cstdio>

namespace openvr_pair::common::sidecar_owner {
namespace {

constexpr uint32_t kLeaseMagic = 0x534F574Eu; // "SOWN"
constexpr uint32_t kLeaseVersion = 1;
constexpr uint64_t kDefaultNonceSalt = 0x9E3779B97F4A7C15ull;

struct LeaseDataLayout
{
	uint32_t magic;
	uint32_t version;
	uint32_t module_id;
	uint32_t owner_pid;
	uint64_t nonce;
	uint32_t state;
	uint32_t reserved0;
	std::atomic<uint64_t> heartbeat_mono_ms;
	uint64_t reserved[3];
};

static_assert(offsetof(LeaseDataLayout, magic) == 0, "Sidecar owner lease magic offset drifted");
static_assert(offsetof(LeaseDataLayout, version) == 4, "Sidecar owner lease version offset drifted");
static_assert(offsetof(LeaseDataLayout, module_id) == 8, "Sidecar owner lease module offset drifted");
static_assert(offsetof(LeaseDataLayout, owner_pid) == 12, "Sidecar owner lease owner offset drifted");
static_assert(offsetof(LeaseDataLayout, nonce) == 16, "Sidecar owner lease nonce offset drifted");
static_assert(offsetof(LeaseDataLayout, state) == 24, "Sidecar owner lease state offset drifted");
static_assert(offsetof(LeaseDataLayout, heartbeat_mono_ms) == 32, "Sidecar owner lease heartbeat offset drifted");
static_assert(sizeof(LeaseDataLayout) == 64, "Sidecar owner lease layout drifted; bump version");

uint32_t ModuleIdWire(modules::ModuleId id)
{
	return static_cast<uint32_t>(id);
}

} // namespace

struct LeaseOwner::LeaseData : LeaseDataLayout
{
};

struct LeaseReader::LeaseData : LeaseDataLayout
{
};

const char* WatchdogStatusName(WatchdogStatus status)
{
	switch (status) {
		case WatchdogStatus::Alive:
			return "alive";
		case WatchdogStatus::Missing:
			return "missing";
		case WatchdogStatus::HeaderMismatch:
			return "header-mismatch";
		case WatchdogStatus::ModuleMismatch:
			return "module-mismatch";
		case WatchdogStatus::NonceMismatch:
			return "nonce-mismatch";
		case WatchdogStatus::ShuttingDown:
			return "shutting-down";
		case WatchdogStatus::Disabled:
			return "disabled";
		case WatchdogStatus::Stale:
			return "stale";
	}
	return "unknown";
}

uint64_t MonotonicMillis()
{
	return GetTickCount64();
}

uint64_t GenerateNonce()
{
	LARGE_INTEGER qpc{};
	QueryPerformanceCounter(&qpc);
	const uint64_t pid = static_cast<uint64_t>(GetCurrentProcessId());
	uint64_t value = static_cast<uint64_t>(qpc.QuadPart) ^ (MonotonicMillis() << 17) ^ (pid << 32) ^ kDefaultNonceSalt;
	if (value == 0) value = kDefaultNonceSalt;
	return value;
}

std::string MakeLeaseName(modules::ModuleId id, uint32_t ownerPid, uint64_t nonce)
{
	char buf[192] = {};
	std::snprintf(buf, sizeof(buf), "Local\\WKOpenVR-SidecarOwner-%s-%lu-%016llX", modules::Slug(id),
	              static_cast<unsigned long>(ownerPid), static_cast<unsigned long long>(nonce));
	return buf;
}

WatchdogStatus EvaluateLeaseSnapshot(const LeaseSnapshot& snapshot, modules::ModuleId expectedModule,
                                     uint64_t expectedNonce, uint64_t nowMonoMs, uint64_t staleAfterMs)
{
	if (snapshot.magic != kLeaseMagic || snapshot.version != kLeaseVersion) return WatchdogStatus::HeaderMismatch;
	if (snapshot.module_id != ModuleIdWire(expectedModule)) return WatchdogStatus::ModuleMismatch;
	if (snapshot.nonce != expectedNonce) return WatchdogStatus::NonceMismatch;
	if (snapshot.state == static_cast<uint32_t>(LeaseState::Disabled)) return WatchdogStatus::Disabled;
	if (snapshot.state == static_cast<uint32_t>(LeaseState::ShuttingDown)) return WatchdogStatus::ShuttingDown;
	if (snapshot.heartbeat_mono_ms == 0 || nowMonoMs < snapshot.heartbeat_mono_ms ||
	    nowMonoMs - snapshot.heartbeat_mono_ms > staleAfterMs) {
		return WatchdogStatus::Stale;
	}
	return WatchdogStatus::Alive;
}

bool LeaseOwner::Create(modules::ModuleId module)
{
	Close();
	module_ = module;
	nonce_ = GenerateNonce();
	name_ = MakeLeaseName(module, GetCurrentProcessId(), nonce_);
	mapping_ = CreateFileMappingA(INVALID_HANDLE_VALUE, nullptr, PAGE_READWRITE, 0, sizeof(LeaseData), name_.c_str());
	if (!mapping_) {
		name_.clear();
		nonce_ = 0;
		return false;
	}
	data_ = reinterpret_cast<LeaseData*>(MapViewOfFile(mapping_, FILE_MAP_ALL_ACCESS, 0, 0, sizeof(LeaseData)));
	if (!data_) {
		CloseHandle(mapping_);
		mapping_ = nullptr;
		name_.clear();
		nonce_ = 0;
		return false;
	}

	data_->magic = kLeaseMagic;
	data_->version = kLeaseVersion;
	data_->module_id = ModuleIdWire(module);
	data_->owner_pid = GetCurrentProcessId();
	data_->nonce = nonce_;
	data_->state = static_cast<uint32_t>(LeaseState::Starting);
	data_->reserved0 = 0;
	for (uint64_t& reserved : data_->reserved) {
		reserved = 0;
	}
	data_->heartbeat_mono_ms.store(MonotonicMillis(), std::memory_order_release);
	return true;
}

void LeaseOwner::Close()
{
	if (data_) {
		UnmapViewOfFile(data_);
		data_ = nullptr;
	}
	if (mapping_) {
		CloseHandle(mapping_);
		mapping_ = nullptr;
	}
	name_.clear();
	nonce_ = 0;
}

void LeaseOwner::Heartbeat(LeaseState state)
{
	if (!data_) return;
	data_->state = static_cast<uint32_t>(state);
	data_->heartbeat_mono_ms.store(MonotonicMillis(), std::memory_order_release);
}

void LeaseOwner::MarkShuttingDown()
{
	Heartbeat(LeaseState::ShuttingDown);
}

void LeaseOwner::MarkDisabled()
{
	Heartbeat(LeaseState::Disabled);
}

bool LeaseReader::Open(const std::string& name)
{
	Close();
	mapping_ = OpenFileMappingA(FILE_MAP_READ, FALSE, name.c_str());
	if (!mapping_) return false;
	data_ = reinterpret_cast<const LeaseData*>(MapViewOfFile(mapping_, FILE_MAP_READ, 0, 0, sizeof(LeaseData)));
	if (!data_) {
		CloseHandle(mapping_);
		mapping_ = nullptr;
		return false;
	}
	return true;
}

void LeaseReader::Close()
{
	if (data_) {
		UnmapViewOfFile(data_);
		data_ = nullptr;
	}
	if (mapping_) {
		CloseHandle(mapping_);
		mapping_ = nullptr;
	}
}

bool LeaseReader::TryRead(LeaseSnapshot& out) const
{
	if (!data_) return false;
	out.magic = data_->magic;
	out.version = data_->version;
	out.module_id = data_->module_id;
	out.owner_pid = data_->owner_pid;
	out.nonce = data_->nonce;
	out.state = data_->state;
	out.heartbeat_mono_ms = data_->heartbeat_mono_ms.load(std::memory_order_acquire);
	return true;
}

} // namespace openvr_pair::common::sidecar_owner
