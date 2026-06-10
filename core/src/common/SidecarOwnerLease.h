#pragma once

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>

#include "ModuleRegistry.h"

#include <atomic>
#include <cstdint>
#include <string>

namespace openvr_pair::common::sidecar_owner {

enum class LeaseState : uint32_t
{
	Starting = 1,
	Alive = 2,
	ShuttingDown = 3,
	Disabled = 4,
};

enum class WatchdogStatus
{
	Alive,
	Missing,
	HeaderMismatch,
	ModuleMismatch,
	NonceMismatch,
	ShuttingDown,
	Disabled,
	Stale,
};

struct LeaseSnapshot
{
	uint32_t magic = 0;
	uint32_t version = 0;
	uint32_t module_id = 0;
	uint32_t owner_pid = 0;
	uint64_t nonce = 0;
	uint32_t state = 0;
	uint64_t heartbeat_mono_ms = 0;
};

const char* WatchdogStatusName(WatchdogStatus status);
uint64_t MonotonicMillis();
uint64_t GenerateNonce();
std::string MakeLeaseName(modules::ModuleId id, uint32_t ownerPid, uint64_t nonce);

WatchdogStatus EvaluateLeaseSnapshot(const LeaseSnapshot& snapshot, modules::ModuleId expectedModule,
                                     uint64_t expectedNonce, uint64_t nowMonoMs, uint64_t staleAfterMs);

class LeaseOwner
{
public:
	LeaseOwner() = default;
	~LeaseOwner() { Close(); }

	LeaseOwner(const LeaseOwner&) = delete;
	LeaseOwner& operator=(const LeaseOwner&) = delete;

	bool Create(modules::ModuleId module);
	void Close();
	void Heartbeat(LeaseState state = LeaseState::Alive);
	void MarkShuttingDown();
	void MarkDisabled();

	bool IsOpen() const { return data_ != nullptr; }
	const std::string& Name() const { return name_; }
	uint64_t Nonce() const { return nonce_; }
	modules::ModuleId Module() const { return module_; }

private:
	struct LeaseData;

	HANDLE mapping_ = nullptr;
	LeaseData* data_ = nullptr;
	std::string name_;
	uint64_t nonce_ = 0;
	modules::ModuleId module_ = modules::ModuleId::Calibration;
};

class LeaseReader
{
public:
	LeaseReader() = default;
	~LeaseReader() { Close(); }

	LeaseReader(const LeaseReader&) = delete;
	LeaseReader& operator=(const LeaseReader&) = delete;

	bool Open(const std::string& name);
	void Close();
	bool TryRead(LeaseSnapshot& out) const;
	bool IsOpen() const { return data_ != nullptr; }

private:
	struct LeaseData;

	HANDLE mapping_ = nullptr;
	const LeaseData* data_ = nullptr;
};

} // namespace openvr_pair::common::sidecar_owner
