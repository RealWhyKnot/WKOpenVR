#include "SidecarOwnerLease.h"

#include <gtest/gtest.h>

namespace owner = openvr_pair::common::sidecar_owner;
namespace modules = openvr_pair::common::modules;

TEST(SidecarOwnerLease, FreshHeartbeatIsAlive)
{
	owner::LeaseOwner lease;
	ASSERT_TRUE(lease.Create(modules::ModuleId::FaceTracking));
	lease.Heartbeat();

	owner::LeaseReader reader;
	ASSERT_TRUE(reader.Open(lease.Name()));
	owner::LeaseSnapshot snapshot{};
	ASSERT_TRUE(reader.TryRead(snapshot));

	EXPECT_EQ(owner::EvaluateLeaseSnapshot(snapshot, modules::ModuleId::FaceTracking, lease.Nonce(),
	                                       owner::MonotonicMillis(), 3000),
	          owner::WatchdogStatus::Alive);
}

TEST(SidecarOwnerLease, DisabledAndShutdownStatesStopSidecar)
{
	owner::LeaseOwner lease;
	ASSERT_TRUE(lease.Create(modules::ModuleId::Captions));

	owner::LeaseReader reader;
	ASSERT_TRUE(reader.Open(lease.Name()));

	lease.MarkShuttingDown();
	owner::LeaseSnapshot snapshot{};
	ASSERT_TRUE(reader.TryRead(snapshot));
	EXPECT_EQ(owner::EvaluateLeaseSnapshot(snapshot, modules::ModuleId::Captions, lease.Nonce(),
	                                       owner::MonotonicMillis(), 3000),
	          owner::WatchdogStatus::ShuttingDown);

	lease.MarkDisabled();
	ASSERT_TRUE(reader.TryRead(snapshot));
	EXPECT_EQ(owner::EvaluateLeaseSnapshot(snapshot, modules::ModuleId::Captions, lease.Nonce(),
	                                       owner::MonotonicMillis(), 3000),
	          owner::WatchdogStatus::Disabled);
}

TEST(SidecarOwnerLease, RejectsWrongModuleAndNonce)
{
	owner::LeaseOwner lease;
	ASSERT_TRUE(lease.Create(modules::ModuleId::FaceTracking));
	lease.Heartbeat();

	owner::LeaseReader reader;
	ASSERT_TRUE(reader.Open(lease.Name()));
	owner::LeaseSnapshot snapshot{};
	ASSERT_TRUE(reader.TryRead(snapshot));

	EXPECT_EQ(owner::EvaluateLeaseSnapshot(snapshot, modules::ModuleId::Captions, lease.Nonce(),
	                                       owner::MonotonicMillis(), 3000),
	          owner::WatchdogStatus::ModuleMismatch);
	EXPECT_EQ(owner::EvaluateLeaseSnapshot(snapshot, modules::ModuleId::FaceTracking, lease.Nonce() + 1,
	                                       owner::MonotonicMillis(), 3000),
	          owner::WatchdogStatus::NonceMismatch);
}

TEST(SidecarOwnerLease, RejectsStaleHeartbeat)
{
	owner::LeaseOwner lease;
	ASSERT_TRUE(lease.Create(modules::ModuleId::FaceTracking));
	lease.Heartbeat();

	owner::LeaseReader reader;
	ASSERT_TRUE(reader.Open(lease.Name()));
	owner::LeaseSnapshot snapshot{};
	ASSERT_TRUE(reader.TryRead(snapshot));

	EXPECT_EQ(owner::EvaluateLeaseSnapshot(snapshot, modules::ModuleId::FaceTracking, lease.Nonce(),
	                                       snapshot.heartbeat_mono_ms + 3001, 3000),
	          owner::WatchdogStatus::Stale);
}

TEST(SidecarOwnerLease, MissingMappingDoesNotOpen)
{
	owner::LeaseReader reader;
	EXPECT_FALSE(reader.Open("Local\\WKOpenVR-MissingSidecarOwnerLease-UnitTest"));
}
