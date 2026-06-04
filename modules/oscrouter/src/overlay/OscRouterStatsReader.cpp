#include "OscRouterStatsReader.h"

bool OscRouterStatsReader::TryOpen()
{
	if (shmem_) return true;
	try {
		shmem_.Open(OPENVR_PAIRDRIVER_OSCROUTER_SHMEM_NAME);
		return true;
	}
	catch (...) {
		return false;
	}
}

void OscRouterStatsReader::Close()
{
	shmem_.Close();
}

bool OscRouterStatsReader::ReadGlobal(protocol::OscRouterStats& out) const
{
	return shmem_.ReadGlobalStats(out);
}

bool OscRouterStatsReader::ReadRoute(uint32_t index, protocol::OscRouterRouteSlot& out) const
{
	return shmem_.TryReadRoute(index, out);
}
