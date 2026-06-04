#include "DriverMemoryProbe.h"

#include <cstdio>
#include <cstdint>

namespace openvr_pair::common {

bool IsReadableMemoryRange(const void* addr, size_t bytes)
{
	if (!addr) return false;

	MEMORY_BASIC_INFORMATION mbi{};
	if (!VirtualQuery(addr, &mbi, sizeof mbi)) return false;
	if (mbi.State != MEM_COMMIT) return false;

	const DWORD prot = mbi.Protect & 0xFF;
	if (prot == 0 || (prot & PAGE_NOACCESS) || (prot & PAGE_GUARD)) return false;

	const auto regionEnd = reinterpret_cast<uintptr_t>(mbi.BaseAddress) + mbi.RegionSize;
	const auto needEnd = reinterpret_cast<uintptr_t>(addr) + bytes;
	return needEnd <= regionEnd;
}

std::string DescribeVirtualQueryRegion(const char* tag, const void* addr)
{
	const char* label = tag ? tag : "(null)";
	char buffer[384]{};

	if (!addr) {
		std::snprintf(buffer, sizeof buffer, "%s: addr=NULL", label);
		return buffer;
	}

	MEMORY_BASIC_INFORMATION mbi{};
	if (!VirtualQuery(addr, &mbi, sizeof mbi)) {
		std::snprintf(buffer, sizeof buffer, "%s: addr=%p VirtualQuery FAILED (err=%lu)", label, addr, GetLastError());
		return buffer;
	}

	std::snprintf(buffer, sizeof buffer, "%s: addr=%p base=%p size=0x%llx state=0x%lx prot=0x%lx type=0x%lx", label,
	              addr, mbi.BaseAddress, static_cast<unsigned long long>(mbi.RegionSize), mbi.State, mbi.Protect,
	              mbi.Type);
	return buffer;
}

} // namespace openvr_pair::common
