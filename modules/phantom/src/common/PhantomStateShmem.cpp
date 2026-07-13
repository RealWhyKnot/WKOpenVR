#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif

#include "PhantomStateShmem.h"

#include <cstring>

namespace phantom {

namespace {

constexpr DWORD kMappingSize = sizeof(PhantomStateShmemLayout);

} // namespace

bool PhantomStateShmem::Create(const char* segmentName)
{
	Close();

	mapping_ = CreateFileMappingA(INVALID_HANDLE_VALUE, nullptr, PAGE_READWRITE, 0, kMappingSize, segmentName);
	if (!mapping_) {
		return false;
	}

	layout_ =
	    reinterpret_cast<PhantomStateShmemLayout*>(MapViewOfFile(mapping_, FILE_MAP_ALL_ACCESS, 0, 0, kMappingSize));
	if (!layout_) {
		CloseHandle(mapping_);
		mapping_ = nullptr;
		return false;
	}

	// Zero on first map; CreateFileMappingA returns ERROR_ALREADY_EXISTS if a
	// previous driver instance left state behind, in which case we still
	// re-stamp the header to recover from any stale partial state.
	std::memset(layout_, 0, kMappingSize);
	layout_->magic = kPhantomStateShmemMagic;
	layout_->version = kPhantomStateShmemVersion;
	layout_->device_count = kMaxPhantomDevices;
	layout_->last_snap_status = 255; // zero would read as SnapStatus Ok
	return true;
}

bool PhantomStateShmem::Open(const char* segmentName)
{
	Close();

	mapping_ = OpenFileMappingA(FILE_MAP_READ, FALSE, segmentName);
	if (!mapping_) {
		return false;
	}

	layout_ = reinterpret_cast<PhantomStateShmemLayout*>(MapViewOfFile(mapping_, FILE_MAP_READ, 0, 0, kMappingSize));
	if (!layout_) {
		CloseHandle(mapping_);
		mapping_ = nullptr;
		return false;
	}
	return true;
}

void PhantomStateShmem::Close()
{
	if (layout_) {
		UnmapViewOfFile(layout_);
		layout_ = nullptr;
	}
	if (mapping_) {
		CloseHandle(mapping_);
		mapping_ = nullptr;
	}
}

} // namespace phantom
