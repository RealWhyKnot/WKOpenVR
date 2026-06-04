#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif

#include "PhantomInferenceShmem.h"

#include <cstring>

namespace phantom {

namespace {

template <typename Layout>
bool CreateMapping(const char* name, HANDLE& mapping_out, Layout*& layout_out, uint32_t magic)
{
	mapping_out = CreateFileMappingA(INVALID_HANDLE_VALUE, nullptr, PAGE_READWRITE, 0, sizeof(Layout), name);
	if (!mapping_out) return false;
	layout_out = reinterpret_cast<Layout*>(MapViewOfFile(mapping_out, FILE_MAP_ALL_ACCESS, 0, 0, sizeof(Layout)));
	if (!layout_out) {
		CloseHandle(mapping_out);
		mapping_out = nullptr;
		return false;
	}
	std::memset(layout_out, 0, sizeof(Layout));
	layout_out->magic = magic;
	layout_out->version = kPhantomInferenceShmemVersion;
	return true;
}

template <typename Layout> bool OpenMapping(const char* name, HANDLE& mapping_out, Layout*& layout_out, DWORD access)
{
	mapping_out = OpenFileMappingA(access, FALSE, name);
	if (!mapping_out) return false;
	layout_out = reinterpret_cast<Layout*>(MapViewOfFile(mapping_out, access, 0, 0, sizeof(Layout)));
	if (!layout_out) {
		CloseHandle(mapping_out);
		mapping_out = nullptr;
		return false;
	}
	return true;
}

template <typename Layout> void CloseMapping(HANDLE& mapping, Layout*& layout)
{
	if (layout) {
		UnmapViewOfFile(layout);
		layout = nullptr;
	}
	if (mapping) {
		CloseHandle(mapping);
		mapping = nullptr;
	}
}

} // namespace

bool PhantomInferenceInShmem::Create(const char* name)
{
	Close();
	return CreateMapping(name, mapping_, layout_, kPhantomInferenceShmemMagic);
}
bool PhantomInferenceInShmem::Open(const char* name)
{
	Close();
	return OpenMapping(name, mapping_, layout_, FILE_MAP_READ);
}
void PhantomInferenceInShmem::Close()
{
	CloseMapping(mapping_, layout_);
}

bool PhantomInferenceOutShmem::Create(const char* name)
{
	Close();
	return CreateMapping(name, mapping_, layout_, kPhantomInferenceShmemMagic ^ 0xFFFF);
}
bool PhantomInferenceOutShmem::Open(const char* name)
{
	Close();
	return OpenMapping(name, mapping_, layout_, FILE_MAP_READ);
}
void PhantomInferenceOutShmem::Close()
{
	CloseMapping(mapping_, layout_);
}

} // namespace phantom
