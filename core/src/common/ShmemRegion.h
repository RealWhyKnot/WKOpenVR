#pragma once

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>

#include <cstddef>

namespace openvr_pair::common {

// RAII view of a named shared-memory segment. Owns both the mapping handle
// and the mapped view; both are released on destruction.
class ShmemRegion
{
public:
	ShmemRegion() = default;
	ShmemRegion(const ShmemRegion&) = delete;
	ShmemRegion& operator=(const ShmemRegion&) = delete;
	ShmemRegion(ShmemRegion&& other) noexcept { *this = static_cast<ShmemRegion&&>(other); }
	ShmemRegion& operator=(ShmemRegion&& other) noexcept
	{
		if (this != &other) {
			Reset();
			m_mapping = other.m_mapping;
			m_view = other.m_view;
			other.m_mapping = nullptr;
			other.m_view = nullptr;
		}
		return *this;
	}
	~ShmemRegion() { Reset(); }

	// Opens an existing segment read-only. valid() is false when the segment
	// is missing or the view cannot be mapped. viewBytes == 0 maps the whole
	// segment.
	static ShmemRegion OpenForRead(const char* name, size_t viewBytes = 0)
	{
		ShmemRegion r;
		r.m_mapping = ::OpenFileMappingA(FILE_MAP_READ, FALSE, name);
		if (r.m_mapping == nullptr) return r;
		r.m_view = ::MapViewOfFile(r.m_mapping, FILE_MAP_READ, 0, 0, viewBytes);
		if (r.m_view == nullptr) r.Reset();
		return r;
	}

	bool valid() const { return m_view != nullptr; }
	const void* data() const { return m_view; }
	template <typename T> const T* as() const { return static_cast<const T*>(m_view); }

	void Reset()
	{
		if (m_view != nullptr) ::UnmapViewOfFile(m_view);
		if (m_mapping != nullptr) ::CloseHandle(m_mapping);
		m_view = nullptr;
		m_mapping = nullptr;
	}

private:
	HANDLE m_mapping = nullptr;
	void* m_view = nullptr;
};

} // namespace openvr_pair::common
