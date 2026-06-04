#pragma once

// Minimal OSC 1.0 packet parser and serializer.
//
// Handles: string, int32, float32, blob, and bundles.
// Byte order: OSC is big-endian on the wire; all multi-byte values are
// swapped on x86/x64 Windows.
//
// This is a hand-rolled implementation sized for the driver's needs.
// The spec is at https://opensoundcontrol.stanford.edu/spec-1_0.html

#include <cstdint>
#include <cstring>
#include <cstddef>
#include <functional>

namespace oscrouter {

// OSC 1.0 round-up to the next 4-byte boundary.
inline size_t OscPad4(size_t n)
{
	return (n + 3u) & ~3u;
}

// -------------------------------------------------------------------------
// Serialization
// -------------------------------------------------------------------------

// OscPacket holds a fixed-size output buffer and fills it with one OSC
// message. Caller checks Ok() before use.
//
// Usage:
//   OscPacket<512> pkt;
//   pkt.Begin("/avatar/parameters/JawOpen", ",f");
//   pkt.WriteFloat(0.75f);
//   if (pkt.Ok()) { send(pkt.Data(), pkt.Size()); }
template <size_t N> class OscPacket
{
public:
	OscPacket() : pos_(0), ok_(true) {}

	// Start a new message with the given address and typetag.
	// typetag must start with ',' (e.g. ",f" or ",ifs").
	void Begin(const char* address, const char* typetag)
	{
		pos_ = 0;
		ok_ = true;
		WriteStr(address);
		WriteStr(typetag);
	}

	void WriteInt32(int32_t v)
	{
		uint8_t b[4];
		b[0] = (uint8_t)(v >> 24);
		b[1] = (uint8_t)(v >> 16);
		b[2] = (uint8_t)(v >> 8);
		b[3] = (uint8_t)(v);
		Append(b, 4);
	}

	void WriteFloat(float v)
	{
		uint32_t bits;
		memcpy(&bits, &v, 4);
		WriteInt32(static_cast<int32_t>(bits));
	}

	void WriteStr(const char* s)
	{
		size_t len = strlen(s) + 1; // include NUL
		Append(reinterpret_cast<const uint8_t*>(s), len);
		// Pad to 4-byte boundary.
		size_t padded = OscPad4(len);
		for (size_t i = len; i < padded; ++i)
			AppendByte(0);
	}

	void WriteBlob(const void* data, uint32_t len)
	{
		WriteInt32(static_cast<int32_t>(len));
		Append(static_cast<const uint8_t*>(data), len);
		size_t padded = OscPad4(len);
		for (size_t i = len; i < padded; ++i)
			AppendByte(0);
	}

	// Write raw pre-encoded OSC argument bytes (already big-endian padded).
	void WriteRawArgs(const void* data, size_t len) { Append(static_cast<const uint8_t*>(data), len); }

	bool Ok() const { return ok_; }
	size_t Size() const { return pos_; }
	const uint8_t* Data() const { return buf_; }

private:
	uint8_t buf_[N];
	size_t pos_;
	bool ok_;

	void AppendByte(uint8_t b)
	{
		if (!ok_) return;
		if (pos_ >= N) {
			ok_ = false;
			return;
		}
		buf_[pos_++] = b;
	}

	void Append(const uint8_t* src, size_t n)
	{
		if (!ok_) return;
		if (pos_ + n > N) {
			ok_ = false;
			return;
		}
		memcpy(buf_ + pos_, src, n);
		pos_ += n;
	}
};

// -------------------------------------------------------------------------
// Parsing
// -------------------------------------------------------------------------

// OscReader wraps a raw OSC packet and extracts fields.
// All reads are bounds-checked; IsValid() goes false on overflow.
class OscReader
{
public:
	OscReader(const uint8_t* data, size_t size) : data_(data), size_(size), pos_(0), valid_(size > 0 && data != nullptr)
	{
	}

	bool IsValid() const { return valid_; }

	// Read a NUL-terminated OSC string (padded to 4 bytes). Returns pointer
	// into the buffer (valid for buffer lifetime) or nullptr on error.
	const char* ReadStr()
	{
		if (!valid_) return nullptr;
		const char* start = reinterpret_cast<const char*>(data_ + pos_);
		size_t remaining = size_ - pos_;
		size_t len = strnlen(start, remaining);
		if (len == remaining) {
			valid_ = false;
			return nullptr;
		} // no NUL
		size_t advance = OscPad4(len + 1);
		if (pos_ + advance > size_) {
			valid_ = false;
			return nullptr;
		}
		pos_ += advance;
		return start;
	}

	int32_t ReadInt32()
	{
		uint8_t b[4];
		if (!ReadBytes(b, 4)) return 0;
		return (int32_t)(((uint32_t)b[0] << 24) | ((uint32_t)b[1] << 16) | ((uint32_t)b[2] << 8) | (uint32_t)b[3]);
	}

	float ReadFloat()
	{
		int32_t bits = ReadInt32();
		float v;
		memcpy(&v, &bits, 4);
		return v;
	}

	// Read a blob: 4-byte big-endian length followed by padded data.
	// Returns pointer into buffer; sets `len_out` to blob length.
	const uint8_t* ReadBlob(uint32_t& len_out)
	{
		int32_t raw = ReadInt32();
		if (!valid_ || raw < 0) {
			valid_ = false;
			len_out = 0;
			return nullptr;
		}
		len_out = static_cast<uint32_t>(raw);
		if (pos_ + len_out > size_) {
			valid_ = false;
			return nullptr;
		}
		const uint8_t* ptr = data_ + pos_;
		pos_ += OscPad4(len_out);
		if (pos_ > size_) {
			valid_ = false;
			return nullptr;
		}
		return ptr;
	}

	bool AtEnd() const { return pos_ >= size_; }
	size_t Remaining() const { return size_ - pos_; }
	size_t Pos() const { return pos_; }

private:
	const uint8_t* data_;
	size_t size_;
	size_t pos_;
	bool valid_;

	bool ReadBytes(uint8_t* out, size_t n)
	{
		if (!valid_) return false;
		if (pos_ + n > size_) {
			valid_ = false;
			return false;
		}
		memcpy(out, data_ + pos_, n);
		pos_ += n;
		return true;
	}
};

// -------------------------------------------------------------------------
// Parsed message result
// -------------------------------------------------------------------------

struct OscMessage
{
	const char* address = nullptr;
	const char* typetag = nullptr;     // includes leading ','
	const uint8_t* arg_data = nullptr; // raw argument bytes
	size_t arg_size = 0;
	bool valid = false;
};

// Parse one complete OSC message from [data, data+size).
// Returns a view into the original buffer -- lifetime is that of `data`.
inline OscMessage OscParseMessage(const uint8_t* data, size_t size)
{
	OscMessage msg;
	OscReader r(data, size);
	msg.address = r.ReadStr();
	msg.typetag = r.ReadStr();
	if (!r.IsValid() || !msg.address || !msg.typetag) return msg;
	msg.arg_data = data + r.Pos();
	msg.arg_size = r.Remaining();
	msg.valid = true;
	return msg;
}

// -------------------------------------------------------------------------
// Bundle dispatch
// -------------------------------------------------------------------------

static const char kOscBundlePrefix[] = "#bundle";
static const size_t kOscBundlePrefixLen = 8; // "#bundle\0" padded to 8

// Callback type for dispatch -- (address, typetag, arg_data, arg_size).
using OscDispatchFn = std::function<void(const char*, const char*, const uint8_t*, size_t)>;

// Parse and dispatch one packet. If it is a bundle, each sub-message is
// dispatched individually (timetag discarded -- no timed delivery in v1).
// If it is a plain message, dispatched once.
inline void OscDispatch(const uint8_t* data, size_t size, OscDispatchFn fn)
{
	if (size < kOscBundlePrefixLen) return;
	if (memcmp(data, kOscBundlePrefix, kOscBundlePrefixLen) == 0) {
		// Bundle: skip 8-byte prefix + 8-byte timetag = 16 bytes.
		if (size < 16) return;
		size_t pos = 16;
		while (pos + 4 <= size) {
			OscReader lr(data + pos, 4);
			uint32_t len = static_cast<uint32_t>(lr.ReadInt32());
			pos += 4;
			if (pos + len > size) break;
			OscDispatch(data + pos, len, fn); // recursive for nested bundles
			pos += len;
		}
	}
	else {
		OscMessage msg = OscParseMessage(data, size);
		if (msg.valid) fn(msg.address, msg.typetag, msg.arg_data, msg.arg_size);
	}
}

// -------------------------------------------------------------------------
// OSC 1.0 address pattern matching
// -------------------------------------------------------------------------

// Matches an OSC address against an OSC 1.0 glob pattern.
// Supported: ?, *, [abc], [a-z], {a,b,c}, exact characters.
// Both address and pattern must be NUL-terminated.
bool OscPatternMatch(const char* pattern, const char* address);

} // namespace oscrouter
