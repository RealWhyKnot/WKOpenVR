#pragma once

#include <cstddef>

// In-process publish API for other driver modules (e.g. face-tracking).
// Thread-safe. Non-blocking -- returns false if the router is not active or
// the internal send queue is full (caller should treat this as a drop and
// continue; don't block or retry in a tight loop).
//
// source_id: short ASCII label for stats attribution (e.g. "facetracking").
// address:   OSC address string (e.g. "/avatar/parameters/JawOpen").
// typetag:   OSC typetag including leading ',' (e.g. ",f" or ",i").
// args:      pre-encoded OSC argument bytes (big-endian, padded to 4 bytes).
// arg_len:   byte count of `args`.
//
// PR #3 (FT migration) calls this from FacetrackingDriverModule::WorkerLoop
// after applying its filters. The OscRouter must be constructed before any
// module that calls this (enforced by the ordering in ServerTrackedDeviceProvider::Init).

namespace pairdriver::oscrouter {

bool PublishOsc(const char* source_id, const char* address, const char* typetag, const void* args, size_t arg_len);

} // namespace pairdriver::oscrouter
