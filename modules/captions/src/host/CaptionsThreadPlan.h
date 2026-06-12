#pragma once

#include <thread>

namespace captions {

// Choose how many CPU threads whisper may use for decode. Reserve a couple of
// logical cores so a CPU-path decode never starves the Windows desktop
// compositor or the overlay render thread, and cap the count where whisper's
// CPU scaling flattens out. Kept pure (parameterized on the core count) so the
// boundaries are unit-testable without touching the real machine.
inline int SelectWhisperThreadCount(unsigned hardware_concurrency)
{
	int cores = (hardware_concurrency == 0) ? 4 : static_cast<int>(hardware_concurrency);
	int threads = cores - 2; // leave room for desktop + overlay
	if (threads < 2) threads = 2;
	if (threads > 8) threads = 8;
	return threads;
}

inline int ResolveWhisperThreadCount()
{
	return SelectWhisperThreadCount(std::thread::hardware_concurrency());
}

} // namespace captions
