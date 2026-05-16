// WKOpenVRPhantomSidecar.exe -- out-of-process ML pose-completion host.
//
// Phase 3 scaffold ships this as a passthrough stub: it opens the IN
// shmem the driver creates, mirrors per-role inputs into the OUT shmem
// with global_confidence = 0 so the bridge always falls through to the
// IK fallback. The follow-up that lands ONNX Runtime + a SparsePoser
// model replaces the inference body without touching the IPC / shmem
// boundary. AMASS / SMPL licensing review gates that follow-up.

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>

#include "PhantomInferenceShmem.h"
#include "Protocol.h"

#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <thread>

namespace {

std::atomic<bool> g_stop{false};

BOOL WINAPI CtrlHandler(DWORD type)
{
    switch (type) {
        case CTRL_C_EVENT:
        case CTRL_BREAK_EVENT:
        case CTRL_CLOSE_EVENT:
        case CTRL_SHUTDOWN_EVENT:
            g_stop.store(true, std::memory_order_release);
            return TRUE;
        default:
            return FALSE;
    }
}

} // namespace

int main(int argc, char* argv[])
{
    SetConsoleCtrlHandler(CtrlHandler, TRUE);

    bool self_test = false;
    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--self-test") == 0 ||
            std::strcmp(argv[i], "--healthcheck") == 0) {
            self_test = true;
        }
    }

    if (self_test) {
        std::fprintf(stderr,
            "[phantom-sidecar] self-test: magic=0x%08x version=%u role-count=%u "
            "sizeof(In)=%zu sizeof(Out)=%zu\n",
            phantom::kPhantomInferenceShmemMagic,
            phantom::kPhantomInferenceShmemVersion,
            (unsigned)phantom::kBodyRoleCount,
            sizeof(phantom::PhantomInferenceInLayout),
            sizeof(phantom::PhantomInferenceOutLayout));
        // Exercise the OUT shmem create path the sidecar uses at startup so a
        // broken Win32 mapping surface (named-object permissions, SDDL, etc.)
        // shows up here instead of only at first run against a real driver.
        phantom::PhantomInferenceOutShmem out_probe;
        if (!out_probe.Create("WKOpenVRPhantomSidecarSelfTestOut")) {
            std::fprintf(stderr, "[phantom-sidecar] self-test FAILED: OUT shmem Create (err=%u)\n",
                         (unsigned)GetLastError());
            return 10;
        }
        auto* L = out_probe.layout();
        if (!L ||
            L->magic   != (phantom::kPhantomInferenceShmemMagic ^ 0xFFFF) ||
            L->version != phantom::kPhantomInferenceShmemVersion) {
            std::fprintf(stderr, "[phantom-sidecar] self-test FAILED: OUT header mismatch\n");
            return 11;
        }
        std::fprintf(stderr, "[phantom-sidecar] self-test complete\n");
        return 0;
    }

    phantom::PhantomInferenceInShmem in;
    phantom::PhantomInferenceOutShmem out;

    // The driver creates IN. We poll for it to appear; the supervisor may
    // spawn us before the driver fully sets up.
    int attempt = 0;
    while (!g_stop.load(std::memory_order_acquire)
           && !in.Open(OPENVR_PAIRDRIVER_PHANTOM_INFERENCE_IN_SHMEM_NAME)) {
        if (++attempt > 600) {           // ~60 s total wait
            std::fprintf(stderr, "[phantom-sidecar] IN shmem never appeared; exiting\n");
            return 2;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    if (g_stop.load(std::memory_order_acquire)) return 0;

    if (!out.Create(OPENVR_PAIRDRIVER_PHANTOM_INFERENCE_OUT_SHMEM_NAME)) {
        std::fprintf(stderr, "[phantom-sidecar] OUT shmem create failed (err=%u)\n",
                     (unsigned)GetLastError());
        return 3;
    }

    std::fprintf(stderr, "[phantom-sidecar] connected; running passthrough stub\n");

    LARGE_INTEGER qpcFreq{}; QueryPerformanceFrequency(&qpcFreq);
    uint32_t last_frame = UINT32_MAX;

    while (!g_stop.load(std::memory_order_acquire)) {
        auto* L_in  = in.layout();
        auto* L_out = out.layout();
        if (!L_in || !L_out) break;

        const uint32_t frame = L_in->frame_id;
        if (frame != last_frame) {
            last_frame = frame;

            // Write OUT epoch into "writing" state (odd).
            L_out->epoch = L_out->epoch + 1;
            MemoryBarrier();

            L_out->frame_id = frame;
            // Stub: global_confidence = 0 means the driver bridge always
            // falls through to the IK fallback. When ONNX inference lands
            // this becomes a real per-frame number from the model head.
            L_out->global_confidence = 0.0f;
            LARGE_INTEGER qpc{}; QueryPerformanceCounter(&qpc);
            L_out->sidecar_qpc_ns = static_cast<uint64_t>(qpc.QuadPart);

            for (uint8_t i = 0; i < phantom::kBodyRoleCount; ++i) {
                const auto& tin = L_in->trackers[i];
                auto& tout = L_out->trackers[i];
                tout.role  = tin.role;
                tout.valid = 0;            // stub: never produce a pose
                std::memcpy(tout.position, tin.position, sizeof(tout.position));
                std::memcpy(tout.rotation, tin.rotation, sizeof(tout.rotation));
                tout.confidence = 0.0f;
                tout._pad[0] = tout._pad[1] = tout._pad[2] = tout._pad[3] = 0;
                tout._pad[4] = tout._pad[5] = 0;
                tout._reserved = 0.0f;
            }

            MemoryBarrier();
            L_out->epoch = L_out->epoch + 1; // back to even (stable)
        }

        // 90 Hz polling cadence is plenty for the passthrough stub. A real
        // ML backend would either tick on the IN-epoch change (event-driven)
        // or run at its own steady cadence with a frame-id drop guard.
        std::this_thread::sleep_for(std::chrono::milliseconds(11));
    }

    std::fprintf(stderr, "[phantom-sidecar] shutdown\n");
    return 0;
}
