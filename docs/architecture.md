# Architecture

WKOpenVR ships two binaries. Everything else in the repo is a feature module
that compiles into one of them or runs as a sidecar process next to them.

## One executable, one driver DLL

- `WKOpenVR.exe` -- the overlay host and UI shell. It registers the SteamVR
  dashboard overlay, draws every module's tab from a shared ImGui component
  library, owns configuration, and sends commands to the driver over IPC. It
  is control-plane only: no pose data flows through it.
- `driver_wkopenvr.dll` -- a single SteamVR driver loaded by `vrserver.exe`.
  All feature driver modules link into this one DLL.

There is one DLL instead of one per feature because the driver hooks
`vrserver.exe` with MinHook, and MinHook detours are process-global: only one
detour can exist per target function. Two independently installed driver DLLs
patching the same slot of `IVRDriverContext::GetGenericInterface` would
collide, and the second one's detours would silently never fire. One shared
DLL means one detour install per target function no matter which features are
enabled.

## Repo layout

- `core/src/common/` -- code shared by the exe, the DLL, and the sidecars:
  the wire protocol, the module registry, logging, JSON and Win32 helpers,
  sidecar supervision, recording envelopes.
- `core/src/driver/` -- the umbrella driver: MinHook install, interface hook
  injection, the IPC server, feature flag files, and the tracked-device
  provider that feature driver modules plug into.
- `core/src/overlay/` -- the umbrella overlay app: window and overlay
  lifecycle, the module tab host, the shared UI component library
  (`openvr_pair_overlay_ui`, see `../core/src/overlay/README.md`), IPC client
  base, manifest registration, safe-mode recovery.
- `modules/<name>/` -- one directory per feature, each with up to four
  subtrees: `src/overlay` (UI tab), `src/driver` (in-DLL logic), `src/host`
  (sidecar process), and `tests/` (unit tests). Some modules add extra
  subtrees (shared `src/common`, questapp's `src/android`).
- `tests/` -- cross-module suites: common and driver units, UI render smoke
  tests, end-to-end IPC tests, fixtures.
- `tools/` -- PowerShell replay and analysis scripts (calibration replay
  matrix, session replay gate, recording pinning) plus pinned replay
  baselines.
- `lib/` -- vendored third-party code and submodules: OpenVR SDK, MinHook,
  ImGui, ImPlot, GLFW, Eigen, picojson, stb_image, gl3w.
- `cmake/` -- the module gating helpers described below.

## Component diagram

```
                     SteamVR (vrserver.exe)
  +---------------------------------------------------------+
  |  driver_wkopenvr.dll                                    |
  |  MinHook detours + all feature driver modules           |
  +--------^---------------------^--------------------------+
           |                     |
           | named pipes         | shared memory
           | (one per module,    | (face-tracking frame ring,
           |  typed requests)    |  input-health snapshots)
           |                     |
  +--------+------------+   +---+------------------------+
  | WKOpenVR.exe        |   | sidecar hosts              |
  | overlay host + UI,  |   | facetracking (C# .NET)     |
  | config, IPC cmds    |   | captions (speech engine)   |
  +--------+------------+   | phantom (pose inference)   |
           |                +--------------^-------------+
           |   spawn / restart / owner lease
           +-------------------------------+
```

Each overlay module talks to its driver counterpart over its own named pipe
(names are defined in the module registry in `core/src/common/`). The driver
routes by request type and rejects messages on the wrong pipe. High-rate data
bypasses the pipes: the face-tracking host feeds frames to the driver through
a shared-memory ring, and the driver publishes input-health snapshots through
a shared-memory segment. The handshake fails fast on version skew; the wire
version is the `protocol::Version` constant in `core/src/common/Protocol.h`,
bumped on any layout change so a mismatched overlay/driver pair is caught at
startup instead of misrouting bytes.

Sidecar hosts are separate processes. A supervisor built on shared base code
in `core/src/common/` spawns the host, restarts it after a crash, halts on a
crash loop, and holds a shared-memory owner lease so two processes cannot
both launch the same host.

## The module pattern

`cmake/WKOpenVRModules.cmake` defines four helpers, one per subtree:
`wkopenvr_add_overlay_module`, `wkopenvr_add_driver_module`,
`wkopenvr_add_host_module`, and `wkopenvr_add_module_tests`. The core overlay
and driver CMakeLists call the first two per slug, the root CMakeLists adds
the sidecar hosts, and `tests/` registers the module test suites. Each helper
adds the matching `modules/<slug>/` subdirectory and, for overlay and driver,
appends the module's static library to the umbrella link list. Overlay
libraries are named
`openvr_pair_feature_<slug>_overlay` and driver libraries
`openvr_pair_feature_<slug>_driver`; the helpers also define
`OPENVR_PAIR_HAS_<SLUG>_OVERLAY/_DRIVER` so core code can compile against the
set of linked modules.

Two gates control what actually runs:

- Build time: a module with a `modules/<slug>/disabled-in-release.flag`
  marker is skipped across all four subtrees when CMake is configured with
  `-DWKOPENVR_RELEASE_BUILD=ON`, so its code never links into release
  binaries. Dev builds (the default) ignore the marker and build everything.
- Runtime: each built-in module is enabled by an `enable_<slug>.flag` marker
  file in the driver's `resources/` directory, read at SteamVR startup. The
  Modules tab in the overlay toggles these flags. The module registry in
  `core/src/common/` is the single table mapping slugs to flag files, display
  names, and pipe names.

## Pose data flow

Poses stay inside `vrserver.exe`. Device pose updates from hardware drivers
hit the detoured driver interfaces, where the feature driver modules process
them in place:

1. Calibration computes the transform between tracking universes (for
   example, a standalone HMD against lighthouse-tracked devices) and applies
   it to the poses of the calibrated devices.
2. Smoothing filters finger curls and suppresses per-device pose prediction.
3. Phantom (dev-only) can synthesize virtual trackers for missing or
   dropped-out hardware, using its inference sidecar.

The overlay exe never sits on that path. It reads state and pushes
configuration over the pipes; if it exits, tracking continues unchanged.

## Logging

Core logging lives in `core/src/common/`: `DebugLogging` (the global on/off
gate, toggled from the Logs tab and persisted as a flag file), `FileLog` (the
line-oriented file writer), `DiagnosticsLog` (structured diagnostics), and
`LogPaths` (the `%LocalAppDataLow%\WKOpenVR\Logs\` destination shared by all
processes).

On top of that, each module component (overlay, driver, host) carries its own
small `Logging.h` with its own namespace, macro, and log filename. This
duplication is deliberate, not an oversight:

- The driver DLL, the exe, and the sidecars are separate link targets; a
  per-component header keeps a sidecar from inheriting driver-side includes
  just to log a line.
- Each component names its own log file, so a session produces one file per
  process/module and interleaving problems cannot occur.
- Phantom's unit tests shadow `Logging.h` with a stub via include-path
  ordering (`modules/phantom/tests/teststubs/`) to capture log output in
  assertions. A single shared logging header would defeat that shadowing.

Do not consolidate these headers into one.

## Where to go next

- Wire protocol, pipe names, and shared-memory layouts: `ipc-protocol.md`.
- Per-module detail: `../modules/calibration/README.md`,
  `../modules/captions/README.md`, `../modules/dev/README.md`,
  `../modules/dynamicres/README.md`, `../modules/facetracking/README.md`,
  `../modules/inputhealth/README.md`, `../modules/oscrouter/README.md`,
  `../modules/phantom/README.md`, `../modules/questapp/README.md`,
  `../modules/smoothing/README.md`.
