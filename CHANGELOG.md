# Changelog

All notable user-visible changes to WKOpenVR. The "Unreleased" section is auto-appended by `.github/workflows/changelog-append.yml` from conventional-commit subjects on `main`; tagged sections are promoted by `.github/workflows/release.yml` on a `v*` tag push.

The `release.yml` body for each tag is composed mechanically from the slice between the prior tag and the new tag plus the templated sections under `.github/release-template/`. Hand-writing release bodies is not part of the workflow.

## Unreleased

### Added
- **diagnostics:** Add OSC + translator + host-supervisor instrumentation (b0fa13d)
- **hosts:** Singleton mutex + connect-first spawn for face and translator hosts (cd69dac)
- **translator/overlay:** Diagnostics drawer on host-failed banner (d93a654)
- **translator/driver:** Native-dep probe + symbolic exit-code logging (d6ea090)
- **translator-host:** Delay-load CUDA imports + early-crash logger (f832345)
- **scripts:** Split local deploy and test paths (e14fd17)
- **osc:** Coordinate router-backed module outputs (2026.5.15.0-TEST) (8b2483c)
- **overlay:** Integrate Discord rich presence (770d457)
- **translator:** Speech and translation pack install flow (30f82ae)
- **overlay:** Per-module Discord presence composer (a0b7a3c)
- **inputhealth:** Classify pressure-sensitive analog axes as triggers (7dfdcec)
- **tests:** Cross-module e2e suite under tests/, consolidate test dirs (c9835b3)
- **translator:** Reliability pass -- config retry, vrmanifest, phase, ptt status (09409ba)
- **facetracking:** Reliability pass -- phase, telemetry, ALC, lifecycle, paths (f53b09e)
- **oscrouter:** Expose test endpoint override for e2e harness (8083e86)
- **smoothing:** Adaptive prediction smoothing toggle (c286c5d)
- **facetracking:** Emit upstream alias OSC names + fix OSCQuery advertise (b33289a)
- **facetracking:** Move VRCFT shape remap from host to driver (1ed0eb4)
- **facetracking:** Wedge detection via shmem heartbeat + host_state (6a52694)
- **supervisor:** Require singleton mutex + sweep stale hosts before spawn (9dc05d4)

### Changed
- host(facetracking): extend shmem to v2 with head data (2813401)
- host(facetracking): add SubprocessManager skeleton (unused) (5b74cf9)
- vendor(facetracking-host): bring in upstream VRCFT subprocess runtime + IPC (f908dc9)
- host(facetracking): switch active module manager from ModuleLoader to SubprocessManager (ac78c91)
- host(facetracking): restore v2 shmem (head fields + SHMEM_VERSION=2) (f1427c7)
- host(facetracking): robustness + diagnostic logging pass on SubprocessManager (70397b0)
- **facetracking:** Back out v2 shmem extension until driver rebuild (222f83b)
- Improve diagnostics and deploy flow (6048e4f)
- **build:** Wire compiler cache via sccache or ccache (29e4a64)
- **inputhealth:** Rename shmem segment to WKOpenVR scheme (34002fb)
- **supervisor:** Migrate translator host onto shared base (176b661)
- **supervisor:** Lift sidecar lifecycle into shared HostSupervisorBase (3814450)
- **facetracking:** Drop CalibrationCache stub, guard publish on shutdown (199de1b)
- **rename:** Align legacy shmem and pipe names with WKOpenVR brand (583a53c)
- **smoothing:** Split skeletal lock; shared_mutex for handedness lookup (8a81897)
- **driver:** Drop hot-path string allocations in detours (cad7ace)
- **inputhealth:** Drop try_lock from detours; single-pass StageSnapshots (dfa388f)
- **captions:** Rename translator module to captions (edf78cc)

### Fixed
- **smoothing:** Lock out per-tracker prediction on the calibration anchor (e87f318)
- **installer:** Include translator host tree in setup-exe and release zip (ca55616)
- **calibration:** Hide-tracker now flags pose as disconnected instead of punting out of bounds (2a0e8db)
- **host,supervisor:** Hide console window, guard respawn race, add circuit breaker (95ec265)
- **facetracking-host:** Wire VRCFT singleton and map upstream expression enum (03e85f0)
- **translator:** Surface circuit-breaker halt state with deps-missing guidance (6a318a0)
- **facetracking-host:** Pin Microsoft.Extensions.Logging.Abstractions v7.0.0 via PackageReference (0e41705)
- **facetracking-host:** Route bridge diagnostics through the file logger (a453100)
- **facetracking-host:** Load VrcftCompat adapter in default ALC; per-module ALC for upstream only (34e72b6)
- **facetracking-host:** Extract ALC names before interpolation (CS1056) (8085b84)
- **facetracking-host:** Force-load MELA so InjectNullLogger finds it (c5db8c7)
- **facetracking-host:** Force-load MELA via static member; resolve Eye as field-or-property (1f4435e)
- **smoothing:** Hide internal calibration devices (482ee0a)
- **inputhealth:** Persist global toggles and correct trigger compensation (da16380)
- **calibration:** Reject degenerate solves and reset sample state on restart (789e405)
- **quick:** Show elevated helper window during deploy (e9b413c)
- **inputhealth:** Retry late serial-hash resolution on hot path (39198dc)
- **smoothing:** Replay predictions on ipc reconnect and device-list change (e34072a)
- **driver:** Logging fallback errno, TRACE fallthrough, smoothness log throttle (cf78723)
- **paths:** Multi-candidate LocalAppDataLow with %TEMP% fallback (6eec7a2)
- **facetracking:** Align vendored UnifiedExpressions with upstream v5.1.1.0 (aaf92fa)
- **math:** Guard solver and pose inputs against NaN, Inf, and zero-mag (d0530b5)
- **facetracking:** Job-object + wait-check + read-timeout for host supervisor (72bd95d)
- **ipc:** Tighten IPCServer error paths and drop short-read dispatch (f6100c1)
- **ipc:** Null DriverPoseShmem handles after Close to prevent double-free (e1ff392)
- **facetracking:** Sanitize NaN/Inf face data at host and driver boundary (982592c)
- **calibration:** Clamp acos input and dedupe AngleFromRotationMatrix3 (160e8b0)

---

## [v2026.5.14.0](https://github.com/RealWhyKnot/WKOpenVR/releases/tag/v2026.5.14.0) — 2026-05-14

### Added
- **oscrouter:** OSC router substrate (v16 protocol, UDP send, OOP publish pipe, stats shmem) (0cdd797)
- **facetracking:** Migrate face-tracking OSC through router (a2dd55b)
- **translator:** Real-time STT + translation module for VR (203a18c)
- **overlay:** Expose OSC Router + Translator in Start Menu shortcut tables (ba74dfe)
- **facetracking:** Emit OSC parameters on both legacy and v2 paths (164c476)

### Changed
- **common:** Share Win32 and IPC helpers (4f3b435)

### Fixed
- **facetracking:** Host path walk, ALC type-identity, lenient version parse (68fd11d)
- **overlay:** VR scroll wheel sensitivity (360*8 -> 2) (1482ab3)
- **calibration:** Faster initial-cal, paired-motion validity, mode-aware defaults (e26d16f)
- **translator,facetracking:** Host path walk + CBOR control-pipe wire (f8e7f9d)
- **quick.ps1:** Also deploy WKOpenVR.TranslatorHost tree (a28eae4)

---

## [v2026.5.13.1](https://github.com/RealWhyKnot/WKOpenVR/releases/tag/v2026.5.13.1) — 2026-05-13

### Fixed
- **overlay:** Face-tracking version, legacy registry, scroll, search dedup, update notice (a274cfa)

---

## [v2026.5.13.0](https://github.com/RealWhyKnot/WKOpenVR/releases/tag/v2026.5.13.0) — 2026-05-13

### Added
- **inputhealth:** Publish scalar observed range (0.1.0.0) (3f6f42e)
- **driver:** InputHealth Stage 1E reset wiring and Stage 1F shmem publisher (0.1.0.0) (96a2829)
- **driver:** InputHealth Stage 1D -- per-tick stats wired into detour bodies (0.1.0.0) (ceb1416)
- **driver:** InputHealth Stage 1C -- pure-function math primitives (0.1.0.0) (050bc5c)
- **driver:** InputHealth Stage 1B -- install boolean/scalar hooks (pass-through) (0.1.0.0) (16c17ae)
- **driver:** Seed InputHealth subsystem (Stage 1A scaffold) (0.1.0.0) (26c42a4)
- **driver:** Split per-feature pipes and gate subsystems on enable_*.flag (d731767)
- **inputhealth:** Share snapshot diagnostics helpers (0.1.0.0) (5f2d770)
- **inputhealth:** Per-component learned compensation push (0.1.0.0) (a32e1c4)
- **overlay:** Full UI overhaul -- sidebar, cards, theme, typography (0.1.0.0) (e6a8c8e)
- **overlay:** Clean up Modules tab + propagate build version to SC plugin (2026.5.11.1-BD83) (0.1.0.0) (8cb45d4)
- **protocol:** Split per-device prediction off SetDeviceTransform (v12) (b5b5e22)
- **overlay:** Shared ShellFooter helper for feature plugins (2026.5.11.8-6B5C) (488501b)
- **shell:** Shared UI helpers, wrapping footer, end-to-end deploy (d274d2a)
- **driver:** Protocol v13 + cross-module robustness pass (d660034)
- **protocol:** Bump Version 13 -> 14 for InputHealthConfig shrinking (0e72bf9)
- **overlay:** SteamVR auto-launch via vrmanifest + UI polish pass (d284aae)
- **overlay:** Unify logs + add Smoothing logging + icon + dev-channel gate (6736e10)
- **install:** Stage facetracking host in build + deploy + NSIS installer (2026.5.12.13-3A37) (4c700ca)
- **facetracking:** Add module - driver math, overlay UI, .NET 10 host, tests (2026.5.12.13-3A37) (2fe23e8)
- **protocol:** Bump to v15 with face-tracking wire types (2026.5.12.13-3A37) (88651af)
- **overlay:** Add dashboard renderer for the SteamVR overlay surface (2026.5.12.13-3A37) (1aa0fbf)
- **facetracking:** Reflection bridge for VRCFT upstream modules (3d6bc1f)
- **facetracking:** Host_status.json sidecar + native-lib resolver (2372d02)
- **facetracking:** Strip Ed25519 signing path; point host at legacy-registry.whyknot.dev (25a36f0)
- **facetracking:** Wire driver telemetry; replace V1 stubs with real wiring or removal (1c0b487)
- **facetracking:** Module sources (folder + GitHub) with SHA verify; drop legacy trust UI (46a4224)
- **facetracking:** Redesign Modules tab around multi-select (32e0d77)
- **overlay:** Responsive resize, theme system, and UI helper additions (afc4c05)

### Changed
- Rename export macro to OPENVRPAIRDRIVER_EXPORTS (c7df7e6)
- Rename driver entry files to OpenVR-WKPairDriver (0.1.0.0) (52e05f7)
- **inputhealth:** Split hook injector by concern (0.1.0.0) (102a28c)
- **repo:** Become the glue that composes feature modules into one binary (0.1.0.0) (9d493f7)
- **deps:** Bump SpaceCalibrator submodule for BuildStamp.h fix (0.1.0.0) (e3ef50a)
- Revert "feat(overlay): full UI overhaul -- sidebar, cards, theme, typography" (0.1.0.0) (4582786)
- **deps:** Bump SpaceCalibrator submodule for umbrella tab + font fixes (0.1.0.0) (e4d8e3a)
- Rename OpenVR modules to WK variants (1ca4829)
- Refresh feature module pins after rebase (294a192)
- Bump overlay feature modules (abaed99)
- Unify overlay status banners (a79982a)
- Refresh feature pins for consumer build fixes (61c9fe6)
- Refresh feature module pins after build-fix rebase (ae0d3bb)
- Bump feature modules for build fixes (439ec19)
- Clear stale CMake generator instance cache (35ccfb5)
- **repo:** Collapse four-repo layout into monorepo tree (f2e1b48)
- rename: OpenVR-WK* repos -> WKOpenVR* scheme (4135339)
- ux: drop FT + smoothing master toggles; smoothing strength=0 disables fingers (40ce0a6)
- rename: product OpenVR-Pair -> WKOpenVR with first-launch AppData + registry + vrmanifest migration (8c99984)

### Fixed
- **inputhealth:** Make observation fail open (0.1.0.0) (a01318b)
- **build:** Wrap cmake calls with EAP='Continue' to dodge PS 5.1 ErrorRecord wrap (0.1.0.0) (bded14d)
- **overlay:** Drop top header, move transient status to footer line (2026.5.11.5-BAE4) (0.1.0.0) (6ee20ce)
- **overlay:** Correct module-toggle status copy for desktop-mode users (2026.5.11.2-30C9) (0.1.0.0) (4631071)
- **driver:** MotionGate Tiny requires both axes in noise (AND, not OR) (2ede4c6)
- **build:** Silence PowerShell 5.1 NativeCommandError noise + add -Wno-dev (9839a7b)
- **overlay:** Set GLFW title-bar icon via WM_SETICON on Windows (9f55188)
- **overlay:** Elevate flag-toggle via EncodedCommand and -Path for New-Item (2026.5.12.13-3A37) (eb72415)
- **build:** Resolve compile errors after phases 2-4 (friend decls, includes, namespace qualifications) (7e664fd)
- **hooks:** Refresh stale version.txt stamp to today when prepare-commit-msg fires (b3e127a)
- **calibration:** Suppress continuous-cal when a non-HMD anchor goes silent (528ab12)
- **quick.ps1:** Copy openvr_api.dll alongside the umbrella exe (22e2ccb)
- **quick.ps1:** Deploy the overlay resources tree alongside the exe (2f4ca75)
- **facetracking:** Make the Sync subprocess actually work end-to-end (7322dc2)
- **overlay:** Suppress shell scrollbar so SC tab does not stack two (61d51f6)
- **ci:** Delimit MaxAttempts in release verify-body log lines (ad51073)
