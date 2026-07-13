# oscrouter module

Routes OSC messages from driver modules and external sidecar processes to one
UDP endpoint, by default 127.0.0.1:9000 (VRChat's input port). The router runs
inside driver_wkopenvr.dll, gated on enable_oscrouter.flag; the overlay piece
adds a Router tab to WKOpenVR.exe for route management and live message rates.

## Pieces

- src/driver -- in-driver router: route table, OSC wire serialization, bounded
  send queue with a UDP sender thread, publish pipe server, and an in-process
  publish API other driver modules call directly.
- src/overlay -- Router tab plugin for the umbrella overlay: route
  subscribe/unsubscribe and config editing over the command pipe, plus a stats
  reader for per-route rates.
- tests -- gtest suite covering routing, shutdown ordering, and tab UI logic.

## Talks over

- \\.\pipe\WKOpenVR-OscRouter (overlay -> driver): route subscribe and
  unsubscribe, config push, and test-publish requests. Wire format is the
  protocol::Request/Response structs versioned by protocol::Version.
- \\.\pipe\WKOpenVR-OscRouterPub (sidecar -> driver): fire-and-forget publish
  pipe. Each frame is a 32-byte source id, a 4-byte little-endian length, then
  that many bytes of raw OSC.
- WKOpenVROscRouterStatsV1 shmem (driver -> overlay): per-route message rates,
  read at about 10 Hz.
- UDP out (driver -> network): serialized OSC packets to the configured
  host and port.

## Build and test

./test.ps1 -Suite oscrouter builds and runs the oscrouter_tests binary.

## Logs

oscrouter_log.<date>T<time>.txt under %LocalAppDataLow%\WKOpenVR\Logs, written
by the driver side only while debug logging is enabled.
