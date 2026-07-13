# questapp module

Manages a Quest headset from a WKOpenVR overlay tab: adb setup over USB or Wi-Fi, listing and
launching installed packages, and installing the bundled companion app. The module is
overlay-only -- the driver is not involved; the headset side is an Android foreground service
that answers HTTP requests and restarts itself after a reboot.

## Pieces

- `src/overlay/` -- overlay tab plugin: adb.exe wrapper and setup wizard, app catalog and
  per-user config, companion installer, and the HTTP client that reads companion settings.
- `src/android/` -- Gradle project for the companion app (boot receiver plus a foreground
  service that serves HTTP on the headset).
- `resources/` -- prebuilt companion APK and platform-tools install/uninstall scripts.
- `tests/` -- unit suite for the overlay pieces above.

## Talks over

- No driver pipes or shared-memory segments; the module's registry entry carries an empty
  pipe name.
- `adb.exe` subprocess (overlay -> headset): USB or wireless adb (`adb tcpip`, default
  port 5555).
- HTTP on TCP port 39789 (overlay -> companion service): the service listens on the headset;
  the overlay reaches it through an adb port forward of `tcp:39789`.

## Build and test

`./test.ps1 -Suite questapp` runs the module suite. The overlay piece builds into
WKOpenVR.exe; the APK ships prebuilt from `resources/`.

## Logs

No dedicated log file. Overlay-side lines are tagged `[adb]` and `[questapp]` inside the
shared `diagnostics_log.<timestamp>.txt` under `%LocalAppDataLow%\WKOpenVR\Logs`, written
only when debug logging is enabled.
