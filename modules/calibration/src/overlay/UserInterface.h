#pragma once

#include <string>

void BuildMainWindow(bool runningInOverlay);
void CCal_DrawTab();

// Signal to SC's UI that it is being rendered inside the OpenVR-Pair umbrella
// shell. While true, code paths that would otherwise spawn their own
// fullscreen ImGui window (the continuous-calibration display in particular)
// render their content inline so the umbrella's top tab bar stays visible
// and clickable. Reset to false for the standalone SpaceCalibrator binary.
void CCal_SetInUmbrella(bool inUmbrella);

void RequestImmediateRedraw();
void RequestExit();

// True once the VR stack (OpenVR runtime + driver IPC + shmem) has connected
// and the overlay is ready for calibration. False at startup if SteamVR
// isn't running yet -- the main loop retries the init sequence and flips
// this to true when the connection lands. UI gates calibration-action
// buttons and device dropdowns on this so the user can still browse the
// settings tabs while waiting.
bool IsVRReady();

// Last human-readable error from a failed connection attempt, for display
// in the "Waiting for SteamVR" banner. Empty string before the first
// attempt; never reset to empty after that (we just overwrite on each
// retry).
const std::string& LastVRConnectError();

// Presence snapshot: POD summary of CalibrationContext state, safe to call
// from SpaceCalibratorPlugin.cpp without including Calibration.h (which pulls
// in openvr_driver.h and conflicts with openvr.h in the plugin translation unit).
struct CCalPresenceSnapshot
{
	int state; // CalibrationState enum value (0=None..6=ContinuousStandby)
	bool validProfile;
	bool referencePoseOk;
	bool targetPoseOk;
	int sampleProgress;               // most recent progress counter from CalibrationContext::messages
	int sampleTarget;                 // corresponding target
	std::string targetTrackingSystem; // safe to copy; no VR types
};
CCalPresenceSnapshot CCal_GetPresenceSnapshot();
