#pragma once

#include "IpcClientBase.h"
#include "Protocol.h"

// Named-pipe client for the Phantom-tracker pipe. Sends
// Phantom config, solver calibration, per-serial dropout toggles, role
// assignments, mounting offsets, and virtual-role toggles. Identical shape to
// the other per-module IPC clients (smoothing, inputhealth) -- thin wrapper
// over IpcClientBase that hard-codes the pipe name and supplies the
// user-facing error strings.
class PhantomIPCClient : public openvr_pair::overlay::IpcClientBase
{
public:
    void Connect();
};
