// ----------------------------------------------------------------------------
// Vendored from VRCFaceTracking (Apache-2.0).
// Original: https://github.com/benaclejames/VRCFaceTracking/blob/35857c01315c32e0e45dcde2f6f8fe495216fa0c/
//   VRCFaceTracking.ModuleProcess/ModuleProcessExitCodes.cs
// Copyright (c) benaclejames and contributors. Licensed under Apache 2.0.
// Modifications: namespace renamed to WKOpenVR.FaceModuleProcess.
// ----------------------------------------------------------------------------
namespace WKOpenVR.FaceModuleProcess;

public static class ModuleProcessExitCodes
{
    public const int OK = 0;
    public const int INVALID_ARGS = -1;
    public const int NETWORK_CONNECTION_TIMED_OUT = -2;
    public const int EXCEPTION_CRASH = -3;
}
