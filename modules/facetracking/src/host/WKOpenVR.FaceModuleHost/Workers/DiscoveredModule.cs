using WKOpenVR.FaceTracking.Registry;

namespace WKOpenVR.FaceModuleHost.Workers;

/// <summary>
/// Lightweight descriptor for a module found on disk. Replaces LoadedModule for the
/// subprocess model: no in-process Instance or AssemblyLoadContext -- the module DLL
/// runs inside WKOpenVR.FaceModuleProcess.exe, not the host.
/// </summary>
public sealed record DiscoveredModule(
    string Uuid,
    Manifest Manifest,
    string ModuleDllPath,
    ulong UuidHash,
    DateTime ModuleLastWriteUtc,
    long ModuleFileSize);
