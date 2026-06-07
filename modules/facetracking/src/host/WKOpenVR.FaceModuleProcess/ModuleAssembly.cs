// ----------------------------------------------------------------------------
// Vendored from VRCFaceTracking (Apache-2.0).
// Original: https://github.com/benaclejames/VRCFaceTracking/blob/35857c01315c32e0e45dcde2f6f8fe495216fa0c/
//   VRCFaceTracking.ModuleProcess/ModuleAssembly.cs
// Copyright (c) benaclejames and contributors. Licensed under Apache 2.0.
// Modifications: namespace renamed to WKOpenVR.FaceModuleProcess.
// ----------------------------------------------------------------------------
using System;
using System.Collections.Generic;
using System.Linq;
using System.Reflection;
using System.Runtime.Loader;
using System.Text;
using System.Threading.Tasks;
using Microsoft.Extensions.Logging;
using VRCFaceTracking;

namespace WKOpenVR.FaceModuleProcess;

public class ModuleAssembly
{
    private sealed class ModuleProcessLoadContext(string modulePath) : AssemblyLoadContext(
        name: Path.GetFileName(modulePath), isCollectible: true)
    {
        private static readonly HashSet<string> SharedAssemblyNames = new(StringComparer.OrdinalIgnoreCase)
        {
            "Microsoft.Extensions.Logging.Abstractions",
        };

        private readonly string _moduleDir = Path.GetDirectoryName(modulePath)
            ?? throw new ArgumentException("Module path must include a parent directory.", nameof(modulePath));

        protected override Assembly? Load(AssemblyName assemblyName)
        {
            if (string.Equals(assemblyName.Name, "VRCFaceTracking.Core", StringComparison.OrdinalIgnoreCase))
            {
                return typeof(UnifiedTracking).Assembly;
            }

            if (string.Equals(assemblyName.Name, "VRCFaceTracking.SDK", StringComparison.OrdinalIgnoreCase))
            {
                return typeof(ExtTrackingModule).Assembly;
            }

            if (assemblyName.Name is { } name && SharedAssemblyNames.Contains(name))
            {
                return null;
            }

            string candidate = Path.Combine(_moduleDir, assemblyName.Name + ".dll");
            return File.Exists(candidate) ? LoadFromAssemblyPath(candidate) : null;
        }

        protected override IntPtr LoadUnmanagedDll(string unmanagedDllName)
        {
            foreach (string name in new[] { unmanagedDllName, unmanagedDllName + ".dll" })
            {
                string candidate = Path.Combine(_moduleDir, name);
                if (File.Exists(candidate))
                {
                    return LoadUnmanagedDllFromPath(candidate);
                }
            }
            return IntPtr.Zero;
        }
    }

    public Assembly Assembly;
    public string ModulePath;
    public bool Loaded;
    private readonly ILogger<ModuleProcessMain> _logger;
    private readonly ILoggerFactory? _loggerFactory;
    public ExtTrackingModule TrackingModule;
    public CancellationTokenSource _updateCts;

    public ModuleAssembly(ILogger<ModuleProcessMain> logger, ILoggerFactory loggerFactory, string dllPath)
    {
        if (!File.Exists(dllPath))
        {
            throw new ArgumentException($"Invalid file path: \"{dllPath}\" does not exist!");
        }
        if (Path.GetExtension(dllPath.ToLowerInvariant()) != ".dll" && Path.GetExtension(dllPath.ToLowerInvariant()) != "dll")
        {
            throw new ArgumentException($"{dllPath} is not a DLL file and cannot be loaded.");
        }

        _logger = logger;
        _loggerFactory = loggerFactory;
        ModulePath = dllPath;
        Loaded = false;
    }

    public void TryLoadAssembly()
    {
        if (Loaded)
        {
            return;
        }

        try
        {
            var alc = new ModuleProcessLoadContext(ModulePath);
            Assembly = alc.LoadFromAssemblyPath(ModulePath);

            AssemblyName[] references = Assembly.GetReferencedAssemblies();
            var oldRefs = false;
            foreach (AssemblyName reference in references)
            {
                if (reference.Name is "VRCFaceTracking" or
                     "VRCFaceTracking.Core" or
                     "VRCFaceTracking.SDK")
                {
                    if (reference.Version < new Version(5, 0, 0, 0))
                    {
                        _logger.LogWarning("Module {dll} references an older version of VRCFaceTracking. Skipping.", Path.GetFileName(ModulePath));
                        oldRefs = true;
                    }
                }
            }
            if (oldRefs)
            {
                return;
            }

            foreach (Type type in Assembly.GetExportedTypes())
            {
                if (!type.IsSubclassOf(typeof(ExtTrackingModule)))
                {
                    continue;
                }

                _logger.LogDebug("{module} implements ExtTrackingModule.", type.Name);
                Loaded = true;
                TrackingModule = LoadExternalModule(type);
                break;
            }

            if (Loaded)
            {
                return;
            }

            foreach (Type type in Assembly.GetExportedTypes())
            {
                if (!NativeSdkTrackingModuleAdapter.IsNativeSdkModule(type))
                {
                    continue;
                }

                int moduleAbi = ReadModuleSdkAbi(type);
                string status = ClassifyModuleAbi(moduleAbi, out bool canLoad);
                if (!canLoad)
                {
                    _logger.LogError("Skipping native module {module}: {status}.", type.Name, status);
                    break;
                }

                _logger.LogInformation("{module} implements WKOpenVR native face module SDK ({status}).", type.Name, status);
                Loaded = true;
                TrackingModule = LoadNativeSdkModule(type);
                break;
            }
        }
        catch (Exception e)
        {
            _logger.LogWarning("{error} Assembly not able to be loaded. Skipping.", e.Message);
        }
    }

    // Native SDK contract (ABI) window this host understands. A module reports the SDK ABI it was
    // built against via WKOpenVR.FaceTracking.Sdk.FaceModuleAbi.Version (read by reflection below).
    // Modules within [HostMinimumSdkAbi, HostSupportedSdkAbi] load; older are too old to run; newer
    // were built for a newer app. Bump these together with breaking SDK contract changes.
    private const int HostSupportedSdkAbi = 1;
    private const int HostMinimumSdkAbi = 1;

    private static int ReadModuleSdkAbi(Type nativeType)
    {
        try
        {
            Type? abiType = nativeType.GetInterfaces()
                .FirstOrDefault(i => i.FullName == "WKOpenVR.FaceTracking.Sdk.IFaceTrackingModule")?
                .Assembly.GetType("WKOpenVR.FaceTracking.Sdk.FaceModuleAbi");
            object? value = abiType?.GetField("Version", BindingFlags.Public | BindingFlags.Static)?.GetRawConstantValue();
            return value is int abi ? abi : 1; // SDKs predating FaceModuleAbi are treated as the baseline (1).
        }
        catch (Exception)
        {
            return 1;
        }
    }

    private static string ClassifyModuleAbi(int moduleAbi, out bool canLoad)
    {
        if (moduleAbi > HostSupportedSdkAbi)
        {
            canLoad = false;
            return $"built against a newer SDK (ABI {moduleAbi}) than this app supports (ABI {HostSupportedSdkAbi}); update WKOpenVR";
        }

        if (moduleAbi == HostSupportedSdkAbi)
        {
            canLoad = true;
            return $"SDK ABI {moduleAbi}, current";
        }

        if (moduleAbi >= HostMinimumSdkAbi)
        {
            canLoad = true;
            return $"SDK ABI {moduleAbi}, older than this app (ABI {HostSupportedSdkAbi}) but supported in compatibility mode";
        }

        canLoad = false;
        return $"SDK ABI {moduleAbi} is below the minimum supported (ABI {HostMinimumSdkAbi}); rebuild against a newer SDK";
    }

    private ExtTrackingModule LoadExternalModule(Type moduleType)
    {
        if (Assembly == null)
        {
            throw new Exception("Assembly failed to load but tried setting up module!");
        }

        _logger.LogInformation("Loading External Module " + Assembly.FullName);

        try
        {
            var moduleObj = (ExtTrackingModule)Activator.CreateInstance(moduleType);
            ILogger logger = _loggerFactory.CreateLogger(moduleObj.GetType().Name);
            moduleObj.Logger = logger;

            return moduleObj;
        }
        catch (Exception e)
        {
            _logger.LogError("Exception loading {dll}. Skipping. {e}", Assembly.FullName, e);
        }

        return null;
    }

    private ExtTrackingModule LoadNativeSdkModule(Type moduleType)
    {
        if (Assembly == null)
        {
            throw new Exception("Assembly failed to load but tried setting up module!");
        }

        _logger.LogInformation("Loading Native SDK Module " + Assembly.FullName);

        try
        {
            var moduleObj = NativeSdkTrackingModuleAdapter.Create(moduleType, ModulePath);
            ILogger logger = _loggerFactory.CreateLogger(moduleObj.GetType().Name);
            moduleObj.Logger = logger;

            return moduleObj;
        }
        catch (Exception e)
        {
            _logger.LogError("Exception loading {dll}. Skipping. {e}", Assembly.FullName, e);
        }

        return null;
    }
}
