// ----------------------------------------------------------------------------
// Vendored from VRCFaceTracking (Apache-2.0).
// Original: https://github.com/benaclejames/VRCFaceTracking/blob/35857c01315c32e0e45dcde2f6f8fe495216fa0c/
//   VRCFaceTracking.ModuleProcess/ProxyLoggerProvider.cs
// Copyright (c) benaclejames and contributors. Licensed under Apache 2.0.
// Modifications: namespace renamed to WKOpenVR.FaceModuleProcess.
// ----------------------------------------------------------------------------
using System.Collections.Concurrent;
using Microsoft.Extensions.Logging;

namespace WKOpenVR.FaceModuleProcess;

public class ProxyLoggerProvider : ILoggerProvider
{
    private readonly ConcurrentDictionary<string, ProxyLogger> _loggers =
        new(StringComparer.OrdinalIgnoreCase);

    public ILogger CreateLogger(string categoryName)
    {
        return _loggers.GetOrAdd(categoryName, name => new ProxyLogger(name));
    }

    public void Dispose()
    {
        _loggers.Clear();
    }
}
