// ----------------------------------------------------------------------------
// Vendored from VRCFaceTracking (Apache-2.0).
// Original: https://github.com/benaclejames/VRCFaceTracking/blob/35857c01315c32e0e45dcde2f6f8fe495216fa0c/
//   VRCFaceTracking.ModuleProcess/ProxyLogger.cs
// Copyright (c) benaclejames and contributors. Licensed under Apache 2.0.
// Modifications: namespace renamed to WKOpenVR.FaceModuleProcess.
// ----------------------------------------------------------------------------
using System.Collections.ObjectModel;
using Microsoft.Extensions.Logging;

namespace WKOpenVR.FaceModuleProcess;

public delegate void OnLog(LogLevel level, string msg);

public class ProxyLogger(string categoryName) : ILogger
{
    private readonly string _categoryName = categoryName;
    public static OnLog OnLog;

    public IDisposable BeginScope<TState>(TState state) where TState : notnull
    {
        return default!;
    }

    public bool IsEnabled(LogLevel logLevel)
    {
        return true;
    }

    public void Log<TState>(
        LogLevel logLevel,
        EventId eventId,
        TState state,
        Exception? exception,
        Func<TState, Exception?, string> formatter)
    {
        OnLog?.Invoke(logLevel, $"[{_categoryName}] {logLevel}: {formatter(state, exception)}");
    }
}
