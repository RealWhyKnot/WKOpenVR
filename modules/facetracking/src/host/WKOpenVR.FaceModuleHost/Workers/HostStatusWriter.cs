// Periodically writes a host_status.json file the overlay polls to
// populate the Settings/Modules/Advanced tabs with live state.
//
// The host and the overlay are different processes (the driver spawns
// the host; the overlay talks to the driver). Plumbing real-time
// status all the way from host -> driver -> overlay would need a new
// IPC channel and a Protocol.h version bump. For values that update
// at human-readable rates (active module name, installed module list),
// a file the overlay re-reads once a second is plenty: zero protocol
// changes, zero driver-side C++ changes, and the file format can grow
// without breaking on-wire compatibility.
//
// Atomicity: writes to a sibling .tmp file then File.Move with
// overwrite. Readers that race with a write see either the previous
// or the new file, never a partial one. Same trick the host's calib
// flush uses.

using System.Diagnostics;
using System.Text.Json;
using System.Text.Json.Serialization;
using WKOpenVR.FaceModuleHost.Logging;

namespace WKOpenVR.FaceModuleHost.Workers;

public sealed class HostStatusWriter(
    string statusFilePath,
    SubprocessManager loader,
    HostLogger logger,
    HostOptions options)
{
    private readonly Stopwatch _uptime = Stopwatch.StartNew();
    private readonly DateTime _startedAt = DateTime.UtcNow;
    private readonly int _pid = Environment.ProcessId;

    public static void WriteStartupFailure(
        string statusFilePath,
        string phase,
        string lastError,
        HostOptions options,
        HostLogger logger)
    {
        try
        {
            string? dir = Path.GetDirectoryName(statusFilePath);
            if (!string.IsNullOrEmpty(dir))
            {
                Directory.CreateDirectory(dir);
            }

            var status = new HostStatus
            {
                SchemaVersion = 1,
                HostPid = Environment.ProcessId,
                HostStartedAt = DateTime.UtcNow,
                HostUptimeSeconds = 0,
                HostShuttingDown = true,
                Phase = phase,
                LastError = lastError,
                ModuleCount = 0,
                InstalledModules = [],
                ModulesInstallDir = options.ModulesInstallDir,
                FramesWritten = 0,
                FramesWithData = 0,
                FramesRead = 0,
                OscMessagesSent = 0,
                LastExitCode = 0,
                LastRestartTime = "",
            };
            string json = JsonSerializer.Serialize(
                status,
                HostStatusJsonContext.Default.HostStatus);
            string tmp = statusFilePath + ".tmp";
            File.WriteAllText(tmp, json);
            File.Move(tmp, statusFilePath, overwrite: true);
        }
        catch (Exception ex)
        {
            logger.Warn($"HostStatusWriter: startup failure write failed: {ex.Message}");
        }
    }

    public static void WriteE2eStatus(
        string statusFilePath,
        string phase,
        long framesWritten,
        HostOptions options,
        HostLogger logger)
    {
        try
        {
            string? dir = Path.GetDirectoryName(statusFilePath);
            if (!string.IsNullOrEmpty(dir))
            {
                Directory.CreateDirectory(dir);
            }

            var status = new HostStatus
            {
                SchemaVersion = 1,
                HostPid = Environment.ProcessId,
                HostStartedAt = DateTime.UtcNow,
                HostUptimeSeconds = 0,
                HostShuttingDown = phase.EndsWith("complete", StringComparison.OrdinalIgnoreCase),
                Phase = phase,
                LastError = "",
                ModuleCount = 0,
                InstalledModules = [],
                ModulesInstallDir = options.ModulesInstallDir,
                FramesWritten = framesWritten,
                FramesWithData = 0,
                FramesRead = 0,
                OscMessagesSent = 0,
                LastExitCode = 0,
                LastRestartTime = "",
            };
            string json = JsonSerializer.Serialize(
                status,
                HostStatusJsonContext.Default.HostStatus);
            string tmp = statusFilePath + ".tmp";
            File.WriteAllText(tmp, json);
            File.Move(tmp, statusFilePath, overwrite: true);
        }
        catch (Exception ex)
        {
            logger.Warn($"HostStatusWriter: e2e status write failed: {ex.Message}");
        }
    }

    public async Task RunAsync(CancellationToken ct)
    {
        // Ensure parent directory exists; the driver creates it on first install
        // but a freshly-extracted zip might not have hit that path yet.
        string? dir = Path.GetDirectoryName(statusFilePath);
        if (!string.IsNullOrEmpty(dir))
        {
            try { Directory.CreateDirectory(dir); }
            catch (Exception ex) { logger.Warn($"HostStatusWriter: could not ensure {dir}: {ex.Message}"); }
        }

        logger.Info($"HostStatusWriter: publishing to {statusFilePath} every 1 s.");

        while (!ct.IsCancellationRequested)
        {
            try
            {
                WriteOnce();
            }
            catch (Exception ex)
            {
                // Don't kill the worker; a transient FS error shouldn't take the host down.
                logger.Warn($"HostStatusWriter: write failed: {ex.Message}");
            }

            try { await Task.Delay(TimeSpan.FromSeconds(1), ct); }
            catch (OperationCanceledException) { break; }
        }

        // Final write so the overlay sees a clean "host shutting down" state if it polls again.
        try
        {
            WriteOnce(shuttingDown: true);
        }
        catch { /* shutdown noise */ }
    }

    private void WriteOnce(bool shuttingDown = false)
    {
        HostRuntimeStatus runtime = loader.SnapshotStatus();
        var status = new HostStatus
        {
            SchemaVersion = 1,
            HostPid = _pid,
            HostStartedAt = _startedAt,
            HostUptimeSeconds = (int)_uptime.Elapsed.TotalSeconds,
            HostShuttingDown = shuttingDown,
            Phase = runtime.Phase,
            LastError = runtime.LastError,
            ModuleCount = loader.Loaded.Count,
            ActiveModule = BuildActiveModule(),
            InstalledModules = ScanInstalledModules(),
            ModulesInstallDir = options.ModulesInstallDir,
            ActiveModuleUuid = loader.Active?.Uuid ?? "",
            ActiveModuleName = loader.Active?.Manifest.Name ?? "",
            FramesWritten = runtime.FramesWritten,
            FramesWithData = runtime.FramesWithData,
            FramesRead = 0,
            OscMessagesSent = 0,
            LastExitCode = runtime.LastExitCode,
            LastRestartTime = runtime.LastRestartTime?.ToString("O") ?? "",
        };

        string json = JsonSerializer.Serialize(
            status,
            HostStatusJsonContext.Default.HostStatus);

        string tmp = statusFilePath + ".tmp";
        File.WriteAllText(tmp, json);
        // File.Move with overwrite is atomic on NTFS for replace-with-same-volume.
        File.Move(tmp, statusFilePath, overwrite: true);
    }

    private ActiveModuleStatus? BuildActiveModule()
    {
        DiscoveredModule? m = loader.Active;
        if (m is null)
        {
            return null;
        }

        return new ActiveModuleStatus
        {
            Uuid = m.Manifest.Uuid,
            Name = m.Manifest.Name,
            Vendor = m.Manifest.Vendor,
            Version = m.Manifest.Version?.ToString() ?? "",
        };
    }

    private List<InstalledModule> ScanInstalledModules()
    {
        var result = new List<InstalledModule>();
        if (!Directory.Exists(options.ModulesInstallDir))
        {
            return result;
        }

        foreach (var uuidDir in Directory.EnumerateDirectories(options.ModulesInstallDir))
        {
            string uuid = Path.GetFileName(uuidDir);
            string? newestVersion = null;
            DateTime newestMtime = DateTime.MinValue;

            foreach (var versionDir in Directory.EnumerateDirectories(uuidDir))
            {
                string version = Path.GetFileName(versionDir);
                DateTime mtime = Directory.GetLastWriteTimeUtc(versionDir);
                if (mtime > newestMtime)
                {
                    newestMtime = mtime;
                    newestVersion = version;
                }
            }

            if (newestVersion is null)
            {
                continue;
            }

            // Pull manifest fields from the per-version manifest.json if present.
            string manifestPath = Path.Combine(uuidDir, newestVersion, "manifest.json");
            string? name = null, vendor = null;
            if (File.Exists(manifestPath))
            {
                try
                {
                    using var doc = JsonDocument.Parse(File.ReadAllText(manifestPath));
                    if (doc.RootElement.TryGetProperty("name", out JsonElement n))
                    {
                        name = n.GetString();
                    }

                    if (doc.RootElement.TryGetProperty("vendor", out JsonElement v))
                    {
                        vendor = v.GetString();
                    }
                }
                catch { /* malformed manifest -- skip the extras, keep the listing */ }
            }

            result.Add(new InstalledModule
            {
                Uuid = uuid,
                Name = name ?? uuid,
                Vendor = vendor ?? "",
                Version = newestVersion,
            });
        }

        result.Sort(static (a, b) => string.CompareOrdinal(a.Name, b.Name));
        return result;
    }
}

public sealed class HostStatus
{
    [JsonPropertyName("schema_version")] public int SchemaVersion { get; init; }
    [JsonPropertyName("host_pid")] public int HostPid { get; init; }
    [JsonPropertyName("host_started_at")] public DateTime HostStartedAt { get; init; }
    [JsonPropertyName("host_uptime_s")] public int HostUptimeSeconds { get; init; }
    [JsonPropertyName("host_shutting_down")] public bool HostShuttingDown { get; init; }
    [JsonPropertyName("phase")] public string Phase { get; init; } = "";
    [JsonPropertyName("last_error")] public string LastError { get; init; } = "";
    [JsonPropertyName("module_count")] public int ModuleCount { get; init; }
    [JsonPropertyName("active_module_uuid")] public string ActiveModuleUuid { get; init; } = "";
    [JsonPropertyName("active_module_name")] public string ActiveModuleName { get; init; } = "";
    [JsonPropertyName("frames_written")] public long FramesWritten { get; init; }
    [JsonPropertyName("frames_with_data")] public long FramesWithData { get; init; }
    [JsonPropertyName("frames_read")] public long FramesRead { get; init; }
    [JsonPropertyName("osc_messages_sent")] public long OscMessagesSent { get; init; }
    [JsonPropertyName("last_exit_code")] public int LastExitCode { get; init; }
    [JsonPropertyName("last_restart_time")] public string LastRestartTime { get; init; } = "";
    [JsonPropertyName("modules_install_dir")] public string ModulesInstallDir { get; init; } = "";
    [JsonPropertyName("active_module")] public ActiveModuleStatus? ActiveModule { get; init; }
    [JsonPropertyName("installed_modules")] public IList<InstalledModule> InstalledModules { get; init; } = [];
}

public sealed class ActiveModuleStatus
{
    [JsonPropertyName("uuid")] public string Uuid { get; init; } = "";
    [JsonPropertyName("name")] public string Name { get; init; } = "";
    [JsonPropertyName("vendor")] public string Vendor { get; init; } = "";
    [JsonPropertyName("version")] public string Version { get; init; } = "";
}

public sealed class InstalledModule
{
    [JsonPropertyName("uuid")] public string Uuid { get; init; } = "";
    [JsonPropertyName("name")] public string Name { get; init; } = "";
    [JsonPropertyName("vendor")] public string Vendor { get; init; } = "";
    [JsonPropertyName("version")] public string Version { get; init; } = "";
}

// Source generator context for AOT-safe JSON serialization. Even though the host
// is currently JIT-only, declaring this up front lets a future AOT publish reuse
// it without surprises.
[JsonSourceGenerationOptions(
    WriteIndented = true,
    DefaultIgnoreCondition = JsonIgnoreCondition.WhenWritingNull)]
[JsonSerializable(typeof(HostStatus))]
internal partial class HostStatusJsonContext : JsonSerializerContext { }
