using System.ComponentModel;
using System.Diagnostics;
using System.Numerics;
using System.Text;
using System.Text.Json;
using System.Text.Json.Serialization;
using VRCFaceTracking.Core.Library;
using VRCFaceTracking.Core.Sandboxing;
using VRCFaceTracking.Core.Sandboxing.IPC;
using WKOpenVR.FaceModuleHost.Logging;
using WKOpenVR.FaceTracking.Registry;

namespace WKOpenVR.FaceModuleHost.Workers;

public sealed record HostRuntimeStatus(
    string Phase,
    string LastError,
    long FramesWritten,
    long FramesWithData,
    int LastExitCode,
    DateTime? LastRestartTime);

/// <summary>
/// Module loader using upstream's subprocess-per-module architecture.
/// Spawns one <c>WKOpenVR.FaceModuleProcess.exe</c> at a time; tears the
/// old one down before starting a new one when <see cref="SelectActive"/>
/// is called.
/// </summary>
public sealed class SubprocessManager : IDisposable
{
    private const float InvalidFloatSentinel = unchecked((float)0xFFFFFFFF);
    private const float LargeInvalidSignalMin = 1_000_000.0f;

    // bridge.json fields we read to resolve the upstream DLL path.
    private sealed class BridgeConfig
    {
        [JsonPropertyName("upstream_assembly")] public required string UpstreamAssembly { get; init; }
    }

    private sealed class ActiveSubprocess : IDisposable
    {
        public Process Process { get; init; } = null!;
        public int Port { get; init; }
        public DiscoveredModule Module { get; init; } = null!;
        public bool Exited { get; private set; }
        public DateTime SpawnTime { get; init; } = DateTime.UtcNow;

        public void MarkExited()
        {
            Exited = true;
        }

        public bool TerminateProcessTree()
        {
            try
            {
                if (!Process.HasExited)
                {
                    Process.Kill(entireProcessTree: true);
                    if (!Process.WaitForExit(2000))
                    {
                        using var killer = Process.Start(new ProcessStartInfo
                        {
                            FileName = "taskkill",
                            Arguments = $"/F /T /PID {Process.Id}",
                            CreateNoWindow = true,
                            UseShellExecute = false,
                        });
                        killer?.WaitForExit(2000);
                    }
                }

                if (Process.HasExited)
                {
                    MarkExited();
                }

                return Process.HasExited;
            }
            catch
            {
                return false;
            }
        }

        public void Dispose()
        {
            TerminateProcessTree();
            Process.Dispose();
        }
    }

    private static readonly JsonSerializerOptions JsonOpts = new(JsonSerializerDefaults.Web);

    private const string VrcftCompatAdapterType =
        "WKOpenVR.FaceTracking.VrcftCompat.ReflectingExtTrackingModuleAdapter";
    private const string LegacyOpenVrPairVrcftCompatAdapterType =
        "OpenVRPair.FaceTracking.VrcftCompat.ReflectingExtTrackingModuleAdapter";

    private readonly HostOptions _opts;
    private readonly HostLogger _logger;
    private readonly VrcftSandboxServer _server;
    private readonly string _subprocessExePath;

    private readonly List<DiscoveredModule> _loaded = [];
    private ActiveSubprocess? _activeProcess;
    private readonly Lock _statusLock = new();
    private string _phase = "starting";
    private string _lastError = "";
    private long _framesWritten;
    private long _framesWithData;
    private int _lastExitCode;
    private DateTime? _lastRestartTime;

    // Host activity classification mirrored into the shmem header so the
    // driver-side supervisor can interpret heartbeat-age. Updated in
    // SetPhase; read by the heartbeat-publishing loop in Program.cs.
    // Values match FrameWriter.HostState* constants.
    private int _hostState = (int)FrameWriter.HostStateLegacy;
    public uint CurrentHostState => (uint)Volatile.Read(ref _hostState);
    public void SetHostStateDraining()
    {
        Volatile.Write(ref _hostState, (int)FrameWriter.HostStateDraining);
    }

    // Serializes SelectActive calls: teardown must finish before next spawn.
    private readonly SemaphoreSlim _selectLock = new(1, 1);

    // Circuit breaker state: track consecutive fast-exit crashes per module uuid.
    private string? _breakerUuid;
    private int _breakerCount;
    private const int CrashHaltThreshold = 3;
    private static readonly TimeSpan FastExitThreshold = TimeSpan.FromSeconds(5);

    // Decode symbolic names mirroring WKOpenVR.FaceModuleProcess.ModuleProcessExitCodes.
    // Inlined (not a project reference) because the subprocess EXE is not a managed
    // reference of the host -- it's a runtime-spawned process.
    private static readonly IReadOnlyDictionary<int, string> ExitCodeNames =
        new Dictionary<int, string>
        {
            [0] = "OK",
            [-1] = "INVALID_ARGS",
            [-2] = "NETWORK_CONNECTION_TIMED_OUT",
            [-3] = "EXCEPTION_CRASH",
        };

    // Minimal logger factory for VrcftSandboxServer.
    private static readonly Microsoft.Extensions.Logging.ILoggerFactory _nullLoggerFactory =
        Microsoft.Extensions.Logging.Abstractions.NullLoggerFactory.Instance;

    public SubprocessManager(HostOptions opts, HostLogger logger)
    {
        _opts = opts;
        _logger = logger;
        _subprocessExePath = Path.Combine(
            Path.GetDirectoryName(typeof(SubprocessManager).Assembly.Location)!,
            "WKOpenVR.FaceModuleProcess.exe");

        _logger.Info($"[ftp/spawn] subprocess-exe-path={_subprocessExePath} " +
                     $"exists={File.Exists(_subprocessExePath)}");

        if (!File.Exists(_subprocessExePath))
        {
            throw new FileNotFoundException(
                $"[ftp/spawn] FATAL: subprocess exe not found at {_subprocessExePath}");
        }

        _server = new VrcftSandboxServer(_nullLoggerFactory, reservedPorts: []);
        _server.OnPacketReceived += OnServerPacket;
        _logger.Info($"[ftp/ipc] sandbox server listening on port={_server.Port}");
    }

    // -------------------------------------------------------------------------
    // Public surface
    // -------------------------------------------------------------------------

    public DiscoveredModule? Active { get; private set; }
    public IReadOnlyList<DiscoveredModule> Loaded => _loaded;

    public HostRuntimeStatus SnapshotStatus()
    {
        lock (_statusLock)
        {
            return new HostRuntimeStatus(
                _phase,
                _lastError,
                Interlocked.Read(ref _framesWritten),
                Interlocked.Read(ref _framesWithData),
                _lastExitCode,
                _lastRestartTime);
        }
    }

    public async Task<IReadOnlyList<DiscoveredModule>> LoadAllAsync()
    {
        SetPhase("discovering-modules");
        _loaded.Clear();
        if (!Directory.Exists(_opts.ModulesInstallDir))
        {
            _logger.Info("[ftp/spawn] module install directory does not exist; no modules loaded. " +
                         $"dir={_opts.ModulesInstallDir}");
            return _loaded;
        }

        _logger.Info($"[ftp/spawn] scanning module dir: {_opts.ModulesInstallDir}");
        foreach (string uuidDir in Directory.EnumerateDirectories(_opts.ModulesInstallDir))
        {
            string? versionDir = Directory.EnumerateDirectories(uuidDir)
                .OrderDescending()
                .FirstOrDefault();
            if (versionDir is null)
            {
                _logger.Info($"[ftp/spawn] skipping {Path.GetFileName(uuidDir)}: no version subdirectory");
                continue;
            }

            await TryDiscoverModuleAsync(versionDir);
        }

        var names = string.Join(", ", _loaded.Select(m => $"{m.Uuid[..8]}../{m.Manifest.Name}"));
        _logger.Info($"[ftp/spawn] discovery complete: {_loaded.Count} modules: {names}");
        SetPhase(_loaded.Count == 0 ? "no-modules" : "modules-loaded");
        return _loaded;
    }

    public async Task UnloadAllAsync()
    {
        await TeardownActiveAsync(ct: CancellationToken.None);
        _loaded.Clear();
        Active = null;
    }

    private static bool IsInvalidSignal(float value)
    {
        return !float.IsFinite(value)
        || value == InvalidFloatSentinel
        || value >= LargeInvalidSignalMin;
    }

    private static bool TryUnitSignal(float value, out float clamped, out bool wasClamped)
    {
        clamped = 0.0f;
        wasClamped = false;
        if (IsInvalidSignal(value))
        {
            return false;
        }

        clamped = Math.Clamp(value, 0.0f, 1.0f);
        wasClamped = clamped != value;
        return true;
    }

    private static bool TrySignedUnitSignal(float value, out float clamped, out bool wasClamped)
    {
        clamped = 0.0f;
        wasClamped = false;
        if (IsInvalidSignal(value))
        {
            return false;
        }

        clamped = Math.Clamp(value, -1.0f, 1.0f);
        wasClamped = clamped != value;
        return true;
    }

    private static bool TryPositiveSignal(float value, out float sanitized)
    {
        sanitized = 0.0f;
        if (IsInvalidSignal(value) || value <= 0.0f || value > 100.0f)
        {
            return false;
        }

        sanitized = value;
        return true;
    }

    private static bool TryFiniteSignal(float value, out float sanitized)
    {
        sanitized = 0.0f;
        if (IsInvalidSignal(value))
        {
            return false;
        }

        sanitized = value;
        return true;
    }

    private static Vector3 GazeDirection(float x, float y)
    {
        Vector3 dir = new(x, y, -1.0f);
        float lenSq = dir.LengthSquared();
        return lenSq > 0.000001f
            ? Vector3.Normalize(dir)
            : new Vector3(0.0f, 0.0f, -1.0f);
    }

    private static bool SameModuleIdentity(DiscoveredModule a, DiscoveredModule b)
    {
        return string.Equals(a.Uuid, b.Uuid, StringComparison.Ordinal)
        && string.Equals(a.ModuleDllPath, b.ModuleDllPath, StringComparison.OrdinalIgnoreCase)
        && string.Equals(a.Manifest.Version, b.Manifest.Version, StringComparison.Ordinal)
        && a.ModuleLastWriteUtc == b.ModuleLastWriteUtc
        && a.ModuleFileSize == b.ModuleFileSize;
    }

    public void SelectActive(string uuid)
    {
        uuid ??= "";
        _logger.Info($"[ftp/spawn] SelectActive(uuid={uuid}) -- current active: {Active?.Uuid ?? "none"}");

        if (string.IsNullOrWhiteSpace(uuid))
        {
            if (Active is not null)
            {
                _logger.Info($"[ftp/spawn] disabling active module {Active.Manifest.Name} ({Active.Uuid})");
            }

            Active = null;
            _breakerUuid = null;
            _breakerCount = 0;
            SetPhase(_loaded.Count == 0 ? "no-modules" : "module-disabled");
            return;
        }

        // Module installs/updates can happen while the host is already running.
        // Rescan on every explicit selection so a newly installed UUID is
        // immediately visible, and so re-selecting an updated module swaps to
        // the freshly discovered path and restarts the subprocess loop.
        try
        {
            LoadAllAsync().GetAwaiter().GetResult();
        }
        catch (Exception ex)
        {
            _logger.Warn($"[ftp/spawn] module rescan failed before selection: {ex.Message}");
        }

        DiscoveredModule? previous = Active;
        DiscoveredModule? m = _loaded.FirstOrDefault(m => m.Uuid == uuid);
        if (m is null)
        {
            string msg = $"SelectActive: module {uuid} not found in loaded list " +
                         $"(loaded={string.Join(",", _loaded.Select(x => x.Uuid[..8]))})";
            if (Active?.Uuid == uuid)
            {
                Active = null;
            }

            SetPhase("module-select-failed", msg);
            _logger.Warn($"[ftp/spawn] {msg}");
            return;
        }
        if (previous is not null && SameModuleIdentity(previous, m))
        {
            SetPhase("module-selected");
            _logger.Info($"[ftp/spawn] active module unchanged: {m.Manifest.Name} v{m.Manifest.Version} ({uuid})");
            return;
        }
        Active = m;
        if (_breakerUuid == uuid)
        {
            _breakerUuid = null;
            _breakerCount = 0;
        }
        SetPhase("module-selected");
        _logger.Info($"[ftp/spawn] active module set to {m.Manifest.Name} v{m.Manifest.Version} ({uuid})");
    }

    /// <summary>
    /// Pull loop: spawns the active subprocess, drives EventUpdate at ~120 Hz,
    /// decodes ReplyUpdate, maps shapes, calls FrameWriter.PublishAsync.
    /// </summary>
    public async Task RunActiveAsync(
        FrameWriter writer,
        CancellationToken ct)
    {
        _logger.Info("[ftp/data] RunActiveAsync started");
        try
        {
            while (!ct.IsCancellationRequested)
            {
                if (Active is null)
                {
                    SetPhase(_loaded.Count == 0 ? "no-modules" : "waiting-for-module-selection");
                    await Task.Delay(250, ct);
                    continue;
                }

                DiscoveredModule module = Active;

                // Circuit-breaker: stop respawning if this module keeps crashing fast.
                if (_breakerUuid == module.Uuid && _breakerCount >= CrashHaltThreshold)
                {
                    _logger.Error($"[ftp/spawn] CIRCUIT BREAKER: halting respawn for {module.Uuid} " +
                                  $"({module.Manifest.Name}); {CrashHaltThreshold} consecutive fast exits");
                    SetPhase("module-circuit-breaker", $"Module {module.Manifest.Name} exited too quickly.");
                    Active = null;
                    continue;
                }

                await SpawnAndRunAsync(module, writer, ct);
            }
        }
        finally
        {
            _logger.Info("[ftp/data] RunActiveAsync stopped");
        }
    }

    public void Dispose()
    {
        if (_activeProcess is not null)
        {
            if (!_activeProcess.Exited && !_activeProcess.Process.HasExited)
            {
                _logger.Warn($"[ftp/teardown] disposing active subprocess PID={_activeProcess.Process.Id}; forcing process tree kill");
            }

            if (!_activeProcess.TerminateProcessTree())
            {
                _logger.Error($"[ftp/teardown] process tree kill did not confirm exit for PID={_activeProcess.Process.Id}");
            }

            _activeProcess.Process.Dispose();
        }
        _activeProcess = null;
        _server.Dispose();
        _selectLock.Dispose();
    }

    // -------------------------------------------------------------------------
    // Internal: discovery
    // -------------------------------------------------------------------------

    private async Task TryDiscoverModuleAsync(string dir)
    {
        string manifestPath = Path.Combine(dir, "manifest.json");
        if (!File.Exists(manifestPath))
        {
            _logger.Info($"[ftp/spawn] skipping {dir}: no manifest.json");
            return;
        }

        Manifest manifest;
        try
        {
            await using FileStream f = File.OpenRead(manifestPath);
            manifest = await JsonSerializer.DeserializeAsync<Manifest>(f, JsonOpts)
                ?? throw new InvalidDataException("Null manifest.");
        }
        catch (Exception ex)
        {
            _logger.Warn($"[ftp/spawn] skipping {dir}: manifest parse failed " +
                         $"({ex.GetType().Name}: {ex.Message})");
            return;
        }

        // Resolve the upstream module DLL path. Legacy registry payloads may
        // still carry the old compat adapter type, but the adapter is not
        // loaded; bridge.json only points us at the real upstream module DLL.
        string moduleDllPath;
        if (IsVrcftCompatAdapter(manifest.EntryType))
        {
            string bridgePath = Path.Combine(dir, "assemblies", "bridge.json");
            if (!File.Exists(bridgePath))
            {
                _logger.Warn($"[ftp/spawn] skipping {manifest.Name}: bridge.json not found at {bridgePath}");
                return;
            }
            try
            {
                await using FileStream f = File.OpenRead(bridgePath);
                BridgeConfig cfg = await JsonSerializer.DeserializeAsync<BridgeConfig>(f, JsonOpts)
                    ?? throw new InvalidDataException("Null bridge config.");
                moduleDllPath = Path.Combine(dir, "assemblies", cfg.UpstreamAssembly);
                _logger.Info($"[ftp/spawn] module dir {Path.GetFileName(Path.GetDirectoryName(dir))}/{Path.GetFileName(dir)}: " +
                             $"bridge.json -> upstream_assembly={cfg.UpstreamAssembly} -> {moduleDllPath}");
            }
            catch (Exception ex)
            {
                _logger.Warn($"[ftp/spawn] skipping {manifest.Name}: bridge.json parse failed " +
                             $"({ex.GetType().Name}: {ex.Message})");
                return;
            }
        }
        else
        {
            moduleDllPath = Path.Combine(dir, "assemblies", manifest.EntryAssembly);
            _logger.Info($"[ftp/spawn] module dir {Path.GetFileName(Path.GetDirectoryName(dir))}/{Path.GetFileName(dir)}: " +
                         $"manifest entry_type={manifest.EntryType} entry_assembly={manifest.EntryAssembly}");
        }

        if (!File.Exists(moduleDllPath))
        {
            _logger.Warn($"[ftp/spawn] skipping {manifest.Name}: DLL not found at {moduleDllPath}");
            return;
        }

        var moduleFile = new FileInfo(moduleDllPath);
        ulong hash = Fnv1a64(manifest.Uuid);
        _loaded.Add(new DiscoveredModule(
            manifest.Uuid,
            manifest,
            moduleDllPath,
            hash,
            moduleFile.LastWriteTimeUtc,
            moduleFile.Length));
        _logger.Info($"[ftp/spawn] discovered {manifest.Name} v{manifest.Version} " +
                      $"dll={Path.GetFileName(moduleDllPath)} uuid_hash=0x{hash:X16}");
    }

    private static bool IsVrcftCompatAdapter(string entryType)
    {
        return string.Equals(entryType, VrcftCompatAdapterType, StringComparison.Ordinal) ||
        string.Equals(entryType, LegacyOpenVrPairVrcftCompatAdapterType, StringComparison.Ordinal);
    }

    // -------------------------------------------------------------------------
    // Internal: subprocess lifecycle
    // -------------------------------------------------------------------------

    private TaskCompletionSource<bool> _handshakeTcs =
        new(TaskCreationOptions.RunContinuationsAsynchronously);

    private TaskCompletionSource<ReplySupportedPacket> _supportedTcs =
        new(TaskCreationOptions.RunContinuationsAsynchronously);

    private TaskCompletionSource<ReplyInitPacket> _initTcs =
        new(TaskCreationOptions.RunContinuationsAsynchronously);

    private TaskCompletionSource<ReplyTeardownPacket> _teardownTcs =
        new(TaskCreationOptions.RunContinuationsAsynchronously);

    private ReplyUpdatePacket? _latestUpdate;
    private int _activePort;
    private long _replyUpdatesReceived;
    private long _sendDroppedNoPort;
    private DateTime _lastNoPortSendLog = DateTime.MinValue;
    private string _lastLoggedPhase = "";

    private void OnServerPacket(in IpcPacket packet, in int port)
    {
        switch (packet.GetPacketType())
        {
            case IpcPacket.PacketType.Handshake:
                _logger.Info($"[ftp/ipc] RECV Handshake from port={port}");
                _activePort = port;
                _handshakeTcs.TrySetResult(true);
                break;

            case IpcPacket.PacketType.ReplyGetSupported:
                var supported = (ReplySupportedPacket)packet;
                _logger.Info($"[ftp/ipc] RECV ReplyGetSupported port={port} eye={supported.eyeAvailable} expr={supported.expressionAvailable}");
                _supportedTcs.TrySetResult(supported);
                break;

            case IpcPacket.PacketType.ReplyInit:
                var init = (ReplyInitPacket)packet;
                _logger.Info($"[ftp/ipc] RECV ReplyInit port={port} eye={init.eyeSuccess} expr={init.expressionSuccess} name={init.ModuleInformationName}");
                _initTcs.TrySetResult(init);
                break;

            case IpcPacket.PacketType.ReplyUpdate:
                // High-frequency (120 Hz): do not log; just snapshot the latest.
                _latestUpdate = (ReplyUpdatePacket)packet;
                Interlocked.Increment(ref _replyUpdatesReceived);
                break;

            case IpcPacket.PacketType.ReplyTeardown:
                _logger.Info($"[ftp/ipc] RECV ReplyTeardown port={port}");
                _teardownTcs.TrySetResult((ReplyTeardownPacket)packet);
                break;

            case IpcPacket.PacketType.EventLog:
                var log = (EventLogPacket)packet;
                _logger.Info($"[ftp/ipc] subprocess-log port={port}: {log.Message}");
                break;

            default:
                _logger.Info($"[ftp/ipc] RECV unknown packet type={packet.GetPacketType()} port={port}");
                break;
        }
    }

    private async Task SpawnAndRunAsync(
        DiscoveredModule module,
        FrameWriter writer,
        CancellationToken ct)
    {
        // Serialize concurrent SelectActive/spawn calls so teardown always completes first.
        await _selectLock.WaitAsync(ct);
        try
        {
            await SpawnAndRunCoreAsync(module, writer, ct);
        }
        finally
        {
            _selectLock.Release();
        }
    }

    private async Task SpawnAndRunCoreAsync(
        DiscoveredModule module,
        FrameWriter writer,
        CancellationToken ct)
    {
        // Reset per-run TCS objects.
        _handshakeTcs = new(TaskCreationOptions.RunContinuationsAsynchronously);
        _supportedTcs = new(TaskCreationOptions.RunContinuationsAsynchronously);
        _initTcs = new(TaskCreationOptions.RunContinuationsAsynchronously);
        _teardownTcs = new(TaskCreationOptions.RunContinuationsAsynchronously);
        _latestUpdate = null;
        _activePort = 0;
        Interlocked.Exchange(ref _replyUpdatesReceived, 0);
        Interlocked.Exchange(ref _sendDroppedNoPort, 0);

        // Re-verify exe on disk just before spawn (could have been deleted after startup check).
        if (!File.Exists(_subprocessExePath))
        {
            string msg = $"subprocess exe missing at spawn time: {_subprocessExePath}";
            SetPhase("subprocess-missing", msg);
            _logger.Error($"[ftp/spawn] FATAL: {msg}");
            return;
        }

        int serverPort = _server.Port;
        string argv = $"--port {serverPort} --module-path \"{module.ModuleDllPath}\" --parent-pid {Environment.ProcessId}";
        SetPhase("spawning-module-process");
        _logger.Info($"[ftp/spawn] exec: {_subprocessExePath} {argv}");

        var psi = new ProcessStartInfo(_subprocessExePath)
        {
            UseShellExecute = false,
            CreateNoWindow = true,
            Arguments = argv,
        };

        var spawnSw = Stopwatch.StartNew();
        Process proc;
        try
        {
            proc = Process.Start(psi)
                ?? throw new InvalidOperationException("Process.Start returned null.");
        }
        catch (Win32Exception ex)
        {
            SetPhase("spawn-failed", $"Win32Exception({ex.NativeErrorCode}) {ex.Message}");
            _logger.Error($"[ftp/spawn] Process.Start failed: Win32Exception({ex.NativeErrorCode}) {ex.Message}");
            return;
        }
        catch (Exception ex)
        {
            SetPhase("spawn-failed", $"{ex.GetType().Name} {ex.Message}");
            _logger.Error($"[ftp/spawn] Process.Start failed: {ex.GetType().Name} {ex.Message}");
            return;
        }
        spawnSw.Stop();

        _logger.Info($"[ftp/spawn] spawned PID={proc.Id} in {spawnSw.ElapsedMilliseconds}ms");

        DateTime spawnTime = DateTime.UtcNow;
        var active = new ActiveSubprocess
        {
            Process = proc,
            Port = serverPort,
            Module = module,
            SpawnTime = spawnTime,
        };
        _activeProcess = active;

        proc.EnableRaisingEvents = true;
        proc.Exited += (_, _) =>
        {
            active.MarkExited();
            int raw = TryGetExitCode(proc);
            RecordExit(raw);
            string sym = ExitCodeNames.TryGetValue(raw, out var name) ? name : $"0x{(uint)raw:X8}";
            _logger.Info($"[ftp/spawn] PID={proc.Id} exited code={raw} ({sym}) " +
                         $"uptime={(DateTime.UtcNow - spawnTime).TotalSeconds:F1}s");
        };

        var selectSw = Stopwatch.StartNew();
        ReplyInitPacket? initReply = null;

        try
        {
            // Handshake -- up to 30 s to connect.
            using var handshakeCts = CancellationTokenSource.CreateLinkedTokenSource(ct);
            handshakeCts.CancelAfter(TimeSpan.FromSeconds(30));
            try
            {
                _logger.Info($"[ftp/ipc] waiting for Handshake from PID={proc.Id} (timeout=30s)");
                SetPhase("waiting-for-module-handshake");
                await _handshakeTcs.Task.WaitAsync(handshakeCts.Token);
            }
            catch (OperationCanceledException) when (!ct.IsCancellationRequested)
            {
                string msg = $"TIMEOUT waiting for Handshake from PID={proc.Id} after 30s; subprocess alive={!active.Exited}";
                SetPhase("module-handshake-timeout", msg);
                _logger.Error($"[ftp/ipc] {msg}");
                return;
            }

            // GetSupported
            _logger.Info($"[ftp/ipc] SEND EventInitGetSupported -> port={_activePort}");
            SetPhase("querying-module-capabilities");
            SendToSubprocess(new EventInitGetSupported());
            using var supportedCts = CancellationTokenSource.CreateLinkedTokenSource(ct);
            supportedCts.CancelAfter(TimeSpan.FromSeconds(30));
            ReplySupportedPacket supportedReply;
            try
            {
                supportedReply = await _supportedTcs.Task.WaitAsync(supportedCts.Token);
            }
            catch (OperationCanceledException) when (!ct.IsCancellationRequested)
            {
                string msg = $"TIMEOUT waiting for ReplyGetSupported from PID={proc.Id} after 30s; subprocess alive={!active.Exited}";
                SetPhase("module-capabilities-timeout", msg);
                _logger.Error($"[ftp/ipc] {msg}");
                return;
            }
            if (!supportedReply.eyeAvailable && !supportedReply.expressionAvailable)
            {
                _logger.Warn($"[ftp/ipc] module reports no supported streams: module={module.Manifest.Name} pid={proc.Id}");
            }

            // Init
            _logger.Info($"[ftp/ipc] SEND EventInit(eye=true,expr=true) -> port={_activePort}");
            SetPhase("initializing-module");
            SendToSubprocess(new EventInitPacket { eyeAvailable = true, expressionAvailable = true });
            using var initCts = CancellationTokenSource.CreateLinkedTokenSource(ct);
            initCts.CancelAfter(TimeSpan.FromSeconds(30));
            try
            {
                initReply = await _initTcs.Task.WaitAsync(initCts.Token);
            }
            catch (OperationCanceledException) when (!ct.IsCancellationRequested)
            {
                string msg = $"TIMEOUT waiting for ReplyInit from PID={proc.Id} after 30s; " +
                             $"subprocess alive={!active.Exited}; supported eye={supportedReply.eyeAvailable} expr={supportedReply.expressionAvailable}";
                SetPhase("module-init-timeout", msg);
                _logger.Error($"[ftp/ipc] {msg}");
                return;
            }
            if (!initReply.eyeSuccess && !initReply.expressionSuccess)
            {
                _logger.Warn($"[ftp/ipc] module init returned no active streams: module={module.Manifest.Name} " +
                             $"supported_eye={supportedReply.eyeAvailable} supported_expr={supportedReply.expressionAvailable}");
            }

            // Tell module to start sampling hardware.
            _logger.Info($"[ftp/ipc] SEND EventStatusUpdate(Active) -> port={_activePort}");
            SetPhase("module-active");
            SendToSubprocess(new EventStatusUpdatePacket { ModuleState = ModuleState.Active });

            selectSw.Stop();
            _logger.Info($"[ftp/spawn] SelectActive done in {selectSw.ElapsedMilliseconds}ms; " +
                         $"module={module.Manifest.Name} eye={initReply.eyeSuccess} expr={initReply.expressionSuccess}");

            // Pull loop at ~120 Hz. Expression shapes flow as the dense
            // upstream UnifiedExpressions array straight through to the
            // wire; the driver remaps to our 63-slot ordering on read.
            float[] upstreamShapes = new float[FrameWriter.UpstreamShapeCount];
            var eyeSink = new EyeFrameSink();
            var headSink = new HeadFrameSink();
            var diag = new PullLoopDiagnostics(
                _logger,
                module.Manifest.Name,
                proc.Id,
                upstreamShapes.Length,
                Interlocked.Read(ref _replyUpdatesReceived));

            _logger.Info($"[ftp/data] pull loop started for {module.Manifest.Name} PID={proc.Id}");
            SetPhase("publishing-frames");
            DateTime nextModuleDiskCheck = DateTime.UtcNow + TimeSpan.FromSeconds(1);

            while (!ct.IsCancellationRequested && !active.Exited &&
                   ReferenceEquals(Active, module))
            {
                if (DateTime.UtcNow >= nextModuleDiskCheck)
                {
                    nextModuleDiskCheck = DateTime.UtcNow + TimeSpan.FromSeconds(1);
                    if (!File.Exists(module.ModuleDllPath))
                    {
                        string msg = $"active module disappeared from disk: {module.ModuleDllPath}";
                        _logger.Warn($"[ftp/spawn] {msg}");
                        if (ReferenceEquals(Active, module))
                        {
                            Active = null;
                        }

                        SetPhase("module-uninstalled", msg);
                        break;
                    }
                }

                SendToSubprocess(new EventUpdatePacket());

                // Brief yield to let the server's async receive thread deliver the reply.
                await Task.Delay(8, ct); // ~120 Hz

                ReplyUpdatePacket? snap = _latestUpdate;
                if (snap is null)
                {
                    // Warn if no update has arrived for >2s while subprocess is still alive.
                    DateTime noReplyNow = DateTime.UtcNow;
                    if (diag.ShouldWarnNoReply(noReplyNow) && !active.Exited)
                    {
                        _logger.Warn($"[ftp/data] no ReplyUpdate from subprocess for >2s; " +
                                     $"subprocess alive={!active.Exited} pid={proc.Id}");
                        diag.MarkNoReplyWarning(noReplyNow);
                    }
                    continue;
                }
                diag.RecordReply(snap, DateTime.UtcNow);

                ReplyUpdatePacket.UpdateDataContiguous decoded = snap.DecodedData;
                float[]? shapes = decoded.GetExpressionShapes();
                int validExprSignals = 0;
                int invalidExprSignals = 0;
                int clampedSignals = 0;
                if (shapes is not null)
                {
                    int n = Math.Min(shapes.Length, upstreamShapes.Length);
                    for (int i = 0; i < n; i++)
                    {
                        if (TryUnitSignal(shapes[i], out float value, out bool wasClamped))
                        {
                            upstreamShapes[i] = value;
                            validExprSignals++;
                            if (wasClamped)
                            {
                                clampedSignals++;
                            }
                        }
                        else
                        {
                            invalidExprSignals++;
                        }
                    }
                }

                // Eye data.
                int validEyeSignals = 0;
                int invalidEyeSignals = 0;
                float leftOpen = decoded.GetEyeLeftOpenness();
                if (TryUnitSignal(leftOpen, out float leftOpenValue, out bool leftOpenClamped))
                {
                    eyeSink.LeftOpenness = leftOpenValue;
                    eyeSink.Left.Confidence = 1.0f;
                    validEyeSignals++;
                    if (leftOpenClamped)
                    {
                        clampedSignals++;
                    }
                }
                else
                {
                    invalidEyeSignals++;
                }
                float rightOpen = decoded.GetEyeRightOpenness();
                if (TryUnitSignal(rightOpen, out float rightOpenValue, out bool rightOpenClamped))
                {
                    eyeSink.RightOpenness = rightOpenValue;
                    eyeSink.Right.Confidence = 1.0f;
                    validEyeSignals++;
                    if (rightOpenClamped)
                    {
                        clampedSignals++;
                    }
                }
                else
                {
                    invalidEyeSignals++;
                }

                bool leftGazeXValid = TrySignedUnitSignal(
                    decoded.GetEyeLeftGazeX(),
                    out float leftGazeX,
                    out bool leftGazeXClamped);
                bool leftGazeYValid = TrySignedUnitSignal(
                    decoded.GetEyeLeftGazeY(),
                    out float leftGazeY,
                    out bool leftGazeYClamped);
                bool leftGazeValid = leftGazeXValid && leftGazeYValid;
                if (leftGazeValid)
                {
                    eyeSink.Left.DirHmd = GazeDirection(leftGazeX, leftGazeY);
                    eyeSink.Left.Confidence = 1.0f;
                    validEyeSignals += 2;
                    if (leftGazeXClamped || leftGazeYClamped)
                    {
                        clampedSignals++;
                    }
                }
                else
                {
                    invalidEyeSignals += 2;
                }

                bool rightGazeXValid = TrySignedUnitSignal(
                    decoded.GetEyeRightGazeX(),
                    out float rightGazeX,
                    out bool rightGazeXClamped);
                bool rightGazeYValid = TrySignedUnitSignal(
                    decoded.GetEyeRightGazeY(),
                    out float rightGazeY,
                    out bool rightGazeYClamped);
                bool rightGazeValid = rightGazeXValid && rightGazeYValid;
                if (rightGazeValid)
                {
                    eyeSink.Right.DirHmd = GazeDirection(rightGazeX, rightGazeY);
                    eyeSink.Right.Confidence = 1.0f;
                    validEyeSignals += 2;
                    if (rightGazeXClamped || rightGazeYClamped)
                    {
                        clampedSignals++;
                    }
                }
                else
                {
                    invalidEyeSignals += 2;
                }

                bool haveMinDilation = TryPositiveSignal(decoded.GetEyeMinDilation(), out float minDilation);
                bool haveMaxDilation = TryPositiveSignal(decoded.GetEyeMaxDilation(), out float maxDilation);
                bool haveDilationRange = haveMinDilation && haveMaxDilation && maxDilation > minDilation;
                if (TryPositiveSignal(decoded.GetEyeLeftPupilMM(), out float leftPupilMm))
                {
                    if (haveDilationRange)
                    {
                        eyeSink.PupilDilationLeft = Math.Clamp(
                            (leftPupilMm - minDilation) / (maxDilation - minDilation),
                            0.0f,
                            1.0f);
                    }

                    validEyeSignals++;
                }
                else
                {
                    invalidEyeSignals++;
                }
                if (TryPositiveSignal(decoded.GetEyeRightPupilMM(), out float rightPupilMm))
                {
                    if (haveDilationRange)
                    {
                        eyeSink.PupilDilationRight = Math.Clamp(
                            (rightPupilMm - minDilation) / (maxDilation - minDilation),
                            0.0f,
                            1.0f);
                    }

                    validEyeSignals++;
                }
                else
                {
                    invalidEyeSignals++;
                }

                float headYaw = 0.0f;
                float headPitch = 0.0f;
                float headRoll = 0.0f;
                float headPosX = 0.0f;
                float headPosY = 0.0f;
                float headPosZ = 0.0f;
                bool headValid =
                    TryFiniteSignal(decoded.GetHeadYaw(), out headYaw) &&
                    TryFiniteSignal(decoded.GetHeadPitch(), out headPitch) &&
                    TryFiniteSignal(decoded.GetHeadRoll(), out headRoll) &&
                    TryFiniteSignal(decoded.GetHeadPosX(), out headPosX) &&
                    TryFiniteSignal(decoded.GetHeadPosY(), out headPosY) &&
                    TryFiniteSignal(decoded.GetHeadPosZ(), out headPosZ);
                if (headValid)
                {
                    headSink.Yaw = headYaw;
                    headSink.Pitch = headPitch;
                    headSink.Roll = headRoll;
                    headSink.PosX = headPosX;
                    headSink.PosY = headPosY;
                    headSink.PosZ = headPosZ;
                    headSink.IsValid = true;
                }
                else
                {
                    headSink.IsValid = false;
                }

                diag.RecordSignalStats(
                    validExprSignals,
                    invalidExprSignals,
                    validEyeSignals,
                    invalidEyeSignals,
                    clampedSignals);

                bool currentFrameHasData = diag.RecordFrame(
                    upstreamShapes,
                    eyeSink,
                    validExprSignals,
                    validEyeSignals);

                if (currentFrameHasData)
                {
                    Interlocked.Increment(ref _framesWithData);
                }

                diag.MaybeLogPeriod(
                    Interlocked.Read(ref _replyUpdatesReceived),
                    Interlocked.Read(ref _sendDroppedNoPort));

                await writer.PublishAsync(
                    eyeSink, headSink, upstreamShapes,
                    initReply.eyeSuccess && diag.SawEyeData,
                    initReply.expressionSuccess && diag.SawExpressionData,
                    module.UuidHash, ct);
                Interlocked.Increment(ref _framesWritten);
            }

            _logger.Info($"[ftp/data] pull loop ended for {module.Manifest.Name}: " +
                         $"ct={ct.IsCancellationRequested} exited={active.Exited} " +
                         $"moduleChanged={!ReferenceEquals(Active, module)}");
        }
        catch (OperationCanceledException)
        {
            // Normal shutdown.
        }
        catch (Exception ex)
        {
            SetPhase("module-run-error", $"{ex.GetType().Name} {ex.Message}");
            _logger.Error($"[ftp/spawn] error in run loop: {ex.GetType().Name} {ex.Message}");
        }
        finally
        {
            _logger.Info($"[ftp/teardown] starting teardown for PID={proc.Id} module={module.Manifest.Name}");
            await TeardownActiveAsync(ct: CancellationToken.None);

            // Circuit-breaker logic.
            TimeSpan lifetime = DateTime.UtcNow - spawnTime;
            if (lifetime < FastExitThreshold)
            {
                if (_breakerUuid == module.Uuid)
                {
                    _breakerCount++;
                }
                else
                {
                    _breakerUuid = module.Uuid;
                    _breakerCount = 1;
                }
                int raw = TryGetExitCode(proc);
                RecordExit(raw);
                string sym = ExitCodeNames.TryGetValue(raw, out var name) ? name : $"0x{(uint)raw:X8}";
                _logger.Warn($"[ftp/spawn] fast-exit #{_breakerCount} (consecutive={_breakerCount} of max={CrashHaltThreshold}) " +
                             $"module={module.Uuid} lifetime={lifetime.TotalSeconds:F1}s " +
                             $"exitCode={raw} ({sym})");
                if (_breakerCount >= CrashHaltThreshold)
                {
                    _logger.Error($"[ftp/spawn] CIRCUIT BREAKER: halting respawn for {module.Uuid} ({module.Manifest.Name}). " +
                                  $"Last exit code: {raw} ({sym})");
                    SetPhase("module-circuit-breaker", $"Last exit code: {raw} ({sym})");
                }
            }
            else
            {
                // Reset breaker on healthy lifetime.
                if (_breakerUuid == module.Uuid) { _breakerUuid = null; _breakerCount = 0; }
            }
        }
    }

    private sealed class PullLoopDiagnostics(
        HostLogger logger,
        string moduleName,
        int pid,
        int shapeCount,
        long replyUpdatesAtPeriodStart)
    {
        private const int UpstreamJawOpenIndex = 22;

        private readonly HostLogger _logger = logger;
        private readonly string _moduleName = moduleName;
        private readonly int _pid = pid;
        private readonly int _shapeCount = shapeCount;
        private readonly Stopwatch _period = Stopwatch.StartNew();

        private int _noDataRun;
        private int _zeroFramesInPeriod;
        private bool _firstNonZero;
        private bool _firstInvalidSignal;
        private long _framesInPeriod;
        private long _invalidExprInPeriod;
        private long _invalidEyeInPeriod;
        private long _clampedInPeriod;
        private long _staleReusesInPeriod;
        private long _replyUpdatesAtPeriodStart = replyUpdatesAtPeriodStart;
        private DateTime _lastUpdateTime = DateTime.UtcNow;
        private ReplyUpdatePacket? _lastProcessedUpdate;

        private float _lastJawOpen;
        private int _lastValidExprSignals;
        private int _lastValidEyeSignals;
        private int _lastNonZeroShapes;
        private float _lastLeftEyeLid;
        private float _lastRightEyeLid;
        private Vector3 _lastLeftGaze;
        private float _lastPupilLeft;
        private float _lastPupilRight;

        public bool SawExpressionData { get; private set; }
        public bool SawEyeData { get; private set; }
        public long FrameNumber { get; private set; }

        public bool ShouldWarnNoReply(DateTime now)
        {
            return (now - _lastUpdateTime).TotalSeconds > 2.0;
        }

        public void MarkNoReplyWarning(DateTime now)
        {
            _lastUpdateTime = now;
        }

        public void RecordReply(ReplyUpdatePacket packet, DateTime now)
        {
            _lastUpdateTime = now;
            if (ReferenceEquals(packet, _lastProcessedUpdate))
            {
                _staleReusesInPeriod++;
            }

            _lastProcessedUpdate = packet;
        }

        public void RecordSignalStats(
            int validExprSignals,
            int invalidExprSignals,
            int validEyeSignals,
            int invalidEyeSignals,
            int clampedSignals)
        {
            if (validExprSignals > 0)
            {
                SawExpressionData = true;
            }

            if (validEyeSignals > 0)
            {
                SawEyeData = true;
            }

            _invalidExprInPeriod += invalidExprSignals;
            _invalidEyeInPeriod += invalidEyeSignals;
            _clampedInPeriod += clampedSignals;

            if (!_firstInvalidSignal && (invalidExprSignals > 0 || invalidEyeSignals > 0))
            {
                _firstInvalidSignal = true;
                _logger.Warn($"[ftp/data] invalid upstream signal(s) ignored: frame={FrameNumber + 1} " +
                             $"expr_invalid={invalidExprSignals} eye_invalid={invalidEyeSignals} " +
                             $"valid_expr={validExprSignals} valid_eye={validEyeSignals}");
            }
        }

        public bool RecordFrame(
            float[] upstreamShapes,
            EyeFrameSink eyeSink,
            int validExprSignals,
            int validEyeSignals)
        {
            FrameNumber++;
            _framesInPeriod++;

            int nonZero = 0;
            foreach (float value in upstreamShapes)
            {
                if (value != 0f)
                {
                    nonZero++;
                }
            }

            float jawOpen = upstreamShapes.Length > UpstreamJawOpenIndex
                ? upstreamShapes[UpstreamJawOpenIndex]
                : 0f;

            _lastJawOpen = jawOpen;
            _lastValidExprSignals = validExprSignals;
            _lastValidEyeSignals = validEyeSignals;
            _lastNonZeroShapes = nonZero;
            _lastLeftEyeLid = eyeSink.LeftOpenness;
            _lastRightEyeLid = eyeSink.RightOpenness;
            _lastLeftGaze = eyeSink.Left.DirHmd;
            _lastPupilLeft = eyeSink.PupilDilationLeft;
            _lastPupilRight = eyeSink.PupilDilationRight;

            if (nonZero > 0 && !_firstNonZero)
            {
                _firstNonZero = true;
                _logger.Info($"[ftp/data] first non-zero shapes: frame={FrameNumber} " +
                             $"nonZeroShapes={nonZero}/{upstreamShapes.Length} validExprSignals={validExprSignals} " +
                             $"jawOpen={jawOpen:F3} leftEyeLid={eyeSink.LeftOpenness:F3}");
            }

            bool currentFrameHasData = nonZero > 0 || validEyeSignals > 0;
            if (!currentFrameHasData)
            {
                _noDataRun++;
                _zeroFramesInPeriod++;
                if (_noDataRun % 60 == 0)
                {
                    _logger.Warn($"[ftp/data] no-data for {_noDataRun} frames " +
                                 $"(module={_moduleName} pid={_pid})");
                }
            }
            else
            {
                _noDataRun = 0;
            }

            return currentFrameHasData;
        }

        public void MaybeLogPeriod(long replyUpdatesReceived, long noPortDrops)
        {
            if (_period.Elapsed.TotalSeconds < 5.0)
            {
                return;
            }

            long replyUpdatesInPeriod = replyUpdatesReceived - _replyUpdatesAtPeriodStart;
            _logger.Info($"[ftp/data] period: published {_framesInPeriod} frames in last " +
                         $"{_period.Elapsed.TotalSeconds:F1}s; lastJawOpen={_lastJawOpen:F3}; " +
                         $"consecutive_zero_frames={_zeroFramesInPeriod}; " +
                         $"invalid_expr={_invalidExprInPeriod}; invalid_eye={_invalidEyeInPeriod}; " +
                         $"clamped={_clampedInPeriod}; reply_updates={replyUpdatesInPeriod}; " +
                         $"stale_reuses={_staleReusesInPeriod}; no_port_drops_total={noPortDrops}; " +
                         $"last_nonzero={_lastNonZeroShapes}/{_shapeCount}; " +
                         $"last_valid_expr={_lastValidExprSignals}; last_valid_eye={_lastValidEyeSignals}; " +
                         $"last_eye=({_lastLeftEyeLid:F3},{_lastRightEyeLid:F3}); " +
                         $"last_gazeL=({_lastLeftGaze.X:F3},{_lastLeftGaze.Y:F3}); " +
                         $"last_pupil=({_lastPupilLeft:F3},{_lastPupilRight:F3})");

            _framesInPeriod = 0;
            _zeroFramesInPeriod = 0;
            _invalidExprInPeriod = 0;
            _invalidEyeInPeriod = 0;
            _clampedInPeriod = 0;
            _staleReusesInPeriod = 0;
            _replyUpdatesAtPeriodStart = replyUpdatesReceived;
            _period.Restart();
        }
    }

    private async Task TeardownActiveAsync(CancellationToken ct)
    {
        ActiveSubprocess? proc = _activeProcess;
        _activeProcess = null;
        if (proc is null)
        {
            return;
        }

        if (!proc.Exited && !proc.Process.HasExited)
        {
            _logger.Info($"[ftp/teardown] sending EventTeardown to PID={proc.Process.Id}");
            SendToSubprocess(new EventTeardownPacket());

            try
            {
                await _teardownTcs.Task.WaitAsync(TimeSpan.FromSeconds(5), ct);
                _logger.Info($"[ftp/teardown] ReplyTeardown received from PID={proc.Process.Id}");
            }
            catch (TimeoutException)
            {
                _logger.Warn($"[ftp/teardown] TIMEOUT waiting for ReplyTeardown from PID={proc.Process.Id}; killing");
            }
            catch (OperationCanceledException)
            {
                _logger.Warn($"[ftp/teardown] cancelled while waiting for ReplyTeardown; killing PID={proc.Process.Id}");
            }
        }
        else
        {
            _logger.Info($"[ftp/teardown] subprocess PID={proc.Process.Id} already exited; skipping EventTeardown");
        }

        if (!proc.Exited && !proc.Process.HasExited)
        {
            try
            {
                await proc.Process.WaitForExitAsync(CancellationToken.None)
                    .WaitAsync(TimeSpan.FromSeconds(3));
                proc.MarkExited();
            }
            catch (TimeoutException)
            {
                _logger.Warn($"[ftp/teardown] PID={proc.Process.Id} did not exit within 3s after teardown");
            }
            catch (Exception ex)
            {
                _logger.Warn($"[ftp/teardown] wait for PID={proc.Process.Id} failed: {ex.GetType().Name} {ex.Message}");
            }
        }

        if (!proc.Exited && !proc.Process.HasExited)
        {
            _logger.Warn($"[ftp/teardown] forcing process tree kill for PID={proc.Process.Id}");
            if (!proc.TerminateProcessTree())
            {
                _logger.Error($"[ftp/teardown] process tree kill did not confirm exit for PID={proc.Process.Id}");
            }
        }

        int code = TryGetExitCode(proc.Process);
        RecordExit(code);
        string sym = ExitCodeNames.TryGetValue(code, out var cname) ? cname : $"0x{(uint)code:X8}";
        _logger.Info($"[ftp/teardown] process PID={proc.Process.Id} gone; " +
                     $"exitCode={code} ({sym}) lifetime={(DateTime.UtcNow - proc.SpawnTime).TotalSeconds:F1}s");
        proc.Dispose();
    }

    private void SetPhase(string phase, string? error = null)
    {
        string previous;
        lock (_statusLock)
        {
            previous = _phase;
            _phase = phase;
            if (error is not null)
            {
                _lastError = error;
            }
            else if (!phase.EndsWith("failed", StringComparison.OrdinalIgnoreCase) &&
                     !phase.EndsWith("timeout", StringComparison.OrdinalIgnoreCase) &&
                     phase != "module-circuit-breaker")
            {
                _lastError = "";
            }
        }

        // Mirror the phase into the shmem heartbeat state byte so the
        // driver can decide whether stalled publish_index is a wedge or
        // legitimate idle. Keep this outside the status lock; Volatile.Write
        // is itself atomic and SetHostStateDraining races on the same field.
        uint hostState = MapPhaseToHostState(phase);
        Volatile.Write(ref _hostState, (int)hostState);
        if (!string.Equals(previous, phase, StringComparison.Ordinal) ||
            !string.Equals(_lastLoggedPhase, phase, StringComparison.Ordinal))
        {
            _lastLoggedPhase = phase;
            _logger.Info($"[host/status] phase {previous} -> {phase} host_state={hostState} " +
                         $"error={(string.IsNullOrEmpty(error) ? "" : error)}");
        }
    }

    /// <summary>
    /// Classify a SetPhase string into the four-value HostState enum the
    /// shmem heartbeat header uses. Any phase that means "actively pumping
    /// frames out to the driver" -> Publishing; everything else where the
    /// host is alive but quiet -> Idle. Failure phases stay Idle because
    /// the supervisor is already tracking failures via exit codes.
    /// </summary>
    private static uint MapPhaseToHostState(string phase)
    {
        return phase switch
        {
            "publishing-frames" => FrameWriter.HostStatePublishing,
            "module-active" => FrameWriter.HostStatePublishing,
            _ => FrameWriter.HostStateIdle,
        };
    }

    private void RecordExit(int exitCode)
    {
        lock (_statusLock)
        {
            _lastExitCode = exitCode;
            _lastRestartTime = DateTime.UtcNow;
        }
    }

    private void SendToSubprocess(IpcPacket packet)
    {
        if (_activePort == 0)
        {
            long dropped = Interlocked.Increment(ref _sendDroppedNoPort);
            IpcPacket.PacketType type = packet.GetPacketType();
            DateTime now = DateTime.UtcNow;
            if (type != IpcPacket.PacketType.EventUpdate ||
                (now - _lastNoPortSendLog) >= TimeSpan.FromSeconds(5))
            {
                _lastNoPortSendLog = now;
                _logger.Warn($"[ftp/ipc] drop SEND {type}: no active subprocess port total={dropped}");
            }
            return;
        }
        // EventUpdate is sent at ~120 Hz; log only non-update traffic to avoid flooding.
        if (packet.GetPacketType() != IpcPacket.PacketType.EventUpdate)
        {
            _logger.Info($"[ftp/ipc] SEND {packet.GetPacketType()} -> port={_activePort}");
        }

        _server.SendData(packet, _activePort);
    }

    private static int TryGetExitCode(Process p)
    {
        try { return p.ExitCode; } catch { return -1; }
    }

    private static ulong Fnv1a64(string s)
    {
        const ulong OffsetBasis = 14695981039346656037UL;
        const ulong Prime = 1099511628211UL;

        ulong hash = OffsetBasis;
        foreach (byte b in Encoding.UTF8.GetBytes(s))
        {
            hash ^= b;
            hash *= Prime;
        }
        return hash;
    }
}
