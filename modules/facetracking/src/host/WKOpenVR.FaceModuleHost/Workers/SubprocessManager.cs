using System.ComponentModel;
using System.Diagnostics;
using System.Text;
using System.Text.Json;
using System.Text.Json.Serialization;
using WKOpenVR.FaceModuleHost.Logging;
using WKOpenVR.FaceTracking.ModuleSdk;
using WKOpenVR.FaceTracking.Registry;
using VRCFaceTracking.Core.Library;
using VRCFaceTracking.Core.Sandboxing;
using VRCFaceTracking.Core.Sandboxing.IPC;

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

        public void MarkExited() => Exited = true;

        public void Dispose()
        {
            try { if (!Process.HasExited) Process.Kill(entireProcessTree: false); } catch { }
            Process.Dispose();
        }
    }

    private static readonly JsonSerializerOptions JsonOpts = new(JsonSerializerDefaults.Web);

    private const string VrcftCompatAdapterType =
        "WKOpenVR.FaceTracking.VrcftCompat.ReflectingExtTrackingModuleAdapter";
    private const string LegacyOpenVrPairVrcftCompatAdapterType =
        "OpenVRPair.FaceTracking.VrcftCompat.ReflectingExtTrackingModuleAdapter";

    private readonly HostOptions    _opts;
    private readonly HostLogger     _logger;
    private readonly VrcftSandboxServer _server;
    private readonly string         _subprocessExePath;

    private readonly List<DiscoveredModule> _loaded = [];
    private DiscoveredModule?   _activeModule;
    private ActiveSubprocess?   _activeProcess;
    private readonly object     _statusLock = new();
    private string              _phase = "starting";
    private string              _lastError = "";
    private long                _framesWritten;
    private long                _framesWithData;
    private int                 _lastExitCode;
    private DateTime?           _lastRestartTime;

    // Host activity classification mirrored into the shmem header so the
    // driver-side supervisor can interpret heartbeat-age. Updated in
    // SetPhase; read by the heartbeat-publishing loop in Program.cs.
    // Values match FrameWriter.HostState* constants.
    private int _hostState = (int)FrameWriter.HostStateLegacy;
    public uint CurrentHostState => (uint)Volatile.Read(ref _hostState);
    public void SetHostStateDraining() =>
        Volatile.Write(ref _hostState, (int)FrameWriter.HostStateDraining);

    // Serializes SelectActive calls: teardown must finish before next spawn.
    private readonly SemaphoreSlim _selectLock = new(1, 1);

    // Circuit breaker state: track consecutive fast-exit crashes per module uuid.
    private string? _breakerUuid;
    private int     _breakerCount;
    private const int CrashHaltThreshold = 3;
    private static readonly TimeSpan FastExitThreshold = TimeSpan.FromSeconds(5);

    // Decode symbolic names mirroring WKOpenVR.FaceModuleProcess.ModuleProcessExitCodes.
    // Inlined (not a project reference) because the subprocess EXE is not a managed
    // reference of the host -- it's a runtime-spawned process.
    private static readonly IReadOnlyDictionary<int, string> ExitCodeNames =
        new Dictionary<int, string>
        {
            [  0 ] = "OK",
            [ -1 ] = "INVALID_ARGS",
            [ -2 ] = "NETWORK_CONNECTION_TIMED_OUT",
            [ -3 ] = "EXCEPTION_CRASH",
        };

    // Minimal logger factory for VrcftSandboxServer.
    private static readonly Microsoft.Extensions.Logging.ILoggerFactory _nullLoggerFactory =
        Microsoft.Extensions.Logging.Abstractions.NullLoggerFactory.Instance;

    public SubprocessManager(HostOptions opts, HostLogger logger)
    {
        _opts   = opts;
        _logger = logger;
        _subprocessExePath = Path.Combine(
            Path.GetDirectoryName(typeof(SubprocessManager).Assembly.Location)!,
            "WKOpenVR.FaceModuleProcess.exe");

        _logger.Info($"[ftp/spawn] subprocess-exe-path={_subprocessExePath} " +
                     $"exists={File.Exists(_subprocessExePath)}");

        if (!File.Exists(_subprocessExePath))
            throw new FileNotFoundException(
                $"[ftp/spawn] FATAL: subprocess exe not found at {_subprocessExePath}");

        _server = new VrcftSandboxServer(_nullLoggerFactory, reservedPorts: []);
        _server.OnPacketReceived += OnServerPacket;
        _logger.Info($"[ftp/ipc] sandbox server listening on port={_server.Port}");
    }

    // -------------------------------------------------------------------------
    // Public surface
    // -------------------------------------------------------------------------

    public DiscoveredModule? Active => _activeModule;
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
        _activeModule = null;
    }

    public void SelectActive(string uuid)
    {
        uuid ??= "";
        _logger.Info($"[ftp/spawn] SelectActive(uuid={uuid}) -- current active: {_activeModule?.Uuid ?? "none"}");

        if (string.IsNullOrWhiteSpace(uuid))
        {
            if (_activeModule is not null)
                _logger.Info($"[ftp/spawn] disabling active module {_activeModule.Manifest.Name} ({_activeModule.Uuid})");
            _activeModule = null;
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

        var m = _loaded.FirstOrDefault(m => m.Uuid == uuid);
        if (m is null)
        {
            string msg = $"SelectActive: module {uuid} not found in loaded list " +
                         $"(loaded={string.Join(",", _loaded.Select(x => x.Uuid[..8]))})";
            if (_activeModule?.Uuid == uuid)
                _activeModule = null;
            SetPhase("module-select-failed", msg);
            _logger.Warn($"[ftp/spawn] {msg}");
            return;
        }
        _activeModule = m;
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
                if (_activeModule is null)
                {
                    SetPhase(_loaded.Count == 0 ? "no-modules" : "waiting-for-module-selection");
                    await Task.Delay(250, ct);
                    continue;
                }

                var module = _activeModule;

                // Circuit-breaker: stop respawning if this module keeps crashing fast.
                if (_breakerUuid == module.Uuid && _breakerCount >= CrashHaltThreshold)
                {
                    _logger.Error($"[ftp/spawn] CIRCUIT BREAKER: halting respawn for {module.Uuid} " +
                                  $"({module.Manifest.Name}); {CrashHaltThreshold} consecutive fast exits");
                    SetPhase("module-circuit-breaker", $"Module {module.Manifest.Name} exited too quickly.");
                    _activeModule = null;
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
        _activeProcess?.Dispose();
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
            await using var f = File.OpenRead(manifestPath);
            manifest = await JsonSerializer.DeserializeAsync<Manifest>(f, JsonOpts)
                ?? throw new InvalidDataException("Null manifest.");
        }
        catch (Exception ex)
        {
            _logger.Warn($"[ftp/spawn] skipping {dir}: manifest parse failed " +
                         $"({ex.GetType().Name}: {ex.Message})");
            return;
        }

        // Resolve the upstream module DLL path.
        // Legacy registry payloads may still carry the pre-rename OpenVRPair
        // adapter type. Both adapter names use bridge.json to point at the
        // real upstream module DLL.
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
                await using var f = File.OpenRead(bridgePath);
                var cfg = await JsonSerializer.DeserializeAsync<BridgeConfig>(f, JsonOpts)
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

        ulong hash = Fnv1a64(manifest.Uuid);
        _loaded.Add(new DiscoveredModule(manifest.Uuid, manifest, moduleDllPath, hash));
        _logger.Info($"[ftp/spawn] discovered {manifest.Name} v{manifest.Version} " +
                      $"dll={Path.GetFileName(moduleDllPath)} uuid_hash=0x{hash:X16}");
    }

    private static bool IsVrcftCompatAdapter(string entryType) =>
        string.Equals(entryType, VrcftCompatAdapterType, StringComparison.Ordinal) ||
        string.Equals(entryType, LegacyOpenVrPairVrcftCompatAdapterType, StringComparison.Ordinal);

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
        _handshakeTcs  = new(TaskCreationOptions.RunContinuationsAsynchronously);
        _supportedTcs  = new(TaskCreationOptions.RunContinuationsAsynchronously);
        _initTcs       = new(TaskCreationOptions.RunContinuationsAsynchronously);
        _teardownTcs   = new(TaskCreationOptions.RunContinuationsAsynchronously);
        _latestUpdate  = null;

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
            UseShellExecute   = false,
            CreateNoWindow    = true,
            Arguments         = argv,
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

        var spawnTime = DateTime.UtcNow;
        var active = new ActiveSubprocess
        {
            Process   = proc,
            Port      = serverPort,
            Module    = module,
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
            var eyeSink  = new EyeFrameSink();
            long frameNum         = 0;
            int  noDataRun        = 0;
            int  zeroRunFrames    = 0;
            bool firstNonZero     = false;
            var  dataPeriodSw     = Stopwatch.StartNew();
            long framesInPeriod   = 0;
            float lastJawOpen     = 0f;
            var  lastUpdateTime   = DateTime.UtcNow;

            _logger.Info($"[ftp/data] pull loop started for {module.Manifest.Name} PID={proc.Id}");
            SetPhase("publishing-frames");
            var nextModuleDiskCheck = DateTime.UtcNow + TimeSpan.FromSeconds(1);

            while (!ct.IsCancellationRequested && !active.Exited &&
                   ReferenceEquals(_activeModule, module))
            {
                if (DateTime.UtcNow >= nextModuleDiskCheck)
                {
                    nextModuleDiskCheck = DateTime.UtcNow + TimeSpan.FromSeconds(1);
                    if (!File.Exists(module.ModuleDllPath))
                    {
                        string msg = $"active module disappeared from disk: {module.ModuleDllPath}";
                        _logger.Warn($"[ftp/spawn] {msg}");
                        if (ReferenceEquals(_activeModule, module))
                            _activeModule = null;
                        SetPhase("module-uninstalled", msg);
                        break;
                    }
                }

                SendToSubprocess(new EventUpdatePacket());

                // Brief yield to let the server's async receive thread deliver the reply.
                await Task.Delay(8, ct); // ~120 Hz

                var snap = _latestUpdate;
                if (snap is null)
                {
                    // Warn if no update has arrived for >2s while subprocess is still alive.
                    if ((DateTime.UtcNow - lastUpdateTime).TotalSeconds > 2.0 && !active.Exited)
                    {
                        _logger.Warn($"[ftp/data] no ReplyUpdate from subprocess for >2s; " +
                                     $"subprocess alive={!active.Exited} pid={proc.Id}");
                        lastUpdateTime = DateTime.UtcNow; // reset to avoid log spam
                    }
                    continue;
                }
                lastUpdateTime = DateTime.UtcNow;

                var decoded = snap.DecodedData;
                float[]? shapes = decoded.GetExpressionShapes();
                Array.Clear(upstreamShapes);
                if (shapes is not null)
                {
                    int n = Math.Min(shapes.Length, upstreamShapes.Length);
                    Array.Copy(shapes, upstreamShapes, n);
                }

                // Eye data.
                const float kInvalid = unchecked((float)0xFFFFFFFF);
                float leftOpen = decoded.GetEyeLeftOpenness();
                if (leftOpen != kInvalid) eyeSink.LeftOpenness  = leftOpen;
                float rightOpen = decoded.GetEyeRightOpenness();
                if (rightOpen != kInvalid) eyeSink.RightOpenness = rightOpen;

                frameNum++;
                framesInPeriod++;
                // Telemetry runs against the upstream array directly now.
                // Upstream's JawOpen index is 22 (UnifiedExpressions.JawOpen).
                ReadOnlySpan<float> s = upstreamShapes;
                int nonZero = 0;
                foreach (float v in s) if (v != 0f) nonZero++;
                const int kUpstreamJawOpenIndex = 22;
                float jawOpen = s.Length > kUpstreamJawOpenIndex
                    ? s[kUpstreamJawOpenIndex]
                    : 0f;
                lastJawOpen = jawOpen;

                if (nonZero > 0 || (leftOpen is not 0f and not kInvalid))
                    Interlocked.Increment(ref _framesWithData);

                if (nonZero > 0 && !firstNonZero)
                {
                    firstNonZero = true;
                    _logger.Info($"[ftp/data] first non-zero shapes: frame={frameNum} " +
                                 $"nonZeroShapes={nonZero}/{s.Length} jawOpen={jawOpen:F3} " +
                                 $"leftEyeLid={eyeSink.LeftOpenness:F3}");
                }

                if (nonZero == 0 && leftOpen is 0f or kInvalid)
                {
                    noDataRun++;
                    zeroRunFrames++;
                    if (noDataRun % 60 == 0)
                        _logger.Warn($"[ftp/data] no-data for {noDataRun} frames " +
                                     $"(module={module.Manifest.Name} pid={proc.Id})");
                }
                else
                {
                    noDataRun = 0;
                }

                if (frameNum % 60 == 0)
                {
                    _logger.Info($"[ftp/data] heartbeat module={module.Manifest.Name} " +
                                 $"frame={frameNum} jawOpen={jawOpen:F3} " +
                                 $"leftEyeLid={eyeSink.LeftOpenness:F3} nonZeroShapes={nonZero}/{s.Length}");
                }

                // 5-second throughput report.
                if (dataPeriodSw.Elapsed.TotalSeconds >= 5.0)
                {
                    _logger.Info($"[ftp/data] period: published {framesInPeriod} frames in last " +
                                 $"{dataPeriodSw.Elapsed.TotalSeconds:F1}s; lastJawOpen={lastJawOpen:F3}; " +
                                 $"consecutive_zero_frames={zeroRunFrames}");
                    framesInPeriod = 0;
                    zeroRunFrames  = 0;
                    dataPeriodSw.Restart();
                }

                await writer.PublishAsync(
                    eyeSink, upstreamShapes,
                    initReply.eyeSuccess, initReply.expressionSuccess,
                    module.UuidHash, ct);
                Interlocked.Increment(ref _framesWritten);
            }

            _logger.Info($"[ftp/data] pull loop ended for {module.Manifest.Name}: " +
                         $"ct={ct.IsCancellationRequested} exited={active.Exited} " +
                         $"moduleChanged={!ReferenceEquals(_activeModule, module)}");
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
            var lifetime = DateTime.UtcNow - spawnTime;
            if (lifetime < FastExitThreshold)
            {
                if (_breakerUuid == module.Uuid)
                    _breakerCount++;
                else
                {
                    _breakerUuid  = module.Uuid;
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

    private async Task TeardownActiveAsync(CancellationToken ct)
    {
        var proc = _activeProcess;
        _activeProcess = null;
        if (proc is null) return;

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

        try
        {
            await proc.Process.WaitForExitAsync(CancellationToken.None)
                .WaitAsync(TimeSpan.FromSeconds(3));
        }
        catch { }

        int code = TryGetExitCode(proc.Process);
        RecordExit(code);
        string sym = ExitCodeNames.TryGetValue(code, out var cname) ? cname : $"0x{(uint)code:X8}";
        _logger.Info($"[ftp/teardown] process PID={proc.Process.Id} gone; " +
                     $"exitCode={code} ({sym}) lifetime={(DateTime.UtcNow - proc.SpawnTime).TotalSeconds:F1}s");
        proc.Dispose();
    }

    private void SetPhase(string phase, string? error = null)
    {
        lock (_statusLock)
        {
            _phase = phase;
            if (error is not null) _lastError = error;
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
        Volatile.Write(ref _hostState, (int)MapPhaseToHostState(phase));
    }

    /// <summary>
    /// Classify a SetPhase string into the four-value HostState enum the
    /// shmem heartbeat header uses. Any phase that means "actively pumping
    /// frames out to the driver" -> Publishing; everything else where the
    /// host is alive but quiet -> Idle. Failure phases stay Idle because
    /// the supervisor is already tracking failures via exit codes.
    /// </summary>
    private static uint MapPhaseToHostState(string phase) => phase switch
    {
        "publishing-frames" => FrameWriter.HostStatePublishing,
        "module-active"     => FrameWriter.HostStatePublishing,
        _                   => FrameWriter.HostStateIdle,
    };

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
        if (_activePort == 0) return;
        // EventUpdate is sent at ~120 Hz; log only non-update traffic to avoid flooding.
        if (packet.GetPacketType() != IpcPacket.PacketType.EventUpdate)
            _logger.Info($"[ftp/ipc] SEND {packet.GetPacketType()} -> port={_activePort}");
        _server.SendData(packet, _activePort);
    }

    private static int TryGetExitCode(Process p)
    {
        try { return p.ExitCode; } catch { return -1; }
    }

    private static ulong Fnv1a64(string s)
    {
        const ulong OffsetBasis = 14695981039346656037UL;
        const ulong Prime       = 1099511628211UL;

        ulong hash = OffsetBasis;
        foreach (byte b in Encoding.UTF8.GetBytes(s))
        {
            hash ^= b;
            hash *= Prime;
        }
        return hash;
    }
}
