using WKOpenVR.FaceModuleHost;
using WKOpenVR.FaceModuleHost.Logging;
using WKOpenVR.FaceModuleHost.Workers;
using WKOpenVR.FaceTracking.Registry;
using System.Numerics;

var opts = HostOptions.FromArgs(args);
var cts  = new CancellationTokenSource();
var ct   = cts.Token;

var logger = new HostLogger(logFilePath: opts.LogFilePath, forceEnabled: opts.DebugLoggingEnabled);
logger.Info("[startup] phase=logger-open");
logger.Info($"[startup] paths root={AppPaths.RootDir()} logs={AppPaths.LogsDir()} modules={opts.ModulesInstallDir} status={opts.StatusFilePath}");

AppDomain.CurrentDomain.UnhandledException += (s, e) => {
    try { logger.Error($"[crash] AppDomain.UnhandledException: {e.ExceptionObject}"); logger.Flush(); } catch { }
};
TaskScheduler.UnobservedTaskException += (s, e) => {
    try { logger.Error($"[crash] TaskScheduler.UnobservedTaskException: {e.Exception}"); logger.Flush(); e.SetObserved(); } catch { }
};
logger.Info("[startup] phase=crash-handlers-installed");

if (opts.E2eFakeFrames)
{
    int rc = await RunE2eFakeFramesAsync(opts, logger, ct);
    logger.Flush();
    logger.Dispose();
    return rc;
}

// Layer 1: system-wide singleton mutex. Acquired before opening any IPC so
// two host processes from overlapping SteamVR sessions cannot coexist.
var _sid = System.Security.Principal.WindowsIdentity.GetCurrent().User?.Value ?? "unknown";
var _mutexName = $@"Global\WKOpenVR-FaceModuleHost-Singleton-{_sid}";
bool _createdNew;
// Static field so the GC cannot collect it while main is running.
var _singleton = new System.Threading.Mutex(initiallyOwned: true, name: _mutexName, createdNew: out _createdNew);
if (!_createdNew)
{
    if (!_singleton.WaitOne(TimeSpan.Zero))
    {
        logger.Info($"[singleton] another host already owns mutex '{_mutexName}'; exiting cleanly (code 3)");
        logger.Flush();
        logger.Dispose();
        Environment.Exit(3);
    }
    logger.Info($"[singleton] acquired stale mutex '{_mutexName}' (previous owner died); proceeding");
}
else
{
    logger.Info($"[singleton] created mutex '{_mutexName}'; proceeding as sole instance");
}
logger.Info("[startup] phase=singleton-acquired");

// Verify the subprocess EXE is present before opening any IPC.
var hostDir      = Path.GetDirectoryName(typeof(Program).Assembly.Location) ?? AppContext.BaseDirectory;
var subprocessExe = Path.Combine(hostDir, "WKOpenVR.FaceModuleProcess.exe");
logger.Info($"[startup] subprocess-exe-path={subprocessExe} (exists={File.Exists(subprocessExe)})");
if (!File.Exists(subprocessExe))
{
    logger.Error("[startup] FATAL: subprocess EXE missing; aborting");
    HostStatusWriter.WriteStartupFailure(
        opts.StatusFilePath,
        "subprocess-missing",
        $"Subprocess EXE missing: {subprocessExe}",
        opts,
        logger);
    logger.Flush();
    Environment.Exit(5);
}

// Wire SIGINT / SIGTERM -> graceful shutdown.
Console.CancelKeyPress += (_, e) => { e.Cancel = true; cts.Cancel(); };
AppDomain.CurrentDomain.ProcessExit += (_, _) => cts.Cancel();

// Phase logs are flushed per step so the next reader of a hung-host log
// sees exactly where the main thread last reached. The constructors and
// awaits below have historically all been "instant" but Windows Defender
// real-time scanning of a freshly-extracted .NET process can stretch any
// of them into multi-second waits.
logger.Info("[startup] phase=signal-handlers-wired");
logger.Flush();

var registry = RegistryClient.Create();
logger.Info("[startup] phase=registry-client-created");
logger.Flush();

var loader   = new SubprocessManager(opts, logger);
logger.Info("[startup] phase=subprocess-manager-created");
logger.Flush();

var writer   = new FrameWriter(opts.ShmemName, logger);
logger.Info("[startup] phase=frame-writer-created");
logger.Flush();

logger.Info($"[startup] phase=opening-ipc-pipe pipe={opts.DriverHandshakePipe}");
logger.Flush();
// Pass the same CTS so MsgShutdown cancels all workers.
var pipe     = new HostControlPipeServer(opts.DriverHandshakePipe, loader, logger, cts);
logger.Info("[startup] phase=ipc-pipe-created");
logger.Flush();

var status   = new HostStatusWriter(opts.StatusFilePath, loader, logger, opts);
logger.Info("[startup] phase=status-writer-created");
logger.Flush();

logger.Info($"WKOpenVR.FaceModuleHost starting. shmem={opts.ShmemName} pipe={opts.DriverHandshakePipe}");

var oscQuery = new OscQueryAdvertiser();
logger.Info("[startup] phase=osc-query-advertiser-created");
logger.Flush();

try
{
    await writer.OpenAsync(ct);
    logger.Info("[startup] phase=shmem-ring-opened");
    logger.Flush();

    await oscQuery.StartAsync(logger, ct);
    logger.Info("[startup] phase=osc-query-started");
    logger.Flush();

    logger.Info($"[startup] phase=discovering-modules path={opts.ModulesInstallDir}");
    logger.Flush();
    var loadedModules = await loader.LoadAllAsync();
    logger.Info($"[startup] phase=modules-loaded count={loadedModules.Count}");
    logger.Flush();
    // (Expression remap moved to the driver; the host now forwards the
    // upstream UnifiedExpressions array on the wire.)

    logger.Info("[startup] phase=starting-workers");
    // Start I/O workers concurrently.
    var workers = new Task[]
    {
        pipe.RunAsync(ct),
        RunRegistryPollAsync(registry, logger, ct),
        loader.RunActiveAsync(writer, ct),
        status.RunAsync(ct),
        RunHostHeartbeatAsync(loader, writer, ct),
    };

    logger.Info("[startup] phase=running");

    // Surface any worker fault rather than silently discarding it.
    var finishedTask = await Task.WhenAny(workers);
    try
    {
        await finishedTask;
    }
    catch (OperationCanceledException) { /* normal shutdown path */ }
    catch (Exception ex)
    {
        logger.Error($"Worker faulted: {ex}");
    }

    // Cancel remaining workers and wait for them all to finish cleanly.
    await cts.CancelAsync();
    try { await Task.WhenAll(workers); }
    catch { /* individual faults already logged above or in the workers themselves */ }
}
catch (OperationCanceledException) { /* clean shutdown */ }
catch (Exception ex)
{
    logger.Error($"[crash] Main threw: {ex}");
    HostStatusWriter.WriteStartupFailure(
        opts.StatusFilePath,
        "main-crash",
        ex.Message,
        opts,
        logger);
    logger.Flush();
    oscQuery.Stop();
    await loader.UnloadAllAsync();
    writer.Dispose();
    logger.Info("Shutdown complete.");
    logger.Flush();
    logger.Dispose();
    return 1;
}

// Mark the shmem header so the driver-side supervisor distinguishes a
// graceful shutdown (heartbeat stale because we're going away) from a wedge
// (heartbeat stale because the CLR hung).
loader.SetHostStateDraining();
try { writer.WriteHostState(FrameWriter.HostStateDraining); } catch { }

oscQuery.Stop();
await loader.UnloadAllAsync();
writer.Dispose();
logger.Info("Shutdown complete.");
logger.Flush();
logger.Dispose();
return 0;

// Publish the host's current state + a heartbeat into the shmem header
// every 100 ms. The driver compares the heartbeat against the publish-
// state to decide whether a stalled publish_index is a wedge or normal
// idle. Keeping this loop separate from the per-frame publish path means
// the driver still sees a fresh heartbeat when no module is selected.
static async Task RunHostHeartbeatAsync(
    SubprocessManager loader, FrameWriter writer, CancellationToken ct)
{
    try
    {
        while (!ct.IsCancellationRequested)
        {
            writer.WriteHostState(loader.CurrentHostState);
            writer.WriteHeartbeatTick();
            await Task.Delay(100, ct);
        }
    }
    catch (OperationCanceledException) { /* shutdown */ }
}

static async Task RunRegistryPollAsync(
    RegistryClient registry, HostLogger logger, CancellationToken ct)
{
    while (!ct.IsCancellationRequested)
    {
        try
        {
            await Task.Delay(TimeSpan.FromHours(6), ct);
            var index = await registry.GetIndexAsync(ct);
            if (index is not null)
                logger.Info($"Registry index refreshed: {index.Modules.Length} module(s) listed.");
        }
        catch (OperationCanceledException) { break; }
        catch (Exception ex)
        {
            logger.Warn($"Registry poll failed: {ex.Message}");
        }
    }
}

static async Task<int> RunE2eFakeFramesAsync(
    HostOptions opts, HostLogger logger, CancellationToken ct)
{
    long framesWritten = 0;
    logger.Info($"[e2e] fake face output starting shmem={opts.ShmemName} frames={opts.E2eFakeFrameCount}");
    logger.Flush();
    HostStatusWriter.WriteE2eStatus(
        opts.StatusFilePath, "e2e-fake-running", framesWritten, opts, logger);
    logger.Info("[e2e] phase=initial-status-written");
    logger.Flush();

    try
    {
        using var writer = new FrameWriter(opts.ShmemName, logger);
        logger.Info("[e2e] phase=frame-writer-created");
        logger.Flush();
        await writer.OpenAsync(ct);
        logger.Info("[e2e] phase=shmem-ring-opened");
        logger.Flush();

        var eye = new EyeFrameSink
        {
            LeftOpenness       = 0.62f,
            RightOpenness      = 0.58f,
            PupilDilationLeft  = 0.33f,
            PupilDilationRight = 0.37f,
        };
        eye.Left.OriginHmd = new Vector3(-0.032f, 0.012f, -0.045f);
        eye.Right.OriginHmd = new Vector3(0.032f, 0.012f, -0.045f);
        eye.Left.DirHmd = new Vector3(-0.10f, 0.20f, -0.97f);
        eye.Right.DirHmd = new Vector3(0.11f, 0.18f, -0.97f);
        eye.Left.Confidence = 0.95f;
        eye.Right.Confidence = 0.96f;

        // Wire format is upstream VRCFaceTracking.UnifiedExpressions order.
        // Anchor a few well-known shapes so the e2e harness can verify the
        // host->wire->driver remap chain end to end, including the semantic
        // aliases that bridge upstream's later renames to our pre-rename
        // names (MouthCornerPull -> MouthSmile).
        float[] upstreamShapes = new float[FrameWriter.UpstreamShapeCount];
        const int kUpstreamJawOpenIndex            = 22;
        const int kUpstreamMouthCornerPullLeftIdx  = 57; // upstream v5 name
        upstreamShapes[kUpstreamJawOpenIndex]           = 0.75f;
        upstreamShapes[kUpstreamMouthCornerPullLeftIdx] = 0.25f;

        logger.Info("[e2e] phase=publishing-frames");
        logger.Flush();
        for (int i = 0; i < opts.E2eFakeFrameCount; ++i)
        {
            await writer.PublishAsync(
                eye,
                upstreamShapes,
                eyeValid: true,
                exprValid: true,
                moduleUuidHash: 0xE2E0FACADE123456UL,
                ct);
            framesWritten++;
            if (opts.E2eFakeFrameIntervalMs > 0)
                await Task.Delay(opts.E2eFakeFrameIntervalMs, ct);
        }

        HostStatusWriter.WriteE2eStatus(
            opts.StatusFilePath, "e2e-fake-complete", framesWritten, opts, logger);
        logger.Info($"[e2e] fake face output complete frames_written={framesWritten}");
        logger.Flush();
        return 0;
    }
    catch (Exception ex)
    {
        logger.Error($"[e2e] fake face output failed: {ex}");
        HostStatusWriter.WriteStartupFailure(
            opts.StatusFilePath, "e2e-fake-failed", ex.Message, opts, logger);
        return 1;
    }
}
