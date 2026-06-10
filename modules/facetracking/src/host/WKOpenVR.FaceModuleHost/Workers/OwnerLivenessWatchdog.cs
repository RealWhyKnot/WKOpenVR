using System.IO.MemoryMappedFiles;
using WKOpenVR.FaceModuleHost.Logging;

namespace WKOpenVR.FaceModuleHost.Workers;

internal static class OwnerLivenessWatchdog
{
    private const uint Magic = 0x534F574Eu; // "SOWN"
    private const uint Version = 1;
    private const uint FaceTrackingModuleId = 4;
    private const uint StateShuttingDown = 3;
    private const uint StateDisabled = 4;
    private const long StaleAfterMs = 3000;

    private const long MagicOffset = 0;
    private const long VersionOffset = 4;
    private const long ModuleIdOffset = 8;
    private const long NonceOffset = 16;
    private const long StateOffset = 24;
    private const long HeartbeatOffset = 32;
    private const long LeaseSize = 64;

    public static async Task RunAsync(
        HostOptions options,
        HostLogger logger,
        CancellationTokenSource shutdownCts,
        CancellationToken ct)
    {
        if (!options.OwnerLivenessConfigured)
        {
            try { await Task.Delay(Timeout.InfiniteTimeSpan, ct); }
            catch (OperationCanceledException) { }
            return;
        }

        using var mapping = OpenMapping(options.OwnerLivenessName!, logger);
        if (mapping is null)
        {
            await shutdownCts.CancelAsync();
            return;
        }

        using var view = mapping.CreateViewAccessor(0, LeaseSize, MemoryMappedFileAccess.Read);
        logger.Info($"[owner-liveness] watching lease '{options.OwnerLivenessName}'");

        while (!ct.IsCancellationRequested)
        {
            string status = Evaluate(view, options.OwnerLivenessNonce);
            if (status != "alive")
            {
                logger.Warn($"[owner-liveness] owner lease no longer alive: {status}; shutting down");
                await shutdownCts.CancelAsync();
                return;
            }

            await Task.Delay(250, ct);
        }
    }

    private static MemoryMappedFile? OpenMapping(string name, HostLogger logger)
    {
        try
        {
            return MemoryMappedFile.OpenExisting(name, MemoryMappedFileRights.Read);
        }
        catch (Exception ex)
        {
            logger.Warn($"[owner-liveness] failed to open lease '{name}': {ex.Message}");
            return null;
        }
    }

    private static string Evaluate(MemoryMappedViewAccessor view, ulong expectedNonce)
    {
        uint magic = view.ReadUInt32(MagicOffset);
        uint version = view.ReadUInt32(VersionOffset);
        if (magic != Magic || version != Version)
        {
            return "header-mismatch";
        }

        uint moduleId = view.ReadUInt32(ModuleIdOffset);
        if (moduleId != FaceTrackingModuleId)
        {
            return "module-mismatch";
        }

        ulong nonce = view.ReadUInt64(NonceOffset);
        if (nonce != expectedNonce)
        {
            return "nonce-mismatch";
        }

        uint state = view.ReadUInt32(StateOffset);
        if (state == StateDisabled)
        {
            return "disabled";
        }

        if (state == StateShuttingDown)
        {
            return "shutting-down";
        }

        ulong heartbeat = view.ReadUInt64(HeartbeatOffset);
        ulong now = unchecked((ulong)Environment.TickCount64);
        if (heartbeat == 0 || now < heartbeat || now - heartbeat > StaleAfterMs)
        {
            return "stale";
        }

        return "alive";
    }
}
