using System.Net;
using System.Net.NetworkInformation;
using System.Net.Sockets;
using System.Text;
using MeaMod.DNS.Model;
using MeaMod.DNS.Multicast;
using WKOpenVR.FaceModuleHost.Logging;

namespace WKOpenVR.FaceModuleHost;

/// <summary>
/// Advertises an OSCQuery service so VRChat's router will honour incoming OSC packets.
/// Runs an HTTP server (HttpListener) on a random free TCP port that serves the parameter
/// tree, binds a private OSC UDP receive port for VRChat callbacks, and advertises both
/// _oscjson._tcp.local and _osc._udp.local via mDNS (MeaMod.DNS).
/// </summary>
public sealed class OscQueryAdvertiser : IDisposable
{
    private readonly string _suffix;
    private readonly string _serviceName;

    private HttpListener? _http;
    private UdpClient? _udp;
    private MulticastService? _mdns;
    private ServiceDiscovery? _sd;
    private int _httpPort;
    private int _oscUdpPort;
    private CancellationToken _ct;
    private Task? _httpLoop;
    private Task? _udpLoop;

    private int _inboundOscCount;
    private readonly HashSet<string> _observedAvatarParameters = new(StringComparer.Ordinal);
    private int _lastInferenceObservedCount;
    private int _allowListInferenceInFlight;
    private int _allowListInferenceResolved;

    public OscQueryAdvertiser()
    {
        _suffix = Guid.NewGuid().ToString("N")[..8];
        _serviceName = $"WKOpenVR-FT-{_suffix}";
    }

    public async Task StartAsync(HostLogger logger, CancellationToken ct)
    {
        _ct = ct;
        AvatarParameterAllowList.RestoreLastKnown(logger);

        // --- 1. Bind a private OSC receive socket -------------------------------
        _udp = new UdpClient(new IPEndPoint(IPAddress.Loopback, 0));
        _oscUdpPort = ((IPEndPoint)_udp.Client.LocalEndPoint!).Port;
        _udpLoop = Task.Run(() => UdpLoopAsync(_udp, logger, ct), ct);

        // --- 2. Allocate a random free TCP port ---------------------------------
        _httpPort = FindFreePort();

        // --- 3. Start HttpListener ----------------------------------------------
        _http = new HttpListener();
        _http.Prefixes.Add($"http://127.0.0.1:{_httpPort}/");
        _http.Start();
        _httpLoop = Task.Run(() => HttpLoopAsync(_http, logger, ct), ct);

        // --- 4. Advertise via mDNS (best-effort; swallow 5353-busy errors) ------
        bool mdnsOk = false;
        try
        {
            _mdns = new MulticastService();
            _sd = new ServiceDiscovery(_mdns);

            // _oscjson._tcp.local -- the OSCQuery HTTP service
            var oscJson = new ServiceProfile(
                instanceName: _serviceName,
                serviceName: "_oscjson._tcp",
                port: (ushort)_httpPort,
                addresses: [IPAddress.Loopback]);
            oscJson.AddProperty("txtvers", "1");

            // _osc._udp.local -- the OSC UDP receiver
            var oscUdp = new ServiceProfile(
                instanceName: _serviceName,
                serviceName: "_osc._udp",
                port: (ushort)_oscUdpPort,
                addresses: [IPAddress.Loopback]);
            oscUdp.AddProperty("txtvers", "1");

            _sd.Advertise(oscJson);
            _sd.Advertise(oscUdp);
            _mdns.Start();

            // Announce once immediately so discovery is near-instant.
            await Task.Run(() => { _sd.Announce(oscJson); _sd.Announce(oscUdp); }, ct);
            mdnsOk = true;
        }
        catch (Exception ex)
        {
            logger.Warn($"[ftq] mDNS advertise failed (port 5353 may be in use): {ex.Message}; HTTP-only mode.");
        }

        logger.Info($"[ftq] OSCQuery advertised: name={_serviceName} httpPort={_httpPort} oscPort={_oscUdpPort} mdns={mdnsOk}");
    }

    public void Stop()
    {
        try { _sd?.Dispose(); } catch { }
        try { _mdns?.Stop(); } catch { }
        try { _mdns?.Dispose(); } catch { }
        try { _udp?.Close(); } catch { }
        try { _udp?.Dispose(); } catch { }
        try { _http?.Stop(); } catch { }
        try { _http?.Close(); } catch { }
        _sd = null;
        _mdns = null;
        _udp = null;
        _http = null;
    }

    public void Dispose()
    {
        Stop();
    }

    // -------------------------------------------------------------------------
    // HTTP handler
    // -------------------------------------------------------------------------

    private async Task HttpLoopAsync(HttpListener listener, HostLogger logger, CancellationToken ct)
    {
        while (!ct.IsCancellationRequested)
        {
            HttpListenerContext ctx;
            try
            {
                ctx = await listener.GetContextAsync().WaitAsync(ct);
            }
            catch (OperationCanceledException) { break; }
            catch (HttpListenerException ex) when (ct.IsCancellationRequested || ex.ErrorCode == 995)
            {
                break; // listener stopped
            }
            catch (Exception ex)
            {
                logger.Warn($"[ftq] HTTP accept error: {ex.Message}");
                break;
            }

            _ = Task.Run(() => HandleRequest(ctx, logger), CancellationToken.None);
        }
    }

    private async Task UdpLoopAsync(UdpClient udp, HostLogger logger, CancellationToken ct)
    {
        while (!ct.IsCancellationRequested)
        {
            UdpReceiveResult result;
            try
            {
                result = await udp.ReceiveAsync(ct);
            }
            catch (OperationCanceledException) { break; }
            catch (ObjectDisposedException) { break; }
            catch (SocketException ex) when (ct.IsCancellationRequested || ex.SocketErrorCode == SocketError.Interrupted)
            {
                break;
            }
            catch (Exception ex)
            {
                logger.Warn($"[ftq] OSC receive error: {ex.Message}");
                break;
            }

            int count = Interlocked.Increment(ref _inboundOscCount);
            string address = TryReadOscAddress(result.Buffer);
            if (string.Equals(address, "/avatar/change", StringComparison.Ordinal))
            {
                string avatarId = TryReadFirstOscStringArgument(result.Buffer);
                lock (_observedAvatarParameters)
                {
                    _observedAvatarParameters.Clear();
                    _lastInferenceObservedCount = 0;
                }
                Volatile.Write(ref _allowListInferenceResolved, 0);
                _ = Task.Run(
                    () => AvatarParameterAllowList.RefreshForAvatarAsync(avatarId, logger, ct),
                    CancellationToken.None);
            }
            else
            {
                MaybeScheduleAllowListInference(address, logger, ct);
            }

            if (count == 1 || count % 100000 == 0
                || string.Equals(address, "/avatar/change", StringComparison.Ordinal))
            {
                logger.Info($"[ftq] received OSC callback count={count} addr={address}");
            }
        }
    }

    private void MaybeScheduleAllowListInference(string address, HostLogger logger, CancellationToken ct)
    {
        if (Volatile.Read(ref _allowListInferenceResolved) != 0 ||
            !address.StartsWith("/avatar/parameters/", StringComparison.Ordinal))
        {
            return;
        }

        string[] snapshot;
        lock (_observedAvatarParameters)
        {
            if (!_observedAvatarParameters.Add(address))
            {
                return;
            }

            if (_observedAvatarParameters.Count < 16 ||
                _observedAvatarParameters.Count < _lastInferenceObservedCount + 16)
            {
                return;
            }

            _lastInferenceObservedCount = _observedAvatarParameters.Count;
            snapshot = [.. _observedAvatarParameters];
        }

        if (Interlocked.CompareExchange(ref _allowListInferenceInFlight, 1, 0) != 0)
        {
            return;
        }

        _ = Task.Run(async () =>
        {
            try
            {
                bool restored = await AvatarParameterAllowList.RefreshFromObservedParametersAsync(snapshot, logger, ct);
                if (restored)
                {
                    Volatile.Write(ref _allowListInferenceResolved, 1);
                }
            }
            finally
            {
                Volatile.Write(ref _allowListInferenceInFlight, 0);
            }
        }, CancellationToken.None);
    }

    private void HandleRequest(HttpListenerContext ctx, HostLogger logger)
    {
        try
        {
            bool isHostInfo = ctx.Request.Url?.Query.Contains("HOST_INFO",
                StringComparison.OrdinalIgnoreCase) == true;

            string json = isHostInfo ? BuildHostInfoJson() : BuildRootJson();
            byte[] bytes = Encoding.UTF8.GetBytes(json);
            ctx.Response.ContentType = "application/json";
            ctx.Response.ContentLength64 = bytes.Length;
            ctx.Response.OutputStream.Write(bytes, 0, bytes.Length);
        }
        catch (Exception ex)
        {
            logger.Warn($"[ftq] HTTP response error: {ex.Message}");
        }
        finally
        {
            try { ctx.Response.Close(); } catch { }
        }
    }

    // -------------------------------------------------------------------------
    // JSON builders
    // -------------------------------------------------------------------------

    private string BuildHostInfoJson()
    {
        return $$"""
        {
          "NAME": "{{_serviceName}}",
          "OSC_IP": "127.0.0.1",
          "OSC_PORT": {{_oscUdpPort}},
          "OSC_TRANSPORT": "UDP",
          "EXTENSIONS": { "ACCESS": true, "CLIPMODE": false, "RANGE": true, "VALUE": true, "TYPE": true }
        }
        """;
    }

    private static string BuildRootJson()
    {
        return """
        {"DESCRIPTION":"WKOpenVR Face Tracking","FULL_PATH":"/","ACCESS":0,"CONTENTS":{"avatar":{"FULL_PATH":"/avatar","ACCESS":0,"CONTENTS":{"change":{"FULL_PATH":"/avatar/change","ACCESS":2,"TYPE":"s","VALUE":[""]}}}}}
        """;
    }

    // -------------------------------------------------------------------------
    // Helpers
    // -------------------------------------------------------------------------

    private static int FindFreePort()
    {
        // Bind to port 0 to let the OS assign a free port, then read it back.
        using var tmp = new System.Net.Sockets.TcpListener(IPAddress.Loopback, 0);
        tmp.Start();
        int port = ((IPEndPoint)tmp.LocalEndpoint).Port;
        tmp.Stop();
        return port;
    }

    private static string TryReadOscAddress(byte[] packet)
    {
        if (packet.Length == 0 || packet[0] != (byte)'/')
        {
            return "(unknown)";
        }

        int end = Array.IndexOf(packet, (byte)0);
        if (end <= 0)
        {
            return "(unknown)";
        }

        try
        {
            return Encoding.ASCII.GetString(packet, 0, end);
        }
        catch
        {
            return "(unknown)";
        }
    }

    private static string TryReadFirstOscStringArgument(byte[] packet)
    {
        int offset = 0;
        if (!TryReadOscString(packet, ref offset, out _))
        {
            return "";
        }

        if (!TryReadOscString(packet, ref offset, out string typeTags))
        {
            return "";
        }

        if (!typeTags.Contains('s'))
        {
            return "";
        }

        if (!TryReadOscString(packet, ref offset, out string value))
        {
            return "";
        }

        return value;
    }

    private static bool TryReadOscString(byte[] packet, ref int offset, out string value)
    {
        value = "";
        if (offset < 0 || offset >= packet.Length)
        {
            return false;
        }

        int end = Array.IndexOf(packet, (byte)0, offset);
        if (end < offset)
        {
            return false;
        }

        try
        {
            value = Encoding.UTF8.GetString(packet, offset, end - offset);
        }
        catch
        {
            return false;
        }

        offset = (end + 4) & ~3;
        return offset <= packet.Length;
    }
}
