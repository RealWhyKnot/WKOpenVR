using System.Net;
using System.Net.NetworkInformation;
using System.Text;
using MeaMod.DNS.Model;
using MeaMod.DNS.Multicast;
using WKOpenVR.FaceModuleHost.Logging;

namespace WKOpenVR.FaceModuleHost;

/// <summary>
/// Advertises an OSCQuery service so VRChat's router will honour incoming OSC packets.
/// Runs an HTTP server (HttpListener) on a random free TCP port that serves the parameter
/// tree, and advertises both _oscjson._tcp.local and _osc._udp.local via mDNS (MeaMod.DNS).
/// </summary>
public sealed class OscQueryAdvertiser : IDisposable
{
    // Eye parameters emitted by the driver at /avatar/parameters/<name>.
    // Source: modules/facetracking/src/driver/FaceOscPublisher.cpp::PublishEye.
    private static readonly string[] EyeParamNamesLegacy =
    [
        "LeftEyeX",
        "LeftEyeY",
        "RightEyeX",
        "RightEyeY",
        "LeftEyeLid",
        "RightEyeLid",
        "EyesDilation",
    ];

    // Eye parameters emitted by the driver at /avatar/parameters/v2/<name>.
    // Modern VRCFaceTracking-v5 avatars bind to these; the names differ
    // from the legacy set (EyeOpenLeft vs LeftEyeLid, EyeLeftX vs LeftEyeX).
    private static readonly string[] EyeParamNamesV2 =
    [
        "EyeLeftX",
        "EyeLeftY",
        "EyeRightX",
        "EyeRightY",
        "EyeOpenLeft",
        "EyeOpenRight",
        "PupilDilation",
    ];

    // Unified Expression parameter names emitted at both
    // /avatar/parameters/<name> AND /avatar/parameters/v2/<name>.
    // Indices 0-62 mirror FaceOscPublisher.cpp's kExprParamNames.
    private static readonly string[] ExprParamNames =
    [
        "EyeLookOutLeft",     // 0
        "EyeLookInLeft",      // 1
        "EyeLookUpLeft",      // 2
        "EyeLookDownLeft",    // 3
        "EyeLookOutRight",    // 4
        "EyeLookInRight",     // 5
        "EyeLookUpRight",     // 6
        "EyeLookDownRight",   // 7
        "EyeWideLeft",        // 8
        "EyeWideRight",       // 9
        "EyeSquintLeft",      // 10
        "EyeSquintRight",     // 11
        "BrowLowererLeft",    // 12
        "BrowLowererRight",   // 13
        "BrowInnerUpLeft",    // 14
        "BrowInnerUpRight",   // 15
        "BrowOuterUpLeft",    // 16
        "BrowOuterUpRight",   // 17
        "BrowPinchLeft",      // 18
        "BrowPinchRight",     // 19
        "CheekPuffLeft",      // 20
        "CheekPuffRight",     // 21
        "CheekSuckLeft",      // 22
        "CheekSuckRight",     // 23
        "NoseSneerLeft",      // 24
        "NoseSneerRight",     // 25
        "JawOpen",            // 26
        "JawForward",         // 27
        "JawLeft",            // 28
        "JawRight",           // 29
        "LipSuckUpperLeft",   // 30
        "LipSuckUpperRight",  // 31
        "LipSuckLowerLeft",   // 32
        "LipSuckLowerRight",  // 33
        "LipFunnelUpperLeft", // 34
        "LipFunnelUpperRight",// 35
        "LipFunnelLowerLeft", // 36
        "LipFunnelLowerRight",// 37
        "LipPuckerUpperLeft", // 38
        "LipPuckerUpperRight",// 39
        "MouthClose",         // 40
        "MouthUpperLeft",     // 41
        "MouthUpperRight",    // 42
        "MouthLowerLeft",     // 43
        "MouthLowerRight",    // 44
        "MouthSmileLeft",     // 45
        "MouthSmileRight",    // 46
        "MouthSadLeft",       // 47
        "MouthSadRight",      // 48
        "MouthStretchLeft",   // 49
        "MouthStretchRight",  // 50
        "MouthDimpleLeft",    // 51
        "MouthDimpleRight",   // 52
        "MouthRaiserUpper",   // 53
        "MouthRaiserLower",   // 54
        "MouthPressLeft",     // 55
        "MouthPressRight",    // 56
        "MouthTightenerLeft", // 57
        "MouthTightenerRight",// 58
        "TongueOut",          // 59
        "TongueUp",           // 60
        "TongueDown",         // 61
        "TongueLeft",         // 62
    ];

    // Upstream VRCFaceTracking-v5 alias names emitted alongside the
    // pre-rename names for five slots so avatars built against either
    // naming convention discover the parameter. FaceOscPublisher.cpp's
    // kExprParamUpstreamAliases is the lockstep counterpart.
    private static readonly string[] ExprParamUpstreamAliases =
    [
        "MouthClosed",            // <-> MouthClose
        "MouthCornerPullLeft",    // <-> MouthSmileLeft
        "MouthCornerPullRight",   // <-> MouthSmileRight
        "MouthFrownLeft",         // <-> MouthSadLeft
        "MouthFrownRight",        // <-> MouthSadRight
    ];

    private const int OscUdpPort = 9000;

    private readonly string _suffix;
    private readonly string _serviceName;

    private HttpListener?      _http;
    private MulticastService?  _mdns;
    private ServiceDiscovery?  _sd;
    private int                _httpPort;
    private CancellationToken  _ct;
    private Task?              _httpLoop;

    public OscQueryAdvertiser()
    {
        _suffix      = Guid.NewGuid().ToString("N")[..8];
        _serviceName = $"WKOpenVR-FT-{_suffix}";
    }

    public async Task StartAsync(HostLogger logger, CancellationToken ct)
    {
        _ct = ct;

        // --- 1. Allocate a random free TCP port ---------------------------------
        _httpPort = FindFreePort();

        // --- 2. Start HttpListener ----------------------------------------------
        _http = new HttpListener();
        _http.Prefixes.Add($"http://127.0.0.1:{_httpPort}/");
        _http.Start();
        _httpLoop = Task.Run(() => HttpLoopAsync(_http, logger, ct), ct);

        // --- 3. Advertise via mDNS (best-effort; swallow 5353-busy errors) ------
        bool mdnsOk = false;
        try
        {
            _mdns = new MulticastService();
            _sd   = new ServiceDiscovery(_mdns);

            // _oscjson._tcp.local -- the OSCQuery HTTP service
            var oscJson = new ServiceProfile(
                instanceName: _serviceName,
                serviceName:  "_oscjson._tcp",
                port:         (ushort)_httpPort,
                addresses:    [IPAddress.Loopback]);
            oscJson.AddProperty("txtvers", "1");

            // _osc._udp.local -- the OSC UDP receiver
            var oscUdp = new ServiceProfile(
                instanceName: _serviceName,
                serviceName:  "_osc._udp",
                port:         (ushort)OscUdpPort,
                addresses:    [IPAddress.Loopback]);
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

        int totalParams = EyeParamNamesLegacy.Length + EyeParamNamesV2.Length
                        + ExprParamNames.Length * 2
                        + ExprParamUpstreamAliases.Length * 2;
        logger.Info($"[ftq] OSCQuery advertised: name={_serviceName} httpPort={_httpPort} oscPort={OscUdpPort} params={totalParams} mdns={mdnsOk}");
    }

    public void Stop()
    {
        try { _sd?.Dispose();   } catch { }
        try { _mdns?.Stop();    } catch { }
        try { _mdns?.Dispose(); } catch { }
        try { _http?.Stop();    } catch { }
        try { _http?.Close();   } catch { }
        _sd   = null;
        _mdns = null;
        _http = null;
    }

    public void Dispose() => Stop();

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

    private void HandleRequest(HttpListenerContext ctx, HostLogger logger)
    {
        try
        {
            bool isHostInfo = ctx.Request.Url?.Query.Contains("HOST_INFO",
                StringComparison.OrdinalIgnoreCase) == true;

            string json = isHostInfo ? BuildHostInfoJson() : BuildRootJson();
            byte[] bytes = Encoding.UTF8.GetBytes(json);
            ctx.Response.ContentType    = "application/json";
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

    private string BuildHostInfoJson() =>
        $$"""
        {
          "NAME": "{{_serviceName}}",
          "OSC_IP": "127.0.0.1",
          "OSC_PORT": {{OscUdpPort}},
          "OSC_TRANSPORT": "UDP",
          "EXTENSIONS": { "ACCESS": true, "VALUE": true, "TYPE": true }
        }
        """;

    private string BuildRootJson()
    {
        var sb = new StringBuilder(32768);
        sb.Append("""{"DESCRIPTION":"WKOpenVR Face Tracking","FULL_PATH":"/","ACCESS":0,"CONTENTS":{"avatar":{"FULL_PATH":"/avatar","ACCESS":0,"CONTENTS":{"parameters":{"FULL_PATH":"/avatar/parameters","ACCESS":0,"CONTENTS":{""");

        bool first = true;

        // Legacy eye params (LeftEyeX/Y, RightEyeX/Y, LeftEyeLid, RightEyeLid, EyesDilation)
        foreach (string name in EyeParamNamesLegacy)
        {
            if (!first) sb.Append(',');
            first = false;
            AppendLeafNode(sb, $"/avatar/parameters/{name}", name);
        }

        // Expression params (our names) + upstream aliases for the 5 renamed slots.
        foreach (string name in ExprParamNames)
        {
            sb.Append(',');
            AppendLeafNode(sb, $"/avatar/parameters/{name}", name);
        }
        foreach (string name in ExprParamUpstreamAliases)
        {
            sb.Append(',');
            AppendLeafNode(sb, $"/avatar/parameters/{name}", name);
        }

        // v2 subtree
        sb.Append(""","v2":{"FULL_PATH":"/avatar/parameters/v2","ACCESS":0,"CONTENTS":{""");

        bool firstV2 = true;
        foreach (string name in EyeParamNamesV2)
        {
            if (!firstV2) sb.Append(',');
            firstV2 = false;
            AppendLeafNode(sb, $"/avatar/parameters/v2/{name}", name);
        }
        foreach (string name in ExprParamNames)
        {
            sb.Append(',');
            AppendLeafNode(sb, $"/avatar/parameters/v2/{name}", name);
        }
        foreach (string name in ExprParamUpstreamAliases)
        {
            sb.Append(',');
            AppendLeafNode(sb, $"/avatar/parameters/v2/{name}", name);
        }

        sb.Append("}}"); // close v2 CONTENTS + v2 node
        sb.Append("}}}}}}"); // close parameters CONTENTS, parameters, avatar CONTENTS, avatar, root CONTENTS, root
        return sb.ToString();
    }

    private static void AppendLeafNode(StringBuilder sb, string path, string key)
    {
        // key as JSON-safe string (param names are alphanumeric, no escaping needed)
        sb.Append('"').Append(key).Append('"');
        sb.Append(':');
        sb.Append("{\"FULL_PATH\":\"").Append(path).Append('"');
        sb.Append(",\"ACCESS\":3,\"TYPE\":\"f\",\"VALUE\":[0.0]}");
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
}
