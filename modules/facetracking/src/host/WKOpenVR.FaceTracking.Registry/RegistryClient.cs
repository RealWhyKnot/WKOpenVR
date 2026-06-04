using System.Net.Http.Json;
using System.Text.Json;

namespace WKOpenVR.FaceTracking.Registry;

/// <summary>
/// HTTP client for the legacy-registry.whyknot.dev v1 module registry,
/// the curated mirror of upstream VRCFaceTracking modules.
/// All methods throw <see cref="HttpRequestException"/> on non-2xx responses.
/// </summary>
public sealed class RegistryClient(HttpClient http)
{
    public const string DefaultBaseUrl = "https://legacy-registry.whyknot.dev";

    private static readonly JsonSerializerOptions JsonOpts = new(JsonSerializerDefaults.Web);

    // Cached ETag from the last /v1/index response for conditional GET.
    private string? _indexETag;

    public static RegistryClient Create(string? baseUrl = null)
    {
        var http = new HttpClient { BaseAddress = new Uri(baseUrl ?? DefaultBaseUrl) };
        http.DefaultRequestHeaders.UserAgent.ParseAdd("WKOpenVR.FaceModuleHost/1.0");
        return new RegistryClient(http);
    }

    /// <summary>
    /// Fetches the module index. Uses ETag / If-None-Match to avoid redundant transfers.
    /// Returns null if the server responded 304 Not Modified (caller should use its cached copy).
    /// </summary>
    public async Task<IndexDocument?> GetIndexAsync(CancellationToken ct = default)
    {
        using var req = new HttpRequestMessage(HttpMethod.Get, "/v1/index");
        if (_indexETag is not null)
        {
            req.Headers.TryAddWithoutValidation("If-None-Match", _indexETag);
        }

        using HttpResponseMessage resp = await http.SendAsync(req, HttpCompletionOption.ResponseHeadersRead, ct);

        if (resp.StatusCode == System.Net.HttpStatusCode.NotModified)
        {
            return null;
        }

        resp.EnsureSuccessStatusCode();

        if (resp.Headers.ETag is { } etag)
        {
            _indexETag = etag.Tag;
        }

        return await resp.Content.ReadFromJsonAsync<IndexDocument>(JsonOpts, ct);
    }

    /// <summary>Returns the manifest for the latest version of a module, or a specific version.</summary>
    public async Task<Manifest> GetManifestAsync(
        string uuid, string version = "latest", CancellationToken ct = default)
    {
        string path = version == "latest"
            ? $"/v1/modules/{Uri.EscapeDataString(uuid)}/manifest"
            : $"/v1/modules/{Uri.EscapeDataString(uuid)}/versions/{Uri.EscapeDataString(version)}/manifest";

        Manifest? result = await http.GetFromJsonAsync<Manifest>(path, JsonOpts, ct);
        return result ?? throw new InvalidDataException($"Registry returned null manifest for {uuid}");
    }

    /// <summary>Opens a streaming response body for the module payload zip.</summary>
    public async Task<Stream> GetPayloadAsync(
        string uuid, string version, CancellationToken ct = default)
    {
        string path = $"/v1/modules/{Uri.EscapeDataString(uuid)}/versions/{Uri.EscapeDataString(version)}/payload";
        return await http.GetStreamAsync(path, ct);
    }
}
