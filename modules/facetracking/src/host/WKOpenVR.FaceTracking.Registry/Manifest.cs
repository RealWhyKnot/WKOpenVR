using System.Text.Json.Serialization;

namespace WKOpenVR.FaceTracking.Registry;

/// <summary>
/// Full module manifest as returned by GET /v1/modules/{uuid}/manifest.
/// Field names match the v1 schema served by legacy-registry.whyknot.dev (and
/// any future sibling registry that adopts the same schema).
/// </summary>
public sealed class Manifest
{
    [JsonPropertyName("schema")] public int Schema { get; init; }
    [JsonPropertyName("uuid")] public required string Uuid { get; init; }
    [JsonPropertyName("name")] public required string Name { get; init; }
    [JsonPropertyName("vendor")] public required string Vendor { get; init; }
    [JsonPropertyName("homepage")] public string? Homepage { get; init; }
    [JsonPropertyName("license")] public string? License { get; init; }
    [JsonPropertyName("version")] public required string Version { get; init; }
    [JsonPropertyName("sdk_version")] public required string SdkVersion { get; init; }
    [JsonPropertyName("min_host_version")] public required string MinHostVersion { get; init; }
    [JsonPropertyName("supported_hmds")] public required string[] SupportedHmds { get; init; }
    [JsonPropertyName("capabilities")] public required string[] Capabilities { get; init; }
    [JsonPropertyName("platforms")] public required string[] Platforms { get; init; }
    [JsonPropertyName("entry_assembly")] public required string EntryAssembly { get; init; }
    [JsonPropertyName("entry_type")] public required string EntryType { get; init; }
    [JsonPropertyName("dependencies")] public string[] Dependencies { get; init; } = [];
    [JsonPropertyName("payload_sha256")] public required string PayloadSha256 { get; init; }
    [JsonPropertyName("payload_size")] public long PayloadSize { get; init; }
}
