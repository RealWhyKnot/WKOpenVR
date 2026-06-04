using System.Text.Json.Serialization;

namespace WKOpenVR.FaceTracking.Registry;

public sealed class ManifestSummary
{
    [JsonPropertyName("uuid")] public required string Uuid { get; init; }
    [JsonPropertyName("name")] public required string Name { get; init; }
    [JsonPropertyName("vendor")] public required string Vendor { get; init; }
    [JsonPropertyName("version")] public required string Version { get; init; }
    [JsonPropertyName("capabilities")] public required string[] Capabilities { get; init; }
}

public sealed class IndexDocument
{
    [JsonPropertyName("schema")] public int Schema { get; init; }
    [JsonPropertyName("modules")] public required ManifestSummary[] Modules { get; init; }
    [JsonPropertyName("generated_at")] public DateTimeOffset GeneratedAt { get; init; }
}
