#include "BoundaryPreview.h"

#include "Boundary.h"
#include "DiagnosticsLog.h"

#include <openvr.h>

#include <GL/gl3w.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <string>
#include <utility>

#ifdef _WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#endif

namespace wkopenvr::boundary {
namespace {

constexpr double kMinPreviewSpanMeters = 1.0;
constexpr double kPreviewPadMeters = 0.30;
constexpr int kUploadFailureDisableThreshold = 3;
constexpr int kMaxFileMarkerOverlays = 32;
// Texture-upload retry backoff. A transient SetOverlayTexture failure never
// disables the boundary permanently; failures only pace retries between these
// bounds while a vertex-marker fallback keeps the boundary visible.
constexpr double kUploadBackoffBaseSec = 0.1;
constexpr double kUploadBackoffCapSec = 1.0;
constexpr uint64_t kFnvOffset = 1469598103934665603ull;
constexpr uint64_t kFnvPrime = 1099511628211ull;

struct PreviewState {
    vr::VROverlayHandle_t handle = vr::k_ulOverlayHandleInvalid;
    std::array<vr::VROverlayHandle_t, kMaxFileMarkerOverlays> fileMarkerHandles{};
    std::array<bool, kMaxFileMarkerOverlays> fileMarkerCreated{};
    std::array<bool, kMaxFileMarkerOverlays> fileMarkerTextureReadyBySlot{};
    bool created = false;
    bool visible = false;
    bool fileMarkersVisible = false;
    bool fileMarkerTextureReady = false;
    bool textureReady = false;
    GLuint glTexture = 0;  // GL texture backing the overlay, uploaded via SetOverlayTexture.
    uint64_t uploadedHash = 0;
    uint64_t lastRasterHash = 0;
    int uploadFailureCount = 0;
    int fileMarkerFailureCount = 0;
    bool uploadsDisabled = false;
    uint64_t lastFailedHash = 0;
    double nextUploadAttemptSec = 0.0;
    vr::EVROverlayError lastError = vr::VROverlayError_None;
    vr::EVROverlayError fileMarkerLastError = vr::VROverlayError_None;
    size_t lastVertexCount = 0;
    size_t fileMarkerCount = 0;
    const char* lastSource = "none";
    BoundaryPreviewPlane lastPlane;
};

PreviewState& State()
{
    static PreviewState s;
    return s;
}

double NowSeconds()
{
    using namespace std::chrono;
    return duration<double>(steady_clock::now().time_since_epoch()).count();
}

constexpr const char* kPreviewKey = "wkopenvr.boundary.preview";
constexpr const char* kPreviewName = "Boundary drawing preview";

#ifdef _WIN32
std::filesystem::path ExeDir()
{
    char buf[MAX_PATH] = {};
    const DWORD len = GetModuleFileNameA(nullptr, buf, MAX_PATH);
    if (len == 0 || len == MAX_PATH) return {};
    return std::filesystem::path(buf).parent_path();
}
#else
std::filesystem::path ExeDir()
{
    return {};
}
#endif

const char* OverlayErrorName(vr::EVROverlayError err)
{
    switch (err) {
    case vr::VROverlayError_None: return "None";
    case vr::VROverlayError_UnknownOverlay: return "UnknownOverlay";
    case vr::VROverlayError_InvalidHandle: return "InvalidHandle";
    case vr::VROverlayError_PermissionDenied: return "PermissionDenied";
    case vr::VROverlayError_OverlayLimitExceeded: return "OverlayLimitExceeded";
    case vr::VROverlayError_WrongVisibilityType: return "WrongVisibilityType";
    case vr::VROverlayError_KeyTooLong: return "KeyTooLong";
    case vr::VROverlayError_NameTooLong: return "NameTooLong";
    case vr::VROverlayError_KeyInUse: return "KeyInUse";
    case vr::VROverlayError_WrongTransformType: return "WrongTransformType";
    case vr::VROverlayError_InvalidTrackedDevice: return "InvalidTrackedDevice";
    case vr::VROverlayError_InvalidParameter: return "InvalidParameter";
    case vr::VROverlayError_ThumbnailCantBeDestroyed: return "ThumbnailCantBeDestroyed";
    case vr::VROverlayError_ArrayTooSmall: return "ArrayTooSmall";
    case vr::VROverlayError_RequestFailed: return "RequestFailed";
    case vr::VROverlayError_InvalidTexture: return "InvalidTexture";
    case vr::VROverlayError_UnableToLoadFile: return "UnableToLoadFile";
    case vr::VROverlayError_KeyboardAlreadyInUse: return "KeyboardAlreadyInUse";
    case vr::VROverlayError_NoNeighbor: return "NoNeighbor";
    case vr::VROverlayError_TooManyMaskPrimitives: return "TooManyMaskPrimitives";
    case vr::VROverlayError_BadMaskPrimitive: return "BadMaskPrimitive";
    default: return "Unknown";
    }
}

std::string ResolveFileMarkerTexturePath()
{
    // Prefer the dedicated boundary marker glyph; fall back to the shared app
    // icon if it has not been deployed alongside the binary yet.
    static const char* const kCandidates[] = {
        "boundary_icon.png",
        "dashboard_icon.png",
    };

    const std::filesystem::path exeDir = ExeDir();
    for (const char* name : kCandidates) {
        if (!exeDir.empty()) {
            const std::filesystem::path deployed = exeDir / name;
            if (std::filesystem::exists(deployed)) return deployed.string();
        }
    }
    for (const char* name : kCandidates) {
        const std::filesystem::path source =
            std::filesystem::current_path() / "core" / "src" / "overlay" / name;
        if (std::filesystem::exists(source)) return source.string();
    }
    return {};
}

uint64_t HashU64(uint64_t hash, uint64_t value)
{
    hash ^= value;
    hash *= kFnvPrime;
    return hash;
}

uint64_t HashVertex(uint64_t hash, const BoundaryVertex& v)
{
    hash = HashU64(hash, static_cast<uint64_t>(std::llround(v.x * 100.0)));
    hash = HashU64(hash, static_cast<uint64_t>(std::llround(v.y * 100.0)));
    hash = HashU64(hash, static_cast<uint64_t>(std::llround(v.z * 100.0)));
    return hash;
}

uint64_t HashRenderCommand(uint64_t hash, const SpatialRenderCommand& command)
{
    hash = HashU64(hash, static_cast<uint64_t>(command.kind));
    hash = HashU64(hash, command.closeLoop ? 1u : 0u);
    hash = HashU64(hash, command.ageFade ? 1u : 0u);
    hash = HashU64(hash, static_cast<uint64_t>(command.layer));
    hash = HashU64(hash, static_cast<uint64_t>(command.style.r));
    hash = HashU64(hash, static_cast<uint64_t>(command.style.g));
    hash = HashU64(hash, static_cast<uint64_t>(command.style.b));
    hash = HashU64(hash, static_cast<uint64_t>(command.style.a));
    hash = HashU64(hash, static_cast<uint64_t>(command.style.fillA));
    hash = HashU64(hash, command.style.fill ? 1u : 0u);
    hash = HashU64(hash, static_cast<uint64_t>(std::llround(command.style.strokeMeters * 1000.0)));
    hash = HashU64(hash, static_cast<uint64_t>(std::llround(command.style.dotMeters * 1000.0)));
    hash = HashU64(hash, command.standingVertices.size());
    for (const auto& v : command.standingVertices) {
        hash = HashVertex(hash, v);
    }
    return hash;
}

int ClampPixel(double value)
{
    const int rounded = static_cast<int>(std::lround(value));
    return std::clamp(rounded, 0, BoundaryPreviewRaster::kTextureSize - 1);
}

uint8_t CoveredAlpha(uint8_t alpha, double coverage)
{
    const double scaled = static_cast<double>(alpha) * std::clamp(coverage, 0.0, 1.0);
    return static_cast<uint8_t>(std::clamp(static_cast<int>(std::lround(scaled)), 0, 255));
}

void BlendPixel(std::vector<uint8_t>& pixels, int x, int y, uint8_t r, uint8_t g, uint8_t b, uint8_t a)
{
    constexpr int size = BoundaryPreviewRaster::kTextureSize;
    if (x < 0 || y < 0 || x >= size || y >= size) return;

    const size_t idx = static_cast<size_t>(y * size + x) * 4u;
    const uint16_t inv = static_cast<uint16_t>(255u - a);
    pixels[idx + 0] = static_cast<uint8_t>((static_cast<uint16_t>(pixels[idx + 0]) * inv + r * a) / 255u);
    pixels[idx + 1] = static_cast<uint8_t>((static_cast<uint16_t>(pixels[idx + 1]) * inv + g * a) / 255u);
    pixels[idx + 2] = static_cast<uint8_t>((static_cast<uint16_t>(pixels[idx + 2]) * inv + b * a) / 255u);
    pixels[idx + 3] = static_cast<uint8_t>(std::min<int>(255, pixels[idx + 3] + a));
}

double DistanceToSegment(double px, double py, double ax, double ay, double bx, double by)
{
    const double vx = bx - ax;
    const double vy = by - ay;
    const double lenSq = vx * vx + vy * vy;
    if (lenSq <= 1e-9) {
        const double dx = px - ax;
        const double dy = py - ay;
        return std::sqrt(dx * dx + dy * dy);
    }

    const double t = std::clamp(((px - ax) * vx + (py - ay) * vy) / lenSq, 0.0, 1.0);
    const double cx = ax + vx * t;
    const double cy = ay + vy * t;
    const double dx = px - cx;
    const double dy = py - cy;
    return std::sqrt(dx * dx + dy * dy);
}

void DrawSoftDot(
    std::vector<uint8_t>& pixels,
    int cx,
    int cy,
    double radius,
    double feather,
    uint8_t r,
    uint8_t g,
    uint8_t b,
    uint8_t a)
{
    const int reach = static_cast<int>(std::ceil(radius + feather));
    for (int y = cy - reach; y <= cy + reach; ++y) {
        for (int x = cx - reach; x <= cx + reach; ++x) {
            const double dx = (static_cast<double>(x) + 0.5) - static_cast<double>(cx);
            const double dy = (static_cast<double>(y) + 0.5) - static_cast<double>(cy);
            const double dist = std::sqrt(dx * dx + dy * dy);
            const double coverage = (radius + feather - dist) / std::max(feather, 0.001);
            const uint8_t aa = CoveredAlpha(a, coverage);
            if (aa != 0) BlendPixel(pixels, x, y, r, g, b, aa);
        }
    }
}

void DrawSoftRing(
    std::vector<uint8_t>& pixels,
    int cx,
    int cy,
    double radius,
    double thickness,
    double feather,
    uint8_t r,
    uint8_t g,
    uint8_t b,
    uint8_t a)
{
    const double half = thickness * 0.5;
    const int reach = static_cast<int>(std::ceil(radius + half + feather));
    for (int y = cy - reach; y <= cy + reach; ++y) {
        for (int x = cx - reach; x <= cx + reach; ++x) {
            const double dx = (static_cast<double>(x) + 0.5) - static_cast<double>(cx);
            const double dy = (static_cast<double>(y) + 0.5) - static_cast<double>(cy);
            const double dist = std::sqrt(dx * dx + dy * dy);
            const double edge = std::fabs(dist - radius);
            const double coverage = (half + feather - edge) / std::max(feather, 0.001);
            const uint8_t aa = CoveredAlpha(a, coverage);
            if (aa != 0) BlendPixel(pixels, x, y, r, g, b, aa);
        }
    }
}

void DrawSoftLine(
    std::vector<uint8_t>& pixels,
    int x0,
    int y0,
    int x1,
    int y1,
    double radius,
    double feather,
    uint8_t r,
    uint8_t g,
    uint8_t b,
    uint8_t a)
{
    const int reach = static_cast<int>(std::ceil(radius + feather));
    const int minX = std::min(x0, x1) - reach;
    const int maxX = std::max(x0, x1) + reach;
    const int minY = std::min(y0, y1) - reach;
    const int maxY = std::max(y0, y1) + reach;

    for (int y = minY; y <= maxY; ++y) {
        for (int x = minX; x <= maxX; ++x) {
            const double dist = DistanceToSegment(
                static_cast<double>(x) + 0.5,
                static_cast<double>(y) + 0.5,
                static_cast<double>(x0),
                static_cast<double>(y0),
                static_cast<double>(x1),
                static_cast<double>(y1));
            const double coverage = (radius + feather - dist) / std::max(feather, 0.001);
            const uint8_t aa = CoveredAlpha(a, coverage);
            if (aa != 0) BlendPixel(pixels, x, y, r, g, b, aa);
        }
    }
}

void FillPolygon(
    std::vector<uint8_t>& pixels,
    const std::vector<std::pair<int, int>>& points,
    const SpatialStyle& style)
{
    if (points.size() < 3) return;
    if (!style.fill || style.fillA == 0) return;

    constexpr int size = BoundaryPreviewRaster::kTextureSize;
    std::vector<double> intersections;
    intersections.reserve(points.size());

    for (int y = 0; y < size; ++y) {
        intersections.clear();
        const double scanY = static_cast<double>(y) + 0.5;
        for (size_t i = 0, j = points.size() - 1; i < points.size(); j = i++) {
            const auto& a = points[j];
            const auto& b = points[i];
            const double y0 = static_cast<double>(a.second);
            const double y1 = static_cast<double>(b.second);
            if ((y0 <= scanY && y1 > scanY) || (y1 <= scanY && y0 > scanY)) {
                const double x0 = static_cast<double>(a.first);
                const double x1 = static_cast<double>(b.first);
                const double t = (scanY - y0) / (y1 - y0);
                intersections.push_back(x0 + (x1 - x0) * t);
            }
        }

        std::sort(intersections.begin(), intersections.end());
        for (size_t i = 1; i < intersections.size(); i += 2) {
            const int xStart = std::clamp(
                static_cast<int>(std::ceil(intersections[i - 1])),
                0,
                size - 1);
            const int xEnd = std::clamp(
                static_cast<int>(std::floor(intersections[i])),
                0,
                size - 1);
            for (int x = xStart; x <= xEnd; ++x) {
                BlendPixel(pixels, x, y, style.r, style.g, style.b, style.fillA);
            }
        }
    }
}

void DrawBoundarySegment(
    std::vector<uint8_t>& pixels,
    const std::pair<int, int>& a,
    const std::pair<int, int>& b,
    bool closing,
    double strokeRadius,
    double strokeFeather,
    const SpatialStyle& style)
{
    if (style.strokeMeters <= 0.0 || style.a == 0) return;
    const uint8_t haloAlpha = static_cast<uint8_t>(std::clamp(
        static_cast<int>(style.a) / 4,
        0,
        255));
    const uint8_t midAlpha = static_cast<uint8_t>(std::clamp(
        static_cast<int>(style.a),
        0,
        255));
    DrawSoftLine(pixels, a.first, a.second, b.first, b.second,
        closing ? strokeRadius * 0.9 : strokeRadius,
        strokeFeather,
        style.r, style.g, style.b, haloAlpha);
    DrawSoftLine(pixels, a.first, a.second, b.first, b.second,
        closing ? strokeRadius * 0.35 : strokeRadius * 0.42,
        std::max(1.5, strokeFeather * 0.35),
        style.r, style.g, style.b, midAlpha);
    DrawSoftLine(pixels, a.first, a.second, b.first, b.second,
        std::max(1.25, strokeRadius * 0.14),
        1.5, 255, 255, 255, static_cast<uint8_t>(std::min<int>(style.a, 92)));
}

void DrawBoundaryVertex(
    std::vector<uint8_t>& pixels,
    const std::pair<int, int>& p,
    bool first,
    bool last,
    double dotRadius,
    const SpatialStyle& style)
{
    if (style.dotMeters <= 0.0 || style.a == 0) return;
    const uint8_t haloAlpha = style.fillA != 0
        ? style.fillA
        : static_cast<uint8_t>(std::min<int>(style.a / 4, 72));
    if (last || first) {
        DrawSoftDot(pixels, p.first, p.second, dotRadius * 1.45, dotRadius * 0.55,
            style.r, style.g, style.b, haloAlpha);
        DrawSoftRing(pixels, p.first, p.second, dotRadius,
            std::max(2.0, dotRadius * 0.25),
            2.0,
            style.r,
            style.g,
            style.b,
            static_cast<uint8_t>(std::min<int>(style.a, 240)));
        DrawSoftDot(pixels, p.first, p.second, dotRadius * 0.45, 2.0,
            style.r, style.g, style.b, static_cast<uint8_t>(std::min<int>(style.a, 255)));
        return;
    }

    DrawSoftDot(pixels, p.first, p.second, dotRadius * 0.70, dotRadius * 0.30,
        style.r, style.g, style.b, haloAlpha);
    DrawSoftDot(pixels, p.first, p.second, dotRadius * 0.36, 1.75,
        style.r, style.g, style.b, style.a);
}

bool EnsureCreated()
{
    auto& s = State();
    if (s.created) return true;
    if (!vr::VROverlay()) return false;

    vr::EVROverlayError err =
        vr::VROverlay()->CreateOverlay(kPreviewKey, kPreviewName, &s.handle);
    if (err == vr::VROverlayError_KeyInUse) {
        err = vr::VROverlay()->FindOverlay(kPreviewKey, &s.handle);
    }
    if (err != vr::VROverlayError_None) {
        std::fprintf(stderr, "[boundary-preview] CreateOverlay failed: %d\n", static_cast<int>(err));
        openvr_pair::common::DiagnosticLog("boundary-preview", "create_failed err=%d", static_cast<int>(err));
        return false;
    }

    vr::VROverlay()->SetOverlaySortOrder(s.handle, 12);
    vr::VROverlay()->SetOverlayAlpha(s.handle, 0.95f);
    s.created = true;
    s.visible = false;
    s.uploadedHash = 0;
    s.uploadsDisabled = false;
    s.lastError = vr::VROverlayError_None;
    openvr_pair::common::DiagnosticLog("boundary-preview", "created");
    openvr_pair::common::DiagnosticLog(
        "boundary_preview_status",
        "created=1 visible=0 uploads_disabled=0 failures=0 error=0 error_name=None source=create");
    return true;
}

void HideOnly(const char* source)
{
    auto& s = State();
    if (!s.created) return;
    if (vr::VROverlay() && s.handle != vr::k_ulOverlayHandleInvalid) {
        vr::VROverlay()->HideOverlay(s.handle);
    }
    if (s.visible) {
        openvr_pair::common::DiagnosticLog(
            "boundary_preview_status",
            "created=1 visible=0 uploads_disabled=%d failures=%d error=%d error_name=%s source=%s",
            s.uploadsDisabled ? 1 : 0,
            s.uploadFailureCount,
            static_cast<int>(s.lastError),
            OverlayErrorName(s.lastError),
            source ? source : s.lastSource);
    }
    s.visible = false;
}

void Destroy()
{
    auto& s = State();
    if (!s.created) return;

    if (vr::VROverlay() && s.handle != vr::k_ulOverlayHandleInvalid) {
        vr::VROverlay()->DestroyOverlay(s.handle);
    }
    if (vr::VROverlay()) {
        for (size_t i = 0; i < s.fileMarkerHandles.size(); ++i) {
            if (s.fileMarkerCreated[i] &&
                s.fileMarkerHandles[i] != vr::k_ulOverlayHandleInvalid)
            {
                vr::VROverlay()->DestroyOverlay(s.fileMarkerHandles[i]);
            }
        }
    }
    if (s.glTexture != 0) {
        glDeleteTextures(1, &s.glTexture);
        s.glTexture = 0;
    }
    s.handle = vr::k_ulOverlayHandleInvalid;
    s.fileMarkerHandles.fill(vr::k_ulOverlayHandleInvalid);
    s.fileMarkerCreated.fill(false);
    s.fileMarkerTextureReadyBySlot.fill(false);
    s.created = false;
    s.visible = false;
    s.fileMarkersVisible = false;
    s.fileMarkerTextureReady = false;
    s.textureReady = false;
    s.uploadedHash = 0;
    s.lastRasterHash = 0;
    s.uploadFailureCount = 0;
    s.fileMarkerFailureCount = 0;
    s.uploadsDisabled = false;
    s.lastError = vr::VROverlayError_None;
    s.fileMarkerLastError = vr::VROverlayError_None;
    s.lastVertexCount = 0;
    s.fileMarkerCount = 0;
    openvr_pair::common::DiagnosticLog("boundary-preview", "destroyed");
}

vr::EVROverlayError UploadRasterTexture(const BoundaryPreviewRaster& raster)
{
    auto& s = State();
    if (!vr::VROverlay() || s.handle == vr::k_ulOverlayHandleInvalid) {
        s.lastError = vr::VROverlayError_InvalidHandle;
        return s.lastError;
    }
    if (raster.rgba.size() <
        static_cast<size_t>(BoundaryPreviewRaster::kTextureSize) *
            static_cast<size_t>(BoundaryPreviewRaster::kTextureSize) * 4u)
    {
        s.lastError = vr::VROverlayError_InvalidParameter;
        return s.lastError;
    }

    // Upload the CPU raster into a GL texture and hand SteamVR that texture, the
    // same way the dashboard overlay does (VrOverlayHost::SubmitTexture). The old
    // path, SetOverlayRaw, ships the pixels as an IPC byte stream that has a size
    // cap; a 512x512x4 (1 MB) buffer exceeds it and the call returns
    // VROverlayError_RequestFailed every time. The boundary tick runs on the
    // overlay's main thread with the GL context current (the same thread that
    // submits the dashboard texture), so SetOverlayTexture is valid here.
    constexpr GLsizei size =
        static_cast<GLsizei>(BoundaryPreviewRaster::kTextureSize);
    if (s.glTexture == 0) {
        glGenTextures(1, &s.glTexture);
        if (s.glTexture == 0) {
            // No GL texture (e.g. no current context): report a failure so the
            // caller keeps the marker fallback visible and retries.
            s.lastError = vr::VROverlayError_RequestFailed;
            return s.lastError;
        }
        glBindTexture(GL_TEXTURE_2D, s.glTexture);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, size, size, 0,
            GL_RGBA, GL_UNSIGNED_BYTE, raster.rgba.data());
    } else {
        glBindTexture(GL_TEXTURE_2D, s.glTexture);
        glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, size, size,
            GL_RGBA, GL_UNSIGNED_BYTE, raster.rgba.data());
    }
    glBindTexture(GL_TEXTURE_2D, 0);

    vr::Texture_t tex{};
    tex.handle = reinterpret_cast<void*>(static_cast<uintptr_t>(s.glTexture));
    tex.eType = vr::TextureType_OpenGL;
    tex.eColorSpace = vr::ColorSpace_Auto;
    const vr::EVROverlayError err =
        vr::VROverlay()->SetOverlayTexture(s.handle, &tex);
    s.lastError = err;
    return err;
}

bool EnsureFileMarkerCreated(size_t index)
{
    auto& s = State();
    if (index >= s.fileMarkerHandles.size()) return false;
    if (s.fileMarkerCreated[index]) return s.fileMarkerTextureReadyBySlot[index];
    if (!vr::VROverlay()) return false;

    const std::string texturePath = ResolveFileMarkerTexturePath();
    if (texturePath.empty()) {
        s.fileMarkerLastError = vr::VROverlayError_UnableToLoadFile;
        s.fileMarkerTextureReady = false;
        ++s.fileMarkerFailureCount;
        if (s.fileMarkerFailureCount <= 3 || (s.fileMarkerFailureCount % 30) == 0) {
            openvr_pair::common::DiagnosticLog(
                "boundary_preview_status",
                "file_markers_visible=%d file_markers=%zu file_texture_ready=0 file_failures=%d file_error=%d file_error_name=%s source=%s texture_missing=1",
                s.fileMarkersVisible ? 1 : 0,
                s.fileMarkerCount,
                s.fileMarkerFailureCount,
                static_cast<int>(s.fileMarkerLastError),
                OverlayErrorName(s.fileMarkerLastError),
                s.lastSource);
        }
        return false;
    }

    char key[96];
    std::snprintf(key, sizeof key, "wkopenvr.boundary.preview.marker.%02zu", index);
    char name[96];
    std::snprintf(name, sizeof name, "Boundary preview marker %02zu", index);
    vr::EVROverlayError err =
        vr::VROverlay()->CreateOverlay(key, name, &s.fileMarkerHandles[index]);
    if (err == vr::VROverlayError_KeyInUse) {
        err = vr::VROverlay()->FindOverlay(key, &s.fileMarkerHandles[index]);
    }
    if (err != vr::VROverlayError_None) {
        s.fileMarkerLastError = err;
        ++s.fileMarkerFailureCount;
        if (s.fileMarkerFailureCount <= 3 || (s.fileMarkerFailureCount % 30) == 0) {
            openvr_pair::common::DiagnosticLog(
                "boundary_preview_status",
                "file_marker_create_failed index=%zu error=%d error_name=%s failures=%d source=%s",
                index,
                static_cast<int>(err),
                OverlayErrorName(err),
                s.fileMarkerFailureCount,
                s.lastSource);
        }
        return false;
    }

    vr::VROverlay()->SetOverlaySortOrder(s.fileMarkerHandles[index], 20 + static_cast<uint32_t>(index));
    vr::VROverlay()->SetOverlayAlpha(s.fileMarkerHandles[index], 0.92f);
    err = vr::VROverlay()->SetOverlayFromFile(s.fileMarkerHandles[index], texturePath.c_str());
    s.fileMarkerCreated[index] = true;
    s.fileMarkerTextureReadyBySlot[index] = (err == vr::VROverlayError_None);
    s.fileMarkerLastError = err;
    s.fileMarkerTextureReady =
        s.fileMarkerTextureReady || s.fileMarkerTextureReadyBySlot[index];
    if (err != vr::VROverlayError_None) {
        ++s.fileMarkerFailureCount;
    }
    openvr_pair::common::DiagnosticLog(
        "boundary_preview_status",
        "file_marker_created index=%zu texture_ready=%d error=%d error_name=%s source=%s texture='%s'",
        index,
        s.fileMarkerTextureReady ? 1 : 0,
        static_cast<int>(err),
        OverlayErrorName(err),
        s.lastSource,
        texturePath.c_str());
    return s.fileMarkerTextureReady;
}

void HideFileMarkers(const char* source)
{
    auto& s = State();
    if (!s.fileMarkersVisible && s.fileMarkerCount == 0) return;
    if (vr::VROverlay()) {
        for (size_t i = 0; i < s.fileMarkerHandles.size(); ++i) {
            if (s.fileMarkerCreated[i] &&
                s.fileMarkerHandles[i] != vr::k_ulOverlayHandleInvalid)
            {
                vr::VROverlay()->HideOverlay(s.fileMarkerHandles[i]);
            }
        }
    }
    openvr_pair::common::DiagnosticLog(
        "boundary_preview_status",
        "file_markers_visible=0 file_markers=0 file_texture_ready=%d file_failures=%d file_error=%d file_error_name=%s source=%s",
        s.fileMarkerTextureReady ? 1 : 0,
        s.fileMarkerFailureCount,
        static_cast<int>(s.fileMarkerLastError),
        OverlayErrorName(s.fileMarkerLastError),
        source ? source : s.lastSource);
    s.fileMarkersVisible = false;
    s.fileMarkerCount = 0;
}

float MarkerWidthMeters(const SpatialStyle& style)
{
    const double requested = style.dotMeters > 0.0 ? style.dotMeters * 1.35 : 0.075;
    return static_cast<float>(std::clamp(requested, 0.045, 0.18));
}

void TickFileMarkers(
    const std::vector<SpatialRenderCommand>& commands,
    const char* source)
{
    auto& s = State();
    const std::vector<BoundaryPreviewFileMarker> markers =
        BuildBoundaryPreviewFileMarkers(commands);
    if (markers.empty()) {
        HideFileMarkers(source ? source : "no_file_markers");
        return;
    }

    size_t shown = 0;
    const size_t activeSlots = std::min(markers.size(), s.fileMarkerHandles.size());
    for (size_t i = 0; i < activeSlots; ++i) {
        if (!EnsureFileMarkerCreated(i)) continue;

        const auto& marker = markers[i];
        const float r = static_cast<float>(marker.style.r) / 255.0f;
        const float g = static_cast<float>(marker.style.g) / 255.0f;
        const float b = static_cast<float>(marker.style.b) / 255.0f;
        const float a = static_cast<float>(marker.style.a) / 255.0f;
        vr::VROverlay()->SetOverlayColor(s.fileMarkerHandles[i], r, g, b);
        vr::VROverlay()->SetOverlayAlpha(s.fileMarkerHandles[i], std::clamp(a, 0.25f, 1.0f));
        vr::VROverlay()->SetOverlayWidthInMeters(
            s.fileMarkerHandles[i],
            MarkerWidthMeters(marker.style));
        const vr::HmdMatrix34_t mat = BoundaryPreviewFileMarkerTransform(marker.vertex);
        vr::EVROverlayError err = vr::VROverlay()->SetOverlayTransformAbsolute(
            s.fileMarkerHandles[i],
            BoundaryPreviewTrackingOrigin(),
            &mat);
        if (err == vr::VROverlayError_None) {
            err = vr::VROverlay()->ShowOverlay(s.fileMarkerHandles[i]);
        }
        if (err != vr::VROverlayError_None) {
            if (vr::VROverlay() &&
                s.fileMarkerHandles[i] != vr::k_ulOverlayHandleInvalid)
            {
                vr::VROverlay()->HideOverlay(s.fileMarkerHandles[i]);
            }
            s.fileMarkerLastError = err;
            ++s.fileMarkerFailureCount;
            if (s.fileMarkerFailureCount <= 3 || (s.fileMarkerFailureCount % 30) == 0) {
                openvr_pair::common::DiagnosticLog(
                    "boundary_preview_status",
                    "file_marker_show_failed index=%zu error=%d error_name=%s failures=%d source=%s pos=(%.3f,%.3f,%.3f)",
                    i,
                    static_cast<int>(err),
                    OverlayErrorName(err),
                    s.fileMarkerFailureCount,
                    source ? source : s.lastSource,
                    marker.vertex.x,
                    marker.vertex.y,
                    marker.vertex.z);
            }
            continue;
        }
        ++shown;
    }

    for (size_t i = activeSlots; i < s.fileMarkerHandles.size(); ++i) {
        if (s.fileMarkerCreated[i] &&
            s.fileMarkerHandles[i] != vr::k_ulOverlayHandleInvalid)
        {
            vr::VROverlay()->HideOverlay(s.fileMarkerHandles[i]);
        }
    }

    const bool wasVisible = s.fileMarkersVisible;
    const size_t previousCount = s.fileMarkerCount;
    s.fileMarkersVisible = shown > 0;
    s.fileMarkerCount = shown;
    if (shown > 0) {
        s.fileMarkerLastError = vr::VROverlayError_None;
    }
    if (s.fileMarkersVisible != wasVisible || previousCount != shown) {
        openvr_pair::common::DiagnosticLog(
            "boundary_preview_status",
            "file_markers_visible=%d file_markers=%zu file_texture_ready=%d file_failures=%d file_error=%d file_error_name=%s source=%s raw_visible=%d raw_disabled=%d",
            s.fileMarkersVisible ? 1 : 0,
            s.fileMarkerCount,
            s.fileMarkerTextureReady ? 1 : 0,
            s.fileMarkerFailureCount,
            static_cast<int>(s.fileMarkerLastError),
            OverlayErrorName(s.fileMarkerLastError),
            source ? source : s.lastSource,
            s.visible ? 1 : 0,
            s.uploadsDisabled ? 1 : 0);
    }
}

double WorldMetersToPixels(double spanMeters, double meters, double minPixels, double maxPixels)
{
    const double pixels = meters / std::max(spanMeters, 0.001) *
        static_cast<double>(BoundaryPreviewRaster::kTextureSize - 1);
    return std::clamp(pixels, minPixels, maxPixels);
}

} // namespace

BoundaryPreviewPlane ComputeBoundaryPreviewPlane(
    const std::vector<BoundaryVertex>& vertices)
{
    BoundaryPreviewPlane out;
    if (vertices.empty()) return out;

    const PolygonBounds bounds = ComputePolygonBoundsXZ(vertices);
    const double rangeX = bounds.xMax - bounds.xMin;
    const double rangeZ = bounds.zMax - bounds.zMin;
    const double range = std::max(rangeX, rangeZ);

    out.valid = true;
    out.centerX = (bounds.xMin + bounds.xMax) * 0.5;
    out.centerZ = (bounds.zMin + bounds.zMax) * 0.5;
    out.spanMeters = std::max(kMinPreviewSpanMeters, range + kPreviewPadMeters * 2.0);
    return out;
}

BoundaryPreviewPlane ComputeBoundaryPreviewPlane(
    const std::vector<SpatialRenderCommand>& commands)
{
    std::vector<BoundaryVertex> allVertices;
    for (const SpatialRenderCommand& command : commands) {
        allVertices.insert(
            allVertices.end(),
            command.standingVertices.begin(),
            command.standingVertices.end());
    }
    return ComputeBoundaryPreviewPlane(allVertices);
}

BoundaryPreviewRaster BuildBoundaryPreviewRaster(
    const std::vector<BoundaryVertex>& vertices,
    bool closeLoop)
{
    SpatialSession session = BoundaryCaptureSessionDescriptor(
        StandingSpace(),
        -1,
        {},
        vertices.empty() ? 0.0 : vertices.front().y,
        false,
        0);
    SpatialPrimitive primitive = BoundaryPathPrimitive(session, vertices, closeLoop);
    return BuildBoundaryPreviewRaster(BuildSpatialRenderCommands({ primitive }));
}

BoundaryPreviewRaster BuildBoundaryPreviewRaster(
    const std::vector<SpatialRenderCommand>& commands)
{
    BoundaryPreviewRaster raster;
    raster.plane = ComputeBoundaryPreviewPlane(commands);
    raster.rgba.assign(
        static_cast<size_t>(BoundaryPreviewRaster::kTextureSize) *
            static_cast<size_t>(BoundaryPreviewRaster::kTextureSize) * 4u,
        0u);

    uint64_t hash = HashU64(kFnvOffset, commands.size());
    for (const SpatialRenderCommand& command : commands) {
        hash = HashRenderCommand(hash, command);
    }
    raster.hash = hash;

    if (!raster.plane.valid) return raster;

    const double half = raster.plane.spanMeters * 0.5;
    const double minX = raster.plane.centerX - half;
    const double maxZ = raster.plane.centerZ + half;
    const double scale = static_cast<double>(BoundaryPreviewRaster::kTextureSize - 1) /
        raster.plane.spanMeters;
    auto toPixel = [&](const BoundaryVertex& v) {
        const int x = ClampPixel((v.x - minX) * scale);
        const int y = ClampPixel((maxZ - v.z) * scale);
        return std::pair<int, int>(x, y);
    };

    for (const SpatialRenderCommand& command : commands) {
        const auto& vertices = command.standingVertices;
        if (vertices.empty()) {
            continue;
        }

        std::vector<std::pair<int, int>> pixelPoints;
        pixelPoints.reserve(vertices.size());
        for (const auto& v : vertices) {
            pixelPoints.push_back(toPixel(v));
        }

        if (command.closeLoop && command.style.fill && vertices.size() >= 3) {
            FillPolygon(raster.rgba, pixelPoints, command.style);
        }

        const double strokeRadius = command.style.strokeMeters > 0.0
            ? WorldMetersToPixels(
                raster.plane.spanMeters,
                command.style.strokeMeters,
                2.0,
                18.0)
            : 0.0;
        const double strokeFeather = command.style.strokeMeters > 0.0
            ? WorldMetersToPixels(
                raster.plane.spanMeters,
                command.style.strokeMeters * 0.40,
                1.5,
                8.0)
            : 0.0;
        const double dotRadius = command.style.dotMeters > 0.0
            ? WorldMetersToPixels(
                raster.plane.spanMeters,
                command.style.dotMeters,
                4.0,
                22.0)
            : 0.0;

        for (size_t i = 1; i < vertices.size(); ++i) {
            SpatialStyle segmentStyle = command.style;
            if (command.ageFade) {
                const uint8_t shade = BoundaryAgeShade(i, vertices.size());
                segmentStyle.r = shade;
                segmentStyle.g = shade;
                segmentStyle.b = shade;
            }
            DrawBoundarySegment(
                raster.rgba,
                pixelPoints[i - 1],
                pixelPoints[i],
                false,
                strokeRadius,
                strokeFeather,
                segmentStyle);
        }
        if (command.closeLoop && vertices.size() >= 3) {
            DrawBoundarySegment(
                raster.rgba,
                pixelPoints.back(),
                pixelPoints.front(),
                true,
                strokeRadius,
                strokeFeather,
                command.style);
        }

        for (size_t i = 0; i < vertices.size(); ++i) {
            SpatialStyle vertexStyle = command.style;
            if (command.ageFade) {
                const uint8_t shade = BoundaryAgeShade(i, vertices.size());
                vertexStyle.r = shade;
                vertexStyle.g = shade;
                vertexStyle.b = shade;
            }
            DrawBoundaryVertex(
                raster.rgba,
                pixelPoints[i],
                i == 0,
                i + 1 == vertices.size(),
                dotRadius,
                vertexStyle);
        }
    }
    return raster;
}

int BoundaryPreviewUploadFailureDisableThreshold()
{
    return kUploadFailureDisableThreshold;
}

double BoundaryUploadBackoffSeconds(int consecutiveFailures)
{
    if (consecutiveFailures <= 0) return 0.0;
    const int shift = std::min(consecutiveFailures - 1, 6);
    const double secs = kUploadBackoffBaseSec * static_cast<double>(1 << shift);
    return std::min(secs, kUploadBackoffCapSec);
}

bool BoundaryPreviewShouldDisableUploadsAfterFailureCount(int failureCount)
{
    return failureCount >= kUploadFailureDisableThreshold;
}

bool BoundaryPreviewUsesOpenGlTextureUpload()
{
    return true;
}

int BoundaryPreviewFileMarkerLimit()
{
    return kMaxFileMarkerOverlays;
}

std::vector<BoundaryPreviewFileMarker> BuildBoundaryPreviewFileMarkers(
    const std::vector<SpatialRenderCommand>& commands)
{
    std::vector<BoundaryPreviewFileMarker> explicitMarkers;
    std::vector<BoundaryPreviewFileMarker> pathMarkers;
    for (const SpatialRenderCommand& command : commands) {
        const bool isExplicitMarker = command.kind == SpatialPrimitiveKind::Marker;
        for (const BoundaryVertex& vertex : command.standingVertices) {
            BoundaryPreviewFileMarker marker;
            marker.vertex = vertex;
            marker.style = command.style;
            if (marker.style.dotMeters <= 0.0) {
                marker.style.dotMeters = isExplicitMarker ? 0.075 : 0.055;
            }
            if (isExplicitMarker) {
                explicitMarkers.push_back(marker);
            } else {
                pathMarkers.push_back(marker);
            }
        }
    }

    std::vector<BoundaryPreviewFileMarker>& source =
        explicitMarkers.empty() ? pathMarkers : explicitMarkers;
    if (source.size() <= kMaxFileMarkerOverlays) {
        return source;
    }

    std::vector<BoundaryPreviewFileMarker> sampled;
    sampled.reserve(kMaxFileMarkerOverlays);
    const double last = static_cast<double>(source.size() - 1);
    for (int i = 0; i < kMaxFileMarkerOverlays; ++i) {
        const double t = static_cast<double>(i) /
            static_cast<double>(kMaxFileMarkerOverlays - 1);
        const size_t idx = static_cast<size_t>(std::llround(t * last));
        sampled.push_back(source[std::min(idx, source.size() - 1)]);
    }
    return sampled;
}

vr::HmdMatrix34_t BoundaryPreviewFileMarkerTransform(
    const BoundaryVertex& vertex)
{
    return BoundaryPreviewTransform(vertex.x, vertex.y, vertex.z);
}

BoundaryPreviewStatus GetBoundaryPreviewStatus()
{
    const auto& s = State();
    BoundaryPreviewStatus status;
    status.created = s.created;
    status.visible = s.visible;
    status.uploadsDisabled = s.uploadsDisabled;
    status.fileMarkersVisible = s.fileMarkersVisible;
    status.fileMarkerTextureReady = s.fileMarkerTextureReady;
    status.textureReady = s.textureReady;
    status.uploadFailureCount = s.uploadFailureCount;
    status.fileMarkerFailureCount = s.fileMarkerFailureCount;
    status.lastError = static_cast<int>(s.lastError);
    status.fileMarkerLastError = static_cast<int>(s.fileMarkerLastError);
    status.lastErrorName = OverlayErrorName(s.lastError);
    status.fileMarkerLastErrorName = OverlayErrorName(s.fileMarkerLastError);
    status.uploadedHash = s.uploadedHash;
    status.lastRasterHash = s.lastRasterHash;
    status.lastVertexCount = s.lastVertexCount;
    status.fileMarkerCount = s.fileMarkerCount;
    status.lastSource = s.lastSource ? s.lastSource : "none";
    status.plane = s.lastPlane;
    return status;
}

void ResetBoundaryPreviewUploadFailures()
{
    auto& s = State();
    const bool wasDisabled = s.uploadsDisabled || s.uploadFailureCount > 0;
    s.uploadFailureCount = 0;
    s.uploadsDisabled = false;
    s.lastError = vr::VROverlayError_None;
    s.uploadedHash = 0;
    if (wasDisabled) {
        openvr_pair::common::DiagnosticLog(
            "boundary_preview_status",
            "created=%d visible=%d uploads_disabled=0 failures=0 error=0 error_name=None source=reset",
            s.created ? 1 : 0,
            s.visible ? 1 : 0);
    }
}

vr::ETrackingUniverseOrigin BoundaryPreviewTrackingOrigin()
{
    return vr::TrackingUniverseStanding;
}

vr::HmdMatrix34_t BoundaryPreviewTransform(
    double centerX,
    double floorY,
    double centerZ)
{
    vr::HmdMatrix34_t mat{};
    // Rotate the overlay onto the floor: local X -> world X, local Y -> -world Z,
    // local +Z normal -> world up.
    mat.m[0][0] = 1.0f;
    mat.m[0][1] = 0.0f;
    mat.m[0][2] = 0.0f;
    mat.m[0][3] = static_cast<float>(centerX);

    mat.m[1][0] = 0.0f;
    mat.m[1][1] = 0.0f;
    mat.m[1][2] = 1.0f;
    mat.m[1][3] = static_cast<float>(floorY + 0.025);

    mat.m[2][0] = 0.0f;
    mat.m[2][1] = -1.0f;
    mat.m[2][2] = 0.0f;
    mat.m[2][3] = static_cast<float>(centerZ);
    return mat;
}

void TickBoundaryPreview(
    bool wantVisible,
    const std::vector<BoundaryVertex>& vertices,
    double floorY,
    bool closeLoop,
    const char* source)
{
    SpatialSession session = BoundaryCaptureSessionDescriptor(
        StandingSpace(),
        -1,
        {},
        floorY,
        false,
        0);
    SpatialPrimitive primitive = BoundaryPathPrimitive(session, vertices, closeLoop);
    TickBoundaryPreview(
        wantVisible,
        BuildSpatialRenderCommands({ primitive }),
        floorY,
        source);
}

void TickBoundaryPreview(
    bool wantVisible,
    const std::vector<SpatialRenderCommand>& commands,
    double floorY,
    const char* source,
    bool showFileMarkers)
{
    auto& s = State();
    s.lastSource = source ? source : "unknown";
    size_t vertexCount = 0;
    for (const auto& command : commands) {
        vertexCount += command.standingVertices.size();
    }
    s.lastVertexCount = vertexCount;

    if (!wantVisible || commands.empty()) {
        HideOnly(source ? source : "hidden");
        HideFileMarkers(source ? source : "hidden");
        return;
    }

    BoundaryPreviewRaster raster = BuildBoundaryPreviewRaster(commands);
    s.lastRasterHash = raster.hash;
    s.lastPlane = raster.plane;
    if (!raster.plane.valid) {
        HideOnly(source ? source : "invalid_plane");
        HideFileMarkers(source ? source : "invalid_plane");
        return;
    }
    if (!EnsureCreated()) {
        HideFileMarkers(source ? source : "no_overlay");
        return;
    }

    // Upload the current raster when its content changes. A failed
    // SetOverlayTexture (e.g. a transient RequestFailed while drawing) must
    // NEVER permanently disable a safety boundary, so failures only schedule a
    // short, growing backoff and we always retry. Freshly drawn/applied content
    // (a new hash that hasn't failed) clears the backoff for an immediate clean
    // attempt.
    const bool contentChanged = (raster.hash != s.uploadedHash);
    if (contentChanged && raster.hash != s.lastFailedHash) {
        s.uploadFailureCount = 0;
        s.nextUploadAttemptSec = 0.0;
    }
    if (contentChanged && NowSeconds() >= s.nextUploadAttemptSec) {
        const vr::EVROverlayError err = UploadRasterTexture(raster);
        if (err == vr::VROverlayError_None) {
            s.uploadedHash = raster.hash;
            s.uploadFailureCount = 0;
            s.nextUploadAttemptSec = 0.0;
            s.lastError = vr::VROverlayError_None;
            s.textureReady = true;
            s.uploadsDisabled = false;
        } else {
            s.lastError = err;
            s.textureReady = false;
            ++s.uploadFailureCount;
            s.lastFailedHash = raster.hash;
            // uploadsDisabled now means "in retry backoff" -- transient, never
            // permanent. The time gate above paces retries; success clears it.
            s.uploadsDisabled = true;
            s.nextUploadAttemptSec =
                NowSeconds() + BoundaryUploadBackoffSeconds(s.uploadFailureCount);
            if (s.uploadFailureCount <= 3 || (s.uploadFailureCount % 30) == 0) {
                openvr_pair::common::DiagnosticLog(
                    "boundary-preview",
                    "texture_upload_failed err=%d error_name=%s count=%d source=%s vertices=%zu hash=%llu backoff=%.3f mode=opengl_texture",
                    static_cast<int>(err),
                    OverlayErrorName(err),
                    s.uploadFailureCount,
                    s.lastSource,
                    s.lastVertexCount,
                    static_cast<unsigned long long>(raster.hash),
                    BoundaryUploadBackoffSeconds(s.uploadFailureCount));
            }
        }
    }

    // The overlay only shows the correct filled boundary when the texture for the
    // CURRENT content is uploaded. Until then (first draw, or while retrying after
    // a failure) show vertex markers so the boundary is never invisible. When the
    // caller wants reference markers (capture cursor/origin/HMD) keep them too.
    const bool currentTextureLive = s.textureReady && (s.uploadedHash == raster.hash);
    if (showFileMarkers || !currentTextureLive) {
        TickFileMarkers(commands, source ? source : "file_markers");
    } else {
        HideFileMarkers(source ? source : "markers_suppressed");
    }

    if (!currentTextureLive) {
        // No valid texture for this boundary yet; the marker fallback covers it.
        // Don't show a stale/wrong filled shape.
        HideOnly(source ? source : "await_texture");
        return;
    }

    vr::VROverlay()->SetOverlayWidthInMeters(
        s.handle,
        static_cast<float>(raster.plane.spanMeters));
    vr::HmdMatrix34_t mat = BoundaryPreviewTransform(
        raster.plane.centerX,
        floorY,
        raster.plane.centerZ);
    vr::VROverlay()->SetOverlayTransformAbsolute(
        s.handle,
        BoundaryPreviewTrackingOrigin(),
        &mat);
    vr::VROverlay()->ShowOverlay(s.handle);
    if (!s.visible) {
        openvr_pair::common::DiagnosticLog(
            "boundary_preview_status",
            "created=1 visible=1 uploads_disabled=0 failures=%d error=%d error_name=%s source=%s vertices=%zu span=%.3f center=(%.3f,%.3f) mode=opengl_texture texture_ready=%d",
            s.uploadFailureCount,
            static_cast<int>(s.lastError),
            OverlayErrorName(s.lastError),
            s.lastSource,
            s.lastVertexCount,
            raster.plane.spanMeters,
            raster.plane.centerX,
            raster.plane.centerZ,
            s.textureReady ? 1 : 0);
    }
    s.visible = true;
}

} // namespace wkopenvr::boundary
