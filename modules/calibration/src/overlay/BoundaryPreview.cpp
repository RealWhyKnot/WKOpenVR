#include "BoundaryPreview.h"

#include "Boundary.h"
#include "DiagnosticsLog.h"

#include <openvr.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <utility>

namespace wkopenvr::boundary {
namespace {

constexpr double kMinPreviewSpanMeters = 1.0;
constexpr double kPreviewPadMeters = 0.30;
constexpr uint64_t kFnvOffset = 1469598103934665603ull;
constexpr uint64_t kFnvPrime = 1099511628211ull;

struct PreviewState {
    vr::VROverlayHandle_t handle = vr::k_ulOverlayHandleInvalid;
    bool created = false;
    uint64_t uploadedHash = 0;
};

PreviewState& State()
{
    static PreviewState s;
    return s;
}

constexpr const char* kPreviewKey = "wkopenvr.boundary.preview";
constexpr const char* kPreviewName = "Boundary drawing preview";

uint64_t HashU64(uint64_t hash, uint64_t value)
{
    hash ^= value;
    hash *= kFnvPrime;
    return hash;
}

uint64_t HashVertex(uint64_t hash, const BoundaryVertex& v)
{
    hash = HashU64(hash, static_cast<uint64_t>(std::llround(v.x * 1000.0)));
    hash = HashU64(hash, static_cast<uint64_t>(std::llround(v.y * 1000.0)));
    hash = HashU64(hash, static_cast<uint64_t>(std::llround(v.z * 1000.0)));
    return hash;
}

int ClampPixel(double value)
{
    const int rounded = static_cast<int>(std::lround(value));
    return std::clamp(rounded, 0, BoundaryPreviewRaster::kTextureSize - 1);
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

void DrawDot(std::vector<uint8_t>& pixels, int cx, int cy, int radius, uint8_t r, uint8_t g, uint8_t b, uint8_t a)
{
    const int radiusSq = radius * radius;
    for (int y = cy - radius; y <= cy + radius; ++y) {
        for (int x = cx - radius; x <= cx + radius; ++x) {
            const int dx = x - cx;
            const int dy = y - cy;
            if (dx * dx + dy * dy <= radiusSq) {
                BlendPixel(pixels, x, y, r, g, b, a);
            }
        }
    }
}

void DrawLine(std::vector<uint8_t>& pixels, int x0, int y0, int x1, int y1, int radius, uint8_t r, uint8_t g, uint8_t b, uint8_t a)
{
    const int dx = x1 - x0;
    const int dy = y1 - y0;
    const int steps = std::max(std::abs(dx), std::abs(dy));
    if (steps <= 0) {
        DrawDot(pixels, x0, y0, radius, r, g, b, a);
        return;
    }

    for (int i = 0; i <= steps; ++i) {
        const double t = static_cast<double>(i) / static_cast<double>(steps);
        const int x = ClampPixel(static_cast<double>(x0) + dx * t);
        const int y = ClampPixel(static_cast<double>(y0) + dy * t);
        DrawDot(pixels, x, y, radius, r, g, b, a);
    }
}

void FillPolygon(std::vector<uint8_t>& pixels, const std::vector<std::pair<int, int>>& points)
{
    if (points.size() < 3) return;

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
                BlendPixel(pixels, x, y, 0, 190, 220, 58);
            }
        }
    }
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
    s.uploadedHash = 0;
    openvr_pair::common::DiagnosticLog("boundary-preview", "created");
    return true;
}

void Destroy()
{
    auto& s = State();
    if (!s.created) return;

    if (vr::VROverlay() && s.handle != vr::k_ulOverlayHandleInvalid) {
        vr::VROverlay()->DestroyOverlay(s.handle);
    }
    s.handle = vr::k_ulOverlayHandleInvalid;
    s.created = false;
    s.uploadedHash = 0;
    openvr_pair::common::DiagnosticLog("boundary-preview", "destroyed");
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

BoundaryPreviewRaster BuildBoundaryPreviewRaster(
    const std::vector<BoundaryVertex>& vertices,
    bool closeLoop)
{
    BoundaryPreviewRaster raster;
    raster.plane = ComputeBoundaryPreviewPlane(vertices);
    raster.rgba.assign(
        static_cast<size_t>(BoundaryPreviewRaster::kTextureSize) *
            static_cast<size_t>(BoundaryPreviewRaster::kTextureSize) * 4u,
        0u);

    uint64_t hash = HashU64(kFnvOffset, vertices.size());
    hash = HashU64(hash, closeLoop ? 1u : 0u);
    for (const auto& v : vertices) {
        hash = HashVertex(hash, v);
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

    std::vector<std::pair<int, int>> pixelPoints;
    pixelPoints.reserve(vertices.size());
    for (const auto& v : vertices) {
        pixelPoints.push_back(toPixel(v));
    }

    if (closeLoop && vertices.size() >= 3) {
        FillPolygon(raster.rgba, pixelPoints);
    }

    for (size_t i = 1; i < vertices.size(); ++i) {
        const auto a = pixelPoints[i - 1];
        const auto b = pixelPoints[i];
        DrawLine(raster.rgba, a.first, a.second, b.first, b.second, 4, 0, 255, 190, 245);
    }
    if (closeLoop && vertices.size() >= 3) {
        const auto a = pixelPoints.back();
        const auto b = pixelPoints.front();
        DrawLine(raster.rgba, a.first, a.second, b.first, b.second, 3, 0, 210, 255, 220);
    }

    for (size_t i = 0; i < vertices.size(); ++i) {
        const auto p = pixelPoints[i];
        const bool last = i + 1 == vertices.size();
        DrawDot(raster.rgba, p.first, p.second, last ? 5 : 3,
            last ? 255 : 0,
            last ? 245 : 220,
            last ? 90 : 160,
            245);
    }
    return raster;
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
    bool closeLoop)
{
    if (!wantVisible || vertices.empty()) {
        Destroy();
        return;
    }

    BoundaryPreviewRaster raster = BuildBoundaryPreviewRaster(vertices, closeLoop);
    if (!raster.plane.valid) {
        Destroy();
        return;
    }
    if (!EnsureCreated()) return;

    auto& s = State();
    if (raster.hash != s.uploadedHash) {
        vr::EVROverlayError err = vr::VROverlay()->SetOverlayRaw(
            s.handle,
            raster.rgba.data(),
            BoundaryPreviewRaster::kTextureSize,
            BoundaryPreviewRaster::kTextureSize,
            4);
        if (err == vr::VROverlayError_None) {
            s.uploadedHash = raster.hash;
        } else {
            openvr_pair::common::DiagnosticLog(
                "boundary-preview", "upload_failed err=%d", static_cast<int>(err));
        }
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
}

} // namespace wkopenvr::boundary
