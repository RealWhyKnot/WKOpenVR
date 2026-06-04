#include "BoundaryCapture.h"
#include "Boundary.h"
#include "CalibrationMetrics.h"

#include <cmath>
#include <cstddef>
#include <cstdio>
#include <algorithm>

namespace wkopenvr::boundary {

namespace {

double DistanceSq(const BoundaryVertex& a, const BoundaryVertex& b)
{
	const double dx = a.x - b.x;
	const double dy = a.y - b.y;
	const double dz = a.z - b.z;
	return dx * dx + dy * dy + dz * dz;
}

double SegmentDistanceSq(const BoundaryVertex& p, const BoundaryVertex& a, const BoundaryVertex& b)
{
	const double dx = b.x - a.x;
	const double dy = b.y - a.y;
	const double dz = b.z - a.z;
	const double lenSq = dx * dx + dy * dy + dz * dz;
	const double px = p.x - a.x;
	const double py = p.y - a.y;
	const double pz = p.z - a.z;
	if (lenSq < 1e-20) {
		return px * px + py * py + pz * pz;
	}
	double t = (px * dx + py * dy + pz * dz) / lenSq;
	if (t < 0.0) t = 0.0;
	if (t > 1.0) t = 1.0;
	const double rx = px - t * dx;
	const double ry = py - t * dy;
	const double rz = pz - t * dz;
	return rx * rx + ry * ry + rz * rz;
}

double CrossXZ(const BoundaryVertex& a, const BoundaryVertex& b, const BoundaryVertex& c)
{
	return (b.x - a.x) * (c.z - a.z) - (b.z - a.z) * (c.x - a.x);
}

bool SameXZ(const BoundaryVertex& a, const BoundaryVertex& b)
{
	return std::fabs(a.x - b.x) <= 1e-6 && std::fabs(a.z - b.z) <= 1e-6;
}

bool IsFiniteVertex(const BoundaryVertex& v)
{
	return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z);
}

std::vector<BoundaryVertex> FilterFiniteVertices(const std::vector<BoundaryVertex>& raw)
{
	std::vector<BoundaryVertex> out;
	out.reserve(raw.size());
	for (const auto& v : raw) {
		if (IsFiniteVertex(v)) out.push_back(v);
	}
	return out;
}

double SignedAreaXZ(const std::vector<BoundaryVertex>& poly)
{
	if (poly.size() < 3) return 0.0;
	double twiceArea = 0.0;
	for (size_t i = 0, j = poly.size() - 1; i < poly.size(); j = i++) {
		twiceArea += poly[j].x * poly[i].z - poly[i].x * poly[j].z;
	}
	return twiceArea * 0.5;
}

void NormalizeWindingXZ(std::vector<BoundaryVertex>& poly)
{
	if (SignedAreaXZ(poly) < 0.0) {
		std::reverse(poly.begin(), poly.end());
	}
}

int OrientationXZ(const BoundaryVertex& a, const BoundaryVertex& b, const BoundaryVertex& c)
{
	const double cross = CrossXZ(a, b, c);
	if (std::fabs(cross) <= 1e-9) return 0;
	return cross > 0.0 ? 1 : -1;
}

bool OnSegmentXZ(const BoundaryVertex& a, const BoundaryVertex& b, const BoundaryVertex& p)
{
	if (OrientationXZ(a, b, p) != 0) return false;
	return p.x >= std::min(a.x, b.x) - 1e-9 && p.x <= std::max(a.x, b.x) + 1e-9 && p.z >= std::min(a.z, b.z) - 1e-9 &&
	       p.z <= std::max(a.z, b.z) + 1e-9;
}

bool SegmentsIntersectXZ(const BoundaryVertex& a, const BoundaryVertex& b, const BoundaryVertex& c,
                         const BoundaryVertex& d)
{
	const int o1 = OrientationXZ(a, b, c);
	const int o2 = OrientationXZ(a, b, d);
	const int o3 = OrientationXZ(c, d, a);
	const int o4 = OrientationXZ(c, d, b);

	if (o1 != o2 && o3 != o4) return true;
	if (o1 == 0 && OnSegmentXZ(a, b, c)) return true;
	if (o2 == 0 && OnSegmentXZ(a, b, d)) return true;
	if (o3 == 0 && OnSegmentXZ(c, d, a)) return true;
	if (o4 == 0 && OnSegmentXZ(c, d, b)) return true;
	return false;
}

bool HasSelfIntersectionXZ(const std::vector<BoundaryVertex>& poly)
{
	const size_t n = poly.size();
	if (n < 4) return false;
	for (size_t i = 0; i < n; ++i) {
		const size_t iNext = (i + 1) % n;
		for (size_t j = i + 1; j < n; ++j) {
			const size_t jNext = (j + 1) % n;
			if (i == j || iNext == j || jNext == i) continue;
			if (i == 0 && jNext == 0) continue;
			if (SegmentsIntersectXZ(poly[i], poly[iNext], poly[j], poly[jNext])) {
				return true;
			}
		}
	}
	return false;
}

std::vector<BoundaryVertex> ConvexHullXZ(std::vector<BoundaryVertex> points)
{
	std::sort(points.begin(), points.end(), [](const BoundaryVertex& a, const BoundaryVertex& b) {
		if (a.x != b.x) return a.x < b.x;
		if (a.z != b.z) return a.z < b.z;
		return a.y < b.y;
	});
	points.erase(std::unique(points.begin(), points.end(), SameXZ), points.end());
	if (points.size() <= 3) return points;

	std::vector<BoundaryVertex> hull;
	hull.reserve(points.size() * 2);
	for (const auto& p : points) {
		while (hull.size() >= 2 && CrossXZ(hull[hull.size() - 2], hull[hull.size() - 1], p) <= 0.0) {
			hull.pop_back();
		}
		hull.push_back(p);
	}

	const size_t lowerSize = hull.size();
	for (size_t idx = points.size(); idx-- > 0;) {
		const auto& p = points[idx];
		while (hull.size() > lowerSize && CrossXZ(hull[hull.size() - 2], hull[hull.size() - 1], p) <= 0.0) {
			hull.pop_back();
		}
		hull.push_back(p);
	}
	if (!hull.empty()) {
		hull.pop_back();
	}
	return hull;
}

enum class FloorProjectionStatus
{
	Ok,
	NonFinite,
	NotDownward,
	DistanceOutOfRange,
};

struct FloorProjectionAttempt
{
	FloorProjectionStatus status = FloorProjectionStatus::NonFinite;
	BoundaryVertex hit = {};
	const char* rayName = "-Z";
	double originY = 0.0;
	double aimY = 0.0;
	double distanceMeters = 0.0;
};

const char* ProjectionStatusName(FloorProjectionStatus status)
{
	switch (status) {
		case FloorProjectionStatus::Ok:
			return "ok";
		case FloorProjectionStatus::NonFinite:
			return "non_finite";
		case FloorProjectionStatus::NotDownward:
			return "not_downward";
		case FloorProjectionStatus::DistanceOutOfRange:
			return "distance_out_of_range";
	}
	return "unknown";
}

FloorProjectionAttempt ProjectSingleRayToFloor(const Eigen::Affine3d& controllerPose, const Eigen::Vector3d& localRay,
                                               const char* rayName, double floorY)
{
	FloorProjectionAttempt attempt;
	attempt.rayName = rayName;

	const Eigen::Vector3d origin = controllerPose.translation();
	const Eigen::Vector3d aim = (controllerPose.rotation() * localRay).normalized();
	attempt.originY = origin.y();
	attempt.aimY = aim.y();

	if (!origin.allFinite() || !aim.allFinite()) {
		attempt.status = FloorProjectionStatus::NonFinite;
		return attempt;
	}

	if (aim.y() > -0.20) {
		attempt.status = FloorProjectionStatus::NotDownward;
		return attempt;
	}

	const double distanceMeters = (floorY - origin.y()) / aim.y();
	attempt.distanceMeters = distanceMeters;
	if (!std::isfinite(distanceMeters) || distanceMeters < 0.05 || distanceMeters > 5.0) {
		attempt.status = FloorProjectionStatus::DistanceOutOfRange;
		return attempt;
	}

	const Eigen::Vector3d hit = origin + aim * distanceMeters;
	if (!hit.allFinite()) {
		attempt.status = FloorProjectionStatus::NonFinite;
		return attempt;
	}

	attempt.status = FloorProjectionStatus::Ok;
	attempt.hit = {hit.x(), floorY, hit.z()};
	return attempt;
}

FloorProjectionAttempt ProjectAimToFloor(const Eigen::Affine3d& controllerPose, double floorY)
{
	// OpenVR controller poses are not guaranteed to expose the model pointer
	// ray on the same local axis/sign across controller drivers. Keep -Z first
	// for the historical path, then try every cardinal local axis so an Index
	// controller that reports a different pointer convention can still paint.
	struct CandidateRay
	{
		Eigen::Vector3d ray;
		const char* name;
	};
	const CandidateRay rays[] = {
	    {Eigen::Vector3d(0.0, 0.0, -1.0), "-Z"}, {Eigen::Vector3d(0.0, 0.0, 1.0), "+Z"},
	    {Eigen::Vector3d(0.0, -1.0, 0.0), "-Y"}, {Eigen::Vector3d(0.0, 1.0, 0.0), "+Y"},
	    {Eigen::Vector3d(-1.0, 0.0, 0.0), "-X"}, {Eigen::Vector3d(1.0, 0.0, 0.0), "+X"},
	};

	FloorProjectionAttempt bestOk;
	bestOk.aimY = 1.0;
	bool haveOk = false;
	FloorProjectionAttempt bestFailed;
	bestFailed.aimY = 1.0;
	bool haveFailed = false;
	for (const auto& r : rays) {
		const FloorProjectionAttempt attempt = ProjectSingleRayToFloor(controllerPose, r.ray, r.name, floorY);
		if (attempt.status == FloorProjectionStatus::Ok) {
			if (!haveOk || attempt.aimY < bestOk.aimY) {
				bestOk = attempt;
				haveOk = true;
			}
			continue;
		}
		if (!haveFailed || attempt.aimY < bestFailed.aimY) {
			bestFailed = attempt;
			haveFailed = true;
		}
	}
	if (haveOk) {
		return bestOk;
	}

	return bestFailed;
}

FloorProjectionAttempt ProjectPointerPoseToFloor(const Eigen::Affine3d& pointerPose, double floorY)
{
	return ProjectSingleRayToFloor(pointerPose, Eigen::Vector3d(0.0, 0.0, -1.0), "tip:-Z", floorY);
}

FloorHitPreview ToFloorHitPreview(const FloorProjectionAttempt& attempt)
{
	FloorHitPreview preview;
	preview.valid = attempt.status == FloorProjectionStatus::Ok;
	preview.hit = attempt.hit;
	preview.rayName = attempt.rayName;
	return preview;
}

std::vector<BoundaryVertex> RemoveNearDuplicates(const std::vector<BoundaryVertex>& raw, double minDistanceMeters)
{
	std::vector<BoundaryVertex> out;
	out.reserve(raw.size());

	const double minDistSq = minDistanceMeters * minDistanceMeters;
	for (const auto& v : raw) {
		if (!out.empty() && DistanceSq(out.back(), v) < minDistSq) {
			continue;
		}
		out.push_back(v);
	}
	return out;
}

std::vector<BoundaryVertex> RemoveClosedShortEdges(const std::vector<BoundaryVertex>& raw, double minDistanceMeters)
{
	std::vector<BoundaryVertex> out = RemoveNearDuplicates(raw, minDistanceMeters);
	const double minDistSq = minDistanceMeters * minDistanceMeters;
	while (out.size() > 3 && DistanceSq(out.front(), out.back()) < minDistSq) {
		out.pop_back();
	}
	return out;
}

std::vector<BoundaryVertex> RemoveClosedCollinearVertices(const std::vector<BoundaryVertex>& raw,
                                                          double toleranceMeters)
{
	std::vector<BoundaryVertex> out = raw;
	const double toleranceSq = toleranceMeters * toleranceMeters;

	bool changed = true;
	while (changed && out.size() > 3) {
		changed = false;
		for (size_t i = 0; i < out.size(); ++i) {
			const BoundaryVertex& prev = out[(i + out.size() - 1) % out.size()];
			const BoundaryVertex& cur = out[i];
			const BoundaryVertex& next = out[(i + 1) % out.size()];
			if (SegmentDistanceSq(cur, prev, next) <= toleranceSq) {
				out.erase(out.begin() + static_cast<std::ptrdiff_t>(i));
				changed = true;
				break;
			}
		}
	}

	return out;
}

std::vector<BoundaryVertex> TrimToNearestClosure(const std::vector<BoundaryVertex>& raw, double closeLoopMeters)
{
	if (raw.size() < 6) {
		return raw;
	}

	const BoundaryVertex& last = raw.back();
	const double closeSq = closeLoopMeters * closeLoopMeters;
	size_t bestIndex = raw.size();
	double bestDistSq = closeSq;
	for (size_t i = 0; i + 3 < raw.size(); ++i) {
		const double d = DistanceSq(raw[i], last);
		if (d <= bestDistSq) {
			bestDistSq = d;
			bestIndex = i;
		}
	}

	if (bestIndex == raw.size()) {
		return raw;
	}

	std::vector<BoundaryVertex> out;
	out.reserve(raw.size() - bestIndex);
	for (size_t i = bestIndex; i < raw.size(); ++i) {
		out.push_back(raw[i]);
	}
	return out;
}

std::vector<BoundaryVertex> CleanPaintedLoop(const std::vector<BoundaryVertex>& raw, double debounceMeters,
                                             double closeLoopMeters, double simplifyMeters)
{
	std::vector<BoundaryVertex> path = RemoveNearDuplicates(FilterFiniteVertices(raw), debounceMeters);
	if (path.size() < 3) {
		return path;
	}

	path = TrimToNearestClosure(path, closeLoopMeters);
	if (path.size() < 3) {
		return path;
	}

	const double closeSq = closeLoopMeters * closeLoopMeters;
	while (path.size() >= 3 && DistanceSq(path.front(), path.back()) < closeSq) {
		path.pop_back();
	}

	if (path.size() < 3) {
		return path;
	}

	auto kept = SimplifyDouglasPeucker(path, simplifyMeters);
	std::vector<BoundaryVertex> simplified;
	simplified.reserve(kept.size());
	for (size_t idx : kept) {
		simplified.push_back(path[idx]);
	}

	simplified = RemoveClosedShortEdges(simplified, debounceMeters);
	if (simplified.size() >= 3 && DistanceSq(simplified.front(), simplified.back()) < closeSq) {
		simplified.pop_back();
	}
	simplified = RemoveClosedCollinearVertices(simplified, simplifyMeters);
	if (HasSelfIntersectionXZ(simplified)) {
		simplified = ConvexHullXZ(simplified);
		simplified = RemoveClosedShortEdges(simplified, debounceMeters);
		simplified = RemoveClosedCollinearVertices(simplified, simplifyMeters);
	}
	if (std::fabs(SignedAreaXZ(simplified)) < 0.05) {
		return {};
	}
	NormalizeWindingXZ(simplified);
	return simplified;
}

} // namespace

void CaptureSession::Start()
{
	m_raw.clear();
	m_simplified.clear();
	m_projectionRejectLogCount = 0;
	m_debounceRejectLogCount = 0;
	++m_sessionId;
	m_state = CaptureState::Active;
	Metrics::WriteLogAnnotation("[boundary-capture] started");
}

void CaptureSession::Cancel()
{
	m_raw.clear();
	m_simplified.clear();
	m_state = CaptureState::Idle;
	Metrics::WriteLogAnnotation("[boundary-capture] cancelled");
}

void CaptureSession::Finish()
{
	if (m_state != CaptureState::Active) return;
	m_simplified = CleanPaintedLoop(m_raw, kVertexDebounceMeters, kCloseLoopMeters, kSimplifyEpsilonMeters);
	m_state = CaptureState::Finished;
	{
		char lbuf[96];
		snprintf(lbuf, sizeof lbuf, "[boundary-capture] finished: raw=%zu simplified=%zu", m_raw.size(),
		         m_simplified.size());
		Metrics::WriteLogAnnotation(lbuf);
	}
}

bool CaptureSession::Tick(const Eigen::Affine3d& controllerPose, bool triggerHeld, double floorY)
{
	return AppendProjection(controllerPose, triggerHeld, floorY, false);
}

bool CaptureSession::TickPointerPose(const Eigen::Affine3d& pointerPose, bool triggerHeld, double floorY)
{
	return AppendProjection(pointerPose, triggerHeld, floorY, true);
}

bool CaptureSession::TickProjectedPosition(const Eigen::Affine3d& controllerPose, bool active, double floorY)
{
	return AppendProjectedPosition(controllerPose, active, floorY);
}

FloorHitPreview CaptureSession::PreviewControllerFloorHit(const Eigen::Affine3d& controllerPose, double floorY) const
{
	return ToFloorHitPreview(ProjectAimToFloor(controllerPose, floorY));
}

FloorHitPreview CaptureSession::PreviewPointerFloorHit(const Eigen::Affine3d& pointerPose, double floorY) const
{
	return ToFloorHitPreview(ProjectPointerPoseToFloor(pointerPose, floorY));
}

bool CaptureSession::AppendProjectedPosition(const Eigen::Affine3d& controllerPose, bool active, double floorY)
{
	if (m_state != CaptureState::Active) return false;
	if (!active) return false;

	const Eigen::Vector3d origin = controllerPose.translation();
	if (!origin.allFinite() || !std::isfinite(floorY)) {
		++m_projectionRejectLogCount;
		if (m_projectionRejectLogCount == 1 || (m_projectionRejectLogCount % 120) == 0) {
			Metrics::WriteLogAnnotation("[boundary-capture] projected position rejected: non_finite");
		}
		return false;
	}

	const BoundaryVertex candidate{origin.x(), floorY, origin.z()};
	if (!m_raw.empty()) {
		const BoundaryVertex& last = m_raw.back();
		const double dist = std::sqrt(DistanceSq(candidate, last));
		if (dist < kVertexDebounceMeters) {
			++m_debounceRejectLogCount;
			if (m_debounceRejectLogCount == 1 || (m_debounceRejectLogCount % 120) == 0) {
				char dbuf[192];
				snprintf(dbuf, sizeof dbuf,
				         "[boundary-capture] vertex debounced: dist=%.3f min=%.3f raw=%zu rejects=%zu ray=controllerXZ",
				         dist, kVertexDebounceMeters, m_raw.size(), m_debounceRejectLogCount);
				Metrics::WriteLogAnnotation(dbuf);
			}
			return false;
		}
	}

	if (m_raw.empty() || ((m_raw.size() + 1) % 20) == 0) {
		char lbuf[128];
		snprintf(lbuf, sizeof lbuf, "[boundary-capture] vertex added: index=%zu pos=(%.3f,%.3f,%.3f) ray=controllerXZ",
		         m_raw.size(), candidate.x, candidate.y, candidate.z);
		Metrics::WriteLogAnnotation(lbuf);
	}
	m_raw.push_back(candidate);
	return true;
}

bool CaptureSession::AppendProjection(const Eigen::Affine3d& poseForLog, bool triggerHeld, double floorY,
                                      bool pointerOnly)
{
	if (m_state != CaptureState::Active) return false;
	if (!triggerHeld) return false;

	const FloorProjectionAttempt projection =
	    pointerOnly ? ProjectPointerPoseToFloor(poseForLog, floorY) : ProjectAimToFloor(poseForLog, floorY);
	if (projection.status != FloorProjectionStatus::Ok) {
		++m_projectionRejectLogCount;
		if (m_projectionRejectLogCount == 1 || (m_projectionRejectLogCount % 120) == 0) {
			char lbuf[224];
			snprintf(lbuf, sizeof lbuf,
			         "[boundary-capture] floor projection rejected: reason=%s ray=%s origin_y=%.3f aim_y=%.3f "
			         "distance=%.3f rejects=%zu",
			         ProjectionStatusName(projection.status), projection.rayName, projection.originY, projection.aimY,
			         projection.distanceMeters, m_projectionRejectLogCount);
			Metrics::WriteLogAnnotation(lbuf);
		}
		return false;
	}

	const BoundaryVertex& candidate = projection.hit;
	if (!m_raw.empty()) {
		const BoundaryVertex& last = m_raw.back();
		const double dist = std::sqrt(DistanceSq(candidate, last));
		if (dist < kVertexDebounceMeters) {
			++m_debounceRejectLogCount;
			if (m_debounceRejectLogCount == 1 || (m_debounceRejectLogCount % 120) == 0) {
				char dbuf[192];
				snprintf(dbuf, sizeof dbuf,
				         "[boundary-capture] vertex debounced: dist=%.3f min=%.3f raw=%zu rejects=%zu ray=%s", dist,
				         kVertexDebounceMeters, m_raw.size(), m_debounceRejectLogCount, projection.rayName);
				Metrics::WriteLogAnnotation(dbuf);
			}
			return false;
		}
	}

	if (m_raw.empty() || ((m_raw.size() + 1) % 20) == 0) {
		char lbuf[128];
		snprintf(lbuf, sizeof lbuf, "[boundary-capture] vertex added: index=%zu pos=(%.3f,%.3f,%.3f) ray=%s",
		         m_raw.size(), candidate.x, candidate.y, candidate.z, projection.rayName);
		Metrics::WriteLogAnnotation(lbuf);
	}
	m_raw.push_back(candidate);
	return true;
}

const std::vector<BoundaryVertex>& CaptureSession::vertices() const
{
	if (m_state == CaptureState::Finished) return m_simplified;
	return m_raw;
}

} // namespace wkopenvr::boundary
