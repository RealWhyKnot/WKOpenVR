#include "VergenceLock.h"

#include <algorithm>
#include <cmath>

namespace facetracking {

namespace {

struct Vec3
{
	float x, y, z;
};

inline float Dot(Vec3 a, Vec3 b)
{
	return a.x * b.x + a.y * b.y + a.z * b.z;
}
inline Vec3 Sub(Vec3 a, Vec3 b)
{
	return {a.x - b.x, a.y - b.y, a.z - b.z};
}
inline Vec3 Add(Vec3 a, Vec3 b)
{
	return {a.x + b.x, a.y + b.y, a.z + b.z};
}
inline Vec3 Scale(Vec3 a, float s)
{
	return {a.x * s, a.y * s, a.z * s};
}

inline Vec3 Normalize(Vec3 v)
{
	float len = std::sqrt(Dot(v, v));
	if (len < 1e-9f) return v;
	return Scale(v, 1.f / len);
}

inline Vec3 Lerp(Vec3 a, Vec3 b, float t)
{
	return {a.x + t * (b.x - a.x), a.y + t * (b.y - a.y), a.z + t * (b.z - a.z)};
}

inline float Length(Vec3 v)
{
	return std::sqrt(Dot(v, v));
}

} // namespace

void VergenceLock::Apply(protocol::FaceTrackingFrameBody& frame, uint8_t strength_0_to_100)
{
	if (!(frame.flags & 1u)) return; // eye fields not valid
	if (strength_0_to_100 == 0) return;

	float k = (float)strength_0_to_100 / 100.f;

	const Vec3 o_L = {frame.eye_origin_l[0], frame.eye_origin_l[1], frame.eye_origin_l[2]};
	const Vec3 o_R = {frame.eye_origin_r[0], frame.eye_origin_r[1], frame.eye_origin_r[2]};
	Vec3 d_L = Normalize({frame.eye_gaze_l[0], frame.eye_gaze_l[1], frame.eye_gaze_l[2]});
	Vec3 d_R = Normalize({frame.eye_gaze_r[0], frame.eye_gaze_r[1], frame.eye_gaze_r[2]});

	// Update IPD estimate whenever both origins are valid.
	last_ipd_m_ = Length(Sub(o_L, o_R));

	// Eye-dropout fallback: if one eye's confidence is below 0.3, drive both
	// gaze directions from the surviving eye (better than independent jitter
	// from a low-confidence sensor feeding the reconstruction separately).
	const float cL = frame.eye_confidence_l;
	const float cR = frame.eye_confidence_r;
	const bool lDrop = cL < 0.3f;
	const bool rDrop = cR < 0.3f;

	if (lDrop && rDrop) return; // both eyes dead -- do nothing

	// Single-eye dropout: lerp the dropped eye's gaze toward the surviving
	// eye's direction and return early.  The skew-line solver cannot be used
	// here because both rays would be parallel (same direction), which would
	// trip the parallel-gaze guard and leave the frame untouched.
	if (lDrop) {
		Vec3 d_L_out = Normalize(Lerp(d_L, d_R, k));
		frame.eye_gaze_l[0] = d_L_out.x;
		frame.eye_gaze_l[1] = d_L_out.y;
		frame.eye_gaze_l[2] = d_L_out.z;
		return;
	}
	if (rDrop) {
		Vec3 d_R_out = Normalize(Lerp(d_R, d_L, k));
		frame.eye_gaze_r[0] = d_R_out.x;
		frame.eye_gaze_r[1] = d_R_out.y;
		frame.eye_gaze_r[2] = d_R_out.z;
		return;
	}

	// Skew-line midpoint (Jain & Chlamtac style, standard photogrammetry).
	Vec3 r = Sub(o_L, o_R);
	float a = Dot(d_L, d_L); // = 1 since directions are unit; kept for clarity
	float b = Dot(d_L, d_R);
	float c = Dot(d_R, d_R); // = 1
	float d = Dot(d_L, r);
	float e = Dot(d_R, r);
	float denom = a * c - b * b;

	// Parallel-gaze guard: |denom| < 1e-6 means both rays are nearly
	// collinear and the intersection is numerically undefined.
	if (denom < 1e-6f) return;

	float t_L = (b * e - c * d) / denom;
	float t_R = (a * e - b * d) / denom;

	// Confidence-disagreement penalty: if t_L and t_R disagree by more than
	// 50% of their average magnitude the intersection is unreliable.  Linearly
	// reduce effective strength toward 0 as disagreement approaches 100%.
	float avg_t = 0.5f * (std::abs(t_L) + std::abs(t_R) + 1e-9f);
	float rel_disagree = std::abs(std::abs(t_L) - std::abs(t_R)) / avg_t;
	if (rel_disagree > 0.5f) {
		float scale = 1.f - (rel_disagree - 0.5f) / 0.5f;
		scale = std::max(0.f, std::min(1.f, scale));
		k *= scale;
		if (k < 1e-4f) return;
	}

	Vec3 p_L = Add(o_L, Scale(d_L, t_L));
	Vec3 p_R = Add(o_R, Scale(d_R, t_R));
	Vec3 focus = Scale(Add(p_L, p_R), 0.5f);

	// Clamp focus distance to [0.10 m, 20 m] from the midpoint between eyes.
	Vec3 eyeMid = Scale(Add(o_L, o_R), 0.5f);
	Vec3 toFocus = Sub(focus, eyeMid);
	float dist = Length(toFocus);
	last_focus_m_ = dist; // record before clamping / early-return checks

	if (dist < 0.10f) {
		// Non-physiological close distance -- scale focus out to 0.10 m.
		if (dist < 1e-6f) return;
		focus = Add(eyeMid, Scale(Scale(toFocus, 1.f / dist), 0.10f));
	}
	else if (dist > 20.f) {
		// Treat as parallel gaze.
		return;
	}

	// Reconstruct per-eye gaze directions pointing at focus.
	Vec3 d_L_new = Normalize(Sub(focus, o_L));
	Vec3 d_R_new = Normalize(Sub(focus, o_R));

	// Lerp original directions toward the reconstructed ones.
	Vec3 d_L_out = Normalize(Lerp(d_L, d_L_new, k));
	Vec3 d_R_out = Normalize(Lerp(d_R, d_R_new, k));

	frame.eye_gaze_l[0] = d_L_out.x;
	frame.eye_gaze_l[1] = d_L_out.y;
	frame.eye_gaze_l[2] = d_L_out.z;
	frame.eye_gaze_r[0] = d_R_out.x;
	frame.eye_gaze_r[1] = d_R_out.y;
	frame.eye_gaze_r[2] = d_R_out.z;
}

} // namespace facetracking
