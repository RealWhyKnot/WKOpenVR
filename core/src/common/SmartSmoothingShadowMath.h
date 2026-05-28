#pragma once

#include "PredictionSmoothingMath.h"

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace prediction::smart_shadow {

constexpr double kPi = 3.1415926535897932384626433832795;

inline double Clamp(double value, double lo, double hi)
{
    return std::min(hi, std::max(lo, value));
}

inline double Saturate(double value)
{
    return Clamp(value, 0.0, 1.0);
}

inline double Lerp(double a, double b, double t)
{
    return a + (b - a) * Saturate(t);
}

inline double SmoothStep(double edge0, double edge1, double x)
{
    if (!(edge1 > edge0)) return x >= edge1 ? 1.0 : 0.0;
    const double t = Saturate((x - edge0) / (edge1 - edge0));
    return t * t * (3.0 - 2.0 * t);
}

inline double AlphaFromCutoffHz(double cutoffHz, double dtSeconds)
{
    cutoffHz = Clamp(cutoffHz, 0.001, 120.0);
    dtSeconds = Clamp(dtSeconds, 0.001, 0.050);
    const double tau = 1.0 / (2.0 * kPi * cutoffHz);
    return dtSeconds / (dtSeconds + tau);
}

inline double AlphaFromTau(double tauSeconds, double dtSeconds)
{
    tauSeconds = std::max(0.0005, tauSeconds);
    dtSeconds = Clamp(dtSeconds, 0.001, 0.050);
    return dtSeconds / (dtSeconds + tauSeconds);
}

inline double UpdateEnvelope(double current,
                             double target,
                             double dtSeconds,
                             double attackTauSeconds,
                             double releaseTauSeconds)
{
    target = Saturate(target);
    const double tau = target > current ? attackTauSeconds : releaseTauSeconds;
    const double alpha = AlphaFromTau(tau, dtSeconds);
    return Saturate(current + alpha * (target - current));
}

inline double ReleasedPredictionFactor(double baseFactor, double release)
{
    baseFactor = Saturate(baseFactor);
    release = Saturate(release);
    return baseFactor + (1.0 - baseFactor) * release;
}

struct Params {
    double posMinCutoffHz = 12.0;
    double rotMinCutoffHz = 16.0;
    double posBetaHzPerMps = 6.0;
    double rotBetaHzPerRadps = 1.5;
    double derivCutoffHz = 4.0;
    double maxCutoffHz = 90.0;
    double basePredictionFactor = 1.0;

    double linDerivedDeadband = 0.03;
    double linReportedDeadband = 0.06;
    double angDerivedDeadband = 0.08;
    double angReportedDeadband = 0.15;

    double linStillSpeed = 0.03;
    double linMovingSpeed = 0.35;
    double angStillSpeed = 0.10;
    double angMovingSpeed = 1.20;

    double innovationTauSeconds = 0.08;
    double linInnovationDeadband = 0.0;
    double angInnovationDeadband = 0.0;
    double posErrStillM = 0.012;
    double posErrMovingM = 0.060;
    double rotErrStillRad = 0.01308996938995747;  // 0.75 deg
    double rotErrMovingRad = 0.10471975511965977; // 6 deg

    double gateAttackTauSeconds = 0.006;
    double gateReleaseTauSeconds = 0.045;

    double positionJumpM = 0.50;
    double rotationJumpRad = 1.0471975511965976; // 60 deg
    double resetGapSeconds = 0.25;
};

inline Params BuildParams(uint8_t smoothness)
{
    const double s01 = Clamp(static_cast<double>(smoothness) / 100.0, 0.0, 1.0);
    const double inv = 1.0 - s01;

    Params p;
    p.posMinCutoffHz = 0.45 + 16.0 * std::pow(inv, 1.8);
    p.rotMinCutoffHz = 0.75 + 18.0 * std::pow(inv, 1.6);
    p.posBetaHzPerMps = Lerp(6.0, 24.0, s01);
    p.rotBetaHzPerRadps = Lerp(1.5, 4.5, s01);
    p.basePredictionFactor = prediction::SmoothnessToFactor(smoothness);
    return p;
}

} // namespace prediction::smart_shadow
