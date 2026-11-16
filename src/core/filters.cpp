#include "handtracker/core/filters.hpp"

#include <algorithm>
#include <cmath>

namespace ht {

namespace {
constexpr double kPi = 3.14159265358979323846;
}

OneEuroFilter::OneEuroFilter(double minCutoff, double beta, double derivativeCutoff)
    : minCutoff_(minCutoff), beta_(beta), dCutoff_(derivativeCutoff) {}

double OneEuroFilter::alpha(double cutoff, double dt) {
  const double tau = 1.0 / (2.0 * kPi * cutoff);
  return 1.0 / (1.0 + tau / dt);
}

double OneEuroFilter::filter(double value, double tSeconds) {
  if (!initialised_) {
    initialised_ = true;
    prevValue_ = value;
    prevDeriv_ = 0.0;
    prevT_ = tSeconds;
    return value;
  }
  double dt = tSeconds - prevT_;
  if (dt <= 0.0) dt = 1e-3;  // Duplicate timestamps: assume a tiny step.
  prevT_ = tSeconds;

  const double deriv = (value - prevValue_) / dt;
  const double ad = alpha(dCutoff_, dt);
  prevDeriv_ = ad * deriv + (1.0 - ad) * prevDeriv_;

  const double cutoff = minCutoff_ + beta_ * std::fabs(prevDeriv_);
  const double a = alpha(cutoff, dt);
  prevValue_ = a * value + (1.0 - a) * prevValue_;
  return prevValue_;
}

void OneEuroFilter::reset() { initialised_ = false; }

LevelFilter::LevelFilter(LevelFilterConfig cfg) : cfg_(cfg) {}

std::optional<float> LevelFilter::update(float raw, double tMs) {
  raw = std::clamp(raw, 0.f, 1.f);
  if (!hasValue_) {
    value_ = raw;
    hasValue_ = true;
  } else {
    value_ += cfg_.smoothing * (raw - value_);
  }

  if (hasApplied_) {
    if (tMs - appliedT_ < cfg_.minIntervalMs) return std::nullopt;
    if (std::fabs(value_ - applied_) < cfg_.minDelta) return std::nullopt;
  }
  hasApplied_ = true;
  applied_ = value_;
  appliedT_ = tMs;
  return value_;
}

void LevelFilter::reset() {
  hasValue_ = false;
  hasApplied_ = false;
}

}  // namespace ht
