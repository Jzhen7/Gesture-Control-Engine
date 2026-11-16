// Signal filters for turning jittery landmark readings into usable controls.
#pragma once

#include <optional>

namespace ht {

/// One Euro filter (Casiez et al., CHI 2012): an adaptive low-pass filter that
/// smooths heavily when the signal is slow (killing jitter) and lightly when it
/// moves fast (keeping latency low). Ideal for cursor control.
class OneEuroFilter {
 public:
  OneEuroFilter(double minCutoff = 1.0, double beta = 0.0, double derivativeCutoff = 1.0);

  double filter(double value, double tSeconds);
  void reset();

 private:
  static double alpha(double cutoff, double dt);

  double minCutoff_;
  double beta_;
  double dCutoff_;
  bool initialised_ = false;
  double prevValue_ = 0.0;
  double prevDeriv_ = 0.0;
  double prevT_ = 0.0;
};

struct LevelFilterConfig {
  float smoothing = 0.35f;      ///< EMA weight of the newest sample (1 = no smoothing).
  float minDelta = 0.01f;       ///< Ignore changes smaller than this.
  double minIntervalMs = 50.0;  ///< Apply at most this often.
};

/// Smooths a 0..1 control value and rate-limits how often it is pushed to the OS.
class LevelFilter {
 public:
  explicit LevelFilter(LevelFilterConfig cfg = {});

  /// Returns the value to apply now, or nullopt if nothing should be sent yet.
  std::optional<float> update(float raw, double tMs);
  float value() const { return value_; }
  bool hasValue() const { return hasValue_; }
  void reset();

 private:
  LevelFilterConfig cfg_;
  bool hasValue_ = false;
  float value_ = 0.f;
  bool hasApplied_ = false;
  float applied_ = 0.f;
  double appliedT_ = 0.0;
};

}  // namespace ht
