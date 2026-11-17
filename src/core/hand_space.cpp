#include "handtracker/core/hand_space.hpp"

#include <algorithm>
#include <cmath>

namespace ht {

Vec3 palmCenterNormalized(const HandLandmarks& hand) {
  return (hand.normalized(LandmarkId::Wrist) + hand.normalized(LandmarkId::MiddleMcp)) * 0.5f;
}

float palmSizeNormalized(const HandLandmarks& hand) {
  return hand.palmSize() / std::max(hand.imageWidth, 1.f);
}

Vec3 handPosition(const HandLandmarks& hand, const HandSpaceCalibration& cal,
                  const HandSpaceConfig& cfg) {
  const Vec3 c = palmCenterNormalized(hand);
  const float size = std::max(palmSizeNormalized(hand), 1e-6f);
  const float ref = std::max(cal.palmSize, 1e-6f);
  // Apparent size is inversely proportional to distance, so a log ratio gives a
  // symmetric scale: twice as big and half as big are equally far from centre.
  const float depth = std::log(size / ref);
  return {(c.x - cal.centerX) / cfg.rangeX,
          -(c.y - cal.centerY) / cfg.rangeY,  // image y points down; we want up
          depth / cfg.depthRange};
}

HandCalibrator::HandCalibrator(double durationMs, float maxMotion)
    : durationMs_(durationMs), maxMotion_(maxMotion) {}

void HandCalibrator::reset() {
  holding_ = false;
  samples_ = 0;
  sumX_ = sumY_ = sumSize_ = 0.0;
}

float HandCalibrator::progress(double tMs) const {
  if (!holding_ || durationMs_ <= 0.0) return 0.f;
  return static_cast<float>(std::clamp((tMs - startT_) / durationMs_, 0.0, 1.0));
}

std::optional<HandSpaceCalibration> HandCalibrator::update(const std::optional<HandLandmarks>& hand,
                                                           Gesture smoothedGesture, double tMs) {
  if (!hand || smoothedGesture != Gesture::Palm) {
    reset();
    return std::nullopt;
  }
  const Vec3 c = palmCenterNormalized(*hand);
  if (!holding_ || std::hypot(c.x - anchorX_, c.y - anchorY_) > maxMotion_) {
    reset();  // Start (or restart, if the hand moved) the hold.
    holding_ = true;
    startT_ = tMs;
    anchorX_ = c.x;
    anchorY_ = c.y;
  }
  sumX_ += c.x;
  sumY_ += c.y;
  sumSize_ += palmSizeNormalized(*hand);
  ++samples_;

  if (tMs - startT_ < durationMs_) return std::nullopt;
  HandSpaceCalibration cal;
  cal.centerX = static_cast<float>(sumX_ / samples_);
  cal.centerY = static_cast<float>(sumY_ / samples_);
  cal.palmSize = static_cast<float>(sumSize_ / samples_);
  reset();
  return cal;
}

}  // namespace ht
