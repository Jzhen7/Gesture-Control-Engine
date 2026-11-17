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

float fingertipSpread(const HandLandmarks& hand) {
  const auto& pts = hand.world ? *hand.world : hand.image;
  auto at = [&](LandmarkId id) { return pts[static_cast<std::size_t>(id)]; };
  // Palm length (wrist -> middle knuckle) in the same units as the points, so
  // the result doesn't depend on distance from the camera or hand size.
  const float palm = std::max(distance(at(LandmarkId::Wrist), at(LandmarkId::MiddleMcp)), 1e-6f);
  // Average over all 10 pairs of the five fingertips. Measuring only thumb ->
  // fingertips let an "OK sign" (thumb on the index alone) read as closed,
  // because a flat hand's fingertips already sit close to each other.
  constexpr LandmarkId kTips[] = {LandmarkId::ThumbTip, LandmarkId::IndexTip,
                                  LandmarkId::MiddleTip, LandmarkId::RingTip,
                                  LandmarkId::PinkyTip};
  float sum = 0.f;
  int pairs = 0;
  for (int i = 0; i < 5; ++i) {
    for (int j = i + 1; j < 5; ++j, ++pairs) sum += distance(at(kTips[i]), at(kTips[j]));
  }
  return sum / static_cast<float>(pairs) / palm;
}

float fingerStraightness(const HandLandmarks& hand) {
  const auto& pts = hand.world ? *hand.world : hand.image;
  constexpr LandmarkId kKnuckles[] = {LandmarkId::IndexMcp, LandmarkId::MiddleMcp,
                                      LandmarkId::RingMcp, LandmarkId::PinkyMcp};
  // The *least* straight finger decides: an average let one fully curled
  // finger hide behind three straight ones (mean 0.82 vs a 0.85 threshold).
  float least = 1.f;
  for (LandmarkId mcp : kKnuckles) {
    const auto k = static_cast<std::size_t>(mcp);  // knuckle, then middle joint, end joint, tip
    const float bones = distance(pts[k], pts[k + 1]) + distance(pts[k + 1], pts[k + 2]) +
                        distance(pts[k + 2], pts[k + 3]);
    least = std::min(least, distance(pts[k], pts[k + 3]) / std::max(bones, 1e-6f));
  }
  return least;
}

bool isClawPose(const HandLandmarks& hand, const GripConfig& cfg) {
  return fingerStraightness(hand) >= cfg.minStraightness;
}

float handOpenness(const HandLandmarks& hand, const HandSpaceCalibration& cal,
                   const GripConfig& cfg) {
  // Guard against a calibration palm that was barely more open than a closed claw.
  const float span = std::max(cal.openSpread - cfg.closedSpread, 0.1f);
  const float raw = (fingertipSpread(hand) - cfg.closedSpread) / span;
  const float d = std::clamp(cfg.deadband, 0.f, 0.45f);
  return std::clamp((raw - d) / (1.f - 2.f * d), 0.f, 1.f);
}

HandCalibrator::HandCalibrator(double durationMs, float maxMotion)
    : durationMs_(durationMs), maxMotion_(maxMotion) {}

void HandCalibrator::reset() {
  holding_ = false;
  samples_ = 0;
  sumX_ = sumY_ = sumSize_ = sumSpread_ = 0.0;
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
  sumSpread_ += fingertipSpread(*hand);
  ++samples_;

  if (tMs - startT_ < durationMs_) return std::nullopt;
  HandSpaceCalibration cal;
  cal.centerX = static_cast<float>(sumX_ / samples_);
  cal.centerY = static_cast<float>(sumY_ / samples_);
  cal.palmSize = static_cast<float>(sumSize_ / samples_);
  cal.openSpread = static_cast<float>(sumSpread_ / samples_);
  reset();
  return cal;
}

}  // namespace ht
