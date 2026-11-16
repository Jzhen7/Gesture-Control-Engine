#include "handtracker/core/mapping.hpp"

#include <algorithm>
#include <cmath>

namespace ht {

namespace {
constexpr float kRadToDeg = 57.29577951308232f;
}

float mapToUnit(float v, float inLo, float inHi) {
  if (inHi == inLo) return 0.f;
  return std::clamp((v - inLo) / (inHi - inLo), 0.f, 1.f);
}

float handRollDegrees(const HandLandmarks& hand) {
  const Vec3 d = hand.normalized(LandmarkId::MiddleMcp) - hand.normalized(LandmarkId::Wrist);
  // Correct for aspect ratio so the angle is geometric, then measure from "up" (-y).
  const float dx = d.x * hand.imageWidth;
  const float dy = d.y * hand.imageHeight;
  return std::atan2(dx, -dy) * kRadToDeg;
}

float volumeFromRoll(float rollDegrees, float minDegrees, float maxDegrees) {
  return mapToUnit(rollDegrees, minDegrees, maxDegrees);
}

float brightnessFromHand(const HandLandmarks& hand, float top, float bottom) {
  // Palm centre: halfway between the wrist and the middle knuckle.
  const float y =
      0.5f * (hand.normalized(LandmarkId::Wrist).y + hand.normalized(LandmarkId::MiddleMcp).y);
  return mapToUnit(y, bottom, top);
}

float handDepthProxy(const HandLandmarks& hand) {
  return hand.palmSize() / std::max(hand.imageWidth, 1.f);
}

float pinchRatio(const HandLandmarks& hand) {
  const float palm = std::max(hand.palmSize(), 1e-6f);
  return distance(hand[LandmarkId::ThumbTip], hand[LandmarkId::IndexTip]) / palm;
}

}  // namespace ht
