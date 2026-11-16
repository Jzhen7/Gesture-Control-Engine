// Mappings from hand pose to continuous control values.
#pragma once

#include "handtracker/core/landmarks.hpp"

namespace ht {

/// Linearly maps v from [inLo, inHi] to [0, 1] and clamps. inLo may exceed inHi.
float mapToUnit(float v, float inLo, float inHi);

/// Hand roll in degrees: 0 = fingers pointing straight up, positive = leaning
/// towards image-right, negative = leaning towards image-left.
float handRollDegrees(const HandLandmarks& hand);

/// Volume dial: lean the hand left to lower and right to raise the volume.
float volumeFromRoll(float rollDegrees, float minDegrees, float maxDegrees);

/// Brightness slider: raise the hand to brighten. top/bottom are normalised image y.
float brightnessFromHand(const HandLandmarks& hand, float top, float bottom);

/// Apparent hand size relative to frame width: a proxy for distance to camera.
float handDepthProxy(const HandLandmarks& hand);

/// Thumb-tip to index-tip distance in palm sizes (small = pinching).
float pinchRatio(const HandLandmarks& hand);

}  // namespace ht
