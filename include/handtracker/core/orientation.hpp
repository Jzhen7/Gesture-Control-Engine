// 3D hand orientation estimation (port of v1's hand_tracker_bare.py).
#pragma once

#include <array>

#include "handtracker/core/landmarks.hpp"

namespace ht {

struct HandOrientation {
  /// Euler angles in degrees of the hand frame (X = right, Y = up along the
  /// fingers, Z = palm normal) decomposed as Z-Y-X.
  float rotX = 0.f;
  float rotY = 0.f;
  float rotZ = 0.f;
  /// Twist of the palm around the finger axis in degrees (palm-forward vs back-of-hand).
  float twist = 0.f;
  Vec3 palmNormal;
  Vec3 up;
  /// Row-major 3x3 rotation matrix whose columns are the hand axes.
  std::array<float, 9> matrix{};
};

/// Uses metric world landmarks when available (more reliable depth), otherwise
/// the image landmarks.
HandOrientation computeOrientation(const HandLandmarks& hand);

}  // namespace ht
