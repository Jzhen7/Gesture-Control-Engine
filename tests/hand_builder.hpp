// Generates synthetic but anatomically plausible hand landmarks, so gesture
// logic can be tested deterministically without a camera or a model.
#pragma once

#include <cmath>

#include "handtracker/core/landmarks.hpp"

namespace testutil {

struct HandSpec {
  bool thumb = false;
  bool index = false;
  bool middle = false;
  bool ring = false;
  bool pinky = false;
};

struct Pose {
  float x = 320.f;         ///< Wrist position, pixels.
  float y = 380.f;
  float scale = 100.f;     ///< Wrist -> knuckles distance, pixels.
  float rotationDeg = 0.f; ///< In-plane rotation, positive = clockwise on screen.
  bool mirrored = false;   ///< Swap left/right (back of hand / other hand).
};

inline ht::HandLandmarks makeHand(const HandSpec& s, const Pose& pose = {}) {
  using ht::Vec3;
  // Local frame: wrist at origin, fingers point towards -y, unit = palm length.
  std::array<Vec3, 21> p{};
  p[0] = {0.f, 0.f};
  if (s.thumb) {
    p[1] = {0.20f, -0.15f}; p[2] = {0.45f, -0.35f}; p[3] = {0.65f, -0.50f}; p[4] = {0.85f, -0.62f};
  } else {  // Tucked across the palm.
    p[1] = {0.20f, -0.15f}; p[2] = {0.35f, -0.40f}; p[3] = {0.25f, -0.65f}; p[4] = {0.00f, -0.72f};
  }
  struct Finger { float x, y, len; bool up; };
  const Finger fingers[4] = {
      {0.25f, -1.00f, 1.0f, s.index},
      {0.00f, -1.05f, 1.1f, s.middle},
      {-0.22f, -1.00f, 1.0f, s.ring},
      {-0.42f, -0.90f, 0.8f, s.pinky},
  };
  for (int f = 0; f < 4; ++f) {
    const Finger& fg = fingers[f];
    const int base = 5 + 4 * f;
    p[base] = {fg.x, fg.y};
    if (fg.up) {
      p[base + 1] = {fg.x, fg.y - 0.40f * fg.len};
      p[base + 2] = {fg.x, fg.y - 0.65f * fg.len};
      p[base + 3] = {fg.x, fg.y - 0.85f * fg.len};
    } else {  // Curled down towards the palm.
      p[base + 1] = {fg.x, fg.y - 0.25f * fg.len};
      p[base + 2] = {fg.x, fg.y - 0.05f * fg.len};
      p[base + 3] = {fg.x, fg.y + 0.15f * fg.len};
    }
  }

  const float a = pose.rotationDeg * 3.14159265f / 180.f;
  const float c = std::cos(a);
  const float sn = std::sin(a);
  ht::HandLandmarks h;
  h.imageWidth = 640.f;
  h.imageHeight = 480.f;
  h.score = 1.f;
  for (std::size_t i = 0; i < 21; ++i) {
    float lx = p[i].x * pose.scale;
    const float ly = p[i].y * pose.scale;
    if (pose.mirrored) lx = -lx;
    h.image[i] = {pose.x + lx * c - ly * sn, pose.y + lx * sn + ly * c, 0.f};
  }
  return h;
}

constexpr HandSpec kFist{false, false, false, false, false};
constexpr HandSpec kPeace{false, true, true, false, false};
constexpr HandSpec kMiddle{false, false, true, false, false};
constexpr HandSpec kOne{true, false, false, false, false};
constexpr HandSpec kTwo{true, true, false, false, false};
constexpr HandSpec kThree{true, true, true, false, false};
constexpr HandSpec kFour{true, true, true, true, false};
constexpr HandSpec kPalm{true, true, true, true, true};

}  // namespace testutil
