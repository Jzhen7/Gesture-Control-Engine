#include "handtracker/core/orientation.hpp"

#include <algorithm>
#include <cmath>

namespace ht {

namespace {
constexpr float kRadToDeg = 57.29577951308232f;
}

HandOrientation computeOrientation(const HandLandmarks& hand) {
  const auto& pts = hand.world ? *hand.world : hand.image;
  auto at = [&](LandmarkId id) { return pts[static_cast<std::size_t>(id)]; };

  const Vec3 wrist = at(LandmarkId::Wrist);
  const Vec3 indexMcp = at(LandmarkId::IndexMcp);
  const Vec3 pinkyMcp = at(LandmarkId::PinkyMcp);
  // v1 used the middle fingertip as "up", which swings around whenever the
  // finger curls. The knuckle is rigidly attached to the palm.
  const Vec3 middleMcp = at(LandmarkId::MiddleMcp);

  HandOrientation o;
  const Vec3 z = normalized(cross(indexMcp - wrist, pinkyMcp - wrist));  // palm normal
  Vec3 y = normalized(middleMcp - wrist);                                 // along fingers
  const Vec3 x = normalized(cross(y, z));
  y = normalized(cross(z, x));  // re-orthogonalise
  o.palmNormal = z;
  o.up = y;

  // Columns are the hand axes.
  o.matrix = {x.x, y.x, z.x,
              x.y, y.y, z.y,
              x.z, y.z, z.z};
  const auto& R = o.matrix;
  auto r = [&](int row, int col) { return R[static_cast<std::size_t>(row * 3 + col)]; };

  const float sy = std::sqrt(r(0, 0) * r(0, 0) + r(1, 0) * r(1, 0));
  if (sy > 1e-6f) {
    o.rotX = std::atan2(r(2, 1), r(2, 2)) * kRadToDeg;
    o.rotY = std::atan2(-r(2, 0), sy) * kRadToDeg;
    o.rotZ = std::atan2(r(1, 0), r(0, 0)) * kRadToDeg;
  } else {
    o.rotX = std::atan2(-r(1, 2), r(1, 1)) * kRadToDeg;
    o.rotY = std::atan2(-r(2, 0), sy) * kRadToDeg;
    o.rotZ = 0.f;
  }

  // Twist: project the palm normal onto the plane perpendicular to the finger
  // axis and measure its angle against the camera's x axis.
  Vec3 proj = z - y * dot(z, y);
  proj = normalized(proj);
  const Vec3 ref{1.f, 0.f, 0.f};
  float angle = std::acos(std::clamp(dot(ref, proj), -1.f, 1.f));
  if (dot(cross(ref, proj), y) < 0.f) angle = -angle;
  o.twist = angle * kRadToDeg;
  return o;
}

}  // namespace ht
