#include "handtracker/core/hand_pose.hpp"

#include <algorithm>
#include <cmath>

namespace ht {
namespace {

const HandLandmarks::Points& points(const HandLandmarks& hand) {
  return hand.world ? *hand.world : hand.image;
}

Vec3 at(const HandLandmarks& hand, LandmarkId id) {
  return points(hand)[static_cast<std::size_t>(id)];
}

/// Raw palm normal in arm space (direction depends on which hand it is).
Vec3 rawNormal(const HandLandmarks& hand) {
  const Vec3 w = at(hand, LandmarkId::Wrist);
  return normalized(cross(toArmSpace(at(hand, LandmarkId::IndexMcp) - w),
                          toArmSpace(at(hand, LandmarkId::PinkyMcp) - w)));
}

Vec3 fingers(const HandLandmarks& hand) {
  return normalized(toArmSpace(at(hand, LandmarkId::MiddleMcp) - at(hand, LandmarkId::Wrist)));
}

/// Express v in the frame's own axes.
Vec3 inFrame(const HandFrame& f, const Vec3& v) { return {dot(v, f.x), dot(v, f.y), dot(v, f.z)}; }

/// Rotate v about unit axis a by angle (Rodrigues).
Vec3 rotate(const Vec3& v, const Vec3& a, float angle) {
  const float c = std::cos(angle), s = std::sin(angle);
  return v * c + cross(a, v) * s + a * (dot(a, v) * (1.f - c));
}

}  // namespace

HandFrame orthonormalFrame(const Vec3& fingersDir, const Vec3& front) {
  HandFrame f;
  f.y = normalized(fingersDir);
  f.z = normalized(front - f.y * dot(front, f.y));  // make front exactly perpendicular
  f.x = cross(f.y, f.z);
  return f;
}

float frontSign(const HandLandmarks& hand) {
  // Front = the side facing the camera, i.e. pointing along +X (forward).
  return rawNormal(hand).x >= 0.f ? 1.f : -1.f;
}

HandFrame handFrame(const HandLandmarks& hand, float sign) {
  return orthonormalFrame(fingers(hand), rawNormal(hand) * sign);
}

HandAngles handAngles(const HandFrame& now, const HandFrame& cal) {
  // Where do the fingers point now, in the calibration frame's axes?
  const Vec3 f = inFrame(cal, now.y);
  HandAngles a;
  a.flex = std::atan2(f.z, f.y);
  a.side = std::asin(std::clamp(f.x, -1.f, 1.f));

  // Swing-twist split: undo the "swing" (the shortest rotation that carries the
  // calibration finger axis onto the current one), and whatever rotation is
  // left about the finger axis is the twist.
  const Vec3 axis = cross(cal.y, now.y);
  const float sinA = axis.norm();
  const float cosA = std::clamp(dot(cal.y, now.y), -1.f, 1.f);
  Vec3 swungFront = cal.z;
  if (sinA > 1e-6f) swungFront = rotate(cal.z, axis / sinA, std::atan2(sinA, cosA));
  // Signed angle from the swung calibration front to the current front, about the fingers.
  a.twist = std::atan2(dot(cross(swungFront, now.z), now.y), dot(swungFront, now.z));
  return a;
}

float palmInPlaneFraction(const HandLandmarks& hand) {
  const Vec3 p = at(hand, LandmarkId::MiddleMcp) - at(hand, LandmarkId::Wrist);
  const float len = p.norm();
  if (len < 1e-9f) return 1.f;
  return std::clamp(std::hypot(p.x, p.y) / len, 0.3f, 1.f);
}

}  // namespace ht
