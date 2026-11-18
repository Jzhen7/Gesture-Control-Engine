// How is the hand *rotated*, relative to the calibration pose? (Dash 4)
//
// Everything here works in "arm space": the hand's directions written the
// way the robot arm writes its own, as if the user stood behind the arm,
// facing the same way:
//   X  forward  (towards the camera)
//   Y  left     (the user's left)
//   Z  up
// That is a right-handed frame, so rotations measured on the hand can be
// applied to the arm unchanged. It assumes a mirrored camera feed (image
// right = the user's right), like the rest of hand_space.
//
// The hand's own axes (a "hand frame"), built from palm landmarks only, so
// curling the fingers into a claw or a fist doesn't change them:
//   y  fingers  wrist -> middle-finger knuckle
//   z  front    out of the side of the hand that faced the camera at calibration
//   x  across   y cross z (points to the user's left at calibration)
//
// The rotation since calibration is split into three angles about those axes:
//   flex   fingers tip towards the front (+) or the back (-)   (wrist flexion)
//   twist  turn about the finger axis, like a doorknob           (forearm rotation)
//   side   fingers tilt sideways                                 (wrist side bend)
#pragma once

#include "handtracker/core/landmarks.hpp"

namespace ht {

struct HandFrame {
  Vec3 x{0.f, 1.f, 0.f};  ///< Across the palm.
  Vec3 y{0.f, 0.f, 1.f};  ///< Along the fingers (wrist -> middle knuckle).
  Vec3 z{1.f, 0.f, 0.f};  ///< Out of the front of the hand.
};

/// Angles in radians, relative to the calibration frame.
struct HandAngles {
  float flex = 0.f;   ///< + = fingertips towards the front of the hand.
  float twist = 0.f;  ///< + = counter-clockwise looking down the fingers from the tips (right-hand rule).
  float side = 0.f;   ///< + = fingers tilt towards the hand's x axis (the user's left at calibration).
};

/// Landmark vector (x right, y down, z away from camera) -> arm space.
inline Vec3 toArmSpace(const Vec3& v) { return {-v.z, -v.x, -v.y}; }

/// Which way the palm-normal cross product must be flipped so the hand's
/// "front" faces the camera in this pose: +1 or -1. Fixed at calibration, so
/// it works for either hand and for palm- or back-of-hand-to-camera.
float frontSign(const HandLandmarks& hand);

/// The hand's axes in arm space. `sign` comes from frontSign() at calibration.
HandFrame handFrame(const HandLandmarks& hand, float sign);

/// Rotation of `now` relative to `cal`, split into flex, twist and side.
/// Twist is measured about the *current* finger axis (swing-twist split), so
/// it stays meaningful when the hand is also flexed.
HandAngles handAngles(const HandFrame& now, const HandFrame& cal);

/// Fraction of the palm (wrist -> middle knuckle) lying in the image plane,
/// from the 3D points: 1 when the palm faces the camera, smaller when it tips
/// towards or away from it. Clamped to [0.3, 1]. Used to stop wrist flexion
/// from looking like the hand moving away (depth comes from palm size).
float palmInPlaneFraction(const HandLandmarks& hand);

/// Re-orthonormalises a frame from (possibly averaged) y and z directions.
HandFrame orthonormalFrame(const Vec3& fingers, const Vec3& front);

}  // namespace ht
