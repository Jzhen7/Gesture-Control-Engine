// Where is the hand in 3D, relative to where the user calibrated it?
//
// A single webcam has no depth sensor, so depth is estimated from how big the
// palm looks: a hand closer to the camera looks bigger. The palm is measured
// from the wrist to the middle-finger knuckle, a length that does not change
// when the fingers curl into a fist (otherwise closing the gripper would make
// the arm jump backwards).
//
// The result is a "hand position" in a normalised cube, roughly [-1, 1] per
// axis, centred on the calibration pose:
//   x  +1 = user's right       (camera feed is mirrored, so image right)
//   y  +1 = up
//   z  +1 = towards the camera (pushing the hand forward)
// Front-ends map that cube onto whatever they control (e.g. an arm workspace).
#pragma once

#include <optional>

#include "handtracker/core/gesture.hpp"
#include "handtracker/core/hand_pose.hpp"
#include "handtracker/core/landmarks.hpp"

namespace ht {

struct HandSpaceCalibration {
  float centerX = 0.5f;   ///< Palm centre, normalised image x, at calibration.
  float centerY = 0.5f;   ///< Palm centre, normalised image y, at calibration.
  float palmSize = 0.1f;  ///< Palm size / image width at calibration (the "middle" depth).
  /// fingertipSpread() of the user's open palm at calibration: "100% open" for this hand.
  float openSpread = 0.85f;
  /// Dash 4: the hand's axes at calibration (the "neutral" orientation) and the
  /// front-of-hand sign that makes the frame point at the camera (see hand_pose.hpp).
  HandFrame frame;
  float frontSign = 1.f;
};

struct HandSpaceConfig {
  float rangeX = 0.30f;      ///< Image-x offset (fraction of width) that reaches x = +/-1.
  float rangeY = 0.25f;      ///< Image-y offset (fraction of height) that reaches y = +/-1.
  float depthRange = 0.35f;  ///< |ln(size / calibrated size)| that reaches z = +/-1.
  /// Dash 4: divide the palm size by palmInPlaneFraction(), so tipping the hand
  /// forward/back (to pitch the claw) isn't mistaken for moving away.
  bool tiltCorrection = true;
};

/// Palm centre (midpoint of wrist and middle knuckle) in normalised image coordinates.
Vec3 palmCenterNormalized(const HandLandmarks& hand);

/// Palm size as a fraction of image width (scale-free depth cue). With
/// `tiltCorrected`, divided by palmInPlaneFraction(): the size the palm would
/// have if it faced the camera squarely.
float palmSizeNormalized(const HandLandmarks& hand, bool tiltCorrected = false);

/// Hand position in the calibrated cube. Values may exceed [-1, 1]; callers clamp.
Vec3 handPosition(const HandLandmarks& hand, const HandSpaceCalibration& cal,
                  const HandSpaceConfig& cfg);

/// How open the "claw" is, as a scale-free number (Dash 3, live claw control).
///
/// Closed = all five fingertips touching one another; open = spread wide.
/// Measured as the mean distance between every pair of the five fingertips
/// (10 pairs), divided by the palm length (wrist -> middle knuckle). Small only
/// when *all* tips are bunched together; roughly 0.15-0.25 with the tips
/// touching (fingers have thickness), 0.8-1.0 with an open palm.
/// Uses the metric 3D world landmarks when the model provides them (so a claw
/// pointing at the camera isn't foreshortened), otherwise the image points.
float fingertipSpread(const HandLandmarks& hand);

/// How straight the *least* straight of the four fingers (index..pinky) is.
/// Per finger: the straight-line knuckle -> tip distance divided by the length
/// along its bones (knuckle -> middle joint -> end joint -> tip).
/// About 1.0 for a straight finger, even one bent forward at the knuckle as in
/// the claw; about 0.2-0.5 for a finger curled into a fist. A straight line
/// stays straight in a camera image, so this works on image points too.
float fingerStraightness(const HandLandmarks& hand);

struct GripConfig {
  /// fingertipSpread() that counts as fully closed (fingertips touching).
  float closedSpread = 0.25f;
  /// Snap zone at each end, as a fraction of the 0..1 range: within it the
  /// result is exactly 0 or 1, so a fist really shuts and an open hand really opens.
  float deadband = 0.1f;
  /// fingerStraightness() needed for the hand to count as a claw at all.
  /// Below it (a fist, curled or relaxed fingers) the gripper holds still.
  float minStraightness = 0.8f;
};

/// The claw pose: all four fingers straight (thumb: see docs). Only this pose
/// moves the gripper; closing it is the fingertips meeting, not a fist.
bool isClawPose(const HandLandmarks& hand, const GripConfig& cfg);

/// Hand openness in [0, 1]: 0 = fingertips pinched together, 1 = as open as the
/// calibration palm. Linear in fingertipSpread() between cfg.closedSpread and
/// cal.openSpread.
float handOpenness(const HandLandmarks& hand, const HandSpaceCalibration& cal,
                   const GripConfig& cfg);

/// The MVP gripper rule: closed only for a fist.
inline bool gripClosed(Gesture smoothedGesture) { return smoothedGesture == Gesture::Fist; }

/// Records the calibration pose: an open palm held still for `durationMs`.
class HandCalibrator {
 public:
  explicit HandCalibrator(double durationMs = 2000.0, float maxMotion = 0.05f,
                          bool tiltCorrection = false);

  /// Feed every frame. Returns the calibration once the pose has been held long enough.
  std::optional<HandSpaceCalibration> update(const std::optional<HandLandmarks>& hand,
                                             Gesture smoothedGesture, double tMs);
  float progress(double tMs) const;
  void reset();

 private:
  double durationMs_;
  float maxMotion_;
  bool tiltCorrection_;
  bool holding_ = false;
  double startT_ = 0.0;
  float anchorX_ = 0.f;
  float anchorY_ = 0.f;
  double sumX_ = 0.0;
  double sumY_ = 0.0;
  double sumSize_ = 0.0;
  double sumSpread_ = 0.0;
  Vec3 sumFingers_;  ///< Summed finger directions (arm space).
  Vec3 sumNormal_;   ///< Summed palm normals, each flipped to agree with the first.
  int samples_ = 0;
};

}  // namespace ht
