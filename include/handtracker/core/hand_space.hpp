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
#include "handtracker/core/landmarks.hpp"

namespace ht {

struct HandSpaceCalibration {
  float centerX = 0.5f;   ///< Palm centre, normalised image x, at calibration.
  float centerY = 0.5f;   ///< Palm centre, normalised image y, at calibration.
  float palmSize = 0.1f;  ///< Palm size / image width at calibration (the "middle" depth).
};

struct HandSpaceConfig {
  float rangeX = 0.30f;      ///< Image-x offset (fraction of width) that reaches x = +/-1.
  float rangeY = 0.25f;      ///< Image-y offset (fraction of height) that reaches y = +/-1.
  float depthRange = 0.35f;  ///< |ln(size / calibrated size)| that reaches z = +/-1.
};

/// Palm centre (midpoint of wrist and middle knuckle) in normalised image coordinates.
Vec3 palmCenterNormalized(const HandLandmarks& hand);

/// Palm size as a fraction of image width (scale-free depth cue).
float palmSizeNormalized(const HandLandmarks& hand);

/// Hand position in the calibrated cube. Values may exceed [-1, 1]; callers clamp.
Vec3 handPosition(const HandLandmarks& hand, const HandSpaceCalibration& cal,
                  const HandSpaceConfig& cfg);

/// The MVP gripper rule: closed only for a fist.
inline bool gripClosed(Gesture smoothedGesture) { return smoothedGesture == Gesture::Fist; }

/// Records the calibration pose: an open palm held still for `durationMs`.
class HandCalibrator {
 public:
  explicit HandCalibrator(double durationMs = 2000.0, float maxMotion = 0.05f);

  /// Feed every frame. Returns the calibration once the pose has been held long enough.
  std::optional<HandSpaceCalibration> update(const std::optional<HandLandmarks>& hand,
                                             Gesture smoothedGesture, double tMs);
  float progress(double tMs) const;
  void reset();

 private:
  double durationMs_;
  float maxMotion_;
  bool holding_ = false;
  double startT_ = 0.0;
  float anchorX_ = 0.f;
  float anchorY_ = 0.f;
  double sumX_ = 0.0;
  double sumY_ = 0.0;
  double sumSize_ = 0.0;
  int samples_ = 0;
};

}  // namespace ht
